// Copia modificada para Monitoreo en Tiempo Real (ESP32-CAM)
// Basado en Espressif Systems HTTP Camera Server

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "WiFi.h"
#include "camera_index.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// Configuración LED Flash
#define CONFIG_LED_ILLUMINATOR_ENABLED 1
#if CONFIG_LED_ILLUMINATOR_ENABLED
#define LED_LEDC_CHANNEL 2
#define CONFIG_LED_MAX_INTENSITY 255
int led_duty = 0;
bool isStreaming = false;
#endif

typedef struct {
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// --- Variables de Telemetría y Monitoreo en Tiempo Real ---
static float current_fps = 0.0f;
static int8_t motion_detect_enabled = 1;
static int motion_sensitivity = 25;       // Umbral de cambio (5 - 100)
static bool motion_detected_flag = false;
static uint32_t motion_event_count = 0;
static int64_t last_motion_time_us = 0;
static size_t prev_frame_len = 0;
static uint32_t prev_frame_checksum = 0;

typedef struct {
    size_t size;
    size_t index;
    size_t count;
    int sum;
    int *values;
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));
    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values) return NULL;
    memset(filter->values, 0, sample_size * sizeof(int));
    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values) return value;
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index = (filter->index + 1) % filter->size;
    if (filter->count < filter->size) filter->count++;
    return filter->sum / filter->count;
}

// Evaluación rápida de movimiento basada en variación de entropía/tamaño JPEG y muestreo de bytes
static void evaluate_motion(const uint8_t *buf, size_t len)
{
    if (!motion_detect_enabled || !buf || len < 512) {
        motion_detected_flag = false;
        return;
    }

    // Muestreo rápido de 64 puntos en el cuerpo del frame JPEG
    uint32_t checksum = 0;
    size_t start = len / 6;
    size_t end = len - 64;
    size_t step = (end > start) ? ((end - start) / 64) : 1;
    if (step < 1) step = 1;

    for (size_t i = start; i < end; i += step) {
        checksum += buf[i];
    }

    if (prev_frame_len > 0) {
        int len_diff_pct = abs((int)len - (int)prev_frame_len) * 100 / (int)prev_frame_len;
        int chk_diff_pct = (prev_frame_checksum > 0)
            ? (abs((int)checksum - (int)prev_frame_checksum) * 100 / (int)prev_frame_checksum)
            : 0;

        // Umbral inversamente proporcional a la sensibilidad (mayor sensibilidad = menor umbral)
        int threshold = (105 - motion_sensitivity) / 4;
        if (threshold < 2) threshold = 2;

        if (len_diff_pct > threshold || chk_diff_pct > threshold) {
            int64_t now = esp_timer_get_time();
            // Evitar múltiples disparos en menos de 2 segundos
            if (!motion_detected_flag && (now - last_motion_time_us > 2000000)) {
                motion_event_count++;
                log_i("Alerta de movimiento #%u (Delta len: %d%%, chk: %d%%)", motion_event_count, len_diff_pct, chk_diff_pct);
            }
            motion_detected_flag = true;
            last_motion_time_us = now;
        } else if (esp_timer_get_time() - last_motion_time_us > 2500000) {
            motion_detected_flag = false;
        }
    }

    prev_frame_len = len;
    prev_frame_checksum = checksum;
}

#if CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en)
{
    int duty = en ? led_duty : 0;
    if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledcWrite(LED_LEDC_CHANNEL, duty);
}
#endif

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index) j->len = 0;
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) return 0;
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

#if CONFIG_LED_ILLUMINATOR_ENABLED
    bool led_was_off = (!isStreaming && led_duty > 0);
    if (led_was_off) {
        enable_led(true);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
    fb = esp_camera_fb_get();
    if (led_was_off) enable_led(false);
#else
    fb = esp_camera_fb_get();
#endif

    if (!fb) {
        log_e("Captura de cámara fallida");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

    if (fb->format == PIXFORMAT_JPEG) {
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
    }

    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char part_buf[128];

    int64_t last_frame = esp_timer_get_time();

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

#if CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = true;
    enable_led(true);
#endif

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            log_e("Fallo al obtener frame");
            res = ESP_FAIL;
        } else {
            _timestamp.tv_sec = fb->timestamp.tv_sec;
            _timestamp.tv_usec = fb->timestamp.tv_usec;

            if (fb->format != PIXFORMAT_JPEG) {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted) {
                    log_e("Compresión JPEG fallida");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }

        if (res == ESP_OK) {
            // Analizar movimiento en tiempo real antes de enviar
            evaluate_motion(_jpg_buf, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if (fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if (res != ESP_OK) {
            log_i("Cliente desconectado del stream");
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time_ms = (fr_end - last_frame) / 1000;
        last_frame = fr_end;

        if (frame_time_ms > 0) {
            uint32_t avg_ms = ra_filter_run(&ra_filter, (int)frame_time_ms);
            if (avg_ms > 0) {
                current_fps = 1000.0f / (float)avg_ms;
            }
        }
    }

#if CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = false;
    enable_led(false);
#endif
    current_fps = 0.0f;
    return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf)
{
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char *buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            *obuf = buf;
            return ESP_OK;
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char variable[32];
    char value[32];

    if (parse_get(req, &buf) != ESP_OK) return ESP_FAIL;
    if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
        httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int val = atoi(value);
    sensor_t *s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "framesize")) {
        if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
    else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
    else if (!strcmp(variable, "motion_detect")) {
        motion_detect_enabled = (val != 0) ? 1 : 0;
        if (!motion_detect_enabled) motion_detected_flag = false;
    }
    else if (!strcmp(variable, "motion_sens")) {
        motion_sensitivity = val;
    }
#if CONFIG_LED_ILLUMINATOR_ENABLED
    else if (!strcmp(variable, "led_intensity")) {
        led_duty = val;
        if (isStreaming) enable_led(true);
    }
#endif
    else {
        res = -1;
    }

    if (res < 0) return httpd_resp_send_500(req);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

// Nuevo endpoint /telemetry: entrega métricas del sistema y estado de vigilancia en tiempo real
static esp_err_t telemetry_handler(httpd_req_t *req)
{
    char json_buf[320];
    uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    int rssi = WiFi.RSSI();
    uint32_t free_heap = ESP.getFreeHeap();

 snprintf(json_buf, sizeof(json_buf),
             "{\"fps\":%.1f,\"rssi\":%d,\"free_heap\":%u,\"uptime_sec\":%u,"
             "\"motion_enabled\":%d,\"motion_active\":%s,\"motion_events\":%u,\"streaming\":%s}",
             current_fps,
             rssi,
             free_heap,
             uptime_sec,
             motion_detect_enabled,
             motion_detected_flag ? "true" : "false",
             motion_event_count,
             isStreaming ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_buf, strlen(json_buf));
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)monitor_index_html_gz, monitor_index_html_gz_len);
}

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;

    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t cmd_uri = { .uri = "/control", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL };
    httpd_uri_t telemetry_uri = { .uri = "/telemetry", .method = HTTP_GET, .handler = telemetry_handler, .user_ctx = NULL };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

    ra_filter_init(&ra_filter, 20);

    log_i("Iniciando servidor web de monitoreo en puerto: '%d'", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &telemetry_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    log_i("Iniciando servidor de stream MJPEG en puerto: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

void setupLedFlash(int pin)
{
#if CONFIG_LED_ILLUMINATOR_ENABLED
    ledcSetup(LED_LEDC_CHANNEL, 5000, 8);
    ledcAttachPin(pin, LED_LEDC_CHANNEL);
#endif
}
