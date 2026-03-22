#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_http_server.h"

static const char *TAG = "turret";

// --- Wi-Fi credentials: set via menuconfig (Component config > Experiment 4)
#ifndef WIFI_SSID
#define WIFI_SSID CONFIG_WIFI_SSID
#endif
#ifndef WIFI_PASS
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#endif

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;

// --- Servos (LEDC)
// Pins from turret.yaml: pan=GPIO18, tilt=GPIO19, trigger=GPIO21
#define SERVO_PAN_PIN     GPIO_NUM_18
#define SERVO_TILT_PIN    GPIO_NUM_19
#define SERVO_TRIGGER_PIN GPIO_NUM_21

#define LEDC_MODE     LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ  50
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT

// 50 Hz, 13-bit: period = 20 ms; 1 ms = 409 counts, 2 ms = 819 counts
#define SERVO_DUTY_MIN 409
#define SERVO_DUTY_MAX 819

typedef enum {
    SERVO_PAN     = 0,
    SERVO_TILT    = 1,
    SERVO_TRIGGER = 2,
} servo_id_t;

static const ledc_channel_t servo_channels[] = {
    LEDC_CHANNEL_0,  // pan
    LEDC_CHANNEL_1,  // tilt
    LEDC_CHANNEL_2,  // trigger
};

static const gpio_num_t servo_pins[] = {
    SERVO_PAN_PIN,
    SERVO_TILT_PIN,
    SERVO_TRIGGER_PIN,
};

static void servo_set_angle(servo_id_t id, int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    uint32_t duty = SERVO_DUTY_MIN +
                    (uint32_t)((SERVO_DUTY_MAX - SERVO_DUTY_MIN) * angle / 180);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, servo_channels[id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, servo_channels[id]));
    ESP_LOGI(TAG, "Servo %d angle: %d  duty: %lu",
             (int)id, angle, (unsigned long)duty);
}

static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(servo_pins[i]);
        ledc_channel_config_t channel = {
            .gpio_num   = servo_pins[i],
            .speed_mode = LEDC_MODE,
            .channel    = servo_channels[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = SERVO_DUTY_MIN,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
}

// --- Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

// --- HTTP handlers
static const char *INDEX_HTML =
    "<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Turret Control</title>"
    "<style>"
    "body{font-family:sans-serif;display:flex;flex-direction:column;"
    "align-items:center;justify-content:center;height:100vh;margin:0;"
    "background:#1a1a2e;color:#eee;gap:2rem;}"
    "h1{margin:0;font-size:1.8rem;}"
    ".axis{display:flex;flex-direction:column;align-items:center;gap:.6rem;}"
    "label{font-size:1rem;color:#aaa;}"
    ".controls{display:flex;align-items:center;gap:.6rem;}"
    ".btn{width:3.2rem;height:3.2rem;font-size:1.4rem;background:#e94560;"
    "color:#fff;border:none;border-radius:8px;cursor:pointer;"
    "user-select:none;touch-action:none;}"
    ".btn:active{background:#c73652;}"
    "input[type=number]{width:5rem;text-align:center;font-size:1.5rem;"
    "font-weight:bold;color:#e94560;background:#16213e;"
    "border:2px solid #e94560;border-radius:8px;padding:.3rem;}"
    "input[type=number]::-webkit-inner-spin-button,"
    "input[type=number]::-webkit-outer-spin-button{-webkit-appearance:none;}"
    "input[type=number]{-moz-appearance:textfield;}"
    ".fire{padding:.8rem 3rem;font-size:1.2rem;background:#e94560;"
    "color:#fff;border:none;border-radius:8px;cursor:pointer;}"
    ".fire:active{background:#c73652;}"
    "</style></head><body>"
    "<h1>Turret Control</h1>"
    "<div class='axis'>"
    "  <label>Pan (Horizontal)</label>"
    "  <div class='controls'>"
    "    <button class='btn' id='pan-dec'>&#9668;</button>"
    "    <input type='number' id='pan-val' min='0' max='180' value='90'>"
    "    <button class='btn' id='pan-inc'>&#9658;</button>"
    "  </div>"
    "</div>"
    "<div class='axis'>"
    "  <label>Tilt (Vertical)</label>"
    "  <div class='controls'>"
    "    <button class='btn' id='tilt-inc'>&#9650;</button>"
    "    <input type='number' id='tilt-val' min='0' max='180' value='90'>"
    "    <button class='btn' id='tilt-dec'>&#9660;</button>"
    "  </div>"
    "</div>"
    "<button class='fire' onclick='fire()'>FIRE</button>"
    "<label style='display:flex;align-items:center;gap:.5rem;font-size:1rem;cursor:pointer;'>"
    "  <input type='checkbox' id='pan-sweep' style='width:1.2rem;height:1.2rem;cursor:pointer;'>"
    "  Rotate pan end to end continuously"
    "</label>"
    "<script>"
    "var STEP=1;"
    "var sweepTimer=null,sweepDir=1,sweepAngle=90;"
    "document.getElementById('pan-sweep').addEventListener('change',function(){"
    "  if(this.checked){"
    "    sweepAngle=parseInt(document.getElementById('pan-val').value)||90;"
    "    sweepDir=1;"
    "    sweepTimer=setInterval(function(){"
    "      sweepAngle+=sweepDir*STEP;"
    "      if(sweepAngle>=180){sweepAngle=180;sweepDir=-1;}"
    "      else if(sweepAngle<=0){sweepAngle=0;sweepDir=1;}"
    "      document.getElementById('pan-val').value=sweepAngle;"
    "      send('pan',sweepAngle);"
    "    },30);"
    "  }else{"
    "    clearInterval(sweepTimer);sweepTimer=null;"
    "  }"
    "});"
    "var _inflight={};"
    "var _pending={};"
    "function _flush(servo){"
    "  if(_pending[servo]===undefined)return;"
    "  var v=_pending[servo];"
    "  delete _pending[servo];"
    "  _inflight[servo]=true;"
    "  fetch('/servo',{method:'POST',"
    "    headers:{'Content-Type':'application/x-www-form-urlencoded'},"
    "    body:servo+'='+v})"
    "  .catch(function(){})"
    "  .finally(function(){_inflight[servo]=false;_flush(servo);});"
    "}"
    "function send(servo,val){"
    "  _pending[servo]=val;"
    "  if(!_inflight[servo])_flush(servo);"
    "}"
    "function clamp(v){return Math.max(0,Math.min(180,v));}"
    "function bindAxis(inputId,decId,incId,servo){"
    "  var inp=document.getElementById(inputId);"
    "  function sendVal(){"
    "    var v=clamp(parseInt(inp.value)||0);"
    "    inp.value=v;"
    "    send(servo,v);"
    "  }"
    "  function step(delta){"
    "    inp.value=clamp((parseInt(inp.value)||0)+delta);"
    "    sendVal();"
    "  }"
    "  inp.addEventListener('change',sendVal);"
    "  inp.addEventListener('keydown',function(e){if(e.key==='Enter')sendVal();});"
    "  function bindBtn(btnId,delta){"
    "    var btn=document.getElementById(btnId);"
    "    var timer,interval;"
    "    function start(){"
    "      step(delta);"
    "      timer=setTimeout(function(){"
    "        interval=setInterval(function(){step(delta);},80);"
    "      },350);"
    "    }"
    "    function stop(){clearTimeout(timer);clearInterval(interval);}"
    "    btn.addEventListener('mousedown',start);"
    "    btn.addEventListener('touchstart',function(e){e.preventDefault();start();});"
    "    btn.addEventListener('mouseup',stop);"
    "    btn.addEventListener('mouseleave',stop);"
    "    btn.addEventListener('touchend',stop);"
    "  }"
    "  bindBtn(decId,-STEP);"
    "  bindBtn(incId,STEP);"
    "}"
    "bindAxis('pan-val','pan-dec','pan-inc','pan');"
    "bindAxis('tilt-val','tilt-dec','tilt-inc','tilt');"
    "function fire(){"
    "  fetch('/trigger',{method:'POST'}).catch(console.error);"
    "}"
    "</script></body></html>";

static esp_err_t get_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// POST /servo  body: "pan=<0-180>" or "tilt=<0-180>"
static esp_err_t post_servo(httpd_req_t *req)
{
    char buf[32] = {0};
    int len = req->content_len < (int)sizeof(buf) - 1
                  ? req->content_len
                  : (int)sizeof(buf) - 1;
    if (httpd_req_recv(req, buf, len) <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *val;
    if ((val = strstr(buf, "pan=")) != NULL)
        servo_set_angle(SERVO_PAN, atoi(val + 4));
    if ((val = strstr(buf, "tilt=")) != NULL)
        servo_set_angle(SERVO_TILT, atoi(val + 5));

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /trigger  pulses the trigger servo to 90° then returns to 0°
static esp_err_t post_trigger(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Firing trigger");
    servo_set_angle(SERVO_TRIGGER, 90);
    vTaskDelay(pdMS_TO_TICKS(500));
    servo_set_angle(SERVO_TRIGGER, 10);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t uri_get = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = get_index,
    };
    httpd_register_uri_handler(server, &uri_get);

    httpd_uri_t uri_servo = {
        .uri     = "/servo",
        .method  = HTTP_POST,
        .handler = post_servo,
    };
    httpd_register_uri_handler(server, &uri_servo);

    httpd_uri_t uri_trigger = {
        .uri     = "/trigger",
        .method  = HTTP_POST,
        .handler = post_trigger,
    };
    httpd_register_uri_handler(server, &uri_trigger);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

// --- Entry point
void app_main(void)
{
    ESP_LOGI(TAG, "Turret: Wi-Fi web server + pan/tilt/trigger servos");

    ESP_ERROR_CHECK(nvs_flash_init());

    servo_init();
    servo_set_angle(SERVO_PAN, 90);
    servo_set_angle(SERVO_TILT, 90);
    servo_set_angle(SERVO_TRIGGER, 10);

    wifi_init();
    start_webserver();
}
