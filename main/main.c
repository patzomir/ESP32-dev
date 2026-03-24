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
#include "esp_timer.h"

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
#define SERVO_PAN_PIN GPIO_NUM_18
#define SERVO_TILT_PIN GPIO_NUM_19
#define SERVO_TRIGGER_PIN GPIO_NUM_21

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ 50
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT

// 50 Hz, 13-bit: period = 20 ms; 1 ms = 409 counts, 2 ms = 819 counts
#define SERVO_DUTY_MIN 409
#define SERVO_DUTY_MAX 819

typedef enum
{
    SERVO_PAN = 0,
    SERVO_TILT = 1,
    SERVO_TRIGGER = 2,
} servo_id_t;

static const ledc_channel_t servo_channels[] = {
    LEDC_CHANNEL_0, // pan
    LEDC_CHANNEL_1, // tilt
    LEDC_CHANNEL_2, // trigger
};

static const gpio_num_t servo_pins[] = {
    SERVO_PAN_PIN,
    SERVO_TILT_PIN,
    SERVO_TRIGGER_PIN,
};

static void servo_set_angle(servo_id_t id, int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;
    uint32_t duty = SERVO_DUTY_MIN +
                    (uint32_t)((SERVO_DUTY_MAX - SERVO_DUTY_MIN) * angle / 180);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, servo_channels[id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, servo_channels[id]));
}

// --- Servo motion state (declared here; used by sonar scan task below)
static volatile int pan_target = 90, tilt_target = 90;
static volatile int pan_current = 90, tilt_current = 90;
static volatile bool s_sweep_running = false;

// --- HC-SR04 ultrasonic sensor
// GPIO22 = TRIG (output), GPIO23 = ECHO (input via 1k/2k voltage divider)
#define SONAR_TRIG_PIN  GPIO_NUM_22
#define SONAR_ECHO_PIN  GPIO_NUM_23
#define SONAR_TIMEOUT_US 30000  // ~5 m max; treat as "no target" if exceeded

#define SCAN_STEP_DEG      5    // degrees between sonar pings during scan
#define SCAN_THRESHOLD_CM  50   // stop and lock if distance below this
#define SCAN_MAX_STEPS     ((180 / SCAN_STEP_DEG) + 1)  // 37 positions 0..180

static volatile float  s_scan_distance_cm = -1.0f;
static volatile bool   s_scanning         = false;
static volatile bool   s_target_found     = false;
static TaskHandle_t    s_scan_task_handle = NULL;
static float           s_scan_baseline[SCAN_MAX_STEPS]; // baseline from pass 1
static volatile bool   s_baselining          = false;
static TaskHandle_t    s_baseline_task_handle = NULL;

// --- Lock & follow state
#define FOLLOW_STEP_DEG  3
#define FOLLOW_RANGE_DEG 25
#define FOLLOW_LOST_CM   70   // target considered lost above this distance (cm)

static volatile bool   s_following          = false;
static volatile int    s_follow_angle       = 90;
static TaskHandle_t    s_follow_task_handle = NULL;

static void sonar_init(void)
{
    gpio_reset_pin(SONAR_TRIG_PIN);
    gpio_set_direction(SONAR_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SONAR_TRIG_PIN, 0);

    gpio_reset_pin(SONAR_ECHO_PIN);
    gpio_set_direction(SONAR_ECHO_PIN, GPIO_MODE_INPUT);
}

// Returns distance in cm, or -1.0 on timeout.
static float sonar_ping(void)
{
    // Ensure TRIG is low before pulse
    gpio_set_level(SONAR_TRIG_PIN, 0);
    esp_rom_delay_us(2);

    // 10 µs trigger pulse
    gpio_set_level(SONAR_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(SONAR_TRIG_PIN, 0);

    // Wait for ECHO to go high
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(SONAR_ECHO_PIN) == 0)
    {
        if (esp_timer_get_time() - t0 > SONAR_TIMEOUT_US)
            return -1.0f;
    }

    // Measure ECHO high duration
    int64_t rise = esp_timer_get_time();
    while (gpio_get_level(SONAR_ECHO_PIN) == 1)
    {
        if (esp_timer_get_time() - rise > SONAR_TIMEOUT_US)
            return -1.0f;
    }
    int64_t pulse_us = esp_timer_get_time() - rise;
    return (float)pulse_us / 58.0f;
}

static void scan_task(void *arg)
{
    s_target_found = false;
    for (int i = 0; i < SCAN_MAX_STEPS; i++)
        s_scan_baseline[i] = -1.0f;

    // --- Pass 1: baseline sweep 0 → 180, no detection ---
    for (int angle = 0; angle <= 180 && s_scanning; angle += SCAN_STEP_DEG)
    {
        pan_target = angle;
        while (s_scanning && abs(pan_current - angle) > 1)
            vTaskDelay(pdMS_TO_TICKS(20));
        if (!s_scanning) break;

        float dist = sonar_ping();
        s_scan_distance_cm = dist;
        s_scan_baseline[angle / SCAN_STEP_DEG] = dist;
        ESP_LOGI(TAG, "Baseline pan=%d dist=%.1f cm", angle, dist);
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    // --- Pass 2+: detection sweeps, compare against baseline ---
    int angle = 180;
    int dir   = -1;

    while (s_scanning)
    {
        pan_target = angle;
        while (s_scanning && abs(pan_current - angle) > 1)
            vTaskDelay(pdMS_TO_TICKS(20));
        if (!s_scanning) break;

        float dist = sonar_ping();
        s_scan_distance_cm = dist;
        float base = s_scan_baseline[angle / SCAN_STEP_DEG];
        ESP_LOGI(TAG, "Scan pan=%d dist=%.1f base=%.1f cm", angle, dist, base);

        // Target: something new within threshold that wasn't there in baseline
        if (dist > 0.0f && dist < SCAN_THRESHOLD_CM &&
            (base < 0.0f || base >= SCAN_THRESHOLD_CM))
        {
            s_target_found = true;
            s_follow_angle = angle;
            s_scanning = false;
            ESP_LOGI(TAG, "Target found at pan=%d, %.1f cm", angle, dist);
            break;
        }

        angle += dir * SCAN_STEP_DEG;
        if (angle > 180) { angle = 180; dir = -1; }
        else if (angle < 0) { angle = 0;  dir =  1; }

        vTaskDelay(pdMS_TO_TICKS(60));
    }

    s_scanning = false;
    s_scan_task_handle = NULL;
    vTaskDelete(NULL);
}

static void baseline_task(void *arg)
{
    for (int i = 0; i < SCAN_MAX_STEPS; i++)
        s_scan_baseline[i] = -1.0f;

    for (int angle = 0; angle <= 180 && s_baselining; angle += SCAN_STEP_DEG)
    {
        pan_target = angle;
        while (s_baselining && abs(pan_current - angle) > 1)
            vTaskDelay(pdMS_TO_TICKS(20));
        if (!s_baselining) break;

        float dist = sonar_ping();
        s_scan_distance_cm = dist;
        s_scan_baseline[angle / SCAN_STEP_DEG] = dist;
        ESP_LOGI(TAG, "New baseline pan=%d dist=%.1f cm", angle, dist);
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    pan_target = 90;
    s_baselining = false;
    s_baseline_task_handle = NULL;
    vTaskDelete(NULL);
}

static void follow_task(void *arg)
{
    int base = s_follow_angle;

    while (s_following)
    {
        int lo = base - FOLLOW_RANGE_DEG;
        int hi = base + FOLLOW_RANGE_DEG;
        if (lo < 0)   lo = 0;
        if (hi > 180) hi = 180;

        float best_dist  = 1e9f;
        int   best_angle = base;

        // Narrow sweep: find angle of minimum distance within window
        for (int a = lo; a <= hi && s_following; a += FOLLOW_STEP_DEG)
        {
            pan_target = a;
            while (s_following && abs(pan_current - a) > 1)
                vTaskDelay(pdMS_TO_TICKS(20));
            if (!s_following) break;

            float d = sonar_ping();
            s_scan_distance_cm = d;
            if (d > 0.0f && d < best_dist)
            {
                best_dist  = d;
                best_angle = a;
            }
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        if (!s_following) break;

        if (best_dist > FOLLOW_LOST_CM)
        {
            ESP_LOGI(TAG, "Follow: target lost near pan=%d", base);
            s_following    = false;
            s_target_found = false;
            break;
        }

        // Re-center window on the closest reading
        base           = best_angle;
        s_follow_angle = best_angle;
        ESP_LOGI(TAG, "Follow: tracking pan=%d dist=%.1f cm", best_angle, best_dist);
    }

    s_following          = false;
    s_follow_task_handle = NULL;
    vTaskDelete(NULL);
}

// --- Smooth motion task (pan + tilt interpolation, sweep)
static void servo_smooth_task(void *arg)
{
    int sweep_angle = 90, sweep_dir = 1;
    for (;;)
    {
        if (s_sweep_running)
        {
            sweep_angle += sweep_dir;
            if (sweep_angle >= 180)
            {
                sweep_angle = 180;
                sweep_dir = -1;
            }
            else if (sweep_angle <= 0)
            {
                sweep_angle = 0;
                sweep_dir = 1;
            }
            pan_target = sweep_angle;
        }

        if (pan_current < pan_target)
            pan_current++;
        else if (pan_current > pan_target)
            pan_current--;
        servo_set_angle(SERVO_PAN, pan_current);

        if (tilt_current < tilt_target)
            tilt_current++;
        else if (tilt_current > tilt_target)
            tilt_current--;
        servo_set_angle(SERVO_TILT, tilt_current);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 3; i++)
    {
        gpio_reset_pin(servo_pins[i]);
        ledc_channel_config_t channel = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_MODE,
            .channel = servo_channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = SERVO_DUTY_MIN,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
}

// --- Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
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
            .ssid = WIFI_SSID,
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
    ".scan-box{display:flex;flex-direction:column;align-items:center;gap:.6rem;"
    "border:1px solid #444;border-radius:10px;padding:1rem 2rem;}"
    ".scan-status{font-size:.95rem;color:#aaa;}"
    ".scan-status span{color:#e94560;font-weight:bold;}"
    ".target{color:#4caf50 !important;}"
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
    "<div class='scan-box'>"
    "  <label style='font-size:1rem;'>Sonar Scan</label>"
    "  <div style='display:flex;gap:.6rem;flex-wrap:wrap;justify-content:center;'>"
    "    <button class='fire' id='scan-btn' onclick='scanClick()'>START SCAN</button>"
    "    <button class='fire' id='follow-btn' onclick='followClick()'"
    "      style='display:none;background:#2196f3;'>LOCK &amp; FOLLOW</button>"
    "    <button class='fire' id='baseline-btn' onclick='baselineClick()'"
    "      style='background:#ff9800;'>NEW BASELINE</button>"
    "  </div>"
    "  <div class='scan-status'>Pan: <span id='s-pan'>—</span>&deg;"
    "  &nbsp; Distance: <span id='s-dist'>—</span> cm"
    "  &nbsp; <span id='s-target'></span></div>"
    "</div>"
    "<script>"
    "var STEP=1;"
    "document.getElementById('pan-sweep').addEventListener('change',function(){"
    "  fetch('/sweep',{method:'POST',"
    "    headers:{'Content-Type':'application/x-www-form-urlencoded'},"
    "    body:'running='+(this.checked?'1':'0')}).catch(console.error);"
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
    "var _poll=null,_mode='idle';"
    "function _setMode(m){"
    "  _mode=m;"
    "  var sb=document.getElementById('scan-btn');"
    "  var fb=document.getElementById('follow-btn');"
    "  var bb=document.getElementById('baseline-btn');"
    "  if(m==='idle'||m==='found'){"
    "    sb.textContent='START SCAN';sb.style.display='';"
    "  }else if(m==='scanning'){"
    "    sb.textContent='STOP SCAN';sb.style.display='';"
    "  }else{"
    "    sb.style.display='none';"
    "  }"
    "  if(m==='found'){"
    "    fb.textContent='LOCK \\u0026 FOLLOW';fb.style.background='#2196f3';fb.style.display='';"
    "  }else if(m==='following'){"
    "    fb.textContent='STOP FOLLOW';fb.style.background='#4caf50';fb.style.display='';"
    "  }else{"
    "    fb.style.display='none';"
    "  }"
    "  bb.disabled=(m==='scanning'||m==='following'||m==='baselining');"
    "  bb.textContent=(m==='baselining')?'BASELINING...':'NEW BASELINE';"
    "}"
    "function _setStatus(pan,dist,msg,isGood){"
    "  document.getElementById('s-pan').textContent=pan;"
    "  document.getElementById('s-dist').textContent=dist<0?'\\u2014':dist.toFixed(1);"
    "  var tEl=document.getElementById('s-target');"
    "  tEl.textContent=msg;tEl.className=isGood?'target':'';"
    "}"
    "function scanClick(){"
    "  if(_mode==='scanning'){"
    "    fetch('/scan/stop',{method:'POST'}).catch(console.error);"
    "    if(_poll){clearInterval(_poll);_poll=null;}"
    "    _setStatus('\\u2014',-1,'',false);"
    "    _setMode('idle');"
    "  }else{"
    "    if(_poll){clearInterval(_poll);_poll=null;}"
    "    fetch('/scan/start',{method:'POST'}).catch(console.error);"
    "    _setMode('scanning');"
    "    _poll=setInterval(pollScan,500);"
    "  }"
    "}"
    "function followClick(){"
    "  if(_mode==='following'){"
    "    fetch('/follow/stop',{method:'POST'}).catch(console.error);"
    "    if(_poll){clearInterval(_poll);_poll=null;}"
    "    _setStatus('\\u2014',-1,'',false);"
    "    _setMode('idle');"
    "  }else if(_mode==='found'){"
    "    fetch('/follow/start',{method:'POST'}).catch(console.error);"
    "    _setMode('following');"
    "    _poll=setInterval(pollFollow,500);"
    "  }"
    "}"
    "function pollScan(){"
    "  fetch('/scan/status').then(function(r){return r.json();})"
    "  .then(function(d){"
    "    var msg=d.target_found?'TARGET FOUND':(d.scanning?'SCANNING...':'');"
    "    _setStatus(d.pan,d.distance_cm,msg,d.target_found);"
    "    if(!d.scanning){"
    "      if(_poll){clearInterval(_poll);_poll=null;}"
    "      _setMode(d.target_found?'found':'idle');"
    "      if(!d.target_found){_setStatus('\\u2014',-1,'',false);}"
    "    }"
    "  }).catch(console.error);"
    "}"
    "function pollFollow(){"
    "  fetch('/follow/status').then(function(r){return r.json();})"
    "  .then(function(d){"
    "    _setStatus(d.pan,d.distance_cm,d.following?'FOLLOWING':'TARGET LOST',d.following);"
    "    if(!d.following){"
    "      if(_poll){clearInterval(_poll);_poll=null;}"
    "      _setMode('idle');"
    "    }"
    "  }).catch(console.error);"
    "}"
    "function baselineClick(){"
    "  if(_mode==='baselining')return;"
    "  if(_mode==='scanning'){fetch('/scan/stop',{method:'POST'}).catch(console.error);}"
    "  if(_mode==='following'){fetch('/follow/stop',{method:'POST'}).catch(console.error);}"
    "  if(_poll){clearInterval(_poll);_poll=null;}"
    "  fetch('/scan/baseline',{method:'POST'}).catch(console.error);"
    "  _setMode('baselining');"
    "  _setStatus('\\u2014',-1,'COLLECTING BASELINE...',false);"
    "  _poll=setInterval(pollBaseline,500);"
    "}"
    "function pollBaseline(){"
    "  fetch('/scan/status').then(function(r){return r.json();})"
    "  .then(function(d){"
    "    _setStatus(d.pan,d.distance_cm,'COLLECTING BASELINE...',false);"
    "    if(!d.baselining){"
    "      if(_poll){clearInterval(_poll);_poll=null;}"
    "      _setStatus('\\u2014',-1,'Baseline updated',false);"
    "      _setMode('idle');"
    "    }"
    "  }).catch(console.error);"
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
    if (httpd_req_recv(req, buf, len) <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *val;
    if ((val = strstr(buf, "pan=")) != NULL)
        pan_target = atoi(val + 4);
    if ((val = strstr(buf, "tilt=")) != NULL)
        tilt_target = atoi(val + 5);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /trigger  pulses the trigger servo to 90° then returns to 0°
static esp_err_t post_trigger(httpd_req_t *req)
{
    servo_set_angle(SERVO_TRIGGER, 90);
    vTaskDelay(pdMS_TO_TICKS(500));
    servo_set_angle(SERVO_TRIGGER, 10);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /sweep  body: "running=1" or "running=0"
static esp_err_t post_sweep(httpd_req_t *req)
{
    char buf[32] = {0};
    int len = req->content_len < (int)sizeof(buf) - 1
                  ? req->content_len
                  : (int)sizeof(buf) - 1;
    if (httpd_req_recv(req, buf, len) <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *val = strstr(buf, "running=");
    if (val)
        s_sweep_running = (atoi(val + 8) != 0);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /scan/start
static esp_err_t post_scan_start(httpd_req_t *req)
{
    if (!s_scanning && s_scan_task_handle == NULL)
    {
        s_scanning = true;
        s_target_found = false;
        s_scan_distance_cm = -1.0f;
        xTaskCreatePinnedToCore(scan_task, "scan", 3072, NULL, 2, &s_scan_task_handle, 1);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /scan/stop
static esp_err_t post_scan_stop(httpd_req_t *req)
{
    s_scanning = false;
    pan_target = 90;
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /scan/baseline  — collect a fresh baseline sweep; stops active scan/follow first
static esp_err_t post_scan_baseline(httpd_req_t *req)
{
    s_scanning = false;
    s_following = false;
    if (!s_baselining && s_baseline_task_handle == NULL)
    {
        s_baselining = true;
        s_target_found = false;
        s_scan_distance_cm = -1.0f;
        xTaskCreatePinnedToCore(baseline_task, "baseline", 3072, NULL, 2, &s_baseline_task_handle, 1);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// GET /scan/status  — returns JSON
static esp_err_t get_scan_status(httpd_req_t *req)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"scanning\":%s,\"baselining\":%s,\"pan\":%d,\"distance_cm\":%.1f,\"target_found\":%s}",
             s_scanning ? "true" : "false",
             s_baselining ? "true" : "false",
             pan_current,
             s_scan_distance_cm,
             s_target_found ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// POST /follow/start
static esp_err_t post_follow_start(httpd_req_t *req)
{
    if (!s_following && s_follow_task_handle == NULL && s_target_found)
    {
        s_following = true;
        xTaskCreatePinnedToCore(follow_task, "follow", 3072, NULL, 2, &s_follow_task_handle, 1);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// POST /follow/stop
static esp_err_t post_follow_stop(httpd_req_t *req)
{
    s_following = false;
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// GET /follow/status — returns JSON
static esp_err_t get_follow_status(httpd_req_t *req)
{
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"following\":%s,\"pan\":%d,\"distance_cm\":%.1f}",
             s_following ? "true" : "false",
             pan_current,
             s_scan_distance_cm);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t uri_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_index,
    };
    httpd_register_uri_handler(server, &uri_get);

    httpd_uri_t uri_servo = {
        .uri = "/servo",
        .method = HTTP_POST,
        .handler = post_servo,
    };
    httpd_register_uri_handler(server, &uri_servo);

    httpd_uri_t uri_trigger = {
        .uri = "/trigger",
        .method = HTTP_POST,
        .handler = post_trigger,
    };
    httpd_register_uri_handler(server, &uri_trigger);

    httpd_uri_t uri_sweep = {
        .uri = "/sweep",
        .method = HTTP_POST,
        .handler = post_sweep,
    };
    httpd_register_uri_handler(server, &uri_sweep);

    httpd_uri_t uri_scan_start = {
        .uri = "/scan/start",
        .method = HTTP_POST,
        .handler = post_scan_start,
    };
    httpd_register_uri_handler(server, &uri_scan_start);

    httpd_uri_t uri_scan_stop = {
        .uri = "/scan/stop",
        .method = HTTP_POST,
        .handler = post_scan_stop,
    };
    httpd_register_uri_handler(server, &uri_scan_stop);

    httpd_uri_t uri_scan_baseline = {
        .uri = "/scan/baseline",
        .method = HTTP_POST,
        .handler = post_scan_baseline,
    };
    httpd_register_uri_handler(server, &uri_scan_baseline);

    httpd_uri_t uri_scan_status = {
        .uri = "/scan/status",
        .method = HTTP_GET,
        .handler = get_scan_status,
    };
    httpd_register_uri_handler(server, &uri_scan_status);

    httpd_uri_t uri_follow_start = {
        .uri = "/follow/start",
        .method = HTTP_POST,
        .handler = post_follow_start,
    };
    httpd_register_uri_handler(server, &uri_follow_start);

    httpd_uri_t uri_follow_stop = {
        .uri = "/follow/stop",
        .method = HTTP_POST,
        .handler = post_follow_stop,
    };
    httpd_register_uri_handler(server, &uri_follow_stop);

    httpd_uri_t uri_follow_status = {
        .uri = "/follow/status",
        .method = HTTP_GET,
        .handler = get_follow_status,
    };
    httpd_register_uri_handler(server, &uri_follow_status);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

// --- Entry point
void app_main(void)
{
    ESP_LOGI(TAG, "Turret: Wi-Fi web server + pan/tilt/trigger servos + sonar scan");

    ESP_ERROR_CHECK(nvs_flash_init());

    servo_init();
    sonar_init();
    servo_set_angle(SERVO_PAN, 90);
    servo_set_angle(SERVO_TILT, 90);
    servo_set_angle(SERVO_TRIGGER, 10);
    xTaskCreatePinnedToCore(servo_smooth_task, "servo_smooth", 2048, NULL, 5, NULL, 1);

    wifi_init();
    start_webserver();
}
