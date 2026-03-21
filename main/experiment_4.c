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

static const char *TAG = "exp4";

// --- Wi-Fi credentials: set via menuconfig (Component config > Experiment 4)
#ifndef WIFI_SSID
#define WIFI_SSID CONFIG_WIFI_SSID
#endif
#ifndef WIFI_PASS
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#endif

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;

// --- Servo (LEDC)
#define SERVO_PIN GPIO_NUM_18
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_FREQ_HZ 50
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT

// 50 Hz, 13-bit: period = 20 ms; 1 ms = 409 counts, 2 ms = 819 counts
#define SERVO_DUTY_MIN 409
#define SERVO_DUTY_MAX 819

static void servo_set_angle(int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;
    uint32_t duty = SERVO_DUTY_MIN + (uint32_t)((SERVO_DUTY_MAX - SERVO_DUTY_MIN) * angle / 180);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    ESP_LOGI(TAG, "Servo angle: %d  duty: %lu", angle, (unsigned long)duty);
}

static void servo_init(void)
{
    gpio_reset_pin(SERVO_PIN);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = SERVO_DUTY_MIN,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
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
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Servo Control</title>"
    "<style>"
    "body{font-family:sans-serif;display:flex;flex-direction:column;"
    "align-items:center;justify-content:center;height:100vh;margin:0;background:#1a1a2e;color:#eee;}"
    "h1{margin-bottom:2rem;font-size:1.8rem;}"
    "input[type=range]{width:80vw;max-width:400px;accent-color:#e94560;}"
    "#angle{font-size:3rem;font-weight:bold;margin:1rem 0;color:#e94560;}"
    "</style></head><body>"
    "<h1>Servo Control</h1>"
    "<div id='angle'>90&deg;</div>"
    "<input type='range' min='0' max='180' value='90' id='slider'>"
    "<script>"
    "const slider=document.getElementById('slider');"
    "const label=document.getElementById('angle');"
    "let timer=null;"
    "slider.addEventListener('input',()=>{"
    "  label.textContent=slider.value+'\\u00b0';"
    "  clearTimeout(timer);"
    "  timer=setTimeout(()=>sendAngle(slider.value),80);"
    "});"
    "function sendAngle(v){"
    "  fetch('/servo',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},"
    "  body:'angle='+v}).catch(console.error);"
    "}"
    "</script></body></html>";

static esp_err_t get_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t post_servo(httpd_req_t *req)
{
    char buf[32] = {0};
    int len = req->content_len < (int)sizeof(buf) - 1 ? req->content_len : (int)sizeof(buf) - 1;
    if (httpd_req_recv(req, buf, len) <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Body is "angle=<value>"
    char *val = strstr(buf, "angle=");
    if (val)
    {
        int angle = atoi(val + 6);
        servo_set_angle(angle);
    }

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
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_index,
    };
    httpd_register_uri_handler(server, &uri_get);

    httpd_uri_t uri_post = {
        .uri = "/servo",
        .method = HTTP_POST,
        .handler = post_servo,
    };
    httpd_register_uri_handler(server, &uri_post);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

// --- Entry point
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 experiment 4: Wi-Fi web server + servo slider");

    ESP_ERROR_CHECK(nvs_flash_init());

    servo_init();
    servo_set_angle(90); // start at center

    wifi_init();
    start_webserver();

    // Nothing left to do — HTTP server and Wi-Fi run on their own tasks
}
