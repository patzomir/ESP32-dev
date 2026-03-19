#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "main";

#define BUTTON_PIN  GPIO_NUM_4
#define LED_PIN     GPIO_NUM_5

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 project started");

    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        int button_state = gpio_get_level(BUTTON_PIN);
        ESP_LOGI(TAG, "Button state: %d", button_state);

        gpio_set_level(LED_PIN, button_state);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
