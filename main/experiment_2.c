#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "main";

#define LED_PIN GPIO_NUM_5
#define POT_ADC_UNIT ADC_UNIT_1
#define POT_ADC_CHANNEL ADC_CHANNEL_4
#define POT_ADC_ATTEN ADC_ATTEN_DB_12

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_FREQ_HZ 5000
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_DUTY_MAX ((1 << 13) - 1)

#define ADC_RAW_MAX 4095

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 project started");

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = POT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = POT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &channel_config));

    while (1)
    {
        int pot_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &pot_raw));

        if (pot_raw < 0)
        {
            pot_raw = 0;
        }
        else if (pot_raw > ADC_RAW_MAX)
        {
            pot_raw = ADC_RAW_MAX;
        }

        uint32_t duty = (uint32_t)pot_raw * LEDC_DUTY_MAX / ADC_RAW_MAX;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        ESP_LOGI(TAG, "Potentiometer raw: %d, PWM duty: %lu", pot_raw, (unsigned long)duty);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
