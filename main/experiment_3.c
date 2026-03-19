#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "main";

#define SERVO_PIN GPIO_NUM_18
#define POT_ADC_UNIT ADC_UNIT_1
#define POT_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define POT_ADC_ATTEN ADC_ATTEN_DB_12

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_FREQ_HZ 50
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_DUTY_MAX ((1 << 13) - 1)

// Servo pulse width in timer counts at 50 Hz, 13-bit resolution
// Period = 20 ms; 1 ms = 409 counts, 2 ms = 819 counts
#define SERVO_DUTY_MIN 409
#define SERVO_DUTY_MAX 819

#define ADC_RAW_MAX 4095

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 experiment 3: potentiometer → servo pan");

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = SERVO_DUTY_MIN,
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

        uint32_t duty = SERVO_DUTY_MIN +
                        (uint32_t)pot_raw * (SERVO_DUTY_MAX - SERVO_DUTY_MIN) / ADC_RAW_MAX;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        ESP_LOGI(TAG, "Potentiometer raw: %d, servo duty: %lu", pot_raw, (unsigned long)duty);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
