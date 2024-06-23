/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

//LEDC Configuration Params
#define LED_PIN         2
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO  LED_PIN
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT
#define LEDC_DUTY       (1023)
#define LEDC_FREQUENCY      5000

#define LEDC_DUTY_MAX   4095
#define LEDC_DUTY_MIN   0

#define LED_BRIGHTNESS_MAX 100
#define LED_BRIGHTNESS_MIN 0

typedef struct led_state{
    /**
     * @brief Brightness of the LED
     * Brightness should be value between 0 and 4095
     */
    int brightness;
    
    /**
     * @brief State of the LED
     * State should be 0 or 1
     */
    int state;

    /**
     * @brief Color of the LED
     * Color should be value between 0 and 255
     */
    unsigned color;
}led_state_t;



led_state_t *led_state = NULL;

static void led_state_init(void){
    led_state = (led_state_t *)malloc(sizeof(led_state_t));
    led_state->brightness = 0;
    led_state->state = 0;
    led_state->color = 0;
}

static void led_state_deinit(void){
    free(led_state);
}

static void ledc_init(void){
      // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void led_init(void)
{
    ledc_init();
    led_state_init();
}

int led_api_set_brightness(int brightness){
    if(brightness < LED_BRIGHTNESS_MIN || brightness > LED_BRIGHTNESS_MAX){
        return -1;
    }
    led_state->brightness = brightness;
    return 0;
}

int led_api_set_state(int state){
    if(state < 0 || state > 1){
        return -1;
    }
    led_state->state = state;
    return 0;
}

void led_task()
{
    //Initialize TWDT
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    printf("[%s] subscribed TWDT", __func__);

    //Initialize Led
    led_init();

    //Switch Case Events
    while (1) {
        for (int i = LEDC_DUTY_MIN; i < LEDC_DUTY_MAX; i+=4) {
            printf("Setting duty to %d\n", i);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        for (int i = LEDC_DUTY_MAX; i >= LEDC_DUTY_MIN; i-=4) {
            printf("Setting duty to %d\n", i);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        esp_task_wdt_reset();
    }

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));

    vTaskDelete(NULL);
}


void app_main(void)
{
    led_task_data = (led_task_data_t *)malloc(sizeof(led_task_data_t));
    memset(led_task_data, 0, sizeof(led_task_data_t));
    xTaskCreate(led_task, "led_task", 2048, 5, NULL);
    printf("End of app_main\n");
}
