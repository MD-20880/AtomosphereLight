/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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

#define LED_DUTY_DELTA_RATE 100


typedef struct led_state{

    SemaphoreHandle_t mutex;
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

typedef enum led_evt{
    LED_MSG_BRIGHTNESS,
    LED_MSG_STATE,
} led_evt_t;

typedef struct led_msg{
    led_evt_t evt;
    union{
        int brightness;
        int state;
    };
} led_msg_t;


led_state_t *led_state = NULL;
QueueHandle_t led_task_evt_q = NULL;


static void led_state_init(void){
    led_state = (led_state_t *)malloc(sizeof(led_state_t));
    led_state->mutex = xSemaphoreCreateMutex();
    led_state->brightness = 0;
    led_state->state = 0;
    led_state->color = 0;
}

static void led_state_deinit(void){
    vSemaphoreDelete(led_state->mutex);
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
    led_task_evt_q = xQueueCreate(10, sizeof(struct led_msg));
    if (led_task_evt_q == NULL) {
        printf("[%s]: Failed to create led_task_evt_q\n",__func__);
        return;
    }
    led_state_init();
}

int led_api_set_brightness(int brightness){
    led_msg_t msg;
    if(brightness < LED_BRIGHTNESS_MIN || brightness > LED_BRIGHTNESS_MAX){
        return -1;
    }
    msg.evt = LED_MSG_BRIGHTNESS;
    msg.brightness = brightness;
    xQueueSend(led_task_evt_q, &msg , 0);
    return 0;
}

int led_api_set_state(int state){
    led_msg_t msg;
    if(state < 0 || state > 1){
        return -1;
    }
    msg.evt = LED_MSG_STATE;
    msg.state = state;
    xQueueSend(led_task_evt_q, &msg , 0);
    return 0;
}

void led_pattern_breath(){
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
}

static int brightness_2_duty(int brightness){
    return (brightness * LEDC_DUTY_MAX) / LED_BRIGHTNESS_MAX;
}

static void on_led_brightness_change(int target){
    //change duty smoothly according to the target brightness
    int current_duty = brightness_2_duty(led_state->brightness);
    int target_duty = brightness_2_duty(target);
    led_state->brightness = target;

    int delta = target_duty - current_duty;
    int step = LED_DUTY_DELTA_RATE * (delta > 0 ? 1 : -1);
    while (current_duty != target_duty)
    {   
        if (abs(delta) < abs(step)){
            current_duty = target_duty;
        }else{
            current_duty += step;
            delta -= step;  
        }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, current_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void on_led_sttate_change(int target){
    led_state->state = target;
}

void led_task()
{
    led_msg_t *msg =(led_msg_t*) malloc(sizeof(led_msg_t));
    memset(msg, 0, sizeof(led_msg_t));

    led_init();
    //Switch Case Events
    while (1) {
        if (xQueueReceive(led_task_evt_q, msg, portMAX_DELAY) == pdTRUE) {
            printf("[%s] Received Event\n", __func__);
            xSemaphoreTake(led_state->mutex, portMAX_DELAY);
            switch (msg->evt)
            {
            case LED_MSG_BRIGHTNESS:    
                printf("[%s] LED_MSG_BRIGHTNESS\n", __func__);
                on_led_brightness_change(msg->brightness);
                printf("Setting duty to %d\n", led_state->brightness);
                break;
            case LED_MSG_STATE:
                printf("[%s] LED_MSG_STATE\n", __func__);
                on_led_sttate_change(msg->state);
                printf("Setting state to %d\n", led_state->state);
                break;
            
            default:
                break;
            }
            xSemaphoreGive(led_state->mutex);
        }
    }
    free(msg);
    vTaskDelete(NULL);
}


void app_main(void)
{
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    printf("End of app_main\n");

    //Test LED
        led_api_set_state(0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        led_api_set_state(1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1){
        led_api_set_brightness(100);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        led_api_set_brightness(0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
