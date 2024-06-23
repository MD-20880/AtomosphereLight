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

#include "drv_led.h"



extern void led_task(void *pvParameter);
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
