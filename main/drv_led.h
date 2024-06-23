#include "freertos/FreeRTOS.h"


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


int led_api_set_state(int state);
int led_api_set_brightness(int brightness);