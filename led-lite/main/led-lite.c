#include <stdio.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h" // Optional, for logging

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_OUTPUT);
    while(1){
        gpio_set_level(GPIO_NUM_6, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 300 millisecond
        gpio_set_level(GPIO_NUM_6, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 300 millisecond
        gpio_set_level(GPIO_NUM_6, 1);
    }

}
