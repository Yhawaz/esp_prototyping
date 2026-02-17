#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <sht4x.h>
#include <string.h>
#include <esp_err.h>

//me when yabi writes something 85x better,lightweight, and informative than the damn example code btw
//actually nvm a 3rd grade in a foriegn country probably did this and posted it on youtube at 180p quality already
//actually nvm nvm they prob used the arduino ide

//this feels weird its not ai 
//its the source codes example so im rolling with it
//lowkey makes sense why its static nvm, im culling all that weird parameter bullshit
//thats fake i hat proaganda

static sht4x_t dev;

#define ECHO_TASK_STACK_SIZE   2048

static void task(void *arg)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    // get the measurement duration for high repeatability;
    uint8_t duration = sht4x_get_measurement_duration(&dev);

    while (1)
    {
        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht4x_start_measurement(&dev));

        // Wait until measurement is ready (duration returned from *sht4x_get_measurement_duration*).
        //vTaskDelay(duration);
        vTaskDelay(30/portTICK_PERIOD_MS);

        //30 ms update rate, discoevered imperically via SCIENCE 

        // retrieve the values and do something with them
        ESP_ERROR_CHECK(sht4x_get_results(&dev, &temperature, &humidity));
        printf("sht4x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);

        //mu hahhahahah
        vTaskDelay(200 / portTICK_PERIOD_MS); 

    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&dev, 0, sizeof(sht4x_t));

    ESP_ERROR_CHECK(sht4x_init_desc(&dev, 0, GPIO_NUM_2, GPIO_NUM_3));
    ESP_ERROR_CHECK(sht4x_init(&dev));
    xTaskCreate(task, "rx_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

}

