#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <sht4x.h>


static const char *TAG = "RYLR998_TX_TEST";

#define BUF_SIZE (1024)
#define ECHO_TASK_STACK_SIZE   2048

int len;
float temperature;
float humidity;
static sht4x_t dev;

void tx_init(){
    //toggle reset pin
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_0, 1);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_0, 0);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_0, 1);
    
    uart_config_t rylr998_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &rylr998_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 9, 8, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));

    //send nothing for it to err on
    char *data = (char *) malloc(BUF_SIZE);
    sprintf(data, "AT\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //send reset
    sprintf(data, "AT+RESET\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //send parameter vals
    sprintf(data, "AT+PARAMETER=5,9,1,4\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void tx_task(void *arg){


    char *data = (char *) malloc(BUF_SIZE);
    char *payload = (char *) malloc(BUF_SIZE);
    int payload_len;
    while (1) {
        sprintf(payload,"TEMP=%.2f,HUM=%.2f",temperature, humidity);
        payload_len=strlen(payload);
        sprintf(data, "AT+SEND=6,%d,%s\r\n",payload_len,payload);
        len=strlen(data);

        uart_write_bytes(UART_NUM_1, (const char *) data, len); 

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 500 millisecond

        len = uart_read_bytes(UART_NUM_1, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        //uart_write_bytes(UART_NUM_1, (const char *) data, len);
        if (len) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv from device str: %s", (char *) data);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
    }
}

static void temp_task(void *arg)
{

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
        //mu hahhahahah
        vTaskDelay(2000 / portTICK_PERIOD_MS); 

    }
}

void app_main(void){
    //sht40 init
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&dev, 0, sizeof(sht4x_t));
    ESP_ERROR_CHECK(sht4x_init_desc(&dev, 0, GPIO_NUM_2, GPIO_NUM_3));
    ESP_ERROR_CHECK(sht4x_init(&dev));

    //tx init(chad function)
    tx_init(); 
    xTaskCreate(tx_task, "tx_task", 8096, NULL, 10, NULL);
    xTaskCreate(temp_task, "temp_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
