#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "RYLR998_RX_TEST";

#define BUF_SIZE (1024)
#define ECHO_TASK_STACK_SIZE   2048

static void rx_task(void *arg){
    //toggle reset 
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    vTaskDelay(300 / portTICK_PERIOD_MS); // Delay for 300 millisecond
    gpio_set_level(GPIO_NUM_7, 0);
    vTaskDelay(300 / portTICK_PERIOD_MS); // Delay for 300 millisecond
    gpio_set_level(GPIO_NUM_7, 1);


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

    //setup config writing c  bullshit
    int len;
    char *data = (char *) malloc(BUF_SIZE);

    //send first msg and let it cachemiss
    sprintf(data, "AT\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 millisecond

    //send a msg and see it com bac
    sprintf(data, "AT\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS); 

    //set the prams or whatever
    sprintf(data, "AT+PARAMETER=5,9,1,4\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS); 

    //set address
    sprintf(data, "AT+ADDRESS=6\r\n");
    len = strlen(data);
    uart_write_bytes(UART_NUM_1, (const char *) data, len);
    vTaskDelay(100 / portTICK_PERIOD_MS); 

    while(1){
        len = uart_read_bytes(UART_NUM_1, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv from device str: %s", (char *) data);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS); 
    }
}

void app_main(void){
    xTaskCreate(rx_task, "rx_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
