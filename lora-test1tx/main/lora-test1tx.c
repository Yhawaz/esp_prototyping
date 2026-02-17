#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"


static const char *TAG = "RYLR998_TX_TEST";

#define BUF_SIZE (1024)
#define ECHO_TASK_STACK_SIZE   2048
//write configs to radio

int len;

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
    while (1) {
        sprintf(data, "AT+SEND=6,8,DEADBEEF\r\n");
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
        vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay for 500 millisecond
    }
}

void app_main(void){
    tx_init(); 
    xTaskCreate(tx_task, "tx_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
