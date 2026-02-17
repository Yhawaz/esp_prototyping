#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "mpu6050.h"

static const char *TAG = "RYLR998_RX_TEST";

//buncha variavbles for the mpu6050
static mpu6050_handle_t mpu6050_dev = NULL;
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;
//da i2c
i2c_port_t meow2c = I2C_NUM_0;

#define BUF_SIZE (1024)
#define ECHO_TASK_STACK_SIZE   2048

bool lora_working;

static void mpu6050_init()
{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_3,
        .scl_io_num = GPIO_NUM_2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(meow2c, &conf);
    i2c_driver_install(meow2c, conf.mode, 0, 0, 0);

    mpu6050_dev = mpu6050_create(meow2c, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);
}

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
            lora_working=true;
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv from device str: %s", (char *) data);
        }else{
            lora_working=false;
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS); 
    }
}

static void flash_task(void *args){
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_OUTPUT);
    while(1){
        if(lora_working){
            gpio_set_level(GPIO_NUM_6, 1);
        }else{
            for(int i=0;i<6;i++){
                gpio_set_level(GPIO_NUM_6, 1);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 300 millisecond
                gpio_set_level(GPIO_NUM_6, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 300 millisecond
            }
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay for 300 millisecond
    }

}
static void acc_task(void *args){
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    while(1){
        mpu6050_get_gyro(mpu6050_dev, &gyro);
        if(gyro.gyro_y>2 || gyro.gyro_y<-2){
            gpio_set_level(GPIO_NUM_1, 1);
        }else{
            for(int i=0;i<6;i++){
                gpio_set_level(GPIO_NUM_1, 1);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 300 millisecond
                gpio_set_level(GPIO_NUM_1, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 300 millisecond
            }
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay for 300 millisecond
    }
    
}

void app_main(void){
    mpu6050_init();
    xTaskCreate(rx_task, "rx_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(flash_task, "flash_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(acc_task, "acc_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
