#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_err.h>
#include <string.h>
#include "esp_log.h"
#include "mpu6050.h"

static mpu6050_handle_t mpu6050_dev = NULL;
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;

static const char *TAG = "ACCEL_TEST";


i2c_port_t meow2c = I2C_NUM_0;

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

static void task(void *arg)
{
    while (1)
    {
        mpu6050_get_acce(mpu6050_dev, &acce);
        mpu6050_get_gyro(mpu6050_dev, &gyro);
        mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);

        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        ESP_LOGI(TAG, "roll:%.2f, pitch:%.2f", complimentary_angle.roll, complimentary_angle.pitch);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    mpu6050_init();
    xTaskCreate(task, "task",2048 , NULL, 10, NULL);

}
