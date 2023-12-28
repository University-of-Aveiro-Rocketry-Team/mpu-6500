#include "mpu-6500.h" // Assuming MPU6500.h is adapted for ESP-IDF as previously discussed
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define IMU_ADDRESS 0x68 // IMU Address
#define I2C_SDA 21       // I2C Data pin
#define I2C_SCL 22       // I2C Clock pin
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ_HZ 400000 // 400kHz I2C frequency

MPU6500 IMU;         // MPU6500 instance
calData calib = {0}; // Calibration data
AccelData accelData; // Accelerometer data
GyroData gyroData;   // Gyroscope data

// Function prototypes
void imu_setup();
void imu_loop();

void app_main()
{
    imu_setup();
    while (1)
    {
        imu_loop();
        vTaskDelay(pdMS_TO_TICKS(300)); // Delay for 150ms
    }
}

void imu_setup()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI("IMU", "Starting IMU setup");

    int err = MPU6500_init(&IMU, &calib, IMU_ADDRESS);
    if (err != 0)
    {
        ESP_LOGE("IMU", "Error initializing IMU: %d", err);
        while (true)
        {
            vTaskDelay(portMAX_DELAY); // Infinite loop
        }
    }


    ESP_LOGI("IMU", "Keep IMU level.");
    vTaskDelay(pdMS_TO_TICKS(1000));

    MPU6500_calibrateAccelGyro(&IMU, &calib);
    ESP_LOGI("IMU", "Calibration done!");

    ESP_LOGI("IMU", "Accel biases X/Y/Z: ");
    ESP_LOGI("IMU", "%f, %f, %f", calib.accelBias[0], calib.accelBias[1], calib.accelBias[2]);

    ESP_LOGI("IMU", "Gyro biases X/Y/Z: ");
    ESP_LOGI("IMU", "%f, %f, %f", calib.gyroBias[0], calib.gyroBias[1], calib.gyroBias[2]);

    ESP_LOGI("IMU", "Calib valid: %d", calib.valid);

    MPU6500_init(&IMU, &calib, IMU_ADDRESS);
}

void imu_loop()
{
    MPU6500_update(&IMU);
    
    /*
    MPU6500_getAccel(&IMU, &accelData);
    ESP_LOGI("IMU", "Accel - x:%.0f y:%.0f z:%.0f", accelData.accelX, accelData.accelY, accelData.accelZ - 1);
    */

    MPU6500_getGyro(&IMU, &gyroData);
    ESP_LOGI("IMU", "Gyro - x:%.1f y:%.1f z:%.1f", gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ);
}
