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

mpu6500 IMU;         // MPU6500 instance
calData calib = {0}; // Calibration data
AccelData accelData; // Accelerometer data
GyroData gyroData;   // Gyroscope data

static const char *TAG = "MPU-6500";

// Function prototypes
// Function prototypes
esp_err_t mpu6500_setup();
void mpu6500_loop();

void app_main()
{
    // Initialize MPU-6500
    esp_err_t ret_mpu = mpu6500_setup();
    if (ret_mpu != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU-6500: %d", ret_mpu);
        return;
    }
    ESP_LOGI(TAG, "MPU-6500 initialized successfully");

    // Sensor Tasks
    xTaskCreate(mpu6500_loop, "mpu6500_loop", 4096, NULL, 5, NULL);

    // Infinite loop delay
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t mpu6500_setup()
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

    ESP_LOGI(TAG, "Starting MPU-6500 setup");

    int err = mpu6500_init(&IMU, &calib, IMU_ADDRESS);
    if (err != 0)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Keep level.");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    mpu6500_calibrateAccelGyro(&IMU, &calib);
    ESP_LOGI(TAG, "Calibration done!");

    mpu6500_init(&IMU, &calib, IMU_ADDRESS);

    return ESP_OK;
}

void mpu6500_loop(void *pvParameters)
{
    float accumulatedGyroX = 0, accumulatedGyroY = 0, accumulatedGyroZ = 0;
    int sampleCount = 0;

    while (1)
    {
        mpu6500_update(&IMU);
        mpu6500_getGyro(&IMU, &gyroData);

        /*
        mpu6500_getAccel(&IMU, &accelData);
        ESP_LOGI(TAG, "Accel - x:%.0f y:%.0f z:%.0f", accelData.accelX, accelData.accelY, accelData.accelZ - 1);
        */

        // Accumulate gyro data
        accumulatedGyroX += (int) gyroData.gyroX * 0.01; // 0.1 seconds per sample
        accumulatedGyroY += (int) gyroData.gyroY * 0.01;
        accumulatedGyroZ += (int) gyroData.gyroZ * 0.01;

        // Gyro values must be between 0 and 359
        if (accumulatedGyroX >= 360)
            accumulatedGyroX -= 360;
        else if (accumulatedGyroX < 0)
            accumulatedGyroX += 360;

        sampleCount++;

        // Every 100 samples (1 second), log and reset
        if (sampleCount >= 100)
        {
            ESP_LOGI(TAG, "Angular Displacement - x:%.1f y:%.1f z:%.1f", accumulatedGyroX, accumulatedGyroY, accumulatedGyroZ);

            // Reset for next second
            sampleCount = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
