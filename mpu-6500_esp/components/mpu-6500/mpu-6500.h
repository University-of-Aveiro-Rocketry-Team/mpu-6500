#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "esp_log.h"

    #define MPU6500_SELF_TEST_X_GYRO 0x00
    #define MPU6500_SELF_TEST_Y_GYRO 0x01
    #define MPU6500_SELF_TEST_Z_GYRO 0x02

    #define MPU6500_SELF_TEST_X_ACCEL 0x0D
    #define MPU6500_SELF_TEST_Y_ACCEL 0x0E
    #define MPU6500_SELF_TEST_Z_ACCEL 0x0F

    #define MPU6500_SELF_TEST_A 0x10

    #define MPU6500_XG_OFFSET_H 0x13 // User-defined trim values for gyroscope
    #define MPU6500_XG_OFFSET_L 0x14
    #define MPU6500_YG_OFFSET_H 0x15
    #define MPU6500_YG_OFFSET_L 0x16
    #define MPU6500_ZG_OFFSET_H 0x17
    #define MPU6500_ZG_OFFSET_L 0x18
    #define MPU6500_SMPLRT_DIV 0x19
    #define MPU6500_MPU_CONFIG 0x1A
    #define MPU6500_GYRO_CONFIG 0x1B
    #define MPU6500_ACCEL_CONFIG 0x1C
    #define MPU6500_ACCEL_CONFIG2 0x1D
    #define MPU6500_LP_ACCEL_ODR 0x1E
    #define MPU6500_WOM_THR 0x1F

    #define MPU6500_MOT_DUR 0x20   // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
    #define MPU6500_ZMOT_THR 0x21  // Zero-motion detection threshold bits [7:0]
    #define MPU6500_ZRMOT_DUR 0x22 // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

    #define MPU6500_FIFO_EN 0x23
    #define MPU6500_I2C_MST_CTRL 0x24
    #define MPU6500_I2C_SLV0_ADDR 0x25
    #define MPU6500_I2C_SLV0_REG 0x26
    #define MPU6500_I2C_SLV0_CTRL 0x27
    #define MPU6500_I2C_SLV1_ADDR 0x28
    #define MPU6500_I2C_SLV1_REG 0x29
    #define MPU6500_I2C_SLV1_CTRL 0x2A
    #define MPU6500_I2C_SLV2_ADDR 0x2B
    #define MPU6500_I2C_SLV2_REG 0x2C
    #define MPU6500_I2C_SLV2_CTRL 0x2D
    #define MPU6500_I2C_SLV3_ADDR 0x2E
    #define MPU6500_I2C_SLV3_REG 0x2F
    #define MPU6500_I2C_SLV3_CTRL 0x30
    #define MPU6500_I2C_SLV4_ADDR 0x31
    #define MPU6500_I2C_SLV4_REG 0x32
    #define MPU6500_I2C_SLV4_DO 0x33
    #define MPU6500_I2C_SLV4_CTRL 0x34
    #define MPU6500_I2C_SLV4_DI 0x35
    #define MPU6500_I2C_MST_STATUS 0x36
    #define MPU6500_INT_PIN_CFG 0x37
    #define MPU6500_INT_ENABLE 0x38
    #define MPU6500_DMP_INT_STATUS 0x39 // Check DMP interrupt
    #define MPU6500_INT_STATUS 0x3A
    #define MPU6500_ACCEL_XOUT_H 0x3B
    #define MPU6500_ACCEL_XOUT_L 0x3C
    #define MPU6500_ACCEL_YOUT_H 0x3D
    #define MPU6500_ACCEL_YOUT_L 0x3E
    #define MPU6500_ACCEL_ZOUT_H 0x3F
    #define MPU6500_ACCEL_ZOUT_L 0x40
    #define MPU6500_TEMP_OUT_H 0x41
    #define MPU6500_TEMP_OUT_L 0x42
    #define MPU6500_GYRO_XOUT_H 0x43
    #define MPU6500_GYRO_XOUT_L 0x44
    #define MPU6500_GYRO_YOUT_H 0x45
    #define MPU6500_GYRO_YOUT_L 0x46
    #define MPU6500_GYRO_ZOUT_H 0x47
    #define MPU6500_GYRO_ZOUT_L 0x48
    #define MPU6500_EXT_SENS_DATA_00 0x49
    #define MPU6500_EXT_SENS_DATA_01 0x4A
    #define MPU6500_EXT_SENS_DATA_02 0x4B
    #define MPU6500_EXT_SENS_DATA_03 0x4C
    #define MPU6500_EXT_SENS_DATA_04 0x4D
    #define MPU6500_EXT_SENS_DATA_05 0x4E
    #define MPU6500_EXT_SENS_DATA_06 0x4F
    #define MPU6500_EXT_SENS_DATA_07 0x50
    #define MPU6500_EXT_SENS_DATA_08 0x51
    #define MPU6500_EXT_SENS_DATA_09 0x52
    #define MPU6500_EXT_SENS_DATA_10 0x53
    #define MPU6500_EXT_SENS_DATA_11 0x54
    #define MPU6500_EXT_SENS_DATA_12 0x55
    #define MPU6500_EXT_SENS_DATA_13 0x56
    #define MPU6500_EXT_SENS_DATA_14 0x57
    #define MPU6500_EXT_SENS_DATA_15 0x58
    #define MPU6500_EXT_SENS_DATA_16 0x59
    #define MPU6500_EXT_SENS_DATA_17 0x5A
    #define MPU6500_EXT_SENS_DATA_18 0x5B
    #define MPU6500_EXT_SENS_DATA_19 0x5C
    #define MPU6500_EXT_SENS_DATA_20 0x5D
    #define MPU6500_EXT_SENS_DATA_21 0x5E
    #define MPU6500_EXT_SENS_DATA_22 0x5F
    #define MPU6500_EXT_SENS_DATA_23 0x60
    #define MPU6500_MOT_DETECT_STATUS 0x61
    #define MPU6500_I2C_SLV0_DO 0x63
    #define MPU6500_I2C_SLV1_DO 0x64
    #define MPU6500_I2C_SLV2_DO 0x65
    #define MPU6500_I2C_SLV3_DO 0x66
    #define MPU6500_I2C_MST_DELAY_CTRL 0x67
    #define MPU6500_SIGNAL_PATH_RESET 0x68
    #define MPU6500_MOT_DETECT_CTRL 0x69
    #define MPU6500_USER_CTRL 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
    #define MPU6500_PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
    #define MPU6500_PWR_MGMT_2 0x6C
    #define MPU6500_DMP_BANK 0x6D   // Activates a specific bank in the DMP
    #define MPU6500_DMP_RW_PNT 0x6E // Set read/write pointer to a specific start address in specified DMP bank
    #define MPU6500_DMP_REG 0x6F    // Register in DMP from which to read or to which to write
    #define MPU6500_DMP_REG_1 0x70
    #define MPU6500_DMP_REG_2 0x71
    #define MPU6500_FIFO_COUNTH 0x72
    #define MPU6500_FIFO_COUNTL 0x73
    #define MPU6500_FIFO_R_W 0x74
    #define MPU6500_WHO_AM_I_MPU6500 0x75 // Should return 0x70
    #define MPU6500_WHOAMI_DEFAULT_VALUE 0x70
    #define MPU6500_XA_OFFSET_H 0x77
    #define MPU6500_XA_OFFSET_L 0x78
    #define MPU6500_YA_OFFSET_H 0x7A
    #define MPU6500_YA_OFFSET_L 0x7B
    #define MPU6500_ZA_OFFSET_H 0x7D
    #define MPU6500_ZA_OFFSET_L 0x7E

    typedef struct
    {
        float accelX;
        float accelY;
        float accelZ;
    } AccelData;

    typedef struct
    {
        float gyroX;
        float gyroY;
        float gyroZ;
    } GyroData;

    typedef struct
    {
        bool valid;
        float accelBias[3];
        float gyroBias[3];
        float magBias[3];
        float magScale[3];
    } calData;

    // MPU6500 class structure in C style
    typedef struct
    {
        float aRes;
        float gRes;
        int geometryIndex;
        float temperature;
        AccelData accel;
        GyroData gyro;
        calData calibration;
        uint8_t IMUAddress;
    } mpu6500;

    // Function prototypes
    int mpu6500_init(mpu6500 *device, calData *cal, uint8_t address);
    void mpu6500_update(mpu6500 *device);
    void mpu6500_getAccel(mpu6500 *device, AccelData *out);
    void mpu6500_getGyro(mpu6500 *device, GyroData *out);
    int mpu6500_setGyroRange(mpu6500 *device, int range);
    int mpu6500_setAccelRange(mpu6500 *device, int range);
    void mpu6500_calibrateAccelGyro(mpu6500 *device, calData *cal);

    // I2C communication functions adapted for ESP-IDF
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);

    bool dataAvailable(mpu6500 *device);

#ifdef __cplusplus
}
#endif

#endif // MPU6500_H
