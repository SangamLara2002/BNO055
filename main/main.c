/*
By : Sangam Prajapati
Date : 2024-10-22
Description : Main file for the BNO055 sensor
*/

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C configuration
#define I2C_MASTER_SCL_IO 16        // GPIO for SCL
#define I2C_MASTER_SDA_IO 17        // GPIO for SDA
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number
#define I2C_MASTER_FREQ_HZ 100000   // Frequency of I2C
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 3000

// BNO055 I2C address
#define BNO055_ADDRESS 0x28          // Default BNO055 I2C address
#define BNO055_OPR_MODE_REG 0x3D     // BNO055 operation mode register
#define BNO055_CONFIG_MODE 0x00      // BNO055 config mode
#define BNO055_NDOF_MODE 0x0C        // BNO055 sensor fusion mode (NDOF)
#define BNO055_EULER_H_LSB 0x1A      // Euler angles base register
#define BNO055_ACCEL_DATA_X_LSB 0x08 // Accelerometer data X-axis LSB register
#define BNO055_MAG_DATA_X_LSB 0x0E   // Magnetometer data X-axis LSB register
#define BNO055_GYRO_DATA_X_LSB 0x14  // Gyroscope data X-axis LSB register

/*
    typedef enum
    {
        BNO055_VECTOR_ACCELEROMETER = 0x08, // Default: m/s²
        BNO055_VECTOR_MAGNETOMETER = 0x0E,  // Default: uT
        BNO055_VECTOR_GYROSCOPE = 0x14,     // Default: rad/s
        BNO055_VECTOR_EULER = 0x1A,         // Default: degrees
        BNO055_VECTOR_LINEARACCEL = 0x28,   // Default: m/s²
        BNO055_VECTOR_GRAVITY = 0x2E        // Default: m/s²
    } bno055_vector_type_t;

    typedef enum
    {
        BNO055_OPERATION_MODE_CONFIG = 0x00,
        BNO055_OPERATION_MODE_ACCONLY = 0x01,
        BNO055_OPERATION_MODE_MAGONLY = 0x02,
        BNO055_OPERATION_MODE_GYRONLY = 0x03,
        BNO055_OPERATION_MODE_ACCMAG = 0x04,
        BNO055_OPERATION_MODE_ACCGYRO = 0x05,
        BNO055_OPERATION_MODE_MAGGYRO = 0x06,
        BNO055_OPERATION_MODE_AMG = 0x07,
        BNO055_OPERATION_MODE_IMU = 0x08,
        BNO055_OPERATION_MODE_COMPASS = 0x09,
        BNO055_OPERATION_MODE_M4G = 0x0A,
        BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
        BNO055_OPERATION_MODE_NDOF = 0x0C
    } bno055_opmode_t;
*/

// Function prototypes
esp_err_t i2c_master_init(void);
esp_err_t bno055_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t bno055_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length);
esp_err_t bno055_init(void);
void read_bno055_euler_angles(void);

// Initialize I2C
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Write byte to BNO055
esp_err_t bno055_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read bytes from BNO055
esp_err_t bno055_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initialize BNO055
esp_err_t bno055_init(void)
{
    // Set BNO055 to config mode
    esp_err_t err = bno055_write_byte(BNO055_OPR_MODE_REG, BNO055_CONFIG_MODE);
    vTaskDelay(pdMS_TO_TICKS(20));
    if (err != ESP_OK)
    {
        ESP_LOGE("BNO055", "Failed to initialize 1 BNO055 %d", err);
        return err;
    }

    // Set BNO055 to sensor fusion mode (NDOF)
    bno055_write_byte(BNO055_OPR_MODE_REG, BNO055_NDOF_MODE);
    vTaskDelay(pdMS_TO_TICKS(20));
    if (err != ESP_OK)
    {
        ESP_LOGE("BNO055", "Failed to initialize 2 BNO055 %d", err);
        return err;
    }
    return ESP_OK;
}

// Read Euler angles from BNO055
void read_bno055_euler_angles(void)
{
    uint8_t euler_data[6];
    int16_t heading, roll, pitch;

    while (1)
    {
        // Read 6 bytes from Euler angles register (BNO055_EULER_H_LSB)
        bno055_read_bytes(BNO055_EULER_H_LSB, euler_data, 6);

        // Convert the data (Little Endian)
        heading = ((int16_t)euler_data[1] << 8) | euler_data[0];
        roll = ((int16_t)euler_data[3] << 8) | euler_data[2];
        pitch = ((int16_t)euler_data[5] << 8) | euler_data[4];

        // Convert to degrees (1 unit = 1/16 degree)
        printf("Heading: %.2f, Roll: %.2f, Pitch: %.2f\n",
               heading / 16.0, roll / 16.0, pitch / 16.0);

        // Delay 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Read accelerometer data from BNO055
void read_bno055_accelerometer(void)
{
    uint8_t accel_data[6];
    int16_t accel_x, accel_y, accel_z;

    while (1)
    {
        // Read 6 bytes from Accelerometer data register (BNO055_ACCEL_DATA_X_LSB)
        bno055_read_bytes(BNO055_ACCEL_DATA_X_LSB, accel_data, 6);

        // Convert the data (Little Endian)
        accel_x = ((int16_t)accel_data[1] << 8) | accel_data[0];
        accel_y = ((int16_t)accel_data[3] << 8) | accel_data[2];
        accel_z = ((int16_t)accel_data[5] << 8) | accel_data[4];

        // Convert to m/s^2 (1 unit = 1/100 m/s^2)
        printf("Accel X: %.2f, Y: %.2f, Z: %.2f\n",
               accel_x / 100.0, accel_y / 100.0, accel_z / 100.0);

        // Delay 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Read magnetometer data from BNO055
void read_bno055_magnetometer(void)
{
    uint8_t mag_data[6];
    int16_t mag_x, mag_y, mag_z;

    while (1)
    {
        // Read 6 bytes from Magnetometer data register (BNO055_MAG_DATA_X_LSB)
        bno055_read_bytes(BNO055_MAG_DATA_X_LSB, mag_data, 6);

        // Convert the data (Little Endian)
        mag_x = ((int16_t)mag_data[1] << 8) | mag_data[0];
        mag_y = ((int16_t)mag_data[3] << 8) | mag_data[2];
        mag_z = ((int16_t)mag_data[5] << 8) | mag_data[4];

        // Convert to microTesla (1 unit = 1/16 µT)
        printf("Mag X: %.2f, Y: %.2f, Z: %.2f\n",
               mag_x / 16.0, mag_y / 16.0, mag_z / 16.0);

        // Delay 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Read gyroscope data from BNO055
void read_bno055_gyroscope(void)
{
    uint8_t gyro_data[6];
    int16_t gyro_x, gyro_y, gyro_z;

    while (1)
    {
        // Read 6 bytes from Gyroscope data register (BNO055_GYRO_DATA_X_LSB)
        bno055_read_bytes(BNO055_GYRO_DATA_X_LSB, gyro_data, 6);

        // Convert the data (Little Endian)
        gyro_x = ((int16_t)gyro_data[1] << 8) | gyro_data[0];
        gyro_y = ((int16_t)gyro_data[3] << 8) | gyro_data[2];
        gyro_z = ((int16_t)gyro_data[5] << 8) | gyro_data[4];

        // Convert to degrees per second (1 unit = 1/16 dps)
        printf("Gyro X: %.2f, Y: %.2f, Z: %.2f\n",
               gyro_x / 16.0, gyro_y / 16.0, gyro_z / 16.0);

        // Delay 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define BNO055_CALIB_STAT_ADDR 0x35

void bno055_calibration(void)
{
    uint8_t calib_stat;
    uint8_t sys, gyro, accel, mag;
    while (1)
    {
        bno055_read_bytes(BNO055_CALIB_STAT_ADDR, &calib_stat, 1);

        sys = (calib_stat >> 6) & 0x03;
        gyro = (calib_stat >> 4) & 0x03;
        accel = (calib_stat >> 2) & 0x03;
        mag = calib_stat & 0x03;

        printf("Calibration status: Sys=%d Gyro=%d Accel=%d Mag=%d\n", sys, gyro, accel, mag);

        // if (sys == 3 && gyro == 3 && accel == 3 && mag == 3)
        // {
        //     printf("Calibration completed successfully!\n");
        //     break;
        // }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    esp_err_t err = bno055_init();
    if (err != ESP_OK)
    {
        ESP_LOGE("BNO055", "Failed to initialize BNO055");
        return;
    }
    bno055_calibration();

    // Create tasks to read sensor data from BNO055
    // xTaskCreate(read_bno055_euler_angles, "read_bno055_euler_angles", 2048 * 2, NULL, 5, NULL);
    // xTaskCreate(read_bno055_accelerometer, "read_bno055_accelerometer", 2048 * 2, NULL, 5, NULL);
    xTaskCreate(read_bno055_magnetometer, "read_bno055_magnetometer", 2048 * 2, NULL, 5, NULL);
    // xTaskCreate(read_bno055_gyroscope, "read_bno055_gyroscope", 2048 * 2, NULL, 5, NULL);
}
