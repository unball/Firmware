#ifndef IMU_H
#define IMU_H

#define SCL_PIN GPIO_NUM_35
#define SDA_PIN GPIO_NUM_33
#define I2C_ADDR 0x68
#define I2C_PORT I2C_NUM_0

#include "esp_err.h"

esp_err_t setup();
esp_err_t get_w();

#endif