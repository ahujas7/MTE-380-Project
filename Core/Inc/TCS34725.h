/*
 * TCS34725.h
 *
 *  Created on: Oct 6, 2023
 *      Author: ahuja
 */

#ifndef INC_TCS34725_H_
#define INC_TCS34725_H_

#include "stm32f4xx_hal.h"

#define TCS34725_DEV_ADDR               0x29

// MAX17320G2 Data Registers
#define TCS34725_ENABLE_REG          	0x00
#define TCS34725_RGBC_REG           	0x14
#define TCS34725_DEVICE_ID_REG          0x12



#define TCS34725_COMM_BIT				0x80
#define TCS34725_ITER_COMM				0xA0


#define TO_16_BIT(b1, b2)				((uint16_t)b1 << 8) | (uint16_t)b2

// Device Struct
typedef struct __TCS34725_HandleTypeDef {

    I2C_HandleTypeDef *i2c_handler;

    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;

    uint8_t device_id;
    uint8_t enable_reg;

    double r_ratio;
    double g_ratio;
    double b_ratio;

} TCS34725_HandleTypeDef;


HAL_StatusTypeDef tcs34725_get_data(TCS34725_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c_device);

HAL_StatusTypeDef tcs34725_get_device_id(TCS34725_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c_device);

HAL_StatusTypeDef tcs34725_set_enable_reg(TCS34725_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c_device);

#endif /* INC_TCS34725_H_ */
