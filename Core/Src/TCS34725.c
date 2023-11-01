/*
 *      @file   tcs34725.c
 *      @brief  Implementation
 *      @author Saksham Ahuja
 *
 */

// Include files

#include "tcs34725.h"

HAL_StatusTypeDef tcs34725_get_data(TCS34725_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c_device) {

    // TODO: Check status to see if data is valid and RGBC cycle has completed

	uint8_t val[8] = {0};
	uint8_t command = TCS34725_ITER_COMM | TCS34725_RGBC_REG;

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&command, 1, 1000);

	ret = HAL_I2C_Master_Receive(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&val, 8, 1000);

	dev->clear = TO_16_BIT(val[1], val[0]);
	dev->red = TO_16_BIT(val[3], val[2]);
	dev->green = TO_16_BIT(val[5], val[4]);
	dev->blue = TO_16_BIT(val[7], val[6]);


    return ret;
}

HAL_StatusTypeDef tcs34725_get_device_id(TCS34725_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c_device) {

	uint8_t val = 0x00;
	uint8_t command = TCS34725_COMM_BIT | TCS34725_DEVICE_ID_REG;

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&command, 1, 1000);

	ret = HAL_I2C_Master_Receive(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&val, 1, 1000);


	dev->device_id = val;

	return ret;
}

HAL_StatusTypeDef tcs34725_set_enable_reg(TCS34725_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c_device) {

	uint8_t val = 0x00;
	uint8_t command[2] = {TCS34725_COMM_BIT | TCS34725_ENABLE_REG, 0x01};


	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&command, 2, 1000);

	command[1] = 0x03;

	ret = HAL_I2C_Master_Transmit(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&command, 2, 1000);

	ret = HAL_I2C_Master_Receive(hi2c_device, TCS34725_DEV_ADDR << 1, (uint8_t*)&val, 1, 1000);

	dev->enable_reg = val;

	return ret;
}



