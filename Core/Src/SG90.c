/*
 * SG90.c
 *
 *  Created on: Nov 10, 2023
 *      Author: avane
 */
#include "SG90.h"

HAL_StatusTypeDef sg90_init(TIM_HandleTypeDef *timer) {

	HAL_StatusTypeDef ret = HAL_TIM_Base_Start(timer);

	ret = HAL_TIM_PWM_Start(timer, SERVO_CHANNEL);

	return ret;
}

void sg90_close(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer) {

	for(int i = 475; i < 1400; i += 35){

	__HAL_TIM_SET_COMPARE(timer, SERVO_CHANNEL, i);

	HAL_Delay(20);

	}

	dev->position = 1;
}

void sg90_open(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer) {

	for(int i=1400; i > 475; i -= 35){

	__HAL_TIM_SET_COMPARE(timer, SERVO_CHANNEL, i);

	HAL_Delay(20);

	}

	dev->position = 0;
}
