/*
 * SG90.c
 *
 *  Created on: Nov 10, 2023
 *      Author: avane
 */
#include"SG90.h"


HAL_StatusTypeDef sg90_open(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer) {

	uint16_t i=0;

	for(i=475;i<1400;i+=35){

	__HAL_TIM_SET_COMPARE(timer, TIMER_CHANNEL, i);
	HAL_Delay(20);

	}
}

HAL_StatusTypeDef sg90_close(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer) {

	uint16_t j=0;

	for(j=1400;j>475;j-=35){

	__HAL_TIM_SET_COMPARE(timer, TIMER_CHANNEL, j);
	HAL_Delay(20);

	}
}
