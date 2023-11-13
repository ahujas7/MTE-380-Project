/*
 * HCSR04.c
 *
 *  Created on: Nov. 10, 2023
 *      Author: ahuja
 */
// Include files

#include "HCSR04.h"

HAL_StatusTypeDef hcsr04_init(TIM_HandleTypeDef *timer) {

	HAL_StatusTypeDef ret = HAL_TIM_Base_Start(timer);

	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	return ret;
}


void hcsr04_get_distance(HCSR04_HandleTypeDef *dev, TIM_HandleTypeDef *timer) {

	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(timer, 0);
	while (__HAL_TIM_GET_COUNTER(timer) < 5);  // wait for 5 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	uint32_t current_time = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && current_time + 10 >  HAL_GetTick());
	uint32_t init_value = __HAL_TIM_GET_COUNTER(timer);

	current_time = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && current_time + 10 > HAL_GetTick());
	uint32_t final_value = __HAL_TIM_GET_COUNTER(timer);

	 dev->distance = (final_value - init_value) * 0.034/2;
}
