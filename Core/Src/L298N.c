/*
 * L298N.c
 *
 *  Created on: Nov 10, 2023
 *      Author: ahuja
 */

#include "L298N.h"

HAL_StatusTypeDef l298n_init(TIM_HandleTypeDef *timer) {

	HAL_StatusTypeDef ret = HAL_TIM_PWM_Start(timer, LEFT_MOTOR_CHANNEL);

	ret = HAL_TIM_PWM_Start(timer, RIGHT_MOTOR_CHANNEL);

	return ret;

}

void l298n_brake() {

	set_motor_gpios(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);

}

void l298n_drive_forward(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right) {

	dev->left_duty = duty_left;
	dev->right_duty = duty_right;

	set_motor_gpios(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(timer, LEFT_MOTOR_CHANNEL, ((double)dev->left_duty / 100) * COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(timer, RIGHT_MOTOR_CHANNEL, ((double)dev->right_duty / 100) * COUNTER_PERIOD);

}

void l298n_drive_reverse(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right) {

	dev->left_duty = duty_left;
	dev->right_duty = duty_right;

	set_motor_gpios(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(timer, LEFT_MOTOR_CHANNEL, ((double)dev->left_duty / 100) * COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(timer, RIGHT_MOTOR_CHANNEL, ((double)dev->right_duty / 100) * COUNTER_PERIOD);

}

void l298n_rotate_clockwise(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right) {

	dev->left_duty = duty_left;
	dev->right_duty = duty_right;

	set_motor_gpios(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(timer, LEFT_MOTOR_CHANNEL, ((double)dev->left_duty / 100) * COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(timer, RIGHT_MOTOR_CHANNEL, ((double)dev->right_duty / 100) * COUNTER_PERIOD);

}

void l298n_rotate_counter(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right) {

	dev->left_duty = duty_left;
	dev->right_duty = duty_right;

	set_motor_gpios(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(timer, LEFT_MOTOR_CHANNEL, ((double)dev->left_duty / 100) * COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(timer, RIGHT_MOTOR_CHANNEL, ((double)dev->right_duty / 100) * COUNTER_PERIOD);

}

void set_motor_gpios(GPIO_PinState in1, GPIO_PinState in2, GPIO_PinState in3, GPIO_PinState in4) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, in1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, in2);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, in3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, in4);

}
