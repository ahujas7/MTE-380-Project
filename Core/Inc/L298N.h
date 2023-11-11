/*
 * L298N.h
 *
 *  Created on: Nov 10, 2023
 *      Author: ahuja
 */

#ifndef INC_L298N_H_
#define INC_L298N_H_

#include "stm32f4xx_hal.h"

#define COUNTER_PERIOD 			625

#define LEFT_MOTOR_CHANNEL		TIM_CHANNEL_2
#define RIGHT_MOTOR_CHANNEL		TIM_CHANNEL_3


// Device Struct
typedef struct __L298N_HandleTypeDef {

	int left_duty;
    int right_duty;

} L298N_HandleTypeDef;

HAL_StatusTypeDef l298n_init(TIM_HandleTypeDef *timer);

void l298n_brake();

void l298n_drive_forward(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right);

void l298n_drive_reverse(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right);

void l298n_rotate_clockwise(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right);

void l298n_rotate_counter(L298N_HandleTypeDef *dev, TIM_HandleTypeDef *timer, int duty_left, int duty_right);

void set_motor_gpios(GPIO_PinState in1, GPIO_PinState in2, GPIO_PinState in3, GPIO_PinState in4);


#endif /* INC_L298N_H_ */
