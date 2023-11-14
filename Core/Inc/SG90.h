/*
 * SG90.h
 *
 *  Created on: Nov 10, 2023
 *      Author: avane
 */

#ifndef INC_SG90_H_
#define INC_SG90_H_


#include "stm32f4xx_hal.h"

#define SERVO_CHANNEL TIM_CHANNEL_2

typedef struct __SG90_HandleTypeDef {

	//0 for open 1 for close
   int position;

} SG90_HandleTypeDef;

HAL_StatusTypeDef sg90_init(TIM_HandleTypeDef *timer);

void sg90_open(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer);

void sg90_close(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer);

#endif /* INC_SG90_H_ */
