/*
 * SG90.h
 *
 *  Created on: Nov 10, 2023
 *      Author: avane
 */

#ifndef INC_SG90_H_
#define INC_SG90_H_


#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "main.h"

#define TIMER_CHANNEL TIM_CHANNEL_2

typedef struct __SG90_HandleTypeDef {

	//0 for close 1 for open
   int position;

} SG90_HandleTypeDef;

HAL_StatusTypeDef sg90_open(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer);
HAL_StatusTypeDef sg90_close(SG90_HandleTypeDef *dev, TIM_HandleTypeDef *timer);

#endif /* INC_SG90_H_ */
