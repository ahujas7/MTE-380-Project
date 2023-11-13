/*
 * HCSR04.h
 *
 *  Created on: Nov. 10, 2023
 *      Author: ahuja
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "stm32f4xx_hal.h"

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOC

// Device Struct
typedef struct __HCSR04_HandleTypeDef {

    uint32_t distance; // in cm

} HCSR04_HandleTypeDef;


HAL_StatusTypeDef hcsr04_init(HCSR04_HandleTypeDef *dev, TIM_HandleTypeDef *timer);

void hcsr04_get_distance(HCSR04_HandleTypeDef *dev, TIM_HandleTypeDef *timer);

#endif /* INC_HCSR04_H_ */
