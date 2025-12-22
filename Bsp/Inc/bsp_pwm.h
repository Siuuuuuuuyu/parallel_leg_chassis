//
// Created by sy on 2025/12/22.
//

#ifndef PARALLEL_LEG_CHASSIS_BSP_PWM_H
#define PARALLEL_LEG_CHASSIS_BSP_PWM_H

#include "main.h"
void bsp_pwm_set(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#endif //PARALLEL_LEG_CHASSIS_BSP_PWM_H