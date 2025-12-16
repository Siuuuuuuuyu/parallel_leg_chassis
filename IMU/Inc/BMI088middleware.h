//
// Created by sy on 2025/12/15.
//

#ifndef PARALLEL_LEG_CHASSIS_BMI088MIDDLEWARE_H
#define PARALLEL_LEG_CHASSIS_BMI088MIDDLEWARE_H

#include "stdint.h"
#include "main.h"
#include "stm32h7xx_hal.h"

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t tx_data);

#endif //PARALLEL_LEG_CHASSIS_BMI088MIDDLEWARE_H