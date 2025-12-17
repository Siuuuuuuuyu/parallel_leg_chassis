//
// Created by sy on 2025/12/17.
//

#ifndef PARALLEL_LEG_CHASSIS_DJI_MOTOR_CTRL_H
#define PARALLEL_LEG_CHASSIS_DJI_MOTOR_CTRL_H

#include "main.h"
#include "pid.h"

#define M3508_SPEED_PID_KP 1.0f
#define M3508_SPEED_PID_KI 0.0f
#define M3508_SPEED_PID_KD 0.0f
#define M3508_SPEED_PID_MAX_OUT 1000.0f
#define M3508_SPEED_PID_MAX_IOUT 1000.0f

typedef struct
{
    int16_t rotor_angle;
    int16_t rotor_speed;
    int16_t torque_current;
    int8_t temp;
} dji_motor_info;

extern dji_motor_info M3508_1;
extern pid_type_def m3508_speed_pid;

#endif //PARALLEL_LEG_CHASSIS_DJI_MOTOR_CTRL_H