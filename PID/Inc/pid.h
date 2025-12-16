//
// Created by sy on 2025/12/16.
//

#ifndef PARALLEL_LEG_CHASSIS_PID_H
#define PARALLEL_LEG_CHASSIS_PID_H

#include "main.h"
#define PID_POSITION 0
#define PID_DELTA 1

typedef struct
{
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern void PID_clear(pid_type_def *pid);
extern float PID_calculate(pid_type_def *pid, float set, float fdb);

#endif //PARALLEL_LEG_CHASSIS_PID_H