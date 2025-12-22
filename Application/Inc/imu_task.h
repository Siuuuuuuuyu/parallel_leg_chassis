//
// Created by sy on 2025/12/16.
//
#ifndef PARALLEL_LEG_CHASSIS_IMU_TASK_H
#define PARALLEL_LEG_CHASSIS_IMU_TASK_H

#include "main.h"
#include "pid.h"
#include "QuaternionEKF.h"
#include "tim.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;

void INS_init(void);
void imu_task (void const * argument);
void get_angle(float q0, float q1, float q2, float q3, float angle[3]);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif //PARALLEL_LEG_CHASSIS_IMU_TASK_H