//
// Created by sy on 2025/12/19.
//

#ifndef PARALLEL_LEG_CHASSIS_LEGCONTROLLER_H
#define PARALLEL_LEG_CHASSIS_LEGCONTROLLER_H

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#include "stdlib.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "pid.h"

typedef struct
{
    float Xb, Yb, Xc, Yc, Xd, Yd;
    float lbd;
    float phi_2;
    float phi_3;
    float ABC;

    float l0;
    float phi_0;
    float last_phi_0;
    float d_phi_0;
} leg_position_t;

typedef struct
{
    float T1, T2; // 关节电机力矩
    float Tw; // 轮电机力矩
} leg_torque_t;

typedef struct
{
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 X; // 状态向量
    arm_matrix_instance_f32 Xd; // 期望状态
    arm_matrix_instance_f32 U;
    arm_matrix_instance_f32 J; // 将足端力映射到关节电机力矩的雅可比矩阵
    arm_matrix_instance_f32 F; // 足端期望输出力与力矩
    arm_matrix_instance_f32 Xtemp; // 临时矩阵，储存期望状态与当前状态差值
    arm_matrix_instance_f32 Ttemp; // 临时矩阵，储存关节电机输出力矩
    float *K_data;
    float *X_data;
    float *Xd_data;
    float *U_data;
    float *J_data;
    float *F_data;
    float *Xtemp_data;
    float *Ttemp_data;
} ctrl_matrix_t;

void leg_matrix_init(ctrl_matrix_t *K);
void get_leg_state(leg_position_t *leg, ctrl_matrix_t *mat, float phi_1, float phi_4, float dt);
void get_K(ctrl_matrix_t *K, float l0);
void get_torque(leg_position_t *leg, ctrl_matrix_t *mat, leg_torque_t *t, float phi, float d_phi, float d_x, float F);

#endif //PARALLEL_LEG_CHASSIS_LEGCONTROLLER_H