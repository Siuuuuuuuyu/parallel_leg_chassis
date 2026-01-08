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

#define LEFT_LEG_LENGTH_PID_KP 1200.0f
#define LEFT_LEG_LENGTH_PID_KI 0.0f
#define LEFT_LEG_LENGTH_PID_KD 20.0f
#define LEFT_LEG_LENGTH_PID_MAX_OUT 5000.0f
#define LEFT_LEG_LENGTH_PID_MAX_IOUT 500.0f

#define RIGHT_LEG_LENGTH_PID_KP 1200.0f
#define RIGHT_LEG_LENGTH_PID_KI 0.0f
#define RIGHT_LEG_LENGTH_PID_KD 20.0f
#define RIGHT_LEG_LENGTH_PID_MAX_OUT 5000.0f
#define ROGHT_LEG_LENGTH_PID_MAX_IOUT 500.0f

#define LEG_THETA_PID_KP 100.0f
#define LEG_THETA_PID_KI 0.0f
#define LEG_THETA_PID_KD 0.0f
#define LEG_THETA_PID_MAX_OUT 1000.0f
#define LEG_THETA_PID_MAX_IOUT 1000.0f

#define YAW_PID_KP 100.0f
#define YAW_PID_KI 0.0f
#define YAW_PID_KD 0.0f
#define YAW_PID_MAX_OUT 500.0f
#define YAW_PID_MAX_IOUT 500.0f

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT 5000.0f
#define ROLL_PID_MAX_IOUT 1000.0f

typedef struct
{
    // float Xb, Yb, Xc, Yc, Xd, Yd;
    // float lbd;
    // float phi_2;
    // float phi_3;
    // float ABC;

    float l0;
    float last_l0;
    float d_l0;
    float phi_0;
    float last_phi_0;
    float d_phi_0;
    float theta;
    float last_theta;
    float d_theta;

    float F;    // 足端输出力
    float Tp;   // 髋关节输出力矩
    float Fi;   // 足端所受外力
    float Tpi;  // 足端所受外力矩
    float Tp_forward_feed;

    float intermediate_var1;
    float intermediate_var2;
    float intermediate_var3;

    float ml; // 腿部质量
} leg_state_t;

typedef struct
{
    float T1, T2; // 关节电机力矩
    float Tw; // 轮电机力矩
} leg_torque_t;

typedef struct
{
    // arm_matrix_instance_f32 J;      // 将足端力映射到关节电机力矩的雅可比矩阵
    arm_matrix_instance_f32 Jf;
    arm_matrix_instance_f32 Ji;
    arm_matrix_instance_f32 Jmc_phi;
    arm_matrix_instance_f32 Jmc_ltheta;
    arm_matrix_instance_f32 Jtemp;

    // float *J_data;
    float *Jf_data;
    float *Ji_data;
    float *Jmc_phi_data;
    float *Jmc_ltheta_data;
    float *Jtemp_data;
} jacobin_matrix_t;

typedef struct
{
    // arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 X;      // 状态向量
    arm_matrix_instance_f32 Xd;     // 期望状态
    arm_matrix_instance_f32 U;
    arm_matrix_instance_f32 F;      // 足端期望输出力与力矩
    arm_matrix_instance_f32 Xtemp;  // 临时矩阵，储存期望状态与当前状态差值
    arm_matrix_instance_f32 Ttemp;  // 临时矩阵，储存关节电机输出力矩
    arm_matrix_instance_f32 Klqr;
    arm_matrix_instance_f32 Kk1;
    arm_matrix_instance_f32 Kk2;
    arm_matrix_instance_f32 Kk3;
    arm_matrix_instance_f32 Kk4;
    arm_matrix_instance_f32 Kk5;
    arm_matrix_instance_f32 Kk6;
    arm_matrix_instance_f32 Ktemp;
    arm_matrix_instance_f32 Kl;

    // float *K_data;
    float *X_data;
    float *Xd_data;
    float *U_data;
    float *F_data;
    float *Xtemp_data;
    float *Ttemp_data;
    float *Klqr_data;
    float *Kk1_data;
    float *Kk2_data;
    float *Kk3_data;
    float *Kk4_data;
    float *Kk5_data;
    float *Kk6_data;
    float *Ktemp_data;
    float *Kl_data;

} ctrl_matrix_t;

extern const float rw;
extern const float rl;
extern const float mb;
extern const float g;
extern const float Fi_k;

void leg_controller_init(leg_state_t *leg_l, leg_state_t *leg_r, jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, ctrl_matrix_t *k);

void rod_mass_center(float x1, float y1, float x0, float y0, float lamda, float *x, float *y);
void leg_kinematic(leg_state_t *leg, jacobin_matrix_t *j, float phi1, float phi2, float d_phi1, float d_phi2);
void get_K_LQR(float l0_l, float l0_r, ctrl_matrix_t *k);
void get_Tout(leg_state_t *leg, leg_torque_t *t, jacobin_matrix_t *j);
void get_Fu(leg_state_t *leg, leg_torque_t *t, jacobin_matrix_t *mat);

void get_leg_state(leg_state_t *leg, jacobin_matrix_t *mat, float phi, float phi_1, float phi_4, float dt);
void get_K(leg_state_t *leg, jacobin_matrix_t *mat);
void get_torque(leg_state_t *leg, jacobin_matrix_t *mat, leg_torque_t *t, float phi, float d_phi, float xb, float d_xb);
void leg_length_ctrl(leg_state_t *leg_l, leg_state_t * leg_r, float Ll, float Lr, float roll);
void leg_theta_ctrl(leg_state_t *leg_l, leg_state_t * leg_r);
void yaw_ctrl(leg_torque_t *leg_tl, leg_torque_t * leg_tr, float yaw);

#endif //PARALLEL_LEG_CHASSIS_LEGCONTROLLER_H