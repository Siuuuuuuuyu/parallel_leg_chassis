//
// Created by sy on 2026/1/2.
//

#ifndef PARALLEL_LEG_CHASSIS_CHASSIS_CTRL_H
#define PARALLEL_LEG_CHASSIS_CHASSIS_CTRL_H

#include "dji_motor_ctrl.h"
#include "dm_motor_ctrl.h"
#include "pid.h"
#include "imu_task.h"
#include "LegController.h"
#include "StateObserver.h"

typedef struct
{
    pid_type_def pid_leg_length_average;
    pid_type_def pid_roll;
    uint32_t count;
    uint32_t dwt_count;
} plc_handler1_t;

typedef struct
{
    pid_type_def pid_leg_length_average;
    pid_type_def pid_roll;
    uint32_t count;
    uint32_t dwt_count;
} plc_handler2_t;

typedef struct
{
    uint32_t count;
    uint32_t dwt_count;
} plc_handler3_t;

typedef struct
{
    pid_type_def pid_leg_length_left;
    pid_type_def pid_leg_length_right;
    pid_type_def pid_leg_theta_left;
    pid_type_def pid_leg_theta_right;
    uint32_t count;
    uint32_t dwt_count;
} plc_handler4_t;

extern leg_state_t leg_l;
extern leg_state_t leg_r;
extern leg_torque_t leg_tl;
extern leg_torque_t leg_tr;
extern jacobin_matrix_t leg_jl;
extern jacobin_matrix_t leg_jr;
extern ctrl_matrix_t k;

void plc_obs(double l_phi1, double l_phi2, double r_phi1, double r_phi2,
                double l_d_phi1, double l_d_phi2, double r_d_phi1, double r_d_phi2,
                double l_t1, double l_t2, double r_t1, double r_t2, float dt);
void plc_handler1(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, ctrl_matrix_t *k,
                float yaw, float d_yaw, float phi, float d_phi, float roll, float d_roll, float s, float dt);
void plc_handler2(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, ctrl_matrix_t *k,
                float yaw, float d_yaw, float phi, float d_phi, float roll, float d_roll, float dt);
void plc_handler3(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r);
void plc_handler4(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, float dt);

#endif //PARALLEL_LEG_CHASSIS_CHASSIS_CTRL_H