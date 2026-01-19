//
// Created by sy on 2026/1/2.
//
#include "chassis_ctrl.h"

const float plc_h1_pid_leg_length_average[3] = {2000.0f, 0.0f, 2000.0f};
const float plc_h1_pid_roll[3] = {1500.0f, 0.0f, 100.0f};

const float plc_h2_pid_leg_length_average[3] = {2000.0f, 0.0f, 2000.0f};
const float plc_h2_pid_roll[3] = {1500.0f, 0.0f, 100.0f};

const float plc_h4_pid_leg_length_left[3] = {1500.0f, 0.0f, 1000.0f};
const float plc_h4_pid_leg_length_right[3] = {1500.0f, 0.0f, 1000.0f};
const float plc_h4_pid_leg_theta_left[3] = {20.0f, 0.0f, 5.0f};
const float plc_h4_pid_leg_theta_right[3] = {20.0f, 0.0f, 5.0f};

const float plc_h5_pid_leg_length_left[3] = {1500.0f, 0.0f, 1000.0f};
const float plc_h5_pid_leg_length_right[3] = {1500.0f, 0.0f, 1000.0f};
const float plc_h5_pid_leg_theta_left[3] = {20.0f, 0.0f, 5.0f};
const float plc_h5_pid_leg_theta_right[3] = {20.0f, 0.0f, 5.0f};

leg_state_t leg_l;
leg_state_t leg_r;
leg_torque_t leg_tl;
leg_torque_t leg_tr;
jacobin_matrix_t leg_jl;
jacobin_matrix_t leg_jr;
ctrl_matrix_t k;
extern double s;
extern float roll_cmd;
extern float leg_length_cmd;

float F_add;

void plc_obs(double l_phi1, double l_phi2, double r_phi1, double r_phi2,
                double l_d_phi1, double l_d_phi2, double r_d_phi1, double r_d_phi2,
                double l_t1, double l_t2, double r_t1, double r_t2, float dt)
{
    leg_kinematic(&leg_l, &leg_jl, l_phi1, l_phi2, l_d_phi1, l_d_phi2);
    leg_kinematic(&leg_r, &leg_jr, r_phi1, r_phi2, r_d_phi1, r_d_phi2);
    get_Fn(l_t1, l_t2, &leg_l, &leg_jl);
    get_Fn(r_t1, r_t2, &leg_r, &leg_jr);

    double wl = - M3508_1.m.para.vel_fb / M3508_1.speed_ratio;
    double wr = M3508_2.m.para.vel_fb / M3508_2.speed_ratio;
    velocity_kf_update(wl, wr, INS.Gyro[2], INS.MotionAccel_b[0], dt);
}

plc_handler1_t plc_h1 = {.count = 0, .dwt_count = 0};
void plc_handler1(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, ctrl_matrix_t *k,
                float yaw, float d_yaw, float phi, float d_phi, float roll, float d_roll, float dt)
{
    if (plc_h1.count == 0)
    {
        PID_init(&plc_h1.pid_leg_length_average, PID_POSITION, plc_h1_pid_leg_length_average, 500, 50);
        PID_init(&plc_h1.pid_roll, PID_POSITION, plc_h1_pid_roll, 600, 50);
    }
    float delta_t = DWT_GetDeltaT(&plc_h1.dwt_count);
    if (delta_t - dt > dt * 5)
    {
        // 两次调用时间过长，清除pid内部数据
        PID_clear(&plc_h1.pid_leg_length_average);
        PID_clear(&plc_h1.pid_roll);
    }
    // 状态变量
    k->X_data[0] = 0;                 // 机体位移
    k->X_data[1] = v_kf.v;            // 机体速度
    k->X_data[2] = yaw;               // 机体偏航角
    k->X_data[3] = d_yaw;             // 机体偏航角速度
    k->X_data[4] = leg_l->theta;      // 左腿倾角
    k->X_data[5] = leg_l->d_theta;    // 左腿倾角速度
    k->X_data[6] = leg_r->theta;      // 右腿倾角
    k->X_data[7] = leg_r->d_theta;    // 右腿倾角速度
    k->X_data[8] = phi;               // 机体俯仰角
    k->X_data[9] = d_phi;             // 机体俯仰角速度

    arm_mat_sub_f32(&k->Xd, &k->X, &k->Xtemp);
    get_K_LQR(leg_l->l0, leg_r->l0, k);
    arm_mat_mult_f32(&k->Klqr, &k->Xtemp, &k->U);

    t_l->Tw = k->U_data[0];
    t_r->Tw = k->U_data[1];

    float leg_length_average = (leg_l->l0 + leg_r->l0) / 2.0f;
    float pid_leg_length_out = PID_calculate(&plc_h1.pid_leg_length_average, 0.25f, leg_length_average);
    float pid_roll_out = PID_calculate_d(&plc_h1.pid_roll, 0.0f, roll, d_roll);

    float Fg = mb * g / 2.0f;
    float Fi = Fi_k * v_kf.v * d_yaw;

    leg_l->F = Fg - Fi + pid_leg_length_out + pid_roll_out;
    leg_r->F = Fg + Fi + pid_leg_length_out - pid_roll_out;
    leg_l->Tp = k->U_data[2];
    leg_r->Tp = k->U_data[3];

    get_Tout(leg_l, t_l, j_l);
    get_Tout(leg_r, t_r, j_r);

    plc_h1.count ++;
}

// 无位移闭环控制
plc_handler2_t plc_h2 = {.count = 0, .dwt_count = 0};
void plc_handler2(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, ctrl_matrix_t *k,
                float yaw, float d_yaw, float phi, float d_phi, float roll, float d_roll, float dt)
{
    if (plc_h2.count == 0)
    {
        PID_init(&plc_h2.pid_leg_length_average, PID_POSITION, plc_h2_pid_leg_length_average, 500, 50);
        PID_init(&plc_h2.pid_roll, PID_POSITION, plc_h2_pid_roll, 600, 50);
    }
    float delta_t = DWT_GetDeltaT(&plc_h2.dwt_count);
    if (delta_t - dt > dt * 5)
    {
        // 两次调用时间过长，清除pid内部数据
        PID_clear(&plc_h2.pid_leg_length_average);
        PID_clear(&plc_h2.pid_roll);
    }
    // 状态变量
    k->X_data[0] = 0;                 // 机体位移
    k->X_data[1] = v_kf.v;            // 机体速度
    k->X_data[2] = yaw;               // 机体偏航角
    k->X_data[3] = d_yaw;             // 机体偏航角速度
    k->X_data[4] = leg_l->theta;      // 左腿倾角
    k->X_data[5] = leg_l->d_theta;    // 左腿倾角速度
    k->X_data[6] = leg_r->theta;      // 右腿倾角
    k->X_data[7] = leg_r->d_theta;    // 右腿倾角速度
    k->X_data[8] = phi;               // 机体俯仰角
    k->X_data[9] = d_phi;             // 机体俯仰角速度

    arm_mat_sub_f32(&k->Xd, &k->X, &k->Xtemp);
    get_K_LQR(leg_l->l0, leg_r->l0, k);
    arm_mat_mult_f32(&k->Klqr, &k->Xtemp, &k->U);

    t_l->Tw = k->U_data[0];
    t_r->Tw = k->U_data[1];

    float leg_length_average = (leg_l->l0 + leg_r->l0) / 2.0f;
    float pid_leg_length_out = PID_calculate(&plc_h2.pid_leg_length_average, 0.2f + leg_length_cmd, leg_length_average);
    float pid_roll_out = PID_calculate_d(&plc_h2.pid_roll, 0.0f + roll_cmd, roll, d_roll);

    float Fg = mb * g / 2.0f;
    float Fi = Fi_k * v_kf.v * d_yaw;

    leg_l->F = Fg - Fi + pid_leg_length_out + pid_roll_out;
    leg_r->F = Fg + Fi + pid_leg_length_out - pid_roll_out;
    leg_l->Tp = k->U_data[2];
    leg_r->Tp = k->U_data[3];

    get_Tout(leg_l, t_l, j_l);
    get_Tout(leg_r, t_r, j_r);

    plc_h2.count ++;
}

plc_handler3_t plc_h3 = {.count = 0, .dwt_count = 0};
void plc_handler3(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r)
{
    t_l->Tw = 0.0f;
    t_r->Tw = 0.0f;
    leg_l->F = 0.0f;
    leg_l->Tp = 0.0f;
    leg_r->F = 0.0f;
    leg_r->Tp = 0.0f;

    get_Tout(leg_l, t_l, j_l);
    get_Tout(leg_r, t_r, j_r);
}

// 离地处理
plc_handler4_t plc_h4 = {.count = 0, .dwt_count = 0};
void plc_handler4(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, float phi, float dt)
{
    if (plc_h4.count == 0)
    {
        PID_init(&plc_h4.pid_leg_length_left, PID_POSITION, plc_h4_pid_leg_length_left, 500, 50);
        PID_init(&plc_h4.pid_leg_length_right, PID_POSITION, plc_h4_pid_leg_length_right, 500, 50);
        PID_init(&plc_h4.pid_leg_theta_left, PID_POSITION, plc_h4_pid_leg_theta_left, 50, 10);
        PID_init(&plc_h4.pid_leg_theta_right, PID_POSITION, plc_h4_pid_leg_theta_right, 50, 10);
    }
    float delta_t = DWT_GetDeltaT(&plc_h4.dwt_count);
    if (delta_t - dt > dt * 5)
    {
        // 两次调用时间过长，清除pid内部数据
        PID_clear(&plc_h4.pid_leg_length_left);
        PID_clear(&plc_h4.pid_leg_length_right);
        PID_clear(&plc_h4.pid_leg_theta_left);
        PID_clear(&plc_h4.pid_leg_theta_right);
    }
    float pid_leg_length_left_out = PID_calculate(&plc_h4.pid_leg_length_left, 0.32f, leg_l->l0);
    float pid_leg_length_right_out = PID_calculate(&plc_h4.pid_leg_length_right, 0.32f, leg_r->l0);
    // float Fg = mb * g / 2.0f;

    float leg_theta_left_target = - phi;
    if (leg_theta_left_target > 0.3f)
        leg_theta_left_target = 0.3f;
    else if (leg_theta_left_target < - 0.3f)
        leg_theta_left_target = - 0.3f;

    float leg_theta_right_target = - phi;
    if (leg_theta_right_target > 0.3f)
        leg_theta_right_target = 0.3f;
    else if (leg_theta_right_target < - 0.3f)
        leg_theta_right_target = - 0.3f;

    float pid_leg_theta_left_out = PID_calculate(&plc_h4.pid_leg_theta_left, leg_theta_left_target, leg_l->theta);
    float pid_leg_theta_right_out = PID_calculate(&plc_h4.pid_leg_theta_right, leg_theta_right_target, leg_r->theta);
    leg_l->F = pid_leg_length_left_out;
    leg_r->F = pid_leg_length_right_out;
    leg_l->Tp = pid_leg_theta_left_out;
    leg_r->Tp = pid_leg_theta_right_out;
    t_l->Tw = 0.0f;
    t_r->Tw = 0.0f;

    get_Tout(leg_l, t_l, j_l);
    get_Tout(leg_r, t_r, j_r);

    plc_h4.count ++;
}

// 跳跃状态下离地处理
plc_handler5_t plc_h5 = {.count = 0, .dwt_count = 0};
void plc_handler5(leg_state_t *leg_l, leg_state_t *leg_r, leg_torque_t *t_l, leg_torque_t *t_r,
                jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, float phi, float dt)
{
    if (plc_h5.count == 0)
    {
        PID_init(&plc_h5.pid_leg_length_left, PID_POSITION, plc_h5_pid_leg_length_left, 500, 50);
        PID_init(&plc_h5.pid_leg_length_right, PID_POSITION, plc_h5_pid_leg_length_right, 500, 50);
        PID_init(&plc_h5.pid_leg_theta_left, PID_POSITION, plc_h5_pid_leg_theta_left, 50, 10);
        PID_init(&plc_h5.pid_leg_theta_right, PID_POSITION, plc_h5_pid_leg_theta_right, 50, 10);
    }
    float delta_t = DWT_GetDeltaT(&plc_h5.dwt_count);
    if (delta_t - dt > dt * 5)
    {
        // 两次调用时间过长，清除pid内部数据
        PID_clear(&plc_h5.pid_leg_length_left);
        PID_clear(&plc_h5.pid_leg_length_right);
        PID_clear(&plc_h5.pid_leg_theta_left);
        PID_clear(&plc_h5.pid_leg_theta_right);
    }
    float pid_leg_length_left_out = PID_calculate(&plc_h5.pid_leg_length_left, 0.15f, leg_l->l0);
    float pid_leg_length_right_out = PID_calculate(&plc_h5.pid_leg_length_right, 0.15f, leg_r->l0);
    // float Fg = mb * g / 2.0f;

    float leg_theta_left_target = - phi;
    if (leg_theta_left_target > 0.2f)
        leg_theta_left_target = 0.2f;
    else if (leg_theta_left_target < - 0.2f)
        leg_theta_left_target = - 0.2f;

    float leg_theta_right_target = - phi;
    if (leg_theta_right_target > 0.2f)
        leg_theta_right_target = 0.2f;
    else if (leg_theta_right_target < - 0.2f)
        leg_theta_right_target = - 0.2f;

    float pid_leg_theta_left_out = PID_calculate(&plc_h5.pid_leg_theta_left, leg_theta_left_target, leg_l->theta);
    float pid_leg_theta_right_out = PID_calculate(&plc_h5.pid_leg_theta_right, leg_theta_right_target, leg_r->theta);
    leg_l->F = pid_leg_length_left_out;
    leg_r->F = pid_leg_length_right_out;
    leg_l->Tp = pid_leg_theta_left_out;
    leg_r->Tp = pid_leg_theta_right_out;
    t_l->Tw = 0.0f;
    t_r->Tw = 0.0f;

    get_Tout(leg_l, t_l, j_l);
    get_Tout(leg_r, t_r, j_r);

    plc_h5.count ++;
}
