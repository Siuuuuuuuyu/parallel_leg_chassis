//
// Created by sy on 2025/12/19.
//
#include "LegController.h"

const float l1 = 0.15f;
const float l4 = 0.15f;
const float l2 = 0.27f;
const float l3 = 0.27f;
const float l5 = 0.15f;

// const float left_leg_length_pid_param[3] = {LEFT_LEG_LENGTH_PID_KP, LEFT_LEG_LENGTH_PID_KI, LEFT_LEG_LENGTH_PID_KD};
// const float right_leg_length_pid_param[3] = {RIGHT_LEG_LENGTH_PID_KP, RIGHT_LEG_LENGTH_PID_KI, RIGHT_LEG_LENGTH_PID_KD};
// const float leg_theta_pid_param[3] = {LEG_THETA_PID_KP, LEG_THETA_PID_KI, LEG_THETA_PID_KD};
// const float yaw_pid_param[3] = {YAW_PID_KP, YAW_PID_KI, YAW_PID_KD};
// const float roll_pid_param[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
// static float mt1, mt2, mt3, mt4, mt5, mt6, mt7, mt8, mt9, mt10, mt11, mt12;

const float L1a = 0.075f;
const float L1u = 0.15f;
const float L1d = 0.27f;
const float L2a = 0.075f;
const float L2u = 0.15f;
const float L2d = 0.27f;
const float m1u = 0.2f;
const float m1d = 0.3f;
const float m2u = 0.2f;
const float m2d = 0.3f;
const float lamda_1u = 0.5f;
const float lamda_2u = 0.5f;
const float lamda_1d = 0.5f;
const float lamda_2d = 0.5f;
const float rw = 0.06f;  // 驱动轮半径
const float rl = 0.25f;  // 驱动轮距离/2
const float mb = 13.0f;  // 机体质量 kg
const float mw = 0.68f;  // 驱动轮质量 kg
const float g = 9.81f;
const float Fi_k = 10.0f;
const float Kk1[40] = {
    -1.98344082137506, -3.04830174891451, -2.15085934272261, -0.809433626061089, -4.75526308666410, -1.01663189742075, -2.24877732632585, -0.115297170523552, -22.5110313083925, -5.23565391381798,
    -1.98344082137515, -3.04830174891455, 2.15085934272251, 0.809433626061082, -2.24877732632591, -0.115297170523555, -4.75526308666408, -1.01663189742076, -22.5110313083926, -5.23565391381798,
    7.58469616312108, 10.7170642250178, -3.44466868938853, -1.29913644345027, 19.0508255209775, 4.94526579162321, 6.30048314797992, 0.284601941721598, -34.2576414413762, -6.15740938473985,
    7.58469616312127, 10.7170642250177, 3.44466868938877, 1.29913644345028, 6.30048314797989, 0.284601941721608, 19.0508255209774, 4.94526579162321, -34.2576414413765, -6.15740938473991
};
const float Kk2[40] = {
    -11.5751249978994, -14.1671451242927, 3.79417193089306, 1.64895724149684, -45.8544518258567, -5.59428487226512, 1.19711568751296, 0.0506230980190290, 48.7127649168945, 9.68810710765482,
    5.57144814945666, 8.39089884462188, 2.07470561761744, 1.00258534548602, -24.3552187683659, -3.23385835037916, 16.4975200260873, 0.979379684442611, 22.3549687348218, 6.33681144575201,
    5.30130619756094, 5.13842579114416, -5.28760418059888, -2.66780994343184, 39.3127193542424, -6.10083586197698, -21.2933518600373, -1.04600656466043, -206.034699006721, -43.7672413771459,
    -28.7474098739264, -41.1799403504768, 2.46559613516979, 0.901389948448784, -36.7823908753059, -2.60749380845665, -29.7912362148659, -1.72336171738908, 40.7526987081282, 4.59952621476416
};
const float Kk3[40] = {
    5.57144814945480, 8.39089884462023, -2.07470561761698, -1.00258534548597, 16.4975200260861, 0.979379684442377, -24.3552187683671, -3.23385835037931, 22.3549687348252, 6.33681144575192,
    -11.5751249979002, -14.1671451242936, -3.79417193089256, -1.64895724149683, 1.19711568751282, 0.0506230980189809, -45.8544518258579, -5.59428487226523, 48.7127649168988, 9.68810710765475,
    -28.7474098739272, -41.1799403504783, -2.46559613517151, -0.901389948448906, -29.7912362148676, -1.72336171738926, -36.7823908753068, -2.60749380845678, 40.7526987081212, 4.59952621476374,
    5.30130619755796, 5.13842579114309, 5.28760418059839, 2.66780994343183, -21.2933518600385, -1.04600656466071, 39.3127193542417, -6.10083586197723, -206.034699006726, -43.7672413771459
};
const float Kk4[40] = {
    -0.974937529476683, -4.07085463772101, 0.541592020335210, 0.210184432808498, 12.2311027881391, 2.65201419424424, -0.704690619869502, -2.31145151608020, -31.8612815923661, -8.06976577596999,
    -0.974937529474388, -4.07085463771841, -0.541592020334059, -0.210184432808431, -0.704690619866131, -2.31145151607963, 12.2311027881405, 2.65201419424441, -31.8612815923675, -8.06976577596960,
    32.5210149345266, 48.3618187158318, -4.79422300042142, -2.81664491692962, 69.8298853087878, 2.13321556492509, -18.8322439399846, 0.0158038738860642, 44.5826033172438, 14.9071510617334,
    32.5210149345280, 48.3618187158308, 4.79422300042414, 2.81664491692988, -18.8322439399878, 0.0158038738860665, 69.8298853087905, 2.13321556492548, 44.5826033172449, 14.9071510617330
};
const float Kk5[40] = {
    13.3446502042155, 17.0642676259363, -2.40572139072384, -0.799920459422237, 25.3029003663421, 0.216129759803786, 6.05075022819259, 0.445731121099333, -33.6784966805951, -5.60821572639804,
    -3.87787535081185, -6.02214487946379, -2.16295519993933, -1.02624933386589, 31.2845996382419, 3.20890209907571, -21.1210998303643, -1.55738773612268, -11.9321700461899, -3.95115421280919,
    -30.6653813390513, -39.0122041648927, 7.00761088094847, 3.35572475355542, -70.0750518059585, 7.51147248893334, 9.76219692678457, -0.520809608525403, 246.692209380384, 48.6369856760493,
    23.2363542083219, 33.0486162190288, -2.56331579910634, -0.649818620959061, 14.7736149116643, 0.333706401784968, 25.6101868695490, 0.919560950212557, -50.8344783630578, -6.70851997507905
};
const float Kk6[40] = {
    -3.87787535080601, -6.02214487945841, 2.16295519993791, 1.02624933386576, -21.1210998303606, -1.55738773612204, 31.2845996382467, 3.20890209907639, -11.9321700461979, -3.95115421280873,
    13.3446502042169, 17.0642676259375, 2.40572139072216, 0.799920459422186, 6.05075022819157, 0.445731121099216, 25.3029003663443, 0.216129759804006, -33.6784966806039, -5.60821572639801,
    23.2363542083235, 33.0486162190305, 2.56331579910853, 0.649818620959189, 25.6101868695531, 0.919560950212964, 14.7736149116638, 0.333706401785058, -50.8344783630435, -6.70851997507843,
    -30.6653813390459, -39.0122041648899, -7.00761088094883, -3.35572475355553, 9.76219692678873, -0.520809608524837, -70.0750518059586, 7.51147248893375, 246.692209380396, 48.6369856760494
};

pid_type_def left_leg_length_pid;
pid_type_def right_leg_length_pid;
pid_type_def leg_theta_pid;
pid_type_def yaw_pid;
pid_type_def roll_pid;

void leg_controller_init(leg_state_t *leg_l, leg_state_t *leg_r, jacobin_matrix_t *j_l, jacobin_matrix_t *j_r, ctrl_matrix_t *k)
{
    leg_l->Tp_forward_feed = 0.4f;
    leg_r->Tp_forward_feed = 0.4f;

    // 左腿雅可比矩阵
    j_l->Jf_data = user_malloc(sizeof(float) * 4);
    memset(j_l->Jf_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_l->Jf, 2, 2, j_l->Jf_data);
    j_l->Ji_data = user_malloc(sizeof(float) * 4);
    memset(j_l->Ji_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_l->Ji, 2, 2, j_l->Ji_data);
    j_l->Jmc_phi_data = user_malloc(sizeof(float) * 4);
    memset(j_l->Jmc_phi_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_l->Jmc_phi, 2, 2, j_l->Jmc_phi_data);
    j_l->Jmc_ltheta_data = user_malloc(sizeof(float) * 4);
    memset(j_l->Jmc_ltheta_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_l->Jmc_ltheta, 2, 2, j_l->Jmc_ltheta_data);
    j_l->Jtemp_data = user_malloc(sizeof(float) * 4);
    memset(j_l->Jtemp_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_l->Jtemp, 2, 2, j_l->Jtemp_data);

    // 右腿雅可比矩阵
    j_r->Jf_data = user_malloc(sizeof(float) * 4);
    memset(j_r->Jf_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_r->Jf, 2, 2, j_r->Jf_data);
    j_r->Ji_data = user_malloc(sizeof(float) * 4);
    memset(j_r->Ji_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_r->Ji, 2, 2, j_r->Ji_data);
    j_r->Jmc_phi_data = user_malloc(sizeof(float) * 4);
    memset(j_r->Jmc_phi_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_r->Jmc_phi, 2, 2, j_r->Jmc_phi_data);
    j_r->Jmc_ltheta_data = user_malloc(sizeof(float) * 4);
    memset(j_r->Jmc_ltheta_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_r->Jmc_ltheta, 2, 2, j_r->Jmc_ltheta_data);
    j_r->Jtemp_data = user_malloc(sizeof(float) * 4);
    memset(j_r->Jtemp_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&j_r->Jtemp, 2, 2, j_r->Jtemp_data);

    // // 初始化K矩阵
    // mat->K_data = user_malloc(sizeof(float) * 12);
    // memset(mat->K_data, 0, sizeof(float) * 12);
    // arm_mat_init_f32(&mat->K, 2, 6, mat->K_data);
    // 初始化状态向量
    k->X_data = user_malloc(sizeof(float) * 10);
    memset(k->X_data, 0, sizeof(float) * 10);
    arm_mat_init_f32(&k->X, 10, 1, k->X_data);
    // 初始化期望状态向量
    k->Xd_data = user_malloc(sizeof(float) * 10);
    memset(k->Xd_data, 0, sizeof(float) * 10);
    arm_mat_init_f32(&k->Xd, 10, 1, k->Xd_data);
    // 初始化控制矩阵
    k->U_data = user_malloc(sizeof(float) * 4);
    memset(k->U_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&k->U, 4, 1, k->U_data);
    // //初始化雅可比矩阵
    // mat->J_data = user_malloc(sizeof(float) * 4);
    // memset(mat->J_data, 0, sizeof(float) * 4);
    // arm_mat_init_f32(&mat->J, 2, 2, mat->J_data);
    // // 初始化足端力与力矩矩阵
    // k->F_data = user_malloc(sizeof(float) * 2);
    // memset(k->F_data, 0, sizeof(float) * 2);
    // arm_mat_init_f32(&k->F, 2, 1, k->F_data);
    // 初始化状态临时矩阵
    k->Xtemp_data = user_malloc(sizeof(float) * 10);
    memset(k->Xtemp_data, 0, sizeof(float) * 10);
    arm_mat_init_f32(&k->Xtemp, 10, 1, k->Xtemp_data);
    // 初始化力矩临时矩阵
    k->Ttemp_data = user_malloc(sizeof(float) * 4);
    memset(k->Ttemp_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&k->Ttemp, 4, 1, k->Ttemp_data);

    // 初始化K矩阵以及拟合后的各项系数
    k->Klqr_data = user_malloc(sizeof(float) * 40);
    memset(k->Klqr_data, 0, sizeof(float) * 40);
    arm_mat_init_f32(&k->Klqr, 4, 10, k->Klqr_data);

    k->Kk1_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kk1, 4, 10, k->Kk1_data);
    memcpy(k->Kk1_data, Kk1, sizeof(Kk1));
    k->Kk2_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kk2, 4, 10, k->Kk2_data);
    memcpy(k->Kk2_data, Kk2, sizeof(Kk2));
    k->Kk3_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kk3, 4, 10, k->Kk3_data);
    memcpy(k->Kk3_data, Kk3, sizeof(Kk3));
    k->Kk4_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kk4, 4, 10, k->Kk4_data);
    memcpy(k->Kk4_data, Kk4, sizeof(Kk4));
    k->Kk5_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kk5, 4, 10, k->Kk5_data);
    memcpy(k->Kk5_data, Kk5, sizeof(Kk5));
    k->Kk6_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kk6, 4, 10, k->Kk6_data);
    memcpy(k->Kk6_data, Kk6, sizeof(Kk6));

    k->Ktemp_data = user_malloc(sizeof(float) * 40);
    memset(k->Ktemp_data, 0, sizeof(float) * 40);
    arm_mat_init_f32(&k->Ktemp, 4, 10, k->Ktemp_data);

//     PID_init(&left_leg_length_pid, PID_POSITION, left_leg_length_pid_param, LEFT_LEG_LENGTH_PID_MAX_OUT, LEFT_LEG_LENGTH_PID_MAX_IOUT);
//     PID_init(&right_leg_length_pid, PID_POSITION, right_leg_length_pid_param, RIGHT_LEG_LENGTH_PID_MAX_OUT, ROGHT_LEG_LENGTH_PID_MAX_IOUT);
//     PID_init(&leg_theta_pid, PID_POSITION, leg_theta_pid_param, LEG_THETA_PID_MAX_OUT, LEG_THETA_PID_MAX_OUT);
//     PID_init(&roll_pid, PID_POSITION, roll_pid_param, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
}

void rod_mass_center(float x1, float y1, float x0, float y0, float lamda, float *x, float *y)
{
    *x = (x1 - x0) * lamda + x0;
    *y = (y1 - y0) * lamda + y0;
}

void leg_kinematic(leg_state_t *leg, jacobin_matrix_t *j, float phi1, float phi2, float d_phi1, float d_phi2)
{
    float x_B1 = - L1u * arm_cos_f32(phi1);
    float y_B1 = - L1u * arm_sin_f32(phi1);
    float x_B2 = - (L1a + L2a) + L2u * arm_cos_f32(phi2);
    float y_B2 = - L2u * arm_sin_f32(phi2);
    float delta_x = x_B1 - x_B2;
    float delta_y = y_B2 - y_B1;
    arm_sqrt_f32(powf(2 * delta_y * L1d, 2) + powf(2 * delta_x * L1d, 2), &leg->intermediate_var1);
    float phi3 = asinf((L2d * L2d - delta_x * delta_x - delta_y * delta_y - L1d * L1d) / leg->intermediate_var1)
            + atan2f(delta_x, delta_y);
    float x_C = x_B1 - L1d * arm_cos_f32(phi3);
    float y_C = y_B1 - L1d * arm_sin_f32(phi3);
    arm_sqrt_f32(powf(x_C + L1a, 2) + powf(y_C, 2), &leg->l0);
    leg->theta = asinf((- L1a - x_C) / leg->l0);
    float A1 = x_B1 - x_B2 - L1d * arm_cos_f32(phi3);
    float B1 = y_B2 - y_B1 + L1d * arm_sin_f32(phi3);
    float C0 = L1d * arm_sin_f32(phi3) / (A1 * L1d * arm_sin_f32(phi3) + B1 * L1d * arm_cos_f32(phi3));
    float D0 = - L1d * arm_cos_f32(phi3) / (A1 * L1d * arm_sin_f32(phi3) + B1 * L1d * arm_cos_f32(phi3));
    float M11 = (1 - A1 * C0) * L1u * arm_sin_f32(phi1) - B1 * C0 * L1u * arm_cos_f32(phi1);
    float M12 = - A1 * C0 * L2u * arm_sin_f32(phi2) + B1 * C0 * L2u * arm_cos_f32(phi2);
    float M21 = - A1 * D0 * L1u * arm_sin_f32(phi1) - (1 + B1 * D0) * L1u * arm_cos_f32(phi1);
    float M22 = - A1 * D0 * L2u * arm_sin_f32(phi2) + B1 * D0 * L2u * arm_cos_f32(phi2);
    float s = arm_sin_f32(leg->theta);
    float c = arm_cos_f32(leg->theta);
    // Jf : 足端映射到关机电机的雅可比矩阵
    j->Jf_data[0] = - s * M11 - c * M21;
    j->Jf_data[1] = - s * M12 - c * M22;
    j->Jf_data[2] = (- c * M11 + s * M21) / leg->l0;
    j->Jf_data[3] = (- c * M12 + s * M22) / leg->l0;
    leg->d_l0 = j->Jf_data[0] * d_phi1 + j->Jf_data[1] * d_phi2;
    leg->d_theta = j->Jf_data[2] * d_phi1 + j->Jf_data[3] * d_phi2;

    memcpy(j->Jtemp_data, j->Jf_data, sizeof(j->Jf));
    arm_mat_inverse_f32(&j->Jtemp, &j->Ji);

    float dxT_dphi1 = 0;
    float dyT_dphi1 = 0;
    float dxB1_dphi1 = L1u * arm_sin_f32(phi1);
    float dyB1_dphi1 = - L1u * arm_cos_f32(phi1);
    float dxB2_dphi1 = 0;
    float dyB2_dphi1 = 0;
    float dxC_dphi1 = M11;
    float dyC_dphi1 = M21;
    float dxT_dphi2 = 0;
    float dyT_dphi2 = 0;
    float dxB1_dphi2 = 0;
    float dyB1_dphi2 = 0;
    float dxB2_dphi2 = - L2u * arm_sin_f32(phi2);
    float dyB2_dphi2 = - L2u * arm_cos_f32(phi2);
    float dxC_dphi2 = M12;
    float dyC_dphi2 = M22;
    float dx1u_dphi1 = 0;
    float dy1u_dphi1 = 0;
    float dx2u_dphi1 = 0;
    float dy2u_dphi1 = 0;
    float dx1d_dphi1 = 0;
    float dy1d_dphi1 = 0;
    float dx2d_dphi1 = 0;
    float dy2d_dphi1 = 0;
    float dx1u_dphi2 = 0;
    float dy1u_dphi2 = 0;
    float dx2u_dphi2 = 0;
    float dy2u_dphi2 = 0;
    float dx1d_dphi2 = 0;
    float dy1d_dphi2 = 0;
    float dx2d_dphi2 = 0;
    float dy2d_dphi2 = 0;
    rod_mass_center(dxB1_dphi1, dyB1_dphi1, 0, 0, lamda_1u, &dx1u_dphi1, &dy1u_dphi1);
    rod_mass_center(dxC_dphi1, dyC_dphi1, dxB1_dphi1, dyB1_dphi1, lamda_1d, &dx1d_dphi1, &dy1d_dphi1);
    rod_mass_center(dxB2_dphi1, dyB2_dphi1, dxT_dphi1, dyT_dphi1, lamda_2u, &dx2u_dphi1, &dy2u_dphi1);
    rod_mass_center(dxC_dphi1, dyC_dphi1, dxB2_dphi1, dyB2_dphi1, lamda_2d, &dx2d_dphi1, &dy2d_dphi1);
    rod_mass_center(dxB1_dphi2, dyB1_dphi2, 0, 0, lamda_1u, &dx1u_dphi2, &dy1u_dphi2);
    rod_mass_center(dxC_dphi2, dyC_dphi2, dxB1_dphi2, dyB1_dphi2, lamda_1d, &dx1d_dphi2, &dy1d_dphi2);
    rod_mass_center(dxB2_dphi2, dyB2_dphi2, dxT_dphi2, dyT_dphi2, lamda_2u, &dx2u_dphi2, &dy2u_dphi2);
    rod_mass_center(dxC_dphi2, dyC_dphi2, dxB2_dphi2, dyB2_dphi2, lamda_2d, &dx2d_dphi2, &dy2d_dphi2);
    leg->ml = m1u + m2u + m1d + m2d;
    float dxmc_dphi1 = (dx1u_dphi1 * m1u / leg->ml + dx1d_dphi1 * m1d / leg->ml + dx2u_dphi1 * m2u / leg->ml + dx2d_dphi1 * m2d / leg->ml);
    float dxmc_dphi2 = (dx1u_dphi2 * m1u / leg->ml + dx1d_dphi2 * m1d / leg->ml + dx2u_dphi2 * m2u / leg->ml + dx2d_dphi2 * m2d / leg->ml);
    float dymc_dphi1 = (dy1u_dphi1 * m1u / leg->ml + dy1d_dphi1 * m1d / leg->ml + dy2u_dphi1 * m2u / leg->ml + dy2d_dphi1 * m2d / leg->ml);
    float dymc_dphi2 = (dy1u_dphi2 * m1u / leg->ml + dy1d_dphi2 * m1d / leg->ml + dy2u_dphi2 * m2u / leg->ml + dy2d_dphi2 * m2d / leg->ml);
    // Jmc_phi : 腿质心坐标映射到关节角的雅可比矩阵
    j->Jmc_phi_data[0] = dxmc_dphi1;
    j->Jmc_phi_data[1] = dxmc_dphi2;
    j->Jmc_phi_data[2] = dymc_dphi1;
    j->Jmc_phi_data[3] = dymc_dphi2;
    float dphi1_dl = j->Ji_data[0];
    float dphi1_dtheta = j->Ji_data[1];
    float dphi2_dl = j->Ji_data[2];
    float dphi2_dtheta = j->Ji_data[3];
    float dxmc_dl = dxmc_dphi1 * dphi1_dl + dxmc_dphi2 * dphi2_dl;
    float dxmc_dtheta = dxmc_dphi1 * dphi1_dtheta + dxmc_dphi2 * dphi2_dtheta;
    float dymc_dl = dymc_dphi1 * dphi1_dl + dymc_dphi2 * dphi2_dl;
    float dymc_dtheta = dymc_dphi1 * dphi1_dtheta + dymc_dphi2 * dphi2_dtheta;
    // Jmc_ltheta : 腿质心坐标映射到腿长和腿倾角的雅可比矩阵
    j->Jmc_ltheta_data[0] = dxmc_dl;
    j->Jmc_ltheta_data[1] = dxmc_dtheta;
    j->Jmc_ltheta_data[2] = dymc_dl;
    j->Jmc_ltheta_data[3] = dymc_dtheta;
}

void get_K_LQR(float l0_l, float l0_r, ctrl_matrix_t *k)
{
    float t4 = l0_l * l0_r;
    float t5 = l0_l * l0_l;
    float t6 = l0_r * l0_r;

    for (uint8_t i = 0; i < 40; i++)
    {
        k->Ktemp_data[i] = l0_l * k->Kk2_data[i] + l0_r * k->Kk3_data[i]
                            + t4 * k->Kk4_data[i] + t5 * k->Kk5_data[i] + t6 * k->Kk6_data[i];
    }
    arm_mat_add_f32(&k->Kk1, &k->Ktemp, &k->Klqr);
}

void get_Tout(leg_state_t *leg, leg_torque_t *t, jacobin_matrix_t *j)
{
    // 计算关节电机输出力矩
    // Jf转置
    // 补偿腿部重力
    // t->T1 = j->Jf_data[0] * leg->F + j->Jf_data[2] * leg->Tp - j->Jmc_phi_data[2] * (- leg->ml * g);
    // t->T2 = j->Jf_data[1] * leg->F + j->Jf_data[3] * leg->Tp - j->Jmc_phi_data[3] * (- leg->ml * g);
    t->T1 = j->Jf_data[0] * leg->F + j->Jf_data[2] * leg->Tp;
    t->T2 = j->Jf_data[1] * leg->F + j->Jf_data[3] * leg->Tp;
}

// 通过关节电机力矩，计算足端输出力
// 足端支持力解算
void get_Fn(float T1, float T2, leg_state_t *leg, jacobin_matrix_t *j)
{
    // Ji转置
    // leg->Fi = - (j->Ji_data[0] * t->T1 + j->Ji_data[2] * t->T2) - j->Jmc_ltheta_data[2] * (- leg->ml * g);
    // leg->Tpi = - (j->Ji_data[1] * t->T1 + j->Ji_data[3] * t->T2) - j->Jmc_ltheta_data[3] * (- leg->ml * g);
    leg->Fi = - (j->Ji_data[0] * T1 + j->Ji_data[2] * T2);
    leg->Tpi = - (j->Ji_data[1] * T1 + j->Ji_data[3] * T2);
    // leg->Fn = mw * g + leg->Fi * arm_cos_f32(leg->theta) - leg->Tpi * arm_sin_f32(leg->theta) / leg->l0;
}

// void get_leg_state(leg_state_t *leg, ctrl_matrix_t *mat, float phi, float phi_1, float phi_4, float dt)
// {
//     leg->Xb = l1 * arm_cos_f32(phi_1);
//     leg->Yb = l1 * arm_sin_f32(phi_1);
//     leg->Xd = l4 * arm_cos_f32(phi_4) + l5;
//     leg->Yd = l4 * arm_sin_f32(phi_4);
//     float deltaX = leg->Xd - leg->Xb;
//     float deltaY = leg->Yd - leg->Yb;
//     arm_sqrt_f32(deltaX * deltaX + deltaY * deltaY, &leg->lbd);
//     float A0 = 2.0f * l2 * deltaX;
//     float B0 = 2.0f * l2 * deltaY;
//     float C0 = l2 * l2 + leg->lbd * leg->lbd - l3 * l3;
//     arm_sqrt_f32(A0 * A0 + B0 * B0 - C0 * C0, &leg->ABC);
//     leg->phi_2 = 2.0f * atan2f(B0 + leg->ABC, A0 + C0);
//     leg->phi_3 = atan2f(leg->Yb - leg->Yd + l2 * arm_sin_f32(leg->phi_2),
//                         leg->Xb - leg->Xd + l2 * arm_cos_f32(leg->phi_2));
//     leg->Xc = leg->Xb + l2 * arm_cos_f32(leg->phi_2);
//     leg->Yc = leg->Yb + l2 * arm_sin_f32(leg->phi_2);
//     float refX = leg->Xc - l5 / 2.0f;
//     arm_sqrt_f32(refX * refX + leg->Yc * leg->Yc, &leg->l0);
//     leg->d_l0 = (leg->l0 - leg->last_l0) / dt;                  // 腿长变化率
//     leg->phi_0 = atan2f(leg->Yc, refX);                         // 等效腿摆角phi_0
//     leg->d_phi_0 = (leg->phi_0 - leg->last_phi_0) / dt;         // 等效腿摆角速度d_phi_0
//     leg->theta = leg->phi_0 - phi- (PI / 2.0f);                 // 等效腿倾角theta
//     leg->d_theta = (leg->theta - leg->last_theta) / dt;         // 等效腿倾角速度
//     leg->last_l0 = leg->l0;
//     leg->last_phi_0 = leg->phi_0;
//     leg->last_theta = leg->theta;
//     // 计算足端力映射到关机电机力矩的雅可比矩阵
//     mat->J_data[0] = (l1 * arm_sin_f32(leg->phi_0 - leg->phi_3) * arm_sin_f32(phi_1 - leg->phi_2))
//                         / arm_sin_f32(leg->phi_3 - leg->phi_2);
//     mat->J_data[1] = (l1 * arm_cos_f32(leg->phi_0 - leg->phi_3) * arm_sin_f32(phi_1 - leg->phi_2))
//                         / (leg->l0 * arm_sin_f32(leg->phi_3 - leg->phi_2));
//     mat->J_data[2] = (l4 * arm_sin_f32(leg->phi_0 - leg->phi_2) * arm_sin_f32(leg->phi_3 - phi_4))
//                         / arm_sin_f32(leg->phi_3 - leg->phi_2);
//     mat->J_data[3] = (l4 * arm_cos_f32(leg->phi_0 - leg->phi_2) * arm_sin_f32(leg->phi_3 - phi_4))
//                         / (leg->l0 * arm_sin_f32(leg->phi_3 - leg->phi_2));
// }

// void get_K(leg_state_t *leg, ctrl_matrix_t *mat)
// {
//     float L0 = leg->l0;
//     float t2 = L0 * L0;
//     float t3 = t2 * L0;
//
//     mt1 = L0*(-1.170067736972087e+2)+t2*2.592482048141503e+2-t3*2.23817693466304e+2-2.683491882400822e-2;
//     mt2 = L0*3.729929192742041e+2-t2*1.39859414276839e+3+t3*1.62062362671733e+3-1.088995438165585e+1;
//     mt3 = L0*(-2.962033438112152e+1)+t2*7.532667894422579e+1-t3*7.179884195785564e+1+1.050333387341123;
//     mt4 = L0*1.047599197711019e+2-t2*4.412240115156856e+2+t3*5.424430392194918e+2-1.278625419145605;
//     mt5 = L0*(-5.661729298553493e-1)+t2*1.124366088818472-t3*7.77870134825133e-1-1.677608324830564e-2;
//     mt6 = L0*4.28235827588279-t2*1.727558501599943e+1+t3*2.098339175684849e+1-1.464890048467275e-1;
//     mt7 = L0*(-4.75741167510087)+t2*9.462203200213576-t3*6.526787097719793-1.709963323087147e-1;
//     mt8 = L0*3.69612298565867e+1-t2*1.501711993051356e+2+t3*1.831694770902427e+2-1.224787649802036;
//     mt9 = L0*(-2.459762915476045e+1)+t2*2.432013807053373e+1-t3*3.910225154834764+9.528945939051729;
//     mt10 = L0*7.294549772234784e+1-t2*8.556890905930656e+1-t3*3.379744445656995+9.306174563570195;
//     mt11 = L0*(-8.289946315106063)+t2*8.185803935458368e-1+t3*1.040661177612115e+1+3.972624898648111;
//     mt12 = L0*5.12423833610619e+1-t2*1.028571630081231e+2+t3*7.382319690919277e+1+3.510775630265001;
//
//     mat->K_data[0] = mt1;
//     mat->K_data[1] = mt3;
//     mat->K_data[2] = mt5;
//     mat->K_data[3] = mt7;
//     mat->K_data[4] = mt9;
//     mat->K_data[5] = mt11;
//     mat->K_data[6] = mt2;
//     mat->K_data[7] = mt4;
//     mat->K_data[8] = mt6;
//     mat->K_data[9] = mt8;
//     mat->K_data[10] = mt10;
//     mat->K_data[11] = mt12;
// }

// void get_torque(leg_state_t *leg, ctrl_matrix_t *mat, leg_torque_t *t, float phi, float d_phi, float xb, float d_xb)
// {
//     leg->F = (M * g / 2.0f);
//     mat->X_data[0] = leg->theta;
//     mat->X_data[1] = leg->d_theta;
//     mat->X_data[2] = 0;             // 机体位移
//     mat->X_data[3] = d_xb;          // 机体速度
//     mat->X_data[4] = phi;           // 机体俯仰角
//     mat->X_data[5] = d_phi;         // 机体俯仰角速度
//
//     arm_mat_sub_f32(&mat->Xd, &mat->X, &mat->Xtemp);
//     arm_mat_mult_f32(&mat->K, &mat->Xtemp, &mat->U); // 得到输出矩阵
//
//     t->Tw = mat->U_data[0];     // 轮电机输出力矩
//     leg->Tp = mat->U_data[1];   // 髋关节虚拟输出力矩
//
//     mat->F_data[0] = leg->F;
//     mat->F_data[1] = leg->Tp;
//
//     arm_mat_mult_f32(&mat->J, &mat->F, &mat->Ttemp);
//     t->T1 = mat->Ttemp_data[0];
//     t->T2 = mat->Ttemp_data[1];
// }

// void leg_length_ctrl(leg_state_t *leg_l, leg_state_t * leg_r, float Ll, float Lr, float roll) // 目标腿长
// {
//     PID_calculate(&left_leg_length_pid, Ll, leg_l->l0);
//     PID_calculate(&right_leg_length_pid, Lr, leg_r->l0);
//     PID_calculate(&roll_pid, 0.0f, roll);
//     leg_l->F = leg_l->F + left_leg_length_pid.out - roll_pid.out;
//     leg_r->F = leg_r->F + right_leg_length_pid.out + roll_pid.out;
// }
//
// void leg_theta_ctrl(leg_state_t *leg_l, leg_state_t * leg_r) // 两腿theta角度差控制
// {
//     float theta_delta = leg_l->theta - leg_r->theta;
//     PID_calculate(&leg_theta_pid, 0.0f, theta_delta);
//     leg_l->Tp = leg_l->Tp + leg_theta_pid.out;
//     leg_r->Tp = leg_r->Tp - leg_theta_pid.out;
// }
//
// void yaw_ctrl(leg_torque_t *leg_tl, leg_torque_t * leg_tr, float yaw) // 机体偏航角控制
// {
//     PID_calculate(&yaw_pid, 0.0f, yaw);
//     leg_tl->Tw = leg_tl->Tw - yaw_pid.out;
//     leg_tr->Tw = leg_tr->Tw + yaw_pid.out;
// }
