//
// Created by sy on 2025/12/19.
//
#include "LegController.h"

const float l1 = 0.15f;
const float l4 = 0.15f;
const float l2 = 0.27f;
const float l3 = 0.27f;
const float l5 = 0.15f;

const float left_leg_length_pid_param[3] = {LEFT_LEG_LENGTH_PID_KP, LEFT_LEG_LENGTH_PID_KI, LEFT_LEG_LENGTH_PID_KD};
const float right_leg_length_pid_param[3] = {RIGHT_LEG_LENGTH_PID_KP, RIGHT_LEG_LENGTH_PID_KI, RIGHT_LEG_LENGTH_PID_KD};
const float leg_theta_pid_param[3] = {LEG_THETA_PID_KP, LEG_THETA_PID_KI, LEG_THETA_PID_KD};
const float yaw_pid_param[3] = {YAW_PID_KP, YAW_PID_KI, YAW_PID_KD};
const float roll_pid_param[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
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
const float g = 9.81f;
const float Fi_k = 10.0f;
const float Kk1[40] = {
    -2.36108512016555, -3.49109505395687, -2.33204105543659, -0.868410037549278, -5.09450233994573, -1.00031974646749, -2.38172769821469, -0.120062707891680, -16.4789178443566, -5.42428431911823,
    -2.36108512016543, -3.49109505395678, 2.33204105543660, 0.868410037549276, -2.38172769821459, -0.120062707891668, -5.09450233994563, -1.00031974646748, -16.4789178443565, -5.42428431911823,
    7.25896766443245, 9.71154664435203, -3.75724341462558, -1.39925352559386, 16.2569920556041, 3.81071072906151, 5.11130438747510, 0.214220456612017, -22.5414136856976, -6.87296115758663,
    7.25896766443221, 9.71154664435183, 3.75724341462556, 1.39925352559386, 5.11130438747476, 0.214220456611972, 16.2569920556040, 3.81071072906148, -22.5414136856969, -6.87296115758666
};
const float Kk2[40] = {
    -12.8620904807628, -15.1479925997255, 4.33702154294900, 1.87157510841740, -46.1881214270546, -5.23276541530355, 1.87003987442783, -0.0734184403236244, 35.6639237895127, 11.0833227522175,
    6.75393837505302, 9.71720388435436, 2.31524127426716, 1.07214064643823, -24.0448163552573, -3.11876043113466, 16.7527161759505, 1.08347516843796, 18.6408245669878, 6.44588188722424,
    3.43808877537642, 3.05576718888811, -4.79659013384676, -2.45900497801200, 34.4966732962621, -4.49338082773609, -20.9791971899057, -0.907439349151596, -124.436491830357, -39.8079522897407,
    -26.9474611852559, -36.7170712145101, 2.53917675398817, 1.04953077300217, -35.8795278637468, -1.92612187249629, -21.1516239836230, -1.64157349969135, 19.7363723016852, 5.06509509985457
};
const float Kk3[40] = {
    6.75393837505185, 9.71720388435394, -2.31524127426711, -1.07214064643821, 16.7527161759508, 1.08347516843790, -24.0448163552570, -3.11876043113469, 18.6408245669887, 6.44588188722425,
    -12.8620904807649, -15.1479925997267, -4.33702154294920, -1.87157510841741, 1.87003987442716, -0.0734184403237550, -46.1881214270551, -5.23276541530365, 35.6639237895125, 11.0833227522174,
    -26.9474611852571, -36.7170712145119, -2.53917675398769, -1.04953077300217, -21.1516239836254, -1.64157349969169, -35.8795278637500, -1.92612187249665, 19.7363723016863, 5.06509509985429,
    3.43808877537811, 3.05576718888882, 4.79659013384664, 2.45900497801201, -20.9791971899052, -0.907439349151460, 34.4966732962617, -4.49338082773600, -124.436491830362, -39.8079522897407
};
const float Kk4[40] = {
    -1.66813078719172, -4.93560509028538, 0.743380640190205, 0.373494471024606, 9.78741782359607, 2.66744848454949, 1.43405928089401, -2.67430919015921, -24.0017276351096, -8.07145911627817,
    -1.66813078719108, -4.93560509028509, -0.743380640189151, -0.373494471024556, 1.43405928089354, -2.67430919015905, 9.78741782359723, 2.66744848454946, -24.0017276351106, -8.07145911627826,
    28.9616384811428, 40.7229947833212, -4.58510948015227, -2.77477192025103, 64.3634719247345, 2.02958600208771, -28.4949621206826, -0.901610342195293, 34.1527449141422, 12.4374786008996,
    28.9616384811429, 40.7229947833196, 4.58510948015132, 2.77477192025090, -28.4949621206837, -0.901610342195596, 64.3634719247310, 2.02958600208727, 34.1527449141480, 12.4374786008996
};
const float Kk5[40] = {
    15.6057167931986, 18.8750755683580, -3.08160599934066, -1.08703607817465, 24.7811786793542, -0.889353486903293, 4.80414749945542, 0.760425235238369, -26.8772230039135, -8.00090166521646,
    -5.16662149271602, -7.67779420466938, -2.54131301193749, -1.12488601567684, 29.7089365686540, 2.83450013036788, -21.5952426330882, -1.74050060985341, -11.8520118572403, -4.21629161090401,
    -25.8715731807749, -31.4152087048450, 6.71672749123641, 3.25494211728612, -56.5568038116152, 7.03941234997549, 16.5494817743786, -0.278730179706381, 147.356759775939, 46.1116416840996,
    23.3459505067062, 31.4462136517493, -2.55870492772011, -0.860183831091590, 21.1028982569249, -0.129123077895452, 16.6601669221378, 1.12963659838506, -26.3887444096992, -7.20677260788792
};
const float Kk6[40] = {
    -5.16662149271221, -7.67779420466784, 2.54131301193700, 1.12488601567680, -21.5952426330879, -1.74050060985324, 29.7089365686535, 2.83450013036808, -11.8520118572427, -4.21629161090403,
    15.6057167932040, 18.8750755683610, 3.08160599934069, 1.08703607817465, 4.80414749945761, 0.760425235238648, 24.7811786793552, -0.889353486902939, -26.8772230039121, -8.00090166521632,
    23.3459505067088, 31.4462136517532, 2.55870492771919, 0.860183831091634, 16.6601669221418, 1.12963659838569, 21.1028982569320, -0.129123077894721, -26.3887444096991, -7.20677260788723,
    -25.8715731807791, -31.4152087048461, -6.71672749123573, -3.25494211728608, 16.5494817743779, -0.278730179706545, -56.5568038116128, 7.03941234997546, 147.356759775947, 46.1116416840994
};

const float Kl[40] = {
    -3.82734060910715, -4.35492420257722, -2.31036864436683, -0.837820216708498, -9.90962104768260, -2.23912414303323, -5.17825871201365, -0.505773011375995, -12.4944850073777, -4.05981551938490,
    -3.82734060910716, -4.35492420257721, 2.31036864436683, 0.837820216708497, -5.17825871201366, -0.505773011376001, -9.90962104768259, -2.23912414303322, -12.4944850073776, -4.05981551938490,
    5.17274775234515, 5.62436836076125, -4.82814494766191, -1.86395457698398, 18.6469412026397, 4.14166034279364, -1.24136348540476, -0.270745381251778, -33.2688050739286, -10.4382493079419,
    5.17274775234525, 5.62436836076134, 4.82814494766191, 1.86395457698399, -1.24136348540469, -0.270745381251762, 18.6469412026397, 4.14166034279364, -33.2688050739288, -10.4382493079419
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
    arm_mat_init_f32(&k->Ktemp, 4, 10, k->Ktemp_data);
    memset(k->Ttemp_data, 0, sizeof(float) * 4);

    k->Kl_data = user_malloc(sizeof(float) * 40);
    arm_mat_init_f32(&k->Kl, 4, 10, k->Kl_data);
    memcpy(k->Kl_data, Kl, sizeof(float) * 40);

    PID_init(&left_leg_length_pid, PID_POSITION, left_leg_length_pid_param, LEFT_LEG_LENGTH_PID_MAX_OUT, LEFT_LEG_LENGTH_PID_MAX_IOUT);
    PID_init(&right_leg_length_pid, PID_POSITION, right_leg_length_pid_param, RIGHT_LEG_LENGTH_PID_MAX_OUT, ROGHT_LEG_LENGTH_PID_MAX_IOUT);
    PID_init(&leg_theta_pid, PID_POSITION, leg_theta_pid_param, LEG_THETA_PID_MAX_OUT, LEG_THETA_PID_MAX_OUT);
    PID_init(&roll_pid, PID_POSITION, roll_pid_param, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
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

void get_Fu(leg_state_t *leg, leg_torque_t *t, jacobin_matrix_t *j)
{
    // 静止状态求末端所受外力
    // Ji转置
    leg->Fi = - (j->Ji_data[0] * t->T1 + j->Ji_data[2] * t->T2) - j->Jmc_ltheta_data[2] * (- leg->ml * g);
    leg->Tpi = - (j->Ji_data[1] * t->T1 + j->Ji_data[3] * t->T2) - j->Jmc_ltheta_data[3] * (- leg->ml * g);
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

void leg_length_ctrl(leg_state_t *leg_l, leg_state_t * leg_r, float Ll, float Lr, float roll) // 目标腿长
{
    PID_calculate(&left_leg_length_pid, Ll, leg_l->l0);
    PID_calculate(&right_leg_length_pid, Lr, leg_r->l0);
    PID_calculate(&roll_pid, 0.0f, roll);
    leg_l->F = leg_l->F + left_leg_length_pid.out - roll_pid.out;
    leg_r->F = leg_r->F + right_leg_length_pid.out + roll_pid.out;
}

void leg_theta_ctrl(leg_state_t *leg_l, leg_state_t * leg_r) // 两腿theta角度差控制
{
    float theta_delta = leg_l->theta - leg_r->theta;
    PID_calculate(&leg_theta_pid, 0.0f, theta_delta);
    leg_l->Tp = leg_l->Tp + leg_theta_pid.out;
    leg_r->Tp = leg_r->Tp - leg_theta_pid.out;
}

void yaw_ctrl(leg_torque_t *leg_tl, leg_torque_t * leg_tr, float yaw) // 机体偏航角控制
{
    PID_calculate(&yaw_pid, 0.0f, yaw);
    leg_tl->Tw = leg_tl->Tw - yaw_pid.out;
    leg_tr->Tw = leg_tr->Tw + yaw_pid.out;
}
