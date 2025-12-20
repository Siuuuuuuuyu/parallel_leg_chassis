//
// Created by sy on 2025/12/19.
//
#include "LegController.h"

const float l1, l4 = 0.15f;
const float l2, l3 = 0.27f;
const float l5 = 0.15f;
const float r = 0.06f;

void leg_matrix_init(ctrl_matrix_t *mat)
{
    // 初始化K矩阵
    mat->K_data = user_malloc(sizeof(float) * 12);
    memset(mat->K_data, 0, sizeof(float) * 12);
    arm_mat_init_f32(&mat->K, 2, 6, mat->K_data);
    // 初始化状态向量
    mat->X_data = user_malloc(sizeof(float) * 6);
    memset(mat->X_data, 0, sizeof(float) * 6);
    arm_mat_init_f32(&mat->X, 6, 1, mat->X_data);
    // 初始化期望状态
    mat->Xd_data = user_malloc(sizeof(float) * 6);
    memset(mat->Xd_data, 0, sizeof(float) * 6);
    arm_mat_init_f32(&mat->Xd, 6, 1, mat->Xd_data);
    // 初始化控制矩阵
    mat->U_data = user_malloc(sizeof(float) * 2);
    memset(mat->U_data, 0, sizeof(float) * 2);
    arm_mat_init_f32(&mat->U, 2, 1, mat->U_data);
    //初始化雅可比矩阵
    mat->J_data = user_malloc(sizeof(float) * 4);
    memset(mat->J_data, 0, sizeof(float) * 4);
    arm_mat_init_f32(&mat->J, 2, 2, mat->J_data);
    //初始化足端力与力矩矩阵
    mat->F_data = user_malloc(sizeof(float) * 2);
    memset(mat->F_data, 0, sizeof(float) * 2);
    arm_mat_init_f32(&mat->F, 2, 1, mat->F_data);
    // 初始化状态临时矩阵
    mat->Xtemp_data = user_malloc(sizeof(float) * 6);
    memset(mat->Xtemp_data, 0, sizeof(float) * 6);
    arm_mat_init_f32(&mat->J, 6, 1, mat->Xtemp_data);
    // 初始化力矩临时矩阵
    mat->Ttemp_data = user_malloc(sizeof(float) * 2);
    memset(mat->Ttemp_data, 0, sizeof(float) * 2);
    arm_mat_init_f32(&mat->J, 2, 1, mat->Ttemp_data);
}

void get_leg_state(leg_position_t *leg, ctrl_matrix_t *mat, float phi_1, float phi_4, float dt)
{
    leg->Xb = l1 * arm_cos_f32(phi_1);
    leg->Yb = l1 * arm_sin_f32(phi_1);
    leg->Xd = l4 * arm_cos_f32(phi_4) + l5;
    leg->Yd = l4 * arm_sin_f32(phi_4);
    float deltaX = leg->Xd - leg->Xb;
    float deltaY = leg->Yd - leg->Yb;
    arm_sqrt_f32(deltaX * deltaX + deltaY * deltaY, &leg->lbd);
    float A0 = 2.0f * l2 * deltaX;
    float B0 = 2.0f * l2 * deltaY;
    float C0 = l2 * l2 + leg->lbd * leg->lbd - l3 * l3;
    arm_sqrt_f32(A0 * A0 + B0 * B0 - C0 * C0, &leg->ABC);
    leg->phi_2 = 2.0f * atan2f(B0 + leg->ABC, A0 + C0);
    leg->phi_3 = atan2f(leg->Yb - leg->Yd + l2 * arm_sin_f32(leg->phi_2),
                        leg->Xb - leg->Xd + l2 * arm_cos_f32(leg->phi_2));
    leg->Xc = leg->Xb + l2 * arm_cos_f32(leg->phi_2);
    leg->Yc = leg->Yb + l2 * arm_sin_f32(leg->phi_2);
    float refX = leg->Xc - l5/2.0f;
    arm_sqrt_f32(refX * refX + leg->Yc * leg->Yc, &leg->l0);
    leg->phi_0 = atan2f(leg->Yc, refX);
    leg->d_phi_0 = (leg->phi_0 - leg->last_phi_0) / dt;
    leg->last_phi_0 = leg->phi_0;
    // 计算足端力映射到关机电机力矩的雅可比矩阵
    mat->J_data[0] = (l1 * arm_sin_f32(leg->phi_0 - leg->phi_3) * arm_sin_f32(phi_1 - leg->phi_2))
                        / arm_sin_f32(leg->phi_3 - leg->phi_2);
    mat->J_data[1] = (l1 * arm_cos_f32(leg->phi_0 - leg->phi_3) * arm_sin_f32(phi_1 - leg->phi_2))
                        / (leg->l0 * arm_sin_f32(leg->phi_3 - leg->phi_2));
    mat->J_data[2] = (l4 * arm_sin_f32(leg->phi_0 - leg->phi_2) * arm_sin_f32(leg->phi_3 - phi_4))
                        / arm_sin_f32(leg->phi_3 - leg->phi_2);
    mat->J_data[3] = (l4 * arm_cos_f32(leg->phi_0 - leg->phi_2) * arm_sin_f32(leg->phi_3 - phi_4))
                        / (leg->l0 * arm_sin_f32(leg->phi_3 - leg->phi_2));
}

void get_K(ctrl_matrix_t *mat, float l0)
{
    // 使用Horner方法优化的LQR增益计算，可以减少乘法次数，提高数值稳定性
    // 使用Horner方法：a3*x^3 + a2*x^2 + a1*x + a0 = ((a3*x + a2)*x + a1)*x + a0
    mat->K_data[0] = ((-2.305633199617884e+2 * l0 + 2.769144411225284e+2) * l0 - 1.738768715878373e+2) * l0 - 5.037378842181335e-1;
    mat->K_data[1] = ((8.48445136223112 * l0 - 8.474830898543729) * l0 - 1.76125011212762e+1) * l0 + 1.422505502414415e-1;
    mat->K_data[2] = ((-5.20171179606681e+1 * l0 + 5.037437913406749e+1) * l0 - 1.709664996028396e+1) * l0 - 6.809917957113252e-1;
    mat->K_data[3] = ((-1.713139726385663e+2 * l0 + 1.669513557246775e+2) * l0 - 5.84775902096206e+1) * l0 - 2.451140940168691;
    mat->K_data[4] = ((-5.286166938684254e+1 * l0 + 9.062838912531303e+1) * l0 - 5.77040877687421e+1) * l0 + 1.813499895556407e+1;
    mat->K_data[5] = ((-3.498392667782311 * l0 + 8.547157926306484) * l0 - 6.526136149574446) * l0 + 2.683940865200834;
    mat->K_data[6] = ((4.047141931925083e+2 * l0 - 3.634904951505106e+2) * l0 + 1.001790416397507e+2) * l0 + 4.493375342020929;
    mat->K_data[7] = ((3.79858158793623e+1 * l0 - 3.805344415806194e+1) * l0 + 1.372243172896451e+1) * l0 + 4.709197893760396e-1;
    mat->K_data[8] = ((1.79496147364881e+1 * l0 - 5.648898310601982) * l0 - 5.303327435622222) * l0 + 2.935793682740416;
    mat->K_data[9] = ((6.346789673699568e+1 * l0 - 2.24050536167074e+1) * l0 - 1.656119409712176e+1) * l0 + 9.882852127151995;
    mat->K_data[10] = ((2.079207625200414e+2 * l0 - 2.254568074459751e+2) * l0 + 8.854962908408282e+1) * l0 + 1.230681109592864e+1;
    mat->K_data[11] = ((2.295378889875587e+1 * l0 - 2.744938756723369e+1) * l0 + 1.209880149868675e+1) * l0 + 7.716935260759946e-1;
}

void get_torque(leg_position_t *leg, ctrl_matrix_t *mat, leg_torque_t *t, float phi, float d_phi, float d_x, float F)
{
    mat->X_data[0] = leg->phi_0 - phi- (PI / 2.0f); // 计算theta0
    mat->X_data[1] = leg->d_phi_0 - d_phi; // 计算d_theta0
    mat->X_data[2] = d_x * r; // 计算不打滑状态下的轮位移
    mat->X_data[3] = d_x; // 轮速
    mat->X_data[4] = phi; // 机体俯仰角
    mat->X_data[5] = d_phi; // 机体俯仰角速度

    arm_mat_sub_f32(&mat->Xd, &mat->X, &mat->Xtemp);
    arm_mat_mult_f32(&mat->K, &mat->Xtemp, &mat->U); // 得到输出力矩

    t->Tw = mat->U_data[0]; // 得到轮电机输出力矩

    mat->F_data[0] = F;
    mat->F_data[1] = mat->U_data[1]; // Tp

    arm_mat_mult_f32(&mat->J, &mat->F, &mat->Ttemp);
    t->T1 = mat->Ttemp_data[0];
    t->T2 = mat->Ttemp_data[1];
}
