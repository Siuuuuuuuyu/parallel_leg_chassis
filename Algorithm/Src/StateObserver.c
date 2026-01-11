//
// Created by sy on 2025/12/20.
//
#include "StateObserver.h"

Vel_KF_t v_kf;

void velocity_kf_init(float process_noise, float measure_noise1, float measure_noise2)
{
    float v_kf_F[4] ={1.0f, 0.0f,
                      0.0f, 1.0f};
    float v_kf_P[4] ={1.0f, 0.0f,
                      0.0f, 1.0f};
    float v_kf_H[4] ={1.0f, 0.0f,
                      0.0f, 1.0f};
    Kalman_Filter_Init(&v_kf.Velocity_KF, 2, 0, 2);
    memcpy(v_kf.Velocity_KF.F_data, v_kf_F, sizeof(v_kf_F));
    memcpy(v_kf.Velocity_KF.P_data, v_kf_P, sizeof(v_kf_P));
    memcpy(v_kf.Velocity_KF.H_data, v_kf_H, sizeof(v_kf_H));
    v_kf.Q = process_noise;
    v_kf.R1 = measure_noise1;
    v_kf.R2 = measure_noise2;
}

void velocity_kf_update(double wl, double wr, float d_yaw, float ax_b, float dt)
{
    v_kf.Velocity_KF.F_data[1] = dt;
    double l_pre_v = wl * rw;
    double r_pre_v = wr * rw;
    float v_wheel = (l_pre_v + r_pre_v) / 2.0f;

    // // 科里奥利补偿
    // float coriolis_acc = - d_yaw * v_kf.v * 0.5f;  // 0.5为经验系数
    // ax_b += coriolis_acc;

    // // 打滑检测
    // // 比较轮速差计算的角速度与IMU角速度
    // float d_yaw_check = (r_pre_v - l_pre_v) / (rl * 2.0f);
    // if (fabsf(d_yaw_check - d_yaw) > 0.3f)
    //     v_wheel = 0.0f;

    v_kf.Velocity_KF.MeasuredVector[0] = v_wheel;
    v_kf.Velocity_KF.MeasuredVector[1] = ax_b;

    v_kf.Velocity_KF.Q_data[0] = v_kf.Q;
    v_kf.Velocity_KF.Q_data[2] = v_kf.Q;

    v_kf.Velocity_KF.R_data[0] = v_kf.R1;
    v_kf.Velocity_KF.R_data[2] = v_kf.R2;

    Kalman_Filter_Update(&v_kf.Velocity_KF);

    v_kf.v = v_kf.Velocity_KF.FilteredValue[0];
    v_kf.a = v_kf.Velocity_KF.FilteredValue[1];
}
