//
// Created by sy on 2025/12/20.
//
#include "StateObserver.h"

Vel_KF_t v_kf;
float d_xb;
float dd_xb;

void velocity_kf_init(float process_noise, float measure_noise)
{
    float v_kf_F[4] ={1, 0,
                      0, 1};
    float v_kf_P[4] ={1, 0,
                      0, 1};
    float v_kf_H[4] ={1, 0,
                      0, 1};
    Kalman_Filter_Init(&v_kf.Velocity_KF, 2, 0, 2);
    memcpy(v_kf.Velocity_KF.F_data, v_kf_F, sizeof(v_kf_F));
    memcpy(v_kf.Velocity_KF.P_data, v_kf_P, sizeof(v_kf_P));
    memcpy(v_kf.Velocity_KF.H_data, v_kf_H, sizeof(v_kf_H));
    v_kf.Q = process_noise;
    v_kf.R = measure_noise;
}

void velocity_kf_update(leg_state_t *leg_l, leg_state_t *leg_r, float wl, float wr, float ax, float dt)
{
    v_kf.Velocity_KF.F_data[1] = dt;
    float a_leg_length = (leg_l->l0 + leg_r->l0) / 2.0f;
    float a_w = (wl + wr) / 2.0f;
    float a_theta = (leg_l->theta + leg_r->theta) / 2.0f;
    float a_d_leg_length = (leg_l->d_theta + leg_r->d_theta) / 2.0f;
    float a_d_theta = (leg_l->d_theta + leg_r->d_theta) / 2.0f;
    v_kf.Velocity_KF.MeasuredVector[0] = a_w * 0.06f + a_leg_length * a_d_theta * arm_cos_f32(a_theta) + a_d_leg_length * arm_sin_f32(a_theta);
    v_kf.Velocity_KF.MeasuredVector[1] = ax;

    v_kf.Velocity_KF.Q_data[0] = v_kf.Q * dt;
    v_kf.Velocity_KF.Q_data[2] = v_kf.Q * dt;
    v_kf.Velocity_KF.R_data[0] = v_kf.R;
    v_kf.Velocity_KF.R_data[2] = v_kf.R;

    Kalman_Filter_Update(&v_kf.Velocity_KF);

    d_xb = v_kf.Velocity_KF.FilteredValue[0];
    dd_xb = v_kf.Velocity_KF.FilteredValue[1];
}