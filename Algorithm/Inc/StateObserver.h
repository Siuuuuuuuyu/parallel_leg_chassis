//
// Created by sy on 2025/12/20.
//

#ifndef PARALLEL_LEG_CHASSIS_STATEOBSERVER_H
#define PARALLEL_LEG_CHASSIS_STATEOBSERVER_H

#include "KalmanFilter.h"
#include "LegController.h"

typedef struct
{
    float Q;
    float R1;
    float R2;
    float v;
    float a;
    KalmanFilter_t Velocity_KF;
} Vel_KF_t;

extern Vel_KF_t v_kf;

void velocity_kf_init(float process_noise, float measure_noise1, float measure_noise2);
void velocity_kf_update(double wl, double wr, float d_yaw, float ax_b, float dt);

#endif //PARALLEL_LEG_CHASSIS_STATEOBSERVER_H