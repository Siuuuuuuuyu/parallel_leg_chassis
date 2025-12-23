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
    float R;
    KalmanFilter_t Velocity_KF;
} Vel_KF_t;

extern float d_xb;
extern float dd_xb;

void velocity_kf_init(float process_noise, float measure_noise);
void velocity_kf_update(leg_state_t *leg_l, leg_state_t *leg_r, float wl, float wr, float ax, float dt);

#endif //PARALLEL_LEG_CHASSIS_STATEOBSERVER_H