//
// Created by sy on 2025/12/24.
//
#include "observer_task.h"

float d_xb = 0;
float dd_xb = 0;

uint32_t obs_DWT_Count = 0;
static float dt = 0;
uint32_t observe_time = 1;

void observer_task_init()
{
    velocity_kf_init(1.0f, 100.0f);
    osDelay(10000);
}

void observer_task(void const * argument)
{
    observer_task_init();
    while (1)
    {
        dt = DWT_GetDeltaT(&obs_DWT_Count);
        float wl = - M3508_2.rotor_speed * 17.0f / 268.0f;
        float wr = M3508_1.rotor_speed * 17.0f / 268.0f;
        velocity_kf_update(&leg_l, &leg_r, wl, wr, INS.Gyro[1], - INS.MotionAccel_b[0], dt);
        d_xb = v_kf.v;
        dd_xb = v_kf.a;
        osDelay(observe_time);
    }
}