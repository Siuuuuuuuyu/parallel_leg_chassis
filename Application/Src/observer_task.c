//
// Created by sy on 2025/12/24.
//
#include "observer_task.h"
float obs_xb = 0;
float obs_d_xb = 0;
float obs_dd_xb = 0;

uint32_t obs_DWT_Count = 0;
// static float dt = 0;
uint32_t observe_time = 1;

void observer_task_init()
{
    velocity_kf_init(1.0f, 1000.0f, 1000.0f);
}

void observer_task(void const * argument)
{
    while (INS.ready_flag == 0)
    {
    }
    observer_task_init();
    while (1)
    {
        // dt = DWT_GetDeltaT(&obs_DWT_Count);
        // float wl = - M3508_2.rotor_speed * 17.0f / 268.0f;
        // float wr = M3508_1.rotor_speed * 17.0f / 268.0f;
        // velocity_kf_update(&leg_l, &leg_r, wl, wr, INS.Gyro[1], - INS.MotionAccel_b[0], 0.001f);
        // obs_d_xb = v_kf.v;
        // obs_dd_xb = v_kf.a;
        // obs_xb = obs_xb + obs_d_xb * dt;
        // for (uint8_t i = 0; i < 4; i++)
        // {
        //     UART1_Tx_Data[i + 1] = *((uint8_t *)&obs_xb + i);
        //     UART1_Tx_Data[i + 5] = *((uint8_t *)&obs_d_xb + i);
        //     UART1_Tx_Data[i + 9] = *((uint8_t *)&obs_dd_xb + i);
        //     UART1_Tx_Data[i + 13] = *((uint8_t *)&INS.MotionAccel_b[0] + i);
        // }
        // bsp_uart_send(&huart1, UART1_Tx_Data, 17);
    }
}