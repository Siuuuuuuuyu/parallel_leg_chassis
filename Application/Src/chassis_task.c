//
// Created by sy on 2025/12/22.
//
#include "chassis_task.h"

// leg_state_t leg_l;
// leg_state_t leg_r;
// leg_torque_t leg_tl;
// leg_torque_t leg_tr;
// jacobin_matrix_t leg_jl;
// jacobin_matrix_t leg_jr;
// ctrl_matrix_t k;

// uint32_t leg_DWT_Count = 0;
// static float dt = 0;
// uint32_t chassis_time = 1;

void chassis_init()
{
    // leg_controller_init(&leg_jl, &leg_jr, &k);
    // M3508_init();
    // DM8009_init(&DM8009_1, &hfdcan1, TO4_MODE, 0x3FE, 0x301, -2.0550181220771719, 0.01, 1);
    // DM8009_init(&DM8009_2, &hfdcan1, TO4_MODE, 0x3FE, 0x302, -2.1716148949609826, 0.01, 2);
    // DM8009_init(&DM8009_3, &hfdcan1, TO4_MODE, 0x3FE, 0x303, -1.0670138886932987, 0.01, 3);
    // DM8009_init(&DM8009_4, &hfdcan1, TO4_MODE, 0x3FE, 0x304, -2.0649902144948662, 0.01, 4);
}

void chassis_task(void const * argument)
{
    while (INS.ready_flag == 0)
    {
    }
    chassis_init();
    while (1)
    {
        // dt = DWT_GetDeltaT(&leg_DWT_Count);
        //
        // leg_kinematic(&leg_l, &leg_jl, 0.0f, 0.0f, 0.0f, 0.0f);
        // leg_kinematic(&leg_r, &leg_jr, 0.0f, 0.0f, 0.0f, 0.0f);
        // get_K_LQR(&leg_l, &leg_r, &k);
        // get_Tout(&leg_l, &leg_r, &leg_tl, &leg_tr, &leg_jl, &leg_jr, &k,
        //         INS.Yaw, INS.Gyro[2], INS.Roll, INS.Gyro[1]);
        // DM8009_cmd_upgrade(&DM8009_1, 0);
        // DM8009_cmd_upgrade(&DM8009_2, - 0);
        // DM8009_cmd_upgrade(&DM8009_3, - 0);
        // DM8009_cmd_upgrade(&DM8009_4, 0);
        // DM8009_send_1to4(0x3FE, &hfdcan1, DM8009_1.m.cmd.cmd_signal, DM8009_2.m.cmd.cmd_signal,
        //               DM8009_3.m.cmd.cmd_signal, DM8009_4.m.cmd.cmd_signal);
        // float l_phi1 = 5.0f * PI/4.0f + DM8009_3.m.para.pos_fb;
        // float l_phi4 = DM8009_4.m.para.pos_fb - PI/4.0f;
        // float r_phi1 = 5.0f * PI/4.0f - DM8009_1.m.para.pos_fb;
        // float r_phi4 = - DM8009_2.m.para.pos_fb - PI / 4.0f;
        // get_leg_state(&leg_l, &leg_ml, INS.Roll, l_phi1, l_phi4, 0.001f);
        // get_leg_state(&leg_r, &leg_mr, INS.Roll, r_phi1, r_phi4, 0.001f);
        // leg_length_ctrl(&leg_l, &leg_r, 0.18f, 0.18f, INS.Pitch);
        // get_K(&leg_l, &leg_ml);
        // get_K(&leg_r, &leg_mr);
        // get_torque(&leg_l, &leg_ml, &leg_tl, INS.Roll, INS.Gyro[1], 0, obs_d_xb);
        // get_torque(&leg_r, &leg_mr, &leg_tr, INS.Roll, INS.Gyro[1], 0, obs_d_xb);
        // leg_length_ctrl(&leg_l, &leg_r, 0.18f, 0.18f);
        // leg_theta_ctrl(&leg_l, &leg_r);
        // yaw_ctrl(&leg_tl, &leg_tr, INS.Yaw);

        // DM8009_cmd_upgrade(&DM8009_1, - leg_tr.T1);
        // DM8009_cmd_upgrade(&DM8009_2, - leg_tr.T2);
        // DM8009_cmd_upgrade(&DM8009_3, leg_tl.T1);
        // DM8009_cmd_upgrade(&DM8009_4, leg_tl.T2);
        // DM8009_cmd_upgrade(&DM8009_1, 0);
        // DM8009_cmd_upgrade(&DM8009_2, - 0);
        // DM8009_cmd_upgrade(&DM8009_3, - 0);
        // DM8009_cmd_upgrade(&DM8009_4, 0);

        // M3508_cmd_upgrade(&M3508_1, leg_tr.Tw * 17.0f / 268.0f);
        // M3508_cmd_upgrade(&M3508_2, - leg_tl.Tw * 17.0f / 268.0f);
        // DM8009_send_1to4(0x3FE, &hfdcan1, DM8009_1.m.cmd.cmd_signal, DM8009_2.m.cmd.cmd_signal,
        //               DM8009_3.m.cmd.cmd_signal, DM8009_4.m.cmd.cmd_signal);
        // dji_motor_ctrl_send(((leg_tr.Tw * 17.0f / 268.0f) / (0.3f * 187.0f / 3591.0f) * 819.2f,
        //                     ((- leg_tl.Tw * 17.0f / 268.0f) / (0.3f * 187.0f / 3591.0f)) * 819.2f, 0, 0);
        // dji_motor_ctrl_send(0x200, M3508_1.tor_cmd, M3508_2.tor_cmd, 0, 0);
        // dji_motor_ctrl_send(0x200, 300, 0, 0, 0);
    }
}
