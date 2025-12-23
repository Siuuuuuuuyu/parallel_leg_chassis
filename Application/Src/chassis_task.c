//
// Created by sy on 2025/12/22.
//
#include "chassis_task.h"

#include "cmsis_os.h"

leg_state_t leg_l;
leg_state_t leg_r;
leg_torque_t leg_tl;
leg_torque_t leg_tr;
ctrl_matrix_t leg_ml;
ctrl_matrix_t leg_mr;
uint32_t leg_DWT_Count = 0;
static float dt = 0;

void chassis_init()
{
    leg_controller_init(&leg_ml);
    leg_controller_init(&leg_mr);

    DM8009_init(&DM8009_1, &hfdcan1, TO4_MODE, 0x3FE, 0x301, -2.03814227337, 0.01, 1);
    DM8009_init(&DM8009_2, &hfdcan1, TO4_MODE, 0x3FE, 0x302, -2.144766953836, 0.01, 2);
    DM8009_init(&DM8009_3, &hfdcan1, TO4_MODE, 0x3FE, 0x303, -1.104601006268, 0.01, 3);
    DM8009_init(&DM8009_4, &hfdcan1, TO4_MODE, 0x3FE, 0x304, -2.05962062627, 0.01, 4);
}

void chassis_task(void const * argument)
{
    chassis_init();
    osDelay(10000);
    while (1)
    {
        dt = DWT_GetDeltaT(&leg_DWT_Count);
        get_leg_state(&leg_l, &leg_ml, DM8009_3.m.para.pos_fb + 4.0f * PI / 9.0f,
                DM8009_4.m.para.pos_fb + 113.7f * PI / 180.0f, dt);
        get_leg_state(&leg_r, &leg_mr, 3.0f * PI / 2.0f - DM8009_1.m.para.pos_fb,
                - (DM8009_2.m.para.pos_fb - 5.0f * PI / 6.0f), dt);
        get_K(&leg_l, &leg_ml);
        get_K(&leg_r, &leg_mr);
        get_torque(&leg_l, &leg_ml, &leg_tl, -INS.Roll, -INS.Gyro[1], M3508_2.rotor_speed * 17.0f * 0.06f / 256.0f);
        get_torque(&leg_r, &leg_mr, &leg_tr, -INS.Roll, -INS.Gyro[1], - M3508_1.rotor_speed * 17.0f * 0.06f / 256.0f);
        leg_length_ctrl(&leg_l, &leg_r, 0.2f, 0.2f);
        // leg_theta_ctrl(&leg_l, &leg_r);
        // roll_ctrl(&leg_l, &leg_r, 0.0f);

        DM8009_cmd_upgrade(&DM8009_1, - leg_tr.T1);
        DM8009_cmd_upgrade(&DM8009_2, leg_tr.T2);
        DM8009_cmd_upgrade(&DM8009_3, leg_tl.T1);
        DM8009_cmd_upgrade(&DM8009_4, - leg_tl.T2);
        // DM8009_cmd_upgrade(&DM8009_1, 0);
        // DM8009_cmd_upgrade(&DM8009_2, - 0);
        // DM8009_cmd_upgrade(&DM8009_3, - 0);
        // DM8009_cmd_upgrade(&DM8009_4, 0);
        DM8009_send_1to4(0x3FE, &hfdcan1, DM8009_1.m.cmd.cmd_signal, DM8009_2.m.cmd.cmd_signal,
                      DM8009_3.m.cmd.cmd_signal, DM8009_4.m.cmd.cmd_signal);
        // dji_motor_ctrl_send(0, 0, 0, 0);
        dji_motor_ctrl_send(((- leg_tr.Tw * 17.0f / 268.0f) / 0.3f) * (16384.0f / 20.0f),
                            ((leg_tl.Tw * 17.0f / 268.0f)  / 0.3f) * (16384.0f / 20.0f), 0, 0);
        osDelay(0);
    }
}
