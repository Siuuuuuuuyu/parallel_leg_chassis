//
// Created by sy on 2026/1/3.
//

#ifndef PARALLEL_LEG_CHASSIS_REMOTE_CTRL_H
#define PARALLEL_LEG_CHASSIS_REMOTE_CTRL_H

#include "cm_device.h"

typedef struct
{
    struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    struct
    {
        int16_t x, y, z;
        uint8_t press_l, press_r;
    } mouse;
    struct
    {
        uint16_t v;
        uint8_t w_flag, s_flag, a_flag, d_flag, q_flag, e_flag, shift_flag, ctrl_flag,
            r_flag, f_flag, g_flag, z_flag, x_flag, c_flag, v_flag, b_flag;
    } key;
    struct
    {
        int16_t ch[5];
        char s[2];
    } last_rc;
    struct
    {
        int16_t x, y, z;
        uint8_t press_l, press_r;
    } last_mouse;
    struct
    {
        uint16_t v;
        uint8_t w_flag, s_flag, a_flag, d_flag, q_flag, e_flag, shift_flag, ctrl_flag,
            r_flag, f_flag, g_flag, z_flag, x_flag, c_flag, v_flag, b_flag;
    } last_key;
    CM_t cm;
} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

void rc_ctrl_init(RC_ctrl_t *rc_ctrl, double rec_time_out, uint16_t device_index);
void rc_ctrl_fbdata(RC_ctrl_t *rc_ctrl, uint8_t *data);
void rc_ctrl_refresh(RC_ctrl_t *rc_ctrl);

#endif //PARALLEL_LEG_CHASSIS_REMOTE_CTRL_H