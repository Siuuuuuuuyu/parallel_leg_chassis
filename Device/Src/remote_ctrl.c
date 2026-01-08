//
// Created by sy on 2026/1/3.
//
#include "remote_ctrl.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

RC_ctrl_t rc_ctrl;

void rc_ctrl_init(RC_ctrl_t *rc_ctrl, double rec_time_out, uint16_t device_index)
{
    device_init(&(rc_ctrl->cm), rec_time_out, device_index);
}

void rc_ctrl_fbdata(RC_ctrl_t *rc_ctrl, uint8_t *data)
{
    rc_ctrl->rc.ch[0] = (data[0] | (data[1] << 8)) & 0x07ff;                 //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
    rc_ctrl->rc.ch[1] = (((data[1] >> 3) & 0xff) | (data[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = (((data[2] >> 6) & 0xff) | (data[3] << 2) |          //!< Channel 2
                         (data[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = (((data[4] >> 1) & 0xff) | (data[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((data[5] >> 4) & 0x0003);                            //!< Switch right
    rc_ctrl->rc.s[1] = ((data[5] >> 4) & 0x000C) >> 2;                       //!< Switch left
    rc_ctrl->mouse.x = data[6] | (data[7] << 8);                             //!< Mouse X axis
    rc_ctrl->mouse.y = data[8] | (data[9] << 8);                             //!< Mouse Y axis
    rc_ctrl->mouse.z = data[10] | (data[11] << 8);                           //!< Mouse Z axis
    rc_ctrl->mouse.press_l = data[12];                                       //!< Mouse Left Is Press
    rc_ctrl->mouse.press_r = data[13];                                       //!< Mouse Right Is Press
    rc_ctrl->key.v = data[14] | (data[15] << 8);                             //!< KeyBoard value
    rc_ctrl->rc.ch[4] = data[16] | (data[17] << 8);                          // channel 4

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    // Some flag of keyboard
    rc_ctrl->key.w_flag = (data[14] & 0x01);
    rc_ctrl->key.s_flag = (data[14] & 0x02);
    rc_ctrl->key.a_flag = (data[14] & 0x04);
    rc_ctrl->key.d_flag = (data[14] & 0x08);
    rc_ctrl->key.q_flag = (data[14] & 0x40);
    rc_ctrl->key.e_flag = (data[14] & 0x80);
    rc_ctrl->key.shift_flag = (data[14] & 0x10);
    rc_ctrl->key.ctrl_flag = (data[14] & 0x20);
    rc_ctrl->key.r_flag = (data[15] & 0x01);
    rc_ctrl->key.f_flag = (data[15] & 0x02);
    rc_ctrl->key.g_flag = (data[15] & 0x04);
    rc_ctrl->key.z_flag = (data[15] & 0x08);
    rc_ctrl->key.x_flag = (data[15] & 0x10);
    rc_ctrl->key.c_flag = (data[15] & 0x20);
    rc_ctrl->key.v_flag = (data[15] & 0x40);
    rc_ctrl->key.b_flag = (data[15] & 0x80);
    if (rc_ctrl->key.w_flag != 0)
        rc_ctrl->key.w_flag = 1;
    if (rc_ctrl->key.s_flag != 0)
        rc_ctrl->key.s_flag = 1;
    if (rc_ctrl->key.a_flag != 0)
        rc_ctrl->key.a_flag = 1;
    if (rc_ctrl->key.d_flag != 0)
        rc_ctrl->key.d_flag = 1;
    if (rc_ctrl->key.q_flag != 0)
        rc_ctrl->key.q_flag = 1;
    if (rc_ctrl->key.e_flag != 0)
        rc_ctrl->key.e_flag = 1;
    if (rc_ctrl->key.shift_flag != 0)
        rc_ctrl->key.shift_flag = 1;
    if (rc_ctrl->key.ctrl_flag != 0)
        rc_ctrl->key.ctrl_flag = 1;
    if (rc_ctrl->key.r_flag != 0)
        rc_ctrl->key.r_flag = 1;
    if (rc_ctrl->key.f_flag != 0)
        rc_ctrl->key.f_flag = 1;
    if (rc_ctrl->key.g_flag != 0)
        rc_ctrl->key.g_flag = 1;
    if (rc_ctrl->key.z_flag != 0)
        rc_ctrl->key.z_flag = 1;
    if (rc_ctrl->key.x_flag != 0)
        rc_ctrl->key.x_flag = 1;
    if (rc_ctrl->key.c_flag != 0)
        rc_ctrl->key.c_flag = 1;
    if (rc_ctrl->key.v_flag != 0)
        rc_ctrl->key.v_flag = 1;
    if (rc_ctrl->key.b_flag != 0)
        rc_ctrl->key.b_flag = 1;
    device_fb(&(rc_ctrl->cm));
}

void rc_ctrl_refresh(RC_ctrl_t *rc_ctrl)
{
    device_refresh(&(rc_ctrl->cm));
}
