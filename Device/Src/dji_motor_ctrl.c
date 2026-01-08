//
// Created by sy on 2025/12/17.
//
#include "dji_motor_ctrl.h"

M3508motor_t M3508_1;
M3508motor_t M3508_2;

void M3508_init(M3508motor_t *m3508, uint8_t id, double speed_ratio, double rec_time_out, uint16_t device_index)
{
    memset((void *)m3508, 0, sizeof(M3508motor_t));
    if (id <= 4 && id > 0)
        m3508->send_id = 0x200;
    else if (id > 4 && id <= 8)
        m3508->send_id = 0x1FF;
    else
        return;
    m3508->rec_id = 0x200 + id;
    m3508->speed_ratio = speed_ratio;
    m3508->Ct = (0.3 * 187.0 / 3591.0) * speed_ratio;
    motor_init(&(m3508->m), 2.0 * PI, rec_time_out, device_index);
}

void M3508_fbdata(M3508motor_t *m3508, uint8_t *data)
{
    double pos_fbk, vel_fbk, tor_fbk, temperature;
    pos_fbk = ((((int16_t)data[0]) << 8) + data[1]) * (360.0 / 8191.0) * (PI / 180.0) - PI;            // ->degree->rad
    vel_fbk = (double)((int16_t)((((int16_t)data[2]) << 8) | data[3])) * (2.0 * PI) / 60.0;            // rpm->rad/s
    tor_fbk = (double)((int16_t)((((int16_t)data[4]) << 8) | data[5])) * (20.0 / 16384.0) * m3508->Ct; // ->A->N*m
    temperature = data[6];
    motor_fbdata(&(m3508->m), temperature, tor_fbk, vel_fbk, pos_fbk);
}

void M3508_cmd_upgrade(M3508motor_t *m3508, double torque)
{
    m3508->m.cmd.tor_set = torque;
    double I_cmd = torque / m3508->Ct;
    if (I_cmd > 20.0)
        I_cmd = 20.0;
    if (I_cmd < -20.0)
        I_cmd = -20.0;
    int16_t temp_cmd = (I_cmd / 20.0) * 16384.0;
    if (temp_cmd > 16384.0)
        temp_cmd = 16384.0;
    if (temp_cmd < -16384.0)
        temp_cmd = -16384.0;
    m3508->m.cmd.cmd_signal = temp_cmd;
    motor_refresh(&(m3508->m));
}

static uint8_t dji_motor_tx_buf[8] = {0};
void dji_motor_cmd_send(uint16_t id, hcan_t *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    dji_motor_tx_buf[0] = motor1 >> 8;
    dji_motor_tx_buf[1] = motor1;
    dji_motor_tx_buf[2] = motor2 >> 8;
    dji_motor_tx_buf[3] = motor2;
    dji_motor_tx_buf[4] = motor3 >> 8;
    dji_motor_tx_buf[5] = motor3;
    dji_motor_tx_buf[6] = motor4 >> 8;
    dji_motor_tx_buf[7] = motor4;

    bsp_fdcan_send(hcan, id, dji_motor_tx_buf, 8);
}
