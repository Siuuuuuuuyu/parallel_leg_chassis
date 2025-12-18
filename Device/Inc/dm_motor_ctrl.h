//
// Created by sy on 2025/12/15.
//

#ifndef PARALLEL_LEG_CHASSIS_DM_MOTOR_CTRL_H
#define PARALLEL_LEG_CHASSIS_DM_MOTOR_CTRL_H

#include "string.h"
#include "bsp_can.h"
#include "motor_device.h"

typedef enum
{
    MIT_MODE = 0x000,
    POS_MODE = 0x100,
    SPEED_MODE = 0x200,
    TO4_MODE = 0x300
} dm_motor_mode;

/*
8——超压；
9——欠压；
A——过电流；
B——MOS 过温；
C——电机线圈过温；
D——通讯丢失；
E——过载；
*/
typedef enum
{
    OVER_VOL = 0x08,
    UNDER_VOL = 0x09,
    OVER_CURRENT = 0x0a,
    MOS_TOO_HOT = 0x0b,
    WIRE_TOO_HOT = 0x0c,
    COMUNICATION_LOSS = 0x0d,
    OVER_LOAD = 0x0e
} dm_error_type;

typedef struct
{
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
} lim_para;

typedef struct
{
    dm_motor_mode mode;
    uint16_t send_id;
    uint16_t rec_id;
    double Ct;
    motor_t m;
    lim_para lp;
    hcan_t *hcan;
    int state; //达妙电机的状态码
} DM8009_motor_t;

void DM8009_init(DM8009_motor_t *DM8009, hcan_t *hcan, dm_motor_mode mode, uint16_t send_id, uint16_t rec_id, double zero_pos,
                 double rec_time_out, uint16_t device_index);
void DM8009_fbdata(DM8009_motor_t *DM8009, unsigned char data[]);
void DM8009_cmd_upgrade(DM8009_motor_t *DM8009, double torque);
void DM8009_send_1to4(uint16_t id, hcan_t *hcan, short cmd1, short cmd2, short cmd3, short cmd4);


#endif //PARALLEL_LEG_CHASSIS_DM_MOTOR_CTRL_H