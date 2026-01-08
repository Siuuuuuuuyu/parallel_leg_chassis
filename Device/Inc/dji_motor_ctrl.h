//
// Created by sy on 2025/12/17.
//

#ifndef PARALLEL_LEG_CHASSIS_DJI_MOTOR_CTRL_H
#define PARALLEL_LEG_CHASSIS_DJI_MOTOR_CTRL_H

#include "main.h"
#include "bsp_can.h"
#include "motor_device.h"

typedef struct
{
    uint16_t send_id;
    uint16_t rec_id;
    double speed_ratio; // 减速比
    double Ct; // 3508转子的转矩常数
    motor_t m;
} M3508motor_t;

extern M3508motor_t M3508_1;
extern M3508motor_t M3508_2;

void M3508_init(M3508motor_t *m3508, uint8_t id, double speed_ratio, double rec_time_out, uint16_t device_index);
void M3508_fbdata(M3508motor_t *m3508, uint8_t *data);
void M3508_cmd_upgrade(M3508motor_t *m3508, double torque);
void dji_motor_cmd_send(uint16_t id, hcan_t *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif //PARALLEL_LEG_CHASSIS_DJI_MOTOR_CTRL_H