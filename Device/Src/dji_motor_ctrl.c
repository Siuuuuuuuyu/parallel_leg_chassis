//
// Created by sy on 2025/12/17.
//
#include "dji_motor_ctrl.h"

dji_motor_info M3508_1;
pid_type_def m3508_speed_pid;

void dji_motor_ctrl_send(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint16_t id = 0x200;
    uint8_t data[8];

    data[0] = motor1 >> 8;
    data[1] = motor1;
    data[2] = motor2 >> 8;
    data[3] = motor2;
    data[4] = motor3 >> 8;
    data[5] = motor3;
    data[6] = motor4 >> 8;
    data[7] = motor4;

    bsp_fdcan_send(&hfdcan1, id, data, 8);
}
