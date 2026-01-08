//
// Created by sy on 2025/12/22.
//

#ifndef PARALLEL_LEG_CHASSIS_CHASSIS_TASK_H
#define PARALLEL_LEG_CHASSIS_CHASSIS_TASK_H

#include "main.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "dji_motor_ctrl.h"
#include "dm_motor_ctrl.h"
#include "pid.h"
#include "LegController.h"
#include "imu_task.h"
#include "observer_task.h"

void chassis_init();
void chassis_task(void const * argument);

#endif //PARALLEL_LEG_CHASSIS_CHASSIS_TASK_H