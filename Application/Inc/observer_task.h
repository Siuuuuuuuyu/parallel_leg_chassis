//
// Created by sy on 2025/12/24.
//

#ifndef PARALLEL_LEG_CHASSIS_OBSERVER_TASK_H
#define PARALLEL_LEG_CHASSIS_OBSERVER_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "StateObserver.h"
#include "imu_task.h"
#include "chassis_task.h"

extern float d_xb;
extern float dd_xb;

void observer_task_init();
void observer_task(void const * argument);

#endif //PARALLEL_LEG_CHASSIS_OBSERVER_TASK_H