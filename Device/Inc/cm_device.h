//
// Created by sy on 2025/12/18.
//

#ifndef PARALLEL_LEG_CHASSIS_CM_DEVICE_H
#define PARALLEL_LEG_CHASSIS_CM_DEVICE_H

#include "bsp_dwt.h"

#ifndef MAX_DEVICE_NUM
#define MAX_DEVICE_NUM (100)
#endif

typedef enum
{
    DEVICE_OK = 1,
    DEVICE_NOT_CONNECTED = 2
} DEVICE_STATE;
#pragma pack(1)
typedef struct
{
    uint32_t device_count; //用于dwt计算时间
    double no_data_time;
    double time_out;
    uint16_t device_index; //设备状态列表的系数1~MAX_DEVICE_NUM
    DEVICE_STATE state;
} CM_t; // connection monitor
#pragma pack()
extern DEVICE_STATE device_state_list[MAX_DEVICE_NUM];

void device_init(CM_t *cm, double time_out, uint16_t device_index);
void device_fb(CM_t *cm);
void device_refresh(CM_t *cm);

#endif //PARALLEL_LEG_CHASSIS_CM_DEVICE_H