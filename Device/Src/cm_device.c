//
// Created by sy on 2025/12/18.
//
#include "cm_device.h"

DEVICE_STATE device_state_list[MAX_DEVICE_NUM];

void device_init(CM_t *cm, double time_out, uint16_t device_index)
{
    cm->time_out = time_out; // 设置超时阈值
    if (device_index > MAX_DEVICE_NUM)
        device_index = MAX_DEVICE_NUM;
    if (device_index < 1)
        device_index = 1;
    cm->device_index = device_index; // 存储设备索引
    cm->state = DEVICE_OK;
}

void device_fb(CM_t *cm)
{
    // 更新计时点
    cm->no_data_time = 0; // 重置无数据时间
    DWT_GetDeltaT(&(cm->device_count)); // 更新时间戳
}

void device_refresh(CM_t *cm)
{
    // 获取自上次调用以来的时间间隔
    float dt = DWT_GetDeltaT(&(cm->device_count));
    // 累加无数据时间
    cm->no_data_time += dt;
    // 检查是否超时
    if (cm->no_data_time > cm->time_out)
    {
        cm->state = DEVICE_NOT_CONNECTED;
        if (cm->device_index - 1 < 0)
            return;
        device_state_list[cm->device_index - 1] = DEVICE_NOT_CONNECTED;
    }
    else
    {
        cm->state = DEVICE_OK;
        if (cm->device_index - 1 < 0)
            return;
        device_state_list[cm->device_index - 1] = DEVICE_OK;
    }
}
