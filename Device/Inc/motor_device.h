//
// Created by sy on 2025/12/18.
//

#ifndef PARALLEL_LEG_CHASSIS_MOTOR_DEVICE_H
#define PARALLEL_LEG_CHASSIS_MOTOR_DEVICE_H

#include "stdint.h"
#include "string.h"
#include "bsp_dwt.h"
#include "cm_device.h"

#ifndef PI
#define PI (3.14159265358979311599796346854418516159057617)
#endif

typedef struct motor_device
{
    struct
    {
        float tor_set;
        int16_t cmd_signal; //某些电机最终装入发送报文的数据
    } cmd;
    struct
    {
        uint8_t temperature; // degree
        double tor_fb;       // N*m
        double vel_fb;       // rad/s
        double pos_fb;       // rad -pi~pi
    } para;
    struct
    {
        uint8_t temperature; // degree
        double tor_fb;       // N*m
        double vel_fb;       // rad/s
        double pos_fb;       // rad -pi~pi
    } last_para;
    struct
    {
        int ring_num;
        double total_angle; //记录总转角
        char is_init;
        uint32_t package_num;
        double zero_pos; //用户设定的电机0角度值
        uint8_t zero_pos_set_mode;
    } ad; // motor_advanced_data
    CM_t cm;
} motor_t;

// zero_pos -pi~pi 用户指定零点 若超过范围，则上电第一帧数据的pos为零点
void motor_init(motor_t *m, double zero_pos, double rec_time_out, uint16_t device_index);
void motor_fbdata(motor_t *m, uint8_t temperature, double tor_fbk, double vel_fbk, double pos_fbk);
void motor_refresh(motor_t *m);

#endif //PARALLEL_LEG_CHASSIS_MOTOR_DEVICE_H