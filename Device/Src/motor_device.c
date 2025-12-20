//
// Created by sy on 2025/12/18.
//
#include "motor_device.h"
// zero_pos -pi~pi 用户指定零点 若超过范围，则上电第一帧数据的pos为零点
void motor_init(motor_t *m, double zero_pos, double rec_time_out, uint16_t device_index)
{
    memset((void *)m, 0, sizeof(motor_t)); //清零
    if (zero_pos <= PI && zero_pos >= -PI)
    {
        m->ad.zero_pos = zero_pos;
        m->ad.zero_pos_set_mode = 0;
    }
    else
        m->ad.zero_pos_set_mode = 1;
    m->ad.is_init = 1;
    m->ad.package_num = 0;
    device_init(&(m->cm), rec_time_out, device_index);
}
// tor_fbk vel_fbk pos_fbk电机数据反馈 N*m rad/s rad -pi~pi
void motor_fbdata(motor_t *m, uint8_t temperature, double tor_fbk, double vel_fbk, double pos_fbk)
{
    if (m->ad.zero_pos_set_mode == 1 && m->ad.is_init == 1 && m->ad.package_num == 0) // 电机开启记录零点模式，初始化后第一次收到数据
        m->ad.zero_pos = pos_fbk;
    m->para.temperature = temperature;
    m->para.pos_fb = pos_fbk - m->ad.zero_pos;
    if (m->para.pos_fb > PI)
        m->para.pos_fb -= 2 * PI;
    if (m->para.pos_fb < -PI)
        m->para.pos_fb += 2 * PI;
    m->para.vel_fb = vel_fbk;
    m->para.tor_fb = tor_fbk;
    if (m->ad.is_init == 0) //第一次收到数据
    {
        m->last_para.temperature = temperature;
        m->last_para.pos_fb = pos_fbk;
        m->last_para.vel_fb = vel_fbk;
        m->last_para.tor_fb = tor_fbk;
    }
    //记录圈数
    if (m->para.pos_fb - m->last_para.pos_fb < -PI)
        m->ad.ring_num += 1;
    if (m->para.pos_fb - m->last_para.pos_fb > PI)
        m->ad.ring_num -= 1;
    m->ad.total_angle = m->ad.ring_num * PI * 2 + m->para.pos_fb; //记录总角度
    //更新计时点
    device_fb(&(m->cm));
    //记录上一次
    m->last_para.temperature = m->para.temperature;
    m->last_para.pos_fb = m->para.pos_fb;
    m->last_para.vel_fb = m->para.vel_fb;
    m->last_para.tor_fb = m->para.tor_fb;
    //包计数
    m->ad.package_num++;
}
//电机刷新记录时间,在某个循环里执行即可
void motor_refresh(motor_t *m)
{
    device_refresh(&(m->cm));
}

