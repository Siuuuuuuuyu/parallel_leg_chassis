//
// Created by sy on 2025/12/15.
//
#include "dm_motor_ctrl.h"

//内部函数声明
static int float_to_uint(float x_float, float x_min, float x_max, int bits);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);
static void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
static void mit_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq, lim_para *lp);
/***********************************DM8009电机**********************************/
//扭矩系数=1.5*Npp(极对数)*flux(磁链)*减速比 (1.5*21*0.004374458*9)
// #define DM8009_ROTOR_TORQUE_CONSTANT (1.240158843)
#define DM8009_IMAX (41.044777)

void DM8009_init(DM8009_motor_t *DM8009, hcan_t *hcan, dm_motor_mode mode, uint16_t send_id, uint16_t rec_id, double zero_pos,
                 double rec_time_out, uint16_t device_index)
{
    memset((void *)DM8009, 0, sizeof(DM8009_motor_t));
    DM8009->send_id = send_id;
    DM8009->rec_id = rec_id;
    DM8009->mode = mode;
    DM8009->hcan = hcan;
    DM8009->lp.p_min = -12.5f;
    DM8009->lp.p_max = 12.5f;
    DM8009->lp.v_min = -45.0f;
    DM8009->lp.v_max = 45.0f;
    DM8009->lp.kp_min = 0.0f;
    DM8009->lp.kp_max = 500.0f;
    DM8009->lp.kd_min = 0.0f;
    DM8009->lp.kd_max = 5.0f;
    DM8009->lp.t_min = -54.0f;
    DM8009->lp.t_max = 54.0f;
    DM8009->Ct = 1.240158843;
    if (mode != TO4_MODE)
    {
        enable_motor_mode(hcan, send_id, mode);
    }
    motor_init(&(DM8009->m), zero_pos, rec_time_out, device_index);
}

void DM8009_fbdata(DM8009_motor_t *DM8009, unsigned char data[])
{
    double pos_fbk, vel_fbk, tor_fbk, temperature;
    if (DM8009->mode != TO4_MODE)
    {
        DM8009->state = (data[0]) >> 4;
        int p_int, v_int, t_int;
        p_int = (data[1] << 8) | data[2];
        v_int = (data[3] << 4) | (data[4] >> 4);
        t_int = ((data[4] & 0xF) << 8) | data[5];
        pos_fbk = uint_to_float(p_int, DM8009->lp.p_min, DM8009->lp.p_max, 16);
        vel_fbk = uint_to_float(v_int, DM8009->lp.v_min, DM8009->lp.v_max, 12);
        tor_fbk = uint_to_float(t_int, DM8009->lp.t_min, DM8009->lp.t_max, 12);
        temperature = (float)(data[7]);
    }
    else
    {
        pos_fbk = ((((short)data[0]) << 8) + data[1]) * (360.0 / 8191) * (PI / 180);                  //->degree->rad
        vel_fbk = (double)((short)((((short)data[2]) << 8) + data[3])) * (2 * PI) / 60 / 100;         //放大100倍数 rpm->rad/s
        tor_fbk = (double)((short)((((short)data[4]) << 8) + data[5])) * (1.0 / 1000.0) * DM8009->Ct; // mA->A->N*m
        temperature = data[6];
    }
    motor_fbdata(&(DM8009->m), temperature, tor_fbk, vel_fbk, pos_fbk);
}

void DM8009_cmd_upgrade(DM8009_motor_t *DM8009, double torque)
{
    if (torque > 45)
        torque = 45;
    if (torque < -45)
        torque = -45;
    DM8009->m.cmd.tor_set = torque;
    if (DM8009->mode == TO4_MODE)
    {
        double I_cmd = torque / DM8009->Ct;
        if (I_cmd > DM8009_IMAX)
            I_cmd = DM8009_IMAX;
        if (I_cmd < -DM8009_IMAX)
            I_cmd = -DM8009_IMAX;
        int16_t temp_cmd = (I_cmd / DM8009_IMAX) * 16384;
        if (temp_cmd > 16384)
            temp_cmd = 16384;
        if (temp_cmd < -16384)
            temp_cmd = -16384;
        DM8009->m.cmd.cmd_signal = temp_cmd;
        motor_refresh(&(DM8009->m));
    }
    else
    {
        mit_ctrl(DM8009->hcan, DM8009->send_id, 0, 0, 0, 0, torque, &(DM8009->lp));
        motor_refresh(&(DM8009->m));
    }
}

/*********************************1to4协议发送**********************************/
static uint8_t dm_motor_tx_buf[8] = {0};
void DM8009_send_1to4(uint16_t id, hcan_t *hcan, short cmd1, short cmd2, short cmd3, short cmd4)
{
    dm_motor_tx_buf[1] = (cmd1 >> 8);
    dm_motor_tx_buf[0] = cmd1;
    dm_motor_tx_buf[3] = (cmd2 >> 8);
    dm_motor_tx_buf[2] = cmd2;
    dm_motor_tx_buf[5] = (cmd3 >> 8);
    dm_motor_tx_buf[4] = cmd3;
    dm_motor_tx_buf[7] = (cmd4 >> 8);
    dm_motor_tx_buf[6] = cmd4;
    // can发送
    bsp_fdcan_send(hcan, id, dm_motor_tx_buf, 8);
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向hcan_tTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
static void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    bsp_fdcan_send(hcan, id, data, 8);
}

static void mit_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq, lim_para *lp)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;
    pos_tmp = float_to_uint(pos, lp->p_min, lp->p_max, 16);
    vel_tmp = float_to_uint(vel, lp->v_min, lp->v_max, 12);
    kp_tmp = float_to_uint(kp, lp->kp_min, lp->kp_max, 12);
    kd_tmp = float_to_uint(kd, lp->kd_min, lp->kd_max, 12);
    tor_tmp = float_to_uint(torq, lp->t_min, lp->t_max, 12);
    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    bsp_fdcan_send(hcan, id, data, 8);
}
