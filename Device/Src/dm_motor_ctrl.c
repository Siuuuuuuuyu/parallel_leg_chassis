//
// Created by sy on 2025/12/15.
//
#include "dm_motor_ctrl.h"

motor_t motor[num];

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化1个DM4310型号的电机，设置默认参数和控制模式。
*               设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm_motor_init(void)
{
    // 初始化Motor1的电机结构
    memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
    // memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
    // memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
    // memset(&motor[Motor4], 0, sizeof(motor[Motor4]));

    // 设置Motor1的电机信息
    motor[Motor1].id = 0x01;
    // motor[Motor1].mst_id = 0x00;
    motor[Motor1].tmp.read_flag = 1;
    motor[Motor1].ctrl.mode 	= mit_mode;
    motor[Motor1].ctrl.vel_set 	= 1.0f;
    motor[Motor1].ctrl.pos_set 	= 5.0f;
    motor[Motor1].ctrl.tor_set 	= 0.5f;
    motor[Motor1].ctrl.kp_set 	= 10.0f;
    motor[Motor1].ctrl.kd_set 	= 1.0f;
    motor[Motor1].tmp.PMAX		= 12.5f;
    motor[Motor1].tmp.VMAX		= 30.0f;
    motor[Motor1].tmp.TMAX		= 10.0f;
}
/**
************************************************************************
* @brief:      	dm4310_enable: 启用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式启用相应的模式，通过CAN总线发送启用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm_motor_enable(hcan_t* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            enable_motor_mode(hcan, motor->id, MIT_MODE);break;
        case pos_mode:
            enable_motor_mode(hcan, motor->id, POS_MODE);break;
        case spd_mode:
            enable_motor_mode(hcan, motor->id, SPD_MODE);break;
        case psi_mode:
            enable_motor_mode(hcan, motor->id, PSI_MODE);break;
    }
}

/**
************************************************************************
* @brief:      	dm4310_disable: 禁用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式禁用相应的模式，通过CAN总线发送禁用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm_motor_disable(hcan_t* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            disable_motor_mode(hcan, motor->id, MIT_MODE);
            break;
        case pos_mode:
            disable_motor_mode(hcan, motor->id, POS_MODE);
            break;
        case spd_mode:
            disable_motor_mode(hcan, motor->id, SPD_MODE);
            break;
        case psi_mode:
            disable_motor_mode(hcan, motor->id, PSI_MODE);
            break;
    }
    dm_motor_clear_para(motor);
}

/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 发送DM4310电机控制命令函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式发送相应的命令到DM4310电机
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
            break;
        case pos_mode:
            pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
            break;
        case spd_mode:
            spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
            break;
        case psi_mode:
            psi_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
            break;
    }
}

/**
************************************************************************
* @brief:      	dm4310_set: 设置DM4310电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据命令参数设置DM4310电机的控制参数，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void dm_motor_set_para(motor_t *motor)
{
    motor->ctrl.kd_set 	= motor->cmd.kd_set;
    motor->ctrl.kp_set	= motor->cmd.kp_set;
    motor->ctrl.pos_set	= motor->cmd.pos_set;
    motor->ctrl.vel_set	= motor->cmd.vel_set;
    motor->ctrl.tor_set	= motor->cmd.tor_set;
}

/**
************************************************************************
* @brief:      	dm4310_clear: 清除DM4310电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	将DM4310电机的命令参数和控制参数清零，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void dm_motor_clear_para(motor_t *motor)
{
    motor->ctrl.kd_set 	= 0;
    motor->ctrl.kp_set	= 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
    motor->ctrl.cur_set = 0;
}

/**
************************************************************************
* @brief:      	dm4310_clear_err: 清除DM4310电机错误函数
* @param[in]:   hcan: 	 指向CAN控制结构体的指针
* @param[in]:  	motor:   指向电机结构体的指针
* @retval:     	void
* @details:    	根据电机的控制模式，调用对应模式的清除错误函数
************************************************************************
**/
void dm_motor_clear_err(hcan_t* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            clear_err(hcan, motor->id, MIT_MODE);
            break;
        case pos_mode:
            clear_err(hcan, motor->id, POS_MODE);
            break;
        case spd_mode:
            clear_err(hcan, motor->id, SPD_MODE);
            break;
        case psi_mode:
            clear_err(hcan, motor->id, PSI_MODE);
            break;
    }
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id = (rx_data[0])&0x0F;
    motor->para.state = (rx_data[0])>>4;
    motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
    motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
    motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
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
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
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
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
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

/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
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
    data[7] = 0xFD;

    bsp_fdcan_send(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	save_pos_zero: 保存位置零点函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要保存位置零点的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
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
    data[7] = 0xFE;

    bsp_fdcan_send(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
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
    data[7] = 0xFB;

    bsp_fdcan_send(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor)
{
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

    data[0] = pos_tmp >> 8;
    data[1] = pos_tmp;
    data[2] = vel_tmp >> 4;
    data[3] = (vel_tmp & 0xF) << 4 | kp_tmp >> 8;
    data[4] = kp_tmp;
    data[5] = kd_tmp >> 4;
    data[6] = (kd_tmp & 0xF) << 4 | tor_tmp >> 8;
    data[7] = tor_tmp;

    bsp_fdcan_send(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + POS_MODE;
    pbuf=(uint8_t*)&pos;
    vbuf=(uint8_t*)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf+1);
    data[2] = *(pbuf+2);
    data[3] = *(pbuf+3);

    data[4] = *vbuf;
    data[5] = *(vbuf+1);
    data[6] = *(vbuf+2);
    data[7] = *(vbuf+3);

    bsp_fdcan_send(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];

    id = motor_id + SPD_MODE;
    vbuf=(uint8_t*)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);

    bsp_fdcan_send(hcan, id, data, 4);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 混控模式
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   i:				电流给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf, *ibuf;
    uint8_t data[8];

    uint16_t u16_vel = vel*100;
    uint16_t u16_cur  = cur*10000;

    id = motor_id + PSI_MODE;
    pbuf=(uint8_t*)&pos;
    vbuf=(uint8_t*)&u16_vel;
    ibuf=(uint8_t*)&u16_cur;

    data[0] = *pbuf;
    data[1] = *(pbuf+1);
    data[2] = *(pbuf+2);
    data[3] = *(pbuf+3);

    data[4] = *vbuf;
    data[5] = *(vbuf+1);

    data[6] = *ibuf;
    data[7] = *(ibuf+1);

    bsp_fdcan_send(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	read_motor_data: 发送读取寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	读取电机参数
************************************************************************
**/
// void read_motor_data(uint16_t id, uint8_t rid)
// {
//     uint8_t can_id_l = id & 0xFF;       // 低 8 位
//     uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
//
//     uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
//     fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
// }

/**
************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 发送读取电机反馈数据的命令
* @param[in]:   id:    电机can id
* @retval:     	void
* @details:    	读取电机控制反馈的数据
************************************************************************
**/
// void read_motor_ctrl_fbdata(uint16_t id)
// {
//     uint8_t can_id_l = id & 0xFF;       // 低 8 位
//     uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
//
//     uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
//     fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
// }

/**
************************************************************************
* @brief:      	write_motor_data: 发送写寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @param[in]:   d0-d3: 写入的数据
* @retval:     	void
* @details:    	向寄存器写入数据
************************************************************************
**/
// void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
// {
//     uint8_t can_id_l = id & 0x0F;
//     uint8_t can_id_h = (id >> 4) & 0x0F;
//
//     uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
//     fdcanx_send_data(&hfdcan1, 0x7FF, data, 8);
// }

/**
************************************************************************
* @brief:      	save_motor_data: 发送保存命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	保存写入的电机参数
************************************************************************
**/
// void save_motor_data(uint16_t id, uint8_t rid)
// {
//     uint8_t can_id_l = id & 0xFF;       // 低 8 位
//     uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
//
//     uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
//     fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
// }

/**
************************************************************************
* @brief:      	read_all_motor_data: 读取电机的所有寄存器的数据信息
* @param:      	motor_t：电机参数结构体
* @retval:     	void
* @details:    	逐次发送读取命令
************************************************************************
**/
// void read_all_motor_data(motor_t *motor)
// {
//     switch (motor->tmp.read_flag)
//     {
// 		case 1:	 read_motor_data(motor->id, 0);  break; // UV_Value
//         case 2:	 read_motor_data(motor->id, 1);  break; // KT_Value
// 		case 3:  read_motor_data(motor->id, 2);  break; // OT_Value
//         case 4:  read_motor_data(motor->id, 3);  break; // OC_Value
// 		case 5:	 read_motor_data(motor->id, 4);  break; // ACC
//         case 6:	 read_motor_data(motor->id, 5);  break; // DEC
// 		case 7:  read_motor_data(motor->id, 6);  break; // MAX_SPD
//         case 8:  read_motor_data(motor->id, 7);  break; // MSC_ID
// 		case 9:  read_motor_data(motor->id, 8);  break; // ESC_ID
//         case 10: read_motor_data(motor->id, 9);  break; // TIMEOUT
// 		case 11: read_motor_data(motor->id, 10); break; // CTRL_MODE
//         case 12: read_motor_data(motor->id, 11); break; // Damp
// 		case 13: read_motor_data(motor->id, 12); break; // Inertia
//         case 14: read_motor_data(motor->id, 13); break; // Rsv1
// 		case 15: read_motor_data(motor->id, 14); break; // sw_ver
//         case 16: read_motor_data(motor->id, 15); break; // Rsv2
// 		case 17: read_motor_data(motor->id, 16); break; // NPP
//         case 18: read_motor_data(motor->id, 17); break; // Rs
// 		case 19: read_motor_data(motor->id, 18); break; // Ls
//         case 20: read_motor_data(motor->id, 19); break; // Flux
// 		case 21: read_motor_data(motor->id, 20); break; // Gr
//         case 22: read_motor_data(motor->id, 21); break; // PMAX
// 		case 23: read_motor_data(motor->id, 22); break; // VMAX
//         case 24: read_motor_data(motor->id, 23); break; // TMAX
// 		case 25: read_motor_data(motor->id, 24); break; // I_BW
//         case 26: read_motor_data(motor->id, 25); break; // KP_ASR
// 		case 27: read_motor_data(motor->id, 26); break; // KI_ASR
//         case 28: read_motor_data(motor->id, 27); break; // KP_APR
// 		case 29: read_motor_data(motor->id, 28); break; // KI_APR
// 		case 30: read_motor_data(motor->id, 29); break; // OV_Value
//         case 31: read_motor_data(motor->id, 30); break; // GREF
// 		case 32: read_motor_data(motor->id, 31); break; // Deta
//         case 33: read_motor_data(motor->id, 32); break; // V_BW
// 		case 34: read_motor_data(motor->id, 33); break; // IQ_c1
//         case 35: read_motor_data(motor->id, 34); break; // VL_c1
// 		case 36: read_motor_data(motor->id, 35); break; // can_br
//         case 37: read_motor_data(motor->id, 36); break; // sub_ver
// 		case 38: read_motor_data(motor->id, 50); break; // u_off
//         case 39: read_motor_data(motor->id, 51); break; // v_off
// 		case 40: read_motor_data(motor->id, 52); break; // k1
//         case 41: read_motor_data(motor->id, 53); break; // k2
// 		case 42: read_motor_data(motor->id, 54); break; // m_off
// 		case 43: read_motor_data(motor->id, 55); break; // dir
// 		case 44: read_motor_data(motor->id, 80); break; // pm
// 		case 45: read_motor_data(motor->id, 81); break; // xout
//     }
// }

/**
************************************************************************
* @brief:      	receive_motor_data: 接收电机返回的数据信息
* @param:      	motor_t：电机参数结构体
* @param:      	data：接收的数据
* @retval:     	void
* @details:    	逐次接收电机回传的参数信息
************************************************************************
**/
// void receive_motor_data(motor_t *motor, uint8_t *data)
// {
// 	if(motor->tmp.read_flag == 0)
// 		return ;
//
// 	float_type_u y;
//
// 	if(data[2] == 0x33)
// 	{
// 		y.b_val[0] = data[4];
// 		y.b_val[1] = data[5];
// 		y.b_val[2] = data[6];
// 		y.b_val[3] = data[7];
//
// 		switch(data[3])
// 		{
// 			case  0: motor->tmp.UV_Value = y.f_val; motor->tmp.read_flag =  2; break;
// 			case  1: motor->tmp.KT_Value = y.f_val; motor->tmp.read_flag =  3; break;
// 			case  2: motor->tmp.OT_Value = y.f_val; motor->tmp.read_flag =  4; break;
// 			case  3: motor->tmp.OC_Value = y.f_val; motor->tmp.read_flag =  5; break;
// 			case  4: motor->tmp.ACC 	 = y.f_val; motor->tmp.read_flag =  6; break;
// 			case  5: motor->tmp.DEC 	 = y.f_val; motor->tmp.read_flag =  7; break;
// 			case  6: motor->tmp.MAX_SPD  = y.f_val; motor->tmp.read_flag =  8; break;
// 			case  7: motor->tmp.MST_ID   = y.u_val; motor->tmp.read_flag =  9; break;
// 			case  8: motor->tmp.ESC_ID   = y.u_val; motor->tmp.read_flag = 10; break;
// 			case  9: motor->tmp.TIMEOUT  = y.u_val; motor->tmp.read_flag = 11; break;
// 			case 10: motor->tmp.cmode    = y.u_val; motor->tmp.read_flag = 12; break;
// 			case 11: motor->tmp.Damp 	 = y.f_val; motor->tmp.read_flag = 13; break;
// 			case 12: motor->tmp.Inertia  = y.f_val; motor->tmp.read_flag = 14; break;
// 			case 13: motor->tmp.hw_ver   = y.u_val; motor->tmp.read_flag = 15; break;
// 			case 14: motor->tmp.sw_ver   = y.u_val; motor->tmp.read_flag = 16; break;
// 			case 15: motor->tmp.SN 	  	 = y.u_val; motor->tmp.read_flag = 17; break;
// 			case 16: motor->tmp.NPP 	 = y.u_val; motor->tmp.read_flag = 18; break;
// 			case 17: motor->tmp.Rs 	  	 = y.f_val; motor->tmp.read_flag = 19; break;
// 			case 18: motor->tmp.Ls 	  	 = y.f_val; motor->tmp.read_flag = 20; break;
// 			case 19: motor->tmp.Flux 	 = y.f_val; motor->tmp.read_flag = 21; break;
// 			case 20: motor->tmp.Gr 	  	 = y.f_val; motor->tmp.read_flag = 22; break;
// 			case 21: motor->tmp.PMAX 	 = y.f_val; motor->tmp.read_flag = 23; break;
// 			case 22: motor->tmp.VMAX 	 = y.f_val; motor->tmp.read_flag = 24; break;
// 			case 23: motor->tmp.TMAX 	 = y.f_val; motor->tmp.read_flag = 25; break;
// 			case 24: motor->tmp.I_BW 	 = y.f_val; motor->tmp.read_flag = 26; break;
// 			case 25: motor->tmp.KP_ASR   = y.f_val; motor->tmp.read_flag = 27; break;
// 			case 26: motor->tmp.KI_ASR   = y.f_val; motor->tmp.read_flag = 28; break;
// 			case 27: motor->tmp.KP_APR   = y.f_val; motor->tmp.read_flag = 29; break;
// 			case 28: motor->tmp.KI_APR   = y.f_val; motor->tmp.read_flag = 30; break;
// 			case 29: motor->tmp.OV_Value = y.f_val; motor->tmp.read_flag = 31; break;
// 			case 30: motor->tmp.GREF 	 = y.f_val; motor->tmp.read_flag = 32; break;
// 			case 31: motor->tmp.Deta 	 = y.f_val; motor->tmp.read_flag = 33; break;
// 			case 32: motor->tmp.V_BW 	 = y.f_val; motor->tmp.read_flag = 34; break;
// 			case 33: motor->tmp.IQ_cl 	 = y.f_val; motor->tmp.read_flag = 35; break;
// 			case 34: motor->tmp.VL_cl 	 = y.f_val; motor->tmp.read_flag = 36; break;
// 			case 35: motor->tmp.can_br   = y.u_val; motor->tmp.read_flag = 37; break;
// 			case 36: motor->tmp.sub_ver  = y.u_val; motor->tmp.read_flag = 38; break;
// 			case 50: motor->tmp.u_off 	 = y.f_val; motor->tmp.read_flag = 39; break;
// 			case 51: motor->tmp.v_off 	 = y.f_val; motor->tmp.read_flag = 40; break;
// 			case 52: motor->tmp.k1 		 = y.f_val; motor->tmp.read_flag = 41; break;
// 			case 53: motor->tmp.k2		 = y.f_val; motor->tmp.read_flag = 42; break;
// 			case 54: motor->tmp.m_off 	 = y.f_val; motor->tmp.read_flag = 43; break;
// 			case 55: motor->tmp.dir 	 = y.f_val; motor->tmp.read_flag = 44; break;
// 			case 80: motor->tmp.p_m 	 = y.f_val; motor->tmp.read_flag = 45; break;
// 			case 81: motor->tmp.x_out 	 = y.f_val; motor->tmp.read_flag = 0 ; break;
// 		}
// 	}
// }

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
// void can1_rx_callback(void)
// {
// 	uint16_t rec_id;
// 	uint8_t rx_data[8] = {0};
// 	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
// 	switch (rec_id)
// 	{
// 		case 0x11: dm_motor_fbdata(&motor[Motor1], rx_data); receive_motor_data(&motor[Motor1], rx_data); break;
// 	}
// }