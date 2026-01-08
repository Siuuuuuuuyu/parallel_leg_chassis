//
// Created by sy on 2025/12/14.
//

#ifndef PARALLEL_LEG_CHASSIS_BSP_CAN_H
#define PARALLEL_LEG_CHASSIS_BSP_CAN_H

#define hcan_t FDCAN_HandleTypeDef

#include "main.h"
#include "fdcan.h"
#include "dm_motor_ctrl.h"
#include "dji_motor_ctrl.h"

void bsp_can_init(void);
void can_filter_init(void);
uint8_t bsp_fdcan_send(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t bsp_fdcan_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
// void fdcan3_rx_callback(void);

#endif //PARALLEL_LEG_CHASSIS_BSP_CAN_H