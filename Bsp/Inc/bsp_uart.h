//
// Created by sy on 2025/12/15.
//

#ifndef PARALLEL_LEG_CHASSIS_BSP_UART_H
#define PARALLEL_LEG_CHASSIS_BSP_UART_H

#include "main.h"
#include "usart.h"

extern uint8_t UART1_Tx_Data[256];
extern uint8_t UART1_Rx_Buff[256];

typedef void (*UART_Call_Back)();//函数指针，用于调用函数

typedef struct
{
    UART_HandleTypeDef *UART_Handler;
    uint8_t *Rx_Buffer;
    uint16_t Rx_Buffer_Size;
    UART_Call_Back Callback_Function;
}Struct_UART_Manage_Object;

void bsp_uart_init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function);
void bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

#endif //PARALLEL_LEG_CHASSIS_BSP_UART_H