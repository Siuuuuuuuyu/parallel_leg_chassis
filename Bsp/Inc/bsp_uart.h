//
// Created by sy on 2025/12/15.
//

#ifndef PARALLEL_LEG_CHASSIS_BSP_UART_H
#define PARALLEL_LEG_CHASSIS_BSP_UART_H

#include "main.h"
#include "usart.h"
#include <string.h>

extern uint8_t dbus_rx_buff[25];

typedef struct
{
    uint16_t online;

    struct
    {
        int16_t ch[10];
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;
        int16_t left_hori;
        int16_t right_vert;
        int16_t right_hori;
    } joy;

    struct
    {
        /* VAR VALUE */
        float a;
        float b;
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
        uint8_t e;
        uint8_t f;
        uint8_t g;
        uint8_t h;
    } key;
} remoter_t;

typedef void (*UART_Call_Back)(); // 函数指针，用于调用函数
typedef struct
{
    UART_HandleTypeDef *UART_Handler;
    uint8_t *Rx_Buffer;
    uint16_t Rx_Buffer_Size;
    UART_Call_Back Callback_Function;
}UART_Manage_Object;

void bsp_uart_init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function);
void bsp_uart_send_data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);
void sbus_frame_parse(remoter_t *remoter, uint8_t *buf);

#endif //PARALLEL_LEG_CHASSIS_BSP_UART_H