//
// Created by sy on 2025/12/15.
//
#include "bsp_uart.h"

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define DBUS_BUFF_SIZE 25

uint8_t dbus_rx_buff[25];
remoter_t remoter;
UART_Manage_Object UART1_Manage_Object;
UART_Manage_Object UART7_Manage_Object;
UART_Manage_Object UART10_Manage_Object;

uint8_t UART1_Tx_Data[256];

void bsp_uart_init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART1_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
    }
}

void bsp_uart_send_data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    HAL_UART_Transmit_DMA(huart, Data, Length);
}

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf) // 解析SBUS数据帧，提取遥控器通道数据
{
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;

    if (buf[23] == 0x0C) // 判断遥控器是否在线
        remoter->online = 0; // 离线
    else
        remoter->online = 1; // 在线

    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == UART5)
    {
        if (Size == 25)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_rx_buff, DBUS_BUFF_SIZE * 2); // 接收完毕后重启
            sbus_frame_parse(&remoter, dbus_rx_buff);
            // memset(rx_buff, 0, BUFF_SIZE);
        }
        else // 接收数据长度大于BUFF_SIZE，错误处理
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_rx_buff, DBUS_BUFF_SIZE * 2); // 接收完毕后重启
            memset(dbus_rx_buff, 0, DBUS_BUFF_SIZE * 2);
        }
    }
    else if (huart->Instance == USART1)
    {

    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_rx_buff, DBUS_BUFF_SIZE*2); // 接收发生错误后重启
        memset(dbus_rx_buff, 0, DBUS_BUFF_SIZE); // 清除接收缓存
    }
}
