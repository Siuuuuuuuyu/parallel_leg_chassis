//
// Created by sy on 2025/12/15.
//
#include "bsp_uart.h"

Struct_UART_Manage_Object UART1_Manage_Object;

uint8_t UART1_Tx_Data[256] __attribute__((section(".uart_section")));
uint8_t UART1_Rx_Buff[256] __attribute__((section(".uart_section")));

void bsp_uart_init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART1_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, Rx_Buffer_Size);
}

void bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    HAL_UART_Transmit_DMA(huart, Data, Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Callback_Function();
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Size);
    }
}

