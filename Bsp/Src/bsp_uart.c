//
// Created by sy on 2025/12/15.
//
#include "bsp_uart.h"

// uart发送缓冲区
/*0x24002CC6*/
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 0))) uint8_t uart1_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 1))) static uint8_t uart2_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 2))) static uint8_t uart3_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 3))) static uint8_t uart4_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 4))) static uint8_t uart5_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 5))) static uint8_t uart6_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 6))) static uint8_t uart7_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 7))) static uint8_t uart8_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 8))) static uint8_t uart9_tx_buf[UART_TX_BUF_LEN] = {0};
// __attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 9))) static uint8_t uart10_tx_buf[UART_TX_BUF_LEN] = {0};

// uart接收缓冲区
uint8_t uart1_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart2_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart3_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart4_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart5_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart6_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart7_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart8_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart9_rx_buf[UART_RX_BUF_LEN] = {0};
// static uint8_t uart10_rx_buf[UART_RX_BUF_LEN] = {0};

uart_tx_manager_t uart1_tx_manager;

// bsp_uart初始化
void bsp_uart_init(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_buf, UART_RX_BUF_LEN);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_rx_buf, UART_RX_BUF_LEN);
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf, UART_RX_BUF_LEN);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart7, uart7_rx_buf, UART_RX_BUF_LEN);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart10, uart10_rx_buf, UART_RX_BUF_LEN);
}

void uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit_DMA(huart, data, len);
}

UART_TX_STATE uart1_add_package(uint8_t data[], int len)
{
    if (uart1_tx_manager.package_index >= UART_TX_QUEUE_MAX)
        return TX_FULL;
    if (len > UART_TX_BUF_LEN)
        return TX_LEN_ERROR;

    memcpy(uart1_tx_manager.uart_tx_packages[uart1_tx_manager.package_index], data, len);
    uart1_tx_manager.uart_tx_packages_len[uart1_tx_manager.package_index] = len;
    uart1_tx_manager.package_index++;
    return TX_OK;
}

UART_TX_STATE uart1_tx_refresh(void)
{
    if (uart1_tx_manager.uart_tx_state == TX_BUSY)
        return TX_BUSY;
    else
    {
        float dt = DWT_GetDeltaT(&uart1_tx_manager.uart_tx_count);
        if (dt <= UART_TX_MIN_TIMEOUT)
            return TX_BUSY;
        else
        {
            if (uart1_tx_manager.package_index >= 1) //代表有数据
            {
                memcpy(uart1_tx_buf, uart1_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
                uart_send(&huart1, uart1_tx_buf, uart1_tx_manager.uart_tx_packages_len[0]);
                uart1_tx_manager.package_index--;
                uart1_tx_manager.uart_tx_state = TX_BUSY;
                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
                {
                    memcpy(uart1_tx_manager.uart_tx_packages[i], uart1_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
                    uart1_tx_manager.uart_tx_packages_len[i] = uart1_tx_manager.uart_tx_packages_len[i + 1];
                }
                return TX_OK;
            }
            else
                return TX_NO_DATA;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uart1_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
        DWT_GetDeltaT(&uart1_tx_manager.uart_tx_count); //标记时间
    }
    // if (huart == &huart2)
    // {
    //     uart2_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
    //     DWT_GetDeltaT(&uart2_tx_manager.uart_tx_count); //标记时间
    // }
    // if (huart == &huart3)
    // {
    //     uart3_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
    //     DWT_GetDeltaT(&uart3_tx_manager.uart_tx_count); //标记时间
    // }
    // if (huart == &huart5)
    // {
    //     uart5_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
    //     DWT_GetDeltaT(&uart5_tx_manager.uart_tx_count); //标记时间
    // }
    // if (huart == &huart7)
    // {
    //     uart7_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
    //     DWT_GetDeltaT(&uart7_tx_manager.uart_tx_count); //标记时间
    // }
    // if (huart == &huart10)
    // {
    //     uart10_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
    //     DWT_GetDeltaT(&uart10_tx_manager.uart_tx_count); //标记时间
    // }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        const char *msg = "UART1 got data\t\n";
        uart_send(&huart1, (uint8_t *)msg, strlen(msg));
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
    }
}
