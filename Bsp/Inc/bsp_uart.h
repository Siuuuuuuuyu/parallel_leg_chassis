//
// Created by sy on 2025/12/15.
//

#ifndef PARALLEL_LEG_CHASSIS_BSP_UART_H
#define PARALLEL_LEG_CHASSIS_BSP_UART_H

#include "main.h"
#include "usart.h"
#include <string.h>
#include "bsp_dwt.h"

// rx缓冲区长度
#ifndef UART_RX_BUF_LEN
#define UART_RX_BUF_LEN (350)
#endif
// tx缓冲区长度
#ifndef UART_TX_BUF_LEN
#define UART_TX_BUF_LEN (150)
#endif
// uart——tx发送队列最大值
#ifndef UART_TX_QUEUE_MAX
#define UART_TX_QUEUE_MAX (20)
#endif
//两包数据发送的最小时间间隔为0.0005s 防止接收方uart线路没有空闲段，该值一定不能大于refresh循环的频率，否则串口一直繁忙一直无法发送
#ifndef UART_TX_MIN_TIMEOUT
#define UART_TX_MIN_TIMEOUT (0.0005f)
#endif

typedef enum
{
    TX_OK = 0,
    TX_BUSY = 1,
    TX_FULL = 2,
    TX_LEN_ERROR = 3,
    TX_NO_DATA = 4
} UART_TX_STATE;

//发送管理结构体
typedef struct
{
    uint8_t uart_tx_packages[UART_TX_QUEUE_MAX][UART_TX_BUF_LEN];
    int uart_tx_packages_len[UART_TX_QUEUE_MAX];
    uint8_t package_index;
    UART_TX_STATE uart_tx_state;
    uint32_t uart_tx_count; //用于dwt记录时间
} uart_tx_manager_t;

void bsp_uart_init(void);
/******************************* UART1 *******************************/
UART_TX_STATE uart1_add_package(uint8_t data[], int len);
UART_TX_STATE uart1_tx_refresh(void);
void uart1_IRQHandler(void);

#endif //PARALLEL_LEG_CHASSIS_BSP_UART_H