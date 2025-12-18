//
// Created by sy on 2025/12/14.
//
#include "bsp_can.h"

#include <stdio.h>

void bsp_can_init(void)
{
    can_filter_init();
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
    // HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    // HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准ID
    fdcan_filter.FilterIndex = 0; // 滤波器索引，即使用第一个滤波器
    fdcan_filter.FilterType = FDCAN_FILTER_MASK; // 设置滤波器类型为掩码模式
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
    // 在掩码模式下，FilterID1为期望的ID，FilterID2为掩码，掩码为1的位表示必须匹配，为0的位表示不关心
    // 这里都设为0x00，表示接收所有ID，掩码为0表示不检查任何位
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 接收ID2
    // 拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    // 设置RX FIFO0的水印值为1
    // 当FIFO0中有至少1条消息时，可以触发中断
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
    // HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}

uint8_t bsp_fdcan_send(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID; // 使用标准ID，11位
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧，非远程帧

    if (len <= 8)
        pTxHeader.DataLength = len;
    else if (len == 12)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    else if (len == 16)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    else if (len == 20)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    else if (len == 24)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    else if (len == 32)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    else if (len == 48)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    else if (len == 64)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示器激活
    pTxHeader.BitRateSwitch = FDCAN_BRS_ON; // 是否比特率切换
    pTxHeader.FDFormat = FDCAN_FD_CAN; // 是否使用CAN FD格式
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不生成发送事件
    pTxHeader.MessageMarker = 0;
    // 添加到发送队列
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
    {
        return 1;//发送
    }
    return 0;
}

uint8_t bsp_fdcan_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;
        if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_12)
            len = 12;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_16)
            len = 16;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_20)
            len = 20;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_24)
            len = 24;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_32)
            len = 32;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_48)
            len = 48;
        else if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_64)
            len = 64;

        return len;
    }
    return 0;
}

uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
    bsp_fdcan_receive(&hfdcan1, &rec_id1, rx_data1);
    switch (rec_id1)
    {
        case 0x201:
        {
            M3508_1.rotor_angle =    ((rx_data1[0] << 8) | rx_data1[1]);
            M3508_1.rotor_speed =    ((rx_data1[2] << 8) | rx_data1[3]);
            M3508_1.torque_current = ((rx_data1[4] << 8) | rx_data1[5]);
            M3508_1.temp =           ((rx_data1[6] << 8) | rx_data1[7]);
            break;
        }
        case 0x01:
        {

        }
    }
}

// uint8_t rx_data2[8] = {0};
// uint16_t rec_id2;
// void fdcan2_rx_callback(void)
// {
//     fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
// }

// uint8_t rx_data3[8] = {0};
// uint16_t rec_id3;
// void fdcan3_rx_callback(void)
// {
//     fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
// }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
    {
        fdcan1_rx_callback();
    }
    // if(hfdcan == &hfdcan2)
    // {
    //     fdcan2_rx_callback();
    // }
    // if(hfdcan == &hfdcan3)
    // {
    //     fdcan3_rx_callback();
    // }
}