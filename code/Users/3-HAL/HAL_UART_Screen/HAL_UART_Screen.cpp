/**
* @brief 头文件
 */
#include <cstring>
#include "stdio.h"
#include "main.h"
#include "string.h"
#include "HAL_UART_Screen.h"
#include "HAL_USART.h"


/* 串口屏的初始化函数 */
void UART_screen_init()
{
    /* 针对 UART2 的初始化部分 */
    UART_Init(&UART2_Manage_Object,UART_RX_BUFFER_SIZE);
    HAL_UART_Init(&UART_screen);
    HAL_UARTEx_ReceiveToIdle_DMA(&UART_screen, UART2_Manage_Object.rx_buffer, UART_RX_BUFFER_SIZE);   // 启用串口空闲中断
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);        //  关闭传输过半中断
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
}