#pragma once

/**
 * @brief 头文件
 */
#include "HAL_USART.h"

/**
 * @brief 宏定义
 */
/* 串口屏使用的串口 */
#define UART_screen           huart2
#define UART_screen_DMA_RX    hdma_usart2_rx
#define UART_screen_DMA_TX    hdma_usart2_tx


/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */
extern UART_Manage_Object_t UART2_Manage_Object;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
/**
 * @brief CPP部分
 */
#ifdef __cplusplus

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


extern void UART_screen_init(void);



#ifdef __cplusplus
}
#endif
