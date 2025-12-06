/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_USART.cpp
  * @brief      USART外设再封装
  * @note       Hardware Abstract Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-29-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考中科大2024年工程机器人电控代码开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#pragma once

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#define UART_TX_BUFFER_SIZE     256     // 串口TX缓冲区字节长度
#define UART_RX_BUFFER_SIZE     512     // 串口RX缓冲区字节长度
#define QUEUE_RX_ITEM_NUM       3       // 接收队列中数据段数目_注意不要超出堆栈configTOTAL_HEAP_SIZE

#define CUSTOM_ROBOT_DATA_SIZE  30
#define REMOTE_ROBOT_DATA_SIZE  12
#define VT_RC_ROBOT_DATA_SIZE   21

/**
 * @brief UART处理结构体
 */
typedef struct
{
    UART_HandleTypeDef * huart;
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    uint8_t rx_buffer[4];
    uint16_t rx_data_size;
}UART_Manage_Object_t;

/**
 * @brief 变量外部声明
 */
extern UART_Manage_Object_t UART1_Manage_Object;


#ifdef __cplusplus

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief          UART串口初始化
* @param[in]      UART_Manage_Object   串口实例对象
*                 rx_data_size         接收数据长度
* @retval         none
*/
extern void UART_Init(UART_Manage_Object_t *UART_Manage_Object, uint16_t rx_data_size);


/**
 * @brief          UART发送数据
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 data                 数据
 *                 length               数据长度
 * @retval         发送结果
 */
extern bool UART_Send_Data(UART_Manage_Object_t *UART_Manage_Object, uint8_t *data, uint16_t length);


#ifdef __cplusplus
}
#endif