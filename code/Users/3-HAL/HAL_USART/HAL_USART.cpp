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
#include "HAL_USART.h"

/**
 * @brief  UART管理对象实例化
 */
UART_Manage_Object_t UART1_Manage_Object  = {&huart1};
UART_Manage_Object_t UART2_Manage_Object = {&huart2};
/**
 * @brief          UART串口初始化
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 rx_data_size         接收数据长度
 * @retval         none
 */
void UART_Init(UART_Manage_Object_t *UART_Manage_Object, uint16_t rx_data_size)
{
    UART_Manage_Object->rx_data_size = rx_data_size;
    HAL_UARTEx_ReceiveToIdle_DMA(UART_Manage_Object->huart, UART_Manage_Object->rx_buffer, rx_data_size);
}


/**
 * @brief          UART发送数据
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 data                 数据
 *                 length               数据长度
 * @retval         发送结果
 */
bool UART_Send_Data(UART_Manage_Object_t *UART_Manage_Object, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(UART_Manage_Object->huart, data, length));
}