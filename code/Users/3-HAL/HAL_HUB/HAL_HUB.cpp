/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_HUB.h
  * @brief      硬件抽象层总中心 驱动板级支持包
  * @note       Hardware Abstraction Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-24-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "HAL_HUB.h"

/**
* @brief          HAL层初始化
* @param[in]      none
* @retval         none
*/
void HAL_BSP_Init()
{
    HRTIM_Init();

    ADC_Init();

    HAL_TIM_Init();

    UART_screen_init();
}