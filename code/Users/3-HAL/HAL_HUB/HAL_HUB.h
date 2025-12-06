/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_HUB.h
  * @brief      硬件抽象层总中心
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
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"
#include "HAL_ADC.h"
#include "HAL_HRTIM.h"
#include "HAL_SPI.h"
#include "HAL_TIM.h"
#include "HAL_UART_Screen.h"

/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */


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

/**
* @brief          HAL层初始化
* @param[in]      none
* @retval         none
*/
extern void HAL_BSP_Init();

#ifdef __cplusplus
}
#endif
