/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_TIM.h
  * @brief      驱动定时器
  * @note       Hardware Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-13-2025     Light            1. done
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


/**
 * @brief 宏定义
 */
#define SPWM_FREQ_KHZ   0.05        // PWM载波频率（HRTIM频率）[kHz]
#define SPWM_FREQ_HZ   SPWM_FREQ_KHZ * 1000.0f        // PWM载波频率（HRTIM频率）[Hz]
#define USART1_FREQ_KHZ 1        //USART1的发送频率[kHz]
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

void HAL_TIM_Init(void);

#ifdef __cplusplus
}
#endif
