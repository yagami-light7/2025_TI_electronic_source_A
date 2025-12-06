/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       CallBack_ADC.h
  * @brief      ADC中断回调函数，在此处完成计算与闭环控制
  * @note       Application Layer应用层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-28-2025     Light            1. done
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
#include "CallBack_HRTIM.h"

/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */
extern uint8_t PR_flag; // 为1时，PR正在进行运算

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


#ifdef __cplusplus
}
#endif
