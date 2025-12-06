/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_PWM_Modulation.cpp
  * @brief      驱动高分辨率定时器生成SPWM信号
  * @note       Hardware Driver Layer硬件驱动层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-12-2025     Light            1. done
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
#include "HAL_HRTIM.h"
#include "MWL_Data_Utils.h"
#include "arm_math.h"

/**
 * @brief 宏定义
 */
// 占空比范围  11.25~86.25       对应调制度不超过75%
#define MODULATION_INDEX modulation / 2.0f
#define SPWM_OFFSET 0.1125f / (MODULATION_INDEX) + 1


/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */
extern float wt_step; // 调频步长

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
* @brief          单相逆变PWM波调制 SPWM波
* @note           此函数会在TIMCallBack中被循环调用
* @retval         none
*/
extern void Signle_Phase_Inverter(void);

/**
* @brief          三相逆变PWM波调制 初步尝试SPWM波 之后尝试更换为SVPWM波
* @note           此函数会在TIMCallBack中被循环调用
* @retval         none
*/
extern void Three_Phase_Inverter_Update(void);

/**
* @brief          开环调制 前馈占空比获取
* @param          *duty 以指针形式获取前馈占空比
* @param          *sin_theta 以指针形式获取相位角
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
extern void Three_Phase_Inverter_GetOpenDuty(float *duty, float *sin_theta);

/**
* @brief          生成参考波形并且调频
* @param          *duty 以指针形式获取未经调制的前馈占空比
* @param          *sin_theta 以指针形式获取相位角
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
extern void DC_AC_Adjust(float *sin_theta);

/**
* @brief          通过调整wt步长实现调频
* @param          f0    期望频率
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
extern void DC_AC_FrequencyAdjust(float f0);

#ifdef __cplusplus
}
#endif
