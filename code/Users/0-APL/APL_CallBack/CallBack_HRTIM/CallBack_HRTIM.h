/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       CallBack_HRTIM.h
  * @brief      高分辨率定时器hrtim中断回调函数
  * @note       Application Layer应用层
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
#include "HAL_HRTIM.h"
#include "HDL_AC_Measure.h"
#include "MWL_PR.h"
#include "MWL_PID.h"
#include "HDL_PWM_Modulation.h"
#include "HDL_bearADC_Measure.h"
#include "HDL_PLL_Function.h"
#include "HAL_HRTIM.h"
#include "HAL_TIM.h"
#include "arm_math.h"
#include "MWL_Toolbox.h"
#include "CallBack_ADC.h"

/**
 * @brief 宏定义
 */
#define DC_BUS_VOLTAGE      60.0f           // 三相逆变输入的直流母线电压
#define LL_RMS_SET          32.0f           // 三相线电压有效值
#define PHASE_V_AMPLITUDE   45.254f         // 相电压幅值--->马鞍波形会有改变？
#define Line_I_AMPLITUDE    1.0f * 1.414f         // 线电流幅值（留有一定富裕）2 * Sqrt(2.0)
#define PR_OUT_MAX          0.45f
#define LINE_VOLT_RMS_MAX     DC_BUS_VOLTAGE / 2.0f / Sqrt(2) * MODULATION_INDEX * 2.0f * Sqrt(3)    // 线电压最大有效值
#define PHASE_VOLT_RMS_MAX    DC_BUS_VOLTAGE / 2.0f / Sqrt(2) * MODULATION_INDEX * 2.0f    // 相电压最大有效值
#define PHASE_VOLT_MAX        DC_BUS_VOLTAGE / 2.0f * MODULATION_INDEX * 2.0f    // 相电压最大瞬时值

/**
 * @brief 结构体
 */
typedef struct
{
    AC_Measure_t *AC_Measure;           // ADC 测量
    PR *PR_V_Control[3];                // PR电压外环控制结构体
    PidTypeDef_t *PID_I_Control[3];      // P电流内环控制结构体
    PidTypeDef_t *Volt_PID;

    // 依次存放三相abc
    float un_rms_target;            // 目标相电压
    float in_target[3];             // 目标相电流
}
Three_Phase_Inverter_t;

typedef struct
{
    AC_Measure_t *AC_Measure;           // ADC 测量
    PR *PR_I_Control[3];                // PR电流外环控制结构体
    PidTypeDef_t *PID_V_Control[3];     // P电压内环控制结构体
    PidTypeDef_t *Curr_PID;

    // 依次存放三相abc
    float un_rms_target;            // 目标相电压
    float in_target[3];             // 目标相电流
}
Three_Phase_Rectifier_t;

typedef struct
{
    bearAC_Measure_t *bearAC_Measure;       // ADC 测量
    PR_Controller_t *PR_Control;     // PR控制结构体

    //目标输出直流
    float dc_out_target[2];             // 目标直流电压/电流
}
PWMRectifier_t;


/**
 * @brief 变量外部声明
 */
extern Three_Phase_Inverter_t Three_Phase_Inverter;

extern Three_Phase_Rectifier_t Three_Phase_Rectifier;

extern PWMRectifier_t PWMRectifier;

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
* @brief   用于三相逆变的PR控制器初始化
* @note    此代码会在初始化时被调用
*/
extern void Controller_Init(void);


#ifdef __cplusplus
}
#endif
