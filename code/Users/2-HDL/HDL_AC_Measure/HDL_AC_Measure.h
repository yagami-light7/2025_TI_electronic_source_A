/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_AC_Measure.h
  * @brief      驱动ADC实现交流电压电流测量
  * @note       Hardware Driver Layer硬件驱动层
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
#include "MWL_Toolbox.h"
#include "HAL_HRTIM.h"
#include "arm_math.h"

/**
 * @brief 宏定义
 */
#define BUF_LENRTH 200

/**
 * @brief 结构体
 */
typedef struct
{
    ADC_Manage_Object_t *adc;
    first_order_filter_type_t *un_filter[3];
    float un[3];        // 依次存放abc三相的交流相电压瞬时值
    float il[6];        // 依次存放abc三相的交流线电流瞬时值
    float un_rms;       // 依次存放abc三相的交流相电压有效值
    float il_rms[3];       // 依次存放abc三相的交流线电流有效值

    // 线电压
    float Uab;
    float Ubc;
    float Uca;

    float ul_rms;       //  线电压有效值

    // 缓冲区存放采集数据
    float un_buf[BUF_LENRTH];
    float a_il_buf[BUF_LENRTH];
    float b_il_buf[BUF_LENRTH];
    float c_il_buf[BUF_LENRTH];

}AC_Measure_t;

/**
 * @brief 变量外部声明
 */
extern AC_Measure_t AC_Measure;

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
* @brief          逆变 根据ADC采样数据计算交流电压电流
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
extern void DC_AC_Calc(AC_Measure_t *ac);

/**
* @brief          整流 根据ADC采样数据计算交流电压电流
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
extern void AC_DC_Calc(AC_Measure_t *ac);

/**
* @brief          初始化ADC使用的低通滤波器
* @note           初始化时调用
* @retval         none
*/
extern void ADC_LowPass_Filter_Init();

#ifdef __cplusplus
}
#endif
