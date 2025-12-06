/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_ADC.h
  * @brief      驱动ADC
  * @note       Hardware Abstract Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-23-2025     Light            1. done
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
#include "adc.h"

/**
 * @brief 宏定义
 */
#define ADC1_CHANNEL_NUM 7
#define ADC2_CHANNEL_NUM 2

/**
 * @brief 结构体
 */
typedef struct
{
    // 句柄
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;

    // 采样原始数据
    uint16_t adc1_samp_buf[ADC1_CHANNEL_NUM];
    uint16_t adc2_samp_buf[ADC2_CHANNEL_NUM];

    // 转化后的交流电压/电流
    float volt_buf[3];
    float curr_buf[6];

    // 滤波完毕数据
    float volt_filter_buf[ADC1_CHANNEL_NUM + ADC2_CHANNEL_NUM];
    float curr_filter_buf[ADC2_CHANNEL_NUM + ADC2_CHANNEL_NUM];


}ADC_Manage_Object_t;   // adc对象结构体

/**
 * @brief 变量外部声明
 */
extern ADC_Manage_Object_t ADC_Manage_Object;

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
* @brief          ADC初始化 进行采样前的校准并启动采样
* @param[in]      none
* @retval         none
*/
extern void ADC_Init(void);

#ifdef __cplusplus
}
#endif
