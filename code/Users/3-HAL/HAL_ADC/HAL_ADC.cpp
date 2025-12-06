/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_ADC.cpp
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

/**
 * @brief 头文件
 */
#include "HAL_ADC.h"

/**
 * @brief  ADC管理对象实例化
 */
ADC_Manage_Object_t ADC_Manage_Object = {&hadc1, &hadc2};

/**
* @brief          以DMA方式启动ADC采样，hrtim1外部事件1作为驱动源
 *                ADC采样与hrtim主定时器同步
* @param[in]      none
* @retval         none
*/
static void ADC_Start(void)
{
    HAL_ADC_Start_DMA(ADC_Manage_Object.hadc1, (uint32_t *)ADC_Manage_Object.adc1_samp_buf, ADC1_CHANNEL_NUM);
    HAL_ADC_Start_DMA(ADC_Manage_Object.hadc2, (uint32_t *)ADC_Manage_Object.adc2_samp_buf, ADC2_CHANNEL_NUM);
}

/**
* @brief          ADC初始化 进行采样前的校准并启动采样
* @param[in]      none
* @retval         none
*/
void ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(ADC_Manage_Object.hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(ADC_Manage_Object.hadc2, ADC_SINGLE_ENDED);
    ADC_Start();
}