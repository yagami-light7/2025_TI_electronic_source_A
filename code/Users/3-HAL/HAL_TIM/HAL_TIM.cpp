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

/**
 * @brief 头文件
 */
#include "HAL_TIM.h"
#include "tim.h"
#include "MWL_Data_Utils.h"


/**
 * @brief          定时器初始化
 */
void HAL_TIM_Init(void)
{
    /* TIM2 */
//    TIM2->PSC = 0;
//    // TIM2->ARR = (uint32_t)( 170e3 / (SIN_POINTS * SPWM_FREQ_KHZ) ) - 1;    // 170MHZ
//    TIM2->ARR = (uint32_t)( 170e3 / (SIN_POINTS * USART1_FREQ_KHZ) ) - 1;
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);
}
