/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       CallBack_ADC.cpp
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

/**
 * @brief 头文件
 */
#include "CallBack_ADC.h"

uint8_t softswitch = 0;
uint8_t sw_flag = 0;
uint8_t PR_flag  = 0;

uint16_t f0_test = 50;
uint16_t last_f0_test = 50;

uint8_t B1_State = 1;
uint8_t B1_Last_State = 1;
uint8_t B2_State = 1;
uint8_t B2_Last_State = 1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//    B1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
//    B2_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
//
//    if(B2_State == GPIO_PIN_RESET && B2_Last_State == GPIO_PIN_SET)
//    {
//        f0_test++;
//    }
//
//    if(B2_State == GPIO_PIN_RESET && B2_Last_State == GPIO_PIN_SET)
//    {
//        f0_test--;
//    }

    if(last_f0_test != f0_test && f0_test <= 100 && f0_test >= 20 && PR_flag == 0)
    {
        // 变频
        DC_AC_FrequencyAdjust(f0_test);
        for (int i = 0; i < 3; ++i)
        {
            PR_change(Three_Phase_Inverter.PR_V_Control[i], 6.283f * f0_test);
//            PR_change(Three_Phase_Rectifier.PR_I_Control[i], 6.283f * f0_test);
        }
    }

    last_f0_test = f0_test;

    B1_Last_State = B1_State;
    B2_Last_State = B2_State;
}