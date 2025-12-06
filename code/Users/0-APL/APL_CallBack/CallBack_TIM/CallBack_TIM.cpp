/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       CallBack_TIM.c
  * @brief      定时器中断回调函数 主要实现调制部分
  * @note       Application Layer应用层
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

/**
 * @brief 头文件
 */
#include "CallBack_TIM.h"
#include <cstdio>
#include <HDL_PLL_Function.h>



/**
 * @brief   定时中断回调函数
 * @note    开环调制部分实现
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) // 周期：1 / (SPWM频率 * SIN点数)
    {

        // 利用查表法生成SPWM波
//        Signle_Phase_Inverter();
//        Three_Phase_Inverter_Update();
        // sprintf((char*)UART1_Manage_Object.rx_buffer,"PLL:%.2f,%.2f,%.2f\n",SOGI_U.ui0,SOGI_U.ua0,SOGI_U.ub0);
        // UART_Send_Data(&UART1_Manage_Object,UART1_Manage_Object.rx_buffer,strlen((char*)UART1_Manage_Object.rx_buffer));
    }

}