/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Thread_Adjust.cpp
  * @brief      外部调参任务
  * @note       Application Layer应用层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-30-2025     Light            1. done
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
#include "Thread_Adjust.h"
#include "HAL_HUB.h"
#include "CallBack_HRTIM.h"

/**
 * @brief   用户调整线程 实现变频等参数调整
 */
void Thread_Adjust(void *argument)
{
    // 初始化控制器
    Controller_Init();

    ADC_LowPass_Filter_Init();

    bearADC_LowPass_Filter_Init();

    PLL_sys_init();

    // 初始化BSP层 启动能量回馈系统
    HAL_BSP_Init();

    while(1)
    {
        vTaskDelay(1);
    }
}


