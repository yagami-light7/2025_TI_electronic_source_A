/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_Data_Utils.cpp
  * @brief      本文件提供底层通用数据处理模块
  * @note       Middleware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-28-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "MWL_Data_Utils.h"

/**
 * @brief          将uint8_t数组转化为float数组
 *
 * @param[in]      uint8_data   需要转化的uint8_t数组
 *                 float_data   转化后的float数组
 *                 num_floats   float数组元素个数
 *
 * @retval         none
 */
void uint8_to_float(uint8_t *uint8_data, float *float_data, int num_floats)
{
    for (int i = 0; i < num_floats; i++)
    {
        float_data[i] = *( (float *)(uint8_data + i * sizeof(float)) );
    }
}

/**
 * @brief          查表快速求sin值
 *
 * @param[in]      theta: angle to be calculated
 *
 * @retval         对应角度的sin值
 */
float fast_sin(float theta)
{
    /* congruence of angle theta to 2pi */
    while (1)
    {
        if (theta > 6.2831854f && theta > 0)
            theta = theta - 6.2831854f;
        else if (theta < 0)
            theta = theta + 6.2831854f;
        else
            break;
    }
    /* look up the table to obtain the sine value */
    return sin_tab[(int) (81.4873308f * theta)];
}
