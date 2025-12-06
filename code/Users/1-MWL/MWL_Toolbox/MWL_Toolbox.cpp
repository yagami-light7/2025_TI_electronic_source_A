/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_Toolbox. h
  * @brief      提供常用的数学求解工具
  * @note       Middleware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-21-2025     Light            1. done
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
#include "MWL_Toolbox.h"

first_order_filter_type_t un_filter[3];
first_order_filter_type_t ac_in_filter[2];
first_order_filter_type_t dc_out_filter[3];

/**
  * @brief          快速求平方根的倒数
  * @param[in]      num
  * @retval         num的平方根倒数
  */
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          快速求平方根
  * @param[in]      num
  * @retval         num的平方根
  */
float Sqrt(float num)
{
    return 1.0f / invSqrt(num);
}

/**
  * @brief          限幅
  * @param[in]      *num    限幅数据
  * @retval         limit   幅度
  */
void float_limit(float *num, float limit)
{
    if (*num > limit)
        *num = limit;
    else if (*num < -limit)
        *num = -limit;
}

/**
  * @brief          限幅
  * @param[in]      *num    限幅数据
  * @param[in]      max     最大值
  * @param[in]      min     最小值
  */
void float_clamp(float *num, float max, float min)
{
    if (*num > max)
        *num = max;
    else if (*num < min)
        *num = min;
}

/**
  * @brief          一阶低通滤波初始化
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @param[in]      一阶低通滤波结构体
  * @param[in]      原始输入数据
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
            first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

