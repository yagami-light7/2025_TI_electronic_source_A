/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_Toolbox.h
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
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"


/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */
typedef struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

/**
 * @brief 变量外部声明
 */
extern first_order_filter_type_t un_filter[3];
extern first_order_filter_type_t ac_in_filter[2];
extern first_order_filter_type_t dc_out_filter[3];

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
* @brief          快速求平方根的倒数
* @param[in]      num
* @retval         num的平方根倒数
*/
extern float invSqrt(float num);

/**
  * @brief          快速求平方根
  * @param[in]      num
  * @retval         num的平方根
  */
extern float Sqrt(float num);


/**
  * @brief          限幅
  * @param[in]      *num    限幅数据
  * @retval         limit   幅度
  */
extern void float_limit(float *num, float limit);

/**
  * @brief          限幅
  * @param[in]      *num    限幅数据
  * @param[in]      max     最大值
  * @param[in]      min     最小值
  */
extern void float_clamp(float *num, float max, float min);

/**
  * @brief          一阶低通滤波初始化
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);

/**
  * @brief          一阶低通滤波计算
  * @param[in]      一阶低通滤波结构体
  * @param[in]      原始输入数据
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);

#ifdef __cplusplus
}
#endif
