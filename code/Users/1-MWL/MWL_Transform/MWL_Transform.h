/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_Transform.h
  * @brief      常见的数学变换 Clark变换 Park变换
  * @note       Middleware_Layer中间件层
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
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */
typedef struct
{
    float alpha;
    float beta;
} ClarkOutput;


/**
 * @brief 变量外部声明
 */


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


#ifdef __cplusplus
}
#endif
