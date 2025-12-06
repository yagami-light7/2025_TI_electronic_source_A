/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_Transform.cpp
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
#include "MWL_Transform.h"
#include "MWL_Toolbox.h"

/**
  * @brief          快速求平方根的倒数
  * @param[in]      num
  * @retval         num的平方根倒数
  */
ClarkOutput ClarkTransform(float ia, float ib, float ic)
{
    ClarkOutput result;

    // Clarke 变换公式
    result.alpha = (2.0f / 3.0f) * (ia - 0.5f * ib - 0.5f * ic);
    result.beta  = (2.0f / 3.0f) * ((Sqrt(3.0f) / 2.0f) * (ib - ic));

    return result;
}

