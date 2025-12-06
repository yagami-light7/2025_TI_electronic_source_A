/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       			MWL_PLL.h
  * @brief      			锁相环相关算法：二阶广义积分器、低通滤波器、PR控制器
  * @note       			Middleware_Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-26-2025     kafeizizi            1. done
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

#include "MWL_Data_Utils.h"//数学库

/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */

typedef struct
{
    float x, y;
    float a1, a2;
    float b0, b2;
    float qb0, qb1, qb2;
    float ui0, ui1, ui2;    //ui:二阶广义积分器的输入值，ui0:v(n),ui1:v(n-1),ui2:v(n-2)该时刻输入/上一时刻输入/上上一时刻输入
    float ua0, ua1, ua2;    //α：ua:二阶广义积分器的输出值，ua0:v(n),ua1:v(n-1),ua2:v(n-2)该时刻输出/上一时刻输出/上上一时刻输出
    float ub0, ub1, ub2;    //β：ub:二阶广义积分器的输出值，ub0:v(n),ub1:v(n-1),ub2:v(n-2)该时刻输出/上一时刻输出/上上一时刻输出
} SOGI_TypeDef;

typedef struct
{
    float x0, x1,x2;
    float y0, y1,y2;

} PR_TypeDef;
typedef struct
{
    float error_wt;
    float ActivePower;
    float ReactivePower;
    float n,m; //下垂控制系数
    float Vref,PQ_wt,PQ_wast_wt,PQ_v;
    float pi_out;

} Droop_TypeDef;
typedef struct
{
    float a,b,c;
    float ui0,ui1;
    float uo0,uo1;
} LP_filter_TypeDef;

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

float MyFmod(float _X, float _Y);


void SOGI_Parameter_Init(SOGI_TypeDef *Pamer, float k, float Ts, float w);//SOGI变换初始化
void SOGI_transfrom(SOGI_TypeDef *Pamer, float in);//SOGI变换
void park_f32(SOGI_TypeDef *Pamer,float sinVal,float cosVal);//park变换函数

void Low_pass_filter_init(LP_filter_TypeDef *Pamer,float A,float B,float C);  //低通滤波器
void Low_pass_filter(LP_filter_TypeDef *Pamer ,float in);  //低通滤波器

float f32_PR_Calculate(PR_TypeDef *PR_Pamer ,float error );
//void PR_init(PR_TypeDef *PR_Pamer);


#ifdef __cplusplus
}
#endif