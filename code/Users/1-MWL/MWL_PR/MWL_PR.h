/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_PR.h
  * @brief      PR控制器模块
  * @note       Middleware_Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-17-2025     Light            1. done
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
#include "MWL_Data_Utils.h"

/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */

typedef struct
{
    float Ts;                // 采样周期 (s)
    float Kp, Kr;            // 比例增益 & 谐振增益
    float wc, wo;            // 谐振带宽 (rad/s) & 谐振中心频率 (rad/s)
    float B0, B1, B2;        // 差分方程分子系数
    float A1, A2;            // 差分方程分母系数(去掉y[n]系数)
    float vi, vi_1, vi_2;    // 输入当前值 & 历史值
    float vo, vo_1, vo_2;    // 输出当前值 & 历史值

} PR;



typedef struct {
    float kp;       // 比例系数
    float kr;       // 谐振增益
    float w0;       // 谐振频率 (rad/s)
    float bw;       // 谐振带宽 (rad/s)
    float Ts;       // 采样周期 (s)

    float x1, x2;   // 状态变量
    float res_limit;  // 谐振器限幅
    float out_max;    // 输出最大值
    float out_min;    // 输出最小值
} PR_Controller_t;


//// PR控制器结构体
//typedef struct {
//    // 控制器参数
//    float kp;           // 比例增益
//    float kr;           // 谐振增益
//    float omega_r;      // 谐振频率 (rad/s)
//    float sample_time;  // 采样时间 (秒)
//
//    // 内部状态变量
//    float prev_error;   // 上一次误差
//    float prev_output;  // 上一次输出
//    float integral;     // 积分项
//    float cos_term;     // 余弦项系数
//    float sin_term;     // 正弦项系数
//
//    // 限幅参数
//    float output_min;   // 输出最小值
//    float output_max;   // 输出最大值
//
//    // 自适应参数
//    float omega_adapt;  // 自适应频率 (用于频率变化场景)
//}PRController;

//
//typedef struct
//{
//    float Kp;   // 比例增益
//    float Kr;   // 谐振增益
//    float wc;   // 截止频率
//    float w0;   // 正弦频率 rad/s
//    float Ts;   // 采样周期 s
//
//    // 内部状态
//    float error[3];
//    float out_prev;
//
//    // 计算用的系数
//    float a1, a2, a3;
//
//    // 限幅
//    float out_max;
//    float out_min;
//
//}PR_Control_t;

/**
 * @brief 变量外部声明
 */
extern PR PR_DC_AC[3];
extern PR PR_AC_DC[3];
extern PR_Controller_t PR_PWM_Rectifier;


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
 * @brief  初始化准PR控制器
 * @param  p   控制器对象指针
 * @param  Kp  比例增益
 * @param  Kr  谐振增益
 * @param  Ts  采样周期 (秒)
 * @param  wc  谐振带宽 (rad/s)
 * @param  wo  谐振中心频率 (rad/s)
 */
extern void PR_init(PR *p, float Kp, float Kr, float Ts, float wc, float wo);


/**
 * @brief  PR控制器计算（两个输入）
 * @param  p        控制器对象指针
 * @param  set      目标值
 * @param  fdb      反馈值
 * @return 控制器输出
 */
extern float PR_calc(PR *p, float set, float fdb);


/**
 * @brief  PR控制器计算（两个输入）
 * @param  p        控制器对象指针
 * @param  wc       谐振带宽 (rad/s)
 * @return 控制器输出
 */
extern void PR_change(PR *p, float wo);


/**
 * @brief 初始化PR控制器
 *
 * @param pr            控制器指针
 * @param kp            比例增益
 * @param kr            谐振增益
 * @param f0            谐振频率 (Hz)
 * @param bw            谐振带宽 (Hz)
 * @param Ts            采样时间 (秒)
 * @param res_limit     谐振项输出限幅
 * @param out_limit     输出限幅
 */
extern void PR_Init(PR_Controller_t *pr, float kp, float kr, float f0, float bw, float Ts, float res_limit, float out_limit);

/**
 * @brief   PR控制器计算
 * @param   pr              控制器指针
 * @param   set             目标输出
 * @param   fdb             参考输出（实际输出）
 * @retval
 */
extern float PR_Calc(PR_Controller_t *pr, float set, float fdb);



#ifdef __cplusplus
}
#endif
