/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_PR.cpp
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

/**
 * @brief 头文件
 */
#include <cmath>
#include "MWL_PR.h"
#include "arm_math.h"

PR PR_DC_AC[3]; // 逆变外环
PR PR_AC_DC[3]; // 整流内环

PR_Controller_t PR_PWM_Rectifier;


/**
 * @brief  初始化准PR控制器
 * @param  p   控制器对象指针
 * @param  Kp  比例增益
 * @param  Kr  谐振增益
 * @param  Ts  采样周期 (秒)
 * @param  wc  谐振带宽 (rad/s)
 * @param  wo  谐振中心频率 (rad/s)
 */
void PR_init(PR *p,float Kp,float Kr,float Ts,float wc,float wo)
{
    p->Ts = Ts;
    p->Kp = Kp;
    p->Kr = Kr;
    p->wc = wc;
    p->wo = wo;

    float Ts2 = Ts * Ts;
    float temp = 4.0f / Ts2 + 4.0f * wc / Ts + wo * wo;

    // ===== 差分方程离散化系数 (Tustin双线性变换) =====
    // y[n] = -A1*y[n-1] - A2*y[n-2] + B0*x[n] + B1*x[n-1] + B2*x[n-2]
    p->B0 = (4.0f*Kp / Ts2 + 4.0f*wc*(Kp+Kr)/Ts + Kp*wo*wo) / temp;
    p->B1 = (-8.0f*Kp / Ts2 + 2.0f*Kp*wo*wo) / temp;
    p->B2 = (4.0f*Kp / Ts2 - 4.0f*wc*(Kp+Kr)/Ts + Kp*wo*wo) / temp;
    p->A1 = (-8.0f / Ts2 + 2.0f*wo*wo) / temp;
    p->A2 = (4.0f / Ts2 - 4.0f*wc / Ts + wo*wo) / temp;

    // ===== 初始化历史值 =====
    p->vi = p->vi_1 = p->vi_2 = 0.0f;
    p->vo = p->vo_1 = p->vo_2 = 0.0f;
}

/**
 * @brief  PR控制器计算（两个输入）
 * @param  p        控制器对象指针
 * @param  wc       谐振带宽 (rad/s)
 * @return 控制器输出
 */
float PR_calc(PR *p, float set, float fdb)
{
    // 更新输入
    p->vi = set - fdb;

    // 差分方程计算
    // y[n] = -A1*y[n-1] - A2*y[n-2] + B0*x[n] + B1*x[n-1] + B2*x[n-2]
    p->vo = -p->A1 * p->vo_1
            -p->A2 * p->vo_2
            + p->B0 * p->vi
            + p->B1 * p->vi_1
            + p->B2 * p->vi_2;

    // 状态更新
    p->vo_2 = p->vo_1;
    p->vo_1 = p->vo;
    p->vi_2 = p->vi_1;
    p->vi_1 = p->vi;

    return p->vo;
}

/**
 * @brief  PR控制器调整谐振频率
 * @param  p        控制器对象指针
 * @param  set      目标值
 * @param  fdb      反馈值
 * @return 控制器输出
 */
void PR_change(PR *p, float wo)
{
    p->wo = wo;

    float Ts2 = p->Ts * p->Ts;
    float temp = 4.0f / Ts2 + 4.0f * p->wc / p->Ts + wo * wo;

    // ===== 差分方程离散化系数 (Tustin双线性变换) =====
    // y[n] = -A1*y[n-1] - A2*y[n-2] + B0*x[n] + B1*x[n-1] + B2*x[n-2]
    p->B0 = (4.0f * p->Kp / Ts2 + 4.0f * p->wc * (p->Kp + p->Kr) / p->Ts + p->Kp * wo * wo) / temp;
    p->B1 = (-8.0f * p->Kp / Ts2 + 2.0f * p->Kp * wo * wo) / temp;
    p->B2 = (4.0f * p->Kp / Ts2 - 4.0f * p->wc * (p->Kp + p->Kr) / p->Ts + p->Kp * wo * wo) / temp;
    p->A1 = (-8.0f / Ts2 + 2.0f * wo * wo) / temp;
    p->A2 = (4.0f / Ts2 - 4.0f * p->wc / p->Ts + wo * wo) / temp;

    p->vi = p->vi_1 = p->vi_2 = 0.0f;
    p->vo = p->vo_1 = p->vo_2 = 0.0f;

}
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
//void PR_Init(PR_Controller_t *pr, float kp, float kr, float f0, float bw, float Ts, float res_limit, float out_limit)
//{
//    pr->kp = kp;
//    pr->kr = kr;
//    pr->w0 = 2.0f * PI * f0;
//    pr->bw = 2.0f * PI * bw;
//    pr->Ts = Ts;
//
//    pr->x1 = 0.0f;
//    pr->x2 = 0.0f;
//    pr->res_limit = res_limit;
//    pr->out_max = out_limit;
//    pr->out_min = -out_limit;
//}





/**
 * @brief   PR控制器计算
 * @param   pr              控制器指针
 * @param   set             目标输出
 * @param   fdb             参考输出（实际输出）
 * @retval
 */
//float PR_Calc(PR_Controller_t *pr, float set, float fdb)
//{
//    float error = set - fdb;
//
//    // 计算离散状态方程（带阻尼的谐振器）：
//    float x1_dot = pr->x2;
//    float x2_dot = -2.0f * pr->bw * pr->x2 - pr->w0 * pr->w0 * pr->x1 + pr->kr * error;
//
//    pr->x1 += x1_dot * pr->Ts;
//    pr->x2 += x2_dot * pr->Ts;
//
//    // 谐振器限幅
//    if (pr->x1 > pr->res_limit) pr->x1 = pr->res_limit;
//    if (pr->x1 < -pr->res_limit) pr->x1 = -pr->res_limit;
//
//    // 输出 = 比例项 + 谐振项
//    float out = pr->kp * error + pr->x1;
//
//    // 输出限幅
//    if (out > pr->out_max) out = pr->out_max;
//    if (out < pr->out_min) out = pr->out_min;
//
//    return out;
//}


///**
// * @brief 初始化PR控制器
// *
// * @param ctrl       控制器指针
// * @param kp         比例增益
// * @param kr         谐振增益
// * @param freq       谐振频率 (Hz)
// * @param sample_time 采样时间 (秒)
// * @param min        输出最小值
// * @param max        输出最大值
// */
//void PRController_Init(PRController *ctrl, float kp, float kr, float freq,
//                       float sample_time, float min, float max)
//{
//    ctrl->kp = kp;
//    ctrl->kr = kr;
//    ctrl->omega_r = 2 * PI * freq;  // 转换为rad/s
//    ctrl->sample_time = sample_time;
//
//    // 初始化状态变量
//    ctrl->prev_error = 0.0f;
//    ctrl->prev_output = 0.0f;
//    ctrl->integral = 0.0f;
//
//    // 预计算常数项
//    float T = sample_time;
//    float w0 = ctrl->omega_r;
//
//    // 离散化系数 (使用双线性变换)
//    ctrl->cos_term = arm_cos_f32(w0 * T);
//    ctrl->sin_term = arm_sin_f32(w0 * T);
//
//    // 限幅设置
//    ctrl->output_min = min;
//    ctrl->output_max = max;
//
//    // 自适应频率初始值
//    ctrl->omega_adapt = w0;
//}
//
///**
// * @brief 更新谐振频率 (用于电网频率变化)
// *
// * @param ctrl 控制器指针
// * @param freq 新的谐振频率 (Hz)
// */
//void PRController_UpdateFrequency(PRController *ctrl, float freq)
//{
//    ctrl->omega_r = 2 * PI * freq;
//    float T = ctrl->sample_time;
//    ctrl->cos_term = arm_cos_f32(ctrl->omega_r * T);
//    ctrl->sin_term = arm_sin_f32(ctrl->omega_r * T);
//    ctrl->omega_adapt = ctrl->omega_r;  // 更新自适应频率
//}
//
//
///**
// * @brief 执行PR控制计算
// *
// * @param ctrl 控制器指针
// * @param setpoint 设定值
// * @param feedback 反馈值
// * @return float 控制器输出
// */
//float PRController_Compute(PRController *ctrl, float setpoint, float feedback)
//{
//    // 计算当前误差
//    float error = setpoint - feedback;
//
//    // 比例项
//    float proportional = ctrl->kp * error;
//
//    // 谐振项 (使用二阶积分器实现)
//    float T = ctrl->sample_time;
//    float w0 = ctrl->omega_adapt;  // 使用自适应频率
//
//    // 离散实现 (Tustin变换)
//    float integral_new = ctrl->integral +
//                         (T/2) * (error + ctrl->prev_error) * ctrl->kr;
//
//    // 谐振输出计算
//    float resonant = 2 * w0 * integral_new * ctrl->sin_term;
//
//    // 完整输出
//    float output = proportional + resonant;
//
//    // 输出限幅
//    if (output > ctrl->output_max)
//    {
//        output = ctrl->output_max;
//        // 抗饱和处理
//    }
//    else if (output < ctrl->output_min)
//    {
//        output = ctrl->output_min;
//        // 抗饱和处理
//    }
//    else
//    {
//        // 更新积分状态
//        ctrl->integral = integral_new;
//    }
//
//    // 更新状态
//    ctrl->prev_error = error;
//    ctrl->prev_output = output;
//
//    return output;
//}
//
///**
// * @brief 自适应频率更新 (用于电网频率跟踪)
// *
// * @param ctrl 控制器指针
// * @param measured_freq 测量的电网频率 (Hz)
// * @param alpha 滤波系数 (0.0-1.0)
// */
//void PRController_AdaptFrequency(PRController *ctrl, float measured_freq, float alpha)
//{
//    float target_omega = 2 * PI * measured_freq;
//
//    // 低通滤波更新自适应频率
//    ctrl->omega_adapt = alpha * ctrl->omega_adapt + (1 - alpha) * target_omega;
//
//    // 如果频率变化超过阈值，更新谐振频率
//    if (fabsf(ctrl->omega_adapt - ctrl->omega_r) > (0.02f * 2 * PI))
//    {
//        PRController_UpdateFrequency(ctrl, measured_freq);
//    }
//}
//
/////**
//// * @brief   PR控制器初始化
//// * @param   pr      控制器参数结构体指针
//// * @param   kp      比例参数
//// * @param   kr      谐振参数
//// * @param   f0      谐振频率
//// * @param   Ts      采样间隔
//// * @param   max     输出最大值
//// * @retval  none
//// */
////void PR_Init(PR_Control_t *pr, float kp, float kr, float f0, float Ts, float max)
////{
////    // 清除状态
////    pr->error[0] = pr->error[1] = pr->error[2] = 0.0f;
////    pr->out_prev = 0.0f;
////
////    pr->Kp = kp;
////    pr->Kr = kr;
////    pr->w0 = 2.0f * 3.1415926f * f0;
////    pr->Ts = Ts;
////
////    float w0 = pr->w0;
////    float Ts2 = Ts * Ts;
////
////    // 预计算离散化后的系数（双线性变换）
////    float alpha = 4.0f + Ts2 * w0 * w0;
////    pr->a1 = (2.0f * pr->Kr * Ts * w0 * w0 + 8.0f * pr->Kp) / alpha;
////    pr->a2 = (2.0f * pr->Kr * Ts * w0 * w0 - 8.0f * pr->Kp) / alpha;
////    pr->a3 = (-4.0f + Ts2 * w0 * w0) / alpha;
////
////    pr->out_max = max;
////    pr->out_min = -max;
////}
////
////
/////**
//// * @brief   PR控制器计算
//// * @param   pr       控制器参数结构体指针
//// * @param   target   目标值
//// * @param   feedback 实际反馈值
//// * @retval  控制器输出
//// */
////float PR_Calc(PR_Control_t *pr, float target, float feedback)
////{
////    float error = target - feedback;
////
////    // 移动误差历史
////    pr->error[2] = pr->error[1];
////    pr->error[1] = pr->error[0];
////    pr->error[0] = error;
////
////    // 输出计算
////    float out = pr->out_prev
////                + pr->a1 * pr->error[0]
////                + pr->a2 * pr->error[1]
////                + pr->a3 * pr->error[2];
////
////    // 输出限幅
////    if (out > pr->out_max)
////        out = pr->out_max;
////    else if (out < pr->out_min)
////        out = pr->out_min;
////
////    pr->out_prev = out;
////
////    return out;
////}
////
////
