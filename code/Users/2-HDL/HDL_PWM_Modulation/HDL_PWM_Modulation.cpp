/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_PWM_Modulation.cpp
  * @brief      驱动高分辨率定时器生成SPWM信号
  * @note       Hardware Driver Layer硬件驱动层
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
#include <cmath>
#include "HDL_PWM_Modulation.h"

float wt_step = 2 * PI / 500;// 调频步长

/**
* @brief          单相逆变PWM波调制 SPWM波
* @note           此函数会在TIMCallBack中被循环调用
* @retval         none
*/
void Signle_Phase_Inverter(void)
{
    static uint16_t index = 0;

    // 更新占空比
//    float single_phase_duty = (sin_tab[index] + SPWM_OFFSET) * MODULATION_INDEX; // 单相逆变占空比

    // 使用Channel C 与 D 进行单相逆变
//    HRTIM_SetPara(CHC, single_phase_duty); // 有效电平为高
//    HRTIM_SetPara(CHD, single_phase_duty); // 有效电平为低

    index++;
    if(index >= SIN_POINTS)
        index = 0;
}

/**
* @brief          三相逆变PWM波调制 初步尝试SPWM波 之后尝试更换为SVPWM波
* @note           此函数会在TIMCallBack中被循环调用
* @retval         none
*/
//void Three_Phase_Inverter_Update(void)
//{
//    static uint16_t index = 0;
//    const uint16_t points = SIN_POINTS; // sin_tab 点数
//    const uint16_t phase_shift = points / 3; // 约 170 点，对应 120°
//
//    // 计算三个相位（120°相差） 使用逆相序
//    // U相：~ sin(wt)
//    // V相：~ sin(wt + 120°)
//    // W相：~ sin(wt - 120°)
//
//    uint16_t idx_a = index;
//    uint16_t idx_b = (index + phase_shift) % points;
//    uint16_t idx_c = (index + 2 * phase_shift) % points;
//
//    // 计算占空比：偏移+调制度
//    float u_phase_duty = (sin_tab[idx_a] + SPWM_OFFSET) * MODULATION_INDEX; // u相逆变占空比    14管
//    float v_phase_duty = (sin_tab[idx_b] + SPWM_OFFSET) * MODULATION_INDEX; // v相逆变占空比    36管
//    float w_phase_duty = (sin_tab[idx_c] + SPWM_OFFSET) * MODULATION_INDEX; // w相逆变占空比    52管
//
//    // 使用Channel BCD 进行三相逆变
//    HRTIM_SetPara(CHB, u_phase_duty);
//    HRTIM_SetPara(CHC, v_phase_duty);
//    HRTIM_SetPara(CHD, w_phase_duty);
//
//    index++;
//    if(index >= SIN_POINTS)
//        index = 0;
//}

/**
* @brief          开环调制 前馈占空比获取
* @param          *duty 以指针形式获取未经调制的前馈占空比
* @param          *sin_theta 以指针形式获取相位角
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
void Three_Phase_Inverter_GetOpenDuty(float *duty, float *sin_theta)
{
    static uint16_t index = 0;
    const uint16_t points = SIN_POINTS; // sin_tab 点数
    const uint16_t phase_shift = points / 3; // 约 170 点，对应 120°

    // 计算三个相位（120°相差） 使用逆相序
    // U相：~ sin(wt)
    // V相：~ sin(wt + 120°)
    // W相：~ sin(wt - 120°)

    uint16_t idx_a = (index) % points;
    uint16_t idx_b = (index + phase_shift) % points;
    uint16_t idx_c = (index + 2 * phase_shift) % points;

    // 计算占空比：偏移+调制度
    float u_phase_duty = sin_tab[idx_a]; // u相逆变占空比    14管
    float v_phase_duty = sin_tab[idx_b]; // v相逆变占空比    36管
    float w_phase_duty = sin_tab[idx_c]; // w相逆变占空比    52管

    // 使用Channel BCD 进行三相逆变
    duty[0] = u_phase_duty;
    duty[1] = v_phase_duty;
    duty[2] = w_phase_duty;

    sin_theta[0] = sin_tab[idx_a];
    sin_theta[1] = sin_tab[idx_b];
    sin_theta[2] = sin_tab[idx_c];

    index++;
    if(index >= SIN_POINTS)
        index = 0;

}


/**
* @brief          生成参考波形并且调频
* @param          *duty 以指针形式获取未经调制的前馈占空比
* @param          *sin_theta 以指针形式获取相位角
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
void DC_AC_Adjust(float *sin_theta)
{
    static float wt = 0;    // a相相位角
    #define  shift  0.66667f * PI2 // 120度相移

    // 计算三个相位（120°相差） 使用逆相序
    // U相：~ sin(wt)
    // V相：~ sin(wt + 120°)
    // W相：~ sin(wt - 120°)

//    float theta_a = (wt);
//    float theta_b = (wt + shift);
//    float theta_c = (wt + 2 * shift);
//
//    // 计算占空比：偏移+调制度
//    float u_phase_duty = arm_sin_f32(theta_a); // u相逆变占空比    14管
//    float v_phase_duty = arm_sin_f32(theta_b); // v相逆变占空比    36管
//    float w_phase_duty = arm_sin_f32(theta_c); // w相逆变占空比    52管

    sin_theta[0] = arm_sin_f32(wt);
    sin_theta[1] = arm_sin_f32(wt + shift);
    sin_theta[2] = 0 - sin_theta[0] - sin_theta[1];

    wt += wt_step;
    if(wt >= 6.283f)
        wt -= 6.283f;

}

/**
* @brief          通过调整wt步长实现调频
* @param          f0    期望频率 hz
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
void DC_AC_FrequencyAdjust(float f0)
{
    if (f0 <= 20)
        f0 = 20;
    if (f0 >= 100)
        f0 = 100;

    // 控制频率 25KHZ
    float ticks = 25e3 / f0;
    wt_step = 2 * PI / ticks;
}