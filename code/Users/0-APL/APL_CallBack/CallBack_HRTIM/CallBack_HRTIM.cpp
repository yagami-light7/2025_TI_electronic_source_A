/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       CallBack_HRTIM.h
  * @brief      高分辨率定时器hrtim中断回调函数
  * @note       Application Layer应用层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     07-24-2025     Light            1. done
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
#include "CallBack_HRTIM.h"
#include <cstdio>
#include "HAL_USART.h"
#include <arm_math.h>


Three_Phase_Inverter_t Three_Phase_Inverter = {&AC_Measure,
                               &PR_DC_AC[0], &PR_DC_AC[1], &PR_DC_AC[2],
                               &PID_DC_AC[0], &PID_DC_AC[1], &PID_DC_AC[2], &Volt_PID};

//PWMRectifier_t PWMRectifier = {&bearAC_Measure,&PR_PWM_Rectifier};

Three_Phase_Rectifier_t Three_Phase_Rectifier = {&AC_Measure,
                &PR_AC_DC[0], &PR_AC_DC[1], &PR_AC_DC[2],
               &PID_AC_DC[0], &PID_AC_DC[1], &PID_AC_DC[2], &Curr_PID};
/**
 * @brief   逆变控制部分
 * @note    在Hrtim定时器A溢出中断中被调用
 */
static void DC_AC_Loop(void);

/**
 * @brief   整流控制部分
 * @note    在Hrtim定时器A溢出中断中被调用 紧跟逆变之后 使用同一个参考波形
 */
static void AC_DC_Loop(void);

/**
 * @brief   定时中断回调函数 100kHz / 4
 * @note    PR后馈部分实现 边沿检测
 */
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    // 定时器TimerA
    if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A)
    {
        DC_AC_Loop();
//        // 逆变正常输出再开启整流通道

        AC_DC_Loop();
    }
}

/**
 * @brief   定时中断回调函数
 * @note    PR后馈部分实现 中央检测
 */
void HAL_HRTIM_Compare3EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    // 设置TIMER A Compare2 = 13600 为中央？
    if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A)
    {


    }
}

/**
 * @brief   用于三相逆变的PR控制器初始化
 * @note    此代码会在初始化时被调用
 */
void Controller_Init(void)
{
//    const float Volt_Kpid[3] = {0.1, 0.001, 0};
//    PID_init(&Volt_PID, PID_Incremental, Volt_Kpid, 0.75f, 0.3f);

    const float Current_Kpid[3] = {0.01, 0.0001, 0};
//    const float Voltage_Kpid[3] = {0.016667 , 0, 0};    // 1 / DC_BUS_VOLTAGE = 1 / 60 = 0.016667f
    for (int i = 0; i < 3; ++i)
    {
        float ts = 4e-5;   // 采样周期 40us
        float f0 = 50.0f;      // 谐振频率 50Hz
        float wc = 2.0f * PI * 5.0f;  // 带宽 约0.5Hz
        float w0 = 2.0f * PI * f0;    // 谐振中心频率

        PR_init(Three_Phase_Inverter.PR_V_Control[i], 1.0, 51.0f, ts, wc , w0);
//        PID_init(Three_Phase_Inverter.PID_I_Control[i], PID_POSITION, Current_Kpid, 50.0, -50.0f);
        PR_init(Three_Phase_Rectifier.PR_I_Control[i], 0.1f, 5.0f, ts, wc , w0);
        PID_init(Three_Phase_Rectifier.Curr_PID, PID_POSITION, Current_Kpid, 0.8f, 0.2f);
//        PR_Init(Three_Phase_Inverter.PR_Control[i], 0.01f, 10.0f, , 1,  PR_OUT_MAX, PR_OUT_MAX);
    }

}

static inline void findMinMax_fast(float a, float b, float c, float *max, float *min)
{
    *max = a;
    *min = a;

    // 手动优化比较
    *max = (b > *max) ? b : *max;
    *min = (b < *min) ? b : *min;

    *max = (c > *max) ? c : *max;
    *min = (c < *min) ? c : *min;
}

static float sin_theta[3] = {0, 0,0};  // 生成参考波形 乘以幅值就是电压电流参考波形

static float inv_duty[3] = {0, 0, 0};       // 调制后的占空比

float V_inv = 0;
static float V_comp [3] = {0,0,0};      // PR补偿电压--->送入调制模块
/**
 * @brief   逆变控制部分
 * @note    在Hrtim定时器A溢出中断中被调用
 */
static void DC_AC_Loop(void)
{

    static float I_set[3] = {0,0,0};        // 电压外环输出 = 电流内环输入
    static float V_ref[3] = {0, 0, 0};      // 电压参考波形

    /* 设定：生成三相参考波形 */
    DC_AC_Adjust(sin_theta);

    for (int i = 0; i < 3; ++i)
    {
        V_ref[i] = PHASE_V_AMPLITUDE * sin_theta[i];
    }

    /* 反馈：计算交流电压电流参数 */
    DC_AC_Calc(&AC_Measure);

    /* 双闭环：进行PR控制计算*/
    for (int i = 0; i < 2; ++i)
    {
        PR_flag = 1;    // 不可以变频
        I_set[i]  = PR_calc(Three_Phase_Inverter.PR_V_Control[i], V_ref[i], AC_Measure.un[i]);   // PR计算补偿电压
        V_comp[i] = 0.06f * (I_set[i] - AC_Measure.il[i]);
    }
    V_comp[2] = 0 - V_comp[0] - V_comp[1];
    PR_flag = 0; // 可以变频

//    /* 调制：减去零序分量提升电压利用率*/
//    float Max_, Min_;
//
//    float Ua = + V_comp[0];
//    float Ub = + V_comp[1];
//    float Uc = + V_comp[2];
//
//    findMinMax_fast(Ua, Ub, Uc, &Max_, &Min_);
//
//    float Uzero = (Max_ + Min_) * 0.5f; // 计算零序分量
//
//    float Ua_new = Ua - Uzero;  // 注入零序分量
//    float Ub_new = Ub - Uzero;
//    float Uc_new = Uc - Uzero;

#define spwm_kp 0.50f * 1.0f / DC_BUS_VOLTAGE  // 调制比增强系数
    inv_duty[0] = (spwm_kp * V_comp[0])  + 0.50;
    inv_duty[1] = (spwm_kp * V_comp[1])  + 0.50;
    inv_duty[2] = (spwm_kp * V_comp[2])  + 0.50;

    HRTIM_SetPara(CHB, inv_duty[0]);// 更新占空比
    HRTIM_SetPara(CHC, inv_duty[1]);
    HRTIM_SetPara(CHD, inv_duty[2]);


//    snprintf((char*)UART1_Manage_Object.tx_buffer, sizeof(UART1_Manage_Object.tx_buffer),
//     "ADC:%d,%d,%d\n",  (int )(100 * AC_Measure.il[0]),
//                        (int )( 100 *AC_Measure.il[1]),
//                        (int )( 100 * AC_Measure.il[2]));
//
//        UART_Send_Data(&UART1_Manage_Object,UART1_Manage_Object.tx_buffer,strlen((char*)UART1_Manage_Object.tx_buffer));
}

float I_rec = 0;

/**
 * @brief   整流控制部分
 * @note    在Hrtim定时器A溢出中断中被调用 紧跟逆变之后 使用同一个参考波形
 */
static void AC_DC_Loop(void)
{
    static float I_comp [3] = {0,0,0};      // PR补偿电压--->送入调制模块
    static float V_set[3] = {0,0,0};        // 电流外环输出 = 电压内环输入（此处规定外环是第一个控制器）
    static float duty[3] = {0, 0, 0};       // 调制后的占空比
    static float I_ref[3] = {0, 0, 0};      // 电流参考波形

    /* 设定：生成三相参考波形 */
//    DC_AC_Adjust(sin_theta);
//    for (int i = 0; i < 3; ++i)
//    {
//        I_ref[i] = Line_I_AMPLITUDE * sin_theta[i];
//    }

    /* 反馈：计算交流电压电流参数 */
    AC_DC_Calc(&AC_Measure);
//    DC_AC_Calc(Three_Phase_Inverter.AC_Measure);

    /* 双闭环：进行PR控制计算*/
//    for (int i = 0; i < 3; ++i)
//    {
//        PR_flag = 1;    // 不可以变频
//        V_set[i]  = PR_calc(Three_Phase_Rectifier.PR_I_Control[i], I_ref[i], AC_Measure.il[i + 3]);
//        I_comp[i] = (AC_Measure.un[i] - V_set[i]); // 注意负号
//    }
//    PR_flag = 0;

    /* 调制：减去零序分量提升电压利用率*/
//    float Max_, Min_;
//
//    float Ia = + I_comp[0];
//    float Ib = + I_comp[1];
//    float Ic = + I_comp[2];
//
//    findMinMax_fast(Ia, Ib, Ic, &Max_, &Min_);
//
//    float Izero = (Max_ + Min_) * 0.5f; // 计算零序分量
//
//    float Ia_new = Ia - Izero;  // 注入零序分量
//    float Ib_new = Ib - Izero;
//    float Ic_new = Ic - Izero;

    #define spwm_kp_ 0.5 * 1.0f / Line_I_AMPLITUDE   // 调制比增强系数
//    duty[0] = ( spwm_kp_ * I_comp[0])  + 0.50;
//    duty[1] = ( spwm_kp_ * I_comp[1])  + 0.50;
//    duty[2] = ( spwm_kp_ * I_comp[2])  + 0.50;

    float modulation_a = PID_Calc(&Curr_PID, AC_Measure.il_rms[0], 1.414f);
//    float modulation_b = 0.1f * (2.0f - AC_Measure.il_rms[1]);
//    float modulation_c = 0.1f * (2.0f - AC_Measure.il_rms[2]);

    duty[0] = (spwm_kp * 0.904 * V_comp[0])  + 0.50;
    duty[1] = (spwm_kp * 0.904 * V_comp[1])  + 0.50;
    duty[2] = (spwm_kp * 0.904 * V_comp[2])  + 0.50;


    HRTIM_SetPara(CHA, duty[0]);// 更新占空比
    HRTIM_SetPara(CHE, duty[1]);
    HRTIM_SetPara(CHF, duty[2]);

//    snprintf((char*)UART1_Manage_Object.tx_buffer, sizeof(UART1_Manage_Object.tx_buffer),
//     "ADC:%d,%d,%d\n",  (int )(100 * duty[0]),
//                        (int )(100 * duty[1]),
//                        (int )(100 * duty[2]));
//
//            UART_Send_Data(&UART1_Manage_Object,UART1_Manage_Object.tx_buffer,strlen((char*)UART1_Manage_Object.tx_buffer));
}





//        modulation = PID_Calc(Three_Phase_Inverter.Volt_PID, Three_Phase_Inverter.AC_Measure->un_rms, Three_Phase_Inverter.un_rms_target);
//        duty[i] = (forward[i] + SPWM_OFFSET) * MODULATION_INDEX;   // 调制度越大，输出有效值越大，最小占空比在0.1125