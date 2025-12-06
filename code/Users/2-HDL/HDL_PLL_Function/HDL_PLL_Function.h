/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file					HDL_PLL_Function.h
  * @brief      			离网逆变控制系统
  * @note       			包含信号生成(SOGI)、相位同步(PLL)、闭环调节(PID)及功率调制(SPWM)四大核心模块
  * @history				Hardware Driver Layer硬件驱动层
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

#include "MWL_PLL.h"         //PLL相关算法
#include "arm_math.h"        //dsp库
#include "HDL_bearADC_Measure.h"

#include "MWL_Data_Utils.h"
#include "HDL_bearADC_Measure.h"

/**
 * @brief 宏定义
 */
#define Ts  10e-5f    //ADC采样时间:10us

#define OFF_grid_wt  9817.46875e-6f //离网控制角速度定义


/**
 * @brief 结构体
 */
typedef struct
{
    int32_t uo_average;
    int32_t uo_sum;
    int16_t uo_code[640];
    int32_t io_average;
    int32_t io_sum;
    int16_t io_code[640];

} Average_TypeDef; //平均值结构体

typedef struct   //离网控制结构体
{
    float32_t now_yo;//当前输出电压
    float32_t now_yo_goal;//当前输出电压
    float32_t now_yo_pi_out; //当前输出电压控制的pi输出
}Off_grid_Average_pi;

typedef struct   //锁相环控制结构体
{
    float pll_wt;        //锁相环角速度
    float Q_out;         //Q轴
    float D_out;         //D轴
    float pll_pi_out;    //锁相环的pi输出
}SOGI_pll_lock_struct;

/**
 * @brief 变量外部声明
 */
extern uint16_t ADC_CODE[4];   //ADC外设到内存地址
extern int32_t PWM_Duty;
//二阶广义积分器结构体变量
extern SOGI_TypeDef SOGI_U; 				     //交流电压的二阶广义积分器结构体变量
extern SOGI_TypeDef SOGI_I; 				     //交流电流的二阶广义积分器结构体变量
extern Average_TypeDef AC_U_I_average;           //平均值控制结构体
extern Average_TypeDef DC_U_I_average;           //平均值控制结构体
extern arm_pid_instance_f32 Off_grid_pid_U;      //离网控制pid参数
extern Off_grid_Average_pi OFF_UI_;              //离网控制结构体变量
extern SOGI_pll_lock_struct SOGI_pll_lock_U;     //电压锁相环结构体变量
extern arm_pid_instance_f32 U_pll_lock_pi;       //电压锁相环pi控制变量

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

void PLL_sys_init(void); //系统初始化
void PWM_set( int32_t __PWM_Duty); //装载PWM
void SOGI_pll_lock_init();
void getAcOutAverage(Average_TypeDef *pamer,int16_t ADC_U,int16_t ADC_I,int16_t i);
void off_average_contrl_pid_init();
void off_uo_average_contrl(Off_grid_Average_pi *OFF_UI_,
                arm_pid_instance_f32 *Off_grid_pid_U,
                Average_TypeDef *pamer,
                float wt,
                int32_t *PWMDuty
);
void SOGI_pll_lock(SOGI_pll_lock_struct *__pll_lock,
                    arm_pid_instance_f32 *__pll_pi,
                    SOGI_TypeDef *SOGI
                    );


#ifdef __cplusplus
}
#endif