/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-29-2025     Light            1. done
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
#include "hrtim.h"
#include "MWL_Data_Utils.h"

/**
 * @brief 宏定义
 */
#define PWM_FREQ   100          // PWM载波频率（HRTIM频率）[kHz]

#define HRTIM_INPUT_CLK   5440000000.0f  // HRTIM 定时器输入频率（单位Hz）

#define HRTIM_MAX_PERIOD_TICKS 65503
#define HRTIM_MIN_PERIOD_TICKS 96

#define HRTIM_MAX_FREQ_KHZ (HRTIM_INPUT_CLK / (HRTIM_MIN_PERIOD_TICKS * 1000.0f))
#define HRTIM_MIN_FREQ_KHZ (HRTIM_INPUT_CLK / (HRTIM_MAX_PERIOD_TICKS * 1000.0f))

//#define Single_PHASE_INVERTER   60    // 单相逆变相位角为1度
#define THREE_PHASE_INVERTER  1         // 三相逆变相位角为1度

#define PHASE_A 0                           // CHA默认与Master Channel同相位 移相角为0度
#define PHASE_B THREE_PHASE_INVERTER       // CHB相位
#define PHASE_C THREE_PHASE_INVERTER       // CHC相位
#define PHASE_D THREE_PHASE_INVERTER       // CHD相位
#define PHASE_E 1      // CHE相位
#define PHASE_F 1      // CHF相位

/**
 * @brief 结构体
 */

typedef struct
{
    HRTIM_HandleTypeDef *hrtim;
    float freq;    // 频率
    float duty;    // 占空比
}HRTIM_Manage_Object_t;//HRTIM对象结构体

typedef enum
{
    CHA = 0,
    CHB = 1,
    CHC = 2,
    CHD = 3,
    CHE = 4,
    CHF = 5
}HRTIM_Timer_Channel_e;

/**
 * @brief 变量外部声明
 */
extern HRTIM_Manage_Object_t HRTIM_Manage_Object;

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
* @brief          HRTIM高分辨率定时器初始化
* @param[in]      none
* @retval         none
*/
extern void HRTIM_Init(void);

/**
 * @brief          修改PWM波的参数
 * @param[in]      _channel 对应的自定时器通道
 * @param[in]      _duty    占空比，范围 [0.0, 1.0]
 * @retval         none
 */
extern void HRTIM_SetPara(HRTIM_Timer_Channel_e _channel, float _duty);

/**
 * @brief          批量设置定时器 B~E 的移相角，自动归一到 [0, 360)区间
 * @param[in]      phase[4] 依次对应 B/C/D/E 四路的相位角（角度）
 * @retval         none
 */
extern void HRTIM_PWM_SetPhaseDeg(float phase_deg[4]);

/**
 * @brief          对应通道的互补PWM增加占空比
 * @param[in]      _channel     对应的自定时器通道
 * @param[in]      _add_duty    占空比，范围 [0.0, 1.0]
 * @retval         none
 */
extern void HRTIM_AddDuty(HRTIM_Timer_Channel_e _channel, float _duty);

#ifdef __cplusplus
}
#endif
