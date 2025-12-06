/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_HRTIM.cpp
  * @brief      驱动高分辨率定时器hrtim
  * @note       Hardware Abstract Layer硬件抽象层
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

/**
 * @brief 头文件
 */
#include <cmath>
#include "HAL_HRTIM.h"
#include "arm_math.h"

/**
 * @brief  HRTIM管理对象实例化
 */
HRTIM_Manage_Object_t HRTIM_Manage_Object = {&hhrtim1};


/**
 * @brief          HRTIM高分辨率定时器初始化
 * @param[in]      none
 * @retval         none
 */
void HRTIM_Init(void)
{
    // 设置移相角
    float phase_array[4] = {PHASE_B, PHASE_C,PHASE_D,PHASE_E}; // 对于单相逆变 通道同相 但有效电平应该相反

    HRTIM_PWM_SetPhaseDeg(phase_array);

    // 开启通道
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1|HRTIM_OUTPUT_TF2);

    // 使能中断
//    __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MDIER_MREPIE);
    __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, TIM_RCR_REP);  // 启用重复事件中断
//    __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, TIM_RCR_REP);  // 启用重复事件中断

    // 开启定时器
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B
                                | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_E | HRTIM_TIMERID_TIMER_F);


}

/**
 * @brief          修改PWM波的参数
 * @param[in]      _channel 对应的自定时器通道
 * @param[in]      _duty    占空比，范围 [0.0, 1.0]
 * @retval         none
 */
void HRTIM_SetPara(HRTIM_Timer_Channel_e _channel, float _duty)
{
    static uint32_t compare_ticks = 0;

    // 边界检查
    if (_duty < 0.05f)
        _duty = 0.05f;
    if (_duty > 0.95f)
        _duty = 0.95f;

    // 中心对齐模式下计算占空比
    compare_ticks = (uint32_t) (HRTIM_Manage_Object.hrtim->Instance->sTimerxRegs[_channel].PERxR * (1 - _duty));

    // 设置周期与比较寄存器
    HRTIM_Manage_Object.hrtim->Instance->sTimerxRegs[_channel].CMP1xR = compare_ticks;
}

/**
 * @brief          批量设置定时器 B~E 的移相角，自动归一到 [0, 360)区间
 * @param[in]      phase[4] 依次对应 B/C/D/E 四路的相位角（弧度）
 * @retval         none
 */
void HRTIM_PWM_SetPhaseDeg(float phase_deg[4])
{
    // 获取主定时器周期（计数上限）
    uint32_t period = HRTIM_Manage_Object.hrtim->Instance->sMasterRegs.MPER;

    // 指向四个 MCMP 寄存器地址的数组
    volatile uint32_t* mcmp_regs[4] = {
            &HRTIM_Manage_Object.hrtim->Instance->sMasterRegs.MCMP1R, // TimerB
            &HRTIM_Manage_Object.hrtim->Instance->sMasterRegs.MCMP2R, // TimerC
            &HRTIM_Manage_Object.hrtim->Instance->sMasterRegs.MCMP3R, // TimerD
            &HRTIM_Manage_Object.hrtim->Instance->sMasterRegs.MCMP4R  // TimerE
    };

    for (int i = 0; i < 4; i++)
    {
        // 归一化相位角到 [0, 360)
        float deg = fmodf(phase_deg[i], 360.0f);
        if (deg < 0.0f) deg += 360.0f;

        // 角度 → 周期计数
        uint32_t cmp = (uint32_t)((deg / 360.0f) * period);
        if (cmp == 0) cmp = 1;  // 防止设置为0，导致PWM波丢失

        // 设置比较值
        *mcmp_regs[i] = cmp;
    }
}

/**
 * @brief          对应通道的互补PWM增加占空比
 * @param[in]      _channel     对应的自定时器通道
 * @param[in]      _add_duty    占空比，范围 [0.0, 1.0]
 * @retval         none
 */
void HRTIM_AddDuty(HRTIM_Timer_Channel_e _channel, float _duty)
{
    static uint32_t add_compare_ticks = 0;

    // 边界检查
    if (_duty < 0.0f) _duty = 0.0f;
    if (_duty > 1.0f) _duty = 1.0f;

    // 中心对齐模式下计算占空比
    add_compare_ticks = (uint32_t) (HRTIM_Manage_Object.hrtim->Instance->sTimerxRegs[_channel].PERxR * (1 - _duty));

    // 限幅 防止得不到正确占空比
    if (add_compare_ticks > HRTIM_Manage_Object.hrtim->Instance->sTimerxRegs[_channel].PERxR)
        add_compare_ticks = HRTIM_Manage_Object.hrtim->Instance->sTimerxRegs[_channel].PERxR - 1;
    if (add_compare_ticks <= 0)
        add_compare_ticks = 1;

    // 设置周期与比较寄存器
    HRTIM_Manage_Object.hrtim->Instance->sTimerxRegs[_channel].CMP1xR -= add_compare_ticks;
}


//void getAcOutAverage(Average_TypeDef *pamer,int16_t ADC_U,int16_t ADC_I,int16_t i)
//{
//    //获取输出电压电流平均值
//    int16_t Vo;
//    Vo = (ADC_U);  //得到输出电压
//    pamer->uo_sum = pamer->uo_sum + Vo*Vo - pamer->uo_code[i]*pamer->uo_code[i];
//    pamer->uo_code[i] = Vo;
//    arm_sqrt_q31((pamer->uo_sum/640)>>7,&pamer->uo_average);
//    pamer->uo_average = pamer->uo_average>>12;
//    int16_t Io;
//    Io = (ADC_I); //得到输出电流
//    pamer->io_sum = pamer->io_sum + Io*Io - pamer->io_code[i]*pamer->io_code[i];
//    pamer->io_code[i] = Io;
//    arm_sqrt_q31((pamer->io_sum/640)>>7,&pamer->io_average);
//    pamer->io_average = pamer->io_average>>12;
//}
//void getDcOutAverage(Average_TypeDef *pamer,int16_t ADC_U,int16_t ADC_I,int16_t i)
//{
//    //获取输出电压电流平均值
//    int16_t Vo;
//    Vo = (ADC_U);  //得到输出电压
//    pamer->uo_sum = pamer->uo_sum + Vo - pamer->uo_code[i];
//    pamer->uo_code[i] = Vo;
//    pamer->uo_average = pamer->uo_sum/640;
//    int16_t Io;
//    Io = (ADC_I - ADC2_Offset); //得到输出电流
//    pamer->io_sum = pamer->io_sum + Io - pamer->io_code[i];
//    pamer->io_code[i] = Io;
//    pamer->io_average = pamer->io_sum/640 ;
//}