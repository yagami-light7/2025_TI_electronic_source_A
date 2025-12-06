/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_AC_Measure.cpp
  * @brief      驱动ADC实现交流电压电流测量
  * @note       Hardware Driver Layer硬件驱动层
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
#include "HDL_AC_Measure.h"

AC_Measure_t AC_Measure = {&ADC_Manage_Object, &un_filter[0], &un_filter[1], &un_filter[2]};

float ADC_Data_Limit(float input, float max_step);

/**
* @brief          逆变 根据ADC采样数据计算交流电压电流
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
void DC_AC_Calc(AC_Measure_t *ac)
{

    #define adc_v_dc_offset  (1.508f * 50.0f)
    #define adc_v_gain       (1.0f / 4096.0f * 3.3f * 50.0f)

    #define adc_i_dc_offset  (2.75f * 5.0f / 0.625f)
    #define adc_i_gain       (1.0f / 4096.0f * 3.3f * 5.0f / 0.625f)

    // 进行电压电流原始数据的测量
    for (int i = 0; i < 2; i++)
    {
        // 转换
        ac->adc->volt_buf[i] = (float)ac->adc->adc1_samp_buf[i] * adc_v_gain - adc_v_dc_offset;
        ac->adc->curr_buf[i] = ((float)ac->adc->adc1_samp_buf[i + 3] * adc_i_gain - adc_i_dc_offset);

    }

    for (int i = 0; i <2; ++i)
    {
        ac->un[i] = ac->adc->volt_buf[i];
    }
    ac->un[2] = 0 - ac->un[0] - ac->un[1];

    for (int i = 0; i <2; ++i)
    {
        ac->il[i] = ac->adc->curr_buf[i];
    }
    ac->il[2] = 0 - ac->il[0] - ac->il[1];

//    // 计算线电压
//    ac->Uab = ac->un[0] - ac->un[1];
//    ac->Ubc = ac->un[1] - ac->un[2];
//    ac->Uca = ac->un[2] - ac->un[0];

//    for (int i = 0; i < 3; ++i)
//    {
//        // 存入缓冲区
//        if (ticks % 33 == 0)
//        {
//            ticks = 0;
//            ac->un_buf[index % BUF_LENRTH] = ac->un[3];
//            index++;
//
//        }
//    }
//
//
//    if (index >= BUF_LENRTH)
//    {
//        index = 0;
//    }
//    ticks++;


//    arm_rms_f32(&ac->un_buf[0], BUF_LENRTH, &ac->un_rms);

//    ac->un[0] = 0 - ac->un[1] - ac->un[2];

    // 有效值计算


    // 25khz / 1000 = 25hz 存入缓冲区 共 25个点

//    for (int i = 0; i < ADC2_CHANNEL_NUM; i++)
//    {
//        ac->adc->volt_buf[i + ADC1_CHANNEL_NUM] = -(ac->adc->adc2_samp_buf[i] / 4096.0f * 3.3f - 1.5207f) * 51.6f;
//        first_order_filter_cali(ac->un_filter[i], ac->adc->volt_buf[+ ADC1_CHANNEL_NUM]);
//        ac->adc->volt_filter_buf[i] = ac->un_filter[i]->out;
//    }

//    for (int i = 0; i < 3; i++)
//    {
//
//    }


}


/**
* @brief          整流 根据ADC采样数据计算交流电压电流
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
void AC_DC_Calc(AC_Measure_t *ac)
{
//    #define adc_i_dc_offset   1.50f * 3.445f
//    #define adc_i_gain       1.0f / 4096.0f * 3.3f * 3.445f
    // 进行电流原始数据的测量
    ac->adc->curr_buf[3] = (float)ac->adc->adc1_samp_buf[6] * adc_i_gain - adc_i_dc_offset;
//    ac->adc->curr_buf[4] = (float)ac->adc->adc2_samp_buf[0] * adc_i_gain - adc_i_dc_offset;
//    ac->adc->curr_buf[5] = (float)ac->adc->adc2_samp_buf[1] * adc_i_gain - adc_i_dc_offset;

    static uint64_t ticks = 0;
    static uint64_t index = 0;

    // 存入缓冲区
    if (ticks % 10 == 0 )
    {
        ticks = 0;
        ac->a_il_buf[index % BUF_LENRTH] = ac->adc->curr_buf[3];
//        ac->b_il_buf[index % BUF_LENRTH] = ac->adc->curr_buf[4];
//        ac->c_il_buf[index % BUF_LENRTH] = ac->adc->curr_buf[5];
        index++;


//        arm_rms_f32(&ac->b_il_buf[0], BUF_LENRTH, &ac->il_rms[1]);
//        arm_rms_f32(&ac->c_il_buf[0], BUF_LENRTH, &ac->il_rms[2]);
    }


    if (index >= BUF_LENRTH)
    {

        index = 0;
    }
    ticks++;
//    arm_rms_f32(&ac->a_il_buf[0], BUF_LENRTH, &ac->il_rms[0]);

//    arm_rms_f32(&ac->il_buf[0], BUF_LENRTH, &ac->il_rms);
//    ac->il[3] = ac->adc->curr_buf[3];
//    ac->il[4] = ac->adc->curr_buf[4];
//    ac->il[5] = ac->adc->curr_buf[5];
}



/**
* @brief          初始化ADC使用的低通滤波器
* @note           初始化时调用
* @retval         none
*/
void ADC_LowPass_Filter_Init()
{
    float num[1] = {0.00003f};
    for (int i = 0; i < 3; ++i)
    {
        first_order_filter_init(AC_Measure.un_filter[i], 1.0f / PWM_FREQ / 1000.0f * 4, num);
    }
}

float ADC_Data_Limit(float input, float max_step)
{
    // 限制相邻采样之间的变化幅度
    static float last = 0.0f;
    float delta = input - last;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    last += delta;

    return last;
}