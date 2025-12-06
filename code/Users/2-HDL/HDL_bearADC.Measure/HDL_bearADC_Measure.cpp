/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file                            HDL_bearADC_Measure.h
  * @brief                           草莓熊的专属ADC采样数据处理，未经允许，请勿私自调用
  * @note       
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

/**
 * @brief 头文件
 */
#include "HDL_bearADC_Measure.h"

bearAC_Measure_t bearAC_Measure = {&ADC_Manage_Object, &ac_in_filter[0],&ac_in_filter[1], &dc_out_filter[0],&dc_out_filter[0]};

//0 AC_in_v_bias(交流电压偏置), 1 AC_in_v_ratio(交流电压缩放比例), 2 DC_v_bias(直流电压偏置), 3 DC_v_ratio(直流电压缩放比例),
// 4 AC_c_bias(交流电流偏置), 5 AC_c_ratio(交流电流缩放比例), 6 DC_c_bias(直流电流偏置), 7 DC_c_ratio(直流电流缩放比例)
const float samp_parameters[8] ={AC_V_BIAS, AC_V_RATIO, DC_V_BIAS, DC_V_RATIO_2188,
                                   AC_I_BIAS, AC_I_RATIO, DC_I_BIAS, DC_I_RATIO_A1_10M};

//校准参数
const float calibration_parameters[8] = {20.00f, 24.00f, 17.00f, 22.00f,
                                         3.00f, 6.00f, 2.00f, 4.00f};

/**
* @brief          初始化ADC使用的低通滤波器
* @note           初始化时调用
* @retval         none
*/
void bearADC_LowPass_Filter_Init()
{
    float num[1] = {0.005f};
    for (int i = 0; i < 2; ++i)
    {
        first_order_filter_init(bearAC_Measure.ac_in_filter[i], 1.0f / ADC_FREQ / 1000.0f * 4, num);
    }
    for (int i = 0; i < 2; ++i)
    {
        first_order_filter_init(bearAC_Measure.dc_out_filter[i], 1.0f / ADC_FREQ / 1000.0f * 4, num);
    }
}

/**
* @brief          根据ADC采样数据计算交流电压
* @note           此函数会在HRTIMCallBack中被循环调用
* @retval         none
*/
void bearAC_Voltage_Calc(bearAC_Measure_t *ac)
{

    for (int i = 0; i < 2; i++)
    {
        ac->adc->volt_buf[0] = ac->adc->adc2_samp_buf[i];
        first_order_filter_cali(ac->ac_in_filter[i], ac->adc->volt_buf[i]);
        ac->AC_IN[i] = ac->ac_in_filter[i]->out;
    }
    for (int i = 0; i < 2; i++)
    {
        ac->adc->volt_buf[0] = ac->adc->adc2_samp_buf[i];
        first_order_filter_cali(ac->dc_out_filter[i], ac->adc->volt_buf[i]);
        ac->DC_OUT[i] = ac->dc_out_filter[i]->out;
    }

}