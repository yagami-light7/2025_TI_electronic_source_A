/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file                            HDL_bearADC_Measure.h
  * @brief                           草莓熊的专属ADC采样数据处理，未经允许，请勿私自调用!!!
  * @note
  * @history                         Hardware Driver Layer硬件驱动层
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
#include "HAL_ADC.h"
#include "MWL_Toolbox.h"


/**
 * @brief 宏定义
 */
#define ADC_FREQ   67          // PWM载波频率（HRTIM频率）[kHz]

#define ADC_MAXVALU                                 4096                                    //ADC最大量程
#define ADC_VREF                                    3.300f                                  //Vref
#define ADC_FACT                                    ADC_VREF / ADC_MAXVALU                  //3.3/4096

#define DC_V_BIAS                                   0                                       //直流电压采样偏置
#define DC_V_GAIN_2188                              20.0f                                   //OPA2188电压增益
#define AC_V_BIAS                                   2048                                     //交流电压采样偏置
#define AC_V_GIAN                                   52.07f                                      //交流电压采样增益
#define AC_C_BIAS                                   0                                       //交流电流采样偏置

#define DC_V_RATIO_2188                             ADC_FACT * DC_V_GAIN_2188               //3.3/4096*20
#define AC_V_RATIO                                  ADC_FACT * AC_V_GIAN                    //3.3/4096*32

#define DC_I_BIAS                                   1862                                    //直流电流采样偏置
#define DC_I_GAIN_A1                                1.0f / 20.0f                            //INA240A1电压增益20倍
#define DC_I_GAIN_A2                                1.0f / 50.0f                            //INA240A2电压增益50倍
#define AC_I_BIAS                                   0//待补充
#define R_SAMP_INV_2M                               500.0f                                  //2mOhm采样电阻倒数
#define R_SAMP_INV_4M                               250.0f                                  //4mOhm采样电阻倒数
#define R_SAMP_INV_10M                              100.0f                                  //10mOhm采样电阻倒数

#define DC_I_RATIO_A1_2M                            ADC_FACT * DC_I_GAIN_A1 * R_SAMP_INV_2M    //3.3/4096/20/0.002
#define DC_I_RATIO_A1_4M                            ADC_FACT * DC_I_GAIN_A1 * R_SAMP_INV_4M
#define DC_I_RATIO_A1_10M                           ADC_FACT * DC_I_GAIN_A1 * R_SAMP_INV_10M   //3.3/4096/20/0.010
#define DC_I_RATIO_A2_2M                            ADC_FACT * DC_I_GAIN_A2 * R_SAMP_INV_2M
#define DC_I_RATIO_A2_4M                            ADC_FACT * DC_I_GAIN_A2 * R_SAMP_INV_4M
#define DC_I_RATIO_A2_10M                           ADC_FACT * DC_I_GAIN_A2 * R_SAMP_INV_10M
#define AC_I_RATIO                                  0//待补充


//硬件限制相关
#define DC_CIN_MAX                                   7.5f                                //AC输入过流阈值
#define DC_COUT_MAX                                  8.0f                                //DC输出过流阈值
#define AC_VIN_MAX                                   40.0f                               //AC输入过压阈值
#define AC_VIN_MIN                                   19.0f                               //AC输入欠压阈值
#define DC_VOUT_MAX                                  23.0f                               //输出最大电压
#define DC_IOUT_MAX                                  4.0f                                //输出最大电流


//校准相关
#define AC_CIN_CAL_POINT_1                             2.0f
#define AC_CIN_CAL_POINT_2                             4.0f
#define AC_VIN_CAL_POINT_1                             20.0f
#define AC_VIN_CAL_POINT_2                             24.0f
#define DC_COUT_CAL_POINT_1                            2.0f
#define DC_COUT_CAL_POINT_2                            4.0f
#define DC_VOUT_CAL_POINT_1                            3.0f
#define DC_VOUT_CAL_POINT_2                            6.0f
#define CAPV_CAL_POINT_1                            17.0f
#define CAPV_CAL_POINT_2                            22.0f

//ADC衰减值
#define ADC0_scaling_factor 5.838e-3f    //实际电流到ADC衰减值
#define ADC1_scaling_factor	26.382e-3f  	//实际电压到ADC衰减值
#define ADC2_scaling_factor	1.098e-3f 	//实际到ADC衰减值
#define ADC3_scaling_factor	0.01256667f 	//实际到ADC衰减值

/**
 * @brief 结构体
 */
typedef struct
{
    ADC_Manage_Object_t *adc;
    first_order_filter_type_t *ac_in_filter[2];
    first_order_filter_type_t *dc_out_filter[2];
    float AC_IN[2];    // 输入交流
    float DC_OUT[2];    // 输出直流
}bearAC_Measure_t;

/**
 * @brief 变量外部声明
 */
extern bearAC_Measure_t bearAC_Measure;

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

void bearADC_LowPass_Filter_Init();
void bearAC_Voltage_Calc(bearAC_Measure_t *ac);

#ifdef __cplusplus
}
#endif