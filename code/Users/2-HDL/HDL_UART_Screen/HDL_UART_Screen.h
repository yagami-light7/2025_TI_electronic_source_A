#pragma once

/**
 * @brief 头文件
 */
#include "stdint.h"
#include "HAL_USART.h"
/**
 * @brief 宏定义
 */



/**
 * @brief 结构体
 */
/* 存储串口屏的变量 */
struct Screen_var {
 uint8_t rx_buffer[128] = {0};         //  串口屏的接收数据处理的缓冲区
 int8_t Power_Factor;                   //  功率因素：          参数范围 0 - 10,对应 0.0 - 1.0 范围
 int8_t Converter_1_Hz;                 //  变流器1的输出频率:   参数范围 20 - 100 , 单位： Hz
};


/**
 * @brief 变量外部声明
 */
extern UART_Manage_Object_t UART2_Manage_Object;

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


extern void screen_var_sent(const char* var_name);



#ifdef __cplusplus
}
#endif
