/**
* @brief 头文件
 */
#include "HDL_UART_Screen.h"
#include <cstring>
#include "stdio.h"
#include "main.h"
#include "string.h"

/**
* @brief 全局变量
 */
/* 存储串口屏的数据 */
Screen_var Screen_Data;

/* 更新串口屏的变量 */
void screen_var_sent(const char* var_name)
{
    /* 对发送缓冲区做清空处理 */
    int str_len = 0;
    memset(UART2_Manage_Object.tx_buffer, 0, UART_TX_BUFFER_SIZE);

    /* 变量是功率因素的设置代码 */
    if (strcmp(var_name, "Power_Factor") == 0) {
        /* 对数据做处理 */
        if      ( Screen_Data.Power_Factor > 10 ) { Screen_Data.Power_Factor = 10; }
        else if ( Screen_Data.Power_Factor <  0 ) { Screen_Data.Power_Factor =  0; }
        int8_t Power_Factor_H = 0;
        int8_t Power_Factor_L = 0;
        Power_Factor_H = (Screen_Data.Power_Factor / 10);
        Power_Factor_L = (Screen_Data.Power_Factor % 10);
        str_len = snprintf((char*)UART2_Manage_Object.tx_buffer, sizeof(UART2_Manage_Object.tx_buffer), "information.t1.txt=\"%d.%d\"\xff\xff\xff", Power_Factor_H, Power_Factor_L);
        HAL_UART_Transmit_DMA(&huart2, UART2_Manage_Object.tx_buffer,str_len);
    }

    /* 变量是变流器1的设置代码 */
    else if (strcmp(var_name, "Converter_1_Hz") == 0) {
        /* 对数据做处理 */
        if      ( Screen_Data.Converter_1_Hz > 100 ) { Screen_Data.Converter_1_Hz = 100; }
        else if ( Screen_Data.Converter_1_Hz <  20 ) { Screen_Data.Converter_1_Hz =  20; }
        str_len = snprintf((char*)UART2_Manage_Object.tx_buffer, sizeof(UART2_Manage_Object.tx_buffer), "information.t1.txt=\"%d\"\xff\xff\xff", Screen_Data.Converter_1_Hz);
        HAL_UART_Transmit_DMA(&huart2, UART2_Manage_Object.tx_buffer,str_len);
    }
}