/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       APL_USART.cpp
  * @brief      串口回调函数
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-29-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "APL_UASRT.h"

#include <cstdio>
#include <cstring>
#include <stdlib.h>
//#include "Dev_Remote_Control.h"
//#include "Dev_Custom.h"
//#include "Dev_Referee.h"
//#include "HDL_VT.h"

extern uint16_t f0_test;

/**
 * @brief          不定长接收中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if( huart == &huart2 )
    {
        if(UART2_Manage_Object.rx_buffer[0] == 0xAA)
        {
            if(UART2_Manage_Object.rx_buffer[1] == sizeof(UART2_Manage_Object.rx_buffer)+1)
            {
                uint8_t sum = 0;

                for(int i = 0;i < sizeof(UART2_Manage_Object.rx_buffer)-1;i++)
                {
                    sum += UART2_Manage_Object.rx_buffer[i];
                }

                if(sum == UART2_Manage_Object.rx_buffer[sizeof(UART2_Manage_Object.rx_buffer)-1])
                {
                    if(UART2_Manage_Object.rx_buffer[2] == 0x05)
                    {
                        f0_test += 1;
                    }


                    if(UART2_Manage_Object.rx_buffer[2] == 0x06)
                    {
                        f0_test -= 1;
                    }

                    if(UART2_Manage_Object.rx_buffer[2] == 0x07)
                    {
                        f0_test += 10;
                    }

                    if(UART2_Manage_Object.rx_buffer[2] == 0x08)
                    {
                        f0_test -= 10;
                    }

                }
            }
        }

    }
//        /* 串口屏的数据解析 */
//        if ( UART2_Manage_Object.rx_buffer[0] == 0x00 ) {
//            UART2_Manage_Object.rx_buffer[0] = Screen_Data.rx_buffer[0];
//        }
//        else
//        {
//            //  把内容搬运到RX2缓冲区
//            memcpy(&UART2_Manage_Object.rx_buffer[1], &Screen_Data.rx_buffer[1], Size);
//
//            /* 帧头为 0x1A 的 功率因素 解析 */
//            if ( Screen_Data.rx_buffer[0] == 0x1A ) {
//                Screen_Data.Power_Factor = ((atof((char*)&Screen_Data.rx_buffer[1])) * 10);
//            }
//
//            /* 帧头为 0x1B 的 Converter_1_Hz 数据解析 */
//            else if ( Screen_Data.rx_buffer[0] == 0x1B ) {
//                f0_test = atof((char*)&Screen_Data.rx_buffer[1]);
//            }
//
//            /* 解析完毕数据之后的操作 */
//            memset(&Screen_Data.rx_buffer , 0 , UART_RX_BUFFER_SIZE);  //  清空RX2接收区
//        }
//    }

    /* 每次处理完毕之后重新配置 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_Manage_Object.rx_buffer, UART_RX_BUFFER_SIZE);    // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


/**
 * @brief          错误处理中断
 * @param[in]      none
 * @retval         none
 */
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == UART5)
//     {
//         UART5_Error_Handler();
//     }
//     else if(huart->Instance == UART7)
//     {
//         HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.rx_buffer, UART7_Manage_Object.rx_data_size);
//     }
//     else if(huart->Instance == USART10)
//     {
//         UART10_Error_Handler();
//     }
// }