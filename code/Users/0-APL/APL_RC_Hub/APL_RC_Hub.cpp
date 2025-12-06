/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       APL_RC_Hub.c
  * @brief      解析通信数据 接收队列数据进行解析，实现与ISR的解耦
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-28-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "APL_RC_Hub.h"
#include "MWL_Data_Utils.h"
#include "HAL_USART.h"
#include "Dev_Remote_Control.h"
#include "App_Detect_Task.h"

/**
 * @brief 构造数据解析对象
 */
Class_RC_Hub RC_Hub;


/**
 * @brief   远程控制中心线程
 */
void _Thread_RC_Hub_(void *pvParameters)
{
    vTaskDelay(RC_HUB_INIT_TIME);
    RC_Hub.Init(&VT_Module, &UART7_Manage_Object);

    while (1)
    {
        RC_Hub.Data_Analysis();

        vTaskDelay(RC_HUB_CONTROL_TIME);
    }
}


/**
 * @brief          初始化数据解析类
 * @param[in]      *_VT_Module_Obj_    图传模块指针
 *  `              *_UART_Manage_Obj_  串口实例指针
 * @retval         none
 */
void Class_RC_Hub::Init(Class_Video_Transmission *_VT_Module_Obj_, UART_Manage_Object_t *_UART_Manage_Obj_)
{
    _VT_Module_Obj_->Init(_UART_Manage_Obj_);
    this->VT_Moudle = _VT_Module_Obj_;
}


/**
 * @brief          解析机器人交互数据，更新对应控制结构体
 * @param[in]      none
 * @retval         none
 */
void Class_RC_Hub::Data_Analysis()
{
    // 临时缓冲区
    uint8_t custom_data[30];
    uint8_t remote_data[12];
    uint8_t vt_rc_data[21];

    // 自定义控制器
    if(pdPASS == xQueueReceive(VT_Moudle->custom_robot.xcustom_queue, custom_data, 0))
    {
        uint8_to_float(custom_data, custom_control.custom_angle_set, JOINTS_NUM);
        detect_hook(VT_TOE);
        detect_hook(CUSTOM_TOE);
    }

    // 图传链路键鼠
    if(pdPASS == xQueueReceive(VT_Moudle->remote_robot.xremote_queue, remote_data, 0))
    {
        remote_control.last_left_button_down  =  remote_control.left_button_down;
        remote_control.last_right_button_down =  remote_control.right_button_down;
        remote_control.last_keyboard_value    =  remote_control.keyboard_value;

        memcpy(&remote_control.mouse_x, &remote_data[0], sizeof(remote_control.mouse_x));
        memcpy(&remote_control.mouse_y, &remote_data[2], sizeof(remote_control.mouse_y));
        memcpy(&remote_control.mouse_z, &remote_data[4], sizeof(remote_control.mouse_z));
        memcpy(&remote_control.left_button_down, &remote_data[6], sizeof(remote_control.left_button_down));
        memcpy(&remote_control.right_button_down, &remote_data[7], sizeof(remote_control.right_button_down));
        memcpy(&remote_control.keyboard_value, &remote_data[8], sizeof(remote_control.keyboard_value));

        detect_hook(VT_TOE);
    }

    // 新图传键鼠+遥控
    if(pdPASS == xQueueReceive(VT_Moudle->vt_rc_robot.xrc_vt_queue, vt_rc_data, 0))
    {
        //帧头 2bytes
        vt_rc_control.sof_1 = vt_rc_data[0];
        vt_rc_control.sof_2 = vt_rc_data[1];

        // 提取 8 字节的 bitfield 数据（data[2] ~ buf[9]）
        uint64_t bitfield = 0;
        for (int i = 0; i < 8; i++)
        {
            bitfield |= ((uint64_t)vt_rc_data[i + 2]) << (8 * i);
        }

        // 图传键位 8bytes
        vt_rc_control.ch_0     = (bitfield >> 0)  & 0x7FF;
        vt_rc_control.ch_1     = (bitfield >> 11) & 0x7FF;
        vt_rc_control.ch_2     = (bitfield >> 22) & 0x7FF;
        vt_rc_control.ch_3     = (bitfield >> 33) & 0x7FF;
        vt_rc_control.mode_sw  = (bitfield >> 44) & 0x03;
        vt_rc_control.pause    = (bitfield >> 46) & 0x01;
        vt_rc_control.fn_1     = (bitfield >> 47) & 0x01;
        vt_rc_control.fn_2     = (bitfield >> 48) & 0x01;
        vt_rc_control.wheel    = (bitfield >> 49) & 0x7FF;
        vt_rc_control.trigger  = (bitfield >> 60) & 0x01;

        // 键鼠键位 9bytes
        int16_t tmp_mouse_x = (vt_rc_data[10] | (vt_rc_data[11] << 8));
        int16_t tmp_mouse_y = (vt_rc_data[12] | (vt_rc_data[13] << 8));
        int16_t tmp_mouse_z = (vt_rc_data[14] | (vt_rc_data[15] << 8));

        vt_rc_control.mouse_x = (float) tmp_mouse_x;
        vt_rc_control.mouse_y = (float) tmp_mouse_y;
        vt_rc_control.mouse_z = (float) tmp_mouse_z;

        uint8_t mouse_byte = vt_rc_data[16];
        vt_rc_control.mouse_left   = (mouse_byte >> 0) & 0x03;
        vt_rc_control.mouse_right  = (mouse_byte >> 2) & 0x03;
        vt_rc_control.mouse_middle = (mouse_byte >> 4) & 0x03;

        vt_rc_control.key   = vt_rc_data[17] | (vt_rc_data[18] << 8);

        // CRC校验 2bytes
        vt_rc_control.crc16 = vt_rc_data[19] | (vt_rc_data[20] << 8);

        detect_hook(VT_TOE);

        // 摇杆&拨轮归中
        vt_rc_control.ch_0   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.ch_1   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.ch_2   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.ch_3   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.wheel  -= RC_CH_VALUE_OFFSET;

    }
}

/**
 * @brief          将控制结构体的原始数据进行加工
 * @param[in]      none
 * @retval         none
 */
void Class_RC_Hub::Data_Processing()
{
    // 自定义控制器
    if(&custom_control != NULL)
    {

    }

    // 图传链路键鼠
    if(&remote_control != NULL)
    {

    }

    // 新图传键鼠+遥控
    if(&vt_rc_control != NULL)
    {


    }
}