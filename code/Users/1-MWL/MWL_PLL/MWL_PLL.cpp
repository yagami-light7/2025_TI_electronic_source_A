/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       			MWL_PLL.cpp
  * @brief      			锁相环相关算法：二阶广义积分器、低通滤波器、PR控制器
  * @note       			Middleware_Layer
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
#include "MWL_PLL.h"

/**
 * @brief          将uint8_t数组转化为float数组
 *
 * @param[in]      uint8_data   需要转化的uint8_t数组
 *                 float_data   转化后的float数组
 *                 num_floats   float数组元素个数
 *
 * @retval         none
 */
void SOGI_transfrom(SOGI_TypeDef *Pamer, float in)
{
	Pamer->ui0 = in;
	Pamer->ua0 = Pamer->b0*(Pamer->ui0-Pamer->ui2) + Pamer->a1*Pamer->ua1 + Pamer->a2*Pamer->ua2;
	Pamer->ua2 = Pamer->ua1;
	Pamer->ua1 = Pamer->ua0;

	Pamer->ub0 = Pamer->qb0*Pamer->ui0 + Pamer->qb1*Pamer->ui1 + Pamer->qb2*Pamer->ui2 + Pamer->a1*Pamer->ub1 + Pamer->a2*Pamer->ub2;
	Pamer->ub2 = Pamer->ub1;
	Pamer->ub1 = Pamer->ub0;

	Pamer->ui2 = Pamer->ui1;
	Pamer->ui1 = Pamer->ui0;

	/*alpha = Pamer->ua0;
	beta = Pamer->ub0;
	*/
}


/**
 * @brief          求解二阶广义积分器的系数
 *
 * @param[in]      Pamer   二阶广义变量的结构体指针
 *                 k
 *                 Ts   采样周期
 *				   w    离散角速度
 *
 * @retval         none
 */
void SOGI_Parameter_Init(SOGI_TypeDef *Pamer, float k, float Ts, float w)
{

	Pamer->x = 2*k*w*Ts;
	Pamer->y = w*w*Ts*Ts;
	Pamer->b0 = Pamer->x/(Pamer->x+Pamer->y+4);
	Pamer->b2 = -Pamer->b0;
	Pamer->a1 = (8-2*Pamer->y)/(Pamer->x+Pamer->y+4);
	Pamer->a2 = (Pamer->x-Pamer->y-4)/(Pamer->x+Pamer->y+4);
	Pamer->qb0 = k*Pamer->y/(Pamer->x+Pamer->y+4);
	Pamer->qb1 = 2*Pamer->qb0;
	Pamer->qb2 = Pamer->qb0;
}


//void park_f32(SOGI_TypeDef *Pamer,float sinVal,float cosVal)
//{
//	//park变换
//	Pamer->D_out = Pamer->ua0 * sinVal - Pamer->ub0 * cosVal;
//	Pamer->Q_out = Pamer->ua0 * cosVal + Pamer->ub0 * sinVal;
//}

/**
 * @brief          计算浮点数的模运算
 *
 * @param[in]      _X    被除数（浮点数）
 *                 _Y    除数（浮点数）
 *
 * @retval         返回 _X 除以 _Y 的余数
 */
float MyFmod(float _X, float _Y)
{
    return _X - (int32_t)(_X / _Y) * _Y;
}

/**
 * @brief          执行一阶IIR低通滤波计算（基于递归差分方程）
 *
 * @param[in,out]  Pamer    低通滤波器状态及参数结构体指针
 *               - a/b/c:  差分方程系数（需预先配置）
 *               - ui0:    当前时刻输入值 (x[n])
 *               - ui1:    上一时刻输入值 (x[n-1])
 *               - uo0:    当前时刻输出值 (y[n])
 *               - uo1:    上一时刻输出值 (y[n-1])
 * @param[in]      in       当前输入信号
 *
 * @retval         void     滤波结果更新至结构体状态字段（Pamer->uo0）
 */
void Low_pass_filter(LP_filter_TypeDef *Pamer ,float in)  //低通滤波器
{
	Pamer->ui1 = Pamer->ui0;
	Pamer->ui0 = in;
	Pamer->uo0 = Pamer->ui0*Pamer->a + Pamer->ui1 * Pamer->b + Pamer->uo1 * Pamer->c;
	Pamer->uo1 = Pamer->uo0;
}


/**
 * @brief          初始化一阶IIR低通滤波器的系数
 *
 * @param[in,out]  Pamer   低通滤波器结构体指针（需预先分配内存）
 * @param[in]      A       当前输入权重系数（通常与滤波强度负相关）
 * @param[in]      B       历史输入权重系数（通常为0或辅助系数）
 * @param[in]      C       历史输出权重系数（通常与滤波强度正相关）
 *
 * @retval         void    无返回值，系数直接写入结构体
 */
void Low_pass_filter_init(LP_filter_TypeDef *Pamer,float A,float B,float C)  //低通滤波器
{
	Pamer->a = A;
	Pamer->b = B;
	Pamer->c = C;

}

/**
 * @brief          初始化比例谐振控制器（PR控制器）的状态变量
 *
 * @param[in,out]  PR_Pamer   PR控制器状态结构体指针
 *
 * @retval         void       无返回值，所有历史状态清零
 */
void PR_init(PR_TypeDef *PR_Pamer)
{
	PR_Pamer->x0 = 0;
	PR_Pamer->x1 = 0;
	PR_Pamer->x2 = 0;
	PR_Pamer->y0 = 0;
	PR_Pamer->y1 = 0;
	PR_Pamer->y2 = 0;
}

/**
 * @brief          执行比例谐振（PR）控制器的实时计算
 *
 * @param[in,out]  PR_Pamer   PR控制器状态及参数结构体指针
 * @param[in]      error      当前输入误差信号
 *
 * @retval         float      返回当前控制量输出值（PR_Pamer->y0）
 */
float f32_PR_Calculate(PR_TypeDef *PR_Pamer ,float error )
{

	PR_Pamer->x0 = error;
	PR_Pamer->y0 = 1.001f*PR_Pamer->x0 - 2.00f*PR_Pamer->x1 + 0.9998f*PR_Pamer->x2 + 2.0f*PR_Pamer->y1 - 0.9998f*PR_Pamer->y2;
	PR_Pamer->y2 = PR_Pamer->y1;
	PR_Pamer->y1 = PR_Pamer->y0;
	PR_Pamer->x2 = PR_Pamer->x1;
	PR_Pamer->x1 = PR_Pamer->x0;
}



