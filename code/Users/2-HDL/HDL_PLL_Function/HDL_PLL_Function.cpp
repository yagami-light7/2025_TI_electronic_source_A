/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file					HDL_PLL_Function.c
  * @brief      			离网逆变控制系统
  * @note       			包含信号生成(SOGI)、相位同步(PLL)、闭环调节(PID)及功率调制(SPWM)四大核心模块
  * @history				Hardware Driver Layer硬件驱动层
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
#include "HDL_PLL_Function.h"
#include "cordic.h"



int32_t PWM_Duty;  					//PWM占空比变量
//二阶广义积分器结构体变量
SOGI_TypeDef SOGI_U; 				//交流电压的二阶广义积分器结构体变量
SOGI_TypeDef SOGI_I; 				//交流电流的二阶广义积分器结构体变量


/**
 * @brief          初始化离网逆变控制系统核心模块
 *
 * @param[in]      void      无输入参数
 *
 * @retval         void      无返回值，完成系统关键模块初始化
 *
 * @detail 初始化流程：
 *  1. 电压/电流SOGI正交信号生成器（50Hz工频）
 *  2. 离网控制PID参数配置
 *  3. 锁相环状态变量清零
 *
 * @note 关键参数配置：
 *  - SOGI阻尼系数：0.4（平衡响应速度与谐振峰阻尼）[1,2](@ref)
 *  - 基频角速度：314.15926 rad/s（对应50Hz工频）
 *  - 采样周期：31.25μs（32kHz采样率）
 */
void PLL_sys_init(void) //系统初始化
{
	SOGI_Parameter_Init(&SOGI_U, 0.4f, 1.5e-5, 314.15926f);  	 //电压SOGI初始化，50Hz，100pi
	SOGI_Parameter_Init(&SOGI_I, 0.4f, 1.5e-5, 314.15926f); 		 //电流SOGI初始化，50Hz，100pi
	off_average_contrl_pid_init();
	SOGI_pll_lock_init();
}

//锁相环函数
SOGI_pll_lock_struct SOGI_pll_lock_U; //电压锁相环结构体变量
arm_pid_instance_f32 U_pll_lock_pi;   //电压锁相环pi控制变量

/**
 * @brief          初始化SOGI锁相环控制参数及状态变量
 *
 * @param[in]      void      无输入参数
 *
 * @retval         void      无返回值，完成锁相环核心参数配置
 *
 * @detail 配置要点：
 *  1. 锁相环PI控制器参数：Kp=10, Ki=1（快速跟踪相位误差）[5](@ref)
 *  2. 使用ARM DSP库初始化PID实例（自动重置积分状态）
 *  3. 清零四类关键状态变量：
 *     - D/Q轴分量（D_out, Q_out）
 *     - PI输出值（pll_pi_out）
 *     - 累积角度（pll_wt）
 */
void SOGI_pll_lock_init()
{
	//锁相环初始化函数
	U_pll_lock_pi.Kp = 10;
	U_pll_lock_pi.Ki = 0.01;
	U_pll_lock_pi.Kd = 0;  					//离网控制的pi参数初始化
	arm_pid_init_f32(&U_pll_lock_pi, 1);    // 参数1表示需要重置内部状态

	SOGI_pll_lock_U.D_out = 0;
	SOGI_pll_lock_U.pll_pi_out = 0;
	SOGI_pll_lock_U.pll_wt=0;
	SOGI_pll_lock_U.Q_out = 0;

}

/**
 * @brief          执行SOGI锁相环实时相位同步
 *
 * @param[in,out]  __pll_lock   锁相环状态结构体指针
 * @param[in]      __pll_pi     锁相环PI控制器实例指针
 * @param[in]      SOGI         SOGI正交信号发生器输出指针
 *
 * @retval         void         无返回值，相位信息更新至结构体
 *
 * @detail 工作流程：
 *  1. 正弦/余弦生成：基于当前角度计算Park变换所需分量
 *  2. Park变换：将SOGI输出的αβ分量转换为DQ坐标系[6](@ref)
 *  3. PI调节：通过Q轴分量误差修正频率（实现锁频）
 *  4. 角度累积：离散积分生成0-2π连续旋转角度
 *  5. 角度归一化：防止累积溢出（±2π边界处理）
 *
 * @note 频率补偿机制：
 *  叠加基频314.15926 rad/s（50Hz）确保快速锁定工频[5](@ref)
 */
void SOGI_pll_lock(SOGI_pll_lock_struct *__pll_lock,
					arm_pid_instance_f32 *__pll_pi,
					SOGI_TypeDef *SOGI
					)
{
	//锁相环函数
	float sinTheta,cosTheta;
	// angle = __pll_lock->pll_wt;
	// sinTheta = fast_sin(angle);
 //    cosTheta = fast_cos(angle);
	Calculate_Float_Sin_Cos(__pll_lock->pll_wt,&sinTheta, &cosTheta);//生成 当前角速度对应的 sin cos值
	arm_park_f32(SOGI->ua0, SOGI->ub0, &__pll_lock->Q_out, &__pll_lock->D_out, sinTheta, cosTheta);// park变换得到DQ
	__pll_lock->pll_pi_out = arm_pid_f32(__pll_pi,__pll_lock->Q_out); //PI控制DQ，目标q轴，实际q轴
	__pll_lock->pll_pi_out += 100*PI;     //pi输出加前馈值
	__pll_lock->pll_wt =__pll_lock->pll_wt + __pll_lock->pll_pi_out*Ts;//离散积分角速度得到 旋转的角度，Ts：采样间隔
    //限幅
	if(__pll_lock->pll_wt > 2*PI)
		__pll_lock->pll_wt = __pll_lock->pll_wt - 2*PI;
	else if(__pll_lock->pll_wt < -2*PI)
		__pll_lock->pll_wt = __pll_lock->pll_wt + 2*PI;

}

////////////////////////////////////////////////////////////////////////////////////
// void PWM_set( int32_t __PWM_Duty) //装载PWM
// {
// 	if(__PWM_Duty>=54400)
// 	{
// 		__PWM_Duty = 54400;
// 	}
// 	if(__PWM_Duty<=20)
// 	{
// 		__PWM_Duty = 20;
// 	}
// 	HRTIM1->sTimerxRegs[HRTIM_T    IMERINDEX_TIMER_A].CMP2xR =	__PWM_Duty ; //Q1PWM上升沿寄存器更新
// 	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR =	__PWM_Duty>>2 ; //Q1PWM上升沿寄存器更新
// 	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR = __PWM_Duty; //Q1PWM上升沿寄存器更新
// }

Average_TypeDef AC_U_I_average;//平均值控制结构体
Average_TypeDef DC_U_I_average;//平均值控制结构体
/**
 * @brief          计算交流输出电压/电流的实时RMS值
 *
 * @param[in,out]  pamer    平均值控制结构体指针（含环形缓冲区）
 * @param[in]      ADC_U    电压ADC原始采样值
 * @param[in]      ADC_I    电流ADC原始采样值
 * @param[in]      i        当前缓冲区索引位置（0-639）
 *
 * @retval         void     RMS值更新至结构体（uo_average/io_average）
 *
 * @detail 算法原理：
 *  1. 滑动窗口RMS计算：640点环形缓冲区（约20ms@32kHz）
 *  2. 平方累减更新：移除最旧值平方，加入新值平方
 *  3. ARM加速开方：arm_sqrt_q31优化计算效率
 *  4. 标度调整：右移12位匹配实际物理量
 *
 * @note 适用场景：
 *  工频交流信号的有效值计算（如逆变器输出电压/电流）[3](@ref)
 */
void getAcOutAverage(Average_TypeDef *pamer,int16_t ADC_U,int16_t ADC_I,int16_t i)
{
	//获取输出电压电流平均值
	int16_t Vo;
	Vo = (ADC_U);  //得到输出电压
	pamer->uo_sum = pamer->uo_sum + Vo*Vo - pamer->uo_code[i]*pamer->uo_code[i];
	pamer->uo_code[i] = Vo;
	arm_sqrt_q31((pamer->uo_sum/640)>>7,&pamer->uo_average);
	pamer->uo_average = pamer->uo_average>>12;
	int16_t Io;
	Io = (ADC_I); //得到输出电流
	pamer->io_sum = pamer->io_sum + Io*Io - pamer->io_code[i]*pamer->io_code[i];
	pamer->io_code[i] = Io;
	arm_sqrt_q31((pamer->io_sum/640)>>7,&pamer->io_average);
	pamer->io_average = pamer->io_average>>12;
}

/**
 * @brief          计算直流输出电压/电流的实时平均值
 *
 * @param[in,out]  pamer      平均值控制结构体指针
 * @param[in]      ADC_U      电压ADC原始采样值
 * @param[in]      ADC_I      电流ADC原始采样值（带零点校准）
 * @param[in]      i          当前缓冲区索引
 *
 * @retval         void       平均值更新至结构体
 *
 * @detail 算法特点：
 *  1. 滑动算术平均：640点环形缓冲区
 *  2. 电流零点校准：减去ADC2_Offset硬件偏移量
 *  3. 无开方运算：直流信号直接累加平均
 *
 * @note 校准说明：
 *  电流通道使用ADC2_Offset消除传感器零点漂移误差
 */
void getDcOutAverage(Average_TypeDef *pamer,int16_t ADC_U,int16_t ADC_I,int16_t i)
{
	//获取输出电压电流平均值
	int16_t Vo;
	Vo = (ADC_U);  //得到输出电压
	pamer->uo_sum = pamer->uo_sum + Vo - pamer->uo_code[i];
	pamer->uo_code[i] = Vo;
	pamer->uo_average = pamer->uo_sum/640;
	int16_t Io;
	Io = (ADC_I - AC_I_BIAS); //得到输出电流
	pamer->io_sum = pamer->io_sum + Io - pamer->io_code[i];
	pamer->io_code[i] = Io;
	pamer->io_average = pamer->io_sum/640 ;
}


arm_pid_instance_f32 Off_grid_pid_U;  	//离网控制pid参数
Off_grid_Average_pi OFF_UI_;  //离网控制结构体变量
/**
 * @brief          初始化离网电压控制PID参数
 *
 * @param[in]      void      无输入参数
 *
 * @retval         void      完成PID参数装载及状态初始化
 *
 * @detail 参数配置：
 *  - 目标电压：12.0V（直流母线参考值）
 *  - 比例增益Kp=240：确保电压刚性响应
 *  - 积分增益Ki=0.1：抑制稳态误差
 *  - 微分增益Kd=0：未启用
 *
 * @note 初始化特性：
 *  调用arm_pid_init_f32重置积分状态，防止启动冲击[6](@ref)
 */
void off_average_contrl_pid_init()
{
	//离网电压平均控制PI参数初始化
	OFF_UI_.now_yo_goal = 12.0f; //平均电压控制的目标电压
	Off_grid_pid_U.Kp = 240;
	Off_grid_pid_U.Ki = 0.1;
	Off_grid_pid_U.Kd = 0;  //离网控制的pi参数初始化
	arm_pid_init_f32(&Off_grid_pid_U, 1);  // 参数1表示需要重置内部状态
}

/**
 * @brief          离网模式电压闭环控制及PWM生成
 *
 * @param[in,out]  OFF_UI_        离网控制结构体指针
 * @param[in]      Off_grid_pid_U PID控制器实例指针
 * @param[in]      pamer          平均值结构体指针（含反馈值）
 * @param[in]      wt             当前锁相环角度（rad）
 * @param[out]     PWMDuty        PWM占空比输出指针
 *
 * @retval         void           无返回值，占空比写入PWMDuty
 *
 * @detail 控制策略：
 *  1. 电压误差计算：目标值 vs 反馈值（带ADC标度因子）
 *  2. PID输出计算：arm_pid_f32执行实时调节
 *  3. SPWM调制生成：PID输出叠加正弦载波+21250中心值
 *
 * @note PWM参数说明：
 *  - 21250：对应50%占空比（HRTIM定时器配置）
 *  - arm_sin_f32(wt)：生成与电网同步的正弦调制波
 */
void off_uo_average_contrl(Off_grid_Average_pi *OFF_UI_,
				arm_pid_instance_f32 *Off_grid_pid_U,
				Average_TypeDef *pamer,
				float wt,
				int32_t *PWMDuty
)
{
	//电压平均值控制法
	//离网pi
	float error;
	error = OFF_UI_->now_yo_goal - pamer->uo_average*ADC1_scaling_factor; //得到输出电压差值
	OFF_UI_->now_yo_pi_out = arm_pid_f32(Off_grid_pid_U, error);  // 计算PID输出
	*PWMDuty = OFF_UI_->now_yo_pi_out*arm_sin_f32(wt) + 21250;
}