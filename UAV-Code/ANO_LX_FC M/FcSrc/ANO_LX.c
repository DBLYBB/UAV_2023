#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "ANO_DT_LX.h"
#include "ANO_Math.h"
#include "Drv_PwmOut.h"
#include "LX_FC_State.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "Drv_adc.h"
#include "Drv_led.h"
#include "Drv_UbloxGPS.h"
#include "LX_FC_Fun.h"
#include "Drv_Uart.h"

/*==========================================================================
 * 描述    ：凌霄飞控输入、输出主程序
 * 更新时间：2020-01-22 
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。  
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。  
===========================================================================*/

_rt_tar_un rt_tar;
_pwm_st pwm_to_esc;
_fc_bat_un fc_bat;
_fc_att_un fc_att;
_fc_att_qua_un fc_att_qua;
_fc_vel_un fc_vel;
_fc_pos_un fc_pos;

//遥控CH5(AUX1)通道值(1000-1500-2000)设置模式1-2-3，模式0需要通过单独发送指令设置
//模式0：姿态自稳    ->遥控CH1-CH4直接控制姿态和油门。
//模式1：自稳+定高   ->遥控CH1/CH2/CH3控制姿态，但是遥控CH3(油门摇杆)控制垂直方向速度。
//模式2：定点        ->遥控CH1/CH2控制水平方向速度，并且遥控CH3(油门摇杆)控制垂直方向速度,遥控CH4控制YAW姿态。
//模式3：程控        ->遥控摇杆不参与控制

//
#define MAX_ANGLE 3500	//最大打杆时角度， 单位 0.01度
#define MAX_YAW_DPS 200 //最大打杆时YAW角速度，单位度每秒
//
#define MAX_VELOCITY 500  //最大打杆时水平速度，单位厘米每秒
#define MAX_VER_VEL_P 300 //最大打杆时垂直正速度，单位厘米每秒
#define MAX_VER_VEL_N 200 //最大打杆时垂直负速度，单位厘米每秒

//////////////////////////////////////////////////////////////////////
//以下为飞控基础功能程序，不建议用户改动和调用。
//////////////////////////////////////////////////////////////////////

//遥控器数据处理
static inline void RC_Data_Task(float dT_s)
{
	// static u8 fail_safe_change_mod, fail_safe_return_home;
	static u8 mod_f[3];
	static u16 mod_f_time_cnt;

	//遥控没有失控标记才执行
	if ( rc_in.fail_safe == 0)
	{
		//摇杆数据设置模式（姿态+气压定高，定高定点，程控）
		//注意，程控模式下，飞控只响应发送指令提供的信号，不再响应摇杆。
		if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1200)
		{
			LX_Change_Mode(1);
			mod_f[0] = 1;
		}
		else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1700)
		{
			LX_Change_Mode(2);
			mod_f[0] = 2;
		}
		else
		{
			LX_Change_Mode(3);
			mod_f[0] = 3;
		}
		//有切换模式时执行
		if (mod_f[1] != mod_f[0])
		{
			mod_f[1] = mod_f[0];
			//如果是模式3，自增一次。
			if (mod_f[0] == 3)
			{
				mod_f[2]++;
			}
		}
		//此段程序功能时2000ms内检测切换程控模式的次数,达到3次则执行返航
		if (mod_f[2] != 0)
		{
			if (mod_f_time_cnt < 2000)
			{
				mod_f_time_cnt += 1e3f * dT_s;
			}
			else
			{
				u8 tmp;
				if (mod_f[2] >= 3)
				{
					//执行返航
					tmp = OneKey_Return_Home();
				}
				else
				{
					//null
				}
				//reset
				if (tmp)
				{
					mod_f_time_cnt = 0;
					mod_f[2] = 0;
				}
			}
		}

		//摇杆数据转换物理控制量
		//摇杆数据转换到+-500并加死区
		float tmp_ch_dz[4];
		tmp_ch_dz[ch_1_rol] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_1_rol] - 1500), 0, 40);
		tmp_ch_dz[ch_2_pit] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_2_pit] - 1500), 0, 40);
		tmp_ch_dz[ch_3_thr] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_3_thr] - 1500), 0, 80);
		tmp_ch_dz[ch_4_yaw] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_4_yaw] - 1500), 0, 80);
		//准备上锁时，ROL,PIT,YAW无效
		if (sti_fun.pre_locking)
		{
			tmp_ch_dz[ch_1_rol] = 0;
			tmp_ch_dz[ch_2_pit] = 0;
			tmp_ch_dz[ch_4_yaw] = 0;
		}
		//摇杆转换姿态量、油门量
		//注意0.00217f和0.00238f是为了对应的补偿死区减小的部分，原本为0.002f，±500转换到±1。
		//注意正负号需要满足ANO坐标系定义，一般情况姿态角表示方向（包括上位机）和遥控摇杆反向都与飞控使用的ANO坐标系不同。
//		if(mod_f[0]<2)
//		{		
			rt_tar.st_data.rol = tmp_ch_dz[ch_1_rol] * 0.00217f * MAX_ANGLE;
			rt_tar.st_data.pit = -tmp_ch_dz[ch_2_pit] * 0.00217f * MAX_ANGLE;		//因为摇杆俯仰方向和定义的俯仰方向相反，所以取负
			rt_tar.st_data.thr = (rc_in.rc_ch.st_data.ch_[ch_3_thr] - 1000);		//0.1%
			rt_tar.st_data.yaw_dps = -tmp_ch_dz[ch_4_yaw] * 0.00238f * MAX_YAW_DPS; //因为摇杆航向方向和定义的航向方向相反，所以取负		
//		}
		//############(实时控制帧，自主开发闭环控制，在这里赋值即可)##############
		//实时XYZ-YAW期望速度(实时控制帧)
//		rt_tar.st_data.yaw_dps = 0;  //航向转动角速度，度每秒，逆时针为正
//		rt_tar.st_data.vel_x = 0;    //头向速度，厘米每秒
//		rt_tar.st_data.vel_y = 0;    //左向速度，厘米每秒
//		rt_tar.st_data.vel_z = 0;	 //天向速度，厘米每秒
		//########################################################################
		//=====
		dt.fun[0x41].WTS = 1; //将要发送rt_tar数据。
		//失控保护标记复位
		//fail_safe_change_mod = 0;
		//fail_safe_return_home = 0;
	}
	else //无遥控信号
	{
		//解锁后，丢失信号失控需要触发返航（不可返航时会自动触发降落）
		/* if (fc_sta.unlock_sta != 0)
		{
			//对应的改变模式
			if (fail_safe_change_mod == 0)
			{
				//失控保护，切换到程控模式
				fail_safe_change_mod = LX_Change_Mode(3);
			}
			else if (fail_safe_return_home == 0)
			{
				//切换到程控模式后，发送返航指令
				fail_safe_return_home = OneKey_Return_Home();
			}
		}
		*/
		//失控保护时目标值
		rt_tar.st_data.rol = 0;
		rt_tar.st_data.pit = 0;
		rt_tar.st_data.thr = 500; //用于模式0，避免模式0时失控，油门过大飞跑，给一个稍低于中位的油门
		//这里会把实时XYZ-YAW期望速度置零		
		for (u8 i = 0; i < 4; i++) 
		{
			rc_in.rc_ch.st_data.ch_[i] = 1500;  //模拟通道值
		}
		dt.fun[0x41].WTS = 1;
	}
}

//输出给电调
static inline void ESC_Output(u8 unlocked)
{
	static u8 esc_calibrated;
	static s16 pwm[8];
	//
	pwm[0] = pwm_to_esc.pwm_m1 * 0.1f;
	pwm[1] = pwm_to_esc.pwm_m2 * 0.1f;
	pwm[2] = pwm_to_esc.pwm_m3 * 0.1f;
	pwm[3] = pwm_to_esc.pwm_m4 * 0.1f;
	pwm[4] = pwm_to_esc.pwm_m5 * 0.1f;
	pwm[5] = pwm_to_esc.pwm_m6 * 0.1f;
	pwm[6] = pwm_to_esc.pwm_m7 * 0.1f;
	pwm[7] = pwm_to_esc.pwm_m8 * 0.1f;
	//

	if (esc_calibrated == 0)
	{
//注意，若打开ESC校准功能，将可能发生不可预料的损坏或者人身伤害，后果自负。
//一定需要校准时，请拆掉螺旋桨，尽量避免发生意外。
//校准ESC成功后，记得关闭此功能，避免出现意外。
#if (ESC_CALI == 1)
		//
		for (u8 i = 0; i < 8; i++)
		{
			pwm[i] = 1000; //校准时先输出最大油门，注意有危险。
		}
		//
		//有遥控信号 且 油门下拉到低位时，输出0油门信号
		if (rc_in.fail_safe == 0 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1150)
		{
			//
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = 0;
			}
			//标记校准完成。
			esc_calibrated = 1;
		}
#else
		//没有开校准功能，直接标记校准完成。
		esc_calibrated = 1;
#endif
	}
	else
	{
		//解锁才输出，否则输出0油门
		if (unlocked)
		{
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = LIMIT(pwm[i], 0, 1000);
			}
		}
		else
		{
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = 0;
			}
		}
	}
	//给底层PWM驱动输出信号
	DrvMotorPWMSet(pwm);
}

//根据ADC计算电池电压
static void Bat_Voltage_Data_Handle()
{
	fc_bat.st_data.voltage_100 = Drv_AdcGetBatVot() * 100; //单位：10mv
}

//定时1ms调用
void ANO_LX_Task()
{
	static u16 tmp_cnt[2];
	//计10ms
	tmp_cnt[0]++;
	tmp_cnt[0] %= 10;
	if (tmp_cnt[0] == 0)
	{
		//遥控输入
		DrvRcInputTask(0.01f);
		//遥控数据处理
		RC_Data_Task(0.01f);
		//飞控状态处理
		LX_FC_State_Task(0.01f); //
		//匿名光流状态检测
		AnoOF_Check_State(0.01f);
		//==
		//计100ms
		tmp_cnt[1]++;
		tmp_cnt[1] %= 10;
		if (tmp_cnt[1] == 0)
		{
			//读取电池电压信息
			Bat_Voltage_Data_Handle();
		}
	}
	//解析串口接收到的数据
	DrvUartDataCheck();
	//GPS数据处理
	GPS_Data_Prepare_Task(1);
	//外部传感器数据处理
	LX_FC_EXT_Sensor_Task(0.001f);
	//通信交换
	ANO_LX_Data_Exchange_Task(0.001f);
	//电调输出
	ESC_Output(1); //unlocked
	//灯光驱动
	LED_1ms_DRV();
}
