/*==========================================================================
 * 描述    ：凌霄飞控外置传感器处理
 * 更新时间：2020-02-06 
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
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "ANO_DT_LX.h"

_fc_ext_sensor_st ext_sens;

//这里把光流数据打包成通用速度传感器数据
static inline void General_Velocity_Data_Handle()
{
	static u8 of_update_cnt, of_alt_update_cnt;
	static u8 dT_ms = 0;
	//每一毫秒dT_ms+1，用来判断是否长时间无数据
	if (dT_ms != 255)
	{
		dT_ms++;
	}
	//检查OF数据是否更新
	if (of_update_cnt != ano_of.of_update_cnt)
	{
		of_update_cnt = ano_of.of_update_cnt;
		//XY_VEL
		if (ano_of.of1_sta && ano_of.work_sta) //光流有效
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = ano_of.of1_dx;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = ano_of.of1_dy;
		}
		else //无效
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
		}
	}
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//不输入z轴速度，将z速度赋值为无效
		ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = 0x8000;
		//触发发送
		dt.fun[0x33].WTS = 1;
		//reset
		dT_ms = 0;
	}
}

static inline void General_Distance_Data_Handle()
{
	static u8 of_alt_update_cnt;
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//
		ext_sens.gen_dis.st_data.direction = 0;
		ext_sens.gen_dis.st_data.angle_100 = 270;
		ext_sens.gen_dis.st_data.distance_cm = ano_of.of_alt_cm;
		//触发发送
		dt.fun[0x34].WTS = 1;
	}
}

void LX_FC_EXT_Sensor_Task(float dT_s) //1ms
{
	//
	General_Velocity_Data_Handle();
	//
	General_Distance_Data_Handle();
}
