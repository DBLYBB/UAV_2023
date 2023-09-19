/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：飞控初始化
**********************************************************************************/
#include "Drv_BSP.h"
#include "Drv_PwmOut.h"
#include "Drv_led.h"
#include "Drv_adc.h"
#include "Drv_RcIn.h"
#include "Drv_Timer.h"
#include "ANO_DT_LX.h"
#include "Drv_UbloxGPS.h"
#include "Drv_Uart.h"
#include "Drv_Timer.h"

u8 All_Init()
{
	DrvSysInit();
	//延时
	MyDelayMs(100);
	//LED功能初始化
	DvrLedInit();
	//初始化电调输出功能
	DrvPwmOutInit();
	MyDelayMs(100);
	//串口2树莓派
	DrvUart2Init(460800);
	//串口3激光雷达
	DrvUart3Init(115200);
	//串口4接匿名光流
	DrvUart4Init(500000);
	//串口5接imu
	DrvUart5Init(500000);
	MyDelayMs(100);
	//SBUS输入采集初始化
	DrvRcInputInit();
	//电池电压采集初始化
	DrvAdcInit();
	MyDelayMs(100);
	//数传模块初始化
	ANO_DT_Init();
	MyDelayMs(800);
	//GPS接口初始化
	Init_GPS();
	//初始化定时中断
	DrvTimerFcInit();
	//初始化完成，返回1
	return (1);
}

_rc_input_st rc_in;
void DrvRcInputInit(void)
{
	//任意初始化一个模式
	DrvRcPpmInit();
	//DrvRcSbusInit();

	//先标记位丢失
	rc_in.no_signal = 1;
	rc_in.fail_safe = 1;
}
void DrvPpmGetOneCh(u16 data)
{
	static u8 ch_sta = 0;
	if ((data > 2500 && ch_sta > 3) || ch_sta == 10)
	{
		ch_sta = 0;
		rc_in.signal_cnt_tmp++;
		rc_in.rc_in_mode_tmp = 1; //切换模式标记为ppm
	}
	else if (data > 300 && data < 3000) //异常的脉冲过滤掉
	{
		//
		rc_in.ppm_ch[ch_sta] = data;
		ch_sta++;
	}
}
void DrvSbusGetOneByte(u8 data)
{
/*
sbus flags的结构如下所示：
flags：
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10) b: 0001 0000
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a
*/
	const u8 frame_end[4] = {0x04, 0x14, 0x24, 0x34};
	static u32 sbus_time[2];
	static u8 datatmp[25];
	static u8 cnt = 0;
	static u8 frame_cnt;
	//
	sbus_time[0] = sbus_time[1];
	sbus_time[1] = GetSysRunTimeUs();
	if ((u32)(sbus_time[1] - sbus_time[0]) > 2500)
	{
		cnt = 0;
	}
	//
	datatmp[cnt++] = data;
	//
	if (cnt == 25)
	{
		cnt = 24;
		//
		//if(datatmp[0] == 0x0F && (datatmp[24] == 0x00))
		//if(datatmp[0] == 0x0F && ((datatmp[24] == 0x00)||(datatmp[24] == 0x04)||(datatmp[24] == 0x14)||(datatmp[24] == 0x24)||(datatmp[24] == 0x34)))
		if ((datatmp[0] == 0x0F && (datatmp[24] == 0x00 || datatmp[24] == frame_end[frame_cnt])))
		{
			cnt = 0;
			rc_in.sbus_ch[0] = (s16)(datatmp[2] & 0x07) << 8 | datatmp[1];
			rc_in.sbus_ch[1] = (s16)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
			rc_in.sbus_ch[2] = (s16)(datatmp[5] & 0x01) << 10 | ((s16)datatmp[4] << 2) | (datatmp[3] >> 6);
			rc_in.sbus_ch[3] = (s16)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
			rc_in.sbus_ch[4] = (s16)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
			rc_in.sbus_ch[5] = (s16)(datatmp[9] & 0x03) << 9 | ((s16)datatmp[8] << 1) | (datatmp[7] >> 7);
			rc_in.sbus_ch[6] = (s16)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
			rc_in.sbus_ch[7] = (s16)datatmp[11] << 3 | (datatmp[10] >> 5);

			rc_in.sbus_ch[8] = (s16)(datatmp[13] & 0x07) << 8 | datatmp[12];
			rc_in.sbus_ch[9] = (s16)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
			rc_in.sbus_ch[10] = (s16)(datatmp[16] & 0x01) << 10 | ((s16)datatmp[15] << 2) | (datatmp[14] >> 6);
			rc_in.sbus_ch[11] = (s16)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
			rc_in.sbus_ch[12] = (s16)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
			rc_in.sbus_ch[13] = (s16)(datatmp[20] & 0x03) << 9 | ((s16)datatmp[19] << 1) | (datatmp[18] >> 7);
			rc_in.sbus_ch[14] = (s16)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
			rc_in.sbus_ch[15] = (s16)datatmp[22] << 3 | (datatmp[21] >> 5);
			rc_in.sbus_flag = datatmp[23];

			//user
			//
			if (rc_in.sbus_flag & 0x08)
			{
				//如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
			}
			else
			{
				rc_in.signal_cnt_tmp++;
				rc_in.rc_in_mode_tmp = 2; //切换模式标记为sbus
			}
			//帧尾处理
			frame_cnt++;
			frame_cnt %= 4;
		}
		else
		{
			for (u8 i = 0; i < 24; i++)
			{
				datatmp[i] = datatmp[i + 1];
			}
		}
	}
}
static void rcSignalCheck(float *dT_s)
{
	//
	static u8 cnt_tmp;
	static u16 time_dly;
	time_dly += (*dT_s) * 1e3f;
	//==1000ms==
	if (time_dly > 1000)
	{
		time_dly = 0;
		//
		rc_in.signal_fre = rc_in.signal_cnt_tmp;

		//==判断信号是否丢失
		if (rc_in.signal_fre < 5)
		{
			rc_in.no_signal = 1;
		}
		else
		{
			rc_in.no_signal = 0;
		}
		//==判断是否切换输入方式
		if (rc_in.no_signal)
		{
			//初始0
			if (rc_in.sig_mode == 0)
			{
				cnt_tmp++;
				cnt_tmp %= 2;
				if (cnt_tmp == 1)
				{
					DrvRcSbusInit();
				}
				else
				{
					DrvRcPpmInit();
				}
			}
		}
		else
		{
			rc_in.sig_mode = rc_in.rc_in_mode_tmp;
		}
		//==
		rc_in.signal_cnt_tmp = 0;
	}
}

#define RC_NO_CHECK 0  //0：监测遥控信号；1：不检测遥控信号
//
void DrvRcInputTask(float dT_s)
{
	static u8 failsafe;
	//信号检测
	rcSignalCheck(&dT_s);
	//有信号
	if (rc_in.no_signal == 0)
	{
		//ppm
		if (rc_in.sig_mode == 1)
		{
			for (u8 i = 0; i < 10; i++) //注意只有10个通道
			{
				rc_in.rc_ch.st_data.ch_[i] = rc_in.ppm_ch[i]; 
			}
		}
		//sbus
		else if (rc_in.sig_mode == 2)
		{
			for (u8 i = 0; i < 10; i++) //注意只有10个通道
			{
				rc_in.rc_ch.st_data.ch_[i] = 0.644f * (rc_in.sbus_ch[i] - 1024) + 1500; //248 --1024 --1800转换到1000-2000
			}
		}
		//检查失控保护设置
		if ((rc_in.rc_ch.st_data.ch_[ch_5_aux1] > 1200 && rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1400) || (rc_in.rc_ch.st_data.ch_[ch_5_aux1] > 1600 && rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1800)||rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1800||rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800)
		{
			//满足设置，标记为失控
			failsafe = 1;
		}
		else
		{
			failsafe = 0;
		}
	}
	//无信号
	else
	{
		//失控标记置位
		failsafe = 1;
		//
		for (u8 i = 0; i < 10; i++) //注意只有10个通道
		{
			rc_in.rc_ch.st_data.ch_[i] = 0; //
		}
	}
#if (RC_NO_CHECK == 0)
	//失控标记
	rc_in.fail_safe = failsafe;
#else
	//无信号或者检测到失控
	if(rc_in.no_signal != 0 || failsafe != 0)
	{
		for(u8 i=0;i<10;i++)
		{
			rc_in.rc_ch.st_data.ch_[i] = 1500;
		}
	}
	//不标记失控
	rc_in.fail_safe = 0;
#endif
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
