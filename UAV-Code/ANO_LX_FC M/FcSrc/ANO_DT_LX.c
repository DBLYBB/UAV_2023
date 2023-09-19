#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_led.h"
#include "LX_FC_State.h"
#include "Drv_Uart.h"

/*==========================================================================
 * 描述    ：凌霄飞控通信主程序
 * 更新时间：2020-01-22 
 * 作者		 ：匿名科创-茶不思
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

u8 send_buffer[50]; //发送数据缓存
_dt_st dt;

//===================================================================
void ANO_DT_Init(void)
{
	//========定时触发
	//
	dt.fun[0x0d].D_Addr = 0xff;
	dt.fun[0x0d].fre_ms = 100;	  //触发发送的周期100ms
	dt.fun[0x0d].time_cnt_ms = 1; //设置初始相位，单位1ms
	//
	dt.fun[0x40].D_Addr = 0xff;
	dt.fun[0x40].fre_ms = 20;	  //触发发送的周期100ms
	dt.fun[0x40].time_cnt_ms = 0; //设置初始相位，单位1ms
	//========外部触发
	//
	dt.fun[0x30].D_Addr = 0xff;
	dt.fun[0x30].fre_ms = 0;	  //0 由外部触发
	dt.fun[0x30].time_cnt_ms = 0; //设置初始相位，单位1ms
	//
	dt.fun[0x33].D_Addr = 0xff;
	dt.fun[0x33].fre_ms = 0;	  //0 由外部触发
	dt.fun[0x33].time_cnt_ms = 0; //设置初始相位，单位1ms
	//
	dt.fun[0x34].D_Addr = 0xff;
	dt.fun[0x34].fre_ms = 0;	  //0 由外部触发
	dt.fun[0x34].time_cnt_ms = 0; //设置初始相位，单位1ms
	//
	dt.fun[0x41].D_Addr = 0xff;
	dt.fun[0x41].fre_ms = 0;	  //0 由外部触发
	dt.fun[0x41].time_cnt_ms = 0; //设置初始相位，单位1ms
	//
	dt.fun[0xe0].D_Addr = 0xff;
	dt.fun[0xe0].fre_ms = 0;	  //0 由外部触发
	dt.fun[0xe0].time_cnt_ms = 0; //设置初始相位，单位1ms
	//
	dt.fun[0xe2].D_Addr = 0xff;
	dt.fun[0xe2].fre_ms = 0;	  //0 由外部触发
	dt.fun[0xe2].time_cnt_ms = 0; //设置初始相位，单位1ms
}

//数据发送接口
static void ANO_DT_LX_Send_Data(u8 *dataToSend, u8 length)
{
	//
	UartSendLXIMU(dataToSend, length);
}

//===================================================================
//数据接收程序
//===================================================================
static u8 DT_RxBuffer[256], DT_data_cnt = 0;
void ANO_DT_LX_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	//判断帧头是否满足匿名协议的0xAA
	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		DT_RxBuffer[0] = data;
	}
	//判断是不是发送给本模块的数据或者是广播数据
	else if (rxstate == 1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate = 2;
		DT_RxBuffer[1] = data;
	}
	//接收帧CMD字节
	else if (rxstate == 2)
	{
		rxstate = 3;
		DT_RxBuffer[2] = data;
	}
	//接收数据长度字节
	else if (rxstate == 3 && data < 250)
	{
		rxstate = 4;
		DT_RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	//接收数据区
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		DT_RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
	}
	//接收校验字节1
	else if (rxstate == 5)
	{
		rxstate = 6;
		DT_RxBuffer[4 + _data_cnt++] = data;
	}
	//接收校验字节2，表示一帧数据接收完毕，调用数据解析函数
	else if (rxstate == 6)
	{
		rxstate = 0;
		DT_RxBuffer[4 + _data_cnt] = data;
		DT_data_cnt = _data_cnt + 5;
		//ano_dt_data_ok = 1;
		ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt);
	}
	else
	{
		rxstate = 0;
	}
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数ANO_Data_Receive_Prepare自动调用
static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	//判断数据长度是否正确
	if (*(data + 3) != (len - 6))
		return;
	//根据收到的数据计算校验字节1和2
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	//计算出的校验字节和收到的校验字节做对比，完全一致代表本帧数据合法，不一致则跳出解析函数
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //判断sum校验
		return;
	//再次判断帧头以及目标地址是否合法
	if (*(data) != 0xAA || (*(data + 1) != HW_TYPE && *(data + 1) != HW_ALL))
		return;
	//=============================================================================
	//根据帧的CMD，也就是第3字节，进行对应数据的解析
	//PWM数据
	if (*(data + 2) == 0X20)
	{
		pwm_to_esc.pwm_m1 = *((u16 *)(data + 4));
		pwm_to_esc.pwm_m2 = *((u16 *)(data + 6));
		pwm_to_esc.pwm_m3 = *((u16 *)(data + 8));
		pwm_to_esc.pwm_m4 = *((u16 *)(data + 10));
		pwm_to_esc.pwm_m5 = *((u16 *)(data + 12));
		pwm_to_esc.pwm_m6 = *((u16 *)(data + 14));
		pwm_to_esc.pwm_m7 = *((u16 *)(data + 16));
		pwm_to_esc.pwm_m8 = *((u16 *)(data + 18));
	}
	//凌霄IMU发出的RGB灯光数据
	else if (*(data + 2) == 0X0f)
	{
		led.brightness[0] = *(data + 4);
		led.brightness[1] = *(data + 5);
		led.brightness[2] = *(data + 6);
		led.brightness[3] = *(data + 7);
	}
	//凌霄飞控当前的运行状态
	else if (*(data + 2) == 0X06)
	{
		fc_sta.fc_mode_sta = *(data + 4);
		fc_sta.unlock_sta = *(data + 5);
		fc_sta.cmd_fun.CID = *(data + 6);
		fc_sta.cmd_fun.CMD_0 = *(data + 7);
		fc_sta.cmd_fun.CMD_1 = *(data + 8);
	}
	//飞行速度
	else if (*(data + 2) == 0X07)
	{
		for(u8 i=0;i<6;i++)
		{
			fc_vel.byte_data[i] = *(data + 4 + i);
		}
	}
	else if (*(data + 2) == 0X08)
	{
		for(u8 i=0;i<8;i++)
		{
			fc_pos.byte_data[i] = *(data + 4 + i);
		}
	}
	//姿态角（需要在上位机凌霄IMU界面配置输出功能）
	else if (*(data + 2) == 0X03)
	{
		for(u8 i=0;i<7;i++)
		{
			fc_att.byte_data[i] = *(data + 4 + i);
		}		
	}
	//姿态四元数
	else if (*(data + 2) == 0X03)
	{
		for(u8 i=0;i<9;i++)
		{
			fc_att_qua.byte_data[i] = *(data + 4 + i);
		}			
	}
	//传感器数据
	else if (*(data + 2) == 0X01)
	{
		/*
		acc_x = *((s16 *)(data + 4));
		acc_y = *((s16 *)(data + 6));
		acc_z = *((s16 *)(data + 8));
		gyr_x = *((s16 *)(data + 10));
		gyr_y = *((s16 *)(data + 12));
		gyr_z = *((s16 *)(data + 14));	
		state = *(data + 16);
		*/
	}
	//命令E0，具体命令格式及功能，参见匿名通信协议V7版
	else if (*(data + 2) == 0XE0)
	{
		//根据命令ID：(*(data + 4)) ，来执行不同的命令
		switch (*(data + 4))
		{
		case 0x01:
		{
		}
		break;
		case 0x02:
		{
		}
		break;
		case 0x10:
		{
		}
		break;
		case 0x11:
		{
		}
		break;
		default:
			break;
		}
		//收到命令后，需要返回对应的应答信息，也就是CK_Back函数
		dt.ck_send.ID = *(data + 4);
		dt.ck_send.SC = check_sum1;
		dt.ck_send.AC = check_sum2;
		CK_Back(SWJ_ADDR, &dt.ck_send);
	}
	//收到的是ck返回
	else if (*(data + 2) == 0X00)
	{
		//判断收到的CK信息和发送的CK信息是否相等
		if ((dt.ck_back.ID == *(data + 4)) && (dt.ck_back.SC == *(data + 5)) && (dt.ck_back.AC == *(data + 6)))
		{
			//校验成功
			dt.wait_ck = 0;
		}
	}
	//读取参数
	else if (*(data + 2) == 0XE1)
	{
		//获取需要读取的参数的id
		u16 _par = *(data + 4) + *(data + 5) * 256;
		dt.par_data.par_id = _par;
		dt.par_data.par_val = 0;
		//发送该参数
		PAR_Back(0xff, &dt.par_data);
	}
	//写入参数
	else if (*(data + 2) == 0xE2)
	{
		//目前凌霄开源MCU不涉及参数的写入，推荐大家直接使用源码方式调整自己定义的参数，故此处只返回对应的CK校验信息
		//		u16 _par = *(data+4)+*(data+5)*256;
		//		u32 _val = (s32)(((*(data+6))) + ((*(data+7))<<8) + ((*(data+8))<<16) + ((*(data+9))<<24));
		//
		dt.ck_send.ID = *(data + 4);
		dt.ck_send.SC = check_sum1;
		dt.ck_send.AC = check_sum2;
		CK_Back(0xff, &dt.ck_send);
		//赋值参数
		//Parameter_Set(_par,_val);
	}
}

//===================================================================
//数据发送实现程序
//===================================================================
static void Add_Send_Data(u8 frame_num, u8 *_cnt, u8 send_buffer[])
{
	s16 temp_data;
	s32 temp_data_32;
	//根据需要发送的帧ID，也就是frame_num，来填充数据，填充到send_buffer数组内
	switch (frame_num)
	{
	case 0x00: //CHECK返回
	{
		send_buffer[(*_cnt)++] = dt.ck_send.ID;
		send_buffer[(*_cnt)++] = dt.ck_send.SC;
		send_buffer[(*_cnt)++] = dt.ck_send.AC;
	}
	break;
	case 0x0d: //电池数据
	{
		for (u8 i = 0; i < 4; i++)
		{
			send_buffer[(*_cnt)++] = fc_bat.byte_data[i];
		}
	}
	break;
	case 0x30: //GPS数据
	{
		//
		for (u8 i = 0; i < 23; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.fc_gps.byte[i];
		}
	}
	break;
	case 0x33: //通用速度测量数据
	{
		//
		for (u8 i = 0; i < 6; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.gen_vel.byte[i];
		}
	}
	break;
	case 0x34: //通用距离测量数据
	{
		//
		for (u8 i = 0; i < 7; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.gen_dis.byte[i];
		}
	}
	break;
	case 0x40: //遥控数据帧
	{
		for (u8 i = 0; i < 20; i++)
		{
			send_buffer[(*_cnt)++] = rc_in.rc_ch.byte_data[i];
		}
	}
	break;
	case 0x41: //实时控制数据帧
	{
		for (u8 i = 0; i < 14; i++)
		{
			send_buffer[(*_cnt)++] = rt_tar.byte_data[i];
		}
	}
	break;
	case 0xe0: //CMD命令帧
	{
		send_buffer[(*_cnt)++] = dt.cmd_send.CID;
		for (u8 i = 0; i < 10; i++)
		{
			send_buffer[(*_cnt)++] = dt.cmd_send.CMD[i];
		}
	}
	break;
	case 0xe2: //PARA返回
	{
		temp_data = dt.par_data.par_id;
		send_buffer[(*_cnt)++] = BYTE0(temp_data);
		send_buffer[(*_cnt)++] = BYTE1(temp_data);
		temp_data_32 = dt.par_data.par_val;
		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
	}
	break;
	default:
		break;
	}
}

//===================================================================

static void Frame_Send(u8 frame_num, _dt_frame_st *dt_frame)
{
	u8 _cnt = 0;

	send_buffer[_cnt++] = 0xAA;
	send_buffer[_cnt++] = dt_frame->D_Addr;
	send_buffer[_cnt++] = frame_num;
	send_buffer[_cnt++] = 0;
	//==
	//add_send_data
	Add_Send_Data(frame_num, &_cnt, send_buffer);
	//==
	send_buffer[3] = _cnt - 4;
	//==
	u8 check_sum1 = 0, check_sum2 = 0;
	for (u8 i = 0; i < _cnt; i++)
	{
		check_sum1 += send_buffer[i];
		check_sum2 += check_sum1;
	}
	send_buffer[_cnt++] = check_sum1;
	send_buffer[_cnt++] = check_sum2;
	//
	if (dt.wait_ck != 0 && frame_num == 0xe0)
	{
		dt.ck_back.ID = frame_num;
		dt.ck_back.SC = check_sum1;
		dt.ck_back.AC = check_sum2;
	}
	ANO_DT_LX_Send_Data(send_buffer, _cnt);
}
//===================================================================
//
static void Check_To_Send(u8 frame_num)
{
	//
	if (dt.fun[frame_num].fre_ms)
	{
		//
		if (dt.fun[frame_num].time_cnt_ms < dt.fun[frame_num].fre_ms)
		{
			dt.fun[frame_num].time_cnt_ms++;
		}
		else
		{
			dt.fun[frame_num].time_cnt_ms = 1;
			dt.fun[frame_num].WTS = 1; //标记等待发送
		}
	}
	else
	{
		//等待外部触发
	}
	//
	if (dt.fun[frame_num].WTS)
	{
		dt.fun[frame_num].WTS = 0;
		//实际发送
		Frame_Send(frame_num, &dt.fun[frame_num]);
	}
}
//===================================================================

//CMD发送
void CMD_Send(u8 dest_addr, _cmd_st *cmd)
{
	dt.fun[0xe0].D_Addr = dest_addr;
	dt.fun[0xe0].WTS = 1; //标记CMD等待发送
	dt.wait_ck = 1;		  //标记等待校验
}
//CHECK返回
void CK_Back(u8 dest_addr, _ck_st *ck)
{
	dt.fun[0x00].D_Addr = dest_addr;
	dt.fun[0x00].WTS = 1; //标记CMD等待发送
}
//PARA返回
void PAR_Back(u8 dest_addr, _par_st *par)
{
	dt.fun[0xe2].D_Addr = dest_addr;
	dt.fun[0xe2].WTS = 1; //标记CMD等待发送
}

//若指令没发送成功，会持续重新发送，间隔50ms。
static u8 repeat_cnt;
static inline void CK_Back_Check()
{
	static u8 time_dly;
	if (dt.wait_ck == 1)
	{
		if (time_dly < 50) //50ms
		{
			time_dly++;
		}
		else
		{
			time_dly = 0;
			repeat_cnt++;
			if (repeat_cnt < 5)
			{
				dt.fun[0xe0].WTS = 1; //标记等待发送，重发
			}
			else
			{
				repeat_cnt = 0;
				dt.wait_ck = 0;
			}
		}
	}
	else
	{
		time_dly = 0;
		repeat_cnt = 0;
	}
}

//1ms调用一次，用于通信交换数据
void ANO_LX_Data_Exchange_Task(float dT_s)
{
	//=====检测CMD是否返回了校验
	CK_Back_Check();
	//=====检测是否触发发送
	Check_To_Send(0x30);
	Check_To_Send(0x33);
	Check_To_Send(0x34);
	Check_To_Send(0x40);
	Check_To_Send(0x41);
	Check_To_Send(0xe0);
	Check_To_Send(0xe2);
	Check_To_Send(0x0d);
}

//===================================================================
