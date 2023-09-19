#include "Drv_UbloxGPS.h"
#include "LX_FC_EXT_Sensor.h"
#include "ANO_Math.h"
#include "ANO_DT_LX.h"
/*==========================================================================
 * 描述    ：UBLOX_GPS数据解析
 * 更新时间：2018-11-08 
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

///////////////////////////////////////////////////////////////
//匿名协议赋值
///////////////////////////////////////////////////////////////
union _ubx ubx;
u8 pvt_receive_updata;
u16 fre_test_cnt;
void GPS_Data_Prepare_Task(u8 dT_ms)
{
	if (pvt_receive_updata)
	{
		pvt_receive_updata = 0;
		//====
		fre_test_cnt += 200;
		//赋值
		ext_sens.fc_gps.st_data.FIX_STA = ubx.pvt_data.fixType;
		ext_sens.fc_gps.st_data.S_NUM = ubx.pvt_data.numSV;
		ext_sens.fc_gps.st_data.LNG = ubx.pvt_data.lon;
		ext_sens.fc_gps.st_data.LAT = ubx.pvt_data.lat;
		ext_sens.fc_gps.st_data.ALT_GPS = ubx.pvt_data.hMSL / 10; //mm->cm
		ext_sens.fc_gps.st_data.N_SPE = ubx.pvt_data.velN / 10;	  //mm->cm
		ext_sens.fc_gps.st_data.E_SPE = ubx.pvt_data.velE / 10;	  //mm->cm
		ext_sens.fc_gps.st_data.D_SPE = ubx.pvt_data.velD / 10;	  //mm->cm
		//按协议处理赋值
		u32 tmp;
		tmp = ubx.pvt_data.pDOP * 0.01f;
		tmp = LIMIT(tmp, 0, 200);
		ext_sens.fc_gps.st_data.PDOP_001 = tmp;
		tmp = ubx.pvt_data.sAcc * 0.01f;
		tmp = LIMIT(tmp, 0, 200);
		ext_sens.fc_gps.st_data.SACC_001 = tmp;
		tmp = ubx.pvt_data.vAcc * 0.01f;
		tmp = LIMIT(tmp, 0, 200);
		ext_sens.fc_gps.st_data.VACC_001 = tmp;

		//标记置位，触发数据发送
		dt.fun[0x30].WTS = 1;
	}
}

///////////////////////////////////////////////////////////////
//以下为ublox 公司的UBX协议
///////////////////////////////////////////////////////////////

/*
PVT CLOSE:
//B5 62 06 01 03 00 01 07 00 12 50
//
PVT OPEN:
//B5 62 06 01 03 00 01 07 01 13 51

OPEN:
//B5 62 06 01 08 00 01 02 00 01 00 00 00 00 13 BE   POLLSH
//B5 62 06 01 03 00 01 02 01 0E 47  :ON
//B5 62 06 01 03 00 01 02 00 0D 46

//B5 62 06 01 08 00 01 06 00 01 00 00 00 00 17 DA   SOL
//B5 62 06 01 03 00 01 06 01 12 4F  :ON
//B5 62 06 01 03 00 01 06 00 11 4E

//B5 62 06 01 08 00 01 12 00 01 00 00 00 00 23 2E   VELNED
//B5 62 06 01 03 00 01 12 01 1E 67  :ON
//B5 62 06 01 03 00 01 12 00 1D 66  :OFF

//B5 62 06 01 08 00 01 21 00 01 00 00 00 00 32 97   TIMEUTC
//B5 62 06 01 03 00 01 21 01 2D 85  :ON
//B5 62 06 01 03 00 01 21 00 2C 84

//B5 62 06 01 03 00 01 04 01 10 4B 	:ON								DOP
//B5 62 06 01 03 00 01 04 00 0F 4A



//Target:UART1   Protocol in:0-UBX   Protocol out: 0-UBX
//B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 80 25 00 00 01 00 01 00 00 00 00 00 9A 79 波特率9600
//B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 96 00 00 01 00 01 00 00 00 00 00 8B 54 波特率38400
//B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 01 00 01 00 00 00 00 00 B8 42 波特率115200


//Pulse Period:200ms   Pulse Length:100ms
//B5 62 06 07 14 00 40 0D 03 00 A0 86 01 00 01 B5 62 06 07 14 00 40 0D 03 00 A0 86 01 00 01


//B5 62 06 08 06 00 3C 00 01 00 01 00 52 22      //60ms
{0xB5,0x62,0x06,0x08,0x06,0x00,0x3C,0x00,0x01,0x00,0x01,0x00,0x52,0x22};      //60ms

//B5 62 06 08 06 00 64 00 01 00 01 00 7A 12      //100  
{0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};          //100ms

//B5 62 06 08 06 00 7D 00 01 00 01 00 93 A8      //125ms

//B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A
{0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};          //200ms

//1000ms
{0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39  };      //1000ms


//Save current configuration 
// B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB         
*/

u8 Period_Out_H[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; //

u8 Period_Out_L[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}; //

//B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 96 00 00 01 00 01 00 00 00 00 00 8B 54
u8 Pulse_Period[] = {0xB5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x40, 0x0D, 0x03, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x01, 0x01,
					 0x00, 0x00, 0x34, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1, 0xCA};

//u8 Baud38400[]   ={0xB5,0x62,0x06,0x00,0x14,0x00,
//	                 0x01,0x00,0x00,0x00,0xD0,0x08,
//	                 0x00,0x00,0x00,0x96,0x00,0x00,
//                   0x01,0x00,0x01,0x00,0x00,0x00,
//                   0x00,0x00,0x8B,0x54,
//                   };

u8 Baud115200[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
				   0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x42}; //设置波特率

u8 Save_Con[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB}; //保存数据

//ON:
//u8 POLLSH_SET[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0x01 ,0x02 ,0x01 ,0x0E ,0x47};
//u8 SOL_SET[]    ={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0x01 ,0x06 ,0x01 ,0x12 ,0x4F};
//u8 VELNED_SET[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0x01 ,0x12 ,0x01 ,0x1E ,0x67};
//u8 TIMEUTC_SET[]={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0x01 ,0x21 ,0x01 ,0x2D ,0x85};
//u8 DOP_SET[]    ={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0x01 ,0x04 ,0x01 ,0x10 ,0x4B};

//OFF
u8 POLLSH_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x00, 0x0D, 0x46};
u8 SOL_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x00, 0x11, 0x4E};
u8 VELNED_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x00, 0x1D, 0x66};
u8 TIMEUTC_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x00, 0x2C, 0x84};
u8 DOP_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x00, 0x0F, 0x4A};
//pvt
u8 PVT_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51}; //B5 62 06 01 03 00 01 07 01 13 51

//u8 Enter_Send[]={0xB5,0x62,0x06,0x00,0x01,0x00,0x01,0x08,0x22};
//
//====
#include "Drv_Uart.h"
#define GPS_UART_INIT DrvUart1Init
#define GPS_PUT_BUFF DrvUart1SendBuf

void GPS_Rate_H()
{
	GPS_PUT_BUFF(Period_Out_H, sizeof(Period_Out_H));
	//ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));
}

void GPS_Rate_L()
{
	GPS_PUT_BUFF(Period_Out_L, sizeof(Period_Out_L));
	//ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));
}

//尝试用4种波特率初始化GPS
void Init_GPS()
{
#ifdef GPS_USE_UBLOX_M8
	MyDelayMs(100);
	GPS_UART_INIT(9600);

	MyDelayMs(50);
	GPS_PUT_BUFF(Baud115200, sizeof(Baud115200)); //9600 改变波特率 115200
												  //  ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));
												  //
	MyDelayMs(100);
	GPS_UART_INIT(19200);

	MyDelayMs(50);
	GPS_PUT_BUFF(Baud115200, sizeof(Baud115200)); //19200 改变波特率 115200
												  //    ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));
												  //
	MyDelayMs(100);
	GPS_UART_INIT(38400);

	MyDelayMs(50);
	GPS_PUT_BUFF(Baud115200, sizeof(Baud115200)); //38400 改变波特率 115200
												  //    ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));

	MyDelayMs(100);
	GPS_UART_INIT(115200);
	//

	MyDelayMs(20);
	GPS_PUT_BUFF(POLLSH_SET, sizeof(POLLSH_SET));
	//  ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));

	MyDelayMs(20);
	GPS_PUT_BUFF(SOL_SET, sizeof(SOL_SET));
	//	ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));

	MyDelayMs(20);
	GPS_PUT_BUFF(VELNED_SET, sizeof(VELNED_SET));
	//	ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));

	MyDelayMs(20);
	GPS_PUT_BUFF(TIMEUTC_SET, sizeof(TIMEUTC_SET));
	//	ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));

	MyDelayMs(20);
	GPS_PUT_BUFF(DOP_SET, sizeof(DOP_SET));
	//	ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));

	MyDelayMs(20);
	GPS_PUT_BUFF(PVT_SET, sizeof(PVT_SET));
	//	ANO_Uart1_Put_Buf(Enter_Send,sizeof(Enter_Send));
	//
	MyDelayMs(20);
	GPS_Rate_H(); //GPS_Rate_H();
#else
#ifdef GPS_USE_RTK
	//57600波特率用于RTK
	GPS_UART_INIT(57600);
#endif
#endif
}

//====================================

//#define UBX_BUF_NUM 100
//static u8 ubx_buf[UBX_BUF_NUM];
static u8 protocol_class, protocol_id, protocol_length_t;
static u16 protocol_length;
u8 pvt_recerve_ok_cnt;
void UBLOX_M8_GPS_Data_Receive(u8 Data)
{
	static u8 state = 0;
	u8 CK_A = 0, CK_B = 0;
	if (state == 0 && Data == 0xB5)
	{
		//		fre_test_cnt+=200;
		state = 1;
	}
	else if (state == 1 && Data == 0x62)
	{
		state = 2;
	}
	else if (state == 2)
	{
		state = 3;
		protocol_class = Data;
	}
	else if (state == 3)
	{
		state = 4;
		protocol_id = Data;
	}
	else if (state == 4)
	{
		state = 5;
		protocol_length_t = Data;
	}
	else if (state == 5)
	{
		state = 6;
		protocol_length = protocol_length_t + (Data << 8);
		if (protocol_length >= (UBX_BUF_NUM - 2))
		{
			state = 0; //reset
		}
		//protocol_length = (protocol_length>(UBX_BUF_NUM-2))?(UBX_BUF_NUM-2):protocol_length;
	}
	else if (state >= 6)
	{
		if ((state - 6) < (protocol_length + 2))
		{
			ubx.pvt_buf[state - 6] = Data;
			state++;
		}
		else //if((state-6) == (protocol_length+2))
		{
			CK_A += protocol_class;
			CK_B += CK_A;
			CK_A += protocol_id;
			CK_B += CK_A;
			CK_A += (protocol_length)&0xff;
			CK_B += CK_A;
			CK_A += (protocol_length >> 8) & 0xff;
			CK_B += CK_A;
			for (u8 i = 0; i < protocol_length; i++)
			{
				CK_A += ubx.pvt_buf[i];
				CK_B += CK_A;
			}

			if (CK_A == ubx.pvt_buf[protocol_length] && CK_B == ubx.pvt_buf[protocol_length + 1])
			{
				//
				if (protocol_class == 0x01 && protocol_id == 0x07)
				{
					pvt_receive_updata = 1;
				}
			}
			state = 0;
		}
	}
}
