#include "my_protocol.h"
#include "User_Task.h"
struct sdata received_data={0,0,140,0,0,0};
struct PID_inc height_PID;
struct PID_inc xy_PID;
u8 RxBuffer[256];//树莓派数据缓存
u8 LidarBuffer[256];//激光雷达数据缓存
u8 pi_receive_done_sign=0;//树莓派接收完成标志位
u8 lidar_receive_done_sign=0;//激光雷达接收完成标志位
s16 CSPX=0,CSPY=0;
u8 x_high=0;
u8 x_low=0;
u8 y_high=0;
u8 y_low=0;
s16 integ_side=0x4000;
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len)//串口发送函数
{
	u16 i=0;
	while(i<len)
	{
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC )==RESET);
		USART_SendData(USARTx,*s);
		s++;
		i++;
	}
}
void pi_receive(u8 data)//树莓派接受协议 串口2
{
	static u8 state_1 = 0;
	if(state_1==0&&data==0xAA)	//帧头0xAA
	{
		state_1=1;
		RxBuffer[0]=data;
	}
	else if(state_1==1)	//任务模式
	{
		state_1=2;
		RxBuffer[1]=data;

	}
	else if(state_1==2)		//x移动指令
	{
		state_1=3;
		RxBuffer[2]=data;
	}
	else if(state_1==3)		//y移动指令
	{
		state_1=4;
		RxBuffer[3]=data;
	}
	else if(state_1==4)    //z移动指令
	{
		state_1=5;
		RxBuffer[4]=data;
	}
	else if(state_1==5)    //yaw移动指令
	{
		state_1=6;
		RxBuffer[5]=data;
	}
	else if (state_1==6) 		//阶段切换指令 
	{
		RxBuffer[6]=data;
		state_1=7;
	}
	else if (state_1==7) 		//阶段切换指令 
	{
		RxBuffer[7]=data;
		state_1=8;
	}
	else if (state_1==8&&data==0xFF) 	//接收完成 帧尾0xFF
	{
		RxBuffer[8]=data;
		received_data.sp_side=RxBuffer[7];
		received_data.task_sta=RxBuffer[1];//启动指示
		received_data.com_x=RxBuffer[2]-received_data.sp_side;
		received_data.com_y=RxBuffer[3]-received_data.sp_side;
		received_data.com_z=RxBuffer[4];
		received_data.com_yaw=RxBuffer[5]-received_data.sp_side;
		received_data.next_task_sign=RxBuffer[6];
		pi_receive_done_sign=1;
	}
	else
	{
		state_1 = 0;
	}
}
void lidar_receive(u8 data)//激光雷达LDS-08接收 串口3//已废弃
{
	static u8 lidar_state=0;
	static u8 i=0;
	if(lidar_state==0&&data==0x54)
	{
		LidarBuffer[i]=data;
		lidar_state=1;
		i++;
	}
	if(lidar_state==1&&i==46)
	{
		LidarBuffer[i]=data;
		lidar_state=0;
		i=0;
	}
	else if(lidar_state==1)
	{
		LidarBuffer[i]=data;
		i++;
	}
}
void lidar_cal(struct lidar_data * lidar)//lidar缓存数据计算//已废弃
{
		lidar->lidar_speed=(int)LidarBuffer[3]*16+(int)LidarBuffer[2];
		lidar->start_angle=(int)LidarBuffer[5]*16+(int)LidarBuffer[4];
		lidar->end_angle=(int)LidarBuffer[43]*16+(int)LidarBuffer[42];
		lidar->timestamp=(int)LidarBuffer[45]*16+(int)LidarBuffer[44];
		lidar->crc=(int)LidarBuffer[46];
		for(int f=0;f<11;f+=1)
		{
			lidar->point_dis[f]=(int)LidarBuffer[7+f*3]*16+(int)LidarBuffer[6+f*3];
			lidar->point_credit[f]=(int)LidarBuffer[8+f*3];
		}
}
void PID_init()
{
	height_PID.p=2;
	height_PID.i=1.4;
	height_PID.d=0.8;
	height_PID.actual=0;
	height_PID.target=0;
	height_PID.err_current=0;
	height_PID.err_last=0;
	height_PID.err_previous=0;
	xy_PID.p=1.5;
	xy_PID.i=0.9;
	xy_PID.d=0.6;
	xy_PID.actual=0;
	xy_PID.target=0;
	xy_PID.err_current=0;
	xy_PID.err_last=0;
	xy_PID.err_previous=0;
}
s16 height_set(u32 height,u16 height_set)
{
	s16 output=0;
	height_PID.actual=height;
	height_PID.target=height_set; 
	height_PID.err_current=height_PID.target-height_PID.actual;
	output=height_PID.p*(height_PID.err_current-height_PID.err_last)+height_PID.i*height_PID.err_current+height_PID.d*(height_PID.err_current-2*height_PID.err_last+height_PID.err_previous);
	height_PID.err_previous=height_PID.err_last;
	height_PID.err_last=height_PID.err_current;
	if (output>30 ) output=30;
	else if(output<-30) output=-30;
	return output;
}
void pi_send()
{
	static u8 stage=0;
	if(stage==0)
	{
		USART_SendData(USART2,0xAA);
		stage=1;
		s16 num = ano_of.intergral_x + 0x4000;
		x_high=(num >> 8) & 0xFF;
		x_low=num & 0xFF;
		s16 ynum = ano_of.intergral_y + 0x4000;
		y_high=(ynum >> 8) & 0xFF;
		y_low=ynum & 0xFF;
	}
	else if(stage==1)
	{
		USART_SendData(USART2,mission_stage);
		stage=2;
	}
	else if(stage==2)
	{

		USART_SendData(USART2,x_high);
		//USART_SendData(USART2,0x05);
		stage=3;
	}
	else if(stage==3)
	{
		USART_SendData(USART2,x_low);
		stage=4;
	}
	else if(stage==4)
	{

		USART_SendData(USART2,y_high);
		//USART_SendData(USART2,0x05);
		stage=5;
	}
	else if(stage==5)
	{
		USART_SendData(USART2,y_low);
		stage=6;
	}
	else if(stage==6)
	{
		USART_SendData(USART2,0xFF);
		stage=0;
	}
	else stage=0;
}
void my_spcal(s16 x,s16 y)
{
        CSPX = y*0.3 ;
        CSPY = x*0.3 ;
	if(CSPX>30) CSPX=30;
	if(CSPX<-30) CSPX=-30;
	if(CSPY>30) CSPY=30;
	if(CSPY<-30) CSPY=-30;
}
s16 xypid_set(s32 xy,s16 xy_set,u16 speedmax)
{
	s16 output=0;
	xy_PID.actual=xy;
	xy_PID.target=xy_set; 
	xy_PID.err_current=xy_PID.target-xy_PID.actual;
	output=xy_PID.p*(xy_PID.err_current-xy_PID.err_last)+xy_PID.i*xy_PID.err_current+xy_PID.d*(xy_PID.err_current-2*xy_PID.err_last+xy_PID.err_previous);
	xy_PID.err_previous=xy_PID.err_last;
	xy_PID.err_last=xy_PID.err_current;
	if (output>speedmax ) output=speedmax;
	else if(output<-speedmax) output=-speedmax;
	return output;
}