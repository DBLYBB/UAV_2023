#include "my_fun.h"
void tar_setdata(s16 x,s16 y, s16 z,s16 yaw)//实时控制帧发送速度
	{
		rt_tar.st_data.vel_x=x;//头向速度 单位cm/s
		rt_tar.st_data.vel_y=y;//左正右负
		rt_tar.st_data.vel_z=z;//上下速度
		rt_tar.st_data.yaw_dps=yaw;//逆时针为正 单位度/秒
		dt.fun[0x41].WTS = 1;
} 

