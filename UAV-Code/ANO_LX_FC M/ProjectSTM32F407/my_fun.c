#include "my_fun.h"
void tar_setdata(s16 x,s16 y, s16 z,s16 yaw)//ʵʱ����֡�����ٶ�
	{
		rt_tar.st_data.vel_x=x;//ͷ���ٶ� ��λcm/s
		rt_tar.st_data.vel_y=y;//�����Ҹ�
		rt_tar.st_data.vel_z=z;//�����ٶ�
		rt_tar.st_data.yaw_dps=yaw;//��ʱ��Ϊ�� ��λ��/��
		dt.fun[0x41].WTS = 1;
} 

