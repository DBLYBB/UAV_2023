#ifndef __LX_FC_EXT_SENSOR_H
#define __LX_FC_EXT_SENSOR_H

//==引用
#include "SysConfig.h"

//==定义/声明
//====通用传感器数据====
typedef struct
{
	//
	vec3_s16 hca_velocity_cmps;

} __attribute__((__packed__)) _general_vel_st;

typedef struct
{
	//
	vec3_s32 ulhca_pos_cm;

} __attribute__((__packed__)) _general_pos_st;

typedef struct
{
	//
	u8 direction;
	u16 angle_100;
	s32 distance_cm;

} __attribute__((__packed__)) _general_dis_st;

typedef union {
	u8 byte[6];
	_general_vel_st st_data;
} _general_vel_un;

typedef union {
	u8 byte[12];
	_general_pos_st st_data;
} _general_pos_un;

typedef union {
	u8 byte[7];
	_general_dis_st st_data;
} _general_dis_un;
//====GPS数据====
typedef struct
{
	u8 FIX_STA;
	u8 S_NUM;
	s32 LNG;
	s32 LAT;
	s32 ALT_GPS;
	s16 N_SPE;
	s16 E_SPE;
	s16 D_SPE;
	u8 PDOP_001; //0.01f
	u8 SACC_001; //0.01f
	u8 VACC_001; //0.01f

} __attribute__((__packed__)) _fc_gps_st;

typedef union {
	u8 byte[23];
	_fc_gps_st st_data;
} _fc_gps_un;
//====

typedef struct
{
	//
	_general_vel_un gen_vel;
	_general_pos_un gen_pos;
	_general_dis_un gen_dis;
	_fc_gps_un fc_gps;

} _fc_ext_sensor_st;
//==数据声明
extern _fc_ext_sensor_st ext_sens;
//==函数声明
//static

//public
void LX_FC_EXT_Sensor_Task(float dT_s);

#endif
