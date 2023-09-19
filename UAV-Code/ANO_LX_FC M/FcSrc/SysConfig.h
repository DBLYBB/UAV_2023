#ifndef _SYSCONFIG_H_
#define _SYSCONFIG_H_
#include "McuConfig.h"
#include "Drv_BSP.h"
//================define===================
typedef float vec3_f[3];
typedef float vec2_f[2];
typedef s32 vec3_s32[3];
typedef s32 vec2_s32[2];
typedef s16 vec3_s16[3];
typedef s16 vec2_s16[2];
#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)
#define PWM_FRE_HZ 400
#define LED_NUM 4

#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

//================ϵͳ===================
#define HW_ALL 0xFF
#define SWJ_ADDR 0xAF
#define HW_TYPE 0x61
#define HW_VER 1
#define SOFT_VER 17
#define BL_VER 0
#define PT_VER 400

#define LED_R	0x01
#define LED_G	0x02
#define LED_B	0x04
#define LED_S	0x08
#define LED_ALL 0xFF

//#define GPS_USE_RTK
#define GPS_USE_UBLOX_M8

//=========================================
#endif
