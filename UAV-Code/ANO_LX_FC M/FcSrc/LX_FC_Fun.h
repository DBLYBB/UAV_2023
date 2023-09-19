#ifndef __LX_FC_FUN_H
#define __LX_FC_FUN_H

//==引用
#include "SysConfig.h"

//==定义/声明

//==数据声明

//==函数声明
//static

//public
u8 FC_Unlock(void);
u8 FC_Lock(void);
u8 LX_Change_Mode(u8 new_mode);
u8 OneKey_Takeoff(u16 height_cm);
u8 OneKey_Land(void);
u8 OneKey_Flip(void);
u8 OneKey_Return_Home(void);
u8 Horizontal_Calibrate(void);
u8 Horizontal_Move(u16 distance_cm, u16 velocity_cmps, u16 dir_angle_0_360);
u8 Mag_Calibrate(void);
u8 ACC_Calibrate(void);
u8 GYR_Calibrate(void);
#endif
