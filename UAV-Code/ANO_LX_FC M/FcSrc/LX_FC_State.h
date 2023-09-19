#ifndef __LX_FC_STATE_H
#define __LX_FC_STATE_H

//==引用
#include "SysConfig.h"

//==定义/声明
typedef struct
{
	u8 pre_locking;
	u8 stick_mit_pos;

} _sticks_fun_st;

typedef struct
{
	u8 CID;
	u8 CMD_0;
	u8 CMD_1;
} _cmd_fun_st;
//飞控状态
typedef struct
{
	//模式
	u8 fc_mode_cmd;
	u8 fc_mode_sta;

	//解锁上锁
	u8 unlock_cmd;
	u8 unlock_sta;

	//指令功能
	_cmd_fun_st cmd_fun;

	//state
	u8 imu_ready;
	u8 take_off;
	u8 in_air;
	u8 landing;

} _fc_state_st;

//==数据声明
extern _fc_state_st fc_sta;
extern _sticks_fun_st sti_fun;
//==函数声明
//static

//public
void LX_FC_State_Task(float dT_s);

#endif
