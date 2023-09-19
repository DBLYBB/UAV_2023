#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "SysConfig.h"

//注意，若打开ESC校准功能，将可能发生不可预料的损坏或者人身伤害，后果自负。
//一定需要校准时，请拆掉螺旋桨，尽量避免发生意外。
//校准ESC成功后，记得关闭此功能，避免出现意外。
#define ESC_CALI 0 //1：打开；0：关闭。

//#define PWM_FRE_HZ        400
//#define PWM_FRE_HZ        350    //天行者电调可以尝试用 350 hz

void DrvPwmOutInit(void);
void DrvMotorPWMSet(int16_t pwm[]); //范围0-1000

#endif
