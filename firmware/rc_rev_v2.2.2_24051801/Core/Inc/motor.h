#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "common.h"


#define LIMIT_LOW 1
#define LIMIT_HIGH 2
#define LIMIT_NONE 0


#define LIMIT_MAX1_TRIG 0x01
#define LIMIT_MIN1_TRIG 0x02
#define LIMIT_MAX2_TRIG 0x04
#define LIMIT_MIN2_TRIG 0x08
#define LIMIT_MAX3_TRIG 0x10
#define LIMIT_MIN3_TRIG 0x20

enum MotorIndex{
    MOTOR_DIG_BUCKET=0,
    MOTOR_SMALL_ARM,
    MOTOR_BIG_ARM,
    MOTOR_TURN,
    MOTOR_LEFT_TRACK,
    MOTOR_RIGHT_TRACK,
    MOTOR_BUTT
};


typedef struct MotorCtrl_s{
    s16 exp; //-1000 ~ 1000 range
    s16 real;
    
    u8 StepPerTick; 
    u8 idx;
    
    //u8 maxi;//max current limit , 10mA unit
    u8 dirlock; //超过电流，表示限位到了，所以dirlock

}MotorCtrl;

extern MotorCtrl motorlist[MOTOR_BUTT];
extern u8 g_limit_state;


void motorinit(void);
//void joy2motor(u8 joystickvalue, u8 motoridx, u8 dirreverse, float current);
void motor_set_exp(enum MotorIndex idx, u8 joystickvalue, u8 dirreverse);
void motor_input_current(u8 current);

#endif
