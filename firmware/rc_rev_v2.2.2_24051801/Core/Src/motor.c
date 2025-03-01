#include "motor.h"

extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern void SoftPwmSetDuty(u8 ch,u16 dutyin);



MotorCtrl motorlist[MOTOR_BUTT] = {0};

u8 g_current10mA = 0;
u8 g_limit_state = 0;

void motorinit(void)
{
    for (int i = MOTOR_DIG_BUCKET; i < MOTOR_BUTT; i++)
    {
        motorlist[i].idx = i;
        if (i == MOTOR_DIG_BUCKET || i == MOTOR_SMALL_ARM || i == MOTOR_BIG_ARM)
            motorlist[i].StepPerTick = 30;
        else if(i == MOTOR_LEFT_TRACK || i == MOTOR_RIGHT_TRACK)
            motorlist[i].StepPerTick = 10;
        else
            motorlist[i].StepPerTick = 10;
    }

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0); // u2 1
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0); // u2 2
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0); // u3 1
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0); // u3 2

		#if 0
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0); // u6 1
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0); // u4 1
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0); // u7 1
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0); // u7 2
		#endif
		
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0); // u6 2
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0); // u5 2
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0); // u5 1
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0); // u4 2

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		#if 0
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		#endif
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// idx 0~5
// value -1000~1000

uint8_t checksum(uint8_t *buf, int len)
{
  uint8_t sum = 0;
  for (int i = 0; i < len; i++)
  {
    sum += buf[i];
  }
  return sum;
}




void motorOut(u8 idx, s16 motorval)
{
    u32 value = 0;
    u8 dir = 0;
    if (motorval >= 0)
    {
        value = motorval;
        dir = 0;
    }
    else
    {
        value = -motorval;
        dir = 1;
    }

    if (idx == MOTOR_DIG_BUCKET)
    {
        if (dir)
        {

            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, value);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
        }
        else
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, value);
        }
       
    }
    else if (idx == MOTOR_SMALL_ARM)
    {
        if (dir)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, value);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
        }
        else
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, value);
        }
    }
    else if (idx == MOTOR_BIG_ARM)
    {
        if (dir)
        {
            //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, value);
            SoftPwmSetDuty(1,value);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
        }
        else
        {
            //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
            SoftPwmSetDuty(1,0);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, value);
        }
    }
    else if (idx == MOTOR_TURN)
    {
        if (dir)
        {
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, value);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
        }
        else
        {
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, value);
        }
    }
    else if (idx == MOTOR_LEFT_TRACK)
    {
        if (dir)
        {
           // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, value);
           SoftPwmSetDuty(0,value);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
        }
        else
        {
            //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
            SoftPwmSetDuty(0,0);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, value);
        }
    }
    else if (idx == MOTOR_RIGHT_TRACK)
    {
        if (dir)
        {
            //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, value);
            //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
            SoftPwmSetDuty(2,value);
            SoftPwmSetDuty(3,0);
        }
        else
        {
           // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
            //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, value);
            SoftPwmSetDuty(2,0);
            SoftPwmSetDuty(3,value);
        }
    }
}

#if 0 // no use
// direct motor control
void joy2motor(u8 joystickvalue, u8 motoridx, u8 dirreverse, float current)
{
    u32 val = 0;
    u8 dir = 0;
    //static int dirlock = -1;

    if (joystickvalue > (128 + 20)) // 方向处理
    {
        val = joystickvalue - 128;
        dir = dirreverse == 1 ? 1 : 0;
    }
    else if (joystickvalue < (128 - 20))
    {
        val = 128 - joystickvalue;
        dir = dirreverse == 1 ? 0 : 1;
    }
    else
    {
        // 静态位置
    }

    // if(dir == dirlock)
    // return;

    val = val * 1000 / 128; // 0-128 放大到 0-1000
    if (current > 200)
    {
        val = 0;
        //dirlock = dir;
    }

    motorOut(motoridx, dir, val);
}
#endif

void motor_input_current(u8 current)
{
    g_current10mA = current;
}

void motor_set_exp(enum MotorIndex idx, u8 joystickvalue, u8 dirreverse)
{
    u32 val = 0;
    u8 dir = 0;
    MotorCtrl *motor = &motorlist[idx];

    //死区设定，防止摇杆轻度不归中导致电机运动
    if (joystickvalue > (128 + 40))  
    {
        val = joystickvalue - 128;
        dir = dirreverse == 1 ? 1 : 0;
    }
    else if (joystickvalue < (128 - 40))
    {
        val = 128 - joystickvalue;
        dir = dirreverse == 1 ? 0 : 1;
    }
    else
    {
        // 静态位置
    }

#if 0
    //pwm是 0-1000范围，协议是 0-128范围，此处做范围转化
    if (motor->idx == MOTOR_TURN)     
    {
        val = val * 500 / 128; // 0-128 放大到 0-400
    }
    else if(motor->idx == MOTOR_LEFT_TRACK || motor->idx == MOTOR_RIGHT_TRACK)
    {
        val = val * 500 / 128; // 0-128 放大到 0-400
    }
    else
#endif
    {
        val = val * 1000 / 128; // 0-128 放大到 0-1000
    }
    
    
    
    motor->exp = (dir == 0) ? val : -val;
}

/*
2、串口协议如下

用signed char传输推杆pwm -100 ~ 100

0x78  xx   yy    zz    sum
帧头    大臂  小臂  挖斗  校验和

3、开机  亮1秒 然后分别（闪1、2、3次）表示不同的程序

4、然后就是常亮，表示等待数据

5、接受到有效数据，就闪一次

6、串口参数为 115200 8 1 N
*/

extern UART_HandleTypeDef huart1;
s8 uart_cmd[5] = {0x78,0x00,0x00,0x00,0x00};
void uart_cmd_send(u8 idx,s16 value)
{   
    if(idx == MOTOR_BIG_ARM)
        uart_cmd[1] = value / 10;
    else if(idx == MOTOR_SMALL_ARM)
        uart_cmd[2] = value / 10;
    else if(idx == MOTOR_DIG_BUCKET)
        uart_cmd[3] = value / 10;
    
    //check sum
    u8 sum = checksum(uart_cmd,4);
    uart_cmd[4] = sum;

    HAL_UART_Transmit_IT(&huart1, uart_cmd, 5);
}

void motor_update(MotorCtrl *motor)
{
#if 1
    // 锁定判断，如果某方向锁定，就不再启动电机

    if (motor->dirlock == LIMIT_LOW && motor->exp > 0)
    {
        motor->exp = 0;
        goto out;
    }
    if (motor->dirlock == LIMIT_HIGH && motor->exp < 0)
    {
        motor->exp = 0;
        goto out;
    }

#endif
    if (motor->exp != motor->real)
    {

        // 停止的时候快速停止，体验更好
        if (motor->exp == 0)
        {
            motor->real = 0;
            goto out;
        }

        // 加速的时候则是慢慢地加速，根据StepPerTick设定加速度
        if ((motor->exp - motor->real) > motor->StepPerTick)
        {
            motor->real += motor->StepPerTick;
        }
        else if ((motor->real - motor->exp) > motor->StepPerTick)
        {
            motor->real -= motor->StepPerTick;
        }
        else
        {
out:
            motor->real = motor->exp;
        }

        // 输出到tim pwm 通道
        motorOut(motor->idx, motor->real);
        //uart_cmd_send(motor->idx,motor->real);
    }
}

#if 0
// 当 电流 > 500mA 且 持续时间超过 100ms 认为过载，这时候，扫描所有的电机结构体，那个real最大，那个就判定为过载
// 当 电流 > 50mA 且 持续时间超过 200ms 认为过载解除，这时候，扫描所有的电机结构体，过载的电机的过载反方向real>300 则过载解除，如果条件不存在，认为并不是过载电机
void motor_limit_check(void)
{
    static uint32_t lasttick = 0;

    if (g_current10mA >= 50)
    {
        if (lasttick == 0)
            lasttick = HAL_GetTick();
        uint32_t now = HAL_GetTick();

        SEGGER_RTT_printf(0, "A: tim:%d\r\n", abs(now - lasttick));

        if (abs(now - lasttick) > 400) // 检查到有电机过载
        {
            lasttick = 0;
            u16 max_real = 0;
            u8 max_index = 0;
            for (int i = MOTOR_DIG_BUCKET; i < MOTOR_BUTT; i++)
            {
                if (abs(motorlist[i].real) > max_real)
                {
                    max_real = abs(motorlist[i].real);
                    max_index = i;
                }
            }
            motorlist[max_index].dirlock = (motorlist[max_index].real > 0) ? LIMIT_LOW : LIMIT_HIGH;
        }
    }
    else if (g_current10mA > 15 && g_current10mA < 50)
    {
        if (lasttick == 0)
            lasttick = HAL_GetTick();
        uint32_t now = HAL_GetTick();

        SEGGER_RTT_printf(0, "B: tim:%d\r\n", abs(now - lasttick));
        if (abs(now - lasttick) > 100) // 检查到有电机过载解除
        {
            lasttick = 0;
            for (int i = MOTOR_DIG_BUCKET; i < MOTOR_BUTT; i++)
            {
                if (motorlist[i].dirlock != LIMIT_NONE)
                {
                    if (abs(motorlist[i].real) > 100)
                    {
                        motorlist[i].dirlock = LIMIT_NONE;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        lasttick = 0;
    }
}
#else
void motor_limit_check(void)
{
    if (g_limit_state & LIMIT_MIN3_TRIG)
        motorlist[MOTOR_DIG_BUCKET].dirlock = LIMIT_HIGH;
    else if (g_limit_state & LIMIT_MAX3_TRIG)
        motorlist[MOTOR_DIG_BUCKET].dirlock = LIMIT_LOW;
    else
        motorlist[MOTOR_DIG_BUCKET].dirlock = LIMIT_NONE;

    if (g_limit_state & LIMIT_MIN1_TRIG)
        motorlist[MOTOR_SMALL_ARM].dirlock = LIMIT_HIGH;
    else if (g_limit_state & LIMIT_MAX1_TRIG)
        motorlist[MOTOR_SMALL_ARM].dirlock = LIMIT_LOW;
    else
        motorlist[MOTOR_SMALL_ARM].dirlock = LIMIT_NONE;

    if (g_limit_state & LIMIT_MIN2_TRIG)
        motorlist[MOTOR_BIG_ARM].dirlock = LIMIT_HIGH;
    else if (g_limit_state & LIMIT_MAX2_TRIG)
        motorlist[MOTOR_BIG_ARM].dirlock = LIMIT_LOW;
    else
        motorlist[MOTOR_BIG_ARM].dirlock = LIMIT_NONE;
}
#endif

// this function need put in tim interrupt
void motor_service(void)
{
    for (int i = MOTOR_DIG_BUCKET; i < MOTOR_BUTT; i++)
    {
        motor_update(&motorlist[i]);
    }
    motor_limit_check();

}







