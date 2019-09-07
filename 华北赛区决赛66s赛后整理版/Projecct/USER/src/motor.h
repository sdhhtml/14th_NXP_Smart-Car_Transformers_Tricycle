#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"
#define MOTOR_FTM   ftm3
#define Left_MOTOR1_PWM  ftm_ch1
#define Left_MOTOR2_PWM  ftm_ch2
#define Right_MOTOR1_PWM ftm_ch0
#define Right_MOTOR2_PWM ftm_ch5
#define MOTOR_HZ    13000	//滑行模式下，频率应该是 30~100。选用20kHz
				//常规模式下，频率应该是 20k 左右
#define MOTOR_MAX   800
#define Pulse_CM_coe    0.0086080744f            //厘米脉冲转化系数(编码器齿)
#define BodyR           10.0f                        //轴向半径
#define PAI             3.14159265358979f            //圆周率
extern uint8  Point;	//目标点横坐标
extern int8   Point_Num;  //提前系数
extern float SpeedRw, SpeedLw;

float range_protectfloat(float duty, float min, float max);//限幅保护
int range_protect(int32 duty, int32 min, int32 max);//限幅保护
void MotorInit(void);
void Speed_Measure(void);

extern char Crazy_Flag;
extern char Run_Flag;
extern int32   MOTOR_Speed_Right;        //FTM1  正值
extern int32   MOTOR_Speed_Left;         //FTM2 负值
extern int32 MOTOR_Duty_Left, MOTOR_Duty_Right;
extern int Fres;	// 动态参数变量
void MOTOR_Control(int32 LDuty, int32 RDuty);	// 电机控制
/*
11614,11583,11598
11697,11691,11694
11538,11580,11559
*/
#endif
