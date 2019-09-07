#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"
#define MOTOR_FTM   ftm3
#define Left_MOTOR1_PWM  ftm_ch1
#define Left_MOTOR2_PWM  ftm_ch2
#define Right_MOTOR1_PWM ftm_ch0
#define Right_MOTOR2_PWM ftm_ch5
#define MOTOR_HZ    13000	//����ģʽ�£�Ƶ��Ӧ���� 30~100��ѡ��20kHz
				//����ģʽ�£�Ƶ��Ӧ���� 20k ����
#define MOTOR_MAX   800
#define Pulse_CM_coe    0.0086080744f            //��������ת��ϵ��(��������)
#define BodyR           10.0f                        //����뾶
#define PAI             3.14159265358979f            //Բ����
extern uint8  Point;	//Ŀ��������
extern int8   Point_Num;  //��ǰϵ��
extern float SpeedRw, SpeedLw;

float range_protectfloat(float duty, float min, float max);//�޷�����
int range_protect(int32 duty, int32 min, int32 max);//�޷�����
void MotorInit(void);
void Speed_Measure(void);

extern char Crazy_Flag;
extern char Run_Flag;
extern int32   MOTOR_Speed_Right;        //FTM1  ��ֵ
extern int32   MOTOR_Speed_Left;         //FTM2 ��ֵ
extern int32 MOTOR_Duty_Left, MOTOR_Duty_Right;
extern int Fres;	// ��̬��������
void MOTOR_Control(int32 LDuty, int32 RDuty);	// �������
/*
11614,11583,11598
11697,11691,11694
11538,11580,11559
*/
#endif
