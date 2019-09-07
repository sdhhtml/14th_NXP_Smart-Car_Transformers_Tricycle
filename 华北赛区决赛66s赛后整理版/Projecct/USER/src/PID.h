#ifndef _PID_H_
#define _PID_H_

#include "headfile.h"
#define KP 0
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5

typedef struct PID
{
	float SumError;	//����ۼ�	
	int32 LastError;	//Error[-1]
	int32 PrevError;	//Error[-2]	
	int32 LastData;	//Speed[-1]
} PID;
extern float MOTOR[4], Angle[4], Ang_Vel[4], Direct[4], Turn[5][4], Distance[4];
extern float MOTOR_PD[4],Angle_PD[4],Ang_Vel_PD[4],Direct_PD[4] ;
extern float TURN_lim;
extern float  Difference,Differencelast;
extern float Round[4],Round_Angle[4],Round_Direct[4];	//����
extern float Angle_up[4],Ang_Vel_up[4];
// PID������ʼ��
void PID_Parameter_Init(PID *sptr);


// λ��ʽ��̬PID����
int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint);
// ת��λ��ʽPID����
int32 PID_Realize_D(PID *sptr, float *PID, int32 NowData, int32 Point);
// λ��ʽPID����
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);
// ����ʽPID����
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point);
extern PID MOTOR_PID, Angle_PID, Ang_Vel_PID, Direct_PID, Turn_PID, Distance_PID,Roundspeed_PID,Roundangle_PID,Rounddirect_PID;//�����ٶ� �Ƕ� ���ٶ� ���� λ��
#endif
