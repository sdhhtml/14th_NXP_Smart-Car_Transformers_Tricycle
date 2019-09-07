#include "headfile.h"
PID MOTOR_PID, Angle_PID, Ang_Vel_PID, Direct_PID, Turn_PID, Distance_PID,Roundspeed_PID,Roundangle_PID,Rounddirect_PID;//�����ٶ� �Ƕ� ���ٶ� ���� λ��
//float MOTOR[4]   = {16.042, 0, 0, 1000};		// �ٶȻ�PID	���һ��Ϊ�����޷� PD
//float Angle[4]   = {0.736, 0, 0.155, 500};		// �ǶȻ�PID
//float Ang_Vel[4] = {0.870, 0.024, 0, 1000};		// ���ٶȻ�PID
//float Direct[4]  = {0.03, 0, 0,70};	// ת��PID λ��	0.017	0.02
float MOTOR[4]   = {50, 0.05, 0, 1000};		// �ٶȻ�PID	���һ��Ϊ�����޷�
float Angle[4]   = {0.15, 0, 0.06, 500};		// �ǶȻ�PID

float Ang_Vel[4] = {0.25, 0.023, 0, 1000};		// ���ٶȻ�PID
float Direct[4]  = {0.033, 0, 0.03, 70};	// ת��PID λ��	0.017	0.02

float Round[4]  = {5,0, 0, 70};	//	
float Round_Angle[4]  = {3, 0.1, 0, 70};	// 
float Round_Direct[4]  = {0.036, 0, 0.03, 70};	

float Angle_up[4]   = {0.3, 0, 0.6, 500};		// �ǶȻ�PID
float Ang_Vel_up[4] = {0.25, 0.023, 0, 1000};

float MOTOR_PD[4]   = {15, 0, 0, 1000};		// �ٶȻ�PID	���һ��Ϊ�����޷�
float Angle_PD[4]   = {0.15, 0, 0.06, 500};		// �ǶȻ�PID

float Ang_Vel_PD[4] = {0.205, 0.023, 0, 1000};		// ���ٶȻ�PID
float Direct_PD[4]  = {0.033, 0, 0.03, 70};	// ת��PID λ��	0.017	0.02
/*

*/
/*******ת���⻷��̬PID	���߷�********/
float Turn[5][4] = {{100, 3, 2, 100},   //��ת�����PID
		    {100, 3, 2, 100},   //����ת�����PID	
		    {100, 4, 3, 100},   //����ת�����PID	
		    {100, 5, 4, 100},   //����ת�����PID	
		    {200, 10, 8, 100}};  //����ת�����PID
float TURN_lim=165;//ת���⻷�������
float  Difference = 0,Differencelast=0;
// PID������ʼ��
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}
// λ��ʽ��̬PID����
int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
{
	//����Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	//��ǰ���
		  Actual;	//���ó���ʵ�����ֵ
	float Kp;		//��̬P
	iError = SetPoint - NowPiont;	//���㵱ǰ���
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <=-PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
          
	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//Pֵ���ֵ�ɶ��κ�����ϵ���˴�P��I����PID���������Ƕ�̬PID������Ҫע�⣡����
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//ֻ��PD
	sprt->LastError = iError;		//�����ϴ����

//	Actual += sprt->SumError*0.1;
	Actual = range_protect(Actual, -TURN_lim, TURN_lim);

	return Actual;
}

//ת��λ��ʽPID����
int32 PID_Realize_D(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	// ��ǰ���
		 Realize;	// ���ó���ʵ������
        float SetD;
	iError = Point - NowData;	// ���㵱ǰ���
	sptr->SumError += PID[KI] * iError;	// ������
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}
        SetD =PID[KD];
          
        if(Difference*GYRO_Real.X>0)
         SetD =0;
//        Realize = PID[KP] * iError
//			+ sptr->SumError
//			+ SetD * GYRO_Real.Z;
	Realize = PID[KP] * iError
			+ sptr->SumError
			+ SetD * (iError - sptr->LastError);
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����

	return Realize;	// ����ʵ��ֵ
}
// λ��ʽPID����
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	// ��ǰ���
		 Realize;	// ���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���
	sptr->SumError += PID[KI] * iError;	// ������
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����

	return Realize;	// ����ʵ��ֵ
}
// ����ʽPID�������
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	//��ǰ���
		Increase;	//���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���
	Increase =  PID[KP] * (iError - sptr->LastError)
			  + PID[KI] * iError
			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
	
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����
	
	return Increase;	// ��������
}//*/

