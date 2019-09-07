#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "headfile.h"
extern S_FLOAT_XYZ 
	GYRO_Real,		// ������ת���������
	ACC_Real,		// ���ٶȼ�ת���������
	Attitude_Angle,	        // ��ǰ�Ƕ� 
	Last_Angle,		// �ϴνǶ�
	Target_Angle,	        // Ŀ��Ƕ�
        Target_Angle_Grow;      // Ŀ��Ƕ�����
extern S_INT16_XYZ
	GYRO,			// ������ԭʼ����
	GYRO_Offset,	// ��������Ʈ
	GYRO_Last,		// �������ϴ�����
	ACC, 			// ���ٶȼ�����
	ACC_Offset,		// ���ٶȼ���Ʈ
	ACC_Last;		// ���ٶȼ��ϴ�����
extern S_INT32_XYZ
	Tar_Ang_Vel,	// Ŀ����ٶ�
	Tar_Ang_Vel_Last,	// �ϴ�Ŀ����ٶ�
        Tar_Ang_Vel_Grow;       // Ŀ����ٶ�����
extern int32 
        In_Out_Roundabout_I,
        In_Out_Roundabout_ture_I,
        Out_Roundabout_I,        //����������
        Run_I,
	Speed_Now,		// ��ǰʵ���ٶ�
        Speed_Now_Last,       // �ϴ�ʵ���ٶ�
	Speed_Min,		// ������С�ٶ�
	Speed_Set, 		// Ŀ���趨�ٶ�
	Vel_Set,		// Ŀ��ת����ٶ�
	Direct_Parameter,
        Direct_Last,
        Theory_Duty,
	Curvature;
extern short int point_center;
extern float TP;
void Balance_Control(void);
void Round_Control(void);//����2ms����
extern char Speed_Flag, Angle_Flag, Ang_Velocity_Flag;
extern float Zero_Angle,Zero_Angle2;
extern uint8 System_OK;
extern float Multiple_E;
extern float Curvature_E;
extern float point_center_E;
extern int QWG_E;
extern int Round_Increase;
void Vertical_Change_Three_Wheels(void);//ֱ��������
void Three_Wheeled_Upright(void);//���ֱ�ֱ��
void Round_Control_2(void);//ǿ���ҹ�
void Round_Control_3(void);//ǿ���ҹ�
	void Round_Control_4(void);//ǿ���ҹ�
	void Balance_Control_2(void);//ǿ���ҹ�
		void Balance_Control_3(void);//ǿ�����
			void Balance_Control_4(void);//ǿ���ҹ�
			void Balance_Control_PD(void);//PDר��
			void Balance_Control_DC(void);
#endif

