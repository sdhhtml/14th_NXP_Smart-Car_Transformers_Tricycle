#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "headfile.h"
extern S_FLOAT_XYZ 
	GYRO_Real,		// 陀螺仪转化后的数据
	ACC_Real,		// 加速度计转化后的数据
	Attitude_Angle,	        // 当前角度 
	Last_Angle,		// 上次角度
	Target_Angle,	        // 目标角度
        Target_Angle_Grow;      // 目标角度增长
extern S_INT16_XYZ
	GYRO,			// 陀螺仪原始数据
	GYRO_Offset,	// 陀螺仪温飘
	GYRO_Last,		// 陀螺仪上次数据
	ACC, 			// 加速度计数据
	ACC_Offset,		// 加速度计温飘
	ACC_Last;		// 加速度计上次数据
extern S_INT32_XYZ
	Tar_Ang_Vel,	// 目标角速度
	Tar_Ang_Vel_Last,	// 上次目标角速度
        Tar_Ang_Vel_Grow;       // 目标角速度增长
extern int32 
        In_Out_Roundabout_I,
        In_Out_Roundabout_ture_I,
        Out_Roundabout_I,        //出环岛距离
        Run_I,
	Speed_Now,		// 当前实际速度
        Speed_Now_Last,       // 上次实际速度
	Speed_Min,		// 左右最小速度
	Speed_Set, 		// 目标设定速度
	Vel_Set,		// 目标转向角速度
	Direct_Parameter,
        Direct_Last,
        Theory_Duty,
	Curvature;
extern short int point_center;
extern float TP;
void Balance_Control(void);
void Round_Control(void);//三轮2ms控制
extern char Speed_Flag, Angle_Flag, Ang_Velocity_Flag;
extern float Zero_Angle,Zero_Angle2;
extern uint8 System_OK;
extern float Multiple_E;
extern float Curvature_E;
extern float point_center_E;
extern int QWG_E;
extern int Round_Increase;
void Vertical_Change_Three_Wheels(void);//直立变三轮
void Three_Wheeled_Upright(void);//三轮变直立
void Round_Control_2(void);//强制右拐
void Round_Control_3(void);//强制右拐
	void Round_Control_4(void);//强制右拐
	void Balance_Control_2(void);//强制右拐
		void Balance_Control_3(void);//强制左拐
			void Balance_Control_4(void);//强制右拐
			void Balance_Control_PD(void);//PD专用
			void Balance_Control_DC(void);
#endif

