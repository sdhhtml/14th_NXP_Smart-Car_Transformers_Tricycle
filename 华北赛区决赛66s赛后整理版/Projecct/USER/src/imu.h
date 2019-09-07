#ifndef _IMU_H_
#define _IMU_H_

#include "headfile.h"

extern void Balance_Sensor_Init(void) ;
extern void Balance_Sensor_Calculate(void) ;
extern void Sensor_Offset(void);
extern void Data_Filter(void);
extern void Data_Filter2(void);	// 数据滤波
extern float GravityAngle;
extern void Get_Attitude(void);
extern float Kalman_Filter(float angle_m,float gyro_m);

extern float angle_y;    
//extern float Gyro_int;
extern float GravityAngle ; 
extern float AngleSpeed_Y ; 
extern float Gyrox_int;
//直立控制
typedef struct
{
	float X;
	float Y;
	float Z;
} S_FLOAT_XYZ;

typedef struct
{
	int32 X;
	int32 Y;
	int32 Z;
} S_INT32_XYZ;

typedef struct
{
	int16 X;
	int16 Y;
	int16 Z;
} S_INT16_XYZ;
#endif

