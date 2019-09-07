#include "headfile.h"
void Control_Model_0(void)//比赛用的直立发车＋变形 直立障碍
{
	if(Flag_Running_State==0)//直立
	{
		Erect_Obstacle();//直立障碍
	}
	else if(Flag_Running_State==1)//三轮
	{
		Round_PT();//三轮普通带变形
	}
	else if(Flag_Running_State==2)//直立变三轮
	{
		Erect_Round();//直立变三轮
	}
	else if(Flag_Running_State==3)//三轮变直立
	{
		Round_Erect();//三轮变直立
	}
}
void Control_Model_1(void)//练习用的三轮发车＋变形 直立障碍
{
	if(Flag_Running_State==0)//直立
	{
		Erect_Obstacle();//直立障碍
	}
	else if(Flag_Running_State==1)//三轮
	{
		Round_PT();//三轮普通带变形
	}
	else if(Flag_Running_State==2)//直立变三轮
	{
		Erect_Round();//直立变三轮
	}
	else if(Flag_Running_State==3)//三轮变直立
	{
		Round_Erect();//三轮变直立
	}
}
void Control_Model_4(void)//比赛用的直立发车＋变形 三轮障碍
{
	if(Flag_Running_State==0)//直立
	{
		Erect_PT();//直立普通带变形
	}
	else if(Flag_Running_State==1)//三轮
	{
		Round_Obstacle();//三轮避障带变形
	}
	else if(Flag_Running_State==2)//直立变三轮
	{
		Erect_Round();//直立变三轮
	}
	else if(Flag_Running_State==3)//三轮变直立
	{
		Round_Erect();//三轮变直立
	}
}
void Control_Model_5(void)//练习用的三轮发车＋变形 三轮障碍
{
	if(Flag_Running_State==0)//直立
	{
		Erect_PT();//直立普通带变形
	}
	else if(Flag_Running_State==1)//三轮
	{
		Round_Obstacle();//三轮避障带变形
	}
	else if(Flag_Running_State==2)//直立变三轮
	{
		Erect_Round();//直立变三轮
	}
	else if(Flag_Running_State==3)//三轮变直立
	{
		Round_Erect();//三轮变直立
	}
}
void Control_Model_6(void)//直立避障不变形
{
			if(Flag_Zhi_ZA==0||Flag_Zhi_ZA==1)
		{
			Balance_Control();	// 车身平衡控制 
		}
		else if(Flag_Zhi_ZA==2)//强制右拐
		{
			Balance_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<-50)
			{
				Flag_Zhi_ZA=3;
			}
		}
		else if(Flag_Zhi_ZA==3)//强制左拐
		{
			Balance_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z>55)
			{
				Flag_Zhi_ZA=4;
			}
		}
		else if(Flag_Zhi_ZA==4)//强制右拐
		{
			Balance_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<0)
			{
				Gyro_R_Z=0;
				Flag_Zhi_ZA=0;
			}
		}
		if(Flag_R_ZA==2&&ABS(Difference)<10&&Flag_7H==1&&In_Roundabout_Flag==0)
		{
			Flag_R_ZA=0;
			Flag_Zhi_ZA=2;
			In_Roundabout_Flag=0;
		}
}
void Control_Model_7(void)
{
	if(Flag_Round_ZA==0||Flag_Round_ZA==1)
		{
			Round_Control();	// 车身平衡控制 
		}
		else if(Flag_Round_ZA==2)//强制右拐
		{
			Round_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<-50)
			{
				Flag_Round_ZA=3;
			}
		}
		else if(Flag_Round_ZA==3)//强制左拐
		{
			Round_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z>50)
			{
				Flag_Round_ZA=4;
			}
		}
		else if(Flag_Round_ZA==4)//强制右拐
		{
			Round_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<0)
			{
				Flag_Round_ZA=0;
				Gyro_R_Z=0;
			}
		}
		if(Flag_R_ZA==2&&ABS(Difference)<20&&Flag_7H==1&&(In_Roundabout_Flag==0||In_Roundabout_Flag==1||In_Roundabout_Flag==2||In_Roundabout_Flag==3))
		{
			Flag_R_ZA=0;
			Flag_Round_ZA=2;
			In_Roundabout_Flag=0;
		}
}
void Erect_Obstacle(void)//直立障碍
{
		if(Flag_Zhi_ZA==0||Flag_Zhi_ZA==1)
		{
			if(Flag_DC_Control==0)
			{
				Balance_Control();	// 车身平衡控制 
			}
			else//Flag_DC_Control==1
			{
				Gyro_R_Z+=0.002*GYRO_Real.X;
				Balance_Control_DC();	// 车身平衡控制 
				Flag_LZCnt_10ms++;
				if(Flag_LZCnt_10ms>4)
				{
					Flag_LZCnt_10ms=0;
					Speed_LzSum+=Speed_Now;
					if(Speed_LzSum<-6000)//倒车60cm
					{
						Speed_LzSum=0;
						Flag_DC_Control=0;
						Flag_Zhi_ZA=2;
					}
				}
			}
			if(gpio_get(B22)==0)//0白1蓝
			{
				if(Flag_Blue==1)
				{
					Flag_Blue=0;
					Count_Blue=0;
				}
				else if(Flag_Blue==22)
				{
					Count_Blue++;
					{
						if(Count_Blue>30)
						{
							Flag_Blue=0;
							Count_Blue=0;
						}
					}
				}
			}
			else//蓝 高电平
			{
				if(Flag_Blue==0)
				{
					Flag_Blue=1;
				}
				else if(Count_Blue<200&&Flag_Blue==1)//400ms大概40cm不间断
				{
					Count_Blue++;
				}
				else if(Count_Blue>=200&&Flag_Blue==1)
				{
					Flag_Blue=21;
					Count_Blue=0;
					Flag_Running_State=2;
				}
			}
		}
		else if(Flag_Zhi_ZA==2)//强制右拐
		{
			Balance_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<-50)
			{
				Flag_Zhi_ZA=3;
			}
		}
		else if(Flag_Zhi_ZA==3)//强制左拐
		{
			Balance_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z>55)
			{
				Flag_Zhi_ZA=4;
			}
		}
		else if(Flag_Zhi_ZA==4)//强制右拐
		{
			Balance_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<0)
			{
				Gyro_R_Z=0;
				Flag_Zhi_ZA=0;
			}
		}
		if(Flag_R_ZA==2&&ABS(Difference)<15&&Flag_7H==1&&In_Roundabout_Flag==0&&Flag_PD_QP==1)
		{
			Flag_R_ZA=0;
			Flag_Zhi_ZA=2;
			In_Roundabout_Flag=0;
		}
}
void Erect_PT(void)//直立加变形不避障
{
			Balance_Control();	// 车身平衡控制 
			if(gpio_get(B22)==0)//0白1蓝
			{
				if(Flag_Blue==1)
				{
					Flag_Blue=0;
					Count_Blue=0;
				}
				else if(Flag_Blue==22)
				{
					Count_Blue++;
					{
						if(Count_Blue>30)
						{
							Flag_Blue=0;
							Count_Blue=0;
						}
					}
				}
			}
			else//蓝 高电平
			{
				if(Flag_Blue==0)
				{
					Flag_Blue=1;
				}
				else if(Count_Blue<550&&Flag_Blue==1)//400ms大概40cm不间断
				{
					Count_Blue++;
				}
				else if(Count_Blue>=550&&Flag_Blue==1)
				{
					Flag_Blue=21;
					Count_Blue=0;
					Flag_Running_State=2;
				}
			}
}
unsigned char Flag_R_CP=0;
unsigned char Flag_PDWP=0;
void Round_Obstacle(void)//三轮障碍
{
	if(Flag_Round_ZA==0||Flag_Round_ZA==1)
		{
			Round_Control();	// 车身平衡控制 
			if(gpio_get(B22)==0)//0白1蓝
			{
				if(Flag_Blue==1)
				{
					Flag_Blue=0;
					Count_Blue=0;
				}
				else if(Flag_Blue==21)
				{
					Count_Blue++;
					{
						if(Count_Blue>30)
						{
							Flag_Blue=0;
							Count_Blue=0;
						}
					}
				}
			}
			else//蓝
			{
				if(Flag_Blue==0)
				{
					Flag_Blue=1;
				}
				if(Count_Blue<500&&Flag_Blue==1)//100ms大概10cm不间断
				{
					Count_Blue++;
				}
				if(Count_Blue>=500&&Flag_Blue==1)
				{
					Flag_Blue=22;
					Count_Blue=0;
					Flag_Running_State=3;
				}
			}
		}
		else if(Flag_Round_ZA==2)//强制右拐
		{
			Round_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			//if(Gyro_R_Z<-50)
			if(Gyro_R_Z>50)//左拐50度
			{
				Flag_Round_ZA=3;
			}
		}
		else if(Flag_Round_ZA==3)//强制左拐
		{
			Round_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			//if(Gyro_R_Z>50)
			if(Gyro_R_Z<-50)//右拐到50度
			{
				Flag_Round_ZA=4;
			}
		}
		else if(Flag_Round_ZA==4)//强制右拐
		{
			Round_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			//if(Gyro_R_Z<0)
			if(Gyro_R_Z>0)//左拐到0度
			{
				Flag_Round_ZA=0;
				Gyro_R_Z=0;
				Flag_R_CP=1;
				Flag_PDWP=1;
			}
		}
		if(Flag_R_ZA==2&&ABS(Difference)<20&&Flag_7H==1&&(In_Roundabout_Flag==0||In_Roundabout_Flag==1||In_Roundabout_Flag==2||In_Roundabout_Flag==3))
		{
			if(Flag_PDWP==0)
			{
				Flag_R_ZA=0;
				Flag_Round_ZA=2;
				In_Roundabout_Flag=0;
			}
		}
}
void Erect_Round(void)//直立变三轮
{
			Count_Blue++;
			if(Count_Blue<50)
			{
				Vertical_Change_Three_Wheels();
			}
			else if(Count_Blue<300)
			{
				Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
				MOTOR_Control(0, 0);
			}
			else
			{
				//Get_Attitude();	// 姿态解算
				//Attitude_Angle.Y=3;
				PID_Parameter_Init(&Roundspeed_PID);	
				PID_Parameter_Init(&Roundangle_PID);
				PID_Parameter_Init(&Rounddirect_PID);
				Speed_Measure();
				Speed_Now=0;
				Count_Blue=0;
				flag_10ms=0;
				flag_100ms=0;
				Flag_Running_State=1;
				Roundabout_cnt=0,Roundabin_cnt=0,In_Roundabout_Flag=0,In_Roundabout_Flag_O=0,Out_Roundabout_Flag=0;
				Flag_R_ZA=0;
				Flag_Round_ZA=0;
				Flag_7H=0;
				Speed_Set=140;
			}
}
void Round_Erect(void)//三轮变直立
{
		Count_Blue++;
		if(Count_Blue<400)
		{
				Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
				MOTOR_Control(-200,-200);
		}
		else if(Count_Blue<500)
		{
			Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
				MOTOR_Control(0,0);
		}
		else if(Count_Blue<1000)
		{
			Balance_Sensor_Calculate();
			Get_Attitude();	// 姿态解算
			MOTOR_Duty_Left=(33-Attitude_Angle.Y)*2500+6*GYRO_Real.Y;
			MOTOR_Duty_Right=MOTOR_Duty_Left;
			MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);
			//Three_Wheeled_Upright();
		}
		else if(Count_Blue>=600)
		{
			Balance_Sensor_Calculate();
			Get_Attitude();	// 姿态解算
			Count_Blue=0;
			Speed_Measure();
			Speed_Now=0;
			PID_Parameter_Init(&MOTOR_PID);	// 速度环PID参数初始化
			PID_Parameter_Init(&Angle_PID);	// 角度环PID参数初始化
			PID_Parameter_Init(&Ang_Vel_PID);	// 角速度环PID参数初始化
			PID_Parameter_Init(&Direct_PID);	// 转向环PID参数初始化
			PID_Parameter_Init(&Distance_PID);	// 位置环PID参数初始化
			Flag_Running_State=0;
			Flag_R_ZA=0;
			Flag_Round_ZA=0;
			Flag_7H=0;
			Roundabout_cnt=0,Roundabin_cnt=0,In_Roundabout_Flag=0,In_Roundabout_Flag_O=0,Out_Roundabout_Flag=0;
		}
}
void Erect(void)//直立普通变形
{
	
}
void Round_PT(void)//三轮普通带变形
{
		Round_Control();	// 三轮控制
		if(gpio_get(B22)==0)//0白1蓝
		{
			if(Flag_Blue==1)
			{
				Flag_Blue=0;
				Count_Blue=0;
			}
			else if(Flag_Blue==21)
			{
				Count_Blue++;
				{
					if(Count_Blue>30)
					{
						Flag_Blue=0;
						Count_Blue=0;
					}
				}
			}
		}
		else//蓝
		{
			if(Flag_Blue==0)
			{
				Flag_Blue=1;
			}
			if(Count_Blue<100&&Flag_Blue==1)//100ms大概10cm不间断
			{
				Count_Blue++;
			}
			if(Count_Blue>=100&&Flag_Blue==1)
			{
				Flag_Blue=22;
				Count_Blue=0;
				Flag_Running_State=3;
			}
		}
}
/*
	if(Flag_Running_State==0)//直立
	{
		Balance_Control();	// 车身平衡控制 
		if(gpio_get(B22)==0)//0白1蓝
		{
			if(Flag_Blue==1)
			{
				Flag_Blue=0;
				Count_Blue=0;
			}
			else if(Flag_Blue==22)
			{
				Count_Blue++;
				{
					if(Count_Blue>30)
					{
						Flag_Blue=0;
						Count_Blue=0;
					}
				}
			}
		}
		else//蓝
		{
			if(Flag_Blue==0)
			{
				Flag_Blue=1;
			}
			else if(Count_Blue<200&&Flag_Blue==1)//100ms大概10cm不间断
			{
				Count_Blue++;
			}
			else if(Count_Blue>=200&&Flag_Blue==1)
			{
				Flag_Blue=21;
				Count_Blue=0;
				Flag_Running_State=2;
			}
		}
	}
	else if(Flag_Running_State==1)//三轮
	{
				if(Flag_Round_ZA==0||Flag_Round_ZA==1)
		{
			Round_Control();	// 车身平衡控制 
		}
		else if(Flag_Round_ZA==2)//强制右拐
		{
			//flag_2ms_R_ZA++;
			Round_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<-60)
			{
				
				flag_2ms_R_ZA=0;
				Flag_Round_ZA=3;
			}
		}
		else if(Flag_Round_ZA==3)//强制左拐
		{
			//flag_2ms_R_ZA++;
			Round_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z>60)
			{
				flag_2ms_R_ZA=0;
				Flag_Round_ZA=4;
			}
		}
		else if(Flag_Round_ZA==4)//强制右拐
		{
			//flag_2ms_R_ZA++;
			Round_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<0)
			{
				flag_2ms_R_ZA=0;
				Flag_Round_ZA=0;
			}
		}
		flag_50ms_R_ZA++;
//		if(flag_50ms_R_ZA>25)
//		{
//			flag_50ms_R_ZA=0;
//			if(Distance22<1400&&Flag_Round_ZA==0)
//				{
//					Count_Round_ZA++;
//					if(Count_Round_ZA>9)
//					{
//						Count_Round_ZA=0;
//						Flag_Round_ZA=1;
//					}
//				}
//			else if(Distance22>=1400&&(Flag_Round_ZA==0||Flag_Round_ZA==1))
//			{
//				Count_Round_ZA=0;
//				Flag_Round_ZA=0;
//			}
//			else if(Distance22<900&&Flag_Round_ZA==1)
//			{
//				Flag_Round_ZA=2;
//			}
//		}
		if(Flag_R_ZA==2&&ABS(Difference)<10&&Flag_7H==1&&In_Roundabout_Flag==0&&(Attitude_Angle.Y>25&&Attitude_Angle.Y<33))
		{
			Flag_R_ZA=0;
			Flag_Round_ZA=2;
		}
		if(Flag_Round_ZA==0)
		{
			if(gpio_get(B22)==0)//0白1蓝
			{
				if(Flag_Blue==1)
				{
					Flag_Blue=0;
					Count_Blue=0;
				}
				else if(Flag_Blue==21)
				{
					Count_Blue++;
					{
						if(Count_Blue>30)
						{
							Flag_Blue=0;
							Count_Blue=0;
						}
					}
				}
			}
			else//蓝
			{
				if(Flag_Blue==0)
				{
					Flag_Blue=1;
				}
				if(Count_Blue<100&&Flag_Blue==1)//100ms大概10cm不间断
				{
					Count_Blue++;
				}
				if(Count_Blue>=100&&Flag_Blue==1)
				{
					Flag_Blue=22;
					Count_Blue=0;
					Flag_Running_State=3;
				}
			}
		}
	}
	else if(Flag_Running_State==2)//直立变三轮
	{
			Count_Blue++;
			if(Count_Blue<50)
			{
				Vertical_Change_Three_Wheels();
			}
			else if(Count_Blue<300)
			{
				Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
				MOTOR_Control(0, 0);
			}
			else
			{
				//Get_Attitude();	// 姿态解算
				//Attitude_Angle.Y=3;
				PID_Parameter_Init(&Roundspeed_PID);	
		PID_Parameter_Init(&Roundangle_PID);
		PID_Parameter_Init(&Rounddirect_PID);
				Speed_Measure();
				Speed_Now=0;
				Count_Blue=0;
					flag_10ms=0;
				flag_100ms=0;
				Flag_Running_State=1;
				Roundabout_cnt=0,Roundabin_cnt=0,In_Roundabout_Flag=0,In_Roundabout_Flag_O=0,Out_Roundabout_Flag=0;
				
				Flag_R_ZA=0;
				Flag_Round_ZA=0;
				Flag_7H=0;
			}
	}
	else if(Flag_Running_State==3)//三轮变直立
	{
		Count_Blue++;
		if(Count_Blue<300)
		{
				Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
				MOTOR_Control(-200,-200);
		}
		else if(Count_Blue<300)
		{
			Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
				MOTOR_Control(0,0);
		}
		else if(Count_Blue<600)
		{
			Balance_Sensor_Calculate();
			Get_Attitude();	// 姿态解算
			MOTOR_Duty_Left=(33-Attitude_Angle.Y)*2500+6*GYRO_Real.Y;
			MOTOR_Duty_Right=MOTOR_Duty_Left;
			MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);
			//Three_Wheeled_Upright();
		}
		else if(Count_Blue>=600)
		{
			Balance_Sensor_Calculate();
				Get_Attitude();	// 姿态解算
			Count_Blue=0;
			Speed_Measure();
				Speed_Now=0;
			PID_Parameter_Init(&MOTOR_PID);	// 速度环PID参数初始化
    PID_Parameter_Init(&Angle_PID);	// 角度环PID参数初始化
    PID_Parameter_Init(&Ang_Vel_PID);	// 角速度环PID参数初始化
    PID_Parameter_Init(&Direct_PID);	// 转向环PID参数初始化
    PID_Parameter_Init(&Distance_PID);	// 位置环PID参数初始化
			Flag_Running_State=0;
			Flag_R_ZA=0;
				Flag_Round_ZA=0;
				Flag_7H=0;
			Roundabout_cnt=0,Roundabin_cnt=0,In_Roundabout_Flag=0,In_Roundabout_Flag_O=0,Out_Roundabout_Flag=0;
		}
	}
	else if(Flag_Running_State==5)//直立
	{
				if(Flag_Zhi_ZA==0||Flag_Zhi_ZA==1)
		{
			Balance_Control();	// 车身平衡控制 
		}
		else if(Flag_Zhi_ZA==2)//强制右拐
		{
			//flag_2ms_R_ZA++;
			Balance_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<-50)
			{
				
				//flag_2ms_R_ZA=0;
				Flag_Zhi_ZA=3;
			}
		}
		else if(Flag_Zhi_ZA==3)//强制左拐
		{
			//flag_2ms_R_ZA++;
			Balance_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z>55)
			{
				//flag_2ms_R_ZA=0;
				Flag_Zhi_ZA=4;
			}
		}
		else if(Flag_Zhi_ZA==4)//强制右拐
		{
			//flag_2ms_R_ZA++;
			Balance_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<0)
			{
				//flag_2ms_R_ZA=0;
				Gyro_R_Z=0;
				Flag_Zhi_ZA=0;
			}
		}
		//flag_50ms_R_ZA++;
		if(Flag_R_ZA==2&&ABS(Difference)<10&&Flag_7H==1&&In_Roundabout_Flag==0)
		{
			Flag_R_ZA=0;
			Flag_Zhi_ZA=2;
			In_Roundabout_Flag=0;
		}
	}
	else if(Flag_Running_State==6)//三轮
	{
		if(Flag_Round_ZA==0||Flag_Round_ZA==1)
		{
			Round_Control();	// 车身平衡控制 
		}
		else if(Flag_Round_ZA==2)//强制右拐
		{
			//flag_2ms_R_ZA++;
			Round_Control_2();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<-45)
			{
				
				flag_2ms_R_ZA=0;
				Flag_Round_ZA=3;
			}
		}
		else if(Flag_Round_ZA==3)//强制左拐
		{
			//flag_2ms_R_ZA++;
			Round_Control_3();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z>45)
			{
				flag_2ms_R_ZA=0;
				Flag_Round_ZA=4;
			}
		}
		else if(Flag_Round_ZA==4)//强制右拐
		{
			//flag_2ms_R_ZA++;
			Round_Control_4();
			Gyro_R_Z+=0.002*GYRO_Real.X;
			if(Gyro_R_Z<0)
			{
				flag_2ms_R_ZA=0;
				Flag_Round_ZA=0;
				Gyro_R_Z=0;
			}
		}
		flag_50ms_R_ZA++;
		if(Flag_R_ZA==2&&ABS(Difference)<10&&Flag_7H==1&&In_Roundabout_Flag==0)
		{
			Flag_R_ZA=0;
			Flag_Round_ZA=2;
			In_Roundabout_Flag=0;
		}
	}
*/
