#include "headfile.h"
S_FLOAT_XYZ 
	GYRO_Real,		// 陀螺仪转化后的数据
	ACC_Real,		// 加速度计转化后的数据
	Attitude_Angle,    	// 当前角度
	Last_Angle,		// 上次角度
	Target_Angle,	        // 目标角度
        Target_Angle_Grow;      // 目标角度增长
S_INT16_XYZ
	GYRO,			// 陀螺仪原始数据
	GYRO_Offset,	        // 陀螺仪温飘
	GYRO_Last,		// 陀螺仪上次数据
	ACC, 			// 加速度计数据
	ACC_Offset,		// 加速度计温飘
	ACC_Last;		// 加速度计上次数据
S_INT32_XYZ
	Tar_Ang_Vel,	        // 目标角速度
	Tar_Ang_Vel_Last,	// 上次目标角速度
        Tar_Ang_Vel_Grow;       // 目标角速度增长
int32 
        In_Out_Roundabout_I=0,
        In_Out_Roundabout_ture_I=0,
        Out_Roundabout_I=0,
        Run_I=0,
	Speed_Now = 0,	        // 当前实际速度
        Speed_Now_Last=0,       // 上次实际速度
	Speed_Min = 0,	        // 左右最小速度
	Speed_Set = 0, 	        // 目标设定速度
	Theory_Duty = 0,        // 理论直立占空比
	Vel_Set = 0,	        // 目标转向角速度
	Direct_Parameter = 0,   // 转向系数
	Direct_Last = 0,
	Curvature = 0;		// 目标转向半径倒数

float   Target_Angle_min=3000,       //跑动前倾最大角度
        Target_Angle_max=3600;       //跑动后仰最大角度
float Zero_Angle,Zero_Angle2;
uint8 System_OK = 0;
/* 各种标志位，放定时器中进行时序控制 */
char Speed_Flag, Angle_Flag, Ang_Velocity_Flag;
short int point_center=0;
float TP=65;
float Curvature_E;
int Left_Offset=15;
int Round_Increase=0;
void Balance_Control(void)
{
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									Inductance_filter();//电感测量
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, 230);	// 结果为放大100倍的目标角度  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// 目标角度叠加在零点上
					if(Flag_PD_QP==0||Flag_PD_QP==3||Flag_PD_QP==2||Flag_PD_QP==4)
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-800,Zero_Angle*100+600);
					}
					else
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-900,Zero_Angle*100+500);
					}
       }
}
void Round_Control(void)//三轮2ms控制
{
	if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//内环输出

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									Inductance_filter2();//电感测量
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// 速度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					if(Flag_R_CP==0)
					{
						Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, 110);	// 结果为放大100倍的目标角度  Speed_Set
						Target_Angle.Y += Zero_Angle2*100;	// 目标角度叠加在零点上
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
						Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
						Round_Increase=range_protect(Round_Increase, -10,10);					
						Theory_Duty+=Round_Increase;
						Theory_Duty = range_protect(Theory_Duty, -250, 250);
					}
					else if(Flag_R_CP==1)
					{
						Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, 150);	// 结果为放大100倍的目标角度  Speed_Set
						Target_Angle.Y += Zero_Angle2*100;	// 目标角度叠加在零点上
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
						Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
						Round_Increase=range_protect(Round_Increase, -8,15);					
						Theory_Duty+=Round_Increase;
						Theory_Duty = range_protect(Theory_Duty, -300, 300);
					}
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
       }
}
void Vertical_Change_Three_Wheels(void)//直立变三轮
{
	
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            //Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter();//电感测量
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
            MOTOR_Duty_Left= Theory_Duty;
						MOTOR_Duty_Right= Theory_Duty;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(220));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
       }
}
void Three_Wheeled_Upright(void)//三轮变直立
{
	
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            //Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel_up, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter();//电感测量
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
//            Direct_Last = 0;
//						//
//             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
//             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						MOTOR_Duty_Left= Theory_Duty;
						MOTOR_Duty_Right= Theory_Duty;
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
         Get_Attitude(); 
					}
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle_up, ((int32)(Attitude_Angle.Y)*100), (int32)(3600));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
       }
}
void Round_Control_2(void)//强制右拐
{
	if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//内环输出

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter2();//电感测量
							//Difference=50;
							Difference=-50;//左拐
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// 速度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					
					Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, Speed_Set);	// 结果为放大100倍的目标角度  Speed_Set
          Target_Angle.Y += Zero_Angle2*100;	// 目标角度叠加在零点上
					Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
					Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
				  Round_Increase=range_protect(Round_Increase, -10,10);					
					Theory_Duty+=Round_Increase;
					//Theory_Duty=-PID_Realize(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));	
					Theory_Duty = range_protect(Theory_Duty, -250, 250);
					//printf("%d\r\n",Round_Increase);
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
       }
}
void Round_Control_3(void)//强制z拐
{
	if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//内环输出

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter2();//电感测量
							//Difference=-15;
							Difference=15;//右拐
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// 速度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					
					Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, Speed_Set);	// 结果为放大100倍的目标角度  Speed_Set
          Target_Angle.Y += Zero_Angle2*100;	// 目标角度叠加在零点上
					Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
					Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
				  Round_Increase=range_protect(Round_Increase, -10,10);					
					Theory_Duty+=Round_Increase;
					//Theory_Duty=-PID_Realize(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));	
					Theory_Duty = range_protect(Theory_Duty, -250, 250);
					//printf("%d\r\n",Round_Increase);
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
       }
}
void Round_Control_4(void)//强制右拐
{
	if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//内环输出

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter2();//电感测量
							//Difference=25;
							Difference=-25;//左拐
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// 速度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					
					Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, Speed_Set);	// 结果为放大100倍的目标角度  Speed_Set
          Target_Angle.Y += Zero_Angle2*100;	// 目标角度叠加在零点上
					Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
					Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
				  Round_Increase=range_protect(Round_Increase, -10,10);					
					Theory_Duty+=Round_Increase;
					//Theory_Duty=-PID_Realize(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));	
					Theory_Duty = range_protect(Theory_Duty, -250, 250);
					//printf("%d\r\n",Round_Increase);
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
       }
}
void Balance_Control_2(void)//强制右拐
{
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter();//电感测量
							Difference=50;
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);	// 结果为放大100倍的目标角度  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// 目标角度叠加在零点上
            Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-450,Zero_Angle*100+350);	
				 
				 ////
				 //Target_Angle.Y = Zero_Angle*100;
				// printf("%f\r\n",Target_Angle.Y);
				 ////////
       }
}
void Balance_Control_3(void)//强制左拐
{
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter();//电感测量
							Difference=-15;
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);	// 结果为放大100倍的目标角度  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// 目标角度叠加在零点上
            Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-450,Zero_Angle*100+350);	
				 
				 ////
				 //Target_Angle.Y = Zero_Angle*100;
				// printf("%f\r\n",Target_Angle.Y);
				 ////////
       }
}
void Balance_Control_4(void)//强制右拐
{
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									//Inductance_filter();//电感测量
							Difference=25;
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);	// 结果为放大100倍的目标角度  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// 目标角度叠加在零点上
            Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-450,Zero_Angle*100+350);	
				 
				 ////
				 //Target_Angle.Y = Zero_Angle*100;
				// printf("%f\r\n",Target_Angle.Y);
				 ////////
       }
}
void Balance_Control_DC(void)
{
        if (Ang_Velocity_Flag)	// 直立角速度环	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* 角速度环作为最内环控制直立 */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // 获取当前速度	/* 角速度环作为最内环控制转向 */									//Speed_Min					
									Inductance_filter();//电感测量
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// 转向环左正右负
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //中线偏差 半径倒数 最小速度 转向环输入 转向环期望 转向环输出
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// 更新上次角速度环结果
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// 左右电机根据转向系数调整差速
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// 控制左右电机  
              MOTOR_Control(0, 0);// 停车电机控制    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// 直立角度环 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
             /* 角度环加到角速度环上串级控制 */             // 结果为放大10倍的目标角速度	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
        }
        
       if (Speed_Flag)// 速度环     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, -100);	// 结果为放大100倍的目标角度  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// 目标角度叠加在零点上
					if(Flag_PD_QP==0||Flag_PD_QP==3||Flag_PD_QP==2||Flag_PD_QP==4)
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-800,Zero_Angle*100+600);
					}
					else
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-600,Zero_Angle*100+600);
					}
       }
}
