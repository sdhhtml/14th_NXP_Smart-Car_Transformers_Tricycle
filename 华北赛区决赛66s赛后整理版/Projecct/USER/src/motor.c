#include "headfile.h"
/****************************    速度控制变量    ****************************/
char Run_Flag = 0;
int32   MOTOR_Speed_Left = 0;         //FTM1  正值
int32   MOTOR_Speed_Right = 0;         //FTM2 负值
char Left_Crazy = 0;	// 电机疯转
char Right_Crazy = 0;	// 电机疯转
char Crazy_Flag = 0;
int32 MOTOR_Duty_Left  = 0;
int32 MOTOR_Duty_Right =0;
int32 MOTOR_Left_Acc = 0;
int32 MOTOR_Right_Acc = 0;
int32 MOTOR_Speed_Left_Last = 0;
int32 MOTOR_Speed_Right_Last = 0;
float SpeedRw, SpeedLw;

uint8    Point;	//目标点横坐标
int8     Point_Num=0;  //提前系数

float LeftMotorOut  = 0;      
float RightMotorOut = 0;  

int Fres = 0;	// 动态参数变量
/*******************电机引脚初始化**************/

void MotorInit(void)
{
    ftm_pwm_init(MOTOR_FTM,Left_MOTOR1_PWM,MOTOR_HZ,0);
    ftm_pwm_init(MOTOR_FTM,Left_MOTOR2_PWM,MOTOR_HZ,0);
    ftm_pwm_init(MOTOR_FTM,Right_MOTOR1_PWM,MOTOR_HZ,0);
    ftm_pwm_init(MOTOR_FTM,Right_MOTOR2_PWM,MOTOR_HZ,0);
}
unsigned short int Speed_PD_MIN=100;
void Speed_Measure(void)
{
  	static int32 Speed_Last = 0;
	static int32 Crazy_Count = 0;
        /******* 右电机速度相关控制 ********/
              MOTOR_Speed_Right = ftm_quad_get(ftm1)*100*Pulse_CM_coe;;//获取FTM1 正交解码 的脉冲数(负数表示反方向) 
                                                  
              ftm_quad_clean(ftm1);   	//正交解码寄存器清零
              
              
              MOTOR_Right_Acc = MOTOR_Speed_Right - MOTOR_Speed_Right_Last;	// 计算加速度
              
              if(MOTOR_Right_Acc >100)
              MOTOR_Speed_Right += 100;
              
              else if(MOTOR_Right_Acc < -100)
              MOTOR_Speed_Right -=100;
              
							
              MOTOR_Speed_Right = MOTOR_Speed_Right;
               
               MOTOR_Speed_Right = range_protect(MOTOR_Speed_Right,-500, 500); //限幅保护
               MOTOR_Speed_Right_Last=MOTOR_Speed_Right;
               
            /******* 左电机速度相关控制 ********/
             MOTOR_Speed_Left = -ftm_quad_get(ftm2)*100*Pulse_CM_coe;;//获取FTM1 正交解码 的脉冲数(负数表示反方向)
             ftm_quad_clean(ftm2);   	//正交解码寄存器清零
							
             MOTOR_Left_Acc = MOTOR_Speed_Left - MOTOR_Speed_Left_Last;	// 计算加速度
             if(MOTOR_Left_Acc >100)
              MOTOR_Speed_Left += 100;
             
              else if(MOTOR_Left_Acc < -100)
              MOTOR_Speed_Left -= 100;
              
              else 
               MOTOR_Speed_Left = MOTOR_Speed_Left;
              
               MOTOR_Speed_Left = range_protect(MOTOR_Speed_Left,-500, 500); //限幅保护
               MOTOR_Speed_Left_Last=MOTOR_Speed_Left;
   
               
               
               
            /*****************转向和直道真实速度计算*****************/
              
             SpeedLw = MOTOR_Speed_Left +BodyR * GYRO_Real.X * PAI / 180;//左转 x>0 右转 x<0
             SpeedRw = MOTOR_Speed_Right-BodyR * GYRO_Real.X * PAI / 180;
             if (GYRO_Real.X > 50)
              {
                  Speed_Now = SpeedRw;       // 左转，用右边
              } 
             else if (GYRO_Real.X < -50)
              {
                  Speed_Now = SpeedLw;       // 右转，用左边
              } 
              else 
              {
                  Speed_Now = (MOTOR_Speed_Right+MOTOR_Speed_Left)/2;        // 不怎么转，用两边
             }
//							if(Flag_PD_QP==1)
//							{
//								Speed_Now = (MOTOR_Speed_Right+MOTOR_Speed_Left)/2; 
//							}
             if((Speed_Now - Speed_Now_Last) > 100)
             Speed_Now += 100;
             else if((Speed_Now - Speed_Now_Last) < -100)
             Speed_Now -= 100;
             else 
             Speed_Now = Speed_Now;
             Speed_Now = range_protect(Speed_Now,-500,500); //限幅保护
             Speed_Now_Last=Speed_Now;
//             
               
               Speed_Now = (MOTOR_Speed_Right+MOTOR_Speed_Left)/2; 
               Speed_Now = Speed_Now *0.9 + Speed_Last * 0.1;
               Speed_Last = Speed_Now;
             
          /**********转向最小速度计算**********/             
             
            Speed_Min = Speed_Min * 0.1 + Speed_Now * 0.9;
            Speed_Min = range_protect(Speed_Min, 10, 290); //限幅保护
            
          /**********转向参数选择**********/
            
             if(Speed_Min <= Speed_Set*0.3)
                     Fres = 0;
            
             else if(Speed_Min <= Speed_Set*0.7)
                    Fres = 1;
            
             else if(Speed_Min <= Speed_Set)
                      Fres = 2;
             
             else 
                     Fres = 3;
						 if(Flag_PD_QP==0)
						 {
							 if(Speed_Min <= Speed_Set*0.3)
                     Fres = 2;
            
							 else if(Speed_Min <= Speed_Set*0.7)
											Fres = 2;
							 else if(Speed_Min <= Speed_Set)
												Fres = 2;
							 else 
											 Fres = 3;
							 if(Speed_Min>120)
							 {
								 Flag_PD_QP=2;
							 }
						 }
						 else if(Flag_PD_QP==2)
						 {
							 if(Speed_Min<50)
							 {
								 Flag_PD_QP=3;
								 //Speed_PD_MIN
								 //PT_init();
							 }
						 }
						 else if(Flag_PD_QP==3)
						 {
							 if(Speed_PD_MIN>Speed_Min)
							 {
								 Speed_PD_MIN=Speed_Min;
							 }
							 else
							 {
								 if(Speed_Min-Speed_PD_MIN>5)
								 {
//									 PT_init();
//									 Flag_PD_QP=1;
									 
									 Flag_PD_QP=4;//开始下坡
									 Inductance_Compensation=4;//自己的信号源
									 Direct[0]=0.05;
										Direct[2]=0.03;
							//	 Speed_Set=-1000;
//									 Direct[0]=0;
//									 Direct[2]=0;
								 }
							 }
						 }
//                  
}
int range_protect(int32 duty, int32 min, int32 max)//限幅保护
{
	if (duty >= max)
	{
		return max;
	}
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}

float range_protectfloat(float duty, float min, float max)//限幅保护
{
	if (duty >= max)
	{
		return max;
	}
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}
void MOTOR_Control(int32 LDuty, int32 RDuty)
{

            if(RDuty >= 0)
            {
                    RDuty = range_protect(RDuty, 0, MOTOR_MAX);	// 限幅保护
                    ftm_pwm_duty(MOTOR_FTM, Right_MOTOR1_PWM, RDuty);
             	    ftm_pwm_duty(MOTOR_FTM, Right_MOTOR2_PWM, 0);

            }
            else
            {
                    RDuty = range_protect(-RDuty, 0, MOTOR_MAX);// 限幅保护
                    ftm_pwm_duty(MOTOR_FTM, Right_MOTOR1_PWM, 0);
             	    ftm_pwm_duty(MOTOR_FTM, Right_MOTOR2_PWM, RDuty);
            }
            
            if (LDuty >= 0)
            {
                    LDuty = range_protect(LDuty, 0, MOTOR_MAX);	// 限幅保护
                    ftm_pwm_duty(MOTOR_FTM, Left_MOTOR1_PWM, LDuty);
      	            ftm_pwm_duty(MOTOR_FTM, Left_MOTOR2_PWM, 0);
            }
            else
            {
                    LDuty = range_protect(-LDuty, 0, MOTOR_MAX);// 限幅保护
                    ftm_pwm_duty(MOTOR_FTM, Left_MOTOR1_PWM, 0);
      	            ftm_pwm_duty(MOTOR_FTM, Left_MOTOR2_PWM,LDuty);
            }
     
  
}
