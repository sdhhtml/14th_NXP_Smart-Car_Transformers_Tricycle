/************************************************************
 *	        河北工业大学
 * @file       main.c
 * @brief      主控程序
 * @version    v1.0
 * @author     机械工程学院 孙德桓
 * @date       2019
 * 拨码开关使用方法：1右 2右 4右边 直立发车 比赛用   3右 直立障碍（0） 3左 三轮障碍（4）
										1左 2右 4右 三轮发车 比赛不用   3右 直立障碍（1） 3左 三轮障碍（5）
										2左  1右 4右 直立发车 不变形    3左 避障（6） 3右 不避障（2）
										2左  1左 4右 三轮发车不变形     3左 避障（7） 3右 不避障（3）
										123右 4左 小屏幕显示数据 大屏幕显示图像    （8）
										
										0比赛用的直立发车＋变形 直立障碍System_mode
										4比赛用的直立发车＋变形 三轮障碍
										1练习用的三轮发车＋变形 直立障碍
										5练习用的三轮发车＋变形 三轮障碍
										2单调直立速度和进环 不变形不避障
										3单调三轮速度和进环 不变形不避障
										6单调直立速度和进环 不变形但是避障
										7单调三轮速度和进环 不变形但是避障
										8屏幕显示图像和参数 电机打转200
										
										心碎者的最低追求：
	遇到坡道前用以下参数奔跑									
float MOTOR[4]   = {15, 0, 0, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.15, 0, 0.06, 500};		// 角度环PID

float Ang_Vel[4] = {0.205, 0.023, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.033, 0, 0.03, 70};	// 转向环PID 位置	0.017	
Speed_Set = 190;
过了破道之后再换回之前的慢速
*************************************************************/
#include "headfile.h"
int main(void)
{
	unsigned int i=0;
	uint8 DOTA[10];
	get_clk();	
	System_Init();	
	OLED_ShowString(byte(0),line1,"System_Init",6);
	for(;;)
		{ 

//			OLED_ShowNum(byte(0),line2,(u32)(Flag_R_ZA),4,6);
//			OLED_ShowNum(byte(0),line3,(u32)(Flag_7H),4,6);
//			OLED_ShowNum(byte(0),line4,(u32)(In_Roundabout_Flag),4,6);
//			OLED_ShowNum(byte(0),line5,(u32)(Distance22),4,6);
//			OLED_ShowNum(byte(0),line6,(u32)(BlackThres*10),4,6);			
//			OLED_ShowNum(byte(8),line7,(u32)(Flag_Round_ZA),4,6);
//			OLED_ShowNum(byte(0),line7,(u32)(Flag_Zhi_ZA),4,6);
//			OLED_ShowNum(byte(0),line8,(u32)(Flag_PD_QP),4,6);
//			OLED_ShowNum(byte(0),line8,(u32)(Flag_PD_QP),4,6);
			OLED_ShowNum(byte(8),line8,(u32)(Flag_R_CP),4,6);


			if(mt9v032_finish_flag)
				{
					mt9v032_finish_flag = 0;
					if(System_mode==8)
					{
						ips_displayimage032(Image_I [0],188,120);
					}
					else if(System_mode==12)
					{
						seekfree_sendimg_032();
					}
					else if(System_mode==0||System_mode==1||System_mode==6)
					{
						Image_HandleZL();
					}			
					else if(System_mode==4||System_mode==5||System_mode==7)
					{
						Image_HandleSL();
					}					
				}								
		}
}
/*
		//	printf("%f\r\n",Attitude_Angle.Y);
			//printf("%f\r\n",Gyrox_int);
			#if 0
			OLED_ShowNum(byte(0),line2,(u32)(LAD*10),6,6);
			OLED_ShowNum(byte(0),line3,(u32)(LADC*10),6,6);
			OLED_ShowNum(byte(0),line4,(u32)(MAD*10),6,6);
			OLED_ShowNum(byte(0),line5,(u32)(RADC*10),6,6);
			OLED_ShowNum(byte(0),line6,(u32)(RAD*10),6,6);
			OLED_ShowNum(byte(0),line6,(u32)(RAD*10),6,6);
			OLED_ShowNum(byte(0),line6,(u32)(RAD*10),6,6);
			OLED_ShowNum(byte(8),line2,(u32)(Attitude_Angle.Y*100),4,6);
			#endif
			#if 0
			OLED_ShowNum(byte(0),line7,(u32)(MOTOR[KP]*1000),6,6);
			OLED_ShowNum(byte(8),line2,(u32)(Attitude_Angle.Y*100),4,6);
			OLED_ShowNum(byte(8),line3,(u32)(Target_Angle.Y),6,6);
			OLED_ShowNum(byte(8),line4,(u32)(Ang_Vel[KP]*1000),6,6);
			OLED_ShowNum(byte(8),line5,(u32)(Ang_Vel[KI]*1000),6,6);
			OLED_ShowNum(byte(8),line6,(u32)(Angle[KP]*1000),6,6);
			OLED_ShowNum(byte(8),line7,(u32)(Angle[KD]*1000),6,6);
			#endif
			//data_conversion((int16)LAD,(int16)LADC,(int16)RADC,(int16)RAD,DOTA);
			//data_conversion((int16)Speed22_Num,(int16)Dis2_Error,(int16)Distance22,(int16)RAD,DOTA);
			//printf("%d,%d,%d\r\n",Speed22_Num,Dis2_Error,Distance22);
*/
