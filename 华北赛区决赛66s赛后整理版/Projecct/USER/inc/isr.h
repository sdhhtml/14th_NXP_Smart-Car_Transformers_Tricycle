/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK66FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/



#ifndef _isr_h
#define _isr_h


#include "headfile.h"
#define ABS(x) ( (x)>0?(x):-(x) )
void PORTA_IRQHandler(void);
void PORTB_IRQHandler(void);
extern unsigned char Flag_Blue;
extern unsigned short int Distance2;
extern unsigned short int Distance22;

extern unsigned char  Flag_Zhi_ZA;
extern unsigned char  Flag_Round_ZA;
extern unsigned char Res2;
extern unsigned char step2;
extern unsigned char buf2[8];
extern unsigned short int buf22[5];
extern unsigned int Buf2_Num;
extern short int Speed22[5];
extern int Speed22_Num;//50ms的编码器距离
extern unsigned char Count_buf22;
extern unsigned char j22,k22;
extern unsigned short int temp22;
extern int Dis2[2];
extern int Dis2_Error;//50ms的激光距离差
extern unsigned char Flag_R_ZA;//检测的标志位
extern unsigned short int R_ZA_Count;//检测的计数
extern unsigned short int Count_Blue;
extern float Gyro_R_Z;//三轮z轴积分 避障用
extern unsigned char flag_10ms;
extern unsigned char flag_100ms;
/**/
extern unsigned char Flag_LZ_10ms;
extern unsigned char Flag_LZCnt_10ms;//10ms计数变量
extern unsigned char Flag_LZ;//固定避障标志位
extern int LZ_Int;//距离累计值
extern short int Speed_LZ[5];//存放最近5次的速度
extern unsigned char Sp_Lz;//5次计数器
extern int Speed_LzSum;//5次速度总量
extern unsigned short int Speed_LsInt;//速度小于0的个数
extern unsigned char Flag_DC_Control;//直立控制模式选择0前1后
#endif
