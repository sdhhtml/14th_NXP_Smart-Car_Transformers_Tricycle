#include "headfile.h"
/*********************************************
 *	        河北工业大学
 * @file       isr.c
 * @brief      PORTA中断
 * @version    v1.0
 * @author     机械工程学院 孙德桓
 * @date       2019
**********************************************/
unsigned char flag_5ms=0;
unsigned char flag_10ms=0;
unsigned char flag_50ms=0;
unsigned char flag_100ms=0;
unsigned char flag_200ms=0;
unsigned short int flag_500ms=0;
unsigned char Flag_Blue=0;
unsigned short int Count_Blue=0;
//直立障碍
unsigned char  Flag_Zhi_ZA=0;
float Gyro_Z_Z=0;//三轮z轴积分 避障用
//三轮障碍
unsigned short int Distance2=0;
unsigned short int Distance22=0;
unsigned short int Count_Round_ZA=0;
unsigned char  Flag_Round_ZA=0;
unsigned char flag_50ms_R_ZA=0;
unsigned short int flag_2ms_R_ZA=0;
unsigned short int Count_RZA_2=250;
unsigned short int Count_RZA_3=770;
unsigned short int Count_RZA_4=250;
float Gyro_R_Z=0;//三轮z轴积分 避障用
unsigned char Res2;
unsigned char step2=0;
unsigned char buf2[8];
unsigned short int buf22[5];
unsigned int Buf2_Num=0;
short int Speed22[5];
int Speed22_Num=0;//50ms的编码器距离
unsigned char Count_buf22=0;
unsigned char j22=0,k22=0;
unsigned short int temp22=0;
int Dis2[2];
int Dis2_Error=0;//50ms的激光距离差
unsigned char Flag_R_ZA=0;//检测的标志位
unsigned short int R_ZA_Count=0;//检测的计数
//直立障碍
float PD_Z=0;
float PD_Z_wq=-90;//右拐90度减速
///////////////
void PORTA_IRQHandler(void)
{
	if(gpio_get(A19)==0)//第一列第四行
	{
		while(gpio_get(A19)==0);
		gpio_turn(C0);
		//Theory_Duty=0;
		PORTA_FLAG_CLR(A19);
	}
	else if(gpio_get(A27)==0)//第二列第一行
	{
		while(gpio_get(A27)==0);
		gpio_turn(C3);
		PORTA_FLAG_CLR(A27);
	}	
	else if(gpio_get(A26)==0)//第二列第二行
	{
		while(gpio_get(A26)==0);
		gpio_turn(C2);	
		PORTA_FLAG_CLR(A26);
	}
	else if(gpio_get(A25)==0)//第二列第三行
	{
		while(gpio_get(A25)==0);
		gpio_turn(C1);	
		PORTA_FLAG_CLR(A25);
	}
	else if(gpio_get(A24)==0)//第二列第四行
	{
		while(gpio_get(A24)==0);
		gpio_turn(C0);
		PORTA_FLAG_CLR(A24);
	}
	else if(gpio_get(A6)==1)//D
  {
		while(gpio_get(A6));
		gpio_turn(C3);
		//Run_Flag=0;
    PORTA_FLAG_CLR(A6);
	}
	else if(gpio_get(A7)==1)//C
  {
		while(gpio_get(A7));
		gpio_turn(C2);
		//Run_Flag=1;
    PORTA_FLAG_CLR(A7);
	}
	else if(gpio_get(A8)==1)//B
  {
		while(gpio_get(A8));
		gpio_turn(C1);
	//	Speed_Set-=50;
		Speed_Set=range_protect(Speed_Set,-500,500);
    PORTA_FLAG_CLR(A8);
	}
	else if(gpio_get(A9)==1)//A
  {
		while(gpio_get(A9));
		gpio_turn(C0);
		//Speed_Set+=50;
		Speed_Set=range_protect(Speed_Set,-500,500);
    PORTA_FLAG_CLR(A9);
	}	
}
/*********************************************
 *	        河北工业大学
 * @file       isr.c
 * @brief      PORTB中断
 * @version    v1.0
 * @author     机械工程学院 孙德桓
 * @date       2019
**********************************************/
void PORTB_IRQHandler(void)
{
	PORTB->ISFR = 0xffffffff;
}

void PORTC_IRQHandler(void)
{
	PORTC_FLAG_CLR(C6);
	VSYNC();	
}


void DMA0_IRQHandler(void)
{
	DMA_IRQ_CLEAN(DMA_CH0);
	row_finished();
}

unsigned short int Cnt_PD=0;

unsigned char Flag_LZ_10ms=0;
unsigned char Flag_LZCnt_10ms=0;//10ms计数变量
unsigned char Flag_LZ=0;//固定避障标志位
int LZ_Int=0;//距离累计值
short int Speed_LZ[5];//存放最近5次的速度
unsigned char Sp_Lz=0;//5次计数器
int Speed_LzSum=0;//5次速度总量
unsigned short int Speed_LsInt=0;//速度小于0的个数
unsigned char Flag_DC_Control=0;
void PIT0_IRQHandler(void)//2ms
{
	PIT_FlAG_CLR(pit0);
	flag_10ms++;
	flag_100ms++;
//	flag_200ms++;
	flag_500ms++;
	Ang_Velocity_Flag = 1;	// 角速度2ms获取一次
	if(flag_10ms>4)
	{
		flag_10ms=0;
		Angle_Flag = 1;		// 姿态角度10ms控制一次	
	}
	if(flag_100ms>49)
	{
		flag_100ms=0;
		Speed_Flag = 1;		// 速度100ms控制一次
	}
	if(flag_500ms>249)
	{
		flag_500ms=0;
		gpio_turn(C0);
	}
	if(System_mode==0)//比赛用的直立发车＋变形 直立障碍
	{
		Control_Model_0();
	}
	else if(System_mode==1)//练习用的三轮发车＋变形 直立障碍
	{
		Control_Model_1();
	}
	else if(System_mode==2)//单调直立速度和进环 不变形不避障
	{
		
	}
	else if(System_mode==3)//单调三轮速度和进环 不变形不避障
	{
		
	}
	else if(System_mode==4)//比赛用的直立发车＋变形 三轮障碍
	{
		Control_Model_4();
	}
	else if(System_mode==5)//练习用的三轮发车＋变形 三轮障碍
	{
		Control_Model_5();
	}
	else if(System_mode==6)//单调直立速度和进环 不变形但是避障
	{
		Control_Model_6();
	}
	else if(System_mode==7)//单调三轮速度和进环 不变形但是避障
	{
		Control_Model_7();
	}
	else if(System_mode==8)//屏幕显示图像和参数 电机打转200
	{
		
	}
	else if(System_mode==12)//屏幕显示图像和参数 电机打转200
	{
		Balance_Sensor_Calculate();
    Data_Filter();
		Get_Attitude(); 
	}
		if(Flag_PD_QP==4)//开始下坡
	{
//		PD_Z+=0.002*GYRO_Real.X;
//		if(PD_Z<PD_Z_wq)
//		{
//			Flag_PD_QP=1;
//			PT_init();			
//		}
		Flag_PD_QP=1;
		PT_init();
	}
	else if(Flag_PD_QP==1&&Flag_LZ==0)//开始下坡减速，并且没有经过路障
	{
		Flag_LZCnt_10ms++;
		if(Flag_LZCnt_10ms>4)
		{
			Flag_LZCnt_10ms=0;
			Speed_LZ[Sp_Lz]=Speed_Now;
			Sp_Lz++;
			if(Sp_Lz>4)
				{
					Sp_Lz=0;
					Speed_LzSum=Speed_LZ[0]+Speed_LZ[1]+Speed_LZ[2]+Speed_LZ[3]+Speed_LZ[4];
					if(Speed_LzSum<=0)
					{
						Speed_LsInt++;
						if(Speed_LsInt>20)
						{
							Speed_LsInt=0;
							Flag_LZ=1;//放置再次遇到检测路障
							Flag_DC_Control=1;//进直立
							Speed_LzSum=0;
						}
					}
					else
					{
						Speed_LsInt=0;
					}
				}
			}
		}
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART3中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当UART3启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART1中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当UART1启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void UART1_RX_TX_IRQHandler(void)
{
    if(UART1->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {

    }
    if(UART1->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}

/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了DMA0中断，然后就到下面去找哪个是DMA0的中断函数名称，找到后写一个该名称的函数即可
void DMA0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位


DMA0_IRQHandler  
DMA1_IRQHandler  
DMA2_IRQHandler  
DMA3_IRQHandler  
DMA4_IRQHandler  
DMA5_IRQHandler  
DMA6_IRQHandler  
DMA7_IRQHandler  
DMA8_IRQHandler  
DMA9_IRQHandler  
DMA10_IRQHandler 
DMA11_IRQHandler 
DMA12_IRQHandler 
DMA13_IRQHandler 
DMA14_IRQHandler 
DMA15_IRQHandler 
DMA_Error_IRQHandler      
MCM_IRQHandler            
FTFL_IRQHandler           
Read_Collision_IRQHandler 
LVD_LVW_IRQHandler        
LLW_IRQHandler            
Watchdog_IRQHandler       
RNG_IRQHandler            
I2C0_IRQHandler           
I2C1_IRQHandler           
SPI0_IRQHandler           
SPI1_IRQHandler           
SPI2_IRQHandler           
CAN0_ORed_Message_buffer_IRQHandler    
CAN0_Bus_Off_IRQHandler                
CAN0_Error_IRQHandler                  
CAN0_Tx_Warning_IRQHandler             
CAN0_Rx_Warning_IRQHandler             
CAN0_Wake_Up_IRQHandler                
I2S0_Tx_IRQHandler                     
I2S0_Rx_IRQHandler                     
CAN1_ORed_Message_buffer_IRQHandler    
CAN1_Bus_Off_IRQHandler                
CAN1_Error_IRQHandler                  
CAN1_Tx_Warning_IRQHandler             
CAN1_Rx_Warning_IRQHandler             
CAN1_Wake_Up_IRQHandler                
Reserved59_IRQHandler                  
UART0_LON_IRQHandler                   
UART0_RX_TX_IRQHandler                 
UART0_ERR_IRQHandler                   
UART1_RX_TX_IRQHandler                 
UART1_ERR_IRQHandler  
UART2_RX_TX_IRQHandler
UART2_ERR_IRQHandler  
UART3_RX_TX_IRQHandler
UART3_ERR_IRQHandler  
UART4_RX_TX_IRQHandler
UART4_ERR_IRQHandler  
UART5_RX_TX_IRQHandler
UART5_ERR_IRQHandler  
ADC0_IRQHandler
ADC1_IRQHandler
CMP0_IRQHandler
CMP1_IRQHandler
CMP2_IRQHandler
FTM0_IRQHandler
FTM1_IRQHandler
FTM2_IRQHandler
CMT_IRQHandler 
RTC_IRQHandler 
RTC_Seconds_IRQHandler  
PIT0_IRQHandler  
PIT1_IRQHandler  
PIT2_IRQHandler  
PIT3_IRQHandler  
PDB0_IRQHandler  
USB0_IRQHandler  
USBDCD_IRQHandler
ENET_1588_Timer_IRQHandler
ENET_Transmit_IRQHandler  
ENET_Receive_IRQHandler
ENET_Error_IRQHandler  
Reserved95_IRQHandler  
SDHC_IRQHandler
DAC0_IRQHandler
DAC1_IRQHandler
TSI0_IRQHandler
MCG_IRQHandler 
LPTimer_IRQHandler 
Reserved102_IRQHandler 
PORTA_IRQHandler 
PORTB_IRQHandler 
PORTC_IRQHandler 
PORTD_IRQHandler 
PORTE_IRQHandler 
Reserved108_IRQHandler
Reserved109_IRQHandler
SWI_IRQHandler 
*/
                


