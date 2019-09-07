#include "headfile.h"
/*********************************************
 *	        �ӱ���ҵ��ѧ
 * @file       isr.c
 * @brief      PORTA�ж�
 * @version    v1.0
 * @author     ��е����ѧԺ ��»�
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
//ֱ���ϰ�
unsigned char  Flag_Zhi_ZA=0;
float Gyro_Z_Z=0;//����z����� ������
//�����ϰ�
unsigned short int Distance2=0;
unsigned short int Distance22=0;
unsigned short int Count_Round_ZA=0;
unsigned char  Flag_Round_ZA=0;
unsigned char flag_50ms_R_ZA=0;
unsigned short int flag_2ms_R_ZA=0;
unsigned short int Count_RZA_2=250;
unsigned short int Count_RZA_3=770;
unsigned short int Count_RZA_4=250;
float Gyro_R_Z=0;//����z����� ������
unsigned char Res2;
unsigned char step2=0;
unsigned char buf2[8];
unsigned short int buf22[5];
unsigned int Buf2_Num=0;
short int Speed22[5];
int Speed22_Num=0;//50ms�ı���������
unsigned char Count_buf22=0;
unsigned char j22=0,k22=0;
unsigned short int temp22=0;
int Dis2[2];
int Dis2_Error=0;//50ms�ļ�������
unsigned char Flag_R_ZA=0;//���ı�־λ
unsigned short int R_ZA_Count=0;//���ļ���
//ֱ���ϰ�
float PD_Z=0;
float PD_Z_wq=-90;//�ҹ�90�ȼ���
///////////////
void PORTA_IRQHandler(void)
{
	if(gpio_get(A19)==0)//��һ�е�����
	{
		while(gpio_get(A19)==0);
		gpio_turn(C0);
		//Theory_Duty=0;
		PORTA_FLAG_CLR(A19);
	}
	else if(gpio_get(A27)==0)//�ڶ��е�һ��
	{
		while(gpio_get(A27)==0);
		gpio_turn(C3);
		PORTA_FLAG_CLR(A27);
	}	
	else if(gpio_get(A26)==0)//�ڶ��еڶ���
	{
		while(gpio_get(A26)==0);
		gpio_turn(C2);	
		PORTA_FLAG_CLR(A26);
	}
	else if(gpio_get(A25)==0)//�ڶ��е�����
	{
		while(gpio_get(A25)==0);
		gpio_turn(C1);	
		PORTA_FLAG_CLR(A25);
	}
	else if(gpio_get(A24)==0)//�ڶ��е�����
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
 *	        �ӱ���ҵ��ѧ
 * @file       isr.c
 * @brief      PORTB�ж�
 * @version    v1.0
 * @author     ��е����ѧԺ ��»�
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
unsigned char Flag_LZCnt_10ms=0;//10ms��������
unsigned char Flag_LZ=0;//�̶����ϱ�־λ
int LZ_Int=0;//�����ۼ�ֵ
short int Speed_LZ[5];//������5�ε��ٶ�
unsigned char Sp_Lz=0;//5�μ�����
int Speed_LzSum=0;//5���ٶ�����
unsigned short int Speed_LsInt=0;//�ٶ�С��0�ĸ���
unsigned char Flag_DC_Control=0;
void PIT0_IRQHandler(void)//2ms
{
	PIT_FlAG_CLR(pit0);
	flag_10ms++;
	flag_100ms++;
//	flag_200ms++;
	flag_500ms++;
	Ang_Velocity_Flag = 1;	// ���ٶ�2ms��ȡһ��
	if(flag_10ms>4)
	{
		flag_10ms=0;
		Angle_Flag = 1;		// ��̬�Ƕ�10ms����һ��	
	}
	if(flag_100ms>49)
	{
		flag_100ms=0;
		Speed_Flag = 1;		// �ٶ�100ms����һ��
	}
	if(flag_500ms>249)
	{
		flag_500ms=0;
		gpio_turn(C0);
	}
	if(System_mode==0)//�����õ�ֱ������������ ֱ���ϰ�
	{
		Control_Model_0();
	}
	else if(System_mode==1)//��ϰ�õ����ַ��������� ֱ���ϰ�
	{
		Control_Model_1();
	}
	else if(System_mode==2)//����ֱ���ٶȺͽ��� �����β�����
	{
		
	}
	else if(System_mode==3)//���������ٶȺͽ��� �����β�����
	{
		
	}
	else if(System_mode==4)//�����õ�ֱ������������ �����ϰ�
	{
		Control_Model_4();
	}
	else if(System_mode==5)//��ϰ�õ����ַ��������� �����ϰ�
	{
		Control_Model_5();
	}
	else if(System_mode==6)//����ֱ���ٶȺͽ��� �����ε��Ǳ���
	{
		Control_Model_6();
	}
	else if(System_mode==7)//���������ٶȺͽ��� �����ε��Ǳ���
	{
		Control_Model_7();
	}
	else if(System_mode==8)//��Ļ��ʾͼ��Ͳ��� �����ת200
	{
		
	}
	else if(System_mode==12)//��Ļ��ʾͼ��Ͳ��� �����ת200
	{
		Balance_Sensor_Calculate();
    Data_Filter();
		Get_Attitude(); 
	}
		if(Flag_PD_QP==4)//��ʼ����
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
	else if(Flag_PD_QP==1&&Flag_LZ==0)//��ʼ���¼��٣�����û�о���·��
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
							Flag_LZ=1;//�����ٴ��������·��
							Flag_DC_Control=1;//��ֱ��
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
//  @brief      UART3�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��UART3�����жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���
//-------------------------------------------------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //�������ݼĴ�����
    {
        //�û���Ҫ����������

    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART1�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��UART1�����жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���
//-------------------------------------------------------------------------------------------------------------------
void UART1_RX_TX_IRQHandler(void)
{
    if(UART1->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
    {

    }
    if(UART1->S1 & UART_S1_TDRE_MASK )                                    //�������ݼĴ�����
    {
        //�û���Ҫ����������

    }
}

/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ������DMA0�жϣ�Ȼ��͵�����ȥ���ĸ���DMA0���жϺ������ƣ��ҵ���дһ�������Ƶĺ�������
void DMA0_IRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ


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
                


