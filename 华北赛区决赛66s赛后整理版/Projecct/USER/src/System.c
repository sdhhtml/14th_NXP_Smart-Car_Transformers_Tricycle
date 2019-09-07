#include "headfile.h"
/*********************************************
 *	        �ӱ���ҵ��ѧ
 * @file       System.c
 * @brief      ȫ�����ų�ʼ������
 * @version    v1.0
 * @author     ��е����ѧԺ ��»�
 * @date       2019
**********************************************/
unsigned char System_mode=0;
unsigned char Flag_Running_State=0;//1ֱ�� 2���� 3ֱ���л����� 4 �����л�ֱ�� 5��ֱ��ģʽ 6������ģʽ
unsigned char Flag_PD_QP=0;
void System_Init(void)
{
	short int i,j;
		//DisableInterrupts;
		//OLED
		OLED_Init();
		OLED_Clear();
		//BEEP
		gpio_init(E28,GPO,0);	
		gpio_set(E28,0);
		OLED_ShowString(byte(0),line1,"BEEP_Init",6);
		//LED
		gpio_init(C3,GPO,1); 
		gpio_init(C2,GPO,1); 
		gpio_init(C1,GPO,0); 
		gpio_init(C0,GPO,1); 
		OLED_ShowString(byte(0),line1,"LED_Init",6);	
		//KEY
		port_init (A19, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (A27, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (A26, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (A25, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (A24, IRQ_FALLING | PF | ALT1 | PULLUP );
		OLED_ShowString(byte(0),line1,"KEY_Init",6);	
		//Dial switch
		port_init (B9, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (B8, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (A29, IRQ_FALLING | PF | ALT1 | PULLUP );
		port_init (A28, IRQ_FALLING | PF | ALT1 | PULLUP );
		System_Mode_Judgement();
		OLED_ShowString(byte(0),line1,"Dial_switch_Init",6);	
		OLED_ShowNum(byte(14),line1,System_mode,2,6);
		//
		port_init (B22, IRQ_FALLING | PF | ALT1 | PULLUP );
		//Remote control
		port_init (A6, IRQ_RISING | PF | ALT1 | PULLDOWN );
		port_init (A7, IRQ_RISING | PF | ALT1 | PULLDOWN );
		port_init (A8, IRQ_RISING | PF | ALT1 | PULLDOWN );
		port_init (A9, IRQ_RISING | PF | ALT1 | PULLDOWN );
		OLED_ShowString(byte(0),line1,"Remote_control_Init",6);	
		//motor
		MotorInit();
		OLED_ShowString(byte(0),line1,"Ftm_Init",6);
		//QUAD
		ftm_quad_init(ftm1);
		ftm_quad_init(ftm2);
		port_init_NoAlt (A10, PULLUP );
		port_init_NoAlt (A11, PULLUP );
		port_init_NoAlt (A12, PULLUP );//������
		port_init_NoAlt (A13, PULLUP ); 
		ftm_quad_clean(ftm1);
		ftm_quad_clean(ftm2);
		OLED_ShowString(byte(0),line1,"Quad_Init",6);  
		//------Camera
		Delay_ms(500);
		PORTC->ISFR = 0xffffffff;
		camera_init();
		OLED_ShowString(byte(0),line1,"Camera_Init",6); 
		//----------------ICM20602-----------
		//Delay_ms(500);
		Balance_Sensor_Init();           //��������ʼ���ɹ�����LED1��
    Sensor_Offset();                 //�������ɼ���ƫ����
    for (i = 0; i < 400; i++)
    {
            for (j = 0; j < 5; j++)
            {
                    Balance_Sensor_Calculate();// ��ȡ����������
                    Data_Filter();					// ��ԭʼ���ݻ����˲�
							
                    
            }
   	    Get_Attitude();	// ��̬����
						Delay_ms(2);
    }
		//Gyrox_int=0;
		Zero_Angle =32;
		//Zero_Angle =33.5;//һ��
		Zero_Angle2=1.5;
    Run_Flag = 1;
		System_OK=1;
		OLED_ShowString(byte(0),line1,"ICM20602_Init",6); 
		Delay_ms(50);
		//1.8TFT
		//gpio_init(D2,GPO,1);	
		//lcd_init();	
//		dsp_single_colour(BLACK); //����
//		showimage(gImage_qq);
		//IPS
		//ips_init();
		if(System_mode==8)
		{
			//ISP��Ļ��ʼ��
			ips_init();
		}
		else
		{
			//���ڼ���
			uart_init(uart2,38400);
			//set_irq_priority(UART1_RX_TX_IRQn,0);
			set_irq_priority(UART2_RX_TX_IRQn,0);
			//uart_rx_irq_en(uart1);
			uart_rx_irq_en(uart2);
		}
		OLED_ShowString(byte(0),line1,"LCD_Init",6); 
		//interrupt	
		PORTA->ISFR = 0xffffffff;
		set_irq_priority(PORTA_IRQn,0);						//�������ȼ�
		//enable_irq(PORTA_IRQn);								//��PORTA�жϿ���
		//���
		Inductance_init();
//		///////////////////////////////////////////
		PID_Parameter_Init(&MOTOR_PID);	// �ٶȻ�PID������ʼ��
    PID_Parameter_Init(&Angle_PID);	// �ǶȻ�PID������ʼ��
    PID_Parameter_Init(&Ang_Vel_PID);	// ���ٶȻ�PID������ʼ��
    PID_Parameter_Init(&Direct_PID);	// ת��PID������ʼ��
    PID_Parameter_Init(&Distance_PID);	// λ�û�PID������ʼ��
		PID_Parameter_Init(&Roundspeed_PID);	
		PID_Parameter_Init(&Roundangle_PID);
		PID_Parameter_Init(&Rounddirect_PID);
		Speed_Set = 100;
    Curvature = 0;
/////////////////////////////////////////////////
		pit_init_ms(pit0,2);//2ms��ʱ���ж�
		set_irq_priority(PIT0_IRQn,3);
		PIT_FlAG_CLR(pit0);
		PD_init();//�µ�����
//		if(System_mode!=0&&System_mode!=4)
//		{
			PT_init();
			Flag_PD_QP=1;
//		}
		enable_irq(PIT0_IRQn);
		OLED_ShowString(byte(0),line1,"Interrupt_Init",6);        
}
void PD_init(void)//�µ�������ʼ��
{
	MOTOR[0]=15;
	MOTOR[1]=0;
	Ang_Vel[0]=0.205;
	Ang_Vel[1]=0.023;
	Speed_Set=300;
	Inductance_Compensation=2.15;//�Լ����ź�Դ
	Ring_Inductance_Compensation1=1.5;//��������б��в���ϵ��
	Ring_Inductance_Compensation2=2.3;//��������б��в���ϵ��	
	Direct[0]=0.033;
	Direct[2]=0.03;
}
void PT_init(void)//��ͨ�ٶȳ�ʼ��
{
	MOTOR[0]=15;
	MOTOR[1]=0;
	Ang_Vel[0]=0.25;
	Ang_Vel[1]=0.023;
	Speed_Set=140;
	Inductance_Compensation=2.5;//��в���ϵ��
	Direct[0]=0.033;
	Direct[2]=0.03;
	Ring_Inductance_Compensation1=1.35;//��������б��в���ϵ��
	Ring_Inductance_Compensation2=1.95;//��������б��в���ϵ��
}
/*********************************************
 *	        �ӱ���ҵ��ѧ
 * @file       System.c
 * @brief      ��ˮ�Ƴ���
 * @version    v1.0
 * @author     ��е����ѧԺ ��»�
 * @date       2019
**********************************************/
void Water_lamp(void)
{
	gpio_set(C3,0);
	gpio_set(C2,1);
	gpio_set(C1,1);
	gpio_set(C0,1);
	Delay_ms(200);
	gpio_set(C3,1);
	gpio_set(C2,0);
	gpio_set(C1,1);
	gpio_set(C0,1);
	Delay_ms(200);
	gpio_set(C3,1);
	gpio_set(C2,1);
	gpio_set(C1,0);
	gpio_set(C0,1);
	Delay_ms(200);
	gpio_set(C3,1);
	gpio_set(C2,1);
	gpio_set(C1,1);
	gpio_set(C0,0);
	Delay_ms(200);	
}
/*********************************************
 *	        �ӱ���ҵ��ѧ
 * @file       System.c
 * @brief      ϵͳģʽ�жϳ���
 * @version    v1.0
 * @author     ��е����ѧԺ ��»�
 * @date       2019
**********************************************/
void System_Mode_Judgement(void)
{
		if(gpio_get(B9)==0&&gpio_get(B8)==0&&gpio_get(A29)==0&&gpio_get(A28)==0)
		{
			System_mode=0;Flag_Running_State=0;//ֱ��
		}
		else if(gpio_get(B9)==1&&gpio_get(B8)==0&&gpio_get(A29)==0&&gpio_get(A28)==0)
		{
			System_mode=1;Flag_Running_State=1;//����
		}
		else if(gpio_get(B9)==0&&gpio_get(B8)==1&&gpio_get(A29)==0&&gpio_get(A28)==0)
		{
			System_mode=2;Flag_Running_State=5;//��ֱ��
		}
		else if(gpio_get(B9)==1&&gpio_get(B8)==1&&gpio_get(A29)==0&&gpio_get(A28)==0)
		{
			System_mode=3;Flag_Running_State=6;//������
		}		
		else if(gpio_get(B9)==0&&gpio_get(B8)==0&&gpio_get(A29)==1&&gpio_get(A28)==0)
		{
			System_mode=4;Flag_Running_State=0;//ֱ��
		}
		else if(gpio_get(B9)==1&&gpio_get(B8)==0&&gpio_get(A29)==1&&gpio_get(A28)==0)
		{
			System_mode=5;Flag_Running_State=1;//����
		}
		else if(gpio_get(B9)==0&&gpio_get(B8)==1&&gpio_get(A29)==1&&gpio_get(A28)==0)
		{
			System_mode=6;//��ֱ��+����
		}
		else if(gpio_get(B9)==1&&gpio_get(B8)==1&&gpio_get(A29)==1&&gpio_get(A28)==0)
		{
			System_mode=7;
		}
		else if(gpio_get(B9)==0&&gpio_get(B8)==0&&gpio_get(A29)==0&&gpio_get(A28)==1)
		{
			System_mode=8;
		}

		else if(gpio_get(B9)==1&&gpio_get(B8)==0&&gpio_get(A29)==0&&gpio_get(A28)==1)
		{
			System_mode=9;
		}
		else if(gpio_get(B9)==0&&gpio_get(B8)==1&&gpio_get(A29)==0&&gpio_get(A28)==1)
		{
			System_mode=10;
		}
		else if(gpio_get(B9)==1&&gpio_get(B8)==1&&gpio_get(A29)==0&&gpio_get(A28)==1)
		{
			System_mode=11;
		}
		else if(gpio_get(B9)==0&&gpio_get(B8)==0&&gpio_get(A29)==1&&gpio_get(A28)==1)
		{
			System_mode=12;
		}

		else if(gpio_get(B9)==1&&gpio_get(B8)==0&&gpio_get(A29)==1&&gpio_get(A28)==1)
		{
			System_mode=13;
		}
		else if(gpio_get(B9)==0&&gpio_get(B8)==1&&gpio_get(A29)==1&&gpio_get(A28)==1)
		{
			System_mode=14;
		}
		else if(gpio_get(B9)==1&&gpio_get(B8)==1&&gpio_get(A29)==1&&gpio_get(A28)==1)
		{
			System_mode=15;
		}		
}
