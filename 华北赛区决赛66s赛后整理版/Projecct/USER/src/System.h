#ifndef _SYSTEM_h
#define _SYSTEM_h
/*********************************************************************************************************************
��Դ����
////////////////////////////////���� 

        A27
        A26
A19     A25
        A24
///////////////////////////////LED  
C3 C2 C1 C0
///////////////////////////////OLED 
GND VCC   SCL E27   SDA E26
//////////////////////////////���뿪�� 
B9
B8
A29
A28
//////////////////////////////���� uart1
5V                        5V
GND                       GND
UART1_RX_PIN    E9        E1
UART1_TX_PIN    E8        E0
/////////////////////////////////////////////////////
vcc gnd a2 a1 reset
a3 a0
CLK ZUO A0
DIO YOU A3
vcc gnd DIO CLK reset
/////////////////////////////////ң��
GND
5V
A6
A7
A8
A9
/////////////////////////////////ICM20602
3V3
GND
B11
B16
B17
B10
//////////////////////////////////������
E28
///////////////////////////////////���
#define FTM3_CH0_PIN    E5         
#define FTM3_CH1_PIN    E6         
#define FTM3_CH2_PIN    E7         
#define FTM3_CH5_PIN    E10       
/////////////////////////////////1.8TFT
GND VCC SCL SDA RES DC CS BL
GND 3V3 A15 A16 D0 D1 A14 D2
///////////////////////////////////���//46532
8 7 6 5 4 3 2 1
B0 B3 B2 B1 B6 B5 B4 B7
�� �� �� �� �� �� �� һ
////////////////////////////////////������
CLK ZUO A0
DIO YOU A3
vcc gnd DIO CLK reset
/////////////////////////////////////������
X A10 GND       X A12 GND
X A11 5V        X A13 5V
////////////////////��߱���
B23 B22
B21 B20
B19 B18
E8 E9 GND 5V
5V 5V GND GND
********************************************************************************************************************/
extern unsigned char Flag_Running_State;//1ֱ�� 2���� 3ֱ���л����� 4 �����л�ֱ�� 5��ֱ��ģʽ 6������ģʽ
extern unsigned char System_mode;
extern unsigned char Flag_PD_QP;
void System_Init(void);
void Water_lamp(void);
void System_Mode_Judgement(void);
void PD_init(void);//�µ�������ʼ��
void PT_init(void);//��ͨ�ٶȳ�ʼ��
#endif
