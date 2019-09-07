#include "headfile.h"
uint16 AD_Value_temp[7][5]={0,0,0,0,0,
														0,0,0,0,0,
														0,0,0,0,0,
                            0,0,0,0,0,
														0,0,0,0,0,
														0,0,0,0,0,
                            0,0,0,0,0}; 
unsigned int AD_Value[7]={0,0,0,0,0,0,0}; 
//float DianCi_Offset=-0.0575;
float DianCi_Offset=0;
char Dif_Nor_coefficient;
float LAD,RAD,MAD,LADC,RADC;
float LAD_min=260,LAD_max=1300,
			RAD_min=260,RAD_max=1300,
			MAD_min=20,MAD_max=600,
			LADC_min=0,LADC_max=3600,
			RADC_min=0,RADC_max=3600;
int   I_LAD,I_LADC,I_MAD,I_RADC,I_RAD,
      O_I;
char Roundabout_cnt=0,Roundabin_cnt=0,In_Roundabout_Flag=0,In_Roundabout_Flag_O=0,Out_Roundabout_Flag=0;
unsigned char Judge_Time=5;//ÿһ���׶��ж�ʱ����10ms
float Circle_Int=0;
float Circle_Int_Max=0;
unsigned char Ring_Direction=0;//һ�����
/*���˵��ź�Դ*/
#define SL_HUAN 230//265
#define ZL_HUAN 210
float Inductance_Compensation=1.9;//��в���ϵ��
float Ring_Inductance_Compensation1=1.35;//��������б��в���ϵ��
float Ring_Inductance_Compensation2=1.95;//��������б��в���ϵ��
unsigned short int Flag7_time1=100;//����������
float Inductance_Compensation2=0.9;//��в���ϵ��
float Ring_Inductance_Compensation21=8;//��������б��в���ϵ��
float Ring_Inductance_Compensation22=4.5;//��������б��в���ϵ��3.5
unsigned short int Flag7_time2=350;//����������
/*�Լ����ź�Դ*/
//#define SL_HUAN 280
//#define ZL_HUAN 230
//float Inductance_Compensation=1.9;//��в���ϵ��
//float Ring_Inductance_Compensation1=1.35;//��������б��в���ϵ��
//float Ring_Inductance_Compensation2=1.95;//��������б��в���ϵ��
//unsigned short int Flag7_time1=100;//����������
//float Inductance_Compensation2=0.75;//��в���ϵ��
//float Ring_Inductance_Compensation21=8;//��������б��в���ϵ��
//float Ring_Inductance_Compensation22=3.5;//��������б��в���ϵ��
//unsigned short int Flag7_time2=100;//����������
/***************/
float Horizontal_Inductance=0;//ˮƽ��в�Ⱥ�
float Oblique_Inductance=0;//б��в�Ⱥ�
float GyrpZ_Int_ZL=0;//ֱ��״̬z����ٶȻ���


float In_offset=-0.8;
float Ring_In_offset=1.8;
unsigned int Round_int=0;
void Inductance_init(void)
{
	adc_init(ADC0_SE8);//B0
	adc_init(ADC0_SE13);//B3
	adc_init(ADC0_SE12);//B2
	adc_init(ADC0_SE9);//B1
	adc_init(ADC1_SE12);//B6
	adc_init(ADC1_SE11);//B5
	adc_init(ADC1_SE10);//B4
	adc_init(ADC1_SE13);//B7
	Dif_Nor_coefficient=50; 
	I_LAD =1900;                          
  I_LADC=900;                                 
  I_MAD =2700;                                   
  I_RADC=900;                                   
  I_RAD =1900; 
  O_I   =60;
}
//46532
void Inductance_get(void)
{
	short int i,j,k,temp;
	for(i=0;i<5;i++)
	{
		AD_Value_temp[1][i]=adc_once(ADC1_SE12,ADC_12bit);//B6 4
		AD_Value_temp[2][i]=adc_once(ADC0_SE12,ADC_12bit);//B2 6
		AD_Value_temp[3][i]=adc_once(ADC0_SE9,ADC_12bit);//B1 5
		AD_Value_temp[4][i]=adc_once(ADC1_SE11,ADC_12bit);//B5 3
		AD_Value_temp[5][i]=adc_once(ADC1_SE10,ADC_12bit);//B4 2
//		AD_Value_temp[0][i]=adc_once(ADC0_SE8,ADC_12bit);//B0 8	
//		AD_Value_temp[1][i]=adc_once(ADC0_SE13,ADC_12bit);//B3 7
//		AD_Value_temp[2][i]=adc_once(ADC0_SE12,ADC_12bit);//B2 6
//		AD_Value_temp[3][i]=adc_once(ADC1_SE12,ADC_12bit);//B6 4
//		AD_Value_temp[4][i]=adc_once(ADC1_SE11,ADC_12bit);//B5 3
//		AD_Value_temp[5][i]=adc_once(ADC1_SE10,ADC_12bit);//B4 2
//		AD_Value_temp[6][i]=adc_once(ADC1_SE13,ADC_12bit);//B7 1
	}
/*=========================ð����������==========================*///�������ֵ����Сֵ
	 for(i=1;i<6;i++)
	 {
			for(j=0;j<4;j++)
			{
				 for(k=0;k<4-j;k++)
				 {
						if(AD_Value_temp[i][k] > AD_Value_temp[i][k+1])        //ǰ��ıȺ���Ĵ�  ����н���
						{
							 temp = AD_Value_temp[i][k+1];
							 AD_Value_temp[i][k+1] = AD_Value_temp[i][k];
							 AD_Value_temp[i][k] = temp;
						}
				 }
			}
	 }	
}
void Inductance_filter(void)
{
  uint8 i;
  float sum[7] = {0,0,0,0,0,0,0};  
  Inductance_get();
  for(i=1;i<4;i++)//�м�������
  {
 //   sum[0]+=AD_Value_temp[0][i];
    sum[1]+=AD_Value_temp[1][i];
    sum[2]+=AD_Value_temp[2][i];
    sum[3]+=AD_Value_temp[3][i];
		sum[4]+=AD_Value_temp[4][i];
		sum[5]+=AD_Value_temp[5][i];
//		sum[6]+=AD_Value_temp[6][i];
  }
	LAD=(sum[1]/3-LAD_min)/(LAD_max-LAD_min)*100;
	//LAD=sum[1]/3*1.1;
	LADC=(sum[2]/3-LADC_min)/(LADC_max-LADC_min)*100;
	//LADC=sum[2]/3;
	//MAD=(sum[3]/3-MAD_min)/(MAD_max-MAD_min)*100;
	RADC=(sum[4]/3-RADC_min)/(RADC_max-RADC_min)*100;
	//RADC=sum[4]/3;
	RAD=(sum[5]/3-RAD_min)/(RAD_max-RAD_min)*100;
	//RAD=sum[5]/3;
	Difference = Inductance_Compensation*(RAD-LAD)/(LAD+RAD)*Dif_Nor_coefficient;
  Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
/*
	���ַ������߼�����
	L+R>230ʱ In_Roundabout_Flag_O=0���In_Roundabout_Flag==0 ����10ms������������In_Roundabout_Flag��1
																	���In_Roundabout_Flag==1 ��10ms�ж� �ж�Բ������
																														����ұ�б��д������б��� ��Ring_Direction��1
																														������б��д����ұ�б��� �һ�Ring_Direction��2
																														In_Roundabout_Flag��2
																	���In_Roundabout_Flag==2 ������ ���û��ַ���
																														���Ring_Direction==1 �ȴ���ˮƽ��д����ҵ�� In_Roundabout_Flag==3
																														���Ring_Direction==2 �ȴ���ˮƽ���С���ҵ�� In_Roundabout_Flag==3
																	���In_Roundabout_Flag==3 ���ֽ����ǰ ��ˮƽ���ת�� 
																														���Ring_Direction==1 ��L-R���� ����¼���ֵ ����ǰֵ�����ֵС50 In_Roundabout_Flag=4
																														���Ring_Direction==2 ��R-L���� ����¼���ֵ ����ǰֵ�����ֵС50 In_Roundabout_Flag=4
	���In_Roundabout_Flag==4 ��ʼ��б���ת�� Z����ٶȿ�ʼ���� ���ֵ�60 ��In_Roundabout_Flag==5 ������0
																	
																	���In_Roundabout_Flag==6 ����10ms������������In_Roundabout_Flag��7
	L+R<230ʱ												
	���In_Roundabout_Flag==1  ��In_Roundabout_Flag_O������ʱ100ms,��In_Roundabout_Flag=0
	���In_Roundabout_Flag==5    In_Roundabout_Flag=6 Out_Roundabout_Flag=1
	���In_Roundabout_Flag==7 ����100ms������������In_Roundabout_Flag��0 Ring_Direction��0 Out_Roundabout_Flag��0
	*/
//if((LAD+RAD)>260)//23du
//if((LAD+RAD)>230)//���˵��ź�Դ
if((LAD+RAD)>ZL_HUAN)//ò��Ҳ�������Լ����ź�Դ
{
/**************����Ԥ�ж�****(In_Roundabout_Flag==0)&&********************** */
	In_Roundabout_Flag_O=0;
	if(In_Roundabout_Flag==0)
	{
		Roundabin_cnt++;
		if(Roundabin_cnt>=Judge_Time)//10ms
		{
			Roundabin_cnt=0;
			In_Roundabout_Flag=1;
		}
	}
/******************�����е㴦��***************************/	
	else if(In_Roundabout_Flag==1)
	{
		if(RADC>LADC)//��
		{
			Roundabin_cnt++;
			{
				if(Roundabin_cnt>4)
				{
					Roundabin_cnt=0;
					In_Roundabout_Flag=2;
					Ring_Direction=1;///////////////////////////
				}
			}
		}
		else if(LADC>RADC)//��
		{
			Roundabin_cnt--;
			{
				if(Roundabin_cnt<-4)
				{
					Roundabin_cnt=0;
					In_Roundabout_Flag=2;
					Ring_Direction=2;//////////////////////////
				}
			}
		}
	}
	else if(In_Roundabout_Flag==2)
	{
		if(Ring_Direction==1)
		{
			if(RAD>LAD)
			{
				In_Roundabout_Flag=3;
			}
//			printf("%f,%f\r\n",LAD,RAD);
		}
		else if(Ring_Direction==2)
		{
			if(RAD<LAD)
			{
				In_Roundabout_Flag=3;
			}
		}
		//
	}
	else if(In_Roundabout_Flag==3)//�����ǰ
	{
		if(Ring_Direction==1)//��
		{
			if(RAD-LAD>0)
			{
				Circle_Int+=(RAD-LAD);
				Circle_Int_Max=Circle_Int;
			}
			else
			{
				Circle_Int+=(RAD-LAD);
				if(Circle_Int_Max-Circle_Int>20)
				{
					In_Roundabout_Flag=4;
					Circle_Int=0;
					Circle_Int_Max=0;
				}
			}
		}
		else if(Ring_Direction==2)//�һ�
		{
			if(LAD-RAD>0)
			{
				Circle_Int+=(LAD-RAD);
				Circle_Int_Max=Circle_Int;
			}
			else
			{
				Circle_Int+=(LAD-RAD);
				if(Circle_Int_Max-Circle_Int>20)
				{
					In_Roundabout_Flag=4;
					Circle_Int=0;
					Circle_Int_Max=0;
				}
			}
		}
	}
	else if(In_Roundabout_Flag==6)
	{
		Roundabin_cnt++;
		if(Roundabin_cnt>=Judge_Time)//10ms
		{
			Roundabin_cnt=0;
			In_Roundabout_Flag=7;
		}
	}
}
else//����230
{
	if(In_Roundabout_Flag==7)
	{
		In_Roundabout_Flag_O++;
		if(In_Roundabout_Flag_O>=Flag7_time1)
		{
			In_Roundabout_Flag_O=0;
			In_Roundabout_Flag=0;
			Out_Roundabout_Flag=0;
			Ring_Direction=0;
		}
		
	}
	else if(In_Roundabout_Flag==1||In_Roundabout_Flag==2||In_Roundabout_Flag==3)
	{
		In_Roundabout_Flag_O++;
		if(In_Roundabout_Flag_O>=50)
		{
			In_Roundabout_Flag=0;
			In_Roundabout_Flag_O=0;
		}
	}
	else if(In_Roundabout_Flag==5)
	{
		In_Roundabout_Flag_O++;
		if(In_Roundabout_Flag_O>=50)
		{
		In_Roundabout_Flag=6;
			In_Roundabout_Flag_O=0;
		Out_Roundabout_Flag=1;
		}

	}
}
if(In_Roundabout_Flag==4)//������
	{
		if(Ring_Direction==1)//��
		{
			Difference = Ring_Inductance_Compensation1*(RADC-LADC)/(LADC+RADC)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
			GyrpZ_Int_ZL+=GYRO_Real.X*0.002;
			if(GyrpZ_Int_ZL>55||GyrpZ_Int_ZL<-55)
			{
				GyrpZ_Int_ZL=0;
				In_Roundabout_Flag=5;
			}
		}
		else if(Ring_Direction==2)//�һ�
		{
			Difference = Ring_Inductance_Compensation2*(RADC-LADC)/(LADC+RADC)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
			GyrpZ_Int_ZL+=GYRO_Real.X*0.002;
			if(GyrpZ_Int_ZL<-55||GyrpZ_Int_ZL>55)
			{
				GyrpZ_Int_ZL=0;
				In_Roundabout_Flag=5;
			}
		}
		BUZZ_open;
	}
/******************��������***************************/	
/******************�������е㴦��***************************/	
else if(In_Roundabout_Flag==7)
	{
		if(Ring_Direction==1)//��
		{
			Difference = Inductance_Compensation*(RAD*1.18-LAD*0.82)/(LAD*0.82+RAD*1.18)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
		}
		else if(Ring_Direction==2)//��
		{
			Difference = Inductance_Compensation*(RAD*0.82-LAD*1.18)/(LAD*1.18+RAD*0.82)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
		}
		BUZZ_open;
	}
else if((Flag_Blue==21||Flag_Blue==22||Flag_Blue==25||Flag_Blue==26)&&In_Roundabout_Flag==0)
{
	BUZZ_open;
}
else
{
	BUZZ_shut;
}
/**********************ת�����ʼ���*************************/    
      Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
/************************�ƶ�ϵͳ***************************/    
      if(LAD<=2&&RAD<=2)
      {
        Run_Flag=0;
      }	
      else
      {
        //Run_Flag=1;
      }	
}
void Inductance_filter2(void)
{
  uint8 i;
  float sum[7] = {0,0,0,0,0,0,0};  
  Inductance_get();
  for(i=1;i<4;i++)//�м�������
  {
 //   sum[0]+=AD_Value_temp[0][i];
    sum[1]+=AD_Value_temp[1][i];
    sum[2]+=AD_Value_temp[2][i];
    sum[3]+=AD_Value_temp[3][i];
		sum[4]+=AD_Value_temp[4][i];
		sum[5]+=AD_Value_temp[5][i];
//		sum[6]+=AD_Value_temp[6][i];
  }
	LAD=(sum[1]/3-LAD_min)/(LAD_max-LAD_min)*100;
	//LAD=sum[1]/3*1.1;
	LADC=(sum[2]/3-LADC_min)/(LADC_max-LADC_min)*100;
	//LADC=sum[2]/3;
	//MAD=(sum[3]/3-MAD_min)/(MAD_max-MAD_min)*100;
	RADC=(sum[4]/3-RADC_min)/(RADC_max-RADC_min)*100;
	//RADC=sum[4]/3;
	RAD=(sum[5]/3-RAD_min)/(RAD_max-RAD_min)*100;
	//RAD=sum[5]/3;
	Difference = Difference = Inductance_Compensation2*((RAD-LAD)/(LAD+RAD)*Dif_Nor_coefficient-In_offset);
  Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
//if((LAD+RAD)>310)//���˵��ź�Դ
if((LAD+RAD)>SL_HUAN)//�Լ����ź�Դ
{
/**************����Ԥ�ж�****(In_Roundabout_Flag==0)&&********************** */
	In_Roundabout_Flag_O=0;
	if(In_Roundabout_Flag==0)
	{
		Roundabin_cnt++;
		if(Roundabin_cnt>=Judge_Time)//10ms
		{
			Roundabin_cnt=0;
			In_Roundabout_Flag=1;
		}
	}
/******************�����е㴦��***************************/	
	else if(In_Roundabout_Flag==1)
	{
		if(RADC>LADC)//��
		{
			Roundabin_cnt++;
			{
				if(Roundabin_cnt>4)
				{
					Roundabin_cnt=0;
					In_Roundabout_Flag=2;
					Ring_Direction=1;///////////////////////////
				}
			}
		}
		else if(LADC>RADC)//��
		{
			Roundabin_cnt--;
			{
				if(Roundabin_cnt<-4)
				{
					Roundabin_cnt=0;
					In_Roundabout_Flag=2;
					Ring_Direction=2;//////////////////////////
				}
			}
		}
	}
	else if(In_Roundabout_Flag==2)
	{
		if(Ring_Direction==1)
		{
			if(RAD>LAD)
			{
				In_Roundabout_Flag=3;
			}
//			printf("%f,%f\r\n",LAD,RAD);
		}
		else if(Ring_Direction==2)
		{
			if(RAD<LAD)
			{
				In_Roundabout_Flag=3;
			}
		}
		//
	}
	else if(In_Roundabout_Flag==3)//�����ǰ
	{
		if(Ring_Direction==1)//��
		{
			if(RAD-LAD>0)
			{
				Circle_Int+=(RAD-LAD);
				Circle_Int_Max=Circle_Int;
			}
			else
			{
				Circle_Int+=(RAD-LAD);
				if(Circle_Int_Max-Circle_Int>20)
				{
					In_Roundabout_Flag=4;
					Circle_Int=0;
					Circle_Int_Max=0;
				}
			}
		}
		else if(Ring_Direction==2)//�һ�
		{
			if(LAD-RAD>0)
			{
				Circle_Int+=(LAD-RAD);
				Circle_Int_Max=Circle_Int;
			}
			else
			{
				Circle_Int+=(LAD-RAD);
				if(Circle_Int_Max-Circle_Int>20)
				{
					In_Roundabout_Flag=4;
					Circle_Int=0;
					Circle_Int_Max=0;
				}
			}
		}
	}
	else if(In_Roundabout_Flag==6)
	{
		Roundabin_cnt++;
		if(Roundabin_cnt>=Judge_Time)//10ms
		{
			Roundabin_cnt=0;
			In_Roundabout_Flag=7;
		}
	}

}
else//����310
{
	if(In_Roundabout_Flag==7)
	{
		Round_int++;
		if(Round_int>=Flag7_time2)
		{
			In_Roundabout_Flag=0;
			Out_Roundabout_Flag=0;
			Ring_Direction=0;
			Round_int=0;
		}
	}
	else if(In_Roundabout_Flag==1||In_Roundabout_Flag==2||In_Roundabout_Flag==3)
	{
		In_Roundabout_Flag_O++;
		if(In_Roundabout_Flag_O>=200)
		{
			In_Roundabout_Flag=0;
			In_Roundabout_Flag_O=0;
		}
	}
	else if(In_Roundabout_Flag==5)
	{
		In_Roundabout_Flag_O++;
		if(In_Roundabout_Flag_O>=50)
		{
		In_Roundabout_Flag=6;
			In_Roundabout_Flag_O=0;
		Out_Roundabout_Flag=1;
		}
	}
}
if(In_Roundabout_Flag==4)//������
	{
		if(Ring_Direction==1)//��
		{
			Difference = Ring_Inductance_Compensation21*(RADC-LADC)/(LADC+RADC)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
			GyrpZ_Int_ZL+=GYRO_Real.X*0.002;
			if(GyrpZ_Int_ZL>50||GyrpZ_Int_ZL<-50)
			{
				if(GyrpZ_Int_ZL<-50)
				{
					Ring_Direction=2;
				}
				GyrpZ_Int_ZL=0;
				In_Roundabout_Flag=5;
			}
		}
		else if(Ring_Direction==2)//�һ�
		{
			Difference = Ring_Inductance_Compensation22*(RADC-LADC)/(LADC+RADC)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
			GyrpZ_Int_ZL+=GYRO_Real.X*0.002;
			if(GyrpZ_Int_ZL<-50||GyrpZ_Int_ZL>50)
			{
				if(GyrpZ_Int_ZL>50)
				{
					Ring_Direction=1;
				}
				GyrpZ_Int_ZL=0;
				In_Roundabout_Flag=5;
			}
		}
		BUZZ_open;
	}
/******************��������***************************/	
/******************�������е㴦��***************************/	
else if(In_Roundabout_Flag==7)
	{
		if(Ring_Direction==1)//��
		{
			Difference = Inductance_Compensation2*(RAD*1.2-LAD*0.8)/(LAD*0.8+RAD*1.2)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
		}
		else if(Ring_Direction==2)//��
		{
			Difference = Inductance_Compensation2*(RAD*0.8-LAD*1.2)/(LAD*1.2+RAD*0.8)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
		}
		BUZZ_open;
	}
else if((Flag_Blue==21||Flag_Blue==22||Flag_Blue==25||Flag_Blue==26)&&In_Roundabout_Flag==0)
{
	BUZZ_open;
}
else
{
	BUZZ_shut;
}
  Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
/************************�ƶ�ϵͳ***************************/    
      if((LAD<=2)&&(RAD<=2))
      {
        Run_Flag=0;
      }	
      else
      {
        //Run_Flag=1;
      }	
}	
void Put_SDS(void)
{
	unsigned char i;          //��������
	unsigned char Send_Count; //������Ҫ���͵����ݸ���
	DataScope_Get_Channel_Data( Speed_Set , 1 ); //������ 1.0  д��ͨ�� 1
	DataScope_Get_Channel_Data( Speed_Now , 2 ); //������ 2.0  д��ͨ�� 2
//	DataScope_Get_Channel_Data( 3.0 , 3 ); //������ 3.0  д��ͨ�� 3
//	DataScope_Get_Channel_Data( 4.0 , 4 ); //������ 4.0  д��ͨ�� 4
//	DataScope_Get_Channel_Data( 5.0 , 5 ); //������ 5.0  д��ͨ�� 5
	Send_Count = DataScope_Data_Generate(2); //����10��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���
	for( i = 0 ; i < Send_Count; i++)  //ѭ������,ֱ���������   
	{
		uart_putchar(uart1,DataScope_OutPut_Buffer[i]);     
	}
}
//void DAC_Init(void)
//{
//	dac_init(dac0,1000);
//	dac_init(dac1,1000);//�����ѹ����(1000/(1<<12)*3.3)V
//}
//void DAC_Test(void)
//{
//	int i=0;
//	dac_out(dac0,1000);
//	dac_out(dac1,1000);
//	OLED_ShowNum(byte(8),line5,(u32)1000,6,6);//1
//}
