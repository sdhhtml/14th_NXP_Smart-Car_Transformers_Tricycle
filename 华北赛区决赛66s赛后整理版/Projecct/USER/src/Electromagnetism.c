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
unsigned char Judge_Time=5;//每一个阶段判断时间是10ms
float Circle_Int=0;
float Circle_Int_Max=0;
unsigned char Ring_Direction=0;//一左二右
/*别人的信号源*/
#define SL_HUAN 230//265
#define ZL_HUAN 210
float Inductance_Compensation=1.9;//电感补偿系数
float Ring_Inductance_Compensation1=1.35;//环岛中心斜电感补偿系数
float Ring_Inductance_Compensation2=1.95;//环岛中心斜电感补偿系数
unsigned short int Flag7_time1=100;//出环岛参数
float Inductance_Compensation2=0.9;//电感补偿系数
float Ring_Inductance_Compensation21=8;//环岛中心斜电感补偿系数
float Ring_Inductance_Compensation22=4.5;//环岛中心斜电感补偿系数3.5
unsigned short int Flag7_time2=350;//出环岛参数
/*自己的信号源*/
//#define SL_HUAN 280
//#define ZL_HUAN 230
//float Inductance_Compensation=1.9;//电感补偿系数
//float Ring_Inductance_Compensation1=1.35;//环岛中心斜电感补偿系数
//float Ring_Inductance_Compensation2=1.95;//环岛中心斜电感补偿系数
//unsigned short int Flag7_time1=100;//出环岛参数
//float Inductance_Compensation2=0.75;//电感补偿系数
//float Ring_Inductance_Compensation21=8;//环岛中心斜电感补偿系数
//float Ring_Inductance_Compensation22=3.5;//环岛中心斜电感补偿系数
//unsigned short int Flag7_time2=100;//出环岛参数
/***************/
float Horizontal_Inductance=0;//水平电感差比和
float Oblique_Inductance=0;//斜电感差比和
float GyrpZ_Int_ZL=0;//直立状态z轴角速度积分


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
/*=========================冒泡排序升序==========================*///舍弃最大值和最小值
	 for(i=1;i<6;i++)
	 {
			for(j=0;j<4;j++)
			{
				 for(k=0;k<4-j;k++)
				 {
						if(AD_Value_temp[i][k] > AD_Value_temp[i][k+1])        //前面的比后面的大  则进行交换
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
  for(i=1;i<4;i++)//中间三个量
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
	积分法环岛逻辑尝试
	L+R>230时 In_Roundabout_Flag_O=0如果In_Roundabout_Flag==0 连续10ms都是这样，果In_Roundabout_Flag置1
																	如果In_Roundabout_Flag==1 用10ms判断 判断圆环方向
																														如果右边斜电感大于左边斜电感 左环Ring_Direction置1
																														如果左边斜电感大于右边斜电感 右环Ring_Direction置2
																														In_Roundabout_Flag置2
																	如果In_Roundabout_Flag==2 过渡用 放置积分反向
																														如果Ring_Direction==1 等待左水平电感大于右电感 In_Roundabout_Flag==3
																														如果Ring_Direction==2 等待左水平电感小于右电感 In_Roundabout_Flag==3
																	如果In_Roundabout_Flag==3 积分交叉点前 用水平电感转向 
																														如果Ring_Direction==1 将L-R积分 并记录最大值 当当前值比最大值小50 In_Roundabout_Flag=4
																														如果Ring_Direction==2 将R-L积分 并记录最大值 当当前值比最大值小50 In_Roundabout_Flag=4
	如果In_Roundabout_Flag==4 开始用斜电感转向 Z轴角速度开始积分 积分到60 果In_Roundabout_Flag==5 积分清0
																	
																	如果In_Roundabout_Flag==6 连续10ms都是这样，果In_Roundabout_Flag置7
	L+R<230时												
	如果In_Roundabout_Flag==1  用In_Roundabout_Flag_O变量计时100ms,果In_Roundabout_Flag=0
	如果In_Roundabout_Flag==5    In_Roundabout_Flag=6 Out_Roundabout_Flag=1
	如果In_Roundabout_Flag==7 连续100ms都是这样，果In_Roundabout_Flag置0 Ring_Direction置0 Out_Roundabout_Flag置0
	*/
//if((LAD+RAD)>260)//23du
//if((LAD+RAD)>230)//别人的信号源
if((LAD+RAD)>ZL_HUAN)//貌似也可以是自己的信号源
{
/**************环岛预判断****(In_Roundabout_Flag==0)&&********************** */
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
/******************环岛切点处理***************************/	
	else if(In_Roundabout_Flag==1)
	{
		if(RADC>LADC)//左
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
		else if(LADC>RADC)//右
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
	else if(In_Roundabout_Flag==3)//交叉点前
	{
		if(Ring_Direction==1)//左环
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
		else if(Ring_Direction==2)//右环
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
else//不够230
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
if(In_Roundabout_Flag==4)//交叉点后
	{
		if(Ring_Direction==1)//左环
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
		else if(Ring_Direction==2)//右环
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
/******************环岛后处理***************************/	
/******************环岛出切点处理***************************/	
else if(In_Roundabout_Flag==7)
	{
		if(Ring_Direction==1)//左环
		{
			Difference = Inductance_Compensation*(RAD*1.18-LAD*0.82)/(LAD*0.82+RAD*1.18)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
		}
		else if(Ring_Direction==2)//右
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
/**********************转向曲率计算*************************/    
      Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
/************************制动系统***************************/    
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
  for(i=1;i<4;i++)//中间三个量
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
//if((LAD+RAD)>310)//别人的信号源
if((LAD+RAD)>SL_HUAN)//自己的信号源
{
/**************环岛预判断****(In_Roundabout_Flag==0)&&********************** */
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
/******************环岛切点处理***************************/	
	else if(In_Roundabout_Flag==1)
	{
		if(RADC>LADC)//左
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
		else if(LADC>RADC)//右
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
	else if(In_Roundabout_Flag==3)//交叉点前
	{
		if(Ring_Direction==1)//左环
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
		else if(Ring_Direction==2)//右环
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
else//不够310
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
if(In_Roundabout_Flag==4)//交叉点后
	{
		if(Ring_Direction==1)//左环
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
		else if(Ring_Direction==2)//右环
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
/******************环岛后处理***************************/	
/******************环岛出切点处理***************************/	
else if(In_Roundabout_Flag==7)
	{
		if(Ring_Direction==1)//左环
		{
			Difference = Inductance_Compensation2*(RAD*1.2-LAD*0.8)/(LAD*0.8+RAD*1.2)*Dif_Nor_coefficient;
			Difference = range_protectfloat(Difference, -Dif_Nor_coefficient, Dif_Nor_coefficient);
		}
		else if(Ring_Direction==2)//右
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
  Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// 动态PID控制转向
/************************制动系统***************************/    
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
	unsigned char i;          //计数变量
	unsigned char Send_Count; //串口需要发送的数据个数
	DataScope_Get_Channel_Data( Speed_Set , 1 ); //将数据 1.0  写入通道 1
	DataScope_Get_Channel_Data( Speed_Now , 2 ); //将数据 2.0  写入通道 2
//	DataScope_Get_Channel_Data( 3.0 , 3 ); //将数据 3.0  写入通道 3
//	DataScope_Get_Channel_Data( 4.0 , 4 ); //将数据 4.0  写入通道 4
//	DataScope_Get_Channel_Data( 5.0 , 5 ); //将数据 5.0  写入通道 5
	Send_Count = DataScope_Data_Generate(2); //生成10个通道的 格式化帧数据，返回帧数据长度
	for( i = 0 ; i < Send_Count; i++)  //循环发送,直到发送完毕   
	{
		uart_putchar(uart1,DataScope_OutPut_Buffer[i]);     
	}
}
//void DAC_Init(void)
//{
//	dac_init(dac0,1000);
//	dac_init(dac1,1000);//输出电压等于(1000/(1<<12)*3.3)V
//}
//void DAC_Test(void)
//{
//	int i=0;
//	dac_out(dac0,1000);
//	dac_out(dac1,1000);
//	OLED_ShowNum(byte(8),line5,(u32)1000,6,6);//1
//}
