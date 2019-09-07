#include "headfile.h"

#include "math.h"

struct Boundary LEFT;
struct Boundary RIGHT;
uint8 image[ROW_C][COL_C];//����ͼ������
int16  EdgeThres = 17;  //��������ֵ
float  BlackThres = 60;   //�ڰ���ֵ
#define ERROR 2
unsigned char Round_start=35;
unsigned char Round_WhiteH=0;
unsigned char Flag_7H=0;
float Ka_L=0;
float Kb_L=0;
float Ka_R=0;
float Kb_R=0;
unsigned char R_Z=0;
unsigned char R_Y=0;
unsigned char Count_DD=0;
void Image_Get(void)
{
	int i = 0;
	int j = 0;
    for(i = 0; i < 60; i++)//60
    {
        for(j = 0; j < 94; j++)//94
        {
            image[i][j] = Image_I[i*2][j*2];
        }
    }
}
unsigned char my_adapt_threshold(uint8 *IMG, uint16 col, uint16 row)   //ע�������ֵ��һ��Ҫ��ԭͼ��
{
   #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
		uint32 gray_sum=0;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height/4;
    uint8 threshold = 0;
    uint8* data = IMG;  //ָ���������ݵ�ָ��
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;      
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;	
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    
    
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���  
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }
                      
    //����ÿ������ֵ�ĵ�������ͼ���еı���  
  
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        
    }

    //�����Ҷȼ�[0,255]  
        for (j = 0; j < GrayScale; j++)         
        {
            
                w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
                u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ 
           
               w1=1-w0;
               u1tmp=gray_sum/pixelSum-u0tmp;
        
                u0 = u0tmp / w0;              //����ƽ���Ҷ�
                u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
                u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
                deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
                if (deltaTmp > deltaMax)
                {
                    deltaMax = deltaTmp;
                    threshold = j;
                }
                if (deltaTmp < deltaMax)
                {
                break;
                }
          
         }

   return threshold;
}
void yuzhiget(void)
{
	uint16_t i = 0,j = 0,N0 = 0,N1 = 0,flag = 1;
	float T0,T1,T2,T_center;
	uint32_t S0 = 0,S1 = 0;
	T2 = BlackThres;
	
	//for(Count_DD=0;Count_DD<4;Count_DD++)
	while(flag==1&&Count_DD<=8)
	{
		for(i=0;i<ROW_C;i++)
                {
		  for(j=0;j<COL_C;j++)
                   {
			if(image[i][j] < T2)
                         {
                            S0 += image[i][j];
                            N0++;       //���ص���
			 }
			else
                         {
                            S1 += image[i][j];
                            N1++;       //���ص���
			 }
	           }
		}
                
		T0 = S0/N0;
		T1 = S1/N1;
		T_center = (T0+T1)/2;
                
		if(T2 < T_center)
                {
			if((T_center - T2)> ERROR)
                         {
			   flag = 1;
			 }
			else
                         {
			   flag = 0;
			 }
		} 
		else
                {
			if((T2 - T_center) > ERROR)
                         {
			   flag = 1;
			 }
			else
                         {
			   flag = 0;
			 }
		}
		T2 =T_center;
		BlackThres = T2;
		Count_DD++;
	}
	Count_DD=0;
}
void Jian_Yi_7H_ZL(void)
{
	short int i,j;
	for(i=40;i>17;i--)//35-14=21
		{
		for(j=47;j>=1;j--)
			{
			if(j>=2)
				{
				
				if(((image[i][j+1] - image[i][j-1]) > EdgeThres) 
	   		        && ((image[i][j] - image[i][j-2]) > EdgeThres) 
	             	&& (image[i][j-1] < BlackThres+20))
	             	
	             //if(image[i][j+1]>BlackThres&&image[i][j]>BlackThres&&image[i][j-1]<BlackThres)
	            	{
	            	//LEFT.Lose_Sum+=1;
	            	R_Z=i;
					LEFT.Row_Sum+=1;
					LEFT.Side_Line[i]=j;
					LEFT.Lossofthread_Flag[i]=0;
					break;
	            	}
				}
			else//����
				{
				LEFT.Lose_Sum+=1;
				LEFT.Row_Sum+=1;
				LEFT.Side_Line[i]=1;
				LEFT.Lossofthread_Flag[i]=1;
				//break;
				}
			}
		for(j=47;j<=92;j++)
			{
			if(j<=91)
				{
				
				if(((image[i][j-1] - image[i][j+1]) > EdgeThres) 
	   		        && ((image[i][j-2] - image[i][j]) > EdgeThres) 
	             	&& (image[i][j+1] < BlackThres+20))
	             	
	             //if(image[i][j+1]<BlackThres&&image[i][j]>BlackThres&&image[i][j-1]>BlackThres)
	            	{
	            	//RIGHT.Lose_Sum+=1;
	            	R_Y=i;
					RIGHT.Row_Sum+=1;
					RIGHT.Side_Line[i]=j;
					RIGHT.Lossofthread_Flag[i]=0;
					break;
	            	}
				}
			else//����
				{
				RIGHT.Lose_Sum+=1;
				RIGHT.Row_Sum+=1;
				RIGHT.Side_Line[i]=92;
				RIGHT.Lossofthread_Flag[i]=1;
				//break;
				}
			}
			if((image[i][47]<BlackThres))
				{
				Round_WhiteH=i;
				break;
				}
			else
				{
				Round_WhiteH=i;
				}
		}
		if(ABS(40-RIGHT.Row_Sum+RIGHT.Lose_Sum+1-Round_WhiteH)<=1)
			{
			Ka_R=(float)(Round_WhiteH+1-40)/(RIGHT.Side_Line[Round_WhiteH+1]-RIGHT.Side_Line[40]);
			if(RIGHT.Side_Line[Round_WhiteH+1]>47&&Ka_R>0)
				{
					if(Round_WhiteH<39&&Round_WhiteH>=33)
					{
						Flag_7H=1;
					}
					else if(Round_WhiteH>=39)
						{
							Flag_7H=0;
						}
				}
			}
		else if(ABS(40-LEFT.Row_Sum+LEFT.Lose_Sum+1-Round_WhiteH)<=1)
			{
			Ka_L=(float)(Round_WhiteH+1-40)/(LEFT.Side_Line[Round_WhiteH+1]-LEFT.Side_Line[40]);
			if(LEFT.Side_Line[Round_WhiteH+1]<47&&Ka_L<0)
				if(Round_WhiteH<39&&Round_WhiteH>=33)
					{
						Flag_7H=1;
					}		
				else if(Round_WhiteH>=39)
					{
						Flag_7H=0;
					}
			}
		else 
			{
			Flag_7H=0;
			}
}
void Jian_Yi_7H_SL(void)
{
	short int i,j;
	for(i=40;i>14;i--)//35-14=21
		{
		for(j=47;j>=1;j--)
			{
			if(j>=2)
				{
				
				if(((image[i][j+1] - image[i][j-1]) > EdgeThres) 
	   		        && ((image[i][j] - image[i][j-2]) > EdgeThres) 
	             	&& (image[i][j-1] < BlackThres+20))
	             	
	             //if(image[i][j+1]>BlackThres&&image[i][j]>BlackThres&&image[i][j-1]<BlackThres)
	            	{
	            	//LEFT.Lose_Sum+=1;
	            	R_Z=i;
					LEFT.Row_Sum+=1;
					LEFT.Side_Line[i]=j;
					LEFT.Lossofthread_Flag[i]=0;
					break;
	            	}
				}
			else//����
				{
				LEFT.Lose_Sum+=1;
				LEFT.Row_Sum+=1;
				LEFT.Side_Line[i]=1;
				LEFT.Lossofthread_Flag[i]=1;
				//break;
				}
			}
		for(j=47;j<=92;j++)
			{
			if(j<=91)
				{
				
				if(((image[i][j-1] - image[i][j+1]) > EdgeThres) 
	   		        && ((image[i][j-2] - image[i][j]) > EdgeThres) 
	             	&& (image[i][j+1] < BlackThres+20))
	             	
	             //if(image[i][j+1]<BlackThres&&image[i][j]>BlackThres&&image[i][j-1]>BlackThres)
	            	{
	            	//RIGHT.Lose_Sum+=1;
	            	R_Y=i;
					RIGHT.Row_Sum+=1;
					RIGHT.Side_Line[i]=j;
					RIGHT.Lossofthread_Flag[i]=0;
					break;
	            	}
				}
			else//����
				{
				RIGHT.Lose_Sum+=1;
				RIGHT.Row_Sum+=1;
				RIGHT.Side_Line[i]=92;
				RIGHT.Lossofthread_Flag[i]=1;
				//break;
				}
			}
			if((image[i][47]<BlackThres))
				{
				Round_WhiteH=i;
				break;
				}
			else
				{
				Round_WhiteH=i;
				}
		}
		if(ABS(40-RIGHT.Row_Sum+RIGHT.Lose_Sum+1-Round_WhiteH)<=1)
			{
			Ka_R=(float)(Round_WhiteH+1-40)/(RIGHT.Side_Line[Round_WhiteH+1]-RIGHT.Side_Line[40]);
			if(RIGHT.Side_Line[Round_WhiteH+1]>47&&Ka_R>0)
				{
					if(Round_WhiteH<24&&Round_WhiteH>=18)
					{
						Flag_7H=1;
					}
					else if(Round_WhiteH>=24)
						{
							Flag_7H=0;
						}
				}
			}
		else if(ABS(40-LEFT.Row_Sum+LEFT.Lose_Sum+1-Round_WhiteH)<=1)
			{
			Ka_L=(float)(Round_WhiteH+1-40)/(LEFT.Side_Line[Round_WhiteH+1]-LEFT.Side_Line[40]);
			if(LEFT.Side_Line[Round_WhiteH+1]<47&&Ka_L<0)
				if(Round_WhiteH<24&&Round_WhiteH>=18)
					{
						Flag_7H=1;
					}		
				else if(Round_WhiteH>=24)
					{
						Flag_7H=0;
					}
			}
		else 
			{
			Flag_7H=0;
			}
}
void Image_HandleZL(void)
{
	//short int i,j;
	LEFT.Jump_Count=0;//���������
	LEFT.Row_Sum=0;//���˶����� ��59��ʼ��
	LEFT.Lose_Sum=0;//��������
	LEFT.Next_Start=47;//��ʼ��
	LEFT.White_Flag=0;
	RIGHT.Jump_Count=0;
	RIGHT.Row_Sum=0;
	RIGHT.Lose_Sum=0;
	RIGHT.Next_Start=47;
	RIGHT.White_Flag=0;
	Ka_L=0;
	Ka_R=0;
	R_Z=0;
	R_Y=0;
	Count_DD=0;
	Image_Get(); 
	yuzhiget();
	//BlackThres=my_adapt_threshold(image[0],COL_C,ROW_C);
	Jian_Yi_7H_ZL();
	//Seven_Lines_Process(data,1);
	
}
void Image_HandleSL(void)
{
	//short int i,j;
	LEFT.Jump_Count=0;//���������
	LEFT.Row_Sum=0;//���˶����� ��59��ʼ��
	LEFT.Lose_Sum=0;//��������
	LEFT.Next_Start=47;//��ʼ��
	LEFT.White_Flag=0;
	RIGHT.Jump_Count=0;
	RIGHT.Row_Sum=0;
	RIGHT.Lose_Sum=0;
	RIGHT.Next_Start=47;
	RIGHT.White_Flag=0;
	Ka_L=0;
	Ka_R=0;
	R_Z=0;
	R_Y=0;
	Count_DD=0;
	Image_Get(); 
	yuzhiget();
	//BlackThres=my_adapt_threshold(image[0],COL_C,ROW_C);
	Jian_Yi_7H_SL();
	//Seven_Lines_Process(data,1);
	
}
