#include "headfile.h"
S_FLOAT_XYZ 
	GYRO_Real,		// ������ת���������
	ACC_Real,		// ���ٶȼ�ת���������
	Attitude_Angle,    	// ��ǰ�Ƕ�
	Last_Angle,		// �ϴνǶ�
	Target_Angle,	        // Ŀ��Ƕ�
        Target_Angle_Grow;      // Ŀ��Ƕ�����
S_INT16_XYZ
	GYRO,			// ������ԭʼ����
	GYRO_Offset,	        // ��������Ʈ
	GYRO_Last,		// �������ϴ�����
	ACC, 			// ���ٶȼ�����
	ACC_Offset,		// ���ٶȼ���Ʈ
	ACC_Last;		// ���ٶȼ��ϴ�����
S_INT32_XYZ
	Tar_Ang_Vel,	        // Ŀ����ٶ�
	Tar_Ang_Vel_Last,	// �ϴ�Ŀ����ٶ�
        Tar_Ang_Vel_Grow;       // Ŀ����ٶ�����
int32 
        In_Out_Roundabout_I=0,
        In_Out_Roundabout_ture_I=0,
        Out_Roundabout_I=0,
        Run_I=0,
	Speed_Now = 0,	        // ��ǰʵ���ٶ�
        Speed_Now_Last=0,       // �ϴ�ʵ���ٶ�
	Speed_Min = 0,	        // ������С�ٶ�
	Speed_Set = 0, 	        // Ŀ���趨�ٶ�
	Theory_Duty = 0,        // ����ֱ��ռ�ձ�
	Vel_Set = 0,	        // Ŀ��ת����ٶ�
	Direct_Parameter = 0,   // ת��ϵ��
	Direct_Last = 0,
	Curvature = 0;		// Ŀ��ת��뾶����

float   Target_Angle_min=3000,       //�ܶ�ǰ�����Ƕ�
        Target_Angle_max=3600;       //�ܶ��������Ƕ�
float Zero_Angle,Zero_Angle2;
uint8 System_OK = 0;
/* ���ֱ�־λ���Ŷ�ʱ���н���ʱ����� */
char Speed_Flag, Angle_Flag, Ang_Velocity_Flag;
short int point_center=0;
float TP=65;
float Curvature_E;
int Left_Offset=15;
int Round_Increase=0;
void Balance_Control(void)
{
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									Inductance_filter();//��в���
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, 230);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// Ŀ��Ƕȵ����������
					if(Flag_PD_QP==0||Flag_PD_QP==3||Flag_PD_QP==2||Flag_PD_QP==4)
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-800,Zero_Angle*100+600);
					}
					else
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-900,Zero_Angle*100+500);
					}
       }
}
void Round_Control(void)//����2ms����
{
	if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//�ڻ����

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									Inductance_filter2();//��в���
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// �ٶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					if(Flag_R_CP==0)
					{
						Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, 110);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
						Target_Angle.Y += Zero_Angle2*100;	// Ŀ��Ƕȵ����������
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
						Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
						Round_Increase=range_protect(Round_Increase, -10,10);					
						Theory_Duty+=Round_Increase;
						Theory_Duty = range_protect(Theory_Duty, -250, 250);
					}
					else if(Flag_R_CP==1)
					{
						Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, 150);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
						Target_Angle.Y += Zero_Angle2*100;	// Ŀ��Ƕȵ����������
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
						Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
						Round_Increase=range_protect(Round_Increase, -8,15);					
						Theory_Duty+=Round_Increase;
						Theory_Duty = range_protect(Theory_Duty, -300, 300);
					}
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
       }
}
void Vertical_Change_Three_Wheels(void)//ֱ��������
{
	
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            //Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter();//��в���
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
            MOTOR_Duty_Left= Theory_Duty;
						MOTOR_Duty_Right= Theory_Duty;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(220));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
       }
}
void Three_Wheeled_Upright(void)//���ֱ�ֱ��
{
	
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            //Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel_up, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter();//��в���
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
//            Direct_Last = 0;
//						//
//             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
//             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						MOTOR_Duty_Left= Theory_Duty;
						MOTOR_Duty_Right= Theory_Duty;
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
         Get_Attitude(); 
					}
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle_up, ((int32)(Attitude_Angle.Y)*100), (int32)(3600));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
       }
}
void Round_Control_2(void)//ǿ���ҹ�
{
	if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//�ڻ����

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter2();//��в���
							//Difference=50;
							Difference=-50;//���
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// �ٶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					
					Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, Speed_Set);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
          Target_Angle.Y += Zero_Angle2*100;	// Ŀ��Ƕȵ����������
					Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
					Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
				  Round_Increase=range_protect(Round_Increase, -10,10);					
					Theory_Duty+=Round_Increase;
					//Theory_Duty=-PID_Realize(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));	
					Theory_Duty = range_protect(Theory_Duty, -250, 250);
					//printf("%d\r\n",Round_Increase);
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
       }
}
void Round_Control_3(void)//ǿ��z��
{
	if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//�ڻ����

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter2();//��в���
							//Difference=-15;
							Difference=15;//�ҹ�
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// �ٶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					
					Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, Speed_Set);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
          Target_Angle.Y += Zero_Angle2*100;	// Ŀ��Ƕȵ����������
					Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
					Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
				  Round_Increase=range_protect(Round_Increase, -10,10);					
					Theory_Duty+=Round_Increase;
					//Theory_Duty=-PID_Realize(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));	
					Theory_Duty = range_protect(Theory_Duty, -250, 250);
					//printf("%d\r\n",Round_Increase);
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
       }
}
void Round_Control_4(void)//ǿ���ҹ�
{
	if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter2();
//					g_BlanceControl_out_Out=0;
//					BalanceControl_in();//�ڻ����

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter2();//��в���
							//Difference=25;
							Difference=-25;//���
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
                  Direct_Parameter = PID_Realize_D(&Rounddirect_PID, Round_Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -300, 300);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
            //Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(115, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        }
        if (Angle_Flag)// �ٶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
					
					Target_Angle.Y =  PID_Realize(&Roundspeed_PID, Round, Speed_Now, Speed_Set);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
          Target_Angle.Y += Zero_Angle2*100;	// Ŀ��Ƕȵ����������
					Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle2*100-250,Zero_Angle2*100+300);
					Round_Increase=PID_Increase(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));		
				  Round_Increase=range_protect(Round_Increase, -10,10);					
					Theory_Duty+=Round_Increase;
					//Theory_Duty=-PID_Realize(&Roundangle_PID,Round_Angle,((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));	
					Theory_Duty = range_protect(Theory_Duty, -250, 250);
					//printf("%d\r\n",Round_Increase);
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
       }
}
void Balance_Control_2(void)//ǿ���ҹ�
{
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter();//��в���
							Difference=50;
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// Ŀ��Ƕȵ����������
            Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-450,Zero_Angle*100+350);	
				 
				 ////
				 //Target_Angle.Y = Zero_Angle*100;
				// printf("%f\r\n",Target_Angle.Y);
				 ////////
       }
}
void Balance_Control_3(void)//ǿ�����
{
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter();//��в���
							Difference=-15;
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// Ŀ��Ƕȵ����������
            Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-450,Zero_Angle*100+350);	
				 
				 ////
				 //Target_Angle.Y = Zero_Angle*100;
				// printf("%f\r\n",Target_Angle.Y);
				 ////////
       }
}
void Balance_Control_4(void)//ǿ���ҹ�
{
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									//Inductance_filter();//��в���
							Difference=25;
							 Curvature = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);	// ��̬PID����ת��
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
        
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// Ŀ��Ƕȵ����������
            Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-450,Zero_Angle*100+350);	
				 
				 ////
				 //Target_Angle.Y = Zero_Angle*100;
				// printf("%f\r\n",Target_Angle.Y);
				 ////////
       }
}
void Balance_Control_DC(void)
{
        if (Ang_Velocity_Flag)	// ֱ�����ٶȻ�	2ms
        {
            Ang_Velocity_Flag = 0;
            Balance_Sensor_Calculate();
            Data_Filter();
            
            /* ���ٶȻ���Ϊ���ڻ�����ֱ�� */
            Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)( Tar_Ang_Vel.Y));
            //Theory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), 0);	// ����ֱ��PWM  Tar_Ang_Vel.Y
            Theory_Duty = range_protect(Theory_Duty, -950, 950);

            if (System_OK)
            {     
                 // ��ȡ��ǰ�ٶ�	/* ���ٶȻ���Ϊ���ڻ�����ת�� */									//Speed_Min					
									Inductance_filter();//��в���
                  Direct_Parameter = PID_Realize_D(&Direct_PID, Direct, (int32)(GYRO_Real.X*100),Curvature*Speed_Min);	// ת�������Ҹ�
                  Direct_Parameter = range_protect(Direct_Parameter, -400, 400);
							
               
            }
                //����ƫ�� �뾶���� ��С�ٶ� ת������ ת������ ת�����
             Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;// �����ϴν��ٶȻ����
						//debug
           // Direct_Last = 0;
						//
             MOTOR_Duty_Left  = Theory_Duty-Direct_Last;	// ���ҵ������ת��ϵ����������
             MOTOR_Duty_Right = Theory_Duty+Direct_Last;
						
						if(MOTOR_Duty_Left>0)
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left+Left_Offset+4;
						}
						else
						{
							MOTOR_Duty_Left=MOTOR_Duty_Left-Left_Offset+2;
						}
						MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);
						MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
            if(Run_Flag) 
						{
              MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��   
						//MOTOR_Control(119, 100);
						}
            else  
						{
							//MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);// �������ҵ��  
              MOTOR_Control(0, 0);// ͣ���������    						
						}
             Get_Attitude(); 
        
        }
  
  
        if (Angle_Flag)// ֱ���ǶȻ� 10ms
        {   
            Speed_Measure();
            Angle_Flag = 0;
             /* �ǶȻ��ӵ����ٶȻ��ϴ������� */             // ���Ϊ�Ŵ�10����Ŀ����ٶ�	
            Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, ((int32)(Attitude_Angle.Y)*100), (int32)(Target_Angle.Y));
            Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// ע��������
        }
        
       if (Speed_Flag)// �ٶȻ�     100ms
       {
            Speed_Flag=0;
            Target_Angle.Y =  -PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, -100);	// ���Ϊ�Ŵ�100����Ŀ��Ƕ�  Speed_Set
            Target_Angle.Y += Zero_Angle*100;	// Ŀ��Ƕȵ����������
					if(Flag_PD_QP==0||Flag_PD_QP==3||Flag_PD_QP==2||Flag_PD_QP==4)
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-800,Zero_Angle*100+600);
					}
					else
					{
						Target_Angle.Y = range_protect((int32)Target_Angle.Y,Zero_Angle*100-600,Zero_Angle*100+600);
					}
       }
}
