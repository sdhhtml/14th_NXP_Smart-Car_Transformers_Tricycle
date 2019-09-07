#include "headfile.h"
char Offset_OK = 0;
float GravityAngle = 0;                       //���ٶȼƽǶ�
float GYROSCOPE_ANGLE_RATIO = 16.4204;             //�����Ǳ�������
float AngleSpeed_Y = 0;                       //��ֱ���ٶ�
float angle_y = 0;                           //�����ǽǶ�
float Gyrox_int=0;
 /*************************************************************************
*  ��������   Balnace_Sensor_Init
*  ����˵���� ��������ʼ������
*  ����˵����
*  �������أ� ��
*  �޸�ʱ�䣺 2019-5-3
*  ��    ע��
*************************************************************************/

void Balance_Sensor_Init(void)
{

    icm20602_init_spi();    	
    
    Attitude_Angle.Y = 0;
    Target_Angle.Y = 0;
    Tar_Ang_Vel.Y = 0;
    Tar_Ang_Vel.Z = 0;
}



/*************************************************************************
*  ��������   Balance_Sensor_Calculate
*  ����˵���� ���������㺯��
*  ����˵����
*  �������أ� ��
*  �޸�ʱ�䣺 2019-5-3
*  ��    ע��
*************************************************************************/
void Balance_Sensor_Calculate(void)
{  
    uint8 dat[2];
		icm_spi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 2);
    ACC.X=(int16)(((uint16)dat[0]<<8 | dat[1]));
//		icm_spi_r_reg_bytes(ICM20602_ACCEL_YOUT_H, dat, 2);
//    ACC.Y=(int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_spi_r_reg_bytes(ICM20602_ACCEL_ZOUT_H, dat, 2);
    ACC.Z=(int16)(((uint16)dat[0]<<8 | dat[1]));
    if(Offset_OK)  //������Ʈ
    {
				icm_spi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 2);
        GYRO.X = (int16)(((uint16)dat[0]<<8 | dat[1]))- GYRO_Offset.X;
				icm_spi_r_reg_bytes(ICM20602_GYRO_YOUT_H, dat, 2);
        GYRO.Y = (int16)(((uint16)dat[0]<<8 | dat[1])) - GYRO_Offset.Y;
				//GYRO.Y = (int16)(((uint16)dat[0]<<8 | dat[1]))+6;
//				icm_spi_r_reg_bytes(ICM20602_GYRO_ZOUT_H, dat, 2);
//        GYRO.Z = (int16)(((uint16)dat[0]<<8 | dat[1])) - GYRO_Offset.Z;// ��ȡ����������
    }
    else
    {
				icm_spi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 2);
        GYRO.X = (int16)(((uint16)dat[0]<<8 | dat[1]));
				icm_spi_r_reg_bytes(ICM20602_GYRO_YOUT_H, dat, 2);
        GYRO.Y = (int16)(((uint16)dat[0]<<8 | dat[1]));
//				icm_spi_r_reg_bytes(ICM20602_GYRO_ZOUT_H, dat, 2);
//        GYRO.Z = (int16)(((uint16)dat[0]<<8 | dat[1]));// ��ȡ����������
    }	
}
//float Gyro_int=0;
void Get_Attitude(void)
{
    //ֱ�������Ƕ� ͨ��������ٶȼƻ��
    GravityAngle = 57.3248*atan2(-ACC.Z,ACC.X); 
    if(GravityAngle>90)
     GravityAngle = 90;
    else if(GravityAngle<-90)
     GravityAngle = -90;
    AngleSpeed_Y = GYRO.Y/GYROSCOPE_ANGLE_RATIO;  
   // Gyro_int-=(AngleSpeed_Y*0.002);
    Attitude_Angle.Y = Kalman_Filter(GravityAngle,-AngleSpeed_Y);    //�������˲� 
}
float angle, angle_dot; 		//�ⲿ��Ҫ���õı���
//float Q_angle=0.02, Q_gyro=0.001, R_angle = 10, dt=0.005;  //Q��Ԥ��ֵ��Э���R�ǲ���ֵ��Э����0.001 0.03   30    0.0021

float Q_angle=0.001, Q_gyro=0.003, R_angle = 0.5, dt=0.002;  //Q��Ԥ��ֵ��Э���R�ǲ���ֵ��Э����0.001 0.03   30    0.0021

			
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }};

float Pdot[4] ={0,0,0,0};

const char C_0 = 1;

float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------

float Kalman_Filter(float angle_m,float gyro_m)			//gyro_m:gyro_measure,angle_m:angle_measure
{
	angle+=(gyro_m-q_bias) * dt;//�������

	Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' ����������Э�����΢��
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;

	P[0][0] += Pdot[0] * dt;  // Pk- ����������Э����΢�ֵĻ��� = ����������Э����
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;


	angle_err = angle_m - angle;//zk-�������


	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//����������Э����
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;


	angle	+= K_0 * angle_err;//�������
	q_bias	+= K_1 * angle_err;//�������
	angle_dot = gyro_m - q_bias;//���ֵ��������ƣ���΢�� = ���ٶ�
        
        return angle;
}



/***************************************************************
@Fucation �� ���ݻ����˲�
@Author �� bc
@Data �� 2019/5/3
@Note �� 
***************************************************************/
#define AcceRatio   16384.0f
#define GyroRatio   16.4204f
#define GyroRatioX   13.9891987295f
#define GyroRatioX2   16.30281481486f
#define Gyro_Gr		0.0010653	// ���ٶȱ�ɻ���	�˲�����Ӧ����2000��ÿ��
#define ACC_FILTER_NUM 5		// ���ٶȼ��˲����
#define GYRO_FILTER_NUM 1		// �������˲����

int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];	// �˲���������
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];

void Data_Filter(void)	// �����˲�
{
	int i;
	int64 temp1 = 0,// temp2 = 0,
	temp3 = 0, temp4 = 0, temp5 = 0;//, temp6 = 0;
	
	ACC_X_BUF[0] = ACC.X;	// ���»�����������
//	ACC_Y_BUF[0] = ACC.Y;
	ACC_Z_BUF[0] = ACC.Z;
	GYRO_X_BUF[0] = GYRO.X;
	GYRO_Y_BUF[0] = GYRO.Y;
//	GYRO_Z_BUF[0] = GYRO.Z;
	
	for(i=0;i<ACC_FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
//		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
		
	}
	for(i=0;i<GYRO_FILTER_NUM;i++)
	{
		temp4 += GYRO_X_BUF[i];
		temp5 += GYRO_Y_BUF[i];
//		temp6 += GYRO_Z_BUF[i];
	}
	
	ACC_Real.X = temp1 / ACC_FILTER_NUM / AcceRatio;
//	ACC_Real.Y = temp2 / ACC_FILTER_NUM / AcceRatio;
	ACC_Real.Z = temp3 / ACC_FILTER_NUM / AcceRatio;
	GYRO_Real.X = temp4 / GYRO_FILTER_NUM / GyroRatioX;
	GYRO_Real.Y = temp5 / GYRO_FILTER_NUM / GyroRatio;
	Gyrox_int+=GYRO_Real.X*0.002;
	//Gyrox_int+=GYRO_Real.X*0.002;
//	GYRO_Real.Z = temp6 / GYRO_FILTER_NUM / GyroRatio;
	
	for(i = 0; i < ACC_FILTER_NUM - 1; i++)
	{
		ACC_X_BUF[ACC_FILTER_NUM-1-i] = ACC_X_BUF[ACC_FILTER_NUM-2-i];
//		ACC_Y_BUF[ACC_FILTER_NUM-1-i] = ACC_Y_BUF[ACC_FILTER_NUM-2-i];
		ACC_Z_BUF[ACC_FILTER_NUM-1-i] = ACC_Z_BUF[ACC_FILTER_NUM-2-i];
		
	}
	for(i = 0; i < GYRO_FILTER_NUM - 1; i++)
	{
		GYRO_X_BUF[GYRO_FILTER_NUM-1-i] = GYRO_X_BUF[GYRO_FILTER_NUM-2-i];
		GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Y_BUF[GYRO_FILTER_NUM-2-i];
//		GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Z_BUF[GYRO_FILTER_NUM-2-i];
	}
}
void Data_Filter2(void)	// �����˲�
{
	int i;
	int64 temp1 = 0,// temp2 = 0,
	temp3 = 0, temp4 = 0, temp5 = 0;//, temp6 = 0;
	
	ACC_X_BUF[0] = ACC.X;	// ���»�����������
//	ACC_Y_BUF[0] = ACC.Y;
	ACC_Z_BUF[0] = ACC.Z;
	GYRO_X_BUF[0] = GYRO.X;
	GYRO_Y_BUF[0] = GYRO.Y;
//	GYRO_Z_BUF[0] = GYRO.Z;
	
	for(i=0;i<ACC_FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
//		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
		
	}
	for(i=0;i<GYRO_FILTER_NUM;i++)
	{
		temp4 += GYRO_X_BUF[i];
		temp5 += GYRO_Y_BUF[i];
//		temp6 += GYRO_Z_BUF[i];
	}
	
	ACC_Real.X = temp1 / ACC_FILTER_NUM / AcceRatio;
//	ACC_Real.Y = temp2 / ACC_FILTER_NUM / AcceRatio;
	ACC_Real.Z = temp3 / ACC_FILTER_NUM / AcceRatio;
	GYRO_Real.X = temp4 / GYRO_FILTER_NUM / GyroRatioX2;
	GYRO_Real.Y = temp5 / GYRO_FILTER_NUM / GyroRatio;
	//Gyrox_int+=GYRO_Real.X*0.002;
//	GYRO_Real.Z = temp6 / GYRO_FILTER_NUM / GyroRatio;
	
	for(i = 0; i < ACC_FILTER_NUM - 1; i++)
	{
		ACC_X_BUF[ACC_FILTER_NUM-1-i] = ACC_X_BUF[ACC_FILTER_NUM-2-i];
//		ACC_Y_BUF[ACC_FILTER_NUM-1-i] = ACC_Y_BUF[ACC_FILTER_NUM-2-i];
		ACC_Z_BUF[ACC_FILTER_NUM-1-i] = ACC_Z_BUF[ACC_FILTER_NUM-2-i];
		
	}
	for(i = 0; i < GYRO_FILTER_NUM - 1; i++)
	{
		GYRO_X_BUF[GYRO_FILTER_NUM-1-i] = GYRO_X_BUF[GYRO_FILTER_NUM-2-i];
		GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Y_BUF[GYRO_FILTER_NUM-2-i];
//		GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Z_BUF[GYRO_FILTER_NUM-2-i];
	}
}
void Sensor_Offset(void)  ///��ʵ�����ϵ���ȡƽ��ֵ
{
	uint8 i, Count = 100;
	int64 temp[6] = {0};
	
	GYRO_Offset.X = 0;
	GYRO_Offset.Y = 0;
	GYRO_Offset.Z = 0;
	
	for (i = 0; i < Count; i++)
	{
	        Balance_Sensor_Calculate();// ��ȡ����������
		Delay_ms(2);
		
		temp[0] += ACC.X;
//		temp[1] += ACC.Y;
		temp[2] += ACC.Z;
		
		temp[3] += GYRO.X;
		temp[4] += GYRO.Y;
//		temp[5] += GYRO.Z;
	}
	ACC_Offset.X = temp[0] / Count;
//	ACC_Offset.Y = temp[1] / Count;
	ACC_Offset.Z = temp[2] / Count;
	
	GYRO_Offset.X = temp[3] / Count;
	GYRO_Offset.Y = temp[4] / Count;
//	GYRO_Offset.Z = temp[5] / Count;
	
	Offset_OK = 1;
}
