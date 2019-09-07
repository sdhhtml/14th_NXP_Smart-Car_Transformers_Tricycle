/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		ICM20602
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看common.h内VERSION宏定义
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					接线定义：
					------------------------------------ 
						SCL                 查看SEEKFREE_IIC文件内的SEEKFREE_SCL宏定义
						SDA                 查看SEEKFREE_IIC文件内的SEEKFREE_SDA宏定义
					------------------------------------ 
 ********************************************************************************************************************/


#include "SEEKFREE_ICM20602.h"
#include "imu.h"

int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16 icm_acc_x,icm_acc_y,icm_acc_z;
//extern INT16_XYZ	 ICM20602_ACC_RAW,ICM20602_GYRO_RAW;	     	 //读取值原始数据

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self1_check(void)
{
    while(0x12 != simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,IIC)) //读取ICM20602 ID
    {
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init(void)
{
    systick_delay_ms(10);  //上电延时
    
    //检测
    icm20602_self1_check();
    
    //复位
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);               //复位设备
    systick_delay_ms(2);                                                        //延时
    while(0x80 & simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,IIC));//等待复位完成
    
    //配置参数
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);               //时钟设置
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //开启陀螺仪和加速度计
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x04);                   //176HZ 1KHZ
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x00);               //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //±2000 dps
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x01);             //±4g
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x00);           //Average 8 samples   44.8HZ
}


//内部使用用户无需调用
int16 icm_get(uint8 REG_Address)
{
    uint8 L;   uint16 H ;
    H=simiic_read_reg(ICM20602_DEV_ADDR, REG_Address, IIC);
    L=simiic_read_reg(ICM20602_DEV_ADDR, REG_Address+1, IIC);
    return (H<<8)+L;   //合成数据
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata(void)
{
    icm_acc_x = icm_get(ICM20602_ACCEL_XOUT_H);
    icm_acc_y = icm_get(ICM20602_ACCEL_YOUT_H);
    icm_acc_z = icm_get(ICM20602_ACCEL_ZOUT_H);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro(void)
{
    icm_gyro_x = icm_get(ICM20602_GYRO_XOUT_H);
    icm_gyro_y = icm_get(ICM20602_GYRO_YOUT_H);
    icm_gyro_z = icm_get(ICM20602_GYRO_ZOUT_H);

}

//-------------------------------------------------------------------------------------------------------------------
//  以上函数是使用软件IIC通信，相比较硬件IIC，软件IIC引脚更加灵活，可以使用任意普通IO
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  以下函数是使用硬件SPI通信，相比较IIC，速度比IIC快非常多。
//-------------------------------------------------------------------------------------------------------------------
/*
#define SPI_NUM         SPI_4           
#define SPI_SCK_PIN     SPI4_SCK_B19    //接模块SPC
#define SPI_MOSI_PIN    SPI4_MOSI_B21   //接模块SDI
#define SPI_MISO_PIN    SPI4_MISO_B20   //接模块SDO
#define SPI_CS_PIN      SPI4_CS0_B9     //接模块CS

#define SPI1_SCK_PIN    B11       // E2、B11、          全部都是 ALT2
#define SPI1_SOUT_PIN   B16       // E1、B16、          全部都是 ALT2
#define SPI1_SIN_PIN    B17       // E3、B17、          全部都是 ALT2

#define SPI1_PCS0_PIN   B10       // E4、B10、          全部都是 ALT2
*/
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI写寄存器
//  @param      cmd     寄存器地址
//  @param      val     需要写入的数据
//  @return     void					
//  @since      v1.0
//  Sample usage:	
//   spi_mosi_cmd(spi0,SPI_PCS0,cmd,NULL,buff,buff,1,2);
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_w_reg_byte(uint8 cmd, uint8 val)
{
    cmd |= ICM20602_SPI_W;
    spi_mosi_cmd(spi1,SPI_PCS0 ,&cmd,NULL,&val,NULL,1,1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI读寄存器
//  @param      cmd     寄存器地址
//  @param      *val    接收数据的地址
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_byte(uint8 cmd, uint8 *val)
{
    cmd |= ICM20602_SPI_R;
    spi_mosi_cmd(spi1,SPI_PCS0,&cmd,NULL,val,val,1,1);
}
  
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI多字节读寄存器
//  @param      cmd     寄存器地址
//  @param      *val    接收数据的地址
//  @param      num     读取数量
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_bytes(uint8 cmd, uint8 * val, uint8 num)
{
    cmd |= ICM20602_SPI_R;
    spi_mosi_cmd(spi1,SPI_PCS0,&cmd,NULL,val,val,1,num);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self3_check(void)
{
    uint8 val;
    do
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&val);
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }while(0x12 != val);

}
     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
    uint8 val = 0x0;

    systick_delay_ms(10);  //上电延时
    
    (void)spi_init(spi1, SPI_PCS0,MASTER,25*1000*1000);
    
 //   gpio_init(DC_PIN,GPO,0);
   // gpio_init(REST_PIN,GPO,0);
    
    icm20602_self3_check();//检测
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//复位设备
    systick_delay_ms(2);
    do
    {//等待复位成功
        icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
       // printf("\r\n%c",val);
    }while(0x41 != val);
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);            //时钟设置
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);            //开启陀螺仪和加速度计
    icm_spi_w_reg_byte(ICM20602_CONFIG,         0x03);            //176HZ 1KHZ
    icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x00);            //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);            //±2000 dps
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x08);            //±8g
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x00);            //Average 8 samples   44.8HZ  
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(void)
{
    uint8 dat[2];
    
    icm_spi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 2);
    
//    ICM20602_ACC_RAW.X=(int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_spi_r_reg_bytes(ICM20602_ACCEL_ZOUT_H, dat, 2);
 //   ICM20602_ACC_RAW.Z=(int16)(((uint16)dat[0]<<8 | dat[1]));
    
   // icm_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    //icm_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    //icm_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
    //printf("\r\n%d,%d,%d\t",icm_acc_x,icm_acc_y,icm_acc_z);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(void)
{
    uint8 dat[2];
    
    //icm_spi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6);
//		icm_spi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 2);
//    ICM20602_GYRO_RAW.X=(int16)(((uint16)dat[0]<<8 | dat[1]))+12;//三轮逆正
	
    icm_spi_r_reg_bytes(ICM20602_GYRO_YOUT_H, dat, 2);
//    ICM20602_GYRO_RAW.Y=(int16)(((uint16)dat[0]<<8 | dat[1]))+5;
	
//		icm_spi_r_reg_bytes(ICM20602_GYRO_ZOUT_H, dat, 2);
//    ICM20602_GYRO_RAW.Z=(int16)(((uint16)dat[0]<<8 | dat[1]))-15;//逆负
   //icm_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    //icm_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
   // icm_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
   //printf("\r\n%d,%d,%d",ICM20602_GYRO_RAW.X,ICM20602_GYRO_RAW.Y,ICM20602_GYRO_RAW.Z);
}






