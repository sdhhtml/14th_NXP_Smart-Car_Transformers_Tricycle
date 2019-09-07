/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		SEEKFREE_IPS.c
 * @brief      		IPS Һ������
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.2 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-12
 * @note		
					IPSҺ�����߶��壺
					------------------------------------ 
						ģ��ܽ�            ��Ƭ���ܽ�
						SCL                 A15
						SDA                 A16
						RES                 B17
						DC                  B16
						CS                  A14
						
						��Դ����
						BL  3.3V��Դ������������ţ�Ҳ���Խ�PWM���������ȣ�
						VCC 3.3V��Դ
						GND ��Դ��
	
					------------------------------------ 

 ********************************************************************************************************************/


#ifndef _SEEKFREE_IPS_H
#define _SEEKFREE_IPS_H


#include "headfile.h"


//-----------------���Ŷ���------------------------------
#define IPS_BL_PIN      D2	            //Һ���������Ŷ���
#define IPS_REST_PIN    D0             //Һ����λ���Ŷ���
#define IPS_DC_PIN 	    D1	            //Һ������λ���Ŷ���


#define IPS_DC(x)       gpio_set(IPS_DC_PIN,x);
#define IPS_REST(x)     gpio_set(IPS_REST_PIN,x);


//-----------------Һ����С����------------------------------
#define IPS_SIZE        0           //1.14�� IPSҺ��
//#define IPS_SIZE        1           //1.3��  IPSҺ��(��δ�Ƴ�)

#if  (0 == IPS_SIZE)
#define X_MAX   135
#define Y_MAX   240

#elif  (1 == IPS_SIZE)
#define X_MAX   240
#define Y_MAX   240

#else
#error "IPS_SIZE �������"
     
#endif



//-----------------������ɫ------------------------------
#define IPS_RED     	0XF800      //��ɫ
#define IPS_GREEN   	0X07E0      //��ɫ
#define IPS_BLUE    	0X001F      //��ɫ
#define IPS_BRED    	0XF81F  
#define IPS_GRED    	0XFFE0      //��ɫ
#define IPS_GBLUE   	0X07FF      //
#define IPS_BLACK   	0X0000      //��ɫ
#define IPS_WHITE   	0XFFFF      //��ɫ
#define IPS_YELLOW  	0xFFE0      //��ɫ


//����д�ֱʵ���ɫ
#define IPS_PENCOLOR    IPS_RED

//���屳����ɫ
#define IPS_BGCOLOR     IPS_WHITE



//������ʾ����
//0 ����ģʽ
//1 ����ģʽ  ��ת180
//2 ����ģʽ
//3 ����ģʽ  ��ת180
#define IPS_DISPLAY_DIR 3

#if (0==IPS_DISPLAY_DIR || 1==IPS_DISPLAY_DIR)
#define	IPS_X_MAX	X_MAX	//Һ��X�����
#define IPS_Y_MAX	Y_MAX   //Һ��Y�����
     
#elif (2==IPS_DISPLAY_DIR || 3==IPS_DISPLAY_DIR)
#define	IPS_X_MAX	Y_MAX	//Һ��X�����
#define IPS_Y_MAX	X_MAX   //Һ��Y�����
     
#else
#error "IPS_DISPLAY_DIR �������"
     
#endif


void ips_init(void);
void ips_clear(uint16 color);
void ips_drawpoint(uint16 x,uint16 y,uint16 color);
void ips_showchar(uint16 x,uint16 y,uint8 dat);
void ips_showstr(uint16 x,uint16 y,uint8 dat[]);
void ips_showint8(uint16 x,uint16 y,int8 dat);
void ips_showuint8(uint16 x,uint16 y,uint8 dat);
void ips_showint16(uint16 x,uint16 y,int16 dat);
void ips_showuint16(uint16 x,uint16 y,uint16 dat);
void ips_displayimage032(uint8 *p, uint16 width, uint16 height);
void ips_displayimage032_zoom(uint8 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips_displayimage7725(uint8 *p, uint16 width, uint16 height);
void ips_display_chinese(uint16 x, uint16 y, uint8 size, const uint8 *p, uint8 number, uint16 color);

#endif
