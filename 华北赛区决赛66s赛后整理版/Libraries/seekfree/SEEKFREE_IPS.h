/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_IPS.c
 * @brief      		IPS 液晶驱动
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.2 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-12
 * @note		
					IPS液晶接线定义：
					------------------------------------ 
						模块管脚            单片机管脚
						SCL                 A15
						SDA                 A16
						RES                 B17
						DC                  B16
						CS                  A14
						
						电源引脚
						BL  3.3V电源（背光控制引脚，也可以接PWM来控制亮度）
						VCC 3.3V电源
						GND 电源地
	
					------------------------------------ 

 ********************************************************************************************************************/


#ifndef _SEEKFREE_IPS_H
#define _SEEKFREE_IPS_H


#include "headfile.h"


//-----------------引脚定义------------------------------
#define IPS_BL_PIN      D2	            //液晶背光引脚定义
#define IPS_REST_PIN    D0             //液晶复位引脚定义
#define IPS_DC_PIN 	    D1	            //液晶命令位引脚定义


#define IPS_DC(x)       gpio_set(IPS_DC_PIN,x);
#define IPS_REST(x)     gpio_set(IPS_REST_PIN,x);


//-----------------液晶大小定义------------------------------
#define IPS_SIZE        0           //1.14寸 IPS液晶
//#define IPS_SIZE        1           //1.3寸  IPS液晶(暂未推出)

#if  (0 == IPS_SIZE)
#define X_MAX   135
#define Y_MAX   240

#elif  (1 == IPS_SIZE)
#define X_MAX   240
#define Y_MAX   240

#else
#error "IPS_SIZE 定义错误"
     
#endif



//-----------------常用颜色------------------------------
#define IPS_RED     	0XF800      //红色
#define IPS_GREEN   	0X07E0      //绿色
#define IPS_BLUE    	0X001F      //蓝色
#define IPS_BRED    	0XF81F  
#define IPS_GRED    	0XFFE0      //灰色
#define IPS_GBLUE   	0X07FF      //
#define IPS_BLACK   	0X0000      //黑色
#define IPS_WHITE   	0XFFFF      //白色
#define IPS_YELLOW  	0xFFE0      //黄色


//定义写字笔的颜色
#define IPS_PENCOLOR    IPS_RED

//定义背景颜色
#define IPS_BGCOLOR     IPS_WHITE



//定义显示方向
//0 竖屏模式
//1 竖屏模式  旋转180
//2 横屏模式
//3 横屏模式  旋转180
#define IPS_DISPLAY_DIR 3

#if (0==IPS_DISPLAY_DIR || 1==IPS_DISPLAY_DIR)
#define	IPS_X_MAX	X_MAX	//液晶X方宽度
#define IPS_Y_MAX	Y_MAX   //液晶Y方宽度
     
#elif (2==IPS_DISPLAY_DIR || 3==IPS_DISPLAY_DIR)
#define	IPS_X_MAX	Y_MAX	//液晶X方宽度
#define IPS_Y_MAX	X_MAX   //液晶Y方宽度
     
#else
#error "IPS_DISPLAY_DIR 定义错误"
     
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
