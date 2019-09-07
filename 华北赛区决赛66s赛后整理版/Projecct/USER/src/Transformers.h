#ifndef _TRANSFORMERS_h
#define _TRANSFORMERS_h
#include "headfile.h"
void Control_Model_0(void);//比赛用的直立发车＋变形 直立障碍
void Control_Model_4(void);//比赛用的直立发车＋变形 三轮障碍
void Control_Model_6(void);//直立避障不变形
void Control_Model_7(void);//三轮避障不变形
void Control_Model_1(void);//练习用的三轮发车＋变形 直立障碍
void Control_Model_5(void);//练习用的三轮发车＋变形 三轮障碍

void Erect_Obstacle(void);//直立障碍
void Round_PT(void);//三轮普通带变形
void Erect_Round(void);//直立变三轮
void Round_Erect(void);//三轮变直立
void Erect_PT(void);//直立普通带变形
void Round_Obstacle(void);
void Round_Obstacle(void);//三轮障碍//三轮障碍
extern unsigned char Flag_R_CP;
extern unsigned char Flag_PDWP;
#endif
