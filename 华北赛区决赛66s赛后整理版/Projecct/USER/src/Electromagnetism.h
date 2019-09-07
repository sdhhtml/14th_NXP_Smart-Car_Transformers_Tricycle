#ifndef _ELECTROMAGNETISM_H
#define _ELECTROMAGNETISM_H
#define BUZZ_open   gpio_set(E28,1) //蜂鸣器开
#define BUZZ_shut   gpio_set(E28,0)//蜂鸣器关
extern unsigned int AD_Value[7]; 
extern float DianCi_Offset;
extern float  Difference,Differencelast;
extern char Dif_Nor_coefficient;
extern float LAD,RAD,MAD,LADC,RADC;
extern char Roundabout_cnt,Roundabin_cnt,In_Roundabout_Flag,Out_Roundabout_Flag,In_Roundabout_Flag_O;
extern unsigned char Ring_Direction;//一左二右
extern float In_offset;
extern float Ring_In_offset;

extern float Inductance_Compensation;//电感补偿系数
extern float Ring_Inductance_Compensation1;//环岛中心斜电感补偿系数
extern float Ring_Inductance_Compensation2;//环岛中心斜电感补偿系数
extern unsigned short int Flag7_time1;//出环岛参数
extern float Inductance_Compensation2;//电感补偿系数
extern float Ring_Inductance_Compensation21;//环岛中心斜电感补偿系数
extern float Ring_Inductance_Compensation22;//环岛中心斜电感补偿系数
extern unsigned short int Flag7_time2;//出环岛参数

void Put_SDS(void);


void Inductance_init(void);
void Inductance_get(void);
void Inductance_filter(void);
void Inductance_filter2(void);
#endif
/**/
