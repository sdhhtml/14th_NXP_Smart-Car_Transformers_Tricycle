#ifndef _ELECTROMAGNETISM_H
#define _ELECTROMAGNETISM_H
#define BUZZ_open   gpio_set(E28,1) //��������
#define BUZZ_shut   gpio_set(E28,0)//��������
extern unsigned int AD_Value[7]; 
extern float DianCi_Offset;
extern float  Difference,Differencelast;
extern char Dif_Nor_coefficient;
extern float LAD,RAD,MAD,LADC,RADC;
extern char Roundabout_cnt,Roundabin_cnt,In_Roundabout_Flag,Out_Roundabout_Flag,In_Roundabout_Flag_O;
extern unsigned char Ring_Direction;//һ�����
extern float In_offset;
extern float Ring_In_offset;

extern float Inductance_Compensation;//��в���ϵ��
extern float Ring_Inductance_Compensation1;//��������б��в���ϵ��
extern float Ring_Inductance_Compensation2;//��������б��в���ϵ��
extern unsigned short int Flag7_time1;//����������
extern float Inductance_Compensation2;//��в���ϵ��
extern float Ring_Inductance_Compensation21;//��������б��в���ϵ��
extern float Ring_Inductance_Compensation22;//��������б��в���ϵ��
extern unsigned short int Flag7_time2;//����������

void Put_SDS(void);


void Inductance_init(void);
void Inductance_get(void);
void Inductance_filter(void);
void Inductance_filter2(void);
#endif
/**/
