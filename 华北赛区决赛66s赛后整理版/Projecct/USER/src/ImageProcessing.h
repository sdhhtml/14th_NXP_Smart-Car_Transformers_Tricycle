#ifndef _IMAGEPROCESSING_H
#define _IMAGEPROCESSING_H

#define ROW_C        60
#define COL_C        94
extern uint8 image[ROW_C][COL_C];//����ͼ������
struct Boundary{
	int16 Side_Line[ROW_C];//�б��� 
	int16 Mending_line[ROW_C];//�ޱ��� ���油�ߵ�����
	int16 Final_Line[ROW_C];//������ ���ڼ���
	int16 Roundabout[ROW_C];//��������
	uint8 Lossofthread_Flag[ROW_C];//0�б��� 1û����
	uint8 Jump_Count;//��������
	uint8 Row_Sum;//��Ч������
	uint8 Lose_Sum;//��������
	uint8 Next_Start;//��ɨ����ʼ������
	uint8 White_Flag;
};
extern float  BlackThres ;   //�ڰ���ֵ
extern unsigned char Flag_7H;
void Image_HandleZL(void);
void Image_HandleSL(void);
#endif
/**/
