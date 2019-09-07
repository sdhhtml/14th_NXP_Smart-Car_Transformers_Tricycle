#ifndef _IMAGEPROCESSING_H
#define _IMAGEPROCESSING_H

#define ROW_C        60
#define COL_C        94
extern uint8 image[ROW_C][COL_C];//处理图像数组
struct Boundary{
	int16 Side_Line[ROW_C];//有边线 
	int16 Mending_line[ROW_C];//无边线 储存补线的数据
	int16 Final_Line[ROW_C];//最终线 用于计算
	int16 Roundabout[ROW_C];//环岛边线
	uint8 Lossofthread_Flag[ROW_C];//0有边线 1没边线
	uint8 Jump_Count;//跳变点个数
	uint8 Row_Sum;//有效行数量
	uint8 Lose_Sum;//丢线数量
	uint8 Next_Start;//行扫描起始横坐标
	uint8 White_Flag;
};
extern float  BlackThres ;   //黑白阈值
extern unsigned char Flag_7H;
void Image_HandleZL(void);
void Image_HandleSL(void);
#endif
/**/
