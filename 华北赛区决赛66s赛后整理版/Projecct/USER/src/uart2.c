#include "headfile.h"
void UART2_RX_TX_IRQHandler(void)
{
    if(UART2->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
			//gpio_turn(C2);
			uart_getchar(uart2,&Res2);
			switch(step2)
			{
				case 0:
					if(Res2==0xAA)
					{
						buf2[0]=Res2;
						step2=1;
					}
					else
					{
						step2=0;
					}
					break;
				case 1:
					if(Res2==0x55)
					{
						buf2[1]=Res2;
						step2=2;
					}
					else
					{
						step2=0;
					}
					break;
				case 2:
					buf2[2]=Res2;
					step2=3;
					break;
				case 3:
					buf2[3]=Res2;
					step2=4;
					break;
				case 4:
					buf2[4]=Res2;
					step2=5;
					break;
				case 5:
					buf2[5]=Res2;
					step2=6;
					break;
				case 6:
					buf2[6]=Res2;
					step2=7;
					break;
				case 7:
					buf2[7]=buf2[0]+buf2[1]+buf2[2]+
									buf2[3]+buf2[4]+buf2[5]+
									buf2[6];
					if(buf2[7]==Res2)
					{
						//Distance2=(buf2[5]+(buf2[6]<<8))/1000.0f;
						Distance2=buf2[5]+(buf2[6]<<8);
						buf22[Count_buf22]=Distance2;
						Speed22[Count_buf22]=Speed_Now;
						//Buf2_Num+=buf22[Count_buf22];
						Count_buf22++;
						if(Count_buf22>4)
						{
							Count_buf22=0;
							//排序
							for(j22=0;j22<4;j22++)
							{
								 for(k22=0;k22<4-j22;k22++)
								 {
										if(buf22[k22] > buf22[k22+1])        //前面的比后面的大  则进行交换
										{
											 temp22 = buf22[k22+1];
											 buf22[k22+1] = buf22[k22];
											 buf22[k22] = temp22;
										}
								 }
							}
							/////
							Buf2_Num=buf22[1]+buf22[2]+buf22[3];
							Distance22=Buf2_Num/3;
							//if(Flag_Round_ZA==0)
							if(System_mode==0||System_mode==1||System_mode==6)
							{
													if(Flag_Zhi_ZA==0)
													{
													Speed22_Num=(Speed22[0]+Speed22[1]+Speed22[2]+Speed22[3]+Speed22[4])/10;//50ms内的距离变化 编码器
													Dis2[0]=Distance22;
													Dis2_Error=Dis2[1]-Dis2[0];//50ms内的距离变化，来自于激光
													Dis2[1]=Dis2[0];
														if(Distance22<=960&&Distance22>=400)
														{
															if(Flag_R_ZA==0)
															{
																Flag_R_ZA=1;
															}
															if(ABS(Dis2_Error-Speed22_Num)<40)
															{
																R_ZA_Count++;
																if(R_ZA_Count>2&&Flag_R_ZA==1&&Distance22<=800)
																{
																	Flag_R_ZA=2;
																}
															}
															else
															{
																R_ZA_Count=0;
															}
														}
														else 
														{
															Speed22_Num=0;
															Dis2_Error=0;
															Flag_R_ZA=0;
															R_ZA_Count=0;
														}
													}
													else//在闭环内
													{
														Speed22_Num=0;
															Dis2_Error=0;
															Flag_R_ZA=0;
															R_ZA_Count=0;
													}
							}//直立避障
							else if(System_mode==4||System_mode==5||System_mode==7)
							{
													if(Flag_Round_ZA==0)
													{
													Speed22_Num=(Speed22[0]+Speed22[1]+Speed22[2]+Speed22[3]+Speed22[4])/10;//50ms内的距离变化 编码器
													Dis2[0]=Distance22;
													Dis2_Error=Dis2[1]-Dis2[0];//50ms内的距离变化，来自于激光
													Dis2[1]=Dis2[0];
														if(Distance22<=960&&Distance22>=400)
														{
															if(Flag_R_ZA==0)
															{
																Flag_R_ZA=1;
															}
															if(ABS(Dis2_Error-Speed22_Num)<40)
															{
																R_ZA_Count++;
																if(R_ZA_Count>2&&Flag_R_ZA==1&&Distance22<=800)
																{
																	Flag_R_ZA=2;
																}
															}
															else
															{
																R_ZA_Count=0;
															}
														}
														else 
														{
															Speed22_Num=0;
															Dis2_Error=0;
															Flag_R_ZA=0;
															R_ZA_Count=0;
														}
													}
													else//在闭环内
													{
														Speed22_Num=0;
															Dis2_Error=0;
															Flag_R_ZA=0;
															R_ZA_Count=0;
													}
							}//三轮避障
						}
					}
					step2=0;
					break;
				default:
					step2=0;
					break;
			}
    }
    if(UART2->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}
