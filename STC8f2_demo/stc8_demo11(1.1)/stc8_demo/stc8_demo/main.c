
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include <intrins.h>
#include "Lensdrv.h"
#include "EEPROM.h"
#include <stdlib.h>
#include <stdio.h>

#define CKSEL		(*(unsigned char volatile xdata*)0xfe00)
#define	CLKDIV		(*(unsigned char volatile xdata*)0xfe01)
#define	IRC24MCR	(*(unsigned char volatile xdata*)0xfe02)
#define	XOSCCR		(*(unsigned char volatile xdata*)0xfe03)
#define	IRC32KCR	(*(unsigned char volatile xdata*)0xfe04)


xdata enum PELCO_D_CMD
{
	ZOOM_IN = 0,																																	/* 变倍短 (放大+)   0*/
	ZOOM_OUT,																																			/* 变倍长 缩小- 	  1*/
	FOCUS_NEAR,																																		/* 聚焦近 聚焦+	  	2*/
	FOCUS_FAR,																																		/* 聚焦远 聚焦- 		3*/
	STOP,																																					/* 停止	 						4*/
	SELF_TEST_ZOOM,
	SELF_TEST_FOCUS,
	INVALID
};

xdata enum LEN_SPEED
{
	FAST = 0x0400,
	FAST_MIDDLE = 0x0800,
	MIDDLE = 0x0C00,/*0x5555,0xAAAA,0x1500*/
	SLOW = 0xFFFF,
};

/****************************************************************************
PELCO_D 协议说明：
数据格式：1位起始位、8位数据、1位停止位、无校验位;
命令格式：|  字节1   |  字节2 |  字节3   |   字节4  |  字节5   |   字节6  |  字节7 |
					  同步字节   地址码    指令码1    指令码2    数据码1    数据码2		校验码

1.该协议中所有数值都为十六进制数
2.同步字节始终为FFH
3.地址码为摄像机的逻辑地址号，地址范围：00H~FFH
4.指令码表示不同的动作
5.数据码1、2分别表示水平、垂直方向速度(00~3FH),FFH表示“turbo” 速度
6.校验码 = MOD[(字节2+字节3+字节4+字节5+字节6)/100H]
			以下地址码为0x01
****************************************************************************/

xdata unsigned char PELCO_D[][7] = {
	/* 变倍短 放大+ */
	{0xff,0x01,0x00,0x20,0x00,0x00,0x21},
	/* 变倍长 缩小- */
	{0xff,0x01,0x00,0x40,0x00,0x00,0x41},
	/* 聚焦近 聚焦+ */
	{0xff,0x01,0x00,0x80,0x00,0x00,0x81},
	/* 聚焦远 聚焦- */
	{0xff,0x01,0x01,0x00,0x00,0x00,0x02},
	/* 停止 */
	{0xff,0x01,0x00,0x00,0x00,0x00,0x01},
};



void delay_1(void)
{
	char i = 0;
	for(i = 254; i > 0; i--);
	
}


unsigned char SELF_TEST_ZOOM_FLAG = 0;
unsigned char SELF_TEST_FOCUS_FLAG = 0;
unsigned char CNT_FLAG = 0;


/****************************************************************************
* 名		称: void SELF_TEST_UnInit(void)
* 功		能: 给eeprom地址写入数值，结束首次变倍电机 焦距电机自检
* 入口参数: 无
* 出口参数: 无
* 说		明:
* 调用方法:
****************************************************************************/
void SELF_TEST_UnInit(void)
{
	IapErase(0x0400);
	IapErase(0x0401);
	IapErase(0x0402);
	IapErase(0x0403);
	IapErase(0x0404);
	IapProgram(0x0400, 0xFF);
	IapProgram(0x0401, 0xFF);
	IapProgram(0x0402, 0xFF);
	IapProgram(0x0403, 0xFF);
	IapProgram(0x0404, 0xFF);
}



/****************************************************************************
* 名		称: void SAVE_DATA_TO_EEPROM(unsigned char zoomH, unsigned char zoomL, unsigned char focusH, unsigned char focusL)

* 功		能: 把数据保存在eeprom中
* 入口参数: 无
* 出口参数: 无
* 说		明:
* 调用方法:
****************************************************************************/
void SAVE_DATA_TO_EEPROM(unsigned char zoomH, unsigned char zoomL, unsigned char focusH, unsigned char focusL)
{
	IapErase(0x0400);
	IapErase(0x0401);
	IapErase(0x0402);
	IapErase(0x0403);
	IapProgram(0x0400, zoomH); //zoom 高位数据
	IapProgram(0x0401, zoomL); //zoom 低位数据
	IapProgram(0x0402, focusH);//focus 高位数据
	IapProgram(0x0403, focusL);//focus 低位数据
}




/****************************************************************************
* 名		称: unsigned short CHAR_TO_SHORT(unsigned char data1, unsigned char data2)
* 功		能: 将char类型数据转换为short类型数据
* 入口参数: unsigned char , unsigned char
* 出口参数: unsigned short
* 说		明: 返回0 为传参错误
* 调用方法:
****************************************************************************/
unsigned short CHAR_TO_SHORT(unsigned char data1, unsigned char data2)
{
	unsigned char dat[2] = {0};
	if((dat[0] >= 0)&&(dat[1] >= 0)&&(dat[0] <= 0xFF)&&(dat[1] <= 0xFF))
	{
		dat[0] = data1;
		dat[1] = data2;
		return (dat[0]<<8)|dat[1];	
	}
	else
	{
		return 0;
	}
}
	


/****************************************************************************
* 名		称: int SHORT_TO_CHAR(unsigned char *data1, unsigned char *data2, unsigned short)
* 功		能: 将short类型数据转换为char类型数据
* 入口参数: unsigned short
* 出口参数: unsigned char* , unsigned char*
* 说		明: 返回1 转换成功，返回0 传参错误
* 调用方法:
****************************************************************************/
int SHORT_TO_CHAR(unsigned char *data1, unsigned char *data2, unsigned short dat)
{
	if(dat >= 0)
	{
		*data1 = dat >> 8;
		*data2 = dat;	
		return 1;
	}
	else
	{
		return 0;
	}
}




int INT_TO_CHAR(unsigned char *data1, unsigned char *data2, unsigned char *data3, unsigned char *data4,unsigned int dat)
{
	if(dat >= 0)
	{
		*data1 = dat >> 24;
		*data2 = dat >> 16;	
		*data2 = dat >> 8;
		*data2 = dat;
		return 1;
	}
	else
	{
		return 0;
	}
}


/****************************************************************************
* 名		称: void SYS_Init(void)
* 功		能: 系统初始化 使用内部24MHz时钟
* 入口参数: 无
* 出口参数: 无
* 说		明:
* 调用方法:
****************************************************************************/
void SYS_Init(void)
{
	CKSEL = 0x48;	//0100 1000
	/*
	选择内部24Mhz高精度IRC		(MCKSEL[1:0])		 ;
	主时钟分频输出到 P1.6 脚 （MCLKO_S）			 ;
	主时钟分频输出频率MCLK/4  (MCLKODIV[3:0])  ;
	*/
}



void Delay100ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();//1us延时
	_nop_();//1us延时
	i = 13;
	j = 45;
	k = 214;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}



void Delay150ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();
	i = 19;
	j = 68;
	k = 67;
	do
	{
		do
		{
			while (--k);
		}
		while (--j);	
	} 
	while (--i);
}





void Delay200ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();
	_nop_();
	i = 25;
	j = 90;
	k = 176;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}



/*实现printf打印*/
char putchar (char c)
{
	ES = 0;         //禁止串口1中断
	SBUF = c;				
	while(TI==0);		//等待发送完成   标志置1时发送完成
	TI = 0;					//清中断标志
//	ES = 1;				//允许串口1中断
	return 0;
		
}
	






/****************************************************************************
* 名		称: void main(void)
* 功		能: 程序主入口
* 入口参数: 无
* 出口参数: 无
* 说		明:
* 调用方法:
****************************************************************************/
void main(void)
{
													/* xdata声明的变量会存储到外部扩展RAM（Flash）,一般指外部0x0000-0xffff空间,用DPTR访问 */
  xdata unsigned short FcStep = 0;
	xdata unsigned short ZoomStep = 0;    //ZoomStep
	xdata unsigned char FcStepCnt = 0;
	xdata unsigned char ZoomStepCnt = 0;  //ZoomStepCnt
	
	//总步数
	xdata unsigned int ZoomStepNum = 0; 
	xdata unsigned int FcStepNum = 0; 

	xdata char LenStatus = STOP;																	//STOP == PELCO_D的停止指令
	
	//自检
	unsigned int self_test_zoom = 0; 
	unsigned int self_test_focus = 0;

	unsigned char ZoomStepTmp[4] = {0};
	unsigned char FcStepTmp[4] = {0};
	unsigned char num_s=65;
	char tmp_flag = 0;																						//没用到这个
	
	SYS_Init();		//系统初始化 使用内部24Mhz高精度IRC时钟
	Timer4Init();
	UART1_Init();
  SPI_Init(); 
	SPI_Speed(1);


	MS_Rest(); //41918
	LensDrvInit(); //镜头初始化
	LensDrvIrisOpen();//开镜头
	
	
	while (1)
	{		
	//	printf("1");
	//		printf("%bd",num_s);
		UartSendByte('b');
		
			if(UART1_Overflow_Flag)						//缓冲区满标志		（ UART1_Recv_Buf[] 已存放了可以比较的数据）
			{
					/* 变倍短 (放大+) */
					if(	 	 (UART1_Recv_Buf[0] == PELCO_D[ZOOM_IN][0])&&
								 (UART1_Recv_Buf[1] == PELCO_D[ZOOM_IN][1])&&
								 (UART1_Recv_Buf[2] == PELCO_D[ZOOM_IN][2])&&
								 (UART1_Recv_Buf[3] == PELCO_D[ZOOM_IN][3])&&
								 (UART1_Recv_Buf[4] == PELCO_D[ZOOM_IN][4])&&
								 (UART1_Recv_Buf[5] == PELCO_D[ZOOM_IN][5])&&
								 (UART1_Recv_Buf[6] == PELCO_D[ZOOM_IN][6]))
					{
							LenStatus	= ZOOM_IN;
					}
					/* 变倍长 (缩小-) */
					else if((UART1_Recv_Buf[0] == PELCO_D[ZOOM_OUT][0])&&
									(UART1_Recv_Buf[1] == PELCO_D[ZOOM_OUT][1])&&
									(UART1_Recv_Buf[2] == PELCO_D[ZOOM_OUT][2])&&
									(UART1_Recv_Buf[3] == PELCO_D[ZOOM_OUT][3])&&
									(UART1_Recv_Buf[4] == PELCO_D[ZOOM_OUT][4])&&
									(UART1_Recv_Buf[5] == PELCO_D[ZOOM_OUT][5])&&
									(UART1_Recv_Buf[6] == PELCO_D[ZOOM_OUT][6]))
					{
						LenStatus	= ZOOM_OUT;
					}
					/* 聚焦近 (聚焦+) */
					else if((UART1_Recv_Buf[0] == PELCO_D[FOCUS_NEAR][0])&&(UART1_Recv_Buf[1] == PELCO_D[FOCUS_NEAR][1])&&(UART1_Recv_Buf[2] == PELCO_D[FOCUS_NEAR][2])&&(UART1_Recv_Buf[3] == PELCO_D[FOCUS_NEAR][3])
						 &&(UART1_Recv_Buf[4] == PELCO_D[FOCUS_NEAR][4])&&(UART1_Recv_Buf[5] == PELCO_D[FOCUS_NEAR][5])&&(UART1_Recv_Buf[6] == PELCO_D[FOCUS_NEAR][6]))					
					{	
						 	LenStatus	= FOCUS_NEAR;						
					}
					/* 聚焦远 (聚焦-) */
					else if((UART1_Recv_Buf[0] == PELCO_D[FOCUS_FAR][0])&&(UART1_Recv_Buf[1] == PELCO_D[FOCUS_FAR][1])&&(UART1_Recv_Buf[2] == PELCO_D[FOCUS_FAR][2])&&(UART1_Recv_Buf[3] == PELCO_D[FOCUS_FAR][3])
						 &&(UART1_Recv_Buf[4] == PELCO_D[FOCUS_FAR][4])&&(UART1_Recv_Buf[5] == PELCO_D[FOCUS_FAR][5])&&(UART1_Recv_Buf[6] == PELCO_D[FOCUS_FAR][6]))					
					{
							LenStatus	= FOCUS_FAR;
					}
					/* 停止 */
					else if((UART1_Recv_Buf[0] == PELCO_D[STOP][0])&&(UART1_Recv_Buf[1] == PELCO_D[STOP][1])&&(UART1_Recv_Buf[2] == PELCO_D[STOP][2])&&(UART1_Recv_Buf[3] == PELCO_D[STOP][3])
						 &&(UART1_Recv_Buf[4] == PELCO_D[STOP][4])&&(UART1_Recv_Buf[5] == PELCO_D[STOP][5])&&(UART1_Recv_Buf[6] == PELCO_D[STOP][6]))					
					{
							LenStatus	= STOP;
              tmp_flag = 0;						
					}
				Receive_count = 0;   				//缓存清零 																		(接收计数清0						 	)
				UART1_Overflow_Flag = 0;		//允许串口继续接收数据 												 （可进UART1中断进行数据接收）
			}		
			
#if 1
			switch(LenStatus)																														//经过比较对LenStatus赋值开始做相应动作							
			{
				case ZOOM_IN: //放大
				/*  每次进来ZoomStepCnt++;驱动镜头速度相对慢;反转;进来的第21次 Delay100ms; 		*/									
							if(20 >= ZoomStepCnt)
							{
								ZoomStep = 53;//100
							  LensDrvZoomSpeed(MIDDLE);											//MIDDLE = 0x0C00		//驱动镜头速度（相对慢）					
							}

							else
							{																																																 //FAST = 0x0400,
								ZoomStep = 255;																			             																//FAST_MIDDLE = 0x0800,
							  LensDrvZoomSpeed(FAST);											  //FAST = 0x0400 	 //驱动镜头速度  （相对快）			          //MIDDLE = 0x0C00,
							}                                                               															 //SLOW = 0xFFFF,
							                                                      
							LensDrvZoomMove( 1, ZoomStep);																			// 1:反转													/*2020.10.11*/
							ZoomStepNum = ZoomStepNum + ZoomStep;
							ZoomStepCnt++;
							Delay100ms();
					break;
				case ZOOM_OUT:  //缩小
					
							if(20 >= ZoomStepCnt)
							{
								ZoomStep = 53;//100
							  LensDrvZoomSpeed(MIDDLE);								//驱动镜头速度控制MIDDLE = 0x0C00
							}
							else
							{
								ZoomStep = 255;
							  LensDrvZoomSpeed(FAST);									//FAST = 0x0400				
							}	
							
							LensDrvZoomMove( 0, ZoomStep);																			// 0：正转	
							ZoomStepCnt++;
							ZoomStepNum = ZoomStepNum - ZoomStep;						

							Delay100ms();
					break;
				case FOCUS_NEAR:  //聚焦近 聚焦+
	
					if(4 >= FcStepCnt)
					{
						FcStep = 4; //53
						LensDrvFocusSpeed(SLOW); //MIDDLE  FAST_MIDDLE
					}
					else
					{
						FcStep = 255;
						LensDrvFocusSpeed(FAST);											//FAST = 0x0400
					}
							
					//限位判断
					LensDrvFocusMove( 1, FcStep);					
					FcStepCnt++;
					FcStepNum	= FcStepNum - FcStep;
					Delay100ms();					
					break;
				case FOCUS_FAR:	//聚焦远 聚焦-

					if(4 >= FcStepCnt)
					{
						FcStep = 4;//53
					  LensDrvFocusSpeed(SLOW);	//MIDDLE  FAST_MIDDLE					
					}
					else
					{
						FcStep = 255;
					  LensDrvFocusSpeed(FAST);						
					}
					
					//限位判断
					LensDrvFocusMove( 0, FcStep);											
					FcStepCnt++;
					FcStepNum	= FcStepNum + FcStep;	
					Delay100ms();					
					break;
				case STOP:  //停止
					FcStepCnt = 0;
					ZoomStepCnt = 0;
					LensDrvFocusSpeed(SLOW);
					LensDrvZoomSpeed(SLOW);
					LensDrvFocusStop();
					LensDrvZoomStop();
					LenStatus = INVALID;
				
					break;
				case  SELF_TEST_ZOOM:			
					
					LensDrvZoomSpeed(FAST_MIDDLE);
					LensDrvZoomMove( 0, 255);
			
					if(SELF_TEST_ZOOM_FLAG)
							self_test_zoom++;
					
					if(self_test_zoom > 34500)
					{
						LensDrvFocusSpeed(SLOW);
						LensDrvZoomSpeed(SLOW);
						LensDrvFocusStop();
						LensDrvZoomStop();
						LenStatus = SELF_TEST_FOCUS;
			
						delay_1();
						delay_1();
					}
				 break;
				case  SELF_TEST_FOCUS:
					LensDrvFocusSpeed(FAST_MIDDLE);
					LensDrvFocusMove( 1, 255); //聚焦-
			
					if(SELF_TEST_FOCUS_FLAG)
							self_test_focus++;
			
					if(self_test_focus > 28000)
					{
						LenStatus = STOP;
						StopTime();
						//自检结束
						SELF_TEST_UnInit();
						ZoomStepNum = 0;
						FcStepNum = 0;
						
						delay_1();
					}					
				 break;	

				default:
					delay_1();
					break;
			}
#endif
			
	}
	
}






