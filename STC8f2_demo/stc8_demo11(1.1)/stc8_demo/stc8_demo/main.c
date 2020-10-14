
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
	ZOOM_IN = 0,																																	/* �䱶�� (�Ŵ�+)   0*/
	ZOOM_OUT,																																			/* �䱶�� ��С- 	  1*/
	FOCUS_NEAR,																																		/* �۽��� �۽�+	  	2*/
	FOCUS_FAR,																																		/* �۽�Զ �۽�- 		3*/
	STOP,																																					/* ֹͣ	 						4*/
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
PELCO_D Э��˵����
���ݸ�ʽ��1λ��ʼλ��8λ���ݡ�1λֹͣλ����У��λ;
�����ʽ��|  �ֽ�1   |  �ֽ�2 |  �ֽ�3   |   �ֽ�4  |  �ֽ�5   |   �ֽ�6  |  �ֽ�7 |
					  ͬ���ֽ�   ��ַ��    ָ����1    ָ����2    ������1    ������2		У����

1.��Э����������ֵ��Ϊʮ��������
2.ͬ���ֽ�ʼ��ΪFFH
3.��ַ��Ϊ��������߼���ַ�ţ���ַ��Χ��00H~FFH
4.ָ�����ʾ��ͬ�Ķ���
5.������1��2�ֱ��ʾˮƽ����ֱ�����ٶ�(00~3FH),FFH��ʾ��turbo�� �ٶ�
6.У���� = MOD[(�ֽ�2+�ֽ�3+�ֽ�4+�ֽ�5+�ֽ�6)/100H]
			���µ�ַ��Ϊ0x01
****************************************************************************/

xdata unsigned char PELCO_D[][7] = {
	/* �䱶�� �Ŵ�+ */
	{0xff,0x01,0x00,0x20,0x00,0x00,0x21},
	/* �䱶�� ��С- */
	{0xff,0x01,0x00,0x40,0x00,0x00,0x41},
	/* �۽��� �۽�+ */
	{0xff,0x01,0x00,0x80,0x00,0x00,0x81},
	/* �۽�Զ �۽�- */
	{0xff,0x01,0x01,0x00,0x00,0x00,0x02},
	/* ֹͣ */
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
* ��		��: void SELF_TEST_UnInit(void)
* ��		��: ��eeprom��ַд����ֵ�������״α䱶��� �������Լ�
* ��ڲ���: ��
* ���ڲ���: ��
* ˵		��:
* ���÷���:
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
* ��		��: void SAVE_DATA_TO_EEPROM(unsigned char zoomH, unsigned char zoomL, unsigned char focusH, unsigned char focusL)

* ��		��: �����ݱ�����eeprom��
* ��ڲ���: ��
* ���ڲ���: ��
* ˵		��:
* ���÷���:
****************************************************************************/
void SAVE_DATA_TO_EEPROM(unsigned char zoomH, unsigned char zoomL, unsigned char focusH, unsigned char focusL)
{
	IapErase(0x0400);
	IapErase(0x0401);
	IapErase(0x0402);
	IapErase(0x0403);
	IapProgram(0x0400, zoomH); //zoom ��λ����
	IapProgram(0x0401, zoomL); //zoom ��λ����
	IapProgram(0x0402, focusH);//focus ��λ����
	IapProgram(0x0403, focusL);//focus ��λ����
}




/****************************************************************************
* ��		��: unsigned short CHAR_TO_SHORT(unsigned char data1, unsigned char data2)
* ��		��: ��char��������ת��Ϊshort��������
* ��ڲ���: unsigned char , unsigned char
* ���ڲ���: unsigned short
* ˵		��: ����0 Ϊ���δ���
* ���÷���:
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
* ��		��: int SHORT_TO_CHAR(unsigned char *data1, unsigned char *data2, unsigned short)
* ��		��: ��short��������ת��Ϊchar��������
* ��ڲ���: unsigned short
* ���ڲ���: unsigned char* , unsigned char*
* ˵		��: ����1 ת���ɹ�������0 ���δ���
* ���÷���:
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
* ��		��: void SYS_Init(void)
* ��		��: ϵͳ��ʼ�� ʹ���ڲ�24MHzʱ��
* ��ڲ���: ��
* ���ڲ���: ��
* ˵		��:
* ���÷���:
****************************************************************************/
void SYS_Init(void)
{
	CKSEL = 0x48;	//0100 1000
	/*
	ѡ���ڲ�24Mhz�߾���IRC		(MCKSEL[1:0])		 ;
	��ʱ�ӷ�Ƶ����� P1.6 �� ��MCLKO_S��			 ;
	��ʱ�ӷ�Ƶ���Ƶ��MCLK/4  (MCLKODIV[3:0])  ;
	*/
}



void Delay100ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();//1us��ʱ
	_nop_();//1us��ʱ
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



/*ʵ��printf��ӡ*/
char putchar (char c)
{
	ES = 0;         //��ֹ����1�ж�
	SBUF = c;				
	while(TI==0);		//�ȴ��������   ��־��1ʱ�������
	TI = 0;					//���жϱ�־
//	ES = 1;				//������1�ж�
	return 0;
		
}
	






/****************************************************************************
* ��		��: void main(void)
* ��		��: ���������
* ��ڲ���: ��
* ���ڲ���: ��
* ˵		��:
* ���÷���:
****************************************************************************/
void main(void)
{
													/* xdata�����ı�����洢���ⲿ��չRAM��Flash��,һ��ָ�ⲿ0x0000-0xffff�ռ�,��DPTR���� */
  xdata unsigned short FcStep = 0;
	xdata unsigned short ZoomStep = 0;    //ZoomStep
	xdata unsigned char FcStepCnt = 0;
	xdata unsigned char ZoomStepCnt = 0;  //ZoomStepCnt
	
	//�ܲ���
	xdata unsigned int ZoomStepNum = 0; 
	xdata unsigned int FcStepNum = 0; 

	xdata char LenStatus = STOP;																	//STOP == PELCO_D��ָֹͣ��
	
	//�Լ�
	unsigned int self_test_zoom = 0; 
	unsigned int self_test_focus = 0;

	unsigned char ZoomStepTmp[4] = {0};
	unsigned char FcStepTmp[4] = {0};
	unsigned char num_s=65;
	char tmp_flag = 0;																						//û�õ����
	
	SYS_Init();		//ϵͳ��ʼ�� ʹ���ڲ�24Mhz�߾���IRCʱ��
	Timer4Init();
	UART1_Init();
  SPI_Init(); 
	SPI_Speed(1);


	MS_Rest(); //41918
	LensDrvInit(); //��ͷ��ʼ��
	LensDrvIrisOpen();//����ͷ
	
	
	while (1)
	{		
	//	printf("1");
	//		printf("%bd",num_s);
		UartSendByte('b');
		
			if(UART1_Overflow_Flag)						//����������־		�� UART1_Recv_Buf[] �Ѵ���˿��ԱȽϵ����ݣ�
			{
					/* �䱶�� (�Ŵ�+) */
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
					/* �䱶�� (��С-) */
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
					/* �۽��� (�۽�+) */
					else if((UART1_Recv_Buf[0] == PELCO_D[FOCUS_NEAR][0])&&(UART1_Recv_Buf[1] == PELCO_D[FOCUS_NEAR][1])&&(UART1_Recv_Buf[2] == PELCO_D[FOCUS_NEAR][2])&&(UART1_Recv_Buf[3] == PELCO_D[FOCUS_NEAR][3])
						 &&(UART1_Recv_Buf[4] == PELCO_D[FOCUS_NEAR][4])&&(UART1_Recv_Buf[5] == PELCO_D[FOCUS_NEAR][5])&&(UART1_Recv_Buf[6] == PELCO_D[FOCUS_NEAR][6]))					
					{	
						 	LenStatus	= FOCUS_NEAR;						
					}
					/* �۽�Զ (�۽�-) */
					else if((UART1_Recv_Buf[0] == PELCO_D[FOCUS_FAR][0])&&(UART1_Recv_Buf[1] == PELCO_D[FOCUS_FAR][1])&&(UART1_Recv_Buf[2] == PELCO_D[FOCUS_FAR][2])&&(UART1_Recv_Buf[3] == PELCO_D[FOCUS_FAR][3])
						 &&(UART1_Recv_Buf[4] == PELCO_D[FOCUS_FAR][4])&&(UART1_Recv_Buf[5] == PELCO_D[FOCUS_FAR][5])&&(UART1_Recv_Buf[6] == PELCO_D[FOCUS_FAR][6]))					
					{
							LenStatus	= FOCUS_FAR;
					}
					/* ֹͣ */
					else if((UART1_Recv_Buf[0] == PELCO_D[STOP][0])&&(UART1_Recv_Buf[1] == PELCO_D[STOP][1])&&(UART1_Recv_Buf[2] == PELCO_D[STOP][2])&&(UART1_Recv_Buf[3] == PELCO_D[STOP][3])
						 &&(UART1_Recv_Buf[4] == PELCO_D[STOP][4])&&(UART1_Recv_Buf[5] == PELCO_D[STOP][5])&&(UART1_Recv_Buf[6] == PELCO_D[STOP][6]))					
					{
							LenStatus	= STOP;
              tmp_flag = 0;						
					}
				Receive_count = 0;   				//�������� 																		(���ռ�����0						 	)
				UART1_Overflow_Flag = 0;		//�����ڼ����������� 												 ���ɽ�UART1�жϽ������ݽ��գ�
			}		
			
#if 1
			switch(LenStatus)																														//�����Ƚ϶�LenStatus��ֵ��ʼ����Ӧ����							
			{
				case ZOOM_IN: //�Ŵ�
				/*  ÿ�ν���ZoomStepCnt++;������ͷ�ٶ������;��ת;�����ĵ�21�� Delay100ms; 		*/									
							if(20 >= ZoomStepCnt)
							{
								ZoomStep = 53;//100
							  LensDrvZoomSpeed(MIDDLE);											//MIDDLE = 0x0C00		//������ͷ�ٶȣ��������					
							}

							else
							{																																																 //FAST = 0x0400,
								ZoomStep = 255;																			             																//FAST_MIDDLE = 0x0800,
							  LensDrvZoomSpeed(FAST);											  //FAST = 0x0400 	 //������ͷ�ٶ�  ����Կ죩			          //MIDDLE = 0x0C00,
							}                                                               															 //SLOW = 0xFFFF,
							                                                      
							LensDrvZoomMove( 1, ZoomStep);																			// 1:��ת													/*2020.10.11*/
							ZoomStepNum = ZoomStepNum + ZoomStep;
							ZoomStepCnt++;
							Delay100ms();
					break;
				case ZOOM_OUT:  //��С
					
							if(20 >= ZoomStepCnt)
							{
								ZoomStep = 53;//100
							  LensDrvZoomSpeed(MIDDLE);								//������ͷ�ٶȿ���MIDDLE = 0x0C00
							}
							else
							{
								ZoomStep = 255;
							  LensDrvZoomSpeed(FAST);									//FAST = 0x0400				
							}	
							
							LensDrvZoomMove( 0, ZoomStep);																			// 0����ת	
							ZoomStepCnt++;
							ZoomStepNum = ZoomStepNum - ZoomStep;						

							Delay100ms();
					break;
				case FOCUS_NEAR:  //�۽��� �۽�+
	
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
							
					//��λ�ж�
					LensDrvFocusMove( 1, FcStep);					
					FcStepCnt++;
					FcStepNum	= FcStepNum - FcStep;
					Delay100ms();					
					break;
				case FOCUS_FAR:	//�۽�Զ �۽�-

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
					
					//��λ�ж�
					LensDrvFocusMove( 0, FcStep);											
					FcStepCnt++;
					FcStepNum	= FcStepNum + FcStep;	
					Delay100ms();					
					break;
				case STOP:  //ֹͣ
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
					LensDrvFocusMove( 1, 255); //�۽�-
			
					if(SELF_TEST_FOCUS_FLAG)
							self_test_focus++;
			
					if(self_test_focus > 28000)
					{
						LenStatus = STOP;
						StopTime();
						//�Լ����
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






