/****************************************************************************************************************
File :	Lensdrv.c
Function	 :	Lens Driver Ctrl For AN41908A
Made by	 :	2011/10/12	PSISZ DC
Description	 :
Env :	Keil51 C Compiler
Copyright:	MATSUSHITA ELECTRIC INDUSTRIAL CO.,LTD.
****************************************************************************************************************/
#include <intrins.h>
#include <math.h>
#include <stdarg.h>
#include "Lensdrv.h"
#include "spi.h"
#include "stc8.h"

#include "uart.h"

#define VD_FZ_Low()   	do{P34 = 0;}while(0)
#define VD_FZ_Hihg()  	do{P34 = 1;}while(0)

#define VD_IS_Low()   	do{P35 = 0;}while(0)
#define VD_IS_Hihg()  	do{P35 = 1;}while(0)



//#define LENS_SUNNY 1


#if 1
#define LENSDRV_FOCUS_STEPS_REG  0x24 //define AB motor to Focus
#define LENSDRV_ZOOM_STEPS_REG 	0x29   //define CD motor to Zoom

#else
#define LENSDRV_FOCUS_STEPS_REG   0x29 //define AB motor to Focus
#define LENSDRV_ZOOM_STEPS_REG 	0x24   //define CD motor to Zoom
#endif


void LensdrvDelay20us()
{
	unsigned char xdata i;

	i = 158;
	while (--i);

}

/****************************************************************************
* ����:void VD_FZ(void)
* ����:����VD�ź�
* ��ڲ���: ��
* ���ڲ���:��
* ˵��:
* ���÷���:
****************************************************************************/  
void DO_VD_FZ(void)
{

	VD_FZ_Hihg();
	LensdrvDelay20us();
	VD_FZ_Low();
}



/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvInit
DESCRIPTION	: Initial Lens Driver's registor
INPUT		: none
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
void LensDrvInit( void )
{

	Spi_Write(0x0b,0x0080);   //      ʹ�� TESTEN1 
	Spi_Write(0x0b,0x0080);   //      ʹ�� TESTEN1 
	
  Spi_Write(0x20,0x1e03);	 //�趨Ƶ��Ϊ	   DT1��ʱ��Ϊ 3ms
	Spi_Write(0x22,0x0001);	 //DT2��ʱ��Ϊ 0.6ms
	Spi_Write(0x23,0xd8d8);	 //����ABռ�ձ�Ϊ 90%
	Spi_Write(0x24,0x0d20);	 //AB 256ϸ��	�趨��������
	Spi_Write(0x25,0x07ff);  //����STEPΪ3.64ms
	Spi_Write(0x27,0x0002);	 //DT2��ʱ��Ϊ 0.6ms
	Spi_Write(0x28,0xd8d8);	 //����CDռ�ձ�Ϊ 90%
	Spi_Write(0x29,0x0520);	 //AB 256ϸ��	�趨��������
	Spi_Write(0x2a,0x07ff);  //����STEPΪ3.64ms
	
 	Spi_Write(0x21,0x0087);   //      ���ź�����PLS1��PLS2  	
//	Spi_Write(0x00,0x0000);
//	Spi_Write(0x01,0x0000);
//	Spi_Write(0x03,0x0200);
//	Spi_Write(0x08,0x000f);
//	Spi_Write(0x09,0x8001);
//	Spi_Write(0x0a,0x070c);	   //TGT_IN_TEST[9] = 0,��������ΪOUTE2 -> OUTE1    ��TGT_IN_TEST[9] = 1,��������ΪOUTE1 -> OUTE2
//	Spi_Write(0x0b,0x0400);
	
	Spi_Write(0x2C,0x0005);   //��Eͨ��
	
	LensDrvFocusZoomInit();
	LensDrvIrisInit();
}


/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvSPIWriteUint
DESCRIPTION	: write Lens driver's registor  ��д��ͷ��������ļĴ���
INPUT		: addr,wt_data
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
// 																		  0x2A               0X0C00 
void LensDrvSPIWriteUint( unsigned char addr, unsigned int wt_data )
{//											LensDrvSPIWriteUint(0x24,0x0000);

	CS_Hihg();
	SPI_SendByte(addr);
	SPI_SendByte (wt_data%256);
	SPI_SendByte(wt_data/256);
	CS_Low();

}

/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvSPIReadUint
DESCRIPTION	: Read the value of Lens driver's registor   ��ȡ��ͷ��������Ĵ�����ֵ
INPUT		: addr
OUTPUT		: value of Lens driver's registor(addr)
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
//                                             0x29
unsigned int LensDrvSPIReadUint( unsigned char addr ) 																							//ͨ��spi����1�ֽ����ݸ���ͷ������оƬ, ������1�ֽ�����
{

	unsigned int temp = 0;
	CS_Hihg();
	SPI_SendByte(0x40|addr);                                                         
	temp|=SPI_SendByte(0xFF)<<8;  
	temp|=SPI_SendByte(0xFF);	 
	CS_Low();
	return temp;
}


/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvFocusZoomInit
DESCRIPTION	: Initial Focus&Zoom stepping motor potion
INPUT		: none
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
void LensDrvFocusZoomInit( void )
{

#if LENS_SUNNY //Zoom use AB chanel
	LensDrvSPIWriteUint(0x20,0x5c0a);  /*DT1=3.03ms PWM=30.1kHz*/
	LensDrvSPIWriteUint(0x21,0x0000);
	LensDrvSPIWriteUint(0x22,0x0003);  /* DT2A=0.91ms  AB phase 90 degree */
	LensDrvSPIWriteUint(0x23,0xC8C8);  /* PWMA PWMB max duty 0.89 */
	LensDrvSPIWriteUint(0x24,0x0400);	//	
	LensDrvSPIWriteUint(0x25,0x0502);//
#else	
	LensDrvSPIWriteUint(0x20,0x5c0a);
	LensDrvSPIWriteUint(0x21,0x0000);
	
	LensDrvSPIWriteUint(0x22,0x1603);
	LensDrvSPIWriteUint(0x23,0xC8C8);
	LensDrvSPIWriteUint(0x24,0x0400); //�۽����
	LensDrvSPIWriteUint(0x25,0xFFFF);//0x0160   //��СΪ0xFFFF
	
 #endif

	LensDrvSPIWriteUint(0x27,0x1603); 
	LensDrvSPIWriteUint(0x28,0xC8C8);
	LensDrvSPIWriteUint(0x29,0x0400); //�䱶���
	LensDrvSPIWriteUint(0x2A,0x0FFF); //0x0400   //��СΪ0xFFFF
	
}

/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvIrisInit
DESCRIPTION	: Initial Iris potion
INPUT		: none
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
void LensDrvIrisInit( void )
{
#if LENS_SUNNY
	LensDrvSPIWriteUint(0x00,0x0000);  //Set Iris Target
	LensDrvSPIWriteUint(0x01,0x808A);  
	LensDrvSPIWriteUint(0x02,0x66F0);
	LensDrvSPIWriteUint(0x03,0x0E10);
	LensDrvSPIWriteUint(0x04,0x7E20);
	LensDrvSPIWriteUint(0x05,0x0A04);
	LensDrvSPIWriteUint(0x0A,0x0000);
	LensDrvSPIWriteUint(0x0B,0x0400);
	LensDrvSPIWriteUint(0x0E,0x0C00);

#else
	LensDrvSPIWriteUint(0x00,0x0000);  //Set Iris Target
	LensDrvSPIWriteUint(0x01,0x6600);  
	LensDrvSPIWriteUint(0x02,0x5400);
	LensDrvSPIWriteUint(0x03,0x0E10);
	LensDrvSPIWriteUint(0x04,0x8437);
	LensDrvSPIWriteUint(0x05,0x0104);
	LensDrvSPIWriteUint(0x0A,0x0042);
	LensDrvSPIWriteUint(0x0B,0x0400);
	LensDrvSPIWriteUint(0x0E,0x0D00);
#endif
}

/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvIrisMove
DESCRIPTION	: Iris move control
INPUT		: IrsTGT
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
//after implement this function,there should be VD_IS pluse. 
void LensDrvIrisMove(unsigned int  IrsTGT)
{
	//IrsTGT range:0x0000~0x03FF
	if ( IrsTGT> 0x03ff )
	{
		return ;
	}
	
	LensDrvSPIWriteUint(0x00,IrsTGT);  //Set Iris Target
#if 1	//output a VD_IS pluse
	VD_IS_Hihg();
	LensdrvDelay20us();
	VD_IS_Low();
#endif 
}



//Ԥ������
void LensDrvLostStepCorrect(bit dir, int motor)
{
	xdata unsigned int backup_data = 0;

	//Focus
	if(motor == 1)
	{	
		backup_data = LensDrvSPIReadUint( LENSDRV_FOCUS_STEPS_REG );
		backup_data &=  0x0C00; //0xFE00;

		if ( dir == 1)
		{  
				//Add directiion bit (CCWCWAB) to registor:0x24
			backup_data |=0x0100;
			LensDrvSPIWriteUint( LENSDRV_FOCUS_STEPS_REG,backup_data);
		} 
		else
		{
			LensDrvSPIWriteUint( LENSDRV_FOCUS_STEPS_REG,backup_data);
		}
	}
	else  //zoom
	{
		backup_data = LensDrvSPIReadUint( LENSDRV_ZOOM_STEPS_REG );
		backup_data &=0x0400;//0xFE00
		if ( dir == 1) //��ת
		{  
				//Add directiion bit (CCWCWAB) to registor:0x29
			backup_data |=0x0100;
			LensDrvSPIWriteUint( LENSDRV_ZOOM_STEPS_REG,backup_data);
		} 
		else
		{
			LensDrvSPIWriteUint( LENSDRV_ZOOM_STEPS_REG,backup_data);
		}		
	}

	VD_FZ_Hihg();
	LensdrvDelay20us();
	VD_FZ_Low();
}


/***********************************************************************
#ifdef	DOC
FUNCTION	: LensDrvFocusMove
DESCRIPTION	: Focus motor move control
INPUT		: dir,FcStep
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
//after implement this function,there should be VD_FZ pluse. 
void LensDrvFocusMove(bit dir, unsigned int FcStep)
{
	xdata unsigned int backup_data = 0;
	if ( (FcStep > 0x00FF) || (FcStep < 0x00) ) // more than 255step
	{	
		return;
	}
	
	//LensDrvSPIWriteUint(0x29,0x0000); //�䱶��� 0x0000

	backup_data = LensDrvSPIReadUint( LENSDRV_FOCUS_STEPS_REG );
	
	backup_data &=  0x0C00; //0xFE00;
	backup_data |= FcStep;
	if ( dir == 1)
	{  
	    //Add directiion bit (CCWCWAB) to registor:0x24
		backup_data |=0x0100;
		LensDrvSPIWriteUint( LENSDRV_FOCUS_STEPS_REG,backup_data);
	} 
	else
	{
		LensDrvSPIWriteUint( LENSDRV_FOCUS_STEPS_REG,backup_data);
	}
#if 1	//output a VD_FZ pluse
	VD_FZ_Hihg();
	LensdrvDelay20us();
	VD_FZ_Low();
#endif 
}

/***********************************************************************
#ifdef	DOC
FUNCTION	: LenDrvZoomMove
DESCRIPTION	: Zoom motor move control
INPUT		: dir,ZoomStep
OUTPUT		: none
UPDATE		:
DATE		: 2011/02/21
#endif
***********************************************************************/
//after implement this function,there should be VD_FZ pluse. 
void LensDrvZoomMove( bit dir, unsigned int  ZoomStep)   
{//                        1                   53
	xdata unsigned int backup_data = 0;
	if (( ZoomStep > 0x00FF) || ( ZoomStep < 0x00)) // more than 255step
	{	
		return;
	}

	LensDrvSPIWriteUint(0x24,0x0000); //�۽���� 0x0000

	backup_data = LensDrvSPIReadUint( LENSDRV_ZOOM_STEPS_REG );																					//( LENSDRV_ZOOM_STEPS_REG== 0x29) (����CD�������) 69
	backup_data &=0x0400;//0xFE00																																				&ȫ��1��Ϊ1����֮Ϊ��0��
	//backup_data |=0x3000;//�ı�ϸ������ϸ�ָ�Ϊ64ϸ�� 11 -- 64, 10 -- 128, 00 -- 256
	backup_data |=ZoomStep;
	if ( dir == 1) //��ת
	{  
	    //Add directiion bit (CCWCWAB) to registor:0x29
		backup_data |=0x0100;
		LensDrvSPIWriteUint( LENSDRV_ZOOM_STEPS_REG,backup_data);
	}
	else
	{
		LensDrvSPIWriteUint( LENSDRV_ZOOM_STEPS_REG,backup_data);
	}
#if 1	//output a VD_FZ pluse
	VD_FZ_Hihg();
	LensdrvDelay20us();
	VD_FZ_Low();
#endif 
}


void LensDrvFocusStop(void)
{
	LensDrvSPIWriteUint(0x24,0x0400);
}


void LensDrvZoomStop(void)
{
	LensDrvSPIWriteUint(0x29,0x0400);
}


/****************************************************************************
* ��    �ƣ�void LensDrvIrisOpen(void)
* ��    �ܣ��򿪹�Ȧ
* ��ڲ���: ��
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/ 
void LensDrvIrisOpen(void) 
{
	unsigned int IrsTGT = 0;
	
	for(IrsTGT = 0x03FF; IrsTGT > 0; IrsTGT--){
		LensDrvIrisMove(IrsTGT);
	}
	
	LensDrvIrisMove(0x0000);
}

/****************************************************************************
* ��    �ƣ�void LensDrvFocusSpeed(void)
* ��    �ܣ��������ٶȵ���
* ��ڲ���: ��
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/ 
void LensDrvFocusSpeed(unsigned int FcSpeed)
{
	//0xFFFF ����
	//Ĭ�� 0x0160
	LensDrvSPIWriteUint(0x25,FcSpeed);
}


/****************************************************************************
* ��    �ƣ�void LensDrvFocusSpeed(void)
* ��    �ܣ��䱶����ٶȵ���
* ��ڲ���: ��
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/ 
void LensDrvZoomSpeed(unsigned int ZoomSpeed)
{
	//0xFFFF ����
	//Ĭ�� 0x0160
	LensDrvSPIWriteUint(0x2A,ZoomSpeed);
}




void MS_Rest(void)
{
  VD_IS_Hihg();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	VD_IS_Low();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	VD_IS_Hihg();
}


