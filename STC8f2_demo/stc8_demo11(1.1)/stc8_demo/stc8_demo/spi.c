

#include <intrins.h>
#include "spi.h"


unsigned char SPI_RecData=0;
unsigned char SPI_Buffer=0;

void Delay(int b)
{ 
	int a;
	for(a=0;a<b;a++)
	 for(a=0;a<b;a++);	
}


void Delay20us()		//@24.000MHz
{
	unsigned char i;

	i = 158;
	while (--i);
}


void SPI_Isr() interrupt 9 using 1
{
    SPSTAT = 0xc0;                              //?????                                //????
		SPI_Buffer = SPDAT;
}


void SPI_Init(void)
{
		P_SW1 = 0x04;
		SPCTL = 0xfc;//0x50;          
		/*
		使能 SPI 功能
		先发送/接收数据的低位（LSB）
		
		*/
    SPSTAT = 0xc0;    //清中断 、 写冲突 标志                         
    SPDAT = 0; 				//数据寄存器清0
	  CS_Low();
}


/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte 
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
unsigned char SPI_SendByte(unsigned char byte)
{
	SPDAT = byte; 																 //将数据装入SPI数据寄存器
	while((SPSTAT&0x80)==0); //等待发送完毕
	SPSTAT = 0xc0;  															 //清除中断标志，清除写冲突标志,注意是对应位写1才能清零 
	SPI_RecData = SPDAT;

	return SPI_RecData;
}




void SPI_RecvByte(unsigned char *byte)
{
	while((SPSTAT&0x80)==0); //等待发送完毕
	SPSTAT = 0xc0;   /* 清除中断标志 和 写冲突标志,注意是对应位写1才能清零 */
	*byte = SPDAT;
}


/*发送数据*/
void Spi_Write(unsigned char addr, unsigned int data_)
{	
	CS_Hihg();
	SPI_SendByte(addr);
	SPI_SendByte (data_%256);
	SPI_SendByte(data_/256);
	CS_Low();
}



/*接收数据*/
unsigned int Spi_Read(unsigned char addr)
{
	unsigned char ret = 0, ret1 = 0, ret2 = 0;
	CS_Hihg();  //片选
	SPI_SendByte(addr+0x40);
	SPI_RecvByte(&ret1);
	SPI_RecvByte(&ret2);
	CS_Low();
	return (ret2*0x100+ret1);
}


void SPI_Speed(unsigned char speed)			//SPI时钟频率选择
{
	switch(speed)
	{
		case 0: SPCTL &= 0xFC;break;  								  //SYSclk/4
		case 1: SPCTL &= 0xFC; SPCTL |= 0x01; break;		//SYSclk/8
		case 2: SPCTL &= 0xFC; SPCTL |= 0x02; break;		//SYSclk/16
		case 3: SPCTL &= 0xFC; SPCTL |= 0x03; break;		//SYSclk/32
	}
}

