#ifndef __SPI_H_
#define __SPI_H_

#include <intrins.h>
#include "stc8.h"


sbit  	CS					= 		P2^2;	//spi_2 cs


#define CS_Low()   	do{CS = 0;}while(0)
#define CS_Hihg()  	do{CS = 1;}while(0)



void Spi_Write(unsigned char addr, unsigned int data_);
unsigned char SPI_SendByte(unsigned char byte);
void SPI_Init(void);
void SPI_Speed(unsigned char speed);
void Delay(int b);

unsigned int Spi_Read(unsigned char addr);
#endif

