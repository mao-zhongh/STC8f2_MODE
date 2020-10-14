
#include <stdio.h>
#include "stc8.h"
#include <intrins.h>
#include "uart.h"


//����
#define  uchar unsigned char

#define FOSC            24000000UL
#define BRT             (65536 - FOSC / 9600 / 4)//64911

unsigned char Receive_count = 0;  //���ռ���
bit UART1_Overflow_Flag = 0;  				//����������־												���ڷֱ���״̬������ͻ �������� �� �Ƚ���������Ӧ���� 					
unsigned char UART1_Recv_Buf[UART_BUF_LEN] = {0};  //���ڽ��ջ�����  ��16�����ݳ�Ա

void UartSendStr(unsigned char *str);

/*
	�ж��շ�����
*/
void UART1_Isr() interrupt 4 using 1																			/*Ŀ���ն˷���PELCO_D��ʽ���� , ����usart1�����жϸ�ֵUART1_Recv_Buf , ���������ж���ɶ��������*/
{

	if(RI)		//�����жϱ�־λ�ж�    																			  Ӳ����һ													
	{
		RI = 0; //���жϱ�־																									  �������
		if(!UART1_Overflow_Flag)																							//û���ݾͽ���
		{
			if(Receive_count < UART_BUF_LEN)																		//(���մ���0   <   15)
			{
				UART1_Recv_Buf[Receive_count++] = SBUF;                           //�����ݼĴ�����ֵ��ŵ� UART1_Recv_Buf
				/* '\r' == 0x0a    '\n' == 0x0d */  // 01234567
				if(SBUF == '\r' || SBUF == '\n' || Receive_count >= 7)						//������ڡ�\R����\N����  '�������0~7λ���ݲſ���'���ɽ���
				{
					UART1_Overflow_Flag = 1;																				//�رմ����ж�,ֹͣ���� ������������ִ�����
				}
			}
			else
			{
				  UART1_Overflow_Flag = 1;   //�رմ����ж�,ֹͣ����
			}
		}
	}
}


/*
	���ڷ���һ���ֽ�
*/
void UartSendByte(unsigned char dat)
{
	SBUF = dat; //��������
	while(!TI); //�ȴ��������
	TI = 0;			//���㷢�ͱ�־λ
}


/*
  ���ڷ����ַ�������
*/
void UartSendStr(unsigned char *str)
{
	while(*str != '\0')
	{
		UartSendByte(*str);
		str++;
	}
}


/*
	���ڰ��ַ������ȷ����ַ�������
*/
void UartSend(unsigned char *str ,unsigned char len)
{
	while(len--)
	{
		UartSendByte(*str);
		str++;
	}
}


/*
	���ڽ���һ���ֽ�����
*/
unsigned char ReceiveByte(void)
{
	unsigned char rbyte;
	while(!RI); //��ѯ���ձ�־λ���Ƿ������ݵ��ﻺ����
	RI = 0;					//������ձ�־λ
	rbyte = SBUF;   //�ӻ�������ȡ����
	return rbyte;
}



void ReceiveByteStr(unsigned char *str)
{
	while(*str != '\0')
	{
		*str = ReceiveByte();
		str++;
	}
}




/*
	 ����1��ʼ��
	 115200bps @24.000MHz
*/
void UART1_Init(void)
{	
	SCON = 0x50;
	/*����1���ƼĴ���	    0x50: 0101 0000
		SM0=0; SM1=1;	 ����ģʽ1���ɱ䲨����8λ���ݷ�ʽ��
		SM2=0; 				 �����յ���RB8��Ϊ0/1 �������ݼĴ�����SBUF�����ɽ��յ���Ϣ
		REN=1;				 �����ڽ�������
		TB8=0;RB8=0;	 ģʽ0/1����λ����
		T1						 �����ж������־��ģʽ0; �������Ӳ����T1��һ����CPU�����ж�����,��������㣩
		R1						 �����ж������־��ģʽ0; �������Ӳ����R1��һ����CPU�����ж�����,��������㣩
	*/
	T2L = BRT;		//65536 - 24000000 / 115200 / 4 = 0FFCCH  (9600->FD8F)
	T2H = BRT >> 8;     
	/*bBT=(65536 - FOSC / 9600 / 4)==64911==1111 1101 1000 1111
		��ʱ��2�����Ĵ�����16λ����,�ָߵ�λ.T2L�����8λ,T2H�����8λ;��
	*/
	AUXR = 0x15;
	/*
		0001 0101
		TR2=1;	����ʱ��2���п���λ��	 				��ʱ��2��ʼ����
	T2_C/T=0;															0����ʱ���������ڲ�ʱ�ӽ��м�����1��������;����
		T2x12=1;����ʱ��2�ٶȿ���λ��					T1ģʽ����CPUʱ�Ӳ���Ƶ FOSC/1��
		S1ST2=1;������1�����ʷ�����ѡ��λ��		ѡ��ʱ��2��Ϊ�����ʷ�����
		T3/T4��T2һ��,Ҳ���Ե����ڵĲ����ʷ������Ϳɱ��ʱ�����          */
	
	ES = 1;				//����ʹ�ܴ����ж�
	EA = 1;				//CPU�����ж�
	
}



