#ifndef __UART_H_
#define __UART_H_


#define UART_BUF_LEN  	 15

extern unsigned char Receive_count;  //���ռ���
extern bit UART1_Overflow_Flag;  				//����������־
extern unsigned char UART1_Recv_Buf[UART_BUF_LEN];  //���ڽ��ջ�����



void UART1_Init(void);
void UartSendStr(unsigned char *str);
void UartSendByte(unsigned char dat);
void UartSend(unsigned char *str ,unsigned char len);
unsigned char ReceiveByte(void);

#endif

