#ifndef __UART_H_
#define __UART_H_


#define UART_BUF_LEN  	 15

extern unsigned char Receive_count;  //接收计数
extern bit UART1_Overflow_Flag;  				//缓冲区满标志
extern unsigned char UART1_Recv_Buf[UART_BUF_LEN];  //串口接收缓冲区



void UART1_Init(void);
void UartSendStr(unsigned char *str);
void UartSendByte(unsigned char dat);
void UartSend(unsigned char *str ,unsigned char len);
unsigned char ReceiveByte(void);

#endif

