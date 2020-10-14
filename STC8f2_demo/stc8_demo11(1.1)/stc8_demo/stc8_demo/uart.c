
#include <stdio.h>
#include "stc8.h"
#include <intrins.h>
#include "uart.h"


//串口
#define  uchar unsigned char

#define FOSC            24000000UL
#define BRT             (65536 - FOSC / 9600 / 4)//64911

unsigned char Receive_count = 0;  //接收计数
bit UART1_Overflow_Flag = 0;  				//缓冲区满标志												利于分辨俩状态及防冲突 接收数据 和 比较数据做相应动作 					
unsigned char UART1_Recv_Buf[UART_BUF_LEN] = {0};  //串口接收缓冲区  共16个数据成员

void UartSendStr(unsigned char *str);

/*
	中断收发数据
*/
void UART1_Isr() interrupt 4 using 1																			/*目测终端发送PELCO_D格式数据 , 及进usart1接收中断赋值UART1_Recv_Buf , 进主函数判断是啥动作数据*/
{

	if(RI)		//接收中断标志位判断    																			  硬件置一													
	{
		RI = 0; //清中断标志																									  软件清零
		if(!UART1_Overflow_Flag)																							//没数据就进来
		{
			if(Receive_count < UART_BUF_LEN)																		//(接收次数0   <   15)
			{
				UART1_Recv_Buf[Receive_count++] = SBUF;                           //将数据寄存器的值存放到 UART1_Recv_Buf
				/* '\r' == 0x0a    '\n' == 0x0d */  // 01234567
				if(SBUF == '\r' || SBUF == '\n' || Receive_count >= 7)						//如果等于‘\R’‘\N’及  '接收完第0~7位数据才可以'都可进入
				{
					UART1_Overflow_Flag = 1;																				//关闭串口中断,停止接收 并跳到主函数执行语句
				}
			}
			else
			{
				  UART1_Overflow_Flag = 1;   //关闭串口中断,停止接收
			}
		}
	}
}


/*
	串口发送一个字节
*/
void UartSendByte(unsigned char dat)
{
	SBUF = dat; //发送数据
	while(!TI); //等待发送完成
	TI = 0;			//清零发送标志位
}


/*
  串口发送字符串数据
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
	串口按字符串长度发送字符串数据
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
	串口接收一个字节数据
*/
unsigned char ReceiveByte(void)
{
	unsigned char rbyte;
	while(!RI); //查询接收标志位，是否有数据到达缓冲区
	RI = 0;					//清零接收标志位
	rbyte = SBUF;   //从缓冲区读取数据
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
	 串口1初始化
	 115200bps @24.000MHz
*/
void UART1_Init(void)
{	
	SCON = 0x50;
	/*串口1控制寄存器	    0x50: 0101 0000
		SM0=0; SM1=1;	 工作模式1（可变波特率8位数据方式）
		SM2=0; 				 无论收到（RB8）为0/1 串口数据寄存器（SBUF）都可接收到信息
		REN=1;				 允许串口接收数据
		TB8=0;RB8=0;	 模式0/1该俩位不用
		T1						 发送中断请求标志（模式0; 发送完成硬件将T1置一，向CPU发送中断请求,需软件清零）
		R1						 接收中断请求标志（模式0; 接收完成硬件将R1置一，向CPU发送中断请求,需软件清零）
	*/
	T2L = BRT;		//65536 - 24000000 / 115200 / 4 = 0FFCCH  (9600->FD8F)
	T2H = BRT >> 8;     
	/*bBT=(65536 - FOSC / 9600 / 4)==64911==1111 1101 1000 1111
		定时器2计数寄存器（16位重载,分高低位.T2L负责低8位,T2H负责高8位;）
	*/
	AUXR = 0x15;
	/*
		0001 0101
		TR2=1;	（定时器2运行控制位）	 				定时器2开始计数
	T2_C/T=0;															0：定时器；（对内部时钟进行计数）1：计数器;（）
		T2x12=1;（定时器2速度控制位）					T1模式（即CPU时钟不分频 FOSC/1）
		S1ST2=1;（串口1波特率发射器选择位）		选择定时器2作为波特率发射器
		T3/T4于T2一样,也可以当串口的波特率发生器和可编程时钟输出          */
	
	ES = 1;				//允许使能串口中断
	EA = 1;				//CPU开放中断
	
}



