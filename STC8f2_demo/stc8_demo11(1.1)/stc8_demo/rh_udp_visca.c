#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <utime.h>
#include <sys/stat.h>
#include <sys/types.h> 
#include <pthread.h>
#include <ctype.h>
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <netdb.h>  
#include <net/if.h>
#include <errno.h>
#include <termios.h> 
#include <dirent.h> 

#include "rh_protocol.h"
#include "osa_msgq.h"
#include "osa_buf.h"
#include "rh_protocol.h"


static pthread_t gs_ProtoPid;
static HI_BOOL gs_ThreadStart = HI_TRUE;
OSA_MsgqHndl QElectronicPTZ[2];



static int PtzSpeed = 0x3f;

/******************��е��̨����*************/

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

int fd_rs485 = 0;
int UART_Open(int fd, char* port);
void UART_Close(int fd);
int UART_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
int UART_Init(int fd, int speed, int flow_ctrlint, int databits, int stopbits, char parity);
int UART_Recv(int fd, char *rcv_buf, int data_len);
int UART_Send(int fd, char *send_buf, int data_len);

/*****************************************************************
* ���ƣ� UART0_Open
* ���ܣ� �򿪴��ڲ����ش����豸�ļ�����
* ��ڲ����� fd :�ļ������� port :���ں�(ttyS0,ttyS1,ttyS2)
* ���ڲ����� ��ȷ����Ϊ1�����󷵻�Ϊ0
*****************************************************************/
int UART_Open(int fd, char* port)
{

	//fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	//fd = open(port, O_WRONLY | O_NOCTTY);
	fd = open(port, O_WRONLY);
	if (FALSE == fd){
		perror("Can't Open Serial Port");
		return(FALSE);
	}

	////�жϴ��ڵ�״̬�Ƿ�Ϊ����״̬ 
	//if (fcntl(fd, F_SETFL, 0) < 0){
	//	printf("fcntl failed!\n");
	//	return(FALSE);
	//}
	//else {
	//	//    printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
	//}

	////�����Ƿ�Ϊ�ն��豸
	//if (0 == isatty(STDIN_FILENO)){
	//	printf("standard input is not a terminal device\n");
	//	return(FALSE);
	//}

	return fd;
}

void UART_Close(int fd)
{
	close(fd);
}

/*******************************************************************
* ���ƣ� UART0_Set
* ���ܣ� ���ô�������λ��ֹͣλ��Ч��λ
* ��ڲ����� fd �����ļ�������
* speed �����ٶ�
* flow_ctrl ����������
* databits ����λ ȡֵΪ 7 ����8
* stopbits ֹͣλ ȡֵΪ 1 ����2
* parity Ч������ ȡֵΪN,E,O,,S
*���ڲ����� ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int UART_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity)
{

	int i;
	//    int status; 
	int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
		B38400, B19200, B9600, B4800, B2400, B1200, B300
	};
	int name_arr[] = {
		38400, 19200, 9600, 4800, 2400, 1200, 300, 38400,
		19200, 9600, 4800, 2400, 1200, 300
	};
	struct termios options;

	bzero(&options, sizeof(options));
	/*tcgetattr(fd,&options)�õ���fdָ��������ز������������Ǳ�����options,�ú���,�����Բ��������Ƿ���ȷ���ô����Ƿ���õȡ������óɹ�����������ֵΪ0��������ʧ�ܣ���������ֵΪ1.
	*/
	if (tcgetattr(fd, &options) != 0){
		perror("SetupSerial 1");
		return(FALSE);
	}

	//���ô������벨���ʺ����������
	for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
		if (speed == name_arr[i]) {
			cfsetispeed(&options, speed_arr[i]);
			cfsetospeed(&options, speed_arr[i]);
		}
	}
	//�޸Ŀ���ģʽ����֤���򲻻�ռ�ô���        
	options.c_cflag |= CLOCAL;
	//�޸Ŀ���ģʽ��ʹ���ܹ��Ӵ����ж�ȡ��������
	options.c_cflag |= CREAD;

	////�޸����ģʽ��ԭʼ�������  
	//options.c_cflag &= ~(OPOST);
	////����������־λ  
	//options.c_cflag |= (CLOCAL | CREAD);
	//options.c_cflag &= ~CSIZE;


	//��������������
	switch (flow_ctrl){
	case 0: //��ʹ��������
		options.c_cflag &= ~CRTSCTS;
		break;
	case 1: //ʹ��Ӳ��������
		options.c_cflag |= CRTSCTS;
		break;
	case 2: //ʹ�����������
		options.c_cflag |= IXON | IXOFF | IXANY;
		break;
	}
	//��������λ
	options.c_cflag &= ~CSIZE; //����������־λ
	switch (databits){
	case 5:
		options.c_cflag |= CS5;
		break;
	case 6:
		options.c_cflag |= CS6;
		break;
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr, "Unsupported data size\n");
		return (FALSE);
	}
	//����У��λ
	switch (parity) {
	case 'n':
	case 'N': //����żУ��λ��
		options.c_cflag &= ~PARENB;
		options.c_iflag &= ~INPCK;
		break;
	case 'o':
	case 'O': //����Ϊ��У�� 
		options.c_cflag |= (PARODD | PARENB);
		options.c_iflag |= INPCK;
		break;
	case 'e':
	case 'E': //����ΪżУ�� 
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_iflag |= INPCK;
		break;
	case 's':
	case 'S': //����Ϊ�ո� 
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported parity\n");
		return (FALSE);
	}
	// ����ֹͣλ 
	switch (stopbits){
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported stop bits\n");
		return (FALSE);
	}

	//����ģʽ
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	//�޸����ģʽ��ԭʼ�������
	options.c_oflag &= ~OPOST;
	//���õȴ�ʱ�����С�����ַ�
	options.c_cc[VTIME] = 0; /* ��ȡһ���ַ��ȴ�1*(1/10)s */
	options.c_cc[VMIN] = 0; /* ��ȡ�ַ������ٸ���Ϊ1 */

	//�����������������������ݣ����ǲ��ٶ�ȡ
	tcflush(fd, TCIFLUSH);

	//�������� (���޸ĺ��termios�������õ������У�
	if (tcsetattr(fd, TCSANOW, &options) != 0)
	{
		perror("com set error!/n");
		return (FALSE);
	}
	return (TRUE);
}


int UART_Init(int fd, int speed, int flow_ctrlint, int databits, int stopbits, char parity)
{
	//���ô�������֡��ʽ
	if (FALSE == UART_Set(fd, speed, flow_ctrlint, databits, stopbits, parity)) {
		return FALSE;
	}
	else {
		return TRUE;
	}
}



/*******************************************************************
* ���ƣ� UART0_Recv
* ���ܣ� ���մ�������
* ��ڲ����� fd :�ļ�������
* rcv_buf :���մ��������ݴ���rcv_buf��������
* data_len :һ֡���ݵĳ���
* ���ڲ����� ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int UART_Recv(int fd, char *rcv_buf, int data_len)
{
	int len, fs_sel;
	fd_set fs_read;

	struct timeval time;

	FD_ZERO(&fs_read);
	FD_SET(fd, &fs_read);

	time.tv_sec = 10;
	time.tv_usec = 0;

	//ʹ��selectʵ�ִ��ڵĶ�·ͨ��
	fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
	if (fs_sel){
		len = read(fd, rcv_buf, data_len);
		return len;
	}
	else {
		return FALSE;
	}
}

/*******************************************************************
* ���ƣ� UART0_Send
* ���ܣ� ��������
* ��ڲ����� fd :�ļ�������
* send_buf :��Ŵ��ڷ�������
* data_len :һ֡���ݵĸ���
* ���ڲ����� ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int UART_Send(int fd, char *send_buf, int data_len)
{
	int ret, i;
	//system("himm 0x12040090 0x1 ");
	printf("RS485 Send : ");
	for (i = 0; i < data_len; i++)
	{
		printf("0x%x ", send_buf[i]);
	}
	ret = write(fd, send_buf, data_len);
	if (data_len == ret){
		return ret;
	}
	else {
		tcflush(fd, TCOFLUSH);
		return FALSE;

	}

}



/*******************************************************************
* ���ƣ� ��RS485
* ���ܣ� ��������
* ��ڲ����� fd :�ļ�������
* send_buf :��Ŵ��ڷ�������
* data_len :һ֡���ݵĸ���
* ���ڲ����� ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
void RS485_B9600_Initial()
{
	fd_rs485 = UART_Open(fd_rs485, "/dev/ttyAMA1");
	if (fd_rs485 > 0)
	{
		printf("UART_Open /dev/ttyAMA1 success [%d]\n", fd_rs485);
		if (UART_Set(fd_rs485, 9600, 0, 8, 1, 'N') == FALSE)
			//if (UART_Set(com_fd, 115200, 0 , 8, 1, 'N') == FALSE)
		{
			printf("UART_Set /dev/ttyAMA1 Error!\n");
		}
		else
		{
			printf("UART_Set /dev/ttyAMA1 success!\n");
		}
	}
	else
	{
		printf("UART_Open /dev/ttyAMA1 Error [%d]\n", fd_rs485);
	}
}


/******************��е��̨����*************/


//ת��ISPͼ������ģ��
void Tranfer_Protocol_ISP_Module(char *szMsg, int nCmdLen)
{
	int socket1;
	if (nCmdLen < 3)
	{
		return -1;
	}
	if (!szMsg)
	{
		return -1;
	}
	struct sockaddr_in server;
	int len = sizeof(server);
	server.sin_family = AF_INET;
	server.sin_port = htons(16680); ///ISP server�ļ���Ĭ�϶˿�16680
	server.sin_addr.s_addr = inet_addr("127.0.0.1"); ///server�ĵ�ַ
	socket1 = socket(AF_INET, SOCK_DGRAM, 0);
	if (socket1 < 0)
	{
		printf("Tranfer_Protocol_ISP_Module socket err\n");
		return;
	}
	//UDP͸������ISPģ��
	sendto(socket1, szMsg, nCmdLen, 0, (struct sockaddr*)&server, len);
	//������ɹرգ�������ģʽ
	close(socket1);
}
//ת��ISPͼ������ģ��

/*Զң����ʵʱ����ֵ*/
RH_Coord gs_Coord = {0};

int Zoom_CMD = VISCA_ZOOM_STOP;

/*VISCAЭ���ʽ����*/
static RH_Procotol procoVisca = {
	/*����VISCAЭ��*/
	.type	  =   VISCA_PROCO, 
	.cmd_nums =   VISCA_INVALID,  /*��̨������*/
	.stCMD    = 
	{
		{
			VISCA_UP, /*��*/
			9,        /*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x03,0x01,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0xff,0xff,0xff}
		},
		{
			VISCA_DOWN, /*��*/
			9,		    /*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x03,0x02,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0xff,   0xff,0xff}
		},	
		{
			VISCA_LEFT, /*��*/
			9,		    /*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x01,0x03,0xff},
			{0xf0,0xff,0xff,0xff,0,       0,   0xff,0xff,0xff}
		},	
		{
			VISCA_RIGHT, /*��*/
			9,		     /*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x02,0x03,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0xff,   0xff,0xff}
		},	
		{
			VISCA_STOP, /*ֹͣ*/
			9,		    /*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x03,0x03,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,   0xff,0xff,0xff}
		},	
		{
			VISCA_UPLEFT, /*����*/
			9,		    /*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x01,0x01,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0,   0,0xff}
		},	
		{
			VISCA_UPRIGHT, /*����*/
			9,			/*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x02,0x01,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0,   0,0xff}
		},
		{
			VISCA_DOWNLEFT, /*����*/
			9,			/*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x01,0x02,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0,   0,0xff}
		},			
		{
			VISCA_DOWNRIGHT, /*����*/
			9,				/*�����*/
			{0x80,0x01,0x06,0x01,0x00,0x00,0x02,0x02,0xFF},
			{0xf0,0xff,0xff,0xff,0,       0,    0,   0,0xff}
		},
		{
			VISCA_ZOOMTELE, /*�䱶�� ֧���ٶ�*/
			6,				/*�����*/
			{0x80,0x01,0x04,0x07,0x20,0xFF},
			{0xf0,0xff,0xff,0xff,0xf0,0xff}
		},		
		{
			VISCA_ZOOMWIDE, /*�䱶С ֧���ٶ�*/
			6,				/*�����*/
			{0x80,0x01,0x04,0x07,0x30,0xFF},
			{0xf0,0xff,0xff,0xff,0xf0,0xff}
		},	
		{
			VISCA_ZOOMTELE, /*�䱶�� ��׼�ٶ�*/
			6,				/*�����*/
			{0x80,0x01,0x04,0x07,0x02,0xFF},
			{0xf0,0xff,0xff,0xff,0xff,0xff}
		},		
		{
			VISCA_ZOOMWIDE, /*�䱶С ��׼�ٶ�*/
			6,				/*�����*/
			{0x80,0x01,0x04,0x07,0x03,0xFF},
			{0xf0,0xff,0xff,0xff,0xff,0xff}
		},	
		{
			VISCA_ZOOMSTOP, 	/*�䱶ͣ*/
			6,				/*�����*/
			{0x80, 0x01, 0x04, 0x07, 0x00, 0xFF},
			{0xf0, 0xff, 0xff, 0xff, 0xff, 0xff}
		},		
		{
			VISCA_SPEED, 	
			7,				
			{0x80,0x01,0x06,0x01,0xff,0xff,0xff},
			{0}
		},		
		{
			VISCA_PRESET_SET,	
			7,				
			{0x80, 0x01, 0x04, 0x3F, 0x01, 0x00, 0xFF},
			{0xf0, 0xff, 0xff, 0xff, 0xff, 0x00,0xff}
		},
	
		{
			VISCA_PRESET_SWITCH, 
			7,				
			//{0x81,0x01,0x06,0x01,0xff,0xff,0xef,0xef,0x00},
			{0x80, 0x01, 0x04, 0x3F, 0x02, 0x00, 0xFF},
			{0xf0, 0xff, 0xff, 0xff, 0xff, 0x00,0xff}
		},
		
		{
			VISCA_DIRECTOR_AUTO, 
			9,				
			{0x81,0x01,0x06,0x01,0x11,0x11,0x11,0x11,0x01},
			{0}
		},
		{
			VISCA_DIRECTOR_MANUAL, 
			9,				
			{0x81,0x01,0x06,0x01,0x11,0x11,0x11,0x11,0x02},
			{0}
		},			
		{
			VISCA_DIRECTOR_UP_SPEED, 
			9,				
			{0x80,0x01,0x06,0x01,0x11,0x11,0x03,0x01,0xFF},
			{0}
		},
		{
			VISCA_DIRECTOR_DOWN_SPEED, 
			9,				
			{0x80,0x01,0x06,0x01,0x11,0x11,0x03,0x02,0xFF},
			{0}
		},		
		{
			VISCA_DIRECTOR_LEFT_SPEED, 
			9,				
			{0x80,0x01,0x06,0x01,0x11,0x11,0x01,0x03,0xFF},
			{0}
		},		
		{
			VISCA_DIRECTOR_RIGHT_SPEED, 
			9,				
			{0x80,0x01,0x06,0x01,0x11,0x11,0x02,0x03,0xFF},
			{0}
		},	
		{
			VISCA_DIRECTOR_LEFT_SPEED, 
			9,				
			{0x80,0x01,0x06,0x01,0x11,0x11,0x03,0x03,0xFF},
			{0}
		},			
	},
};

//���÷�����  
static void SetNonBlocking(int sockfd) 
{  
    int flag = fcntl(sockfd, F_GETFL, 0);  
    if (flag < 0) 
    {  
        perror("fcntl F_GETFL fail");  
        return;  
    } 
 
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) 
    {  
        perror("fcntl F_SETFL fail");  
    }  
}  


/*�Ƚ������Ƿ����*/
HI_BOOL CompareCmdEx(unsigned char *src ,unsigned char *dst, unsigned char *mask, int len)
{
	int cnt = 0 ;
	
	for (cnt = 0 ;cnt < len ; cnt++)   {

		//printf("%x %x %x\n",*src,*mask,*dst);
		if (((*src++)&(*mask++)) != *dst++) {
			return HI_FALSE;
		}
	}
	return HI_TRUE;
}

HI_BOOL CompareCmd(unsigned char *src ,unsigned char *dst, int len)
{
	int cnt = 0 ;
	
	for (cnt = 0 ;cnt < len ; cnt++)   {
		if ((*src++) != *dst++) {
			return HI_FALSE;
		}
	}
	return HI_TRUE;
}



/*����������̨*/
HI_S32 TestViscaPTZ(int cmd_mode)
{
	RH_Coord  *pCoord = &gs_Coord;
	HI_U32  s32Ret =HI_SUCCESS;

#if 0
	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_UP;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;

/*	
	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_ZOOMTELE;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
*/
	sleep(6);
	SAMPLE_PRT("------------------------------------------------------------UP-----------\n");
	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_DOWN;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(6);
	SAMPLE_PRT("-------------------------------------------------------------DOWN----------\n");
	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_LEFT;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(6);
	SAMPLE_PRT("-------------------------------------------------------------LEFT----------\n");

	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_RIGHT;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(6);
	SAMPLE_PRT("--------------------------------------------------------------RIGHT---------\n");

	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_UPLEFT;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(6);
	SAMPLE_PRT("--------------------------------------------------------------UPLEFT---------\n");
	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_DOWNRIGHT;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(10);
	SAMPLE_PRT("---------------------------------------------------------------DOWNRIGHT--------\n");

	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_DOWNLEFT;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(10);
	SAMPLE_PRT("---------------------------------------------------------------DOWNLEFT--------\n");
	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_UPRIGHT;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6;	
	sleep(10);
	SAMPLE_PRT("----------------------------------------------------------------UPRIGHT-------\n");
#endif

	pCoord->bEnable =HI_TRUE;
	pCoord->cmd_mode = VISCA_ZOOMTELE;
	pCoord->vSpeed = 6;
	pCoord->wSpeed = 6; 
	sleep(4);
	SAMPLE_PRT("----------------------------------------------------------------VISCA_ZOOMTELE-------\n");


/*	s32Ret = RH_MPI_VPSS_GetChnCrop(VpssChn,&pCoord->rect);	
	if (HI_SUCCESS != s32Ret)
	{
		SAMPLE_PRT("GET VPSS Crop Failed 0x%x\n",s32Ret);
	}
	
	SAMPLE_PRT("<TestViscaPTZ > Crop: X=%d   Y =%d   width: %d   height:%d  \n",pCoord->rect.s32X,
					pCoord->rect.s32Y,pCoord->rect.u32Width,pCoord->rect.u32Height);
*/
	
		
	return s32Ret;
}


/*VISCA�����������*/

static int vSpeed = 2;
static int wSpeed = 2;

int GetComCmd(unsigned char *cmd, int length)
{
	RH_Procotol *pVisca = &procoVisca;
	int i = 0;
	for (i = 0; i < pVisca->cmd_nums; i++)  
	{		
		if(pVisca->stCMD[i].code_len == length)
		{

			if(CompareCmdEx(cmd, pVisca->stCMD[i].code,pVisca->stCMD[i].mask ,length))
			{
				return pVisca->stCMD[i].cmd_mode;
			}
		}
	}

	return VISCA_STOP;
}


#define  MoveMaxSpeed    120
#define  MoveMinSpeed    1
#define  MoveStep   (MoveMaxSpeed - MoveMinSpeed)/0x18

#define  ZoomMaxSpeed     100
#define  ZoomMinSpeed    1
#define  ZoomStep       (ZoomMaxSpeed - ZoomMinSpeed)/7


//vv 1~0x18 ww 1~0x14
int GetMoveSpeed(unsigned char *cmd, int length)
{
	if(length < 5 || cmd[4] > 0x18)
	{
		return 9;
	}

	return cmd[4];
}

//0~7
int GetZoomSpeed(unsigned char *cmd, int length)
{

	if(length < 5 || (cmd[4]&0x0f > 7))
	{
		return 7;
	}
	
	if(cmd[4]&0xf0 == 0x20 || cmd[4]&0xf0 == 0x30)
	{
		return cmd[4]&0x0f;
	}
	
	return 7;
}



void* ElectroniPTZ(void* pdata)
{
	RH_Procotol *pVisca = &procoVisca;
	
	int chid = *(int *)(pdata);
	int x = 0;
	int y = 0;	
	int w = 1000;
	int h = 1000;

	printf("ElectroniPTZ ch:%d\n",chid);
	int UsleepTime = 30;
	
	VPSS_CROP_INFO_S stCropInfo;
	stCropInfo.bEnable = 1;
	//stCropInfo.enCropCoordinate = VPSS_CROP_ABS_COOR;
	stCropInfo.enCropCoordinate = VPSS_CROP_RATIO_COOR;
	#if 0
	stCropInfo.stCropRect.s32X   	= x;
	stCropInfo.stCropRect.s32Y      = y;
	stCropInfo.stCropRect.u32Width		=  w;
	stCropInfo.stCropRect.u32Height = h;
	HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VPSS_CROP_EXTCHN0, &stCropInfo);
	#endif

	int CurCmdMode = VISCA_STOP;
	VPSS_CHN VpssChn = VPSS_CROP_EXTCHN0;

	char dataZoomStr[32] = {0};
	char WriteZoomCnt = 0;
	char WriteZoomFlag = HI_FALSE;

	if(chid == 0)
	{
		 VpssChn = VPSS_CROP_EXTCHN0;
	}
	else
	{
		VpssChn = VPSS_CROP_EXTCHN1;
		x = 250;
		y = 250;
		w = 500;
		h = 500;
	}
	
	while(1)
	{

		    OSA_MsgHndl *pMsg;
			if(OSA_SOK == OSA_msgqRecvMsg(&QElectronicPTZ[chid], &pMsg, OSA_TIMEOUT_NONE))
			{
				char *cmd    = (unsigned char *)OSA_msgGetPrm(pMsg);
				int length = (unsigned char *)OSA_msgGetCmd(pMsg);
				OSA_msgqFreeMsgHndl(pMsg);
				CurCmdMode = GetComCmd(cmd,length);
				if(VISCA_LEFT == CurCmdMode || VISCA_RIGHT == CurCmdMode || VISCA_UP == CurCmdMode || VISCA_DOWN == CurCmdMode)
				{
					CurCmdMode = VISCA_STOP;
					continue;
					UsleepTime = MoveMaxSpeed - GetMoveSpeed(cmd,length)*MoveStep;
				}
				else if(VISCA_ZOOMTELE == CurCmdMode || VISCA_ZOOMWIDE == CurCmdMode)
				{
					UsleepTime = ZoomMaxSpeed - GetZoomSpeed(cmd,length)*ZoomStep;
				}

				printf("CurCmdMode %d %d %d %d\n",CurCmdMode,UsleepTime,MoveStep,ZoomStep);
				free(cmd);
			}

			if(VISCA_STOP == CurCmdMode)
			{
				usleep(200*1000);
			}
			else if(VISCA_LEFT == CurCmdMode)
			{
				stCropInfo.stCropRect.u32Width		=  w ;
				stCropInfo.stCropRect.u32Height = h;
				stCropInfo.stCropRect.s32X	= x;
				stCropInfo.stCropRect.s32Y= y;
				printf("left---wxh %dx%d x y %d %d\n",w,h,x,y);
				HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VpssChn, &stCropInfo);

				if( x <=   0)
				{
					CurCmdMode = VISCA_STOP;
					continue;
				}

				x	= ALIGN_BACK(x - 4,2);
				
			}
			else if(VISCA_RIGHT == CurCmdMode)
			{
				stCropInfo.stCropRect.u32Width		=  w ;
				stCropInfo.stCropRect.u32Height = h;
				stCropInfo.stCropRect.s32X	= x;
				stCropInfo.stCropRect.s32Y= y;
				printf("right---wxh %dx%d x y %d %d\n",w,h,x,y);
				HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VpssChn, &stCropInfo);

				if((x + w) >=   1000)
				{
					CurCmdMode = VISCA_STOP;
					continue;
				}

				x	= ALIGN_BACK(x + 4,2);
			}
			else if(VISCA_UP == CurCmdMode)
			{
				stCropInfo.stCropRect.u32Width		=  w ;
				stCropInfo.stCropRect.u32Height = h;
				stCropInfo.stCropRect.s32X	= x;
				stCropInfo.stCropRect.s32Y= y;
				printf("up---wxh %dx%d x y %d %d\n",w,h,x,y);
				HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VpssChn, &stCropInfo);

				if(y <= 0)
				{
					CurCmdMode = VISCA_STOP;
					continue;
				}

				y	= ALIGN_BACK(y - 4,2);
			}
			else if(VISCA_DOWN == CurCmdMode)
			{
				stCropInfo.stCropRect.u32Width		=  w ;
				stCropInfo.stCropRect.u32Height = h;
				stCropInfo.stCropRect.s32X	= x;
				stCropInfo.stCropRect.s32Y= y;

				printf("down---wxh %dx%d x y %d %d\n",w,h,x,y);
				HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VpssChn, &stCropInfo);
				
				if((y + h) >= 1000)
				{
					CurCmdMode = VISCA_STOP;
					continue;
				}

				y	= ALIGN_BACK(y + 4,2);
			}
			else if(VISCA_ZOOMTELE == CurCmdMode)
			{
				stCropInfo.stCropRect.u32Width		=  w ;
				stCropInfo.stCropRect.u32Height = h;
				stCropInfo.stCropRect.s32X	= x;
				stCropInfo.stCropRect.s32Y= y;
				printf("---wxh %dx%d x y %d %d\n",w,h,x,y);
				HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VpssChn, &stCropInfo);

				if(w <=  333  || h <= 333)
				{
					CurCmdMode = VISCA_STOP;
					continue;
				}

				PtzSpeed   = 0x3f*w/1000;
			
				w   = ALIGN_BACK(w - 8,2);
				h   = ALIGN_BACK(h - 8,2);
				x	= ALIGN_BACK(x + 4,2);
				y   = ALIGN_BACK(y + 4,2);
				
				sprintf(dataZoomStr, "x=%d,y=%d,w=%d,h=%d\n", x,y,w,h);
				WriteZoomFlag = HI_TRUE;
				WriteZoomCnt = 0;  //����zoom��?
#if 0
				FILE *fp = fopen("/usr/local/reach/bin/Coordinate.ini", "w+");
				char dataStr[32] = {0};
				sprintf(dataStr, "x=%d,y=%d,w=%d,h=%d\n", x,y,w,h);
				fwrite(dataStr, 1, strlen(dataStr), fp);
				fclose(fp);
#endif
			}
			else if(VISCA_ZOOMWIDE == CurCmdMode)
			{
				stCropInfo.stCropRect.u32Width		=  w ;
				stCropInfo.stCropRect.u32Height = h;
				stCropInfo.stCropRect.s32X	= x;
				stCropInfo.stCropRect.s32Y = y;
				printf("+++wxh %dx%d x y %d %d\n",w,h,x,y);
				HI_MPI_VPSS_SetExtChnCrop(VPSS_GROUP0,VpssChn, &stCropInfo);

				if(w >=  1000  || h >= 1000)
				{
					CurCmdMode = VISCA_STOP;
					continue;
				}

				PtzSpeed   = 0x3f*w/1000;
				w   = ALIGN_BACK(w + 8,2);
				h   = ALIGN_BACK(h + 8,2);

				
				//��߽�
				if(x <= 0)
				{
					x	= 0;
				}
				//�ұ߽�
				else if((x + w)   >= 1000)
				{
					x	= ALIGN_BACK(x   - 8,2);
				}
				else
				{
					x	= ALIGN_BACK(x   - 4,2);
				}

				//�ϱ߽�
				if(y <= 0)
				{
					y	= 0;
				}
				//�±߽�
				else if((y + h)   >= 1000)
				{
					y	= ALIGN_BACK(y   - 8,2);
				}
				else
				{
					y   = ALIGN_BACK(y - 4,2);
				}
				
                                sprintf(dataZoomStr, "x=%d,y=%d,w=%d,h=%d\n", x,y,w,h);
                                WriteZoomFlag = HI_TRUE;
                                WriteZoomCnt = 0;  //����zoom��?									
#if 0				
				FILE *fp = fopen("/usr/local/reach/bin/Coordinate.ini", "w+");
				char dataStr[32] = {0};
				sprintf(dataStr, "x=%d,y=%d,w=%d,h=%d\n", x,y,w,h);
				fwrite(dataStr, 1, strlen(dataStr), fp);
				fclose(fp);
#endif
			}

					
			usleep(UsleepTime*1000);
		
			/* zoom ������?*/
			if(WriteZoomFlag == HI_TRUE)
			{
				WriteZoomCnt ++;
			}	
			
			/* zoom ��?&& ����������zoom ���� */
			if((WriteZoomCnt >= 100) && (WriteZoomFlag == HI_TRUE))
			{
				FILE *fp = fopen("/usr/local/reach/bin/Coordinate.ini", "w+");
				fwrite(dataZoomStr, 1, strlen(dataZoomStr), fp);
				fclose(fp);
				WriteZoomCnt = 0;
				WriteZoomFlag = HI_FALSE;
			}

	}
	
}

extern HI_S32 RH_CoordSet(const RH_Coord *pCoord);

int ViscaProcoParse(unsigned char *pCmd, int length)
{
	RH_Procotol *pVisca = &procoVisca;
	RH_Coord  *pCoord = &gs_Coord;
	int cnt = 0;
	HI_BOOL is_equal = HI_TRUE;

	if (pVisca->cmd_nums > MAX_CMD_NUMS) {
		SAMPLE_PRT("max command numbers:%d\n",pVisca->cmd_nums);
		return HI_FAILURE;
	}
	//转发ISP图像设置模块
	Tranfer_Protocol_ISP_Module(pCmd, length);
	//机械云台
	int ret;
	int i = 0;
	char pelco_d_cmd[10];
	pelco_d_cmd[0] = 0xFF; //pelco_d协议�?	
	pelco_d_cmd[1] = 0x01; //pelco_d地址�?	
	pelco_d_cmd[2] = 0x00; //pelco_d指令1
	pelco_d_cmd[4] = 0x00;  //pelco_d水平速度
	pelco_d_cmd[5] = 0x00;   //pelco_d垂直速度
	printf("----------------------------------------------------\n");
	for (i = 0; i < length; i++)
	{
		printf("0x%x ", pCmd[i]);
	}
	printf("----------------------------------------------------\n");

	if ((pCmd[0] == 0x81 || pCmd[0] == 0x82) && pCmd[1] == 0x01 && pCmd[2] == 0x04 && pCmd[3] == 0x07 && (pCmd[4] == 0x02 || pCmd[4] == 0x26) && pCmd[5] == 0xFF)
	{
		//变倍大
		printf("\n zoom+ \n");
		pelco_d_cmd[3] = 0x20; //pelco_p 变倍长    放大+
		pelco_d_cmd[4] = 0x0;  //pelco_pˮƽ�ٶ�
		pelco_d_cmd[5] = 0x0;   //pelco_p��ֱ�ٶ�
		pelco_d_cmd[6] = ((pelco_d_cmd[1] + pelco_d_cmd[2] + pelco_d_cmd[3] + pelco_d_cmd[4] + pelco_d_cmd[5]) % 0x100);
		//����RS485
		if (fd_rs485 > 0)
		{
			ret = UART_Send(fd_rs485, pelco_d_cmd, 7);
			if (ret > 0)
			{
				printf("RS485_Send Ok [%d]\n", ret);
			}
			else
			{
				printf("RS485_Send Error [%d]\n", ret);
			}
		}
		return HI_SUCCESS;		
	}
	else if ((pCmd[0] == 0x81 || pCmd[0] == 0x82) && pCmd[1] == 0x01 && pCmd[2] == 0x04 && pCmd[3] == 0x07 && (pCmd[4] == 0x03 || pCmd[4] == 0x36) && pCmd[5] == 0xFF)
	{
		//变倍小
		printf("\n zoom- \n");
		pelco_d_cmd[3] = 0x40; //pelco_p 变倍短 缩小-
		pelco_d_cmd[4] = 0x0;  //pelco_pˮƽ�ٶ�
		pelco_d_cmd[5] = 0x0;	//pelco_p��ֱ�ٶ�
		pelco_d_cmd[6] = ((pelco_d_cmd[1] + pelco_d_cmd[2] + pelco_d_cmd[3] + pelco_d_cmd[4] + pelco_d_cmd[5]) % 0x100); //У����
		//����RS485
		if (fd_rs485 > 0)
		{
			ret = UART_Send(fd_rs485, pelco_d_cmd, 7);
			if (ret > 0)
			{
				printf("RS485_Send Ok [%d]\n", ret);
			}
			else
			{
				printf("RS485_Send Error [%d]\n", ret);
			}
		}
		
		return HI_SUCCESS;		
	}
	else if ((pCmd[0] == 0x81 || pCmd[0] == 0x82) && pCmd[1] == 0x01 && pCmd[2] == 0x04 && pCmd[3] == 0x07 && pCmd[4] == 0x00 && pCmd[5] == 0xFF)
	{
	   //变倍停�?		printf("\n zoom stop \n");
		pelco_d_cmd[3] = 0x00; //pelco_p 变倍停�?		pelco_d_cmd[4] = 0x0;  //pelco_pˮƽ�ٶ�
		pelco_d_cmd[5] = 0x0;	//pelco_p��ֱ�ٶ�
		pelco_d_cmd[6] = ((pelco_d_cmd[1] + pelco_d_cmd[2] + pelco_d_cmd[3] + pelco_d_cmd[4] + pelco_d_cmd[5]) % 0x100); //У����
		//����RS485
		if (fd_rs485 > 0)
		{
			ret = UART_Send(fd_rs485, pelco_d_cmd, 7);
			if (ret > 0)
			{
				printf("RS485_Send Ok [%d]\n", ret);
			}
			else
			{
				printf("RS485_Send Error [%d]\n", ret);
			}
		}
		
		return HI_SUCCESS;		   
	}
	//变焦+ Far(Standard) 
	else if ((pCmd[0] == 0x81 || pCmd[0] == 0x82) && pCmd[1] == 0x01 && pCmd[2] == 0x04 && pCmd[3] == 0x08 && pCmd[4] == 0x02 && pCmd[5] == 0xFF)
	{
		//聚焦�?    聚焦+
		
		printf("\n Focus far \n");
		pelco_d_cmd[2] = 0x01;
		pelco_d_cmd[3] = 0x00; //pelco_p FOCUS_FAR
		pelco_d_cmd[4] = 0x00;  //pelco_pˮƽ�ٶ�
		pelco_d_cmd[5] = 0x00;   //pelco_p��ֱ�ٶ�
		pelco_d_cmd[6] = ((pelco_d_cmd[1] + pelco_d_cmd[2] + pelco_d_cmd[3] + pelco_d_cmd[4] + pelco_d_cmd[5]) % 0x100); //У����
		//����RS485
		if (fd_rs485 > 0)
		{
			ret = UART_Send(fd_rs485, pelco_d_cmd, 7);
			if (ret > 0)
			{
				printf("RS485_Send Ok [%d]\n", ret);
			}
			else
			{
				printf("RS485_Send Error [%d]\n", ret);
			}
		}
		return HI_SUCCESS;	
	}
	//变焦- Near(Standard) 
	else if ((pCmd[0] == 0x81 || pCmd[0] == 0x82) && pCmd[1] == 0x01 && pCmd[2] == 0x04 && pCmd[3] == 0x08 && pCmd[4] == 0x03 && pCmd[5] == 0xFF)
	{
		//聚焦�?    聚焦-
		printf("\n Focus near \n");
		pelco_d_cmd[2] = 0x00;
		pelco_d_cmd[3] = 0x80; //pelco_p FOCUS_NEAR
		pelco_d_cmd[4] = 0x00;  //pelco_pˮƽ�ٶ�
		pelco_d_cmd[5] = 0x00;   //pelco_p��ֱ�ٶ�
		pelco_d_cmd[6] = ((pelco_d_cmd[1] + pelco_d_cmd[2] + pelco_d_cmd[3] + pelco_d_cmd[4] + pelco_d_cmd[5]) % 0x100); //У����
		//����RS485
		if (fd_rs485 > 0)
		{
			ret = UART_Send(fd_rs485, pelco_d_cmd, 7);
			if (ret > 0)
			{
				printf("RS485_Send Ok [%d]\n", ret);
			}
			else
			{
				printf("RS485_Send Error [%d]\n", ret);
			}
		}
		return HI_SUCCESS;			
	}
	//聚焦Stop
	else if ((pCmd[0] == 0x81 || pCmd[0] == 0x82) && pCmd[1] == 0x01 && pCmd[2] == 0x04 && pCmd[3] == 0x08 && pCmd[4] == 0x00 && pCmd[5] == 0xFF)
	{
		//聚焦停止
		printf("\n Focus stop \n");
		pelco_d_cmd[2] = 0x00;
		pelco_d_cmd[3] = 0x00; //pelco_p FOCUS_STOP
		pelco_d_cmd[4] = 0x00;  //pelco_pˮƽ�ٶ�
		pelco_d_cmd[5] = 0x00;   //pelco_p��ֱ�ٶ�
		pelco_d_cmd[6] = ((pelco_d_cmd[1] + pelco_d_cmd[2] + pelco_d_cmd[3] + pelco_d_cmd[4] + pelco_d_cmd[5]) % 0x100); //У����
		//����RS485
		if (fd_rs485 > 0)
		{
			ret = UART_Send(fd_rs485, pelco_d_cmd, 7);
			if (ret > 0)
			{
				printf("RS485_Send Ok [%d]\n", ret);
			}
			else
			{
				printf("RS485_Send Error [%d]\n", ret);
			}
		}
		return HI_SUCCESS;			
	}
				
	return HI_SUCCESS;
}

/*Э����ս����߳�*/
HI_VOID* RH_UDP_Recv_Parse(HI_VOID* pdata)
{
    struct sockaddr_in addr;
	unsigned char buff[4096] = {0};
	int sock;
	
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);	
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    {
        perror("socket");
        return ;
    }
	SetNonBlocking(sock);	
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)   {
        perror("bind");
        return ;
    }
	//Ĭ�ϴ�RS485
	RS485_B9600_Initial();
    struct sockaddr_in clientAddr;
    int iSize = 0;
    int length = sizeof(clientAddr);

	while(HI_TRUE == gs_ThreadStart)
	{
		memset(buff, 0, 4096);
        iSize = recvfrom(sock,buff,sizeof(buff), 0, (struct sockaddr*)&clientAddr, &length);
        if (iSize > 0)  {
			/*TODO��Ŀǰֻ����VISCAЭ��*/

			printf("\n\n\niSize(%d), length(%d)  VISCA(0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x)\n\n\n", 
				iSize, length, buff[0], buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8]);


			void *prm = OSA_memAlloc(4096);
			if(prm)
			{
				memcpy(prm, buff, iSize);
				if((buff[0] & 0x0f) == 0x2)
				{
					OSA_msgqSendMsgEx(&QElectronicPTZ[0], NULL, iSize, (void*)prm, 0, NULL, OSA_TIMEOUT_NONE);
				}
				else if((buff[0] & 0x0f) == 0x1)
				{
					OSA_msgqSendMsgEx(&QElectronicPTZ[1], NULL, iSize, (void*)prm, 0, NULL, OSA_TIMEOUT_NONE);
				}
				else
				{	
					OSA_memFree(prm);
				}
			}

			ViscaProcoParse(buff,iSize);
        }
		usleep(2000);
	}
	return ;
}


/*Ycat ����ʹ��UDP����ViscaЭ��*/
HI_S32 RH_UDP_Proto_Start(HI_VOID)
{
	HI_S32 s32Ret = HI_SUCCESS;
	int i = 0;

	OSA_msgqCreate(&QElectronicPTZ[0]);
	OSA_msgqCreate(&QElectronicPTZ[1]);

	
	s32Ret = pthread_create(&gs_ProtoPid, 0, RH_UDP_Recv_Parse, (HI_VOID*)NULL);
	i = 0;

	s32Ret = pthread_create(&gs_ProtoPid, 0, ElectroniPTZ, (HI_VOID*)&i);
	sleep(1);
	i = 1;
	s32Ret = pthread_create(&gs_ProtoPid, 0, ElectroniPTZ, (HI_VOID*)&i);
	sleep(1);
	return s32Ret;
}


/*ֹͣ*/
HI_U32 RH_UDP_Proto_Stop(HI_VOID)
{
    /* join thread */
    if (HI_TRUE == gs_ThreadStart)
    {
        gs_ThreadStart = HI_FALSE;
        pthread_join(gs_ProtoPid, 0);
    }
	return HI_SUCCESS;
}

