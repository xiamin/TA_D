#ifndef __YMODEM_H__
#define __YMODEM_H__


#include "rtthread.h"
#include "stm32f10x.h"


#define YMODEM_SOH	0x01	//���ݿ���ʼ�ַ�
#define YMODEM_STX	0x02	//���������ֽڿ�ʼ
#define YMODEM_EOT	0x04	//�ļ��������
#define YMODEM_ACK	0x06	//ȷ��Ӧ��
#define YMODEM_NAK	0x15	//���ִ���
#define YMODEM_CAN	0x18	//ȡ������
#define YMODEM_C	0x43	//��д��ĸ��



struct ymodemf
{	
	rt_sem_t ysem;
	rt_device_t device;
};


#endif

