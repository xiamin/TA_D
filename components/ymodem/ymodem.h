#ifndef __YMODEM_H__
#define __YMODEM_H__


#include "rtthread.h"
#include "stm32f10x.h"


#define YMODEM_SOH	0x01	//数据块起始字符
#define YMODEM_STX	0x02	//１０２８字节开始
#define YMODEM_EOT	0x04	//文件传输结束
#define YMODEM_ACK	0x06	//确认应答
#define YMODEM_NAK	0x15	//出现错误
#define YMODEM_CAN	0x18	//取消传输
#define YMODEM_C	0x43	//大写字母Ｃ



struct ymodemf
{	
	rt_sem_t ysem;
	rt_device_t device;
};


#endif

