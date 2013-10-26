/*
 * File      : ymodem.c
 * the implemention of Ymodem protocol.
 * Change Logs:
 * Date           Author       Notes
 * 2013-02-03     Fe.M      
 */



#include "ymodem.h"

#ifdef RT_USING_FINSH
#include <finsh.h>
#include <shell.h>


struct ymodemf ymodem;


extern void ymodem_recv_start(char *path);



rt_err_t ymodem_rx_ind(rt_device_t dev, rt_size_t size)
{
	/* release semaphore */
	rt_sem_release(ymodem.ysem);

	return RT_EOK;
}


void finsh_ry(void *parameter)
{
	char *path;	
	rt_err_t (*rx_indicate)(rt_device_t dev, rt_size_t size);
	rt_uint8_t flag;	

	flag = RT_DEVICE_FLAG_STREAM;
	ymodem.device->flag &=(~flag);
	ymodem.ysem = rt_sem_create("ysem",0,RT_IPC_FLAG_FIFO);

	path = rt_thread_self()->parameter;	
	
	/* save old rx_indicate	*/
	rx_indicate = ymodem.device->rx_indicate;
	/* set new rx_indicate */
	rt_device_set_rx_indicate(ymodem.device,ymodem_rx_ind);
	/* start receive remote files */
	ymodem_recv_start(path);
	ymodem.device->flag |=flag;
	/* recovery old rx_indicate	*/
	rt_device_set_rx_indicate(ymodem.device, rx_indicate);
	/* finsh>> */
	rt_kprintf("\r\n");
	rt_kprintf(FINSH_PROMPT);

}

static void ry(char *para)
{
	rt_thread_t init_thread;
	rt_device_t device;
	const char* device_name = finsh_get_device();

	device = rt_device_find(device_name);
	if( device == RT_NULL )
	{
		rt_kprintf("%s not find\r\n",device_name);
	}
	ymodem.device = device;
	init_thread = rt_thread_create("ry",
		finsh_ry,
		(void *)para,
		2048,
		rt_thread_self()->current_priority+1,
		20);

	if (init_thread != RT_NULL) rt_thread_startup(init_thread);
}
FINSH_FUNCTION_EXPORT(ry, receive files by ymodem protocol)

#endif

