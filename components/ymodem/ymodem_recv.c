/*
 * File      : ymodem_recv.c
 * the implemention of receiving files from the remote computers
 * through the Ymodem protocol.
 * Change Logs:
 * Date           Author       Notes
 * 2013-02-03     Fe.M
 */


#include "ymodem.h"
#include <finsh.h>
#include <shell.h>
#include <dfs_posix.h>


extern struct ymodemf ymodem;

#define RX_BUF_SIZE	2048
static rt_err_t yrec_init(rt_uint8_t * rxbuf);
static rt_err_t ymodem_r(rt_uint8_t* rxbuf, rt_uint16_t len);

rt_uint16_t get_crc16(rt_uint8_t *data, rt_uint16_t leng)
{
    rt_int32_t j;
    rt_uint8_t i;
    rt_uint16_t crc, t;
    crc = 0;
    for(j = leng; j > 0; j--)
    {
        crc = (crc ^ (((rt_uint16_t) *data) << 8));
        for(i = 8; i > 0; i--)
        {
            t = crc << 1;
            if(crc & 0x8000)
                t = t ^ 0x1021;
            crc = t;
        }
        data++;
    }
    return crc;
}

uint8_t crc_check(rt_uint8_t *buf,rt_uint32_t length)
{
    rt_err_t result = RT_EOK;

    rt_uint16_t crc = (rt_uint16_t)((buf[length + 3] << 8) | buf[length +4]);
    if (get_crc16(&buf[3],length) != crc)
    {
        result = RT_ERROR;
    }
    return result;
}

void ymodem_send_byte(rt_uint16_t ch)
{
    rt_device_write(ymodem.device,0,&ch,1);
}

rt_uint8_t ymodem_recv_packet(rt_uint8_t *buf,rt_uint32_t *length)
{
    rt_err_t result;
    rt_uint32_t cnt;
    result = rt_sem_take(ymodem.ysem,RT_TICK_PER_SECOND * 4);
    if (result == RT_EOK)
    {
        char ch;
        rt_err_t rx_result;
        cnt = 0;

        rx_result = rt_device_read(ymodem.device, 0, &ch, 1);
        if (rx_result == 1)
        {
            buf[cnt++] = ch;

            while(rt_sem_take(ymodem.ysem, 200) == RT_EOK)
            {
                while (rt_device_read(ymodem.device, 0, &ch, 1) == 1)
                {
                    buf[cnt++] = ch;
                    if (cnt > RX_BUF_SIZE)
                    {
                        cnt = 0;
                    }
                }
            }
        }
        if (buf[0] != YMODEM_EOT)
        {
            if (buf[1] + buf[2] == 255)
            {
                if (buf[0] == YMODEM_SOH)
                {
                    result = crc_check(buf,128);
                }
                else
                {
                    result = crc_check(buf,1024);
                }
            }
        }
    }
    *length = cnt;
    return result;

}

void ymodem_recv_start(char *path)
{
    rt_uint32_t i,length;
    int fd;
    char *file_path = RT_NULL;
    rt_uint8_t *rx_buf = RT_NULL;
    char *file_name = RT_NULL;
    rt_uint32_t file_length,recv_cnt,write_size;
    rx_buf = rt_malloc(RX_BUF_SIZE);
    if (rx_buf == RT_NULL)
    {
        return;
    }
    //yrec_init(rx_buf);
  
    for (i = 0; i < 10; i++)
    {
        ymodem_send_byte(YMODEM_C);
        if (ymodem_recv_packet(rx_buf,&length) == RT_EOK)
        {

            file_name = rt_strdup((char *)&rx_buf[3]);
            length = rt_strlen(file_name);
            file_length = atoi((char *)&rx_buf[3+length+1]);
            ymodem_send_byte(YMODEM_ACK);
            //TODO  这里创建文件...
            if (strcmp(path,"/") != RT_NULL)
            {
                path = strcat(path,"/");
            }
            file_path = strcat(path,file_name);


            if (file_path != RT_NULL)
            {

                fd = open(file_path, O_WRONLY | O_TRUNC, 0);
                if (fd < 0)
                {
                    close(fd);
                    rt_free(file_name);
                    rt_free(rx_buf);
                    return;
                }
            }
            break;
        }

    }

    if (i < 10)
    {
        for (i = 0; i < 10; i++)
        {
            ymodem_send_byte(YMODEM_C);
            if (ymodem_recv_packet(rx_buf,&length) == RT_EOK)
            {
                rt_uint8_t cnt = 0x01;
                if (rx_buf[1] != cnt++)
                {
                    ymodem_send_byte(YMODEM_CAN);
                    rt_thread_delay(100);
                    ymodem_send_byte(YMODEM_CAN);
                    //TODO 做一些结束前的必要工作
                    rt_free(file_name);
                    rt_free(rx_buf);
                    close(fd);
                    break;

                }
                recv_cnt = 0;
                //TODO 这里保存文件第一包数据
                if (rx_buf[0] == YMODEM_SOH)
                {
                    recv_cnt += 128;
                    if (recv_cnt > file_length)
                    {
                        write(fd,&rx_buf[3],file_length);
                    }
                    else
                    {
                        write(fd,&rx_buf[3],128);
                    }
                }
                else
                {
                    recv_cnt += 1024;
                    if (recv_cnt > file_length)
                    {
                        write(fd,&rx_buf[3],file_length);
                    }
                    else
                    {
                        write(fd,&rx_buf[3],1024);

                    }
                }
                ymodem_send_byte(YMODEM_ACK);



                while (1)
                {

                    if (ymodem_recv_packet(rx_buf,&length) == RT_EOK)
                    {
                        if (rx_buf[0] == YMODEM_EOT)
                        {
                            //TODO 文件传输完成
                            ymodem_send_byte(YMODEM_ACK);
                            rt_thread_delay(1);
                            ymodem_send_byte(YMODEM_C);
                            if(ymodem_recv_packet(rx_buf,&length) == RT_EOK)
                            {
                                ymodem_send_byte(YMODEM_ACK);
                                close(fd);
                                rt_free(file_name);
                                rt_free(rx_buf);
                                goto complete;
                            }
                        }
                        else
                        {
                            if (rx_buf[1] != cnt++)
                            {
                                ymodem_send_byte(YMODEM_CAN);
                                rt_thread_delay(100);
                                ymodem_send_byte(YMODEM_CAN);
                                close(fd);
                                //TODO 做一些结束前的必要工作
                                rt_free(file_name);
                                rt_free(rx_buf);
                                goto complete;

                            }


                        }
                        //TODO 这里保存文件
                        if (rx_buf[0] == YMODEM_SOH)
                        {
                            recv_cnt += 128;
                            if (recv_cnt > file_length)
                            {
                                write_size = file_length%128;
                                write(fd,&rx_buf[3],write_size);
                            }
                            else
                            {
                                write(fd,&rx_buf[3],128);
                            }
                        }
                        else
                        {
                            recv_cnt += 1024;
                            if (recv_cnt > file_length)
                            {
                                write_size = file_length%1024;
                                write(fd,&rx_buf[3],write_size);
                            }
                            else
                            {
                                write(fd,&rx_buf[3],1024);

                            }

                        }
                        ymodem_send_byte(YMODEM_ACK);
                    }
                    else
                    {
                        ymodem_send_byte(YMODEM_NAK);
                    }
                }
            }
        }
    }
complete :
    return;
}

#if 0
static rt_err_t yrec_init(rt_uint8_t * rxbuf)
{
        rt_int8_t ch;
	ymodem_send_byte(YMODEM_C);
        ymodem_r(rxbuf, 1029);
#if 0
	while(1)
	{
		if (rt_sem_take(ymodem.ysem, RT_WAITING_FOREVER) != RT_EOK) continue;

            	rt_device_read(ymodem.device, 0, rxbuf, 1);
                
	}
#endif
}
static rt_err_t ymodem_r(rt_uint8_t* rxbuf, rt_uint16_t len)
{
	rt_uint8_t ch;
	//rt_device_read(ymodem.device, 0, &ch, 1);
    while(rxbuf < (rxbuf + len))
    {
        if (rt_sem_take(ymodem.ysem, RT_WAITING_FOREVER) != RT_EOK) continue;
        if(rt_device_read(ymodem.device, 0, &ch, 1))
        {
            *rxbuf++ = ch;
        }
        else
        {	
            ymodem_send_byte(YMODEM_C);
        }
    }
}
#endif
