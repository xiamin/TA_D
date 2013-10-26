
/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include "board.h"
#include <rtthread.h>
#include "tad_LCD.h"
#include "tad_SpiFlash.h"
#include "tad_Downloader.h"
#include "i2c.h"
#include "tad_e2prom.h"
//#define TEST_PC
#ifdef RT_USING_SPI
#include <spi_flash_sst25vfxx.h>
#endif

#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

struct rt_mutex Mutex1;

struct rt_mutex SysInit;

#ifdef TEST_PC

#define THREAD_STACK_SIZE	512
/*定义最大5个元素能被产生*/
#define MAXSEM 5
/*用于放置生产的整数数组*/
rt_int32_t array[MAXSEM];
/*指向生产者、消费者在array数组中的读写位置*/
static rt_uint32_t set, get;
/*定义二值信号量实现临界区互斥*/
struct rt_semaphore sem_lock;
/*定义信号量用于生产者和消费者线程*/
struct rt_semaphore sem_empty, sem_full;
#endif

void producer_thread_entry(void* parameter);
void consumer_thread_entry(void* parameter);

void rt_mutex_init_demo()
{
    rt_err_t rtResult;
    rt_mutex_init(&Mutex1, "ABC", 0);
    rtResult = rt_mutex_take(&Mutex1, RT_WAITING_FOREVER);
    if (rtResult == RT_EOK)
    {
        rt_thread_delay(2000);
        unsigned char i;
        for (i = 0; i < 3; i++)
        {
            unsigned char abyTmp[64];
            rt_device_t plcd1602;
            plcd1602 = rt_device_find("Lcd1602");
            sprintf(abyTmp, "TA Downloader Staring...  %d  s", i);
            plcd1602->write(plcd1602, -1, abyTmp, 0xff);
            rt_thread_delay(1000);
        }
        rt_mutex_release(&Mutex1);
    }
}

void rt_init_thread_entry(void* parameter)
{ 
    
#ifdef RT_USING_I2C
    /*i2c driver init*/
    rt_i2c_core_init();
    rt_hw_i2c_init();
#endif

    /*lcd device init*/    
    //rt_hw_lcd1602_init();
    
    rt_mutex_init(&SysInit, "Init", 0);
    
    /* Filesystem Initialization */
#ifdef RT_USING_DFS
    {
        if(w25qxx_init("flash0", "spi10") != RT_EOK)
        {
                rt_kprintf("[error] No such spi flash!\r\n");
        }
        /* init the device filesystem */
        dfs_init();

#ifdef RT_USING_DFS_ELMFAT
        /* init the elm chan FatFs filesystam*/
        elm_init();

        /* mount sd card fat partition 1 as root directory */
#if 0
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
#else
        if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
#endif
        {
            rt_kprintf("File System initialized!\n");
        }
        else
            rt_kprintf("File System initialzation failed!\n");
#endif
    }
#endif
    e2prom_device_init("e2prom", "i2c1");
#if 0
	/*文件系统测试代码*/
	int fd1 = 0;
	fd1 = open("/myfile.txt", DFS_O_CREAT | DFS_O_RDWR, 0);
    	if(fd1 < 0)
        {
        	rt_kprintf("open file failed!\r\n");
        }
        else
        {
        	int count = write(fd1, "123456", 7);
            	char buf[10];

                close(fd1);
                fd1 = 0;
                rt_thread_delay(50);
                rt_memset(buf, 0, 10);
                fd1 = open("/myfile.txt", DFS_O_RDONLY, 0);
                if(read(fd1, buf, 7))
                {
                	rt_kprintf("read = %s\r\n", buf);
                }
                else
                {
                	rt_kprintf("read file error!\r\n");
                }
        }
#endif
}
#if 0
void rt_485_thread_entry(void *parameter)
{
    uint8_t byRxData[30];
    rt_device_t pCom485;
    pCom485 = rt_device_find("485");

    if (RT_NULL != pCom485)
    {
         rt_size_t RxNum;
         while (1)
         {
             memset (byRxData, 0, 30);
             pCom485->write(pCom485, 0, "This Is 485 Ask, Pls Reply!\n", 27);
             rt_thread_delay(1000);
             RxNum = pCom485->read(pCom485, 0, byRxData, 28);
             if (RxNum)
            {
                uint8_t byTmpRply[30];
                uint8_t *pTmp;
                pTmp = byTmpRply;
                memset(byTmpRply, 0, 30);
                sprintf(byTmpRply, "Get %d Bytes!", RxNum);
                do
                {
                    pCom485->write(pCom485, 0, pTmp, 1);
                    pTmp++;
                 }while (0 != *pTmp);
            }
             else
             {
                    pCom485->write(pCom485, 0, "No Data Rx!", 11);
             }
             rt_thread_delay(5000);
         }
    }
}
#endif
int rt_application_init()
{
    rt_thread_t init_thread;

    rt_thread_t rs485_thread;

    rt_thread_t Process_thread;
    
    rt_err_t result;

    
#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

#ifdef TEST_PC
    rt_thread_t tid;

    /*初始化三个信号量*/
    rt_err_t res;
    res = rt_sem_init(&sem_lock, "lock", 1, RT_IPC_FLAG_FIFO);
    if(res != RT_EOK)
    {
    	goto _error;
    }
    res = rt_sem_init(&sem_empty, "empty", MAXSEM, RT_IPC_FLAG_FIFO);
    if(res != RT_EOK)
    {
    	goto _error;
    }
    result = rt_sem_init(&sem_full , "full", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        goto _error;

    tid = rt_thread_create("producer", producer_thread_entry, RT_NULL, THREAD_STACK_SIZE, 10, 5);
    if(tid != RT_NULL)
    {
    	rt_thread_startup(tid);
    }
    tid = rt_thread_create("cousumer1", consumer_thread_entry, (void*)1, THREAD_STACK_SIZE, 11, 2);
    if(tid != RT_NULL)
    {
    	rt_thread_startup(tid);
    }
    tid = rt_thread_create("cousumer1", consumer_thread_entry, (void*)2, THREAD_STACK_SIZE, 11, 2);
    if(tid != RT_NULL)
    {
    	rt_thread_startup(tid);
    }

#endif

#if 0
    rs485_thread = rt_thread_create("rs485",
                                   rt_485_thread_entry, RT_NULL,
                                   2048, 8, 20);
#if 0
    if (rs485_thread != RT_NULL)
        rt_thread_startup(rs485_thread);
#endif
#endif

#if 0    
    Process_thread = rt_thread_create("TaDown",
                                   rt_TaDownLoader_thread_entry, RT_NULL,
                                   8192, 1, 20);
    
    if (Process_thread != RT_NULL)
        rt_thread_startup(Process_thread);
#endif

    return 0;
_error:
    rt_kprintf("init semaphore failed.\r\n");
    return -1;
}
#ifdef TEST_PC
void producer_thread_entry(void* parameter)
{
	int cnt = 0;
    /*运行20次*/
    while(cnt < 100)
    {
    	/*获取一个空位*/
        rt_sem_take(&sem_empty, RT_WAITING_FOREVER);

        /*修改array内容，上锁*/
        rt_sem_take(&sem_lock, RT_WAITING_FOREVER);
        array[set % MAXSEM] = cnt + 1;
        rt_kprintf("the producer generates a number: %d\n", array[set%MAXSEM]);
        set++;
        rt_sem_release(&sem_lock);

        /*发布一个空位*/
        rt_sem_release(&sem_full);
        cnt++;

        /*暂停一段时间*/
    }
    rt_kprintf("the producer exit!\n");
}

void consumer_thread_entry(void* parameter)
{
    rt_uint32_t no;
    rt_uint32_t sum;
    int* a= NULL;
    /* 第n个线程，由入口参数传进来 */
    no = (rt_uint32_t)parameter;
    sum = 0;

    while(1)
    {
        /* 获取一个满位 */
        rt_sem_take(&sem_full, RT_WAITING_FOREVER);

        /* 临界区，上锁进行操作 */
        rt_sem_take(&sem_lock, RT_WAITING_FOREVER);
        sum += array[get%MAXSEM];
        rt_kprintf("the consumer[%d] get a number: %d\n", no, array[get%MAXSEM] );
        get++;
        rt_sem_release(&sem_lock);

        /* 释放一个空位 */
        rt_sem_release(&sem_empty);

        /* 生产者生产到20个数目，停止，消费者线程相应停止 */
        if (get == 20) 
            break;

        /* 暂停一小会时间 */
        rt_thread_delay(10);
    }

    rt_kprintf("the consumer[%d] exits, sum is %d \n ", no, sum);
}
#endif
/*@}*/
