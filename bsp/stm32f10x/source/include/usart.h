/*
 * File      : usart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#ifndef __USART_H__
#define __USART_H__

#include <rthw.h>
#include <rtthread.h>
#include "stdint.h"
#include "stm32f10x.h"
void uart_SetBps(USART_TypeDef *Port, uint32_t br, uint16_t pariMode);
void rt_hw_usart_init(void);
extern struct rt_device uart1_device;
#endif
