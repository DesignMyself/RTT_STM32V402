/*
 * File      : app_uart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 *2017-12-15      DQL          the first version 
 */

#ifndef __APP_USART_H__
#define __APP_USART_H__

#include <rthw.h>
#include <rtthread.h>
rt_err_t uart1_open(const char *name);
rt_uint8_t uart1_getchar(void);
typedef union SPEED{
	float speed;
	uint8_t PWM_Value[4];
	
}Speed_Get;
void uart1_putchar(const rt_uint8_t c);
void uart1_putstring(const char *s);

rt_err_t uart2_open(const char *name);
rt_uint8_t uart2_getchar(void);
void uart2_putchar(const rt_uint8_t c);
void uart2_putstring( char*s);

rt_err_t uart3_open(const char *name);
rt_uint8_t uart3_getchar(void);
void uart3_putchar(const rt_uint8_t c);
void uart3_putstring( char*s);
rt_uint8_t  uart_getchar_len(rt_device_t dev, uint8_t *ch);

	


#endif
