 /* File      : app_uart.c
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

#include "drv_usart.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>

#include "app_uSart.h"

/* 串口1接收事件标志 */
#define UART1_RX_EVENT (1 << 0)


/* 串口2接收事件标志 */
#define UART2_RX_EVENT (1 << 1)

/* 串口3接收事件标志 */
#define UART3_RX_EVENT (1 << 2)

/* 事件控制块 */
static struct rt_event event1;
static struct rt_event event2;//串口2接收控制事件
static struct rt_event event3;//串口2接收控制事件
static uint8_t uart1_Init_Finsh=0;//串口1初始化完成标准
static uint8_t uart2_Init_Finsh=0;//串口2初始化完成标准
static uint8_t uart3_Init_Finsh=0;//串口3初始化完成标准

/* 串口1设备句柄 */
 rt_device_t uart_device1 = RT_NULL;//用于第一个串口 USART1

/* 串口2设备句柄 */
 rt_device_t uart_device2 = RT_NULL;//用于第一个串口 USART2

/* 串口3设备句柄 */
 rt_device_t uart_device3 = RT_NULL;//用于第一个串口 USART3

	
/* 回调函数 ，当串口1产生中断时会调用这个回调函数*/
static rt_err_t uart1_intput(rt_device_t dev, rt_size_t size)
{
    /* 发送事件 */
		if(uart1_Init_Finsh==1)//防止未初始化event事件，就有数据接收产生中断，造成rt_event_send出现空事件造成错误
		{
			rt_event_send(&event1, UART1_RX_EVENT);
			return RT_EOK;
		}
		
}
/* 回调函数 ，当串口2产生中断时会调用这个回调函数*/
static rt_err_t uart2_intput(rt_device_t dev, rt_size_t size)
{
    /* 发送事件 */
	if(uart2_Init_Finsh==1)//防止未初始化event事件，就有数据接收产生中断，造成rt_event_send出现空事件造成错误
  { 
		rt_event_send(&event2, UART2_RX_EVENT);
    return RT_EOK;
	}
	return RT_ERROR;
}
/* 回调函数 ，当串口3产生中断时会调用这个回调函数*/
static rt_err_t uart3_intput(rt_device_t dev, rt_size_t size)
{
	  /* 发送事件 */
	if(uart3_Init_Finsh==1)//防止未初始化event事件，就有数据接收产生中断，造成rt_event_send出现空事件造成错误
 { 
		rt_event_send(&event3, UART3_RX_EVENT);
    return RT_EOK;
	}
	return RT_ERROR;
}

/***********串口1获取8位数据***************/
rt_uint8_t uart1_getchar(void)
{
    rt_uint32_t e;
     rt_uint8_t ch;

    /* 读取1字节数据 */
    while (rt_device_read(uart_device1, 0, &ch,1) != 1)
    {
         /* 接收事件 */
        rt_event_recv(&event1, UART1_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);

		}
		return ch;
}
/******串口1获取多个字节数据**********函数指针应用*/
rt_uint8_t  uart_getchar_len(rt_device_t dev, uint8_t *ch)
{
    rt_uint32_t e;

    /* 读取1字节数据 */
    while (rt_device_read(uart_device1, -1, &ch,1 )!= 1)
    {
         /* 接收事件 */
        rt_event_recv(&event1, UART1_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);
				
				
			
		}
		return RT_EOK;
}
/***********串口2获取8位数据***************/
rt_uint8_t uart2_getchar(void)
{
    rt_uint32_t e;
     rt_uint8_t ch;
    /* 读取1字节数据 */
    while (rt_device_read(uart_device2, 0, &ch, 1) != 1)
    {
         /* 接收事件 */
				
        rt_event_recv(&event2, UART2_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);
				

			
		}
	return ch;
}
/***********串口3获取8位数据***************/
rt_uint8_t uart3_getchar(void)
{
    rt_uint32_t e;
    rt_uint8_t ch;
		//rt_device_read(uart_device3, 0, &ch1, 1);
    /* 读取1字节数据 */
    while (rt_device_read(uart_device3, 0, &ch,1) != 1)
    {
         /* 接收事件 */
        rt_event_recv(&event3, UART3_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);
				
				
			
		}
		return ch;
}

/***********串口1发送8位数据***************/
void uart1_putchar(const rt_uint8_t c)
{
	
    rt_size_t len = 0;
    rt_uint32_t timeout = 0;

	
    do
    {
        len = rt_device_write(uart_device1, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
}

/***********串口2发送8位数据***************/
void uart2_putchar(const rt_uint8_t c)
{
		
    rt_size_t len = 0;
    rt_uint32_t timeout = 0;

    do
    {
        len = rt_device_write(uart_device2, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
}
/***********串口3发送8位数据***************/
void uart3_putchar(const rt_uint8_t c)
{
	
	 rt_size_t len = 0;
    rt_uint32_t timeout = 0;

    do
    {
        len = rt_device_write(uart_device3, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
	
}

/***********串口1发送字符串***************/
void uart1_putstring(const char *s)
{
    while(*s)
    {
        uart1_putchar(*s++);
    }
}
/***********串口1发送字符串***************/
void uart2_putstring( char *s)
{
    while(*s)
    {
        uart2_putchar(*s++);
    }
}

/***********串口1发送字符串***************/
void uart3_putstring( char *s)
{
    while(*s)
    {
        uart3_putchar(*s++);
    }
}

/***********打开串口1***************/
rt_err_t uart1_open(const char *name)
{
    rt_err_t res;

    /* 查找系统中的串口设备 */
    uart_device1= rt_device_find(name);
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    /* 查找到设备后将其打开 */
    if (uart_device1 != RT_NULL)
    {   
       
        res = rt_device_set_rx_indicate(uart_device1, uart1_intput);//rt_device_set_rx_indicate是当串口收到数据时，通知上层应用线程有数据到达 ，并调用回调函数uart_input(这个是用户自己定义）
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n",name,res);
            return -RT_ERROR;
        }
					/* step2：修改串口配置参数 */
				config.baud_rate = 100000;        //修改波特率为100k
				config.data_bits = DATA_BITS_8;           //数据位 8
				config.stop_bits = STOP_BITS_1;           //停止位 1
				config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
				config.parity    = PARITY_EVEN;           //偶校验位

				/* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
				rt_device_control(uart_device1, RT_DEVICE_CTRL_CONFIG, &config);
        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device1, RT_DEVICE_FLAG_DMA_RX);       
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("open %s device error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 初始化事件对象 */
        rt_event_init(&event1, "event1", RT_IPC_FLAG_FIFO ); 
				uart1_Init_Finsh=1;//创建事件对象和rt_event_create一样，初始化一个事件对象//i防止未初始化event事件，就有数据接收产生中断，造成rt_event_send出现空事件造成错误
    }
    else
    {
        rt_kprintf("can't find %s device.\n",name);
        return -RT_ERROR;
    }

    return RT_EOK;
}
/***********打开串口2***************/
rt_err_t uart2_open(const char *name)
{
    rt_err_t res;

    /* 查找系统中的串口设备 */
    uart_device2= rt_device_find(name);   
    /* 查找到设备后将其打开 */
    if (uart_device2 != RT_NULL)
    {   
       
        res = rt_device_set_rx_indicate(uart_device2, uart2_intput);//rt_device_set_rx_indicate是当串口收到数据时，通知上层应用线程有数据到达 ，并调用回调函数uart_input(这个是用户自己定义）
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device2,  RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX );       
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("open %s device error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 初始化事件对象 */
        rt_event_init(&event2, "event2", RT_IPC_FLAG_FIFO); //创建事件对象和rt_event_create一样
				uart2_Init_Finsh=1;
    }
    else
    {
        rt_kprintf("can't find %s device.\n",name);
        return -RT_ERROR;
    }

    return RT_EOK;
}/***********打开串口1***************/
rt_err_t uart3_open(const char *name)
{
    rt_err_t res;
		
    /* 查找系统中的串口设备 */
    uart_device3= rt_device_find(name); 
		
    /* 查找到设备后将其打开 */
    if (uart_device3 != RT_NULL)
    {   
      
        res = rt_device_set_rx_indicate(uart_device3, uart3_intput);//rt_device_set_rx_indicate是当串口收到数据时，通知上层应用线程有数据到达 ，并调用回调函数uart_input(这个是用户自己定义）
        /* 检查返回值 */
        if (res != RT_EOK)
        {	
            rt_kprintf("set %s rx indicate error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );       
        /* 检查返回值 */
        if (res != RT_EOK)
        {
					
            rt_kprintf("open %s device error.%d\n",name,res);
					
            return -RT_ERROR;
        }
				
        /* 初始化事件对象 */
        res=rt_event_init(&event3, "event3", RT_IPC_FLAG_FIFO); //创建事件对象和rt_event_create一样
					
				uart3_Init_Finsh=1;
			
    }
    else
    {
        rt_kprintf("can't find %s device.\n",name);
        return -RT_ERROR;
    }

    return RT_EOK;
}
///**************************************************************/
//rt_uint8_t ch[4];
/*******用于把ASCII转换为十进制的整数
j表示输入的数据，j表示**************/
/*float shiji(uint8_t i,uint8_t j)
{
	uint8_t i1=0;
	float kk1=1.0f;
	for(i1=j-1;i1>0;i1--)
	{
		kk1*=10.0f;
	}
	kk1*=i;
	return kk1;
}*/
/*******用于把ASCII转换为十进制的小数，i表示输入的数据数组，j表示离小数点的位置**************/
//float Ascii_Xiaoshu(uint8_t i,uint8_t j)
//{
//	uint8_t i1=0;
//	float kk1=1.0f;
//	for(i1=0;i1<j;i1++)
//	{
//		kk1/=10.0f;
//	}
//	kk1*=i;
//	return kk1;
//}


/***********用于ASCII码转换为整数******************/
//float Get_AutoRun(uint8_t *kk,uint8_t j)
//{
//	uint8_t i=0;
//	uint8_t b=0;
//	float total=0.0f;
////	if(j==0)
////	{
////		return() ;//当只有11个数据时，表示没有数字输入，数据保持不变
////	}else{
//		for(i=0;i<j;i++)//EE B1 11 00 00 00 09 11 00 FF FC FF FF 
//		{
//			
//			if(kk[i]==0x2e)
//			{
//				
//				b=i;
//				break;
//			}else{
//				
//				b=0;
//			}
//		}
//			if(b>0)//当存在小数点时
//			{
//				
//				for(i=0;i<b;i++)
//				{
//					
//					total+=shiji(kk[i]-0x30,b-i);//获得整数部分,b表示小数点位置的坐标，
//					
////					uart1_putchar(total);
//				}
//				
//				
//				for(i=b+1;i<j;i++)
//				{
//					
//					total+=Ascii_Xiaoshu(kk[i]-0x30,i-b);
//					
//				}
//				
//			}
//		if(b==0)
//		{
//				for(i=0;i<j;i++)
//				{
//					total+=(float)(shiji(kk[i]-0x30,j-i));//获得整数部分
//			}
//		}
//		
//	return total;
//		
//}
//	
