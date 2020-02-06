/*

 * Copyright (c) 2006-2018, RT-Thread Development Team

 *

 * SPDX-License-Identifier: Apache-2.0

 *

 * Change Logs:

 * Date           Author       Notes

 * 2018-08-15     misonyo      first implementation.

 */

/*

 * 程序清单：这是一个 SPI 设备使用例程

 * 例程导出了 spi_w25q_sample 命令到控制终端

 * 命令调用格式：spi_w25q_sample spi10

 * 命令解释：命令第二个参数是要使用的SPI设备名称，为空则使用默认的SPI设备

 * 程序功能：通过SPI设备读取 w25q 的 ID 数据
* rt_spi_configure(spi_dev_w25q, &cfg);//当调用这个的时候就会调节HAL_msp_c中的底程初始化
rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_PIN_0);//必须把设备spi10注册到spi1上才可以使用spi接口，不然会发生错
*在使用驱动过程中发现在驱动drv.spi中会存在进入不了SLAVE模式的情况，鉴于这个原因，把驱动中的模式写死了，第96行
*spi中当使用从机模式时，需要使用主机的时钟CLK,还有从模式的CS引脚也可使用主机上的引脚
*注意rt_spi_send,rt_spi_transfer,rt_spi_rece......这几种发送方式得到的数据不一样
*/


#include "board.h"
#include <rtthread.h>
#include "drv_spi.h"
#include <rtdevice.h>
#define W25Q_SPI_DEVICE_NAME     "spi10"//这里只能是spi10,因为使用的是spi1的第0人设备
#define NRF_READ_REG    0x00	//读配置寄存器，低5位为寄存器地址
static struct rt_semaphore spi1_rx_sem;     /* 用于接收消息的信号量 */
struct rt_spi_device *spi_dev_w25q;
uint8_t NRF24L01_Read_Reg( uint8_t RegAddr )
{
    uint8_t btmp,COMM;
		COMM=NRF_READ_REG | RegAddr;
		rt_spi_transfer(spi_dev_w25q,&COMM,RT_NULL,1);
			COMM=0XFF;
    rt_spi_transfer(spi_dev_w25q,&COMM,&btmp,1);
    			//读数据
    return btmp;
}
void SPI_Send_Entry(void *parameter)
{
	uint16_t w25x_read_id=0x90;
	uint8_t kk[5]={0x23,0x25,0x66,0x99,0x47};
	uint8_t jj[10]={0}; 
	uint8_t ll=0;
	while(1)
	{
//		rt_spi_transfer(spi_dev_w25q,kk,jj,5);
//	
//		if(jj[0]!=0)
//		{
//			rt_kprintf("接收到SPI数据\n");
//			for(uint8_t i=0;i<5;i++)
//			{
//				rt_kprintf("%x ",jj[i]);
//				
//			}
//			rt_kprintf("接收数据完毕\n");
//		}
		ll=NRF24L01_Read_Reg( 0x1D );
		rt_kprintf("ll的值是%x\n ",ll);	
		rt_thread_delay(500);
		
	}
	
	
}
void SPI_Open(const char* SPI_Device_name,const char* SPI_BUS_name)
{
	rt_hw_spi_device_attach(SPI_BUS_name, SPI_Device_name, GPIOB, GPIO_PIN_0);//必须把设备spi10注册到spi1上才可以使用spi接口，不然会发生错
    spi_dev_w25q = (struct rt_spi_device *)rt_device_find(SPI_Device_name);//找到spi设备各，此时

    if (!spi_dev_w25q)

    {
        rt_kprintf("spi sample run failed! can't find %s device!\n", SPI_Device_name);
    }
		else
		{
						uint8_t i=0;
            struct rt_spi_configuration cfg;
            cfg.data_width = 8;
           cfg.mode =RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 RT_SPI_MODE_0 | RT_SPI_MSB|*/
            cfg.max_hz = 8 * 1000 * 1000; /* 8M */
            i=rt_spi_configure(spi_dev_w25q, &cfg);//当调用这个的时候就会调节HAL_msp_c中的底程初始化
						if(RT_EOK==i)
						{
							rt_kprintf("配置成功\n");
						}
		}
		
	
}
static void spi_w25q_sample(int argc, char *argv[])

{
		rt_thread_t SPI_Send_thread;
    /* 创建数据接收线程 */
     SPI_Open("spi10","spi1");  
    /* 查找 spi 设备获取设备句柄 */

   
		SPI_Send_thread = rt_thread_create("spi_send", SPI_Send_Entry, RT_NULL, 1024, 10, 10);
    if (SPI_Send_thread != RT_NULL)
    {
        rt_thread_startup(SPI_Send_thread);
    }
}
//#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
//    INIT_APP_EXPORT(spi_w25q_sample);
//#endif