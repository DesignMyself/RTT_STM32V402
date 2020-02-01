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
*/


#include "board.h"
#include <rtthread.h>
#include "drv_spi.h"
#include <rtdevice.h>
#define W25Q_SPI_DEVICE_NAME     "spi10"//这里只能是spi10,因为使用的是spi1的第0人设备
struct rt_spi_device *spi_dev_w25q;
void SPI_Send_Entry(void *parameter)
{
	uint16_t w25x_read_id=0x90;
	uint8_t kk[5]={0x23,0x25,0x66,0x99,0x47};
		 
	while(1)
	{
		rt_spi_send(spi_dev_w25q,kk,5);
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
			
            struct rt_spi_configuration cfg;
            cfg.data_width = 8;
            cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
            cfg.max_hz = 2 * 1000 * 1000; /* 2M */
            rt_spi_configure(spi_dev_w25q, &cfg);//当调用这个的时候就会调节HAL_msp_c中的底程初始化
			
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
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(spi_w25q_sample);
#endif