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

 * �����嵥������һ�� SPI �豸ʹ������

 * ���̵����� spi_w25q_sample ��������ն�

 * ������ø�ʽ��spi_w25q_sample spi10

 * ������ͣ�����ڶ���������Ҫʹ�õ�SPI�豸���ƣ�Ϊ����ʹ��Ĭ�ϵ�SPI�豸

 * �����ܣ�ͨ��SPI�豸��ȡ w25q �� ID ����
* rt_spi_configure(spi_dev_w25q, &cfg);//�����������ʱ��ͻ����HAL_msp_c�еĵ׳̳�ʼ��
rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_PIN_0);//������豸spi10ע�ᵽspi1�ϲſ���ʹ��spi�ӿڣ���Ȼ�ᷢ����
*��ʹ�����������з���������drv.spi�л���ڽ��벻��SLAVEģʽ��������������ԭ�򣬰������е�ģʽд���ˣ���96��
*spi�е�ʹ�ôӻ�ģʽʱ����Ҫʹ��������ʱ��CLK
*/


#include "board.h"
#include <rtthread.h>
#include "drv_spi.h"
#include <rtdevice.h>
#define W25Q_SPI_DEVICE_NAME     "spi10"//����ֻ����spi10,��Ϊʹ�õ���spi1�ĵ�0���豸
static struct rt_semaphore spi1_rx_sem;     /* ���ڽ�����Ϣ���ź��� */
struct rt_spi_device *spi_dev_w25q;

void SPI_Send_Entry(void *parameter)
{
	uint16_t w25x_read_id=0x90;
	uint8_t kk[5]={0x23,0x25,0x66,0x99,0x47};
	uint8_t jj[5]={0}; 
	while(1)
	{
		rt_spi_transfer(spi_dev_w25q,kk,jj,5);//RT_SPI�����ֲ�ͬ�ķ��ͷ�ʽ��Ҫע������
	
		if(jj[0]!=0)
		{
			rt_kprintf("���յ�SPI����\n");
			for(uint8_t i=0;i<5;i++)
			{
				rt_kprintf("%x ",jj[i]);
				
			}
			rt_kprintf("�����������\n");
		}
		rt_thread_delay(500);
		
	}
	
	
}
void SPI_Open(const char* SPI_Device_name,const char* SPI_BUS_name)
{
	rt_hw_spi_device_attach(SPI_BUS_name, SPI_Device_name, GPIOB, GPIO_PIN_0);//������豸spi10ע�ᵽspi1�ϲſ���ʹ��spi�ӿڣ���Ȼ�ᷢ����
    spi_dev_w25q = (struct rt_spi_device *)rt_device_find(SPI_Device_name);//�ҵ�spi�豸������ʱ

    if (!spi_dev_w25q)

    {
        rt_kprintf("spi sample run failed! can't find %s device!\n", SPI_Device_name);
    }
		else
		{
						uint8_t i=0;
            struct rt_spi_configuration cfg;
            cfg.data_width = 8;
            cfg.mode =RT_SPI_SLAVE| RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 RT_SPI_MODE_0 | RT_SPI_MSB|*/
            cfg.max_hz = 2 * 100 * 100; /* 2M */
            i=rt_spi_configure(spi_dev_w25q, &cfg);//�����������ʱ��ͻ����HAL_msp_c�еĵ׳̳�ʼ��
						if(RT_EOK==i)
						{
							rt_kprintf("���óɹ�\n");
						}
		}
		
	
}
static void spi_w25q_sample(int argc, char *argv[])

{
		rt_thread_t SPI_Send_thread;
    /* �������ݽ����߳� */
     SPI_Open("spi10","spi1");  
    /* ���� spi �豸��ȡ�豸��� */

   
		SPI_Send_thread = rt_thread_create("spi_send", SPI_Send_Entry, RT_NULL, 1024, 10, 10);
    if (SPI_Send_thread != RT_NULL)
    {
        rt_thread_startup(SPI_Send_thread);
    }
}
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(spi_w25q_sample);
#endif