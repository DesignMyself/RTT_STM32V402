/*
 * �����嵥������һ�� CAN �豸ʹ������
 * ���̵����� can_sample ��������ն�
 * ������ø�ʽ��can_sample can1
 * ������ͣ�����ڶ���������Ҫʹ�õ� CAN �豸���ƣ�Ϊ����ʹ��Ĭ�ϵ� CAN �豸
 * �����ܣ�ͨ�� CAN �豸����һ֡��������һ���߳̽�������Ȼ���ӡ�����
 ʹ��ʱҪע�����¼��1.��stm32f4xx_hal_conf.h�п�����Ӧ�������ļ�������#define HAL_ADC_MODULE_ENABLED
											2.��stm32f4xx_hal_msp.c�н�����Ӧ������ ������void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
											3.���ע�����Ʋ��ɹ���Ӧ�ÿ����ǲ���ȱ��ʲô#define ,����#define BSP_USING_CAN
											4.�����ñ�׼����ʱ��filterConf.FilterIdHigh = (((uint32_t)0x100<<21)&0xFFFF0000)>>16;		
											filterConf.FilterIdLow =(((uint32_t)0x100<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
											5.��������չ����ʱfilterConf.FilterIdHigh = (((uint32_t)0x100<<3)&0xFFFF0000)>>16;		
												filterConf.FilterIdLow =(((uint32_t)0x100<<3)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
*/

#include <rtthread.h>
#include "rtdevice.h"
#include "drv_can.h"
#include "app_uSart.h"
#define CAN_DEV_NAME       "can1"      /* CAN �豸���� */
extern uint32_t num;
static struct rt_semaphore rx_sem;     /* ���ڽ�����Ϣ���ź��� */
static rt_device_t can_dev;            /* CAN �豸��� */
static uint8_t data1[8]={0x12,0x33,0x33,0x36,0x99,0x77,0x33,0x66};
static uint8_t data2[8]={0x19,0x39,0x35,0x56,0x19,0x07,0x35,0x86};
static uint8_t data3[8]={0x89,0x89,0x85,0x86,0x89,0x87,0x85,0x86};
static uint8_t data4[8]={0xa9,0xa9,0xa5,0xa6,0xa9,0xa7,0xa5,0xa6};
/* �������ݻص����� */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN ���յ����ݺ�����жϣ����ô˻ص�������Ȼ���ͽ����ź��� */
		rt_kprintf("come into can inturrept\n");
    rt_sem_release(&rx_sem);

    return RT_EOK;
}
rt_size_t  can_send(struct rt_can_msg msg,uint32_t id,uint8_t data[8])
{
		rt_size_t  size;
		msg.id=id;
	#ifdef CAN_EXT

		msg.ide = RT_CAN_EXTID; 
	#endif
	#ifdef CAN_STD
		msg.ide = RT_CAN_STDID; 
	#endif
		msg.ide = RT_CAN_STDID;     /* ��׼��ʽ */
    msg.rtr = RT_CAN_DTR;       /* ����֡ */
    msg.len =8 ;            
    /* �����͵� 8 �ֽ����� */
    msg.data[0] = data[0];
    msg.data[1] = data[1];
    msg.data[2] = data[2];
    msg.data[3] = data[3];
    msg.data[4] = data[4];
    msg.data[5] = data[5];
    msg.data[6] = data[6];
    msg.data[7] = data[7];
	 size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
	return size;

}
static void can_send_thread(void *parameter)
{
	uint8_t Can_Sign=0;
	 struct rt_can_msg msg = {0};
	     rt_size_t  size=0;
	while(1)
	{
		Can_Sign=uart1_getchar();
		if(Can_Sign==0x06)
		{
				
			 size=can_send(msg,0x100,data1);
			rt_kprintf("come into can 06 send%d\n",size);
			if (size == 0)
			{
					rt_kprintf("can dev write data failed!\n");
			}
		}
    if(Can_Sign==0x07)
		{
			 size=can_send(msg,0x1314,data2);
			if (size == 0)
			{
					rt_kprintf("can dev write data failed!\n");
			}
		}
     if(Can_Sign==0x08)
		{
			 size=can_send(msg,0x211,data3);
			if (size == 0)
			{
					rt_kprintf("can dev write data failed!\n");
			}
		}
		 if(Can_Sign==0x09)
		{
			 size=can_send(msg,0x486,data4);
			if (size == 0)
			{
					rt_kprintf("can dev write data failed!\n");
			}
		}
    /* ����һ֡ CAN ���� */
  
		
	}
	
	
}
static void can_rx_thread(void *parameter)
{
    int i;
    struct rt_can_msg rxmsg = {0};
		
    while (1)
    {
        /* hdr ֵΪ - 1����ʾֱ�Ӵ� uselist �����ȡ���� */
       rxmsg.hdr = -1;
        /* �����ȴ������ź��� */
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* �� CAN ��ȡһ֡���� */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
        /* ��ӡ���� ID ������ */
        rt_kprintf("ID:%4x ", rxmsg.id);
				rt_kprintf("ID:%d ", rxmsg.hdr);
        for (i = 0; i < 8; i++)
        {
            rt_kprintf("%2x ", rxmsg.data[i]);
        }

        rt_kprintf("\n");
    }
}
rt_err_t  CAN_Open(const char* name)
{
		rt_err_t res;
	  can_dev = rt_device_find(name);
    if (!can_dev)
    {
        rt_kprintf("find %s failed!\n", name);
        return RT_ERROR;
    }
	
		/* ���жϽ��ռ����ͷ�ʽ�� CAN �豸 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
		rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    RT_ASSERT(res == RT_EOK);
		
		
	
    /* ���ý��ջص����� */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
#ifdef RT_CAN_USING_HDR
		rt_kprintf("come into can_HDR\n");
    struct rt_can_filter_item items[3] =
 {
				
	 //RT_CAN_FILTER_ITEM_INIT(id,ide,rtr,mode,mask,blank,ind,args) 
				RT_CAN_FILTER_ITEM_INIT(0x100, 1, 0, 1, 0x01,0, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff��hdr Ϊ - 1������Ĭ�Ϲ��˱�,BLANKΪ�������� */
        RT_CAN_FILTER_ITEM_INIT(0x1314, 1, 0, 1, 0x01,1, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff��hdr Ϊ - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x211, 1, 0, 1, 0x01,2, RT_NULL, RT_NULL), /* std,match ID:0x211��hdr Ϊ - 1 */
        
    };
    struct rt_can_filter_config cfg = {3, 1, items}; /* һ���� 5 �����˱� */
		rt_kprintf("cfgis %d\n");
    /* ����Ӳ�����˱� */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
		RT_ASSERT(res == RT_EOK);
#endif

		 /* ��ʼ�� CAN �����ź��� */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
		rt_kprintf("come into can_sem\n");
    return res;
		
    
}
int can_sample(void)
{
    
    

    rt_thread_t Can_Receive_thread;
		rt_thread_t Can_Send_thread;
		CAN_Open("can1");

    /* �������ݽ����߳� */
    Can_Receive_thread = rt_thread_create("can_rx", can_rx_thread, RT_NULL, 1024, 5, 10);
    if (Can_Receive_thread != RT_NULL)
    {
        rt_thread_startup(Can_Receive_thread);
    }
		 Can_Send_thread = rt_thread_create("can_sx", can_send_thread, RT_NULL, 1024, 4, 10);
    if (Can_Send_thread != RT_NULL)
    {
        rt_thread_startup(Can_Send_thread);
    }
    else
    {
        rt_kprintf("create can_sx thread failed!\n");
    }
		
    rt_kprintf("num ��ֵ��%x\n",num);

    return 0;
}

#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(can_sample);
#endif