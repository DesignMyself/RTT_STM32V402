/*
 * 程序清单：这是一个 CAN 设备使用例程
 * 例程导出了 can_sample 命令到控制终端
 * 命令调用格式：can_sample can1
 * 命令解释：命令第二个参数是要使用的 CAN 设备名称，为空则使用默认的 CAN 设备
 * 程序功能：通过 CAN 设备发送一帧，并创建一个线程接收数据然后打印输出。
 使用时要注意以下几项：1.在stm32f4xx_hal_conf.h中开启相应的驱动文件，例如#define HAL_ADC_MODULE_ENABLED
											2.在stm32f4xx_hal_msp.c中进行相应的配置 ，例如void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
											3.如果注册名称不成功，应该看看是不是缺少什么#define ,例如#define BSP_USING_CAN
											4.设置用标准数据时用filterConf.FilterIdHigh = (((uint32_t)0x100<<21)&0xFFFF0000)>>16;		
											filterConf.FilterIdLow =(((uint32_t)0x100<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
											5.设置用扩展数据时filterConf.FilterIdHigh = (((uint32_t)0x100<<3)&0xFFFF0000)>>16;		
												filterConf.FilterIdLow =(((uint32_t)0x100<<3)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
*/

#include <rtthread.h>
#include "rtdevice.h"
#include "drv_can.h"
#include "app_uSart.h"
#define CAN_DEV_NAME       "can1"      /* CAN 设备名称 */
extern uint32_t num;
static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */
static rt_device_t can_dev;            /* CAN 设备句柄 */
static uint8_t data1[8]={0x12,0x33,0x33,0x36,0x99,0x77,0x33,0x66};
static uint8_t data2[8]={0x19,0x39,0x35,0x56,0x19,0x07,0x35,0x86};
static uint8_t data3[8]={0x89,0x89,0x85,0x86,0x89,0x87,0x85,0x86};
static uint8_t data4[8]={0xa9,0xa9,0xa5,0xa6,0xa9,0xa7,0xa5,0xa6};
/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
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
		msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len =8 ;            
    /* 待发送的 8 字节数据 */
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
    /* 发送一帧 CAN 数据 */
  
		
	}
	
	
}
static void can_rx_thread(void *parameter)
{
    int i;
    struct rt_can_msg rxmsg = {0};
		
    while (1)
    {
        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
       rxmsg.hdr = -1;
        /* 阻塞等待接收信号量 */
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
        /* 打印数据 ID 及内容 */
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
	
		/* 以中断接收及发送方式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
		rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    RT_ASSERT(res == RT_EOK);
		
		
	
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
#ifdef RT_CAN_USING_HDR
		rt_kprintf("come into can_HDR\n");
    struct rt_can_filter_item items[3] =
 {
				
	 //RT_CAN_FILTER_ITEM_INIT(id,ide,rtr,mode,mask,blank,ind,args) 
				RT_CAN_FILTER_ITEM_INIT(0x100, 1, 0, 1, 0x01,0, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表,BLANK为过滤数组 */
        RT_CAN_FILTER_ITEM_INIT(0x1314, 1, 0, 1, 0x01,1, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x211, 1, 0, 1, 0x01,2, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
        
    };
    struct rt_can_filter_config cfg = {3, 1, items}; /* 一共有 5 个过滤表 */
		rt_kprintf("cfgis %d\n");
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
		RT_ASSERT(res == RT_EOK);
#endif

		 /* 初始化 CAN 接收信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
		rt_kprintf("come into can_sem\n");
    return res;
		
    
}
int can_sample(void)
{
    
    

    rt_thread_t Can_Receive_thread;
		rt_thread_t Can_Send_thread;
		CAN_Open("can1");

    /* 创建数据接收线程 */
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
		
    rt_kprintf("num 的值是%x\n",num);

    return 0;
}

#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(can_sample);
#endif