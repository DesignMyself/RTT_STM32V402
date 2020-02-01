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
											6.使用时注意波特率的设置，最好使用逻辑分析来看看是否正确，在407vet6中把{CAN1MBaud, (CAN_SJW_2TQ | CAN_BS1_9TQ  | CAN_BS2_5TQ | 3)},中的CAN_BS2_5TQ的值由4u改为了3u
											
											
*/

#include <rtthread.h>
#include "rtdevice.h"
#include "drv_can.h"
#include "app_uSart.h"
#include "pid.h"
#include "HDMI.H"
#define CAN_DEV_NAME       "can1"      /* CAN 设备名称 */
/* USER CODE BEGIN Variables */
typedef __packed struct{
  float position;
  int16_t w;//速度值，单位R/MIN
  int16_t current;
  int8_t temperature;
}MotoInfo_t;
MotoInfo_t MotoInfo[4];
		 Pid_t WheelPid;
	Pid_t WheelPid2;
float P_Pre=0;
uint8_t CAN_Sign=0;
float Circle=5;
float distance=0.00;
float sum=0;//用增量式统计圈数
float first_P=0;
uint8_t end_run=1;
/* USER CODE END Variables */
static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */
static struct rt_semaphore pid_sem;//用于处理PID
static rt_device_t can_dev;            /* CAN 设备句柄 */
int16_t MotoCurrent = 0;//电机ID1电流值 
int16_t MotoCurrent2=0;//电机ID2电流值 
extern uint8_t PWM_Sign;
extern int16_t speed_set;//速度设置
/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
		
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
    msg.len =0x08 ;            
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
/********CAN发送函数用于向电机发送电流参数*******************************/
uint8_t SetMoto( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  uint8_t size=0;
	struct rt_can_msg msg;
	msg.id = 0x200;
	msg.ide = CAN_ID_STD;//标准格式
	msg.rtr= CAN_RTR_DATA;//发送数据
	msg.len = 0x08;//数据长度
	msg.data[0] = (iq1 >> 8);
	msg.data[1] = iq1;
	msg.data[2] = (iq2 >> 8);
	msg.data[3] = iq2;
	msg.data[4] = iq3 >> 8;
	msg.data[5] = iq3;
	msg.data[6] = iq4 >> 8;
	msg.data[7] = iq4;
	size=rt_device_write(can_dev, 0, &msg, sizeof(msg));
	return size;
}
/**********CAN发送线程，用于向电机发送电流参数*****************/
static void can_send_thread(void *parameter)
{

	   PidInit(&WheelPid, DELTA_PID, 10000, 5000,1.5, 0.1, 0.0);
	//PidInit(&WheelPid2, POSITION_PID, 6000, 1500, 1.5, 0.2, 0.0);
	rt_thread_mdelay(1000);
	while(1)
	{
			
    /* 发送一帧 CAN 数据 */
		rt_thread_mdelay(10);
		if(CAN_Sign==1)//增加反馈判断，如果没有反馈就不能进行PID
		{
				MotoCurrent = PidCalc(&WheelPid, MotoInfo[0].w,speed_set);
				if(end_run==0)
				{
					MotoCurrent=0;
					speed_set=0;
					MotoInfo[0].w=0;
					WheelPid.delta_out=0;//防止受到阻止停止时出现突然加速
					WheelPid.last_delta_out=0;//防止受到阻止停止时出现突然加速
					
				}
				CAN_Sign=0;
		}
		//MotoCurrent2 = PidCalc(&WheelPid2, MotoInfo[1].w, speed_set);
		SetMoto(MotoCurrent,0, 0, 0);
		
		
	}
	
	
}

/********获取电机的CAN参数,位置，速度，电流，温度*******************/
static void can_rx_thread(void *parameter)
{
    float i;
    struct rt_can_msg rxmsg = {0};
		
	
    while (1)
    {
//        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
//       rxmsg.hdr = -1;
        /* 阻塞等待接收信号量 */
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));

		/* 处理得到的数据*/
		if (((rxmsg.id>0x200) && (rxmsg.id<0x205)) && (rxmsg.ide== CAN_ID_STD) && (rxmsg.len== 8))
		{
			MotoInfo[rxmsg.id-0x200-1].position = ((rxmsg.data[0]<<8) | rxmsg.data[1])*360.00/8191.00;
			MotoInfo[rxmsg.id-0x200-1].w = ((rxmsg.data[2]<<8) | rxmsg.data[3]);
			MotoInfo[rxmsg.id-0x200-1].current = ((rxmsg.data[4]<<8) | rxmsg.data[5]);
			MotoInfo[rxmsg.id-0x200-1].temperature = rxmsg.data[6];
		}
		CAN_Sign=1;
			if(PWM_Sign==0X08)
		{		
				distance=Circle*19.156*360;
				speed_set=800;
				if(0==sum)//为了得到初始位置
				{
					P_Pre=MotoInfo[0].position;//得到初始的位置
					first_P=P_Pre;
					sum=1;
				}
				else//当不是初始位置后
				{
						float i=0.00;
						
					if(MotoInfo[0].w>=0)//当电流值大于0的时候
						{
						
		
							i=MotoInfo[0].position-P_Pre;//
						
						}	
					else
						{
					
							i=P_Pre-MotoInfo[0].position;
						
						}
							P_Pre=MotoInfo[0].position;
						
					if(i<0)
						{
							i=360+i;
							if(i>=330)//采取这个作为屏蔽是因为这个地方会突然出现很大的数值，具体原因还没找到
							{
								i=3;
							}
							if(i>=50)
							{
								send_string(1,"wrong happen is:");
								Value_Asii(1,MotoInfo[0].position-P_Pre);
								send_string(1,"\n");
									
							}
						}
						sum+=i;
					
						if((sum)>=(distance))
							{
											end_run=0;
											speed_set=0;
											PWM_Sign=0XFF;
										}
				}

			}
			
		
    }
}

static void Para_Display(void *parameter)
{
	float speed=0;
	while(1)
	{
		speed=MotoInfo[0].w/19.00f;;
		send_string(1,"当前速度是");
		Value_Asii(1,MotoInfo[0].w);
		send_string(1,"\n");
//		rt_kprintf("当前速度是%d\n",speed);
		send_string(1,"当前电流值是");
		Value_Asii(1,MotoInfo[0].current);
		send_string(1,"\n");
//		rt_kprintf("PID后的电流值是%d\n",WheelPid-);
//		rt_kprintf("当前位置是%d\n",MotoInfo[0].position);
		send_string(1,"当前位置是");
		Value_Asii(1,MotoInfo[0].position);
		send_string(1,"\n");
		send_string(1,"sum is");
		Value_Asii(1,sum);
		send_string(1,"\n");
		//rt_kprintf("当前圈数是%d\n",Circle);
		send_string(1,"Circle is");
		Value_Asii(1,Circle);
		send_string(1,"\n");
//		rt_kprintf("计数位置是%d\n",first_P);
		send_string(1,"计数始位");
		Value_Asii(1,first_P);
		send_string(1,"\n");
		rt_kprintf("**********************\n");
		rt_thread_mdelay(500);
		
		
		
	}
	
	
	
}
rt_err_t  CAN_Open(const char* name)//打开CAN口的函数
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
		rt_device_control(can_dev,RT_CAN_CMD_SET_BAUD,(void*)CAN1MBaud);
//		rt_device_control(can_dev,RT_CAN_CMD_SET_BAUD,(void*)CAN1MBaud);
    RT_ASSERT(res == RT_EOK);
		
		
	
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
#ifdef RT_CAN_USING_HDR
		rt_kprintf("come into can_HDR\n");
    struct rt_can_filter_item items[5] =
 {
				
	 //RT_CAN_FILTER_ITEM_INIT(id,ide,rtr,mode,mask,blank,ind,args) 
				RT_CAN_FILTER_ITEM_INIT(0x200, 1, 0, 1, 0x00,0, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表,BLANK为过滤数组 */
        RT_CAN_FILTER_ITEM_INIT(0x201, 1, 0, 1, 0x00,1, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x202, 1, 0, 1, 0x00,2, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
				RT_CAN_FILTER_ITEM_INIT(0x203, 1, 0, 1, 0x00,3, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
				RT_CAN_FILTER_ITEM_INIT(0x204, 1, 0, 1, 0x00,4, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
        
    };
    struct rt_can_filter_config cfg = {5, 1, items}; /* 一共有 5 个过滤表 */
		rt_kprintf("cfgis %d\n");
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
		RT_ASSERT(res == RT_EOK);
#endif

		 /* 初始化 CAN 接收信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
		rt_sem_init(&pid_sem,"pid_sem",0,RT_IPC_FLAG_FIFO);
		rt_kprintf("come into can_sem\n");
    return res;
		
    
}
int can_sample(void)
{
    
    

    rt_thread_t Can_Receive_thread;
		rt_thread_t Can_Send_thread;
		rt_thread_t Para_Display_thread;
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
		Para_Display_thread= rt_thread_create("Para_Display", Para_Display, RT_NULL, 1024, 6, 10);
    if (Para_Display_thread != RT_NULL)
    {
        rt_thread_startup(Para_Display_thread);
    }
    else
    {
        rt_kprintf("create can_sx thread failed!\n");
    }

    return 0;
}

#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(can_sample);
#endif