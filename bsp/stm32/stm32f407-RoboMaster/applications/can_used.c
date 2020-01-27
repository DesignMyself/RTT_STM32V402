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
											6.ʹ��ʱע�Ⲩ���ʵ����ã����ʹ���߼������������Ƿ���ȷ����407vet6�а�{CAN1MBaud, (CAN_SJW_2TQ | CAN_BS1_9TQ  | CAN_BS2_5TQ | 3)},�е�CAN_BS2_5TQ��ֵ��4u��Ϊ��3u
											
											
*/

#include <rtthread.h>
#include "rtdevice.h"
#include "drv_can.h"
#include "app_uSart.h"
#include "pid.h"
#include "HDMI.H"
#define CAN_DEV_NAME       "can1"      /* CAN �豸���� */
/* USER CODE BEGIN Variables */
typedef __packed struct{
  float position;
  int16_t w;//�ٶ�ֵ����λR/MIN
  int16_t current;
  int8_t temperature;
}MotoInfo_t;
MotoInfo_t MotoInfo[4];
		 Pid_t WheelPid;
	Pid_t WheelPid2;
float P_Pre=0;
uint8_t CAN_Sign=0;
float Circle=5;
float sum=0;//������ʽͳ��Ȧ��
float first_P=0;
/* USER CODE END Variables */
static struct rt_semaphore rx_sem;     /* ���ڽ�����Ϣ���ź��� */
static rt_device_t can_dev;            /* CAN �豸��� */
int16_t MotoCurrent = 0;//���ID1����ֵ 
int16_t MotoCurrent2=0;//���ID2����ֵ 
extern uint8_t PWM_Sign;
extern int16_t speed_set;//�ٶ�����
/* �������ݻص����� */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN ���յ����ݺ�����жϣ����ô˻ص�������Ȼ���ͽ����ź��� */
		
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
    msg.len =0x08 ;            
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
/********CAN���ͺ��������������͵�������*******************************/
uint8_t SetMoto( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  uint8_t size=0;
	struct rt_can_msg msg;
	msg.id = 0x200;
	msg.ide = CAN_ID_STD;
	msg.rtr= CAN_RTR_DATA;
	msg.len = 0x08;
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
/**********CAN�����̣߳������������͵�������*****************/
static void can_send_thread(void *parameter)
{

	   PidInit(&WheelPid, DELTA_PID, 10000, 5000,1.5, 0.1, 0.0);
	//PidInit(&WheelPid2, POSITION_PID, 6000, 1500, 1.5, 0.2, 0.0);
	rt_thread_mdelay(1000);
	while(1)
	{

    /* ����һ֡ CAN ���� */
		rt_thread_mdelay(10);
		if(CAN_Sign==1)//���ӷ����жϣ����û�з����Ͳ��ܽ���PID
		{
				MotoCurrent = PidCalc(&WheelPid, MotoInfo[0].w,speed_set);
				CAN_Sign=0;
		}
		//MotoCurrent2 = PidCalc(&WheelPid2, MotoInfo[1].w, speed_set);
		SetMoto(MotoCurrent,0, 0, 0);
		
		
	}
	
	
}

/********��ȡ�����CAN����,λ�ã��ٶȣ��������¶�*******************/
static void can_rx_thread(void *parameter)
{
    int i;
    struct rt_can_msg rxmsg = {0};
		
	
    while (1)
    {
//        /* hdr ֵΪ - 1����ʾֱ�Ӵ� uselist �����ȡ���� */
//       rxmsg.hdr = -1;
        /* �����ȴ������ź��� */
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* �� CAN ��ȡһ֡���� */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));

		/* ����õ�������*/
		if (((rxmsg.id>0x200) && (rxmsg.id<0x205)) && (rxmsg.ide== CAN_ID_STD) && (rxmsg.len== 8))
		{
			MotoInfo[rxmsg.id-0x200-1].position = ((rxmsg.data[0]<<8) | rxmsg.data[1])*360.0/8191.0;
			MotoInfo[rxmsg.id-0x200-1].w = ((rxmsg.data[2]<<8) | rxmsg.data[3]);
			MotoInfo[rxmsg.id-0x200-1].current = ((rxmsg.data[4]<<8) | rxmsg.data[5]);
			MotoInfo[rxmsg.id-0x200-1].temperature = rxmsg.data[6];
		}
		CAN_Sign=1;
			if(PWM_Sign==0X08)
		{		
				if(0==sum)//Ϊ�˵õ���ʼλ��
				{
					P_Pre=MotoInfo[0].position;//�õ���ʼ��λ��
					first_P=P_Pre;
					sum=1;
				}
				else//�����ǳ�ʼλ�ú�
				{
						float i=0;
						
					if(MotoInfo[0].w>=0)//������ֵ����0��ʱ��
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
							if(i>=300)//��ȡ�����Ϊ��������Ϊ����ط���ͻȻ���ֺܴ����ֵ������ԭ��û�ҵ�
							{
								i=3;
							}
//							send_string(1,"iֵ��");
//							Value_Asii(1,i);
//							send_string(1,"\n");
//							if(i>300)
//							{
//										send_string(1,"��һ��P��");
//										Value_Asii(1,P);
//										send_string(1,"\n");
//										send_string(1,"Pre��");
//										Value_Asii(1,P_Pre);
//										send_string(1,"\n");
//										send_string(1,"��ǰM1��");
//										Value_Asii(1,MotoInfo[0].position);
//										send_string(1,"\n");
//								
//							}
						}
						sum+=i;
					
						if((sum/360.0)>=(Circle*19))
										{
											speed_set=0;
											PWM_Sign=0XFF;
										}
				}
//										send_string(1,"��ǰȦ����");
//										Value_Asii(1,sum/360.0);
//										send_string(1,"\n");
//										send_string(1,"Pre��");
//										Value_Asii(1,P_Pre);
//										send_string(1,"\n");
//										send_string(1,"��ǰM1��");
//										Value_Asii(1,MotoInfo[0].position);
//										send_string(1,"\n");
//										send_string(1,"��ǰ��ֵ��");
//										Value_Asii(1,P_Pre-MotoInfo[0].position);
//										send_string(1,"\n");
//										rt_kprintf("*************\n");
//								//sum+=(P_Pre-MotoInfo[0].position);//pre:360....now:355
//										P_Pre=MotoInfo[0].position;
			}
		
    }
}

static void Para_Display(void *parameter)
{
	float speed=0;
	while(1)
	{
		speed=MotoInfo[0].w/19.00f;;
		send_string(1,"��ǰ�ٶ���");
		Value_Asii(1,speed);
		send_string(1,"\n");
//		rt_kprintf("��ǰ�ٶ���%d\n",speed);
		send_string(1,"��ǰ����ֵ��");
		Value_Asii(1,MotoInfo[0].current);
		send_string(1,"\n");
//		rt_kprintf("PID��ĵ���ֵ��%d\n",MotoCurrent);
//		rt_kprintf("��ǰλ����%d\n",MotoInfo[0].position);
		send_string(1,"��ǰλ����");
		Value_Asii(1,MotoInfo[0].position);
		send_string(1,"\n");
		send_string(1,"sum is");
		Value_Asii(1,sum);
		send_string(1,"\n");
		//rt_kprintf("��ǰȦ����%d\n",Circle);
		send_string(1,"Circle is");
		Value_Asii(1,Circle);
		send_string(1,"\n");
//		rt_kprintf("����λ����%d\n",first_P);
		send_string(1,"����ʼλ");
		Value_Asii(1,first_P);
		send_string(1,"\n");
		rt_kprintf("**********************\n");
		rt_thread_mdelay(500);
		
		
		
	}
	
	
	
}
rt_err_t  CAN_Open(const char* name)//��CAN�ڵĺ���
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
		rt_device_control(can_dev,RT_CAN_CMD_SET_BAUD,(void*)CAN1MBaud);
//		rt_device_control(can_dev,RT_CAN_CMD_SET_BAUD,(void*)CAN1MBaud);
    RT_ASSERT(res == RT_EOK);
		
		
	
    /* ���ý��ջص����� */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
#ifdef RT_CAN_USING_HDR
		rt_kprintf("come into can_HDR\n");
    struct rt_can_filter_item items[5] =
 {
				
	 //RT_CAN_FILTER_ITEM_INIT(id,ide,rtr,mode,mask,blank,ind,args) 
				RT_CAN_FILTER_ITEM_INIT(0x200, 1, 0, 1, 0x00,0, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff��hdr Ϊ - 1������Ĭ�Ϲ��˱�,BLANKΪ�������� */
        RT_CAN_FILTER_ITEM_INIT(0x201, 1, 0, 1, 0x00,1, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff��hdr Ϊ - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x202, 1, 0, 1, 0x00,2, RT_NULL, RT_NULL), /* std,match ID:0x211��hdr Ϊ - 1 */
				RT_CAN_FILTER_ITEM_INIT(0x203, 1, 0, 1, 0x00,3, RT_NULL, RT_NULL), /* std,match ID:0x211��hdr Ϊ - 1 */
				RT_CAN_FILTER_ITEM_INIT(0x204, 1, 0, 1, 0x00,4, RT_NULL, RT_NULL), /* std,match ID:0x211��hdr Ϊ - 1 */
        
    };
    struct rt_can_filter_config cfg = {5, 1, items}; /* һ���� 5 �����˱� */
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
		rt_thread_t Para_Display_thread;
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