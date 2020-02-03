/*ָ�����
������λ����Ϣ
1   2  3  4  5  6  7  8 10 11 12 13 14 15 16
EE FE 02 00 00 00 02 02 00 00 00 00 00 FD FC
EE FEΪͷ֡
FD FCΪβ֡
������Ϊ����ʶ��֡������01Ϊ���Ƶ���Ŀ��أ�02Ϊ���Ƶ�����ٶȣ�03Ϊ��ͷ��λ�������ݴ��䣬����������λ���ݣ�04Ϊ���ͨ�� 
01Ϊ����PWM,02Ϊ����PWM��05Ϊ������ÿ��2s��λ�������λ������һ�����ݣ���û�н��յ����ݣ�����ͨѶ�жϣ���λ�����ж���ֹͣ��
��������Ϊ01��02ʱ�����鵽��13��ֱ������1�����10;��Ϊ03ʱ�����ʱ������������ݣ�����ʹ�ù�����������


����λ��������Ϣ


EE FE 05 00 00 00 02 02 00 00 00 00 00 FD FC
EE FEΪͷ֡
FD FCΪβ֡
���������ݣ�05  ������λ������Ϣ���� �������ݣ� 01 ���1����  ��02 ���2���� ��03 ���3���� ��04 ���4������05 �ź�����������
����������: 06  ������λ���ĵ���ٶ���Ϣ  �������ݣ�01 ���1�ٶ�  ��02 ���2�ٶ� ��03���3�ٶȡ�04���4�ٶ� 
*/
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/rt_drv_pwm.h>
#include "app_uSart.h"

static rt_thread_t Communicate_Task=RT_NULL;//����433ģ��Ķ�̬�߳�
static rt_thread_t Send_Task=RT_NULL;//����λ����������
rt_sem_t G_Uart3_Get=RT_NULL;
rt_sem_t G_Uart3_Send=RT_NULL;//�����ź��õ��ź���


uint8_t G_Comman[15];
uint8_t G_Send[21]={0XEE,0XFE,0X01,0X00,
											00,00,00,00,00,00,
											00,00,00,00,00,00,00,00,00,0XFD,0XFC};
/*********������λ������������*********************/
static void Uart3_Get_entry(void *parameter)
{
	uint8_t a=0;
	
	while(1)
	{
		uint8_t num=0;
		for(num=0;num<15;num++)
		{
			
			G_Comman[num]=uart3_getchar();
			if(G_Comman[0]!=0xee||(num==14&&(G_Comman[13]!=0XFd||G_Comman[14]!=0XFc)))//���������Ҫ���ͨѶ����
			{	
				num=0;
				break;
			}
		}
		if(num==15)
		{
			rt_pin_write(7,a);
			a=~a;
			rt_sem_release(G_Uart3_Get);//���ݽ��ճɹ����ͷ��ź��������ڵȴ������ݴ����߳̽��뼤��ģʽ
		}
	}
}

/*************����λ�����ͺ���***********************/
static void Uart3_Send_Entry(void * parameter)
{
	while(1)
	{
		uint8_t a=0;
		rt_sem_take(G_Uart3_Send,RT_WAITING_FOREVER);
		for(a=0;a<21  ;a++)
		{
			uart3_putchar(G_Send[a]);
			
		}	
	}
}
int Uart_Control(void)
{
	PWM_Set();
	pin_mode_define();
	uart3_open("uart3");
	rt_pin_mode(7,PIN_MODE_OUTPUT);
	rt_pin_write(7,1);
	
	G_Uart3_Get=rt_sem_create("Uart3_Get",0,RT_IPC_FLAG_FIFO);//�����ź���
	G_Uart3_Send=rt_sem_create("Uart3_Send",0,RT_IPC_FLAG_FIFO);//��������
	Communicate_Task=rt_thread_create("u3",
													Uart3_Get_entry,RT_NULL,/*�����߳�1*/
													1024,	
													1,10);
	Send_Task=rt_thread_create("u3_Send",
													Uart3_Send_Entry,RT_NULL,/*�����߳�1*/
													1024,	
													3,10);
	if(Communicate_Task!=RT_NULL)
		rt_thread_startup(Communicate_Task);
	if(Send_Task!=RT_NULL)
		rt_thread_startup(Send_Task);
	return 0;
}
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(Uart_Control);
#endif
