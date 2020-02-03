/*指令解释
接收上位机信息
1   2  3  4  5  6  7  8 10 11 12 13 14 15 16
EE FE 02 00 00 00 02 02 00 00 00 00 00 FD FC
EE FE为头帧
FD FC为尾帧
第三组为功能识别帧：其中01为控制电机的开关，02为控制电机的速度，03为接头上位机的数据传输，看倒数第三位数据，04为电机通电 
01为左轮PWM,02为右轮PWM，05为心跳（每隔2s上位机会给下位机发送一组数据，若没有接收到数据，表明通讯中断，下位机所有动作停止）
当第三组为01或02时第四组到第13组分别代表电机1到电机10;当为03时代表的时输出过来的数据，数据使用共用体来传输


向上位机发送信息


EE FE 05 00 00 00 02 02 00 00 00 00 00 FD FC
EE FE为头帧
FD FC为尾帧
第三个数据：05  接受下位机的信息传递 第四数据： 01 电机1死机  、02 电机2死机 、03 电机3死机 、04 电机4死机、05 信号在正常工作
第三个数据: 06  接收下位机的电机速度信息  第四数据：01 电机1速度  、02 电机2速度 、03电机3速度、04电机4速度 
*/
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/rt_drv_pwm.h>
#include "app_uSart.h"

static rt_thread_t Communicate_Task=RT_NULL;//创建433模块的动态线程
static rt_thread_t Send_Task=RT_NULL;//向上位机发送数据
rt_sem_t G_Uart3_Get=RT_NULL;
rt_sem_t G_Uart3_Send=RT_NULL;//发送信号用的信号量


uint8_t G_Comman[15];
uint8_t G_Send[21]={0XEE,0XFE,0X01,0X00,
											00,00,00,00,00,00,
											00,00,00,00,00,00,00,00,00,0XFD,0XFC};
/*********接收上位机数据任务函数*********************/
static void Uart3_Get_entry(void *parameter)
{
	uint8_t a=0;
	
	while(1)
	{
		uint8_t num=0;
		for(num=0;num<15;num++)
		{
			
			G_Comman[num]=uart3_getchar();
			if(G_Comman[0]!=0xee||(num==14&&(G_Comman[13]!=0XFd||G_Comman[14]!=0XFc)))//清除不符合要求的通讯数据
			{	
				num=0;
				break;
			}
		}
		if(num==15)
		{
			rt_pin_write(7,a);
			a=~a;
			rt_sem_release(G_Uart3_Get);//数据接收成功后释放信号量，让在等待的数据处理线程进入激活模式
		}
	}
}

/*************向上位机发送函数***********************/
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
	
	G_Uart3_Get=rt_sem_create("Uart3_Get",0,RT_IPC_FLAG_FIFO);//创建信号量
	G_Uart3_Send=rt_sem_create("Uart3_Send",0,RT_IPC_FLAG_FIFO);//创建互斥
	Communicate_Task=rt_thread_create("u3",
													Uart3_Get_entry,RT_NULL,/*创建线程1*/
													1024,	
													1,10);
	Send_Task=rt_thread_create("u3_Send",
													Uart3_Send_Entry,RT_NULL,/*创建线程1*/
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
