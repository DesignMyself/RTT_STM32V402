#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/rt_drv_pwm.h>
#include <board.h>
#include "app_uSart.h"
#define KEY0			3
#define KEY1			2
#define PWM_priority 1//PWM任务优先级
#define PWM_stack_size 512//用来储存任务的栈大小
#define PWM_timeslice  5	//若进入轮询时的时间片
#define PWM_DEV_NAME        "pwm2"  /* PWM设备名称 */
#define Motor1_PWM					"pwm10"
#define PWM_DEV_CHANNEL     2      /* PWM通道 */
#define LED1_PA6	 GET_PIN(A,6)
#define Motor1_PWN_PIN  GET_PIN(B,8)		//PB8
struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */  
static rt_thread_t PWM_task=RT_NULL;//创建任务1的动态线程
static rt_thread_t LED_TASK=RT_NULL;//创建LED任务线程
rt_uint32_t period, pulse;

void PWM_Set()
{
		period = 200;    /* 周期为0.5ms，单位为纳秒ns 对于tim10来说单位是0.0005us*/
		pulse =period/2;          /* PWM脉冲宽度值，单位为纳秒ns */
		/* 查找设备 */
		pwm_dev = (struct rt_device_pwm *)rt_device_find("pwm2");
		/* 设置PWM周期和脉冲宽度 */
		rt_pwm_set(pwm_dev,4, period, pulse);//四个数值分别是哪个PWM,channel,周期，脉宽
		/* 使能设备 */
		rt_pwm_enable(pwm_dev,4);
		
		/* 关闭设备通道 */
	
}

void PWM_Control_Entry(void *parameter)
{
	uint32_t PWM_Rate=200;//pwm值
	uint8_t PWM_Sign=0;
	PWM_Set();
	
	//PWM_Control_Key();
	while(1)
	{
		PWM_Sign=uart1_getchar();
		
		if(0X01==PWM_Sign)
		{
			rt_kprintf("PWM增加\n");
			PWM_Rate+=100;
		}
		if(0X02==PWM_Sign)
		{
			PWM_Rate-=100;
			rt_kprintf("PWM减少\n");
			if(PWM_Rate<=200)
			{
				PWM_Rate=200;
			}
		}
			rt_pwm_set(pwm_dev ,4,PWM_Rate, PWM_Rate/2);
		//rt_pwm_set(pwm_dev,1,500000,100000);
		//	rt_pwm_set(pwm_dev,2,500000,200000);
		//	rt_pwm_set(pwm_dev,3,500000,300000);
		
	}
}
void LED_Entry(void *parameter)
{
	
	uint8_t LED_SIGN=0;
	while(1)
	{
		rt_pin_write(LED1_PA6,LED_SIGN);
		LED_SIGN=~LED_SIGN;
		rt_thread_mdelay(1000);
		
	}
	
	
	
}

int PWM_Control(void)
{
	rt_pin_mode(LED1_PA6,PIN_MODE_OUTPUT);
	uart1_open("uart1");
	/*创建线程1*/
	PWM_task=rt_thread_create("PWM_task",
													PWM_Control_Entry,RT_NULL,
													PWM_stack_size,	
												PWM_priority,PWM_timeslice);
	if(PWM_task!=RT_NULL)
		rt_thread_startup(PWM_task);
	LED_TASK=rt_thread_create("LED_task",
													LED_Entry,RT_NULL,
													PWM_stack_size,	
												2,PWM_timeslice);
	if(LED_TASK!=RT_NULL)
		rt_thread_startup(LED_TASK);
	return 0;
}


#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(PWM_Control);
#endif