#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/rt_drv_pwm.h>
#include <board.h>
#include "app_uSart.h"
#define KEY0			3
#define KEY1			2
uint8_t PWM_Sign=0;
#define PWM_priority 1//PWM�������ȼ�
#define PWM_stack_size 512//�������������ջ��С
#define PWM_timeslice  5	//��������ѯʱ��ʱ��Ƭ
#define PWM_DEV_NAME        "pwm2"  /* PWM�豸���� */
#define Motor1_PWM					"pwm10"
#define PWM_DEV_CHANNEL     2      /* PWMͨ�� */
#define LED1_PA6	 GET_PIN(A,6)
#define Motor1_PWN_PIN  GET_PIN(B,8)		//PB8
struct rt_device_pwm *pwm_dev;      /* PWM�豸��� */  
static rt_thread_t PWM_task=RT_NULL;//��������1�Ķ�̬�߳�
static rt_thread_t LED_TASK=RT_NULL;//����LED�����߳�
extern float Circle;
extern float sum;
rt_uint32_t period, pulse;
int16_t speed_set=300;
extern float P_Pre;
void PWM_Set()
{
		period = 200;    /* ����Ϊ0.5ms����λΪ����ns ����tim10��˵��λ��0.0005us*/
		pulse =period/2;          /* PWM������ֵ����λΪ����ns */
		/* �����豸 */
		pwm_dev = (struct rt_device_pwm *)rt_device_find("pwm2");
		/* ����PWM���ں������� */
		rt_pwm_set(pwm_dev,4, period, pulse);//�ĸ���ֵ�ֱ����ĸ�PWM,channel,���ڣ�����
		/* ʹ���豸 */
		rt_pwm_enable(pwm_dev,4);
		
		/* �ر��豸ͨ�� */
	
}

void PWM_Control_Entry(void *parameter)
{
	uint32_t PWM_Rate=200;//pwmֵ
	
	PWM_Set();
	extern uint8_t end_run;
	
	//PWM_Control_Key();
	while(1)
	{
		PWM_Sign=uart1_getchar();
		
		if(0X01==PWM_Sign)
		{
			rt_kprintf("PWM����\n");
			speed_set+=100;
			PWM_Rate+=100;
			if(5000<=speed_set)
			{
				speed_set=5000;
				
			}
		}
		if(0X02==PWM_Sign)
		{
			PWM_Rate-=100;
			speed_set-=100;
			rt_kprintf("PWM����\n");
			if(PWM_Rate<=200)
			{
				PWM_Rate=200;
			}
			if(speed_set<=-5000)
			{
				speed_set=-5000;
			}
		}
		if(0x03==PWM_Sign)
		{
			
			speed_set=0;
			sum=0;
			end_run=1;
			
		}
		if(0x04==PWM_Sign)
		{
			speed_set=1000;
			
		}
		if(0x05==PWM_Sign)
		{
			speed_set=-1000;
			
		}
		if(0x06==PWM_Sign)
		{
			Circle+=0.5;
		}
		if(0x00==PWM_Sign)
		{
			sum=0;
			Circle=5;
			speed_set=400;
			P_Pre=0;
			end_run=1;
			
		}
			rt_pwm_set(pwm_dev ,4,PWM_Rate, PWM_Rate/2);
		//rt_pwm_set(pwm_dev,1,500000,100000);
		//	rt_pwm_set(pwm_dev,2,500000,200000);
		//	rt_pwm_set(pwm_dev,3,500000,300000);
		
	}
}
void LED_Entry(void *parameter)
{
	 extern uint8_t Init_Sign_CAN;  
extern	uint8_t Init_Sign_SPI;
	uint8_t LED_SIGN=0;
	while(1)
	{
//		rt_pin_write(LED1_PA6,LED_SIGN);
//		LED_SIGN=~LED_SIGN;
		rt_kprintf("Init_Sign_CAN : %d\n",Init_Sign_CAN);
	rt_kprintf("Init_Sign_SPI : %d\n",Init_Sign_SPI);
		rt_thread_mdelay(1000);
		
	}
	
	
	
}

int PWM_Control(void)
{
	rt_pin_mode(LED1_PA6,PIN_MODE_OUTPUT);
	uart1_open("uart1");
	/*�����߳�1*/
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