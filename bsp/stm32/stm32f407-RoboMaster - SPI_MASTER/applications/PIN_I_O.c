#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include <drivers/pin.h>
#include <drivers/rt_drv_pwm.h>
#include "app_usart.h"

#define M1_H  26//PA3
#define M1_L  31//PA6
#define M2_H 	96//PB9
#define M2_L  25//PA2
#define M3_H  24//PA1
#define M3_L  29//PA4
#define M4_H  5//PE6
#define M4_L 4//PE5

//Pmos开关
	#define MOS1 35 //PB0
#define MOS2 61 //PD14
#define MOS3 46 //PE15
//无刷电机1
#define M5_PWM "pwm2"//PB3 TIM2_CH2
#define M5_Brake 80 //PC12
#define M5_FR    84 //PD3
#define M5_EN    78//PC10
//无刷电机2
#define M6_PWM "pwm10"//PB8 TIM10_CH1
#define M6_Brake 66 //PC9
#define M6_FR    59 //PD12
#define M6_EN			79//PC11
//无刷电机3
#define M7_PWM "pwm12"//PB14 TIM12_CH1
#define M7_Brake 40 //PE9
#define M7_FR    41//PE10
#define M7_EN			39//PE8
//无刷电机4
#define M8_PWM "pwm14"//PA7 TIM14_CH1
#define M8_Brake 43 //PE12
#define M8_FR    42 //PE11
#define M8_EN			44	//PE13

#define Motor_PWM_Fre 500 //无刷电机固定频率(500us)
#define task1_priority 20
#define task1_stack_size 1024
#define task1_timeslice  10
#define M1_STOP()  {rt_pin_write(M1_L,1);rt_pin_write(M1_H,1);}//电杆1控制
#define M1_UP()  {rt_pin_write(M1_L,0);rt_pin_write(M1_H,1);}//电杆1控制
#define M1_DOWN()  {rt_pin_write(M1_L,1);rt_pin_write(M1_H,0);}//电杆1控制
#define M2_STOP()  {rt_pin_write(M2_L,1);rt_pin_write(M2_H,1);}//电杆2控制
#define M2_UP()  {rt_pin_write(M2_L,0);rt_pin_write(M2_H,1);}//电杆2控制
#define M2_DOWN()  {rt_pin_write(M2_L,1);rt_pin_write(M2_H,0);}//电杆2控制
#define M3_STOP()  {rt_pin_write(M3_L,1);rt_pin_write(M3_H,1);}//电杆3控制
#define M3_UP()  {rt_pin_write(M3_L,0);rt_pin_write(M3_H,1);}//电杆3控制
#define M3_DOWN()  {rt_pin_write(M3_L,1);rt_pin_write(M3_H,0);}//电杆3控制
#define M4_STOP()  {rt_pin_write(M4_L,1);rt_pin_write(M4_H,1);}//电杆4控制
#define M4_UP()  {rt_pin_write(M4_L,0);rt_pin_write(M4_H,1);}//电杆4控制
#define M4_DOWN()  {rt_pin_write(M4_L,1);rt_pin_write(M4_H,0);}//电杆4控制

#define M5_STOP()  {rt_pin_write(M5_Brake,1);}//前排左轮停止;rt_pin_write(M5_EN,1);
#define M5_Ahead() {rt_pin_write(M5_Brake,0);rt_pin_write(M5_FR,0);rt_pin_write(M5_EN,0);}//前排左轮前进
#define M5_back() {rt_pin_write(M5_Brake,0);rt_pin_write(M5_FR,1);rt_pin_write(M5_EN,0);}//前排左轮后退

#define M6_STOP()  {rt_pin_write(M6_Brake,1);}//后排左轮停止;
#define M6_Ahead() {rt_pin_write(M6_Brake,0);rt_pin_write(M6_FR,0);rt_pin_write(M6_EN,0);}//后排左轮前进
#define M6_back() {rt_pin_write(M6_Brake,0);rt_pin_write(M6_FR,1);rt_pin_write(M6_EN,0);}//后排左轮后退

#define M7_STOP()  {rt_pin_write(M7_Brake,1);}////前排右轮停止
#define M7_Ahead() {rt_pin_write(M7_Brake,0);rt_pin_write(M7_FR,0);rt_pin_write(M7_EN,0);}//前排右轮前进
#define M7_back() {rt_pin_write(M7_Brake,0);rt_pin_write(M7_FR,1);rt_pin_write(M7_EN,0);}//前排右轮后退

#define M8_STOP()  {rt_pin_write(M8_Brake,1);}//后排右轮停止
#define M8_Ahead() {rt_pin_write(M8_Brake,0);rt_pin_write(M8_FR,0);rt_pin_write(M8_EN,0);}//后排右轮前进
#define M8_back() {rt_pin_write(M8_Brake,0);rt_pin_write(M8_FR,1);rt_pin_write(M8_EN,0);}//后排右轮后退

#define Motor_Power_Stop() {rt_pin_write(MOS1,0);rt_pin_write(MOS2,0);rt_pin_write(MOS3,0);}//电机电源开关关闭
#define Motor_Power_Star() {rt_pin_write(MOS1,1);rt_pin_write(MOS2,1);rt_pin_write(MOS3,1);}//电机电源开关开启

uint16_t M_Left_ahead_PWM=80;//PWM初始值
uint16_t M_Right_ahead_PWM=80;//PWM初始值
uint16_t M_Left_back_PWM=80;//PWM初始值
uint16_t M_Right_back_PWM=80;//PWM初始值
uint8_t  Safety_Sign=0;
extern rt_sem_t G_Uart3_Get;
extern uint8_t G_Comman[15];
extern uint8_t G_Send[15];
extern rt_sem_t G_Uart3_Send;//发送信号用的信号量
struct rt_device_pwm *pwm_dev2;      /* PWM设备句柄 */  
struct rt_device_pwm *pwm_dev10;      /* PWM设备句柄 */  
struct rt_device_pwm *pwm_dev12;      /* PWM设备句柄 */  
struct rt_device_pwm *pwm_dev14;      /* PWM设备句柄 */  
#define M5_PWM_SET(X)  {rt_pwm_set(pwm_dev2,2, Motor_PWM_Fre, X);}//设置PWM脉宽
#define M6_PWM_SET(X)  {rt_pwm_set(pwm_dev10,1, Motor_PWM_Fre, X);}//设置PWM脉宽
#define M7_PWM_SET(X) {rt_pwm_set(pwm_dev12,1, Motor_PWM_Fre, X);}//设置PWM脉宽
#define M8_PWM_SET(X) {rt_pwm_set(pwm_dev14,1,Motor_PWM_Fre, X);}//设置PWM脉宽
static rt_thread_t task1=RT_NULL;//创建任务1的动态线程
static rt_thread_t Heart_Jump_Task=NULL;//创建心跳线程
typedef union PWM{
	rt_uint16_t PWM;
	uint8_t PWM_Value[2];
	
}PWM_SET;
PWM_SET L_Wheel;//左轮PWM
PWM_SET R_Wheel;//右轮PWM
PWM_SET S_Wheel;//直线行走PWM
void PWM_Set()
{
	
		/* 查找设备 */
		pwm_dev2 = (struct rt_device_pwm *)rt_device_find(M5_PWM);
		pwm_dev10 = (struct rt_device_pwm *)rt_device_find(M6_PWM);
		pwm_dev12 = (struct rt_device_pwm *)rt_device_find(M7_PWM);
		pwm_dev14 = (struct rt_device_pwm *)rt_device_find(M8_PWM);
		/* 设置PWM脉冲宽度初始值 */
		M5_PWM_SET(400);
	  M6_PWM_SET(400);
		M7_PWM_SET(400);
		M8_PWM_SET(400);
		S_Wheel.PWM=400;
		R_Wheel.PWM=400;
		L_Wheel.PWM=400;
		rt_pwm_enable(pwm_dev10,1);
		rt_pwm_enable(pwm_dev2,2);
		rt_pwm_enable(pwm_dev12,1);
		rt_pwm_enable(pwm_dev14,1);
		//rt_kprintf(" find %s device.\n");
		/* 关闭设备通道 */
	
}
void pin_mode_define(void)//IO口的模式定义
{
	
	rt_pin_mode(M1_L,PIN_MODE_OUTPUT);
	rt_pin_mode(M1_H,PIN_MODE_OUTPUT);
	rt_pin_mode(M2_L,PIN_MODE_OUTPUT);
	rt_pin_mode(M2_H,PIN_MODE_OUTPUT);
	rt_pin_mode(M3_L,PIN_MODE_OUTPUT);
	rt_pin_mode(M3_H,PIN_MODE_OUTPUT);
	rt_pin_mode(M4_L,PIN_MODE_OUTPUT);
	rt_pin_mode(M4_H,PIN_MODE_OUTPUT);
	rt_pin_mode(MOS1,PIN_MODE_OUTPUT);
	rt_pin_mode(MOS2,PIN_MODE_OUTPUT);
	rt_pin_mode(MOS3,PIN_MODE_OUTPUT);
	
	rt_pin_mode(M5_Brake,PIN_MODE_OUTPUT);
	rt_pin_mode(M6_Brake,PIN_MODE_OUTPUT);
	rt_pin_mode(M7_Brake,PIN_MODE_OUTPUT);
	rt_pin_mode(M8_Brake,PIN_MODE_OUTPUT);
	rt_pin_mode(M5_FR,PIN_MODE_OUTPUT);
	rt_pin_mode(M6_FR,PIN_MODE_OUTPUT);
	rt_pin_mode(M7_FR,PIN_MODE_OUTPUT);
	rt_pin_mode(M8_FR,PIN_MODE_OUTPUT);
	rt_pin_mode(M5_EN,PIN_MODE_OUTPUT);
	rt_pin_mode(M6_EN,PIN_MODE_OUTPUT);
	rt_pin_mode(M7_EN,PIN_MODE_OUTPUT);
	rt_pin_mode(M8_EN,PIN_MODE_OUTPUT);
	Motor_Power_Stop();
	rt_pin_write(M5_EN,1);
	rt_pin_write(M6_EN,1);
	rt_pin_write(M7_EN,1);
	rt_pin_write(M8_EN,1);
	rt_pin_write(M5_FR,1);
	rt_pin_write(M6_FR,1);
	rt_pin_write(M7_FR,1);
	rt_pin_write(M8_FR,1);
	M5_STOP();
	M6_STOP();
	M7_STOP();
	M8_STOP();
	
	

	//rt_pin_mode(LED2_PA7,PIN_MODE_OUTPUT);
	
}

static void task1_entry(void *parameter)//任务1要干的事情
{
	
	
	while(1)
	{		
		
			rt_sem_take(G_Uart3_Get,RT_WAITING_FOREVER);
			
			if(G_Comman[2]==0XBB)//上位机发下来喂狗信号
			{
				Safety_Sign=1;
				G_Comman[2]=0XFF;
				
				rt_sem_release(G_Uart3_Send);
				
			}
			
			if(G_Comman[2]==0X01)
			{
				
				switch(G_Comman[3])//提升杆1上下运行控制
				{
					
					case 0x00:
						M1_STOP();
						break;
					case 0x01:
						M1_UP();
						break;
					case 0x02:
						M1_DOWN();
						break;
					default:break;				
				}
				switch(G_Comman[4])//提升杆2上下运行控制
				{
					
					case 0x00:
						M2_STOP();
						break;
					case 0x01:
						M2_UP();
						break;
					case 0x02:
						M2_DOWN();
						break;
					default:break;				
				}
				switch(G_Comman[5])//提升杆上下运行控制
				{
					
					case 0x00:
						M3_STOP();
						break;
					case 0x01:
						M3_UP();
						break;
					case 0x02:
						M3_DOWN();
						break;
					default:break;				
				}
				switch(G_Comman[6])//提升杆上下运行控制
				{
					
					case 0x00:
						M4_STOP();
						break;
					case 0x01:
						M4_UP();
						break;
					case 0x02:
						M4_DOWN();
						break;
					default:break;				
				}
				switch(G_Comman[7])//前方左电机控制
				{
					case 0x00:
						M5_STOP();
						break;
					case 0x01:
						M5_Ahead();
						break;
					case 0x02:
						M5_back();
						break;
					default:break;
				}
				switch(G_Comman[8])//后方左电机控制
				{
					case 0x00:
						M6_STOP();
						break;
					case 0x01:
						M6_Ahead();
						
						break;
					case 0x02:
						M6_back();
						
						break;
					default:break;
				}
				switch(G_Comman[9])//前方右电机控制
				{
					case 0x00:
						M7_STOP();
						break;
					case 0x01:
						M7_Ahead();
						break;
					case 0x02:
						M7_back();
						break;
					default:break;
				}
					switch(G_Comman[10])//后方右电机控制
				{
					case 0x00:
						M8_STOP();
						break;
					case 0x01:
						M8_Ahead();
						break;
					case 0x02:
						M8_back();
						break;
					default:break;
				}
								
			}
			if(G_Comman[2]==0X02)//加速标志
			{
			
					
				switch(G_Comman[7])//前方左轮加减速
				{
					case 0x01:
						M_Left_ahead_PWM+=50;
						if(M_Left_ahead_PWM>=500)
							M_Left_ahead_PWM=500;
						break;
					case 0x02:
						M_Left_ahead_PWM-=50;
						if(M_Left_ahead_PWM<=100)
							M_Left_ahead_PWM=100;
						break;
					default:break;
					}
				switch(G_Comman[8])//后方左轮加减速
				{
					case 0x01:
						M_Left_back_PWM+=50;
						if(M_Left_back_PWM>=500)
							M_Left_back_PWM=500;
						break;
					case 0x02:
						M_Left_back_PWM-=50;
						if(M_Left_back_PWM<=100)
							M_Left_back_PWM=100;
						break;
					default:break;
					}
				switch(G_Comman[9])//前方右轮加减速
				{
					case 0x01:
						M_Right_ahead_PWM+=50;
						if(M_Right_ahead_PWM>=500)
							M_Right_ahead_PWM=500;
						break;
					case 0x02:
						M_Right_ahead_PWM-=50;
						if(M_Right_ahead_PWM<=100)
							M_Right_ahead_PWM=100;
						break;
					default:break;
					}
				switch(G_Comman[10])//后方右轮加减速
				{
					case 0x01:
						M_Right_back_PWM+=50;
						if(M_Right_back_PWM>=500)
							M_Right_back_PWM=500;
						break;
					case 0x02:
						M_Right_back_PWM-=50;
						if(M_Right_back_PWM<=100)
							M_Right_back_PWM=100;
						break;
					default:break;
					}
				}
				/* 设置PWM脉冲宽度 */
			
		
			if(G_Comman[2]==0X03)//PWM数据传输
			{
				
					switch(G_Comman[12])
						
					{
						case 0x01:
							S_Wheel.PWM_Value[0]=G_Comman[3];
							S_Wheel.PWM_Value[1]=G_Comman[4];
							break;
						case 0x02:
							L_Wheel.PWM_Value[0]=G_Comman[3];
							L_Wheel.PWM_Value[1]=G_Comman[4];
							break;
						case 0x03:
							R_Wheel.PWM_Value[0]=G_Comman[3];
							R_Wheel.PWM_Value[1]=G_Comman[4];
							break;
						default:
							break;
						
					}
				}
			if(G_Comman[2]==0X04)//重启电机
			{
				Motor_Power_Stop();
				rt_pin_write(M5_EN,1);
				rt_pin_write(M6_EN,1);
				rt_pin_write(M7_EN,1);
				rt_pin_write(M8_EN,1);//防止启动时运动
				rt_thread_mdelay(2000);
				Motor_Power_Star();
				
			}
			if((G_Comman[2]==0X01&&G_Comman[8]!=G_Comman[9])||(G_Comman[2]==0X03&&G_Comman[12]==0X01))//前进后退速度
			{
				M_Left_ahead_PWM=M_Left_back_PWM=M_Right_ahead_PWM=M_Right_back_PWM=S_Wheel.PWM;
				
			}
				if(G_Comman[2]==0X01&&G_Comman[7]==0X01&&G_Comman[8]==0X01&&G_Comman[9]==0X01&&G_Comman[10]==0X01)//左转速度
			{
				M_Left_ahead_PWM=M_Left_back_PWM=L_Wheel.PWM;
				M_Right_ahead_PWM=M_Right_back_PWM=R_Wheel.PWM;
				
			}
				if(G_Comman[2]==0X01&&G_Comman[7]==0X02&&G_Comman[8]==0X02&&G_Comman[9]==0X02&&G_Comman[10]==0X02)//右转速度
			{
				M_Left_ahead_PWM=M_Left_back_PWM=R_Wheel.PWM;
				M_Right_ahead_PWM=M_Right_back_PWM=L_Wheel.PWM;
				
			}
			M5_PWM_SET(M_Left_ahead_PWM);
			M6_PWM_SET(M_Left_back_PWM);
			M7_PWM_SET(M_Right_ahead_PWM);
			M8_PWM_SET(M_Right_back_PWM);
	}
}
int PIN_Control(void)
{
	/*创建线程1*/
	task1=rt_thread_create("task1",
													task1_entry,RT_NULL,
													task1_stack_size,	
													task1_priority,task1_timeslice);
	if(task1!=RT_NULL)
		rt_thread_startup(task1);
	return 0;
}
//MSH_CMD_EXPORT(PIN_Control,PIN Control_Sample);/* 如果设置了RT_SAMPLES_AUTORUN，则加入到初始化线程中自动运行 */
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
   INIT_APP_EXPORT(PIN_Control);
#endif