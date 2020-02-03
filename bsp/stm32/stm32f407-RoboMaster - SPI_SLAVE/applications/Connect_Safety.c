#include <drivers/watchdog.h>
#include <rtthread.h>
#include <rtdevice.h>
#define WDT_DEVICE_NAME    "wdt"    /* 看门狗设备名称 */
#include "app_usart.h"
static rt_device_t wdg_dev;         /* 看门狗设备句柄 */
extern uint8_t Safety_Sign;
extern uint8_t G_Send[15];
extern rt_sem_t G_Uart3_Send;
static rt_thread_t Safety_Task=RT_NULL;
static void safety(void* paramater)
{
    while(1)/* 在空闲线程的回调函数里喂狗 */
		{
			
			if(Safety_Sign==1)
			
		{
		//	rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
			G_Send[2]=0X05;//信号正常连接反馈
			rt_sem_release(G_Uart3_Send);
			Safety_Sign=0;	
		}else{
				rt_pin_write(80,1);
				rt_pin_write(66,1);
				rt_pin_write(40,1);
				rt_pin_write(43,1);
			
	
			
		}
		rt_thread_mdelay(1000);
	}
}

static int  Safety_Task_sample(void)
{
//    rt_err_t ret = RT_EOK;
//    rt_uint32_t timeout = 1;        /* 溢出时间，单位：秒 */
//		
//    /* 根据设备名称查找看门狗设备，获取设备句柄 */
//    wdg_dev = rt_device_find("iwg");
//    /* 初始化设备 */
//    ret = rt_device_init(wdg_dev);
//    /* 设置看门狗溢出时间 */
//    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    /* 设置空闲线程回调函数 */
  Safety_Task=rt_thread_create("u3_Send",
													safety,RT_NULL,/*创建线程1*/
													1024,	
													10,10);
	if(Safety_Task!=RT_NULL)
		rt_thread_startup(Safety_Task);
		return 0;

   
}

#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(Safety_Task_sample);
#endif
