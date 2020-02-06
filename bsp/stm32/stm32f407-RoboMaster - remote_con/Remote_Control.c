#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "app_uSart.h"
extern rt_device_t uart_device1;
 static rt_thread_t  Remote_Task=RT_NULL;
 static rt_thread_t  Remote_Display=RT_NULL;
uint8_t Re_Pa[18]={0};
/* USER CODE BEGIN FunctionPrototypes */
typedef __packed struct
{
  __packed struct
  {
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;
  }rc;
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  }mouse;
  __packed struct
  {
    uint16_t v;
  }key;
}RC_Ctl_t;

RC_Ctl_t CtrlData;

void RemoteDataProcess(uint8_t *pData)
{  
  CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
  CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
  CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] <<10)) & 0x07FF;
  CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
  CtrlData.rc.s1 = ((pData[5] >> 6) & 0x0003);
  CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
  CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
  CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
  CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
  CtrlData.mouse.press_l = pData[12];
  CtrlData.mouse.press_r = pData[13];
  CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
}
void Remote_ParaGet_Entry(void *parameter)
{
	
	while(1)
	{
		for(uint8_t i=0;i<18;i++)
		{
			Re_Pa[i]=uart1_getchar();
			
		}
//		uart_getchar_len(uart_device1,kk);
//		for(uint8_t i=0;i<18;i++)
//		{
//			rt_kprintf("%x ",kk[i]);
//		}
//		rt_kprintf("\n");
		//		rt_kprintf("ch0 :%d\n",kk[0]);
//		rt_kprintf("ch1 :%d\n",kk[1]);
//		uart_getchar_len(uart_device1, Re_Pa);
			RemoteDataProcess(Re_Pa);
//		

		
	
	}
}

void Remote_Display_Entry(void *parameter)
{
	while(1)
	{
//		rt_kprintf("ch0 :%d\n",CtrlData.rc.ch0);
//		rt_kprintf("ch1 :%d\n",CtrlData.rc.ch1);
//		rt_kprintf("ch2 :%d\n",CtrlData.rc.ch2);
//		rt_kprintf("ch3 :%d\n",CtrlData.rc.ch3);
//		rt_kprintf("s1 :%d\n",CtrlData.rc.s1);
//		rt_kprintf("s2 :%d\n",CtrlData.rc.s2);
//		rt_kprintf("press_l:%d\n",CtrlData.mouse.press_l);
//		rt_kprintf("press_r:%d\n",CtrlData.mouse.press_r);
//		rt_kprintf("key.v:%d\n",CtrlData.key.v);
		for(uint8_t i=0;i<18;i++)
		{
			rt_kprintf("%x ",Re_Pa[i]);
		}
		rt_kprintf("\n");
		rt_thread_mdelay(500);
		
		
	}
	
}
int Remote_Control(void)
{
	
	uart1_open("uart1");
	/*创建线程1*/
	Remote_Task=rt_thread_create("Remote_task",
													Remote_ParaGet_Entry,RT_NULL,
													1024,	
												3,10);
	if(Remote_Task!=RT_NULL)
		rt_thread_startup(Remote_Task);
	
	Remote_Display=rt_thread_create("Remote_Display",
																		Remote_Display_Entry,RT_NULL,
																		1024,
																		4,10);
	if(Remote_Display!=RT_NULL)
		rt_thread_startup(Remote_Display);
	
	return 0;
}
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(Remote_Control);
#endif
