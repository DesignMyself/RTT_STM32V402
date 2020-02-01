#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include <drivers/pin.h>
#include <drivers/rt_drv_pwm.h>
#include "app_uSart.h"
Speed_Get motor1;
Speed_Get motor2;
Speed_Get motor3;
Speed_Get	motor4;
extern uint8_t G_Send[21];


TIM_HandleTypeDef htim1;//time2控件块

TIM_HandleTypeDef htim8;//time8控件块

TIM_HandleTypeDef htim3;//time2控件块

TIM_HandleTypeDef htim4;//time2控件块


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
  
    /**TIM2 GPIO Configuration    
    PD2     ------> TIM3_ETR2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP   ;
    GPIO_InitStruct.Pull = GPIO_PULLUP;//使用JGB37-BLDC3525时必须时不上拉不下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
		 GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP   ;
    GPIO_InitStruct.Pull = GPIO_PULLUP;//使用JGB37-BLDC3525时必须时不上拉不下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }

	else if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
  
    /**TIM2 GPIO Configuration    
    PA15     ------> TIM2_ETR 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
	 else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
//		__HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM8 GPIO Configuration    
    PA0-WKUP     ------> TIM8_ETR 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP ;
    GPIO_InitStruct.Pull =GPIO_PULLUP ;
    GPIO_InitStruct.Speed =  GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */
  }

}


 void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspDeInit 0 */

  /* USER CODE END TIM12_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM12_CLK_DISABLE();
  /* USER CODE BEGIN TIM12_MspDeInit 1 */

  /* USER CODE END TIM12_MspDeInit 1 */
  }
	 if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  
    /**TIM2 GPIO Configuration    
    PA15     ------> TIM2_ETR 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  
  else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
  
    /**TIM8 GPIO Configuration    
    PA0-WKUP     ------> TIM8_ETR 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
	

}


static void MX_TIM1_Init(void)

{

	

  TIM_ClockConfigTypeDef sClockSourceConfig;

  TIM_SlaveConfigTypeDef sSlaveConfig;

  TIM_MasterConfigTypeDef sMasterConfig;



  htim1.Instance = TIM1;

  htim1.Init.Prescaler = 0;//tim频率分频，总线为APB1 42M/(0+1)

  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim1.Init.Period = 0xFB3;//复位数,到计数器到达这一位后复位到0

  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)

  {

    

  }



  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;

  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_RISING ;

  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;

  sClockSourceConfig.ClockFilter = 0;

  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)

  {

  

  }

	 HAL_TIM_Base_Init(&htim1);	

}



static void MX_TIM8_Init(void)

{

	

    TIM_SlaveConfigTypeDef sSlaveConfig;

  TIM_MasterConfigTypeDef sMasterConfig;



  htim8.Instance = TIM8;

  htim8.Init.Prescaler = 0;//分频数，由于处于ABPB2 ，84M

  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim8.Init.Period = 0xFB3;

  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim8.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)

  {

   

  }



  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;

  sSlaveConfig.InputTrigger = TIM_TS_ETRF;

  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;

  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;

  sSlaveConfig.TriggerFilter = 4;

  if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)

  {

    

  }



  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)

  {

    

  }



	



	

}



static void MX_TIM3_Init(void)

{

	

   TIM_SlaveConfigTypeDef sSlaveConfig;

  TIM_MasterConfigTypeDef sMasterConfig;



  htim3.Instance = TIM3;

  htim3.Init.Prescaler = 0;

  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim3.Init.Period = 0xFB3;

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim3.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)

  {

    

  }



  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;

  sSlaveConfig.InputTrigger = TIM_TS_ETRF;

  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;

  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;

  sSlaveConfig.TriggerFilter = 4;

  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)

  {

  

  }



  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)

  {

   

  }



}


static void MX_TIM4_Init(void)

{

	

   TIM_SlaveConfigTypeDef sSlaveConfig;

  TIM_MasterConfigTypeDef sMasterConfig;



  htim4.Instance = TIM4;

  htim4.Init.Prescaler = 0;

  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim4.Init.Period = 0xFB3;

  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim4.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)

  {

    

  }



  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;

  sSlaveConfig.InputTrigger = TIM_TS_ETRF;

  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;

  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;

  sSlaveConfig.TriggerFilter = 4;

  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)

  {

  

  }



  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)

  {

   

  }



}

	

	

	







/*******************电机速度数据采集和显示处理任务********************/



//rt_uint16_t kk1=0;//电机1脉冲获取参数
//rt_uint16_t kk2=0;//电机2脉冲获取参数
//rt_uint16_t kk3=0;//电机3脉冲获取参数
//rt_uint16_t kk4=0;//电机4脉冲获取参数
static rt_thread_t motor1_speed_thread=RT_NULL;
const uint8_t V_Time=100;//采集速度的时间
const float wheel_Rate=1;//300*3.14f/45/30;
rt_uint32_t SumMotor1=0;//电机1
rt_uint32_t SumMotor2=0;//电机2
rt_uint32_t SumMotor3=0;//电机3
rt_uint32_t SumMotor4=0;//电机4
extern rt_sem_t G_Uart3_Send;
void motor1_Dis_Catch()

{

		motor1.speed=__HAL_TIM_GET_COUNTER(&htim1)*wheel_Rate/V_Time*1000;//得到TIME1——ETR中的脉冲
		motor2.speed=__HAL_TIM_GET_COUNTER(&htim3)*wheel_Rate/V_Time*1000;//得到TIME1——ETR中的脉冲
		motor3.speed=__HAL_TIM_GET_COUNTER(&htim4)*wheel_Rate/V_Time*1000;//得到TIME1——ETR中的脉冲
		motor4.speed=__HAL_TIM_GET_COUNTER(&htim8)*wheel_Rate/V_Time*1000;//得到TIM8——ETR中的脉冲
		G_Send[2]=0x04;//传输速度数据标志
		G_Send[3]=motor1.PWM_Value[0];
		G_Send[4]=motor1.PWM_Value[1];
		G_Send[5]=motor1.PWM_Value[2];
		G_Send[6]=motor1.PWM_Value[3];
		
		G_Send[7]=motor2.PWM_Value[0];
		G_Send[8]=motor2.PWM_Value[1];
		G_Send[9]=motor2.PWM_Value[2];
		G_Send[10]=motor2.PWM_Value[3];
	
		G_Send[11]=motor3.PWM_Value[0];
		G_Send[12]=motor3.PWM_Value[1];
		G_Send[13]=motor3.PWM_Value[2];
		G_Send[14]=motor3.PWM_Value[3];
	
		G_Send[15]=motor4.PWM_Value[0];
		G_Send[16]=motor4.PWM_Value[1];
		G_Send[17]=motor4.PWM_Value[2];
		G_Send[18]=motor4.PWM_Value[3];
		__HAL_TIM_SET_COUNTER(&htim1,0);//清空TIME8寄存器的数值
		__HAL_TIM_SET_COUNTER(&htim3,0);//清空TIME2寄存器的数值
		__HAL_TIM_SET_COUNTER(&htim4,0);//清空TIME2寄存器的数值
		__HAL_TIM_SET_COUNTER(&htim8,0);//清空TIME2寄存器的数值
		rt_sem_release(G_Uart3_Send);
		rt_thread_mdelay(V_Time);
	
}



/*************建立两个任务，一个用来收取数据，一个用来显示数据******************/

/* 线程的TCB控制块 */




static void Motor1_entry(void *parameter)

{



	while(1)

	{

		//rt_enter_critical();

		motor1_Dis_Catch();
	}

	

}








/******************任务初始化*****************/

int  Motor1_Hmi_Init()

{

		MX_TIM1_Init();	//	初始化TIM1

		MX_TIM3_Init();	//	初始化TIM3
	
		MX_TIM4_Init();	//	初始化TIM3

		MX_TIM8_Init();//	初始化TIM8
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_Base_Start(&htim3);
		HAL_TIM_Base_Start(&htim4);
		HAL_TIM_Base_Start(&htim8);

	__HAL_TIM_SET_COUNTER(&htim1,0);	
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	__HAL_TIM_SET_COUNTER(&htim8,0);
	

	




		motor1_speed_thread= rt_thread_create("MOTOR1Speed",//任务的名称

														Motor1_entry,//任务的主程序入口

														RT_NULL,//任务的传入参数

														1024,//堆栈的大小

														7,//优先级大小

														10);//时间片，在同一优先级时使用

		if(motor1_speed_thread!= RT_NULL)

		{

			

			rt_thread_startup(motor1_speed_thread);

			

		}

		

		return 0;

}

/* 如果设置了RT_SAMPLES_AUTORUN，则加入到初始化线程中自动运行 */

#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)

    INIT_APP_EXPORT(Motor1_Hmi_Init);

#endif

