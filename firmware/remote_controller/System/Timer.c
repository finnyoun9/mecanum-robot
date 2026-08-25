#include "stm32f10x.h"                  // Device header

/**
  * 函    数：定时中断初始化
  * 参    数：无
  * 返 回 值：无
  */
void Timer_Init(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	uint32_t TIM1Clock;
	uint32_t Prescaler;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//开启TIM1的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM1);		//选择TIM1为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/* Read RCC's active clock rather than assuming 72 MHz.  This board can
	 * fall back to 8 MHz HSI when HSE fails; PSC=71 then made the nominal
	 * 1 ms interrupt last 9 ms and the 10 Hz radio cadence collapse to ~1 Hz. */
	RCC_GetClocksFreq(&RCC_Clocks);
	TIM1Clock = RCC_Clocks.PCLK2_Frequency;
	if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
	{
		TIM1Clock *= 2U;	/* APB2 timer clock doubles when prescaled. */
	}
	Prescaler = TIM1Clock / 1000000U;	/* target a 1 MHz TIM1 counter */

	/*时基单元初始化*/
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;                //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = Prescaler - 1U;       //1 MHz / 1000 = 1 ms
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器会用到，此处配置为0
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单元
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);			//清除定时器更新标志位
	                                                //TIM_TimeBaseInit函数末尾，手动产生了更新事件
	                                                //若不清除此标志位，则开启中断后，会立刻进入一次中断
	                                                //如果不介意此问题，则不清除此标志位也可
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);		//开启TIM1的更新中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//配置NVIC为分组2
	                                                //即抢占优先级范围：0~3，响应优先级范围：0~3
	                                                //此分组配置在整个工程中仅需调用一次
	                                                //若有多个中断，可以把此代码放在main函数内，while循环之前
	                                                //若调用多次配置分组的代码，则后执行的配置会覆盖先执行的配置
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;          //选择配置NVIC的TIM1_UP线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);                             //将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*TIM使能*/
	TIM_Cmd(TIM1, ENABLE);			//使能TIM1，定时器开始运行
}

/* 定时器中断函数，可以复制到使用它的地方
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
*/
