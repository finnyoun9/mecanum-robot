#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/*全局变量，用于存储按键键码*/
uint8_t Key_Num;

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA6、PA7、PA11、PA12、PA4和PA5引脚初始化为上拉输入
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_10 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB0、PB1、PB2、PB10、PB5、和PB6引脚初始化为上拉输入
}

/**
  * 函    数：获取全局变量定义的按键键码
  * 参    数：无
  * 返 回 值：按键键码
  */
uint8_t Key_GetNum(void)
{
	uint8_t Temp;			//定义一个临时变量用于中转
	if (Key_Num)			//如果全局变量的键码不为0
	{
		/*这3句的目的是，实现读取键码并读后清零的效果*/
		Temp = Key_Num;		//先把键码存入临时变量
		Key_Num = 0;		//键码清零
		return Temp;		//返回临时变量，return语句执行后，函数直接结束
	}
	return 0;				//如果if不成立，键码为0，则默认返回0
}

/**
  * 函    数：获取按键状态
  * 参    数：无
  * 返 回 值：有按键按下，直接返回键码（非阻塞），没有按键按下，返回0
  */
uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0)		//如果PA6引脚电平为0
	{
		return 1;		//直接返回键码1
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0)		//如果PA7引脚电平为0
	{
		return 2;		//直接返回键码2
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)		//如果PB0引脚电平为0
	{
		return 3;		//直接返回键码3
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)		//如果PB1引脚电平为0
	{
		return 4;		//直接返回键码4
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == 0)		//如果PB2引脚电平为0
	{
		return 5;		//直接返回键码5
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0)		//如果PB10引脚电平为0
	{
		return 6;		//直接返回键码6
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0)		//如果PA11引脚电平为0
	{
		return 7;		//直接返回键码7
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)		//如果PA12引脚电平为0
	{
		return 8;		//直接返回键码8
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0)		//如果PB5引脚电平为0
	{
		return 9;		//直接返回键码9
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0)		//如果PB6引脚电平为0
	{
		return 10;		//直接返回键码10
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0)		//如果PA4引脚电平为0
	{
		return 11;		//直接返回键码11
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)		//如果PA5引脚电平为0
	{
		return 12;		//直接返回键码12
	}
	return 0;
}

/**
  * 函    数：用于驱动按键模块运行的自定义按键定时中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数必须在主程序中每隔1ms自动执行一次
  */
void Key_Tick(void)
{
	/*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
	static uint8_t Count;					//用于计次分频
	static uint8_t CurrState, PrevState;	//保存按键本次状态和上次状态
	
	Count ++;			//计次自增
	if (Count >= 20)	//如果计次20次，则if成立，即if每隔20ms进一次
	{
		Count = 0;		//计次清零，便于下次计次
		
		/*获取按键的本次状态和上次状态*/
		PrevState = CurrState;			//获取上次状态
		CurrState = Key_GetState();		//获取本次状态
		
		/*如果本次状态的键码为0，且上次键码不为0，即检测到按键松手瞬间*/
		if (CurrState == 0 && PrevState != 0)
		{
			/*将上次状态的键码复制给全局变量，后续读取此变量，即可得知哪个按键按下了*/
			Key_Num = PrevState;
		}
	}
}
