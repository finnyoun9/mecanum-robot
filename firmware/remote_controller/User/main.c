#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "AD.h"
#include "NRF24L01.h"

/*定义全局变量*/
uint8_t KeyNum;							//按键键码

uint16_t AD_LH, AD_LV, AD_RH, AD_RV;	//四个摇杆值的AD原始数据，范围：0~4095
int8_t LH, LV, RH, RV;					//四个摇杆值，范围：-100~100
uint8_t KEY;							//遥控数据包的按键键码

uint8_t Flag;							//定时标志位，在定时中断中每隔100ms置一次1

uint8_t SendFlag;						//发送标志位
uint8_t SuccessRatio;					//发送成功比率，可用于判断信号强度

/*函数声明*/
int8_t DataProcess(uint16_t ADValue);
uint8_t CalculateSuccessRatio(uint8_t SendFlag);

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	NRF24L01_Init();	//NRF24L01初始化
	Key_Init();			//按键初始化
	AD_Init();			//AD初始化
	
	Timer_Init();		//定时器初始化，定时中断时间1ms
	
	while (1)
	{
		/*按键控制*/
		KeyNum = Key_GetNum();		//获取键码
		if (KeyNum)					//如果按键按下
		{
			KEY = KeyNum;			//将键码保存到KEY变量中
		}
		
		/*读取摇杆原始AD值*/
		AD_LH = AD_GetValue(ADC_Channel_0);		//读取PA0，对应左摇杆横向值
		AD_LV = AD_GetValue(ADC_Channel_1);		//读取PA1，对应左摇杆纵向值
		AD_RH = AD_GetValue(ADC_Channel_2);		//读取PA2，对应右摇杆横向值
		AD_RV = AD_GetValue(ADC_Channel_3);		//读取PA3，对应右摇杆纵向值
		
		/*摇杆数据处理*/
		/*调用DataProcess函数，将0~4095的AD值转换为-100~100的摇杆值*/
		LH = DataProcess(AD_LH);
		LV = DataProcess(AD_LV);
		RH = DataProcess(AD_RH);
		RV = DataProcess(AD_RV);
		
		/*Flag在定时中断中每隔100ms置一次1，100ms执行一次发送*/
		if (Flag == 1)			//如果Flag为1
		{
			/*填充发送数据包*/
			NRF24L01_TxPacket[0] = 0x00;	//字节0，规定为ID，值为0x00
			NRF24L01_TxPacket[1] = LH;		//字节1，规定为左摇杆横向值
			NRF24L01_TxPacket[2] = LV;		//字节2，规定为左摇杆纵向值
			NRF24L01_TxPacket[3] = RH;		//字节3，规定为右摇杆横向值
			NRF24L01_TxPacket[4] = RV;		//字节4，规定为右摇杆纵向值
			NRF24L01_TxPacket[5] = KEY;		//字节4，规定为按键键码
			
			/*NRF24L01发送，并得到发送标志位*/
			SendFlag = NRF24L01_Send();
			
			/*调用CalculateSuccessRatio函数，统计最近10次的发送成功率，用于判断信号强度*/
			SuccessRatio = CalculateSuccessRatio(SendFlag);
			
			/*发送成功后，才将KEY置回0，确保按键信号成功发出*/
			if (SendFlag == 1)
			{
				KEY = 0;
			}
			
			/*发送处理完成后，Flag置回0*/
			Flag = 0;
		}
		
		/*OLED显示*/
		OLED_Printf(0, 0, OLED_8X16, "Sig:%02d", SuccessRatio);		//显示信号强度
		OLED_Printf(0, 16, OLED_8X16, "LH:%+04d", LH);				//显示左摇杆横向值
		OLED_Printf(0, 32, OLED_8X16, "LV:%+04d", LV);				//显示左摇杆纵向值
		OLED_Printf(64, 16, OLED_8X16, "RH:%+04d", RH);				//显示右摇杆横向值
		OLED_Printf(64, 32, OLED_8X16, "RV:%+04d", RV);				//显示右摇杆纵向值
		OLED_Update();			//OLED更新
	}
}

/**
  * 函    数：数据处理
  * 参    数：ADValue 原始AD数据，范围：0~4095
  * 返 回 值：处理后的摇杆值，范围：-100~100
  */
int8_t DataProcess(uint16_t ADValue)
{
	int16_t Value;
	
	/*减去AD中值*/
	Value = ADValue - 2048;
	
	/*此处Value的范围：-2048 ~ 2047*/
	
	/*在中值附近设定一段摇杆死区，确保摇杆回中时摇杆值为0*/
	if (Value > 100)
	{
		Value -= 100;	//大于100的值，减去100
	}
	else if (Value < -100)
	{
		Value += 100;	//小于-100的值，增加100
	}
	else
	{
		Value = 0;		//-100到100之间的值，直接归零
	}
	
	/*此处Value的范围：-1948 ~ 1947*/
	
	/*缩放Value值大小*/
	Value = Value * 101 / 1949;		// * 101 / 1949可以确保Value值稳定在-100 ~ 100
	
	/*此处Value的范围：-100 ~ 100*/
	
	return Value;		//返回处理好的摇杆值
}

/**
  * 函    数：计算发送成功比率
  * 参    数：SendFlag 发送标志位，1表示发送成功，其他值为发送失败
  * 返 回 值：最近10次，SendFlag为1的次数，范围：0~10
  */
uint8_t CalculateSuccessRatio(uint8_t SendFlag)
{
	static uint8_t SendFlagArray[10];	//静态数组，存储最近10次的SendFlag
	static uint8_t p;					//指向当前位置
	uint8_t i, SuccessCount;			//最近10次的成功次数
	
	/*依次覆盖存储SendFlag到静态数组中*/
	SendFlagArray[p] = SendFlag;		//在当前位置存储SendFlag
	p ++;								//自增p
	p %= 10;							//最多存10个，随后开始覆盖存储
	
	/*统计数组中，值为1的个数*/
	SuccessCount = 0;					//成功次数置0
	for (i = 0; i < 10; i ++)			//遍历数组
	{
		if (SendFlagArray[i] == 1)		//如果值为1
		{
			SuccessCount ++;			//成功次数加1
		}
	}
	
	return SuccessCount;	//返回最近10次，SendFlag为1的次数
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*定时中断函数1ms自动执行一次*/
		
		Key_Tick();			//调用按键的Tick函数
		
		/*计次分频*/
		Count ++;				//计次自增
		if (Count >= 100)		//如果计次100次，则if成立，即if每隔100ms进一次
		{
			Count = 0;			//计次清零，便于下次计次
			
			Flag = 1;			//每隔100ms，Flag置一次1，用于控制主循环中的数据发送
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
