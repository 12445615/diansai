

#include "BSP/inc/bsp_encoder.h"
#include "BSP/inc/huidu.h"



static volatile Encoder Encoder_A;
static volatile Encoder Encoder_B;
volatile uint32_t Encoder_Raw_Pins;
volatile uint32_t Encoder_IRQ_Count;
volatile uint32_t Encoder_Timer_Count;

/******************************************************************
 * 函 数 名 称：Motor_Init
 * 函 数 说 明：电机与编码器初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void Motor_Init(void)
{
	//编码器引脚外部中断
	NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
	NVIC_EnableIRQ(ENCODER_INT_IRQN);

    //定时器中断
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

	lc_printf("Motor initialized successfully\r\n");
}




/******************************************************************
 * 函 数 名 称：Motor_Get_Encoder
 * 函 数 说 明：获取编码器的值
 * 函 数 形 参：dir=0获取左轮编码器值  dir=1获取右轮编码器值
 * 函 数 返 回：返回对应的编码器值
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
int Motor_Get_Encoder(int dir)
{
	if( !dir )
		return Encoder_A.Obtained_Get_Encoder_Count;

	return Encoder_B.Obtained_Get_Encoder_Count;
}


/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/


//电机编码器脉冲计数
void TIMER_0_INST_IRQHandler(void)
{
	
	//编码器速度计算
	if( DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO )
	{
        Encoder_Timer_Count++;
        Encoder_Raw_Pins = DL_GPIO_readPins(ENCODER_PORT,
            ENCODER_E1A_PIN | ENCODER_E1B_PIN |
            ENCODER_E2A_PIN | ENCODER_E2B_PIN);
        /* 两个电机安装相反，所以编码器值也要相反 */
        Encoder_A.Obtained_Get_Encoder_Count = Encoder_A.Should_Get_Encoder_Count;
        Encoder_B.Obtained_Get_Encoder_Count = -Encoder_B.Should_Get_Encoder_Count;

        /* 编码器计数值清零 */
        Encoder_A.Should_Get_Encoder_Count = 0;
        Encoder_B.Should_Get_Encoder_Count = 0;
	}
}
/*单独用时且不打开hc-sr04的情况下是可以的使用比并且测速的*/
extern int why;
void GROUP1_IRQHandler(void)
{
why=200;
    uint32_t gpio_interrup = 0;
    Encoder_IRQ_Count++;
    Encoder_Raw_Pins = DL_GPIO_readPins(ENCODER_PORT,
        ENCODER_E1A_PIN | ENCODER_E1B_PIN |
        ENCODER_E2A_PIN | ENCODER_E2B_PIN);

	//获取中断信号
	gpio_interrup = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT,ENCODER_E1A_PIN|ENCODER_E1B_PIN|ENCODER_E2A_PIN|ENCODER_E2B_PIN);

    // encoderA
	if((gpio_interrup & ENCODER_E1A_PIN)==ENCODER_E1A_PIN)
	{why=300;
		if(!DL_GPIO_readPins(ENCODER_PORT,ENCODER_E1B_PIN))
		{
			Encoder_A.Should_Get_Encoder_Count--;
		}
		else
		{
			Encoder_A.Should_Get_Encoder_Count++;
		}
	}
	else if((gpio_interrup & ENCODER_E1B_PIN)==ENCODER_E1B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODER_PORT,ENCODER_E1A_PIN))
		{
			Encoder_A.Should_Get_Encoder_Count++;
		}
		else
		{
			Encoder_A.Should_Get_Encoder_Count--;
		}
	}

	// encoderB
	if((gpio_interrup & ENCODER_E2A_PIN)==ENCODER_E2A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODER_PORT,ENCODER_E2B_PIN))
		{
			Encoder_B.Should_Get_Encoder_Count--;
		}
		else
		{
			Encoder_B.Should_Get_Encoder_Count++;
		}
	}
	else if((gpio_interrup & ENCODER_E2B_PIN)==ENCODER_E2B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODER_PORT,ENCODER_E2A_PIN))
		{
			Encoder_B.Should_Get_Encoder_Count++;
		}
		else
		{
			Encoder_B.Should_Get_Encoder_Count--;
		}
	}
	
	
  

	DL_GPIO_clearInterruptStatus(ENCODER_PORT,ENCODER_E1A_PIN|ENCODER_E1B_PIN|ENCODER_E2A_PIN|ENCODER_E2B_PIN);

}
