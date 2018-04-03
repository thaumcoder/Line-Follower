/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */


//#include "stm32f4xx.h"


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"



const unsigned int dt_ms = 20;
const float dt_s = 0.02f;

TIM_OCInitTypeDef TIM_OCInitStruct;
uint16_t ADC1ReadValues[8] = {0,0,0,0,0,0,0,0};
const int sensorValues[8] = {-8, -6, -4, -2, 2, 4, 6, 8};

volatile uint32_t timer_ms = 0;

void SysTick_Handler()
{
	if(timer_ms) timer_ms--;
}

void delay_ms(unsigned int time)
{
	timer_ms = time;
	while(timer_ms > 0){};
}

int round(float val)
{
	int result = (int)val;
	float diff = val - result;
	if(diff >= 0.5f) result++;
	return result;
}

void InitSensors()
{
	ADC_InitTypeDef       ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	DMA_InitTypeDef       DMA_InitStruct;
	GPIO_InitTypeDef      GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	DMA_InitStruct.DMA_Channel = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC1ReadValues;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = 8;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStruct);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;// PC0, PC1, PC2, PC3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;//PA1
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_DeInit();
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = 0;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 8;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_144Cycles);//PA2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_144Cycles);//PA3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_144Cycles);//PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_144Cycles);//PA5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_144Cycles);//PA6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_144Cycles);//PA7
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, ADC_SampleTime_144Cycles);//PC4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 8, ADC_SampleTime_144Cycles);//PC5

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);

}


void InitEngines()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct2;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);

	GPIO_StructInit(&GPIO_InitStruct2);
	GPIO_InitStruct2.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStruct2.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOD, &GPIO_InitStruct2);
	GPIO_SetBits(GPIOD, GPIO_Pin_1);
	GPIO_SetBits(GPIOD, GPIO_Pin_2);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	TIM_Cmd(TIM4, ENABLE);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC2Init(TIM4, &TIM_OCInitStruct); //silnik prawy
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC3Init(TIM4, &TIM_OCInitStruct); //silnik lewy
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

float ReadSensors()
{
	float sum = 0;
	int count = 0;
	for(int i = 0; i < 8; i++)
	{
		if(ADC1ReadValues[i] > 300)
		{
			sum += sensorValues[i];
			count++;
		}
	}
	int pos = round(sum / count);
	float result =  pos * 0.05236f;
	return result;
}

void SteerEngines(float omegaK)
{
	const int forwardPWM = 1259;
	const int ms2PWM = 5036;
	const float rO = 0.046;

	int PWMRight, PWMLeft;

	float VS = omegaK * rO;
	PWMRight = forwardPWM + VS*ms2PWM;
	PWMLeft = forwardPWM - VS*ms2PWM;

	TIM_OCInitStruct.TIM_Pulse = PWMRight;
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_Pulse = PWMLeft;
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
}

float PID(float fiO)
{
	const float KP = 3.8;
	const float KI = 0.3;
	const float KD = 2.2;

	static float previous_fiO = 0;
	static float sum = 0;

	float P = fiO;
	float I = sum + fiO * dt_s;
	float D = (fiO - previous_fiO) / dt_s;

	previous_fiO = fiO;
	sum = I;

	float fiK = KP * P + KI * I + KD * D;
	return fiK;
}

int main()
{
	SystemInit();
	InitEngines();
	InitSensors();

	GPIO_InitTypeDef config_gpioA;
	GPIO_StructInit(&config_gpioA);
	config_gpioA.GPIO_Pin = GPIO_Pin_0;  // przycisk USER
	config_gpioA.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &config_gpioA);

	_Bool active = 0;
	float previous_fiK = 0;

	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{
			active ^= 1;
			if(!active)
			{
				TIM_OCInitStruct.TIM_Pulse = 0;
				TIM_OC2Init(TIM4, &TIM_OCInitStruct);
				TIM_OC3Init(TIM4, &TIM_OCInitStruct);
			}
			delay_ms(200);
		}

		if(active)
		{
			float fiO = ReadSensors();
			float fiK = PID(fiO);
			float omegaK = (fiK - previous_fiK)/dt_s;
			previous_fiK = fiK;
			SteerEngines(omegaK);
			delay_ms(dt_ms);
		}
	}
}
