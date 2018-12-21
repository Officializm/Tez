#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include "string.h"
char str[50];
uint32_t i;
uint32_t x;
uint8_t adc_data;
uint16_t Read_ADC(void)
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_56Cycles);
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	return ADC_GetConversionValue(ADC1);
		
}



void USART_Puts(USART_TypeDef* USARTx, volatile char *s)
{ while(*s)
	{
	 while(!((USARTx ->SR & 0x00000040)))
	 {
		 
	 }
	 USART_SendData(USARTx,*s);
	 *s++; 
  }
}


void UU_PutChar(USART_TypeDef* USARTx, uint8_t ch)
{
  while(!(USARTx->SR & USART_SR_TXE));
  USARTx->DR = ch;  
}

void UU_PutNumber(USART_TypeDef* USARTx, uint32_t a)
{ 
	char value[10];
	int m=0;
	
	do
	{
		value[m++]=(char)(a%10)+ '0';
		a/=10;
	}while(a);
	
	while(m)
	{
		UU_PutChar(USARTx, value[--m]);
   		
	}

}


void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		//GPIO_ToggleBits(GPIOC,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	  adc_data=Read_ADC();
	}
}

int main()
{
	
	//12-bitlik adc verisini bulunduracak
	
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3  ;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC MQ-2 için kullanilan pin konfigurasyonlari
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler          = 8399;
	TIM_TimeBaseStructure.TIM_CounterMode        = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period             = 100;
	TIM_TimeBaseStructure.TIM_ClockDivision      = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_RepetitionCounter  = 50;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	TIM_Cmd(TIM2,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
	
	
	ADC_CommonInitStructure.ADC_Mode     = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler= ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	
	ADC_InitStructure.ADC_Resolution     = ADC_Resolution_8b;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,  ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,   ENABLE);
	
	
	//-----------------------------------------------------------------------------
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	 


   USART_InitStructure.USART_BaudRate = 9600;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_Init(USART1, &USART_InitStructure);
   USART_Cmd(USART1, ENABLE);
 

   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 

   NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
   NVIC_Init(&NVIC_InitStruct);
	 
	 NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
   NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	 NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStruct);
	 
	

	while(1)
	{
		
	 
		//max 255- min 0
		/*
		if( adc_data<=95 )
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_0);
			GPIO_ResetBits(GPIOC,GPIO_Pin_1);
			GPIO_ResetBits(GPIOC,GPIO_Pin_2);
			GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		}
		
		else if(adc_data>95 && adc_data<=100)
	  { 
			GPIO_SetBits(GPIOC,GPIO_Pin_0);
			GPIO_ResetBits(GPIOC,GPIO_Pin_1);
			GPIO_ResetBits(GPIOC,GPIO_Pin_2);
			GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		}	
    else if(adc_data>100 && adc_data<=120)
    {
			GPIO_SetBits(GPIOC,GPIO_Pin_0);
			GPIO_SetBits(GPIOC,GPIO_Pin_1);
			GPIO_ResetBits(GPIOC,GPIO_Pin_2);
			GPIO_ResetBits(GPIOC,GPIO_Pin_3);
    }	 
		else if(adc_data>120 && adc_data<=140)
    {
			GPIO_SetBits(GPIOC,GPIO_Pin_0);
			GPIO_SetBits(GPIOC,GPIO_Pin_1);
			GPIO_SetBits(GPIOC,GPIO_Pin_2);
			GPIO_ResetBits(GPIOC,GPIO_Pin_3);
    }	
		else if(adc_data>140 )
    {
			GPIO_SetBits(GPIOC,GPIO_Pin_0);
			GPIO_SetBits(GPIOC,GPIO_Pin_1);
			GPIO_SetBits(GPIOC,GPIO_Pin_2);
			GPIO_SetBits(GPIOC,GPIO_Pin_3);
    }
		
	*/
	
	//---------------------------------------------------------------------

  sprintf(str,"\n");
	USART_Puts(USART1,str);
	
	UU_PutNumber(USART1,adc_data);
	
//Delay Yaz lan pust
	i=900000;
	while(i)
	i--;
	

		
	}
}

