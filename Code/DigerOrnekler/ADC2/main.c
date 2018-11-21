#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include "string.h"
char str[50];
uint32_t i;
uint32_t x;
char deger[20];
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


int main()
{
	
	uint8_t adc_data;//12-bitlik adc verisini bulunduracak
	GPIO_InitTypeDef GPIO_InitStructure;
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3  ;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	
	ADC_CommonInitStructure.ADC_Mode     = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler= ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	
	ADC_InitStructure.ADC_Resolution     = ADC_Resolution_8b;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);
	
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,  ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,   ENABLE);
	
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	
	
	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);      
	USART_Cmd(USART2,ENABLE);
	
	
	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);      
	USART_Cmd(USART2,ENABLE);
	
	
	
	//---------------------------------------------------------
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
// Initialize pins as alternating function
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 NVIC_InitTypeDef NVIC_InitStruct;
 
/**
 * Enable clock for USART1 peripheral
 */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
 
/**
 * Set Baudrate to value you pass to function
 * Disable Hardware Flow control
 * Set Mode To TX and RX, so USART will work in full-duplex mode
 * Disable parity bit
 * Set 1 stop bit
 * Set Data bits to 8
 *
 * Initialize USART1
 * Activate USART1
 */
USART_InitStructure.USART_BaudRate = 9600;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_Init(USART1, &USART_InitStructure);
USART_Cmd(USART1, ENABLE);
 
/**
 * Enable RX interrupt
 */
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 
/**
 * Set Channel to USART1
 * Set Channel Cmd to enable. That will enable USART1 channel in NVIC
 * Set Both priorities to 0. This means high priority
 *
 * Initialize NVIC
 */
NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
NVIC_Init(&NVIC_InitStruct);
	 
	
	
	
	while(1)
	{
		 
		adc_data=Read_ADC();//max 255- min 0
		
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
	
	/*
	for(x=0;x<20;x++)
	{
		deger[x]=Read_ADC();
	}
	//sprintf(str,"HELLO\n");
	USART_Puts(USART1,deger);
	

	i=600000;
	while(i)
	i--;

*/		
	}
}

