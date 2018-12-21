#include <stdint.h>
#include "stm32f4xx.h"


int main(void)
{
	ADC_Typedef *pADC;
	pADC=ADC1;
	pADC->CR1=0x55;
	return 0;
}