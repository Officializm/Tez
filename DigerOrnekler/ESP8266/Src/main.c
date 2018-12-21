
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_NVIC_Init(void);

void USART_puts(USART_TypeDef* USARTx, volatile char *data);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);

static void ESP8266_Init(void);
static void Clear_ESPBuffer(void);
	

#define ESP8266BUFFER_LENGTH 500
char wifiBuffer[ESP8266BUFFER_LENGTH];
static uint16_t ESPWriteIndex=0;

char rx_buff[500],tx_buff[500];
char tx_buff3[500];
bool led_state=false;

int main(void)
{

  HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
	MX_NVIC_Init();
  
  __HAL_UART_ENABLE_IT(&huart6,USART_SR_RXNE);
	__HAL_UART_ENABLE_IT(&huart6,USART_SR_TC);
 
  while (1)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buff, sprintf(tx_buff, "infinite loop\n"),HAL_MAX_DELAY);
		HAL_Delay(1000);
		ESP8266_Init();
  }
 

}


static void Clear_ESPBuffer(void)
{
    uint16_t counter;
	
    for(counter=0; counter<ESP8266BUFFER_LENGTH; counter++)
        wifiBuffer[counter] = 0;

    ESPWriteIndex=0;
}
static void ESP8266_Init(void)
{
    static uint8_t ESPInitCase=0;
	
		HAL_UART_Transmit(&huart1, (uint8_t *)tx_buff, sprintf(tx_buff, "Fonksiyon basladi\n"),HAL_MAX_DELAY);
	
    //USART_puts(USART3,"Fonksiyon basladi\n");

    switch(ESPInitCase)
    {
        case 0:
					HAL_UART_Transmit(&huart6,(uint8_t*)rx_buff, sprintf(rx_buff,"AT\r\n"),HAL_MAX_DELAY);

					HAL_Delay(1000);
				
					//USART_puts(USART3,"OK\n");
				
					HAL_UART_Transmit(&huart1, (uint8_t *)tx_buff3, sprintf(tx_buff3, "OK\n"), HAL_MAX_DELAY);
				
          //Standart AT Komutu gonderilir , Cevabi OK olmalidir.
					//USART_puts(USART1,"AT\r\n");
					HAL_UART_Receive_IT(&huart6,(uint8_t*)wifiBuffer, sizeof(uint16_t));
						
           // 1 saniye gecikme koyuyoruz.
           //HAL_Delay(1000);
          ESPInitCase = 1;
        break;

        case 1:
					
          // STRSTR fonksiyonu metin icinde metin arayan bir fonksiyondur
          // Eger aranilan metni bulabilirse pointer adresi ile geri donus yapar
          // Bulamaz ise NULL dondurur. Bizde null'dan farkli mi diye sorariz.
          if (strstr(wifiBuffer,"OK") != NULL)
          {
            Clear_ESPBuffer();
            //USART_puts(USART3,"Module Erisildi\n");
						HAL_UART_Transmit(&huart1,(uint8_t*)tx_buff3, sprintf(tx_buff3,"Module Erisildi\n"), HAL_MAX_DELAY);
            ESPInitCase = 2;
          }
					
          else
          {
            Clear_ESPBuffer();
            //USART_puts(USART3,"Modul Bulunamadi, Tekrar Deneniyor\n");
					  HAL_UART_Transmit(&huart1,(uint8_t*)tx_buff3, sprintf(tx_buff3,"Modul Bulunamadi, Tekrar Deneniyor\n"), HAL_MAX_DELAY);
            ESPInitCase = 0;
          }
					
        break;
		}
		
}

/*-------------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------General Initializations-----------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------------*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
