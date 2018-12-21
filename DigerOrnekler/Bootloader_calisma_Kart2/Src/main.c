
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
char rx_buff[50],tx_buff[50];
bool led_state=false; 

#define D_UART   &huart2
#define C_UART   &huart6

uint32_t bAddr=0x08008000;

uint8_t supported_commands[] = {
                               BL_GET_VER ,                               
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               } ;


CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
															 

uint32_t	Flash_Read(uint32_t Flash_Address);
void    	Flash_Write(uint32_t Flash_Address);													 
void      read_data_from_memory(void);
															 
															 
#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];

#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];
	
	
int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
	
	

	if ( HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) != GPIO_PIN_RESET )
      {
 
    	  bootloader_uart_read_data();

      }
      else
      {
    		bootloader_jump_to_user_app();

      }
	
	//Flash_Write(bAddr);
}
/*----------------------------------------------------------Fonksiyonlar-------------------------------------------*/
/*
uint32_t	Flash_Read(uint32_t Flash_Address)
{
	uint32_t	Flash_Data;
	
	Flash_Data = *(uint32_t*) Flash_Address;
	
	
	return Flash_Data;
  
}


void Flash_Write(uint32_t Flash_Address)
{
	HAL_FLASH_Unlock();
  	FLASH_Erase_Sector(FLASH_SECTOR_2,VOLTAGE_RANGE_3);
	for(uint32_t i = 0 ; i <5004 ; i++)
    {
        
   //     veriler[i]=Flash_Read(0x08008000+4*i);
			      
	        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,Flash_Address+i,3333);
	}
	HAL_FLASH_Lock();
}

*/

void read_data_from_memory(void)
{
	
}

void  bootloader_uart_read_data(void)
  {
		

	  uint8_t rcv_len=0;

	  	while(1)
	  	{
	  		memset(bl_rx_buffer,0,200);//200 ile bl_rx_buffer arasini 0 ile doldurur.
	  		//here we will read and decode the commands coming from host
	  		//first read only one byte from the host , which is the "length" field of the command packet
				
	      HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
	  		rcv_len= bl_rx_buffer[0];
	  		HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
				
	  		switch(bl_rx_buffer[1])
	  		{
	              case BL_GET_VER:
	                  bootloader_handle_getver_cmd(bl_rx_buffer);
	                  break;
	              
	              case BL_FLASH_ERASE:
	                  bootloader_handle_flash_erase_cmd(bl_rx_buffer);
	                  break;
								
	              case BL_MEM_WRITE:
	                  bootloader_handle_mem_write_cmd(bl_rx_buffer);
	                  break;
	              
	               default:
	                  break;


	  		}

	  	}




  
	}
	
	/*-----------------------------------------------------------------------------------------------------------------*/
	
void bootloader_jump_to_user_app(void)
  {
		
	  //just a function pointer to hold the address of the reset handler of the user app.
	      void (*app_reset_handler)(void);

	      // 1. configure the MSP by reading the value from the base address of the sector 2
	      uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	      
	      //This function comes from CMSIS.
	      __set_MSP(msp_value);

	      //SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

	      /* 2. Now fetch the reset handler address of the user application
	       * from the location FLASH_SECTOR2_BASE_ADDRESS+4
	       */
	      uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

	      app_reset_handler = (void*) resethandler_address;


	      //3. jump to reset handler of the user application
	  app_reset_handler();
  
	}
 
/*-----------------------------------------------------------------------------------------------------------------*/
	 void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

	 //Total length of the command packet
	  uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	  //extract the CRC32 sent by the Host
	  uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
       
        // checksum is correct..
        bootloader_send_ack(bl_rx_buffer[0],1);
        bl_version=get_bootloader_version();      
        bootloader_uart_write_data(&bl_version,1);

    }else
    {
        
        //checksum is wrong send nack
        bootloader_send_nack();
    }


}

/*-----------------------------------------------------------------------------------------------------------------*/

void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address=0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;
   

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        bootloader_send_ack(pBuffer[0],1);

        //extract the go address
        go_address = *((uint32_t *)&pBuffer[2] );

        if( verify_address(go_address) == ADDR_VALID )
        {
            //tell host that address is fine
            bootloader_uart_write_data(&addr_valid,1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            //watch : https://www.youtube.com/watch?v=VX_12SjnNhY

            go_address+=1; //make T bit =1

            void (*lets_jump)(void) = (void *)go_address;

            lets_jump();

		}else
		{
            //tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid,1);
		}

	}else
	{
        bootloader_send_nack();
	}


}
/*-----------------------------------------------------------------------------------------------------------------*/
/*Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;
    

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        
        bootloader_send_ack(pBuffer[0],1);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);
        erase_status = execute_flash_erase(pBuffer[2] , pBuffer[3]);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0);
        bootloader_uart_write_data(&erase_status,1);

	}else
	{
        bootloader_send_nack();
	}
}
/*-----------------------------------------------------------------------------------------------------------------*/
/*Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	chksum = pBuffer[len];

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        
        bootloader_send_ack(pBuffer[0],1);

		if( verify_address(mem_address) == ADDR_VALID )
		{


            //glow the led to indicate bootloader is currently writing to memory
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            //execute mem write
            write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);

		}else
		{
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}


	}else
	{
        bootloader_send_nack();
	}

}

/*-----------------------------------------------------------------------------------------------------------------*/
/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART,ack_buf,2,HAL_MAX_DELAY);

}
/*-----------------------------------------------------------------------------------------------------------------*/
/*This function sends NACK */
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}
/*-----------------------------------------------------------------------------------------------------------------*/
//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue=0xff;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}
/*-----------------------------------------------------------------------------------------------------------------*/
/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(C_UART,pBuffer,len,HAL_MAX_DELAY);

}

/*-----------------------------------------------------------------------------------------------------------------*/
//Just returns the macro value .
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}

/*-----------------------------------------------------------------------------------------------------------------*/
//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address)
{
	//so, what are the valid addresses to which we can jump ?
		//can we jump to system memory ? yes
		//can we jump to sram1 memory ?  yes
		//can we jump to sram2 memory ? yes
		//can we jump to backup sram memory ? yes
		//can we jump to peripheral memory ? its possible , but dont allow. so no
		//can we jump to external memory ? yes.

	//incomplete -poorly written .. optimize it
		if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END)
		{
			return ADDR_VALID;
		}

		else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
		{
			return ADDR_VALID;
		}

		else if ( go_address >= BKPSRAM_BB_BASE && go_address <= BKPSRAM_BB_END)
			{
				return ADDR_VALID;
		}

		else
	return ADDR_INVALID;
}
/*-----------------------------------------------------------------------------------------------------------------*/
 uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
    //we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if( number_of_sector > 8 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 7) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}else
		{
		    /*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_sector = 8 - sector_number;
            if( number_of_sector > remanining_sector)
            {
            	number_of_sector = remanining_sector;
            }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}


	return INVALID_SECTOR;
}
/*-----------------------------------------------------------------------------------------------------------------*/
/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        //Here we program the flash byte by byte
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
    }

    HAL_FLASH_Lock();

    return status;
}

/*-----------------------------------------------SYSTEM-CONFIGURATIONS------------------------------------------------------------------*/


	 
	 
	 
	 


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);


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

   
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);


  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  huart6.Init.BaudRate = 9600;
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
