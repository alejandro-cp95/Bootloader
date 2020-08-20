/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32f3xx_hal.h"
/* USER CODE END Includes */

// Enable this line to get debug messages over debug UART
#define BL_DEBUG_MSG_EN

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t supported_commands[]=
{
		BL_GET_VER,
		BL_GET_HELP,
		BL_GET_CID,
		BL_GET_RDP_STATUS,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_MEM_WRITE,
		BL_ENDIS_RW_PROTECT,
		BL_MEM_READ,
		BL_READ_SECTOR_STATUS,
		BL_OTP_READ
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void printmsg(char* format,...);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define D_UART &huart1
#define C_UART &huart2

uint8_t bl_rx_buffer [200];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_GPIO_ReadPin(B1_GPIO_Port,GPIO_PIN_13)==GPIO_PIN_RESET)
  {
	  printmsg("BL_DEBUG_MSG: Button is pressed. Going to BL mode.\n\r");
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
	  bootloader_uart_read_data();
  }
  else
  {
	  /* Here I have a breakpoint */
	  printmsg("BL_DEBUG_MSG: Button is not pressed. Executing user app\n\r");
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
	  bootloader_jump_to_user_app();
  }

  /* USER CODE END 2 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
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

void printmsg(char* format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	/* Extract the argument list using VA apis */
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);
	va_end(args);

#endif
}

void bootloader_uart_read_data(void)
{
	uint8_t rcv_len=0;
	while(1)
	{
		memset(bl_rx_buffer,0,200);
		/* Here we will read and decode the commands coming from host
		 * First read only one byte from the host, which is the "length" field of the command packet */
		HAL_UART_Receive(C_UART, bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len=bl_rx_buffer[0];
		HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		switch(bl_rx_buffer[1])
		{
			case BL_GET_VER:
				bootloader_handle_getver_cmd(bl_rx_buffer);
				break;
			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(bl_rx_buffer);
				break;
			case BL_GET_CID:
				bootloader_handle_getcid_cmd(bl_rx_buffer);
				break;
			case BL_GET_RDP_STATUS:
				bootloader_handle_getrdp_cmd(bl_rx_buffer);
				break;
			case BL_GO_TO_ADDR:
				bootloader_handle_go_cmd(bl_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				bootloader_handle_flash_erase_cmd(bl_rx_buffer);
				break;
			case BL_MEM_WRITE:
				bootloader_handle_mem_write_cmd(bl_rx_buffer);
				break;
			case BL_ENDIS_RW_PROTECT:
				bootloader_handle_endis_rw_protect(bl_rx_buffer);
				break;
			case BL_MEM_READ:
				bootloader_handle_mem_read(bl_rx_buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handle_read_sector_status(bl_rx_buffer);
				break;
			case BL_OTP_READ:
				bootloader_handle_read_otp(bl_rx_buffer);
				break;
			default:
				printmsg("BL_DEBUG_MSG: Invalid command code received from host\r\n");
				break;
		}
	}
}

void bootloader_jump_to_user_app(void)
{
	// Just a function pointer to hold the address of the reset handler of the user app
	void(*app_reset_handler)(void);

	printmsg("BL_DEBUG_MSG: bootloader_jump_to_user_app\n\r");

	// 1. Configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t*) FLASH_PAGE16_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG: MSP value: %#x\n\r", msp_value);

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_PAGE16_BASE_ADDRESS+4 */
	uint32_t resethandler_address = *(volatile uint32_t*) (FLASH_PAGE16_BASE_ADDRESS+4);
	app_reset_handler=(void*)resethandler_address;
	printmsg("BL_DEBUG_MSG: app reset handler addr: %#x\n\r",app_reset_handler);

	// This function comes from CMSIS
	__set_MSP(msp_value);

	//SCB -> VTOR = FLASH_PAGE16_BASE_ADDRESS;

	// 3. Jump to reset handler of the user application
	app_reset_handler();
}

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

/* Implementation of Boot-loader Command Handle functions */

/* Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t bl_version;
	// 1) Verify the checksum
	printmsg("BL_DEBUG_MSG: bootloader_handle_getver_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);
		bl_version=get_bootloader_version();
		printmsg("BL_DEBUG_MSG: BL_VER: %d %#x\n",bl_version,bl_version);
		bootloader_uart_write_data(&bl_version,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Helper function to handle BL_GET_HELP command */
/* Bootloader sends out all supported command codes */
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer)
{
	printmsg("BL_DEBUG_MSG: bootloader_handle_gethelp_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(sizeof(supported_commands));
		bootloader_uart_write_data(supported_commands,sizeof(supported_commands));
	}
	else
	{
		printmsg("BL_DEBUG_MSG: checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_ENDIS_RW_PROTECT command */
void bootloader_handle_endis_rw_protect(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_MEM_READ command */
void bootloader_handle_mem_read(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_READ_SECTOR_STATUS command */
void bootloader_handle_read_sector_status(uint8_t* bl_rx_buffer)
{

}

/* Helper function to handle BL_OTP_READ command */
void bootloader_handle_read_otp(uint8_t* bl_rx_buffer)
{

}

/* This function sends ACK if CRC matches along with "length to follow" */
void bootloader_send_ack(uint8_t follow_len)
{
	// Here we send 2 bytes. The first one is ACK, and the second one is the length value
	uint8_t ack_buf[2];
	ack_buf[0]=BL_ACK;
	ack_buf[1]=follow_len;
	HAL_UART_Transmit(C_UART,ack_buf,2,HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	uint8_t nack=BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}

/* This verifies the CRC of the given buffer in pData */
uint8_t bootloader_verify_crc(uint8_t* pData,uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue=0xFF;
	/* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);
	for(uint32_t i=0; i<len; i++)
	{
		uint32_t i_data=pData[i];
		uwCRCValue=HAL_CRC_Accumulate(&hcrc,&i_data,1);
	}
	if(uwCRCValue==crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	return VERIFY_CRC_FAIL;
}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t* pBuffer, uint32_t len)
{
	/* You can replace the below ST's USART driver API call with your MCU's driver API call */
	HAL_UART_Transmit(C_UART,pBuffer,len,HAL_MAX_DELAY);
}

/* Just returns the macro value */
uint8_t get_bootloader_version(void)
{
	return (uint8_t)BL_VERSION;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
