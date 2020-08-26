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
		BL_EN_R_W_PROTECT,
		BL_EN_W_PROTECT,
		BL_MEM_READ,
		BL_READ_PAGE_PROT_STATUS,
		BL_OTP_READ,
		BL_DIS_R_W_PROTECT,
		BL_DIS_W_PROTECT
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
			case BL_EN_R_W_PROTECT:
				bootloader_handle_en_r_w_protect_cmd(bl_rx_buffer);
				break;
			case BL_EN_W_PROTECT:
				bootloader_handle_en_w_protect_cmd(bl_rx_buffer);
				break;
			case BL_MEM_READ:
				bootloader_handle_mem_read_cmd(bl_rx_buffer);
				break;
			case BL_READ_PAGE_PROT_STATUS:
				bootloader_handle_read_page_prot_status_cmd(bl_rx_buffer);
				break;
			case BL_OTP_READ:
				bootloader_handle_read_otp_cmd(bl_rx_buffer);
				break;
			case BL_DIS_R_W_PROTECT:
				bootloader_handle_dis_r_w_protect_cmd(bl_rx_buffer);
				break;
			case BL_DIS_W_PROTECT:
				bootloader_handle_dis_w_protect_cmd(bl_rx_buffer);
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

	// 1. Configure the MSP by reading the value from the base address of the page 2
	uint32_t msp_value = *(volatile uint32_t*) FLASH_PAGE16_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG: MSP value: %#x\n\r", msp_value);

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_PAGE16_BASE_ADDRESS+4 */
	uint32_t resethandler_address = *(uint32_t*) (FLASH_PAGE16_BASE_ADDRESS+4);
	app_reset_handler=(void*)resethandler_address;
	printmsg("BL_DEBUG_MSG: app reset handler addr: %#x\n\r",app_reset_handler);

	// This function comes from CMSIS
	//__set_MSP(msp_value);

	//SCB -> VTOR = FLASH_PAGE16_BASE_ADDRESS;

	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
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

/* It shows the version of the bootloader */
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
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);
		bl_version=get_bootloader_version();
		printmsg("BL_DEBUG_MSG: BL_VER: %d %#x\n",bl_version,bl_version);
		bootloader_uart_write_data(&bl_version,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

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
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(sizeof(supported_commands));
		bootloader_uart_write_data(supported_commands,sizeof(supported_commands));
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Gets the chip identification number */
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer)
{
	uint16_t bl_cid_number=0;
	printmsg("BL_DEBUG_MSG: bootloader_handle_getcid_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(2);
		bl_cid_number=get_mcu_chip_id();
		printmsg("BL_DEBUG_MSG: MCU id: %d %#x!\r\n",bl_cid_number,bl_cid_number);
		bootloader_uart_write_data((uint8_t*)&bl_cid_number,2);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Gets the read protection status */
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t rdp_level[2]={0,0};
	printmsg("BL_DEBUG_MSG: bootloader_handle_getrdp_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(2);
		get_flash_rdp_level(rdp_level);
		printmsg("BL_DEBUG_MSG: RDP level: %d %#x!\r\n",rdp_level[0],rdp_level[0]);
		bootloader_uart_write_data(rdp_level,2);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Goes to the direction we specify */
void bootloader_handle_go_cmd(uint8_t* bl_rx_buffer)
{
	uint32_t go_address=0;
	uint8_t addr_valid=ADDR_VALID;
	uint8_t addr_invalid=ADDR_INVALID;

	printmsg("BL_DEBUG_MSG: bootloader_handle_go_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);
		/* Extract the go address */
		go_address = *((uint32_t*)&bl_rx_buffer[2]);
		printmsg("BL_DEBUG_MSG: GO addr: %#x!\r\n",go_address);
		if(verify_address(go_address)==ADDR_VALID)
		{
			/* Tell the host that address is fine */
			bootloader_uart_write_data(&addr_valid,1);

			/* Jump to "go" address.
			 * we don't care what is being done ther.
			 * Host must ensure that valid code is present over there.
			 * It's not th
			 * e duty of bootloader. So just trust and jump */

			/* Not doing the below line will result in hardfault exception for ARM Cortex M */

			/* Since the Cortex-M processors don't support the ARM instruction set,
			 * but only Thumb instruction set the T bit must always be 1, which
			 * is linked to the 0th bit of the PC */
			go_address+=1; /* Make T bit = 1 */
			void(*lets_jump)(void)=(void*)go_address;
			printmsg("BL_DEBUG_MSG: Jumping to go address!\r\n");
			HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			lets_jump();
		}
		else
		{
            printmsg("BL_DEBUG_MSG: GO addr invalid ! \n");
            //tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid,1);
		}
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Erases a specified portion of the Flash memory */
void bootloader_handle_flash_erase_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t erase_status=0x00;
	printmsg("BL_DEBUG_MSG: bootloader_handle_flash_erase_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);
		printmsg("BL_DEBUG_MSG: Initial page: %d no_of_pages: %d\n\r",bl_rx_buffer[2],bl_rx_buffer[3]);

		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
		erase_status = execute_flash_erase(bl_rx_buffer[2],bl_rx_buffer[3]);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);

		bootloader_uart_write_data(&erase_status,1);
		printmsg("BL_DEBUG_MSG: Flash erase status: %#x\r\n",erase_status);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Writes a specified portion of the Flash memory */
void bootloader_handle_mem_write_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t write_status=0x00;
	uint8_t payload_len=bl_rx_buffer[6];

	uint32_t mem_address=*((uint32_t*)(&bl_rx_buffer[2]));

	printmsg("BL_DEBUG_MSG: bootloader_handle_mem_write_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);
		printmsg("BL_DEBUG_MSG: Mem write address: %#x\n\r",mem_address);
		if(verify_address(mem_address)==ADDR_VALID)
		{
			printmsg("BL_DEBUG_MSG: Valid mem write address\n\r");
			HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			write_status = execute_mem_write((uint16_t*)&bl_rx_buffer[7],mem_address,payload_len);
			HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
			bootloader_uart_write_data(&write_status,1);
		}
		else
		{
			printmsg("BL_DEBUG_MSG: Invalid mem write address\n\r");
			write_status=ADDR_INVALID;
			/* Inform host that address is invalid */
			bootloader_uart_write_data(&write_status,1);
		}
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Invalid mem write address\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Enables Read and Write protections of the flash memory */
void bootloader_handle_en_r_w_protect_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t status=0x00;
	printmsg("BL_DEBUG_MSG: bootloader_handle_en_r_w_protect_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);

		status=configure_flash_page_r_protection(0);

		printmsg("BL_DEBUG_MSG: Configure flash protection status: %#x\n\r",status);

		bootloader_uart_write_data(&status,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Enables Write protection of the flash memory */
void bootloader_handle_en_w_protect_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t status=0x00;
	uint16_t page_details;
	printmsg("BL_DEBUG_MSG: bootloader_handle_en_w_protect_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);

		page_details=(bl_rx_buffer[3]<<8)|(bl_rx_buffer[2]);
		status=configure_flash_page_w_protection(page_details,0);

		printmsg("BL_DEBUG_MSG: Configure flash protection status: %#x\n\r",status);

		bootloader_uart_write_data(&status,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

void bootloader_handle_mem_read_cmd(uint8_t* bl_rx_buffer)
{

}

void bootloader_handle_read_page_prot_status_cmd(uint8_t* bl_rx_buffer)
{

}

void bootloader_handle_read_otp_cmd(uint8_t* bl_rx_buffer)
{

}

/* Disables Read protection of the flash memory */
void bootloader_handle_dis_r_w_protect_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t status=0x00;
	printmsg("BL_DEBUG_MSG: bootloader_handle_dis_r_protect_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);

		status=configure_flash_page_r_protection(1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
}

/* Disables Write protection of the flash memory */
void bootloader_handle_dis_w_protect_cmd(uint8_t* bl_rx_buffer)
{
	uint16_t page_details;
	uint8_t status=0x00;
	printmsg("BL_DEBUG_MSG: bootloader_handle_dis_w_protect_cmd\n\r");

	// Total length of the command packet
	uint32_t command_packet_len=bl_rx_buffer[0]+1;

	// Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len-4));

	if(!bootloader_verify_crc(bl_rx_buffer,command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success!\n\r");
		// Checksum is correct
		bootloader_send_ack(1);

		page_details=(bl_rx_buffer[3]<<8)|(bl_rx_buffer[2]);
		status=configure_flash_page_w_protection(page_details,1);

		printmsg("BL_DEBUG_MSG: Configure flash protection status: %#x\n\r",status);

		bootloader_uart_write_data(&status,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail\n\r");
		// Checksum is wrong. Send nack
		bootloader_send_nack();
	}
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

/* Read the chip identifier or device identifier */
uint16_t get_mcu_chip_id(void)
{
	/* The STM32F334XX MCUs integrate an MCU ID code. This ID identifies the ST MCU part number
	 * and the die revision. It is part of the DBG_MCU component and is mapped on the
	 * external PPB bus (see Section 31.15 on page 1111). This code is accessible using the
	 * JTAG debug port (4 to 5 pins) or the SW debug port (two pins) or by the user software
	 * It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid=(uint16_t)(DBGMCU->IDCODE)&0x0FFF;
	return cid;
}

/* Get the read protection status */
void get_flash_rdp_level(uint8_t* rdp_level)
{
	volatile uint32_t* pOB_addr=(uint32_t*) 0x1FFFF800;
	if(((FLASH->OBR>>1)&0x00000003)==0)
	{
		rdp_level[0]=0xAA;
	}
	else if(((FLASH->OBR>>1)&0x00000003)==1)
	{
		rdp_level[0]=0xFF;
	}
	else
	{
		rdp_level[0]=0xCC;
	}
	rdp_level[1]=(uint8_t)(*pOB_addr);
}

/* Verify the address sent by the host */
uint8_t verify_address(uint32_t go_address)
{
	/* What are the valid addresses to which we can jump?
	 * Can we jump to system memory? Yes
	 * Can we jump to SRAM1 memory? Yes
	 * Can we jump to Backup registers? Yes
	 * Can we jump to peripheral memory? It's possible, but not allowed, so no
	 * Can we jump to external memory? Yes */

	/* Incomplete -poorly written.. optimize it */
	if(go_address>= SRAM_BASE&&go_address<=SRAM_END)
	{
		return ADDR_VALID;
	}
	else if(go_address>= FLASH_BASE&&go_address<=FLASH_END)
	{
		return ADDR_VALID;
	}
	else if(go_address>= SYSTEM_MEMORY_BASE&&go_address<=SYSTEM_MEMORY_END)
	{
		return ADDR_VALID;
	}
	else
	{
		return ADDR_INVALID;
	}
}

uint8_t execute_flash_erase(uint8_t page_number, uint8_t number_of_pages)
{
	/* We have totally 32 pages (0 to 31) in STM32F334 MCU
	 * number_of_pages has to be in the range of 0 to 31
	 * If page_number=0xFF, that means mass erase!
	 * Code needs to be modified if your MCU supports more flash pages/sectors */
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t pageError;
	HAL_StatusTypeDef status;

	if(number_of_pages>32)
	{
		return INVALID_PAGE;
	}
	if((page_number==0xFF)||(page_number<=31))
	{
		if(page_number==(uint8_t)0xFF)
		{
			flashErase_handle.TypeErase=FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			/* Here we are just calculating how many pages need to be erased */
			uint8_t remaining_pages = 32-page_number;
			if(number_of_pages>remaining_pages)
			{
				number_of_pages=remaining_pages;
			}
			flashErase_handle.TypeErase=FLASH_TYPEERASE_PAGES;
			flashErase_handle.PageAddress=(FLASH_BASE+2*1024*page_number);
			flashErase_handle.NbPages=number_of_pages;
		}
		/* Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		status=(uint8_t)HAL_FLASHEx_Erase(&flashErase_handle,&pageError);
		HAL_FLASH_Lock();
		return status;
	}
	return INVALID_PAGE;
}

/* This function writes the contents of pBuffer to "mem_address" byte by byte */
/* Note1: Currently this function supports writeing to Flash only */
/* Note2_ This functions don't check whether "mem_address" is a valid address of the Flash range */
uint8_t execute_mem_write(uint16_t* pBuffer, uint32_t mem_address, uint8_t len)
{
	uint8_t status=HAL_OK;
	/* We have to unlock flash module to get control of registers */
	HAL_FLASH_Unlock();
	for(uint8_t i=0;i<len;i+=2)
	{
		/* Here we program the flash byte by byte */
		status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,mem_address+i,pBuffer[i/2]);
	}
	HAL_FLASH_Lock();
	return status;
}

void configure_flash_protection(uint8_t rw, uint16_t description,uint8_t disable)
{
	volatile uint16_t* pWP1 = (uint16_t*)0x1FFFF80A;
	volatile uint16_t* pWP0 = (uint16_t*)0x1FFFF808;
	volatile uint16_t* pRDP = (uint16_t*)0x1FFFF800;
	//volatile uint32_t* pOB_addr = (uint32_t*) 0x1FFFF800;
	//uint8_t RDP_State = (uint8_t)(*pOB_addr);

	// Disable all R/W protection on pages

	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	/* 1. Unlock flash */
	HAL_FLASH_Unlock();
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
	/* 2. Option byte configuration unlock */
	HAL_FLASH_OB_Unlock();
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	/* 3. Erase option bytes by setting OPTER bit and after that STRT bit */
	FLASH->CR |= (1<<5);
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
	FLASH->CR |= (1<<6);
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	/* 4. After the erasure is complete, reset the OPTER bit */
	FLASH->CR &= ~(1<<5);
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	/* 5. Set the OPTPG bit to allow programming option bytes */
	FLASH->CR |= (1<<4);
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	/* 6. */
	if((rw==0)&&(disable==0))
	{
		/* Here we are setting write protection for the pages */

		/* Put write protection on pages */
		*pWP1=(~((uint16_t)(0x00FF&(description>>8))))&((uint16_t)((FLASH->WRPR>>8)&0x000000FF));
		/* Wait till no active operation on flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		*pWP0=(~((uint16_t)(0x00FF&description)))&((uint16_t)(FLASH->WRPR&0x000000FF));
		/* Wait till no active operation on flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		if(((FLASH->OBR>>1)&0x00000003)==0x0)
		{
			*pRDP=(uint16_t)0xAA;
		}
		else
		{
			*pRDP=(uint16_t)0xFF;
		}
	}
	else if((rw==0)&&(disable==1))
	{
		/* Here we are disabling write protection for the pages */

		/* Put write protection on pages */
		*pWP1=((uint16_t)(0x00FF&(description>>8)))|((uint16_t)((FLASH->WRPR>>8)&0x000000FF));
		/* Wait till no active operation on flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		*pWP0=((uint16_t)(0x00FF&description))|((uint16_t)(FLASH->WRPR&0x000000FF));
		/* Wait till no active operation on flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		if(((FLASH->OBR>>1)&0x00000003)==0x0)
		{
			*pRDP=(uint16_t)0xAA;
		}
		else
		{
			*pRDP=(uint16_t)0xFF;
		}
	}
	else if((rw==1)&&(disable==0))
	{
		/* Here we are enabling read protection for the flash */
		*pRDP=(uint16_t)0xFF;
		/* Wait till no active operation on flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		*pWP1=(uint16_t)((FLASH->WRPR>>8)&0x000000FF);
		/* Wait till no active operation on flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		*pWP0=(uint16_t)(FLASH->WRPR&0x000000FF);
	}
	else if((rw==1)&&(disable==1))
	{
		uint8_t status=0;
		printmsg("BL_DEBUG_MSG: Configure flash protection status: %#x\n\r",status);
		bootloader_uart_write_data(&status,1);
		/* Here we are disabling read protection for the pages.
		 * When read protection is disabled a POR should be done */
		/* The write protections are obviously implicitly disabled */
		*pRDP=(uint16_t)0xAA;
	}
	else
	{

	}
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

	/* 7. Reset the OPTPG bit to complete/close programming option bytes */
	FLASH->CR &= ~(1<<4);
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

	/* 8. (Optional) Launch/Apply the Option bytes (OBL_LAUNCH bit is set-system reset will be performed after this) */
	//FLASH->CR |= (1<<13);
	/* Wait till no active operation on flash */
	//while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

	/* 9. Lock option bytes (OPTWRE is reset) */
	HAL_FLASH_OB_Lock();
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

	/* 10. Lock the Flash (LOCK bit is set) */
	HAL_FLASH_Lock();
	/* Wait till no active operation on flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
}

uint8_t configure_flash_page_r_protection(uint8_t disable)
{
	configure_flash_protection(1,0,disable);
	return 0;
}

uint8_t configure_flash_page_w_protection(uint16_t page_details, uint8_t disable)
{
	configure_flash_protection(0,page_details,disable);
	return 0;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
