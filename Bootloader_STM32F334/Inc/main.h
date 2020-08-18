/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define FLASH_PAGE16_BASE_ADDRESS 0x08008000U
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/*Bootloader function prototypes*/

void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_go_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_flash_erase_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_mem_write_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_endis_rw_protect(uint8_t* bl_rx_buffer);
void bootloader_handle_mem_read(uint8_t* bl_rx_buffer);
void bootloader_handle_read_sector_status(uint8_t* bl_rx_buffer);
void bootloader_handle_read_otp(uint8_t* bl_rx_buffer);

/* Our bootloader commands */

/* This command is used to read the bootloader version from the MCU */
#define BL_GET_VER 0x51
/* This command is used to know what are the commands supported by the bootloader */
#define BL_GET_HELP 0x52
/* This is used to read the MCU chip identification number */
#define BL_GET_CID 0x53
/* This command is used to read the FLASH Read Protection level */
#define BL_GET_RDP_STATUS 0x54
/* This command is used to jump bootloader to specified address */
#define BL_GO_TO_ADDR 0x55
/* This command is used to mass erase or sector erase of the user flash */
#define BL_FLASH_ERASE 0x56
/* This command is used to write data in to different memories of the MCU */
#define BL_MEM_WRITE 0x57
/* This command is used to enable or disable read/write protect on different sectors of the  */
#define BL_ENDIS_RW_PROTECT 0x58
/* This command is used to read data from different memories of the microcontroller */
#define BL_MEM_READ 0x59
/* This command is used to read all the sector protection status */
#define BL_READ_SECTOR_STATUS 0x5A
/* This command is used to read the OTP contents */
#define BL_OTP_READ 0x5B

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
