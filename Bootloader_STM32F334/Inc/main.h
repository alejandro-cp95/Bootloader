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
/* ACK and NACK bytes */
#define BL_ACK 0xA5
#define BL_NACK 0x7F
/* CRC */
#define VERIFY_CRC_SUCCESS 0
#define VERIFY_CRC_FAIL 1
/* Addresses */
#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01
/* Some Start and End addresses of different memories of STM32F334 */
#define SRAM_SIZE			12*1024
#define SRAM_END			(SRAM_BASE+SRAM_SIZE)
#define FLASH_SIZE			64*1024
#define FLASH_END			(FLASH_BASE+FLASH_SIZE)
#define SYSTEM_MEMORY_BASE	((uint32_t)0x1FFFD800U)
#define SYSTEM_MEMORY_SIZE	8*1024
#define SYSTEM_MEMORY_END	(SYSTEM_MEMORY_BASE+SYSTEM_MEMORY_SIZE)
#define INVALID_PAGE 0x04

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/*Bootloader function prototypes*/

void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

/* Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_GET_HELP command */
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_EN_R_W_PROTECT command */
void bootloader_handle_en_r_w_protect_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_EN_W_PROTECT command */
void bootloader_handle_en_w_protect_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_MEM_READ command */
void bootloader_handle_mem_read_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_READ_PAGE_PROT_STATUS command */
void bootloader_handle_read_page_prot_status_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_OTP_READ command */
void bootloader_handle_read_otp_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_DIS_R_W_PROTECT command */
void bootloader_handle_dis_r_w_protect_cmd(uint8_t* bl_rx_buffer);
/* Helper function to handle BL_DIS_W_PROTECT command */
void bootloader_handle_dis_w_protect_cmd(uint8_t* bl_rx_buffer);

void bootloader_send_ack(uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc(uint8_t* pData,uint32_t len, uint32_t crc_host);
void bootloader_uart_write_data(uint8_t* pBuffer, uint32_t len);
uint8_t get_bootloader_version(void);
uint16_t get_mcu_chip_id(void);
void get_flash_rdp_level(uint8_t* rdp_level);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t page_number, uint8_t number_of_pages);
uint8_t execute_mem_write(uint16_t* pBuffer, uint32_t mem_address, uint8_t len);
void configure_flash_protection(uint8_t rw, uint16_t description,uint8_t disable);
uint8_t configure_flash_page_r_protection(uint8_t disable);
uint8_t configure_flash_page_w_protection(uint16_t page_details, uint8_t disable);


/* Bootloader version 1.0 */
#define BL_VERSION 0x10
/* Our bootloader commands */

/* This command is used to read the bootloader version from the MCU */
#define BL_GET_VER					0x51
/* This command is used to know what are the commands supported by the bootloader */
#define BL_GET_HELP					0x52
/* This is used to read the MCU chip identification number */
#define BL_GET_CID					0x53
/* This command is used to read the FLASH Read Protection level */
#define BL_GET_RDP_STATUS			0x54
/* This command is used to jump bootloader to specified address */
#define BL_GO_TO_ADDR				0x55
/* This command is used to mass erase or page erase of the user flash */
#define BL_FLASH_ERASE				0x56
/* This command is used to write data in to different memories of the MCU */
#define BL_MEM_WRITE				0x57
/* This command is used to enable read and write protections on the user flash */
#define BL_EN_R_W_PROTECT			0x58
/* This command is used to enable write protection on different pages of the user flash */
#define BL_EN_W_PROTECT				0x59
/* This command is used to read data from different memories of the microcontroller */
#define BL_MEM_READ					0x5A
/* This command is used to read all the page protection status */
#define BL_READ_PAGE_PROT_STATUS	0x5B
/* This command is used to read the OTP contents */
#define BL_OTP_READ					0x5C
/* This command is used to disable all pages read and write protections */
#define BL_DIS_R_W_PROTECT			0x5D
/* This command is used to disable write protection on different pages of the flash */
#define BL_DIS_W_PROTECT			0x5E

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
