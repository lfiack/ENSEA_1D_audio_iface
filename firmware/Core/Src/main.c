/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "sgtl5000.h"

#include "vu_meter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAI_BUFFER_LENGTH 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_vu_t h_vu;
uint8_t dummy_sai_buffer[SAI_BUFFER_LENGTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);

	return len;
}

static int cplt_counter = 0;
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	if (SAI2_Block_A == hsai->Instance)
	{
		cplt_counter++;
		// TODO Temp juste pour voir
		//		HAL_SAI_DMAStop(&hsai_BlockA2);
	}
}

static int half_cplt_counter = 0;
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	if (SAI2_Block_A == hsai->Instance)
	{
		half_cplt_counter++;
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_I2C2_Init();
	MX_SAI2_Init();
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	vu_init(&h_vu, &hspi3);
	vu_blink(&h_vu);

	printf("\r\n===== 1DA AUDIO IFACE =====\r\n");

	/**
	 * Configure SYS_FS clock to 48 kHz
	 * Configure MCLK_FREQ to 256*Fs = 12288kHz
	 * PLLSAI1P = 12235.294kHz
	 * FS = PLLSAI1P / 256 = 47.79MHz
	 */

	// Il faudrait trouver une solution pour sortir une clock sans démarrer de transmit...

	/**
	 * Pour faire passer l'I2S dans le DAP (en vrai on s'en fout du DAP)
	 * SSS_CTRL->DAP_SELECT to 0x1 (selects I2S_IN)
	 * SSS_CTRL->DAC_SELECT to 0x3 (selects DAP output)
	 *
	 * On commence par un test I2S -> DAC Line output
	 * On ignore donc la partie "ANALOG INPUT BLOCK"
	 * Et on commence par "ANALOG OUTPUTS"
	 * On ne s'intéresse pas non plus à la sortie Headphone
	 *
	 * Routage + gain analogique :
	 * CHIP_DAC_VOL
	 * CHIP_LINE_OUT_VOL
	 * CHIP_ANA_CTRL->MUTE_LO
	 *
	 * Digital input :
	 * CHIP_I2S_CTRL
	 * CHIP_I2S_CTRL->LRPOL
	 * I2S_LRCLK et I2S_SCLK en slave (c'est le STM32 le Master)
	 * -> Synchrone vis à vis de SYS_MCLK, ça devrait être le cas dans le SAI
	 *
	 */


	// Starts SAI to hopefully get MCLK
	//	if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, dummy_sai_buffer, SAI_BUFFER_LENGTH) != HAL_OK)
	//	{
	//		Error_Handler();
	//	}

	//	HAL_SAI_Transmit(&hsai_BlockA2, dummy_sai_buffer, SAI_BUFFER_LENGTH, HAL_MAX_DELAY);

	// Starts MCLK but not the LR clock and other signals
	__HAL_SAI_ENABLE(&hsai_BlockA2);

	/**
	 * For the 32 QFN version of the SGTL5000, the I2C device
	 * address is 0n01010(R/W) where n is determined by
	 * CTRL_ADR0_CS and R/W is the read/write bit from the I2C
	 * protocol.
	 *
	 * CTRL_ADR0_CS is tied to GND
	 * Address is 0001010(R/W)
	 * => 00010100 = 0x14 for Write
	 * => 00010101 = 0x15 for Read
	 * Est-ce que le R/W est ajouté dans la HAL ? Dans ce cas :
	 * => 0001010 = 0x0A -> Eh non!
	 * Cela dit, il rajoute le 1 pour la lecture
	 */

	uint16_t sgtl_address = 0x14;
	uint16_t data;

	/**
	 * Example I2C read
	 * • Start condition
	 * • Device address with the R/W bit cleared to indicate write
	 * • Send two bytes for the 16 bit register address (most significant byte first)
	 * • Stop Condition followed by start condition (or a single restart condition)
	 * • Device address with the R/W bit set to indicate read
	 * • Read two bytes from the addressed register (most significant byte first)
	 * • Stop condition
	 */

	/**
	 * Registers Page 31
	 * CHIP_ID 16 bits
	 * 0xA0HH (0xHH - revision number)
	 */

	h_sgtl5000_t h_sgtl5000;
	h_sgtl5000.hi2c = &hi2c2;
	h_sgtl5000.dev_address = sgtl_address;

	sgtl5000_init(&h_sgtl5000);

	HAL_StatusTypeDef ret;
	ret = sgtl5000_i2c_read_register(&h_sgtl5000, SGTL5000_CHIP_ID, &data);

	if (ret != HAL_OK)
	{
		// Pour l'instant on a un ACK Failure -> C'est réglé!
		printf("HAL_I2C_Mem_Read error\r\n");
		Error_Handler();
	}

#define SAI_TX_BUFFER_LENGTH (480*2)
	static uint16_t sai_tx_buffer[SAI_TX_BUFFER_LENGTH];

	for (int i = 0 ; i < SAI_TX_BUFFER_LENGTH ; i++)
	{
		// Generate a sawtooth at 1kHz
		sai_tx_buffer[i] = i * (0xFFFF/SAI_TX_BUFFER_LENGTH);
	}

	printf("Starting SAI...\r\n");
	// Last parameter is the number of DMA CYCLES (here a cycle is 16 bits/2Bytes)
	HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) sai_tx_buffer, SAI_TX_BUFFER_LENGTH);
	/*
	 * Channels are interleaved in the buffer
	 * Bit LR indicate Left/Right Channel
	 * LSB of right channel is presented when LR indicate Left
	 * Same goes for left channel
	 * (SGTL DIN is shifted by 1 bit on the right on the oscilloscope, it's normal for an SAI apparently)
	 */

	while (1)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
			|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 13;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		vu_blink_red(&h_vu);
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
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
