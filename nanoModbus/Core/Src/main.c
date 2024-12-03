/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nanomodbus.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
nmbs_platform_conf platform_conf;

uint16_t counter = 0;
uint16_t counter2 = 0;
uint16_t read_value;
uint16_t r_regs[2];
uint16_t okunan[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RS485_SetTransmitMode(void) {
	HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_SET); // DE ve RE'yi etkinleştir
}

void RS485_SetReceiveMode(void) {
	HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_RESET); // DE ve RE'yi devre dışı bırak
}
int32_t uart_read(uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg) {
	RS485_SetReceiveMode(); // Alıcı moduna geç
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, buf, count,
			timeout_ms);
	if (status == HAL_OK)
		return count;
	return -1;
}

int32_t uart_write(const uint8_t *buf, uint16_t count, int32_t timeout_ms,
		void *arg) {
	RS485_SetTransmitMode(); // Verici moduna geç
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*) buf, count,
			timeout_ms);
	if (status == HAL_OK) {
		// Gönderme tamamlandığında tekrar alıcı moduna geç
		RS485_SetReceiveMode();
		return count;
	}
	return -1;

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	nmbs_platform_conf_create(&platform_conf);
	platform_conf.transport = NMBS_TRANSPORT_RTU;
	platform_conf.read = uart_read;
	platform_conf.write = uart_write;

	// MODBUS istemcisini oluştur
	nmbs_t nmbs;
	nmbs_error err = nmbs_client_create(&nmbs, &platform_conf);
	if (err != NMBS_ERROR_NONE) {
		Error_Handler();
	}

	// Timeout değerlerini ayarla
	nmbs_set_byte_timeout(&nmbs, 100);
	nmbs_set_read_timeout(&nmbs, 1000);

	// Hedef slave adresini ayarla (örn: 1)
	nmbs_set_destination_rtu_address(&nmbs, 1);

	// Write 2 holding registers at address 26
//  uint16_t w_regs[2] = {123, 124};
//  err = nmbs_write_multiple_registers(&nmbs, 1, 2, w_regs);
//  if (err != NMBS_ERROR_NONE) {
//      fprintf(stderr, "Error writing register at address 26 - %s", nmbs_strerror(err));
//      return 1;
//  }
	// Read 2 holding registers from address 26
//	uint16_t r_regs[2];
//	err = nmbs_read_holding_registers(&nmbs, 1, 1, r_regs);
//	if (err != NMBS_ERROR_NONE) {
//		fprintf(stderr,
//				"Error reading 2 holding registers at address 26 - %s\n",
//				nmbs_strerror(err));
//		return 1;
//	}

//		nmbs_write_single_register(&nmbs, 0, 000);
//		nmbs_write_single_register(&nmbs, 1, 413);
//		nmbs_write_single_register(&nmbs, 2, 213);

/////////

//		nmbs_read_holding_registers(&nmbs, 0, 3, okunan);

//  nmbs_write_single_register(&nmbs, 6, 6);
//  err = nmbs_write_single_register(&nmbs, 2, 9);
//  if (err != NMBS_ERROR_NONE) {
//      fprintf(stderr, "Error writing register at address 1 - %s", nmbs_strerror(err));
//      return 1;
//  }
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//				nmbs_write_single_register(&nmbs, 1, counter);
//				nmbs_write_single_register(&nmbs, 2, counter);
				uint16_t w_regs[2] = {counter, counter2};
				nmbs_write_multiple_registers(&nmbs, 1, 2, w_regs);
				counter++;
				counter2 += 2;

//		  err = nmbs_write_single_register(&nmbs, 1, counter);
//		  if (err != NMBS_ERROR_NONE) {
////		      fprintf(stderr, "Error writing register at address 1 - %s", nmbs_strerror(err));
//		      return 1;
//		  }
//		counter++;
//		HAL_Delay(1000);
//
//		err = nmbs_read_holding_registers(&nmbs, 1, 1, r_regs);
//		if (err != NMBS_ERROR_NONE) {
//			fprintf(stderr,
//					"Error reading 2 holding registers at address 26 - %s\n",
//					nmbs_strerror(err));
//			return 1;
//		}
//		uint16_t w_regs[2] = { counter, counter2 };
//		err = nmbs_write_multiple_registers(&nmbs, 1, 2, w_regs);
//		if (err != NMBS_ERROR_NONE) {
//			fprintf(stderr, "Error writing register at address 26 - %s",
//					nmbs_strerror(err));
//			return 1;
//		}
//		counter++;
//		counter2 += 5;
//		HAL_Delay(1000);
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | RS485_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin RS485_CTRL_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | RS485_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
