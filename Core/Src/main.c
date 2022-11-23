/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_ADDRESS 0x58
#define I2C_READ 0x1
#define I2C_WRITE 0x0
#define INIT_AIR_QUALITY 0x2003
#define MEASURE_AIR_QUALITY 0x2008
#define GET_BASELINE 0x2015
#define SET_BASELINE 0x201e
#define SET_HUMIDITY 0x2061
#define MEASURE_TEST9 0x2032
#define GET_FEATURE_SET_VERSION 0x202f
#define MEASURE_RAW_SIGNALS 0x2050
#define GET_SERIAL_ID 0x3682

#define CO2_TRIGGER_LEVEL 1000
#define MAINTENANCE_TRIGGER 5
#define TIME_RESET 5000

#define CRC8_INIT 0xff
#define CRC8_POLYNOMIAL 0x31
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_Delay() {
	for (int i = 0; i < 5000; ++i)
		__NOP();
}

void SCL_nodelay(int value) {
	HAL_GPIO_WritePin(SOFT_SCL_GPIO_Port, SOFT_SCL_Pin, value ? 1 : 0);
}

void SCL(int value) {
	SCL_nodelay(value);
	I2C_Delay();
}

void SDA_nodelay(int value) {
	HAL_GPIO_WritePin(SOFT_SDA_GPIO_Port, SOFT_SDA_Pin, value ? 1 : 0);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, value ? 1 : 0);
}

void SDA(int value) {
	SDA_nodelay(value);
	I2C_Delay();
}

void I2C_Send_Byte(uint8_t x) {
	for (int i = 7; i >= 0; --i) {
		SDA(x & (1 << i));
		SCL(1);
		SCL(0);
		SDA_nodelay(0);
	}

	// "Read" ACK/NACK bit.
	SDA(1);
	SCL(1);
	SCL(0);
	SDA(0);
}

void I2C_Send_Short(uint16_t x) {
	I2C_Send_Byte(x >> 8);
	I2C_Send_Byte(x & 0xff);
}

void I2C_Init() {
	for (int i = 0; i < 100; ++i) {
		I2C_Delay();
	}
	SDA_nodelay(1);
	SCL(1);
}

void I2C_Start() {
	SDA(0);
	SCL(0);
}

void I2C_Write(uint16_t value) {
	I2C_Send_Byte((TARGET_ADDRESS << 1) | I2C_WRITE);
	I2C_Send_Short(value);
}

// Send an ACK condition by pulsing SCL while pulling SDA LOW.
void I2C_ACK() {
	SDA(0);
	SCL(1);
	SCL(0);
	SDA(1);
}

// Send a NACK condition by pulsing SCL while leaving SDA pulled HIGH.
void I2C_NACK() {
	SCL(1);
	SCL(0);
}

// Send the repeated START condition Sr by ensuring both pins are HIGH and then sending the START condition.
void I2C_Restart() {
	SDA_nodelay(1);
	SCL(1);
	I2C_Start();
}

// Send the STOP condition by pulling SDA followed by SCL HIGH.
void I2C_Stop() {
	SCL(0);
	SDA(0);
	SDA(1);
	SCL(1);
}

void Serial_Send(char *data) {
	size_t len = strlen(data);
	HAL_UART_Transmit(&huart2, (uint8_t*) data, len, HAL_MAX_DELAY);
}

uint8_t crc8(uint8_t *data, size_t capacity) {
	uint8_t crc = CRC8_INIT;
	for (size_t i = 0; i < capacity; ++i) {
		crc ^= data[i];
		for (int j = 0; j < 8; ++j) {
			if ((crc & (1 << 7))) {
				crc <<= 1;
				crc ^= CRC8_POLYNOMIAL;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

// Reads bytes of data then reads one more byte for CRC checking.
// Data is returned in the *data parameter which is guaranteed to
// stay the same if the CRC was invalid.
unsigned int I2C_Read(size_t bytes, unsigned int *data) {
	Serial_Send("-------- BEGIN READ --------\r\n");
	I2C_Send_Byte((TARGET_ADDRESS << 1) | I2C_READ);
	SDA(1);
	unsigned int read = 0;
	for (int i = 0; i < bytes; ++i) {
		for (int bit = 0; bit < 8; ++bit) {
			SCL(1);
			if (HAL_GPIO_ReadPin(SOFT_SDA_GPIO_Port, SOFT_SDA_Pin)) {
				Serial_Send("1");
				read |= 1;
				read <<= 1;
			} else {
				Serial_Send("0");
				read <<= 1;
			}
			SCL(0);
		}
		Serial_Send(" (read) \r\n");
		I2C_ACK();
	}
	// NOTE: multiplication by 2 for some reason.
	read /= 2;

	unsigned int crc = 0;
	for (int bit = 0; bit < 8; ++bit) {
		SCL(1);
		if (HAL_GPIO_ReadPin(SOFT_SDA_GPIO_Port, SOFT_SDA_Pin)) {
			Serial_Send("1");
			crc |= 1;
			crc <<= 1;
		} else {
			Serial_Send("0");
			crc <<= 1;
		}
		SCL(0);
	}
	I2C_NACK();

	// NOTE: multiplication by 2 for some reason.
	crc /= 2;
	Serial_Send(" (crc) \r\n");

	char str[64];
	sprintf(str, "%d (value)\r\n", *data);
	Serial_Send(str);

	// NOTE: Assumes 2 bytes of data are read.
	uint8_t buf[2] = { read >> 8, read & 0xff };
	if (crc8(buf, 2) != crc) {
		Serial_Send("Bad CRC!\r\n");
		return 1;
	}

	// Only update value on valid CRC.
	*data = read;
	return 0;
}

// Controls the buzzer.
void Buzzer(unsigned int time, unsigned int co2, unsigned int badcrc) {
	if (co2 > CO2_TRIGGER_LEVEL) {
		if (time % 800 < 400) {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		}
	} else {
		if (badcrc > MAINTENANCE_TRIGGER) {
			if (time < TIME_RESET / 10 || (time > TIME_RESET / 2 && time < TIME_RESET / 2 + TIME_RESET / 10)) {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period);
			} else {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		}
	}
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
	MX_USART2_UART_Init();
	MX_SPI3_Init();
	MX_TIM3_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
	Serial_Send("Hello, Board!\r\n");
	HAL_Delay(1000);
	Serial_Send("Attempting to mount SD card.\r\n");
	FATFS fs;
	FRESULT fresult = f_mount(&fs, "", 1);
	if (fresult != FR_OK) {
		char str[64];
		sprintf(str, "%d\r\n", fresult);
		Serial_Send(str);
		Serial_Send("Failure mounting!\r\n");
	} else {
		Serial_Send("Success mounting!\r\n");
	}

	FIL fil;
	fresult = f_open(&fil, "log.txt", FA_WRITE | FA_OPEN_APPEND | FA_CREATE_ALWAYS);
	if (fresult != FR_OK) {
		char str[64];
		sprintf(str, "%d\r\n", fresult);
		Serial_Send(str);
		Serial_Send("Failure opening!\r\n");
	} else {
		Serial_Send("Success opening!\r\n");
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	I2C_Init();
	I2C_Start();
	I2C_Write(INIT_AIR_QUALITY);
	I2C_Stop();

	// Delay before valid measurements.
	for (int i = 0; i < 15; ++i) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		HAL_Delay(500);
	}

	unsigned int all_time = 0;
	unsigned int time = 0;
	unsigned int co2 = 400;
	unsigned int badcrc = 0;
	while (1) {
		if (time % (TIME_RESET / 10) == 0) {
			I2C_Start();
			I2C_Write(MEASURE_AIR_QUALITY);
			I2C_Restart();
			if (I2C_Read(2, &co2)) {
				++badcrc;
			} else {
				badcrc = 0;

				UINT bytesWrote;
				BYTE str[35];
				sprintf(str, "%15d,%15d\r\n", all_time, co2);
				fresult = f_write(&fil, str, 35, &bytesWrote);
				if (fresult == FR_OK) {
					Serial_Send("Wrote bytes");
				} else {
					char str[64];
					sprintf(str, "%d\r\n", fresult);
					Serial_Send(str);
					Serial_Send("Failed");
				}
			}
			I2C_Stop();
		}

		Buzzer(time, co2, badcrc);

		HAL_Delay(1);
		++time;
		++all_time;
		time %= TIME_RESET;
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
			Serial_Send("Goodbye!");
			for (int i = 0; i < 3; ++i) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period);
				HAL_Delay(50);

				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				HAL_Delay(50);
			}
			break;
		}
	}

	f_close(&fil);
	f_mount(NULL, "", 0);
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | SOFT_SCL_Pin | SOFT_SDA_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : SOFT_SCL_Pin SOFT_SDA_Pin */
	GPIO_InitStruct.Pin = SOFT_SCL_Pin | SOFT_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI3_CS_Pin */
	GPIO_InitStruct.Pin = SPI3_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
