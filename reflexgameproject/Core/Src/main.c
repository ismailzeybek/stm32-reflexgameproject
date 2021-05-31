/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "time.h"
#include <stdio.h>
#include "LCD.h"
#include "string.h"

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

char mesaj[16];
int count = 0;

char tx_buffer[32];
int isimGirildiBayrak = 0;
int ledbayragi = 0;
int hatabayragi = 0;
int yarismaci1bayrak = 0;
int yarismaci2bayrak = 0;
int saniye1 = 0;
int saniye2 = 0;
int baslabayragi = 0;
char veri[16] = { 0 };
char veri2[16] = { 0 };

char isim1[16];
char isim2[16];
char Rx_data[1];
char message[16];
char gelcek[16];
int isimAlmaBayrak = 0;
int i = 0;
int isim1skor = 0;
int isim2skor = 0;
char isim1sayac[4];
char isim2sayac[4];
char isim11[16];
char isim22[16];
char isim11sayac[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void mydelay(int delay);
void kirmalimydelay(int delay);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void kirmalimydelay(int delay) {
	htim9.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim9);
	while (htim9.Instance->CNT < delay) {
		if (hatabayragi == 1) {
			break;
		}
	}
}
void mydelay(int delay) {
	htim9.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim9);
	while (htim9.Instance->CNT < delay) {
	}
}

void isimsiz1_hatalibasim_yazdirma(void) {
	lcd_clear();
	mydelay(100);
	hatabayragi = 1;
	lcd_print(1, 1, "1.Yarismaci");
	mydelay(100);
	lcd_print(2, 1, "Elendi...");
	mydelay(5000);
}
void isimsiz2_hatalibasim_yazdirma(void) {
	lcd_clear();
	mydelay(100);
	lcd_print(1, 1, "2.Yarismaci");
	mydelay(100);
	hatabayragi = 1;
	lcd_print(2, 1, "Elendi...");
	mydelay(5000);
}
void isimli1_hatalibasim_yazdirma(void) {
	lcd_clear();
	mydelay(100);
	hatabayragi = 1;
	lcd_print(1, 1, isim1);
	mydelay(100);
	lcd_print(2, 1, "Elendi...");
	mydelay(5000);

}
void isimli2_hatalibasim_yazdirma(void) {
	lcd_clear();
	mydelay(100);
	lcd_print(1, 1, isim2);
	mydelay(100);
	hatabayragi = 1;
	lcd_print(2, 1, "Elendi...");
	mydelay(5000);

}
void isimli_sureleri_yazdir(void) {
	lcd_clear();
	mydelay(100);
	lcd_print(1, 1, isim1);
	mydelay(100);
	sprintf(mesaj, "%d msn", saniye1);
	lcd_print(1, 9, mesaj);
	mydelay(100);
	lcd_print(2, 1, isim2);
	sprintf(mesaj, "%d msn", saniye2);
	lcd_print(2, 9, mesaj);
	mydelay(3000);

}
void isimli_kazanan_1_yazdirma_gonderme(void) {
	lcd_clear();
	mydelay(200);
	lcd_print(1, 1, isim1);
	mydelay(100);
	lcd_print(2, 1, "Kazandi.");
	strcpy(isim11, isim1);
	strcat(isim11, "|");
	HAL_UART_Transmit(&huart3, isim11, strlen(isim11), 1000);
	isim1skor = isim1skor + 1;
	itoa(isim1skor, isim1sayac, 10);
	itoa(isim2skor, isim2sayac, 10);
	strcpy(isim11sayac, isim1sayac);
	strcat(isim11sayac, "|");
	HAL_UART_Transmit(&huart3, isim11sayac, strlen(isim11sayac), 1000);
	HAL_UART_Transmit(&huart3, isim2sayac, strlen(isim2sayac), 1000);
	strcpy(isim11, "");
	mydelay(3000);

}
void isimli_kazanan_2_yazdirma_gonderme(void) {
	lcd_clear();
	mydelay(100);
	lcd_print(1, 1, isim2);
	mydelay(100);
	lcd_print(2, 1, "Kazandi.");
	strcpy(isim22, isim2);
	strcat(isim22, "|");
	HAL_UART_Transmit(&huart3, isim22, strlen(isim22), 1000);
	isim2skor = isim2skor + 1;
	itoa(isim1skor, isim1sayac, 10);
	itoa(isim2skor, isim2sayac, 10);
	strcpy(isim11sayac, isim1sayac);
	strcat(isim11sayac, "|");
	HAL_UART_Transmit(&huart3, isim11sayac, strlen(isim11sayac), 1000);
	HAL_UART_Transmit(&huart3, isim2sayac, strlen(isim2sayac), 1000);
	mydelay(3000);
}
void isimsiz_sureleri_yazdir() {
	lcd_clear();
	mydelay(100);
	sprintf(mesaj, "1.time : %d msn", saniye1);
	lcd_print(1, 1, mesaj);
	mydelay(100);
	sprintf(mesaj, "2.time : %d msn", saniye2);
	lcd_print(2, 1, mesaj);
	mydelay(3000);
}
void isimsiz_kazanan_1_yazdirma(void) {
	lcd_clear();
	mydelay(100);
	lcd_print(1, 1, "1.Yarismaci");
	mydelay(100);
	lcd_print(2, 1, "Kazandi.");
	mydelay(3000);

}
void isimsiz_kazanan_2_yazdirma(void) {
	lcd_clear();
	mydelay(100);
	lcd_print(1, 1, "2.Yarismaci");
	mydelay(100);
	lcd_print(2, 1, "Kazandi.");
	mydelay(3000);

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
	MX_I2C1_Init();
	MX_I2S3_Init();
	MX_SPI1_Init();
	MX_USB_HOST_Init();
	MX_TIM2_Init();
	MX_TIM9_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);
	mydelay(50);
	lcd_clear();
	mydelay(50);
	srand(time(NULL));
	HAL_UART_Receive_IT(&huart3, Rx_data, 1);
	strcpy(isim1, "");
	strcpy(isim2, "");
	strcpy(gelcek, "");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

		/* USER CODE BEGIN 3 */
		HAL_TIM_Base_Stop_IT(&htim2);
		ledbayragi = 0;
		HAL_GPIO_WritePin(baslamaled_GPIO_Port, baslamaled_Pin, 0);
		hatabayragi = 0;
		baslabayragi = 0;
		yarismaci1bayrak = 0;
		yarismaci2bayrak = 0;
		saniye1 = 0;
		saniye2 = 0;
		lcd_clear();
		mydelay(100);
		lcd_print(1, 1, "Basla Veya");
		mydelay(100);
		lcd_print(2, 1, "Isim Gir...");
		mydelay(100);
		while (1) {
			if (isimAlmaBayrak == 1) {
				strcpy(isim1, message);
				isimAlmaBayrak = 0;
				isim1skor = 0;
				strcpy(message, "");
				isim2skor = 0;
				break;
			}
			if (baslabayragi == 1) {
				break;
			}
		}
		mydelay(50);
		while (1) {
			if (isimAlmaBayrak == 1) {
				strcpy(isim2, message);
				isimAlmaBayrak = 0;
				isim1skor = 0;
				isim2skor = 0;
				isimGirildiBayrak = 1;
				strcpy(message, "");
				break;
			}
			if (baslabayragi == 1) {
				break;
			}
		}
		lcd_clear();
		mydelay(100);
		lcd_print(1, 1, isim1);
		mydelay(100);
		lcd_print(1, 13, "VS");
		mydelay(100);
		lcd_print(2, 1, isim2);
		while (1) {
			if (baslabayragi == 1) {
				break;
			}
		}
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {

	/* USER CODE BEGIN I2S3_Init 0 */

	/* USER CODE END I2S3_Init 0 */

	/* USER CODE BEGIN I2S3_Init 1 */

	/* USER CODE END I2S3_Init 1 */
	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S3_Init 2 */

	/* USER CODE END I2S3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 9;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 15999;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			CS_I2C_SPI_Pin | LCD_RS_Pin | LCD_EN_Pin | LCD_D4_Pin | LCD_D5_Pin
					| LCD_D6_Pin | LCD_D7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(baslamaled_GPIO_Port, baslamaled_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : CS_I2C_SPI_Pin LCD_RS_Pin LCD_EN_Pin LCD_D4_Pin
	 LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LCD_RS_Pin | LCD_EN_Pin | LCD_D4_Pin
			| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : yarismaci2_Pin yarismaci1_Pin */
	GPIO_InitStruct.Pin = yarismaci2_Pin | yarismaci1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : baslamabuton_Pin */
	GPIO_InitStruct.Pin = baslamabuton_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(baslamabuton_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : baslamaled_Pin */
	GPIO_InitStruct.Pin = baslamaled_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(baslamaled_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (Rx_data[0] == '.') {
		strcpy(message, (char*) gelcek);
		strcpy(gelcek, "");
		isimAlmaBayrak = 1;
	} else {
		strcat(gelcek, (char*) Rx_data);
	}
	HAL_UART_Receive_IT(&huart3, Rx_data, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	count = count + 10;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == yarismaci1_Pin) {
		if (ledbayragi == 0) {
			if (isimGirildiBayrak == 0) {
				isimsiz1_hatalibasim_yazdirma();

			} else if (isimGirildiBayrak == 1) {
				isimli1_hatalibasim_yazdirma();

			}
		} else {
			lcd_clear();
			saniye1 = count;
			yarismaci1bayrak = 1;
		}
	}
	if (GPIO_Pin == yarismaci2_Pin) {
		if (ledbayragi == 0) {
			if (isimGirildiBayrak == 0) {
				isimsiz2_hatalibasim_yazdirma();
			} else if (isimGirildiBayrak == 1) {
				isimli2_hatalibasim_yazdirma();
			}
		} else {
			lcd_clear();
			saniye2 = count;
			yarismaci2bayrak = 1;
		}
	}
	if (GPIO_Pin == baslamabuton_Pin) {
		baslabayragi = 1;
		ledbayragi = 0;
		hatabayragi = 0;
		mydelay(100);
		lcd_clear();
		mydelay(100);
		lcd_print(1, 1, "Hizli Olan");
		mydelay(100);
		lcd_print(2, 1, "Kazansin!");
		HAL_GPIO_WritePin(baslamaled_GPIO_Port, baslamaled_Pin, 0);
		int sayi = (rand() % 10) + 5;
		kirmalimydelay(sayi * 1000);
		if (hatabayragi == 1) {
		} else {
			count = 0;
			htim2.Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_GPIO_WritePin(baslamaled_GPIO_Port, baslamaled_Pin, 1);
			ledbayragi = 1;
			while (1) {
				if (yarismaci1bayrak == 1 && yarismaci2bayrak == 1) {
					if (isimGirildiBayrak == 1) {
						isimli_sureleri_yazdir();
						if (saniye1 < saniye2) {
							isimli_kazanan_1_yazdirma_gonderme();
						} else {
							isimli_kazanan_2_yazdirma_gonderme();
						}
						break;
					} else {
						isimsiz_sureleri_yazdir();

						if (saniye1 < saniye2) {
							isimsiz_kazanan_1_yazdirma();
						} else {
							isimsiz_kazanan_2_yazdirma();
						}
						break;
					}
				}
			}

		}

	}

}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
