/* USER CODE BEGIN Header */
/**
 **************************
 * @file           : main.c
 * @brief          : Main program body
 **************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 **************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

// 2d array first var = adress, second var = value
int ledArray[64][2] = {
		{0x08, 0x08},
		{0x08, 0x04},
		{0x08, 0x02},
		{0x08, 0x01},
		{0x07, 0x08},
		{0x07, 0x04},
		{0x07, 0x02},
		{0x07, 0x01},
		{0x06, 0x08},
		{0x06, 0x04},
		{0x06, 0x02},
		{0x06, 0x01},
		{0x05, 0x08},
		{0x05, 0x04},
		{0x05, 0x02},
		{0x05, 0x01},
		{0x04, 0x08},
		{0x04, 0x04},
		{0x04, 0x02},
		{0x04, 0x01},
		{0x03, 0x08},
		{0x03, 0x04},
		{0x03, 0x02},
		{0x03, 0x01},
		{0x02, 0x08},
		{0x02, 0x04},
		{0x02, 0x02},
		{0x02, 0x01},
		{0x01, 0x08},
		{0x01, 0x04},
		{0x01, 0x02},
		{0x01, 0x01},
		{0x08, 0x08},
		{0x08, 0x04},
		{0x08, 0x02},
		{0x08, 0x01},
		{0x07, 0x08},
		{0x07, 0x04},
		{0x07, 0x02},
		{0x07, 0x01},
		{0x06, 0x08},
		{0x06, 0x04},
		{0x06, 0x02},
		{0x06, 0x11},
		{0x05, 0x88},
		{0x05, 0x44},
		{0x05, 0x22},
		{0x05, 0x11},
		{0x04, 0x88},
		{0x04, 0x44},
		{0x04, 0x22},
		{0x04, 0x11},
		{0x03, 0x88},
		{0x03, 0x40},
		{0x03, 0x20},
		{0x03, 0x10},
		{0x02, 0x80},
		{0x02, 0x40},
		{0x02, 0x20},
		{0x02, 0x10},
		{0x01, 0x80},
		{0x01, 0x40},
		{0x01, 0x20},
		{0x01, 0x10}
};

/* USER CODE BEGIN PV */
uint8_t outputBuffer[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2S2_Init();
	MX_SPI3_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	MAX_Init(); // setup the max7221 chips

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	// main loop
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */


		 //Toggle LED on chip to indicate that program is running
		HAL_GPIO_TogglePin(LD6_GPIO_Port,LD6_Pin);

		// run test for LedBar
		fillTest();

		// Frans add your own code here


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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Macro to configure the PLL multiplication factor
	 */
	__HAL_RCC_PLL_PLLM_CONFIG(16);
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void)
{

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_SLAVE_RX;
	hi2s2.Init.Standard = I2S_STANDARD_MSB;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

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
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
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
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LD6_Pin|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pins : LD6_Pin PD2 PD3 PD4 */
	GPIO_InitStruct.Pin = LD6_Pin|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// PIN Layout
// GPIOD 2 = load, GPIOD 3 = clk, GPIOD 4 = data

// uses SPI interface to send a single byte
void write_byte (uint8_t byte)
{
	// for 8 bits
	for (int i =0; i<8; i++)
	{
		// pull the clock pin low
		HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, 0);
		// write the MSB bit to the data pin
		HAL_GPIO_WritePin (GPIOD, GPIO_PIN_4, byte&0x80);
		// shift left
		byte = byte<<1;
		// pull the clock pin HIGH
		HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, 1);
	}
}

// sends 4 bytes to the 2 MAX chips
void write_max(uint8_t upperAddress, uint8_t upperValue, uint8_t lowerAddress, uint8_t lowerValue)
{
	// pull the CS pin LOW
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);

	// Send the register address & value
	write_byte(upperAddress);
	write_byte(upperValue);
	write_byte(lowerAddress);
	write_byte(lowerValue);

	// pull the CS pin HIGH
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
}

// clears the LedBar
void clearDisplay()
{
	// loops trough each adress form 0x0 to 0x8
	for (int i = 0x0; i <= 0x8; i++) {
		// on the given adress i write zero
		write_max(i, 0x00, i, 0x00);
	}
}

// function that enables the leds
void enableLed(int offset, int led){
	// variable
	int Address = 0; int Data = 0;

	// checks leds in segments of 4 leds each
	for (int i = 0; i < led + 1; i++){
		// if peak led is found put value of peak led in Data variable

		// OR operation on new adress and old adress
		Address = Address | ledArray[i + offset * 4][0];

		// OR operation on new Data and old Data so multiple leds can be turned on at the same time
		Data = Data | ledArray[i + offset * 4][1];
	}

	// first max chip
	if (offset < 8){
		write_max(0, 0, Address, Data);
	}
	// second max chip
	else{
		write_max(Address, Data, 0, 0);
	}
}

// set the peak led on LedBar
void enablePeak(int peak){
	peak--;
	int setData = peak % 4, newData = 0;

	// decide which datapacket to use
	switch(setData){
	case 0:
		newData = 0x80;
		break;
	case 1:
		newData = 0x40;
		break;
	case 2:
		newData = 0x20;
		break;
	case 3:
		newData = 0x10;
		break;
	}

	// decide which shiftregister to use
	if (peak < 32){
		write_max(0, 0x00, ledArray[peak][0], newData);
	}
	else{
		write_max(ledArray[peak][0], newData, 0 , 0 );
	}
}

// sets the led bar
void fillBarTo(int average, int peak) {
	// Prevent glitches when too high values are sent to ledbar
	if (average > 64){
		average = 64;
	}
	if (peak > 64){
		peak = 64;
	}

	// for good practice clear led bar
	clearDisplay();
	// loop trough each number until average led is reached
	for (int i = 0; i < average; i++) {
		// calculate offset & rest value
		int offset = i / 4;
		int restValue = i % 4;

		// lower part of LedBar
		if (i <= 32){
			enableLed(offset, restValue);
		}
		// higher part of LedBar
		else {
			enableLed(offset, restValue);
		}
	}
	// set peak led
	enablePeak(peak);
}

// test function
void fillTest() {
	// generate random number
	int random = rand() % 65;
	// get peak number
	int peak = random + 5;
	// Fill the led-bar to this number (between 1 and 64)
	fillBarTo(random, peak);
	HAL_Delay(100);

/*
	for (int i = 0; i < 65; i++){
		enablePeak(i);
		HAL_Delay(100);
	}
*/
}

// setup for the MAX chips
void MAX_Init()
{
	write_max(0x09, 0x00, 0x09, 0x00);       //  no decoding
	write_max(0x0a, 0x0A, 0x0a, 0x0A);       //  brightness intensity
	write_max(0x0b, 0x07, 0x0b, 0x07);       //  scan limit = 8 LEDs
	write_max(0x0c, 0x01, 0x0c, 0x01);       //  power down =0,normal mode = 1
	write_max(0x0f, 0x00, 0x0f, 0x00);       //  no test display

	// clear LedBar
	clearDisplay();
}

// read I2S data
static uint32_t I2SReadData(void)
{
	uint32_t value = 0;

	int result = HAL_I2S_Receive(&hi2s2, &readAudioData, 2, 100);

	if (result == HAL_OK)
	{
		value = 1; // yaay
	}
	else
	{
		value = 0; // naay
		printf("fail, errorcode = %d\n", result);
	}
	return value;
}

void printBits(size_t const size, void const * const ptr)
{
	unsigned char *b = (unsigned char*) ptr;
	unsigned char byte;
	int i, j;

	for (i=size-1;i>=0;i--)
	{
		for (j=7;j>=0;j--)
		{
			byte = (b[i] >> j) & 1;
			printf("%u", byte);
		}
	}
	puts("");
	printf("\n");
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
