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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "stdio.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 1280
#define F_SAMPLE 48295
#define FREQUENCY_PARTITIONS SAMPLE_SIZE/16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

/* USER CODE BEGIN PV */
int16_t i2s_rxBuf[SAMPLE_SIZE];
uint16_t i2s_txBuf[SAMPLE_SIZE];

uint8_t streamBuf[SAMPLE_SIZE/2];
int16_t dataBuf[SAMPLE_SIZE/4];

float32_t fft_in_buf[SAMPLE_SIZE/8];
float32_t fft_out_buf[SAMPLE_SIZE/8];
uint8_t data_out_buf[12]; //10 frequency bins, 1 start byte, 1 stop byte
arm_rfft_fast_instance_f32 fft_handler;

volatile uint8_t rxBuf_status = 0; // 0 = empty, 1 = half, 2 = full
volatile int debug_sample_i2s;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
float32_t complexABS(float32_t real, float32_t imag);
void DoFFT();
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_LPUART1_UART_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)i2s_rxBuf, SAMPLE_SIZE);
  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)i2s_txBuf, SAMPLE_SIZE);
  arm_rfft_fast_init_f32(&fft_handler, SAMPLE_SIZE/8);
  uint8_t prev_rxBuf_status = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int streamBuf_ptr = 0;
	  int dataBuf_ptr = 0;

	  if(rxBuf_status == 1){
//		  if(prev_rxBuf_status == 1) exit(1); // DEBUG DOUBLING
//		  printf("rxBuf half\n");  // DEBUG PRINTMSG

		  /* READ IN MICROPHONE DATA */
		  for(int i = 0; i < SAMPLE_SIZE/2; i = i+2){
			    i2s_txBuf[i] = i2s_rxBuf[i]; // DEBUG DIRECT DATA TRANSFER
			  	streamBuf[streamBuf_ptr] = (i2s_rxBuf[i] >> 8) & 0xff;
				streamBuf[streamBuf_ptr + 1] = i2s_rxBuf[i] & 0xff;
				dataBuf[dataBuf_ptr] = i2s_rxBuf[i];
		  		++dataBuf_ptr;
		  		streamBuf_ptr = streamBuf_ptr + 2;
//		  		debug_sample_i2s = i2s_rxBuf[i]; // DEBUG READ DATA
		  }

		  /* SEND MICROPHONE DATA OVER SERIAL FOR ANALYSIS */
//		  HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)streamBuf, SAMPLE_SIZE/2);

		  /* SET FLAGS */
		  rxBuf_status = 0;
		  prev_rxBuf_status = 1;
	  }
	  else if (rxBuf_status == 2){
//		  if(prev_rxBuf_status == 2) exit(1); // DEBUG DOUBLING
//		  printf("rxBuf full\n"); // DEBUG PRINTMSG

		  /* READ IN MICROPHONE DATA */
		  for(int i = SAMPLE_SIZE/2; i < SAMPLE_SIZE; i = i+2){
			  	i2s_txBuf[i] = i2s_rxBuf[i]; // DEBUG DIRECT DATA TRANSFER
			    streamBuf[streamBuf_ptr] = (i2s_rxBuf[i] >> 8) & 0xff;
			    streamBuf[streamBuf_ptr + 1] = i2s_rxBuf[i] & 0xff;
			    dataBuf[dataBuf_ptr] = i2s_rxBuf[i];
				++dataBuf_ptr;
				streamBuf_ptr = streamBuf_ptr + 2;
//				debug_sample_i2s = i2s_rxBuf[i]; // DEBUG READ DATA
		  }

		  /* SEND MICROPHONE DATA OVER SERIAL FOR ANALYSIS */
//		  HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)streamBuf, SAMPLE_SIZE/2);

		  /* SET FLAGS */
		  rxBuf_status = 0;
		  prev_rxBuf_status = 2;
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 460800;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float32_t complexABS(float32_t real, float32_t imag){
	return sqrtf(real*real+imag*imag);
}

void DoFFT(){
	arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);

	int freqs[FREQUENCY_PARTITIONS];
	int freqptr = 0;
	int offset = 150; // noise offset

	for(int i = 0; i < SAMPLE_SIZE/8; i = i+2){
	  freqs[freqptr] = (int)(20*log10f(complexABS(fft_out_buf[i], fft_out_buf[i+1]))) - offset;
	  if(freqs[freqptr] < 0) freqs[freqptr]=0;
	  ++freqptr;
	}

	data_out_buf[0] = 0xff; // start byte
	data_out_buf[1] = (uint8_t)freqs[1]; // 31 Hz
	data_out_buf[2] = (uint8_t)freqs[3]; // 63 Hz
	data_out_buf[3] = (uint8_t)freqs[5]; // 125 Hz
	data_out_buf[4] = (uint8_t)freqs[11]; // 250 Hz
	data_out_buf[5] = (uint8_t)freqs[22]; // 500 Hz
	data_out_buf[6] = (uint8_t)freqs[44]; // 1 kHz
	data_out_buf[7] = (uint8_t)freqs[96]; // 2.2 kHz
	data_out_buf[8] = (uint8_t)freqs[197]; // 4.5 kHz
	data_out_buf[9] = (uint8_t)freqs[393]; // 9 kHz
	data_out_buf[10] = (uint8_t)freqs[655]; // 15 kHz
	data_out_buf[11] = 0xdd; // stop byte

	HAL_UART_Transmit_DMA(&hlpuart1, &data_out_buf[0], 12);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
//	 printf("half callback\n");  // DEBUG PRINTMSG
	rxBuf_status = 1;
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // DEBUG TIMING
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
//	printf("full callback\n");  // DEBUG PRINTMSG
	rxBuf_status = 2;
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // DEBUG TIMING
}

// NOTE: Can't do both UART and SWV printf at once

/* PRINTF UART SETUP*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* PRINTF SWV SETUP*/
//int _write(int file, char *ptr, int len)
//{
//  (void)file;
//  int DataIdx;
//
//  for (DataIdx = 0; DataIdx < len; DataIdx++)
//  {
//    ITM_SendChar(*ptr++);
//  }
//  return len;
//}
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
