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
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 8 * 1280
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
int16_t i2s_txBuf[SAMPLE_SIZE];

int16_t dataBuf[SAMPLE_SIZE/4];

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define WIN_LENGTH SAMPLE_SIZE / 4
#define WIN_SHIFT WIN_LENGTH / 2
#define LPC_LEN 10
#define TRIGGER_TRESH 2000
//float *hann_coef;

typedef struct {
	int m, n;
	float ** v;
} mat_t, *mat;

mat matrix_new(int m, int n)
{
	mat x = (mat)malloc(sizeof(mat_t));
	x->v = (float**)malloc(sizeof(float*) * m);
	x->v[0] = (float*)calloc(sizeof(float), m * n);
	for (int i = 0; i < m; i++)
		x->v[i] = x->v[0] + n * i;
	x->m = m;
	x->n = n;
	return x;
}

void matrix_delete(mat m)
{
	free(m->v[0]);
	free(m->v);
	free(m);
}

void matrix_transpose(mat m)
{
	for (int i = 0; i < m->m; i++) {
		for (int j = 0; j < i; j++) {
			double t = m->v[i][j];
			m->v[i][j] = m->v[j][i];
			m->v[j][i] = t;
		}
	}
}

mat matrix_copy(int n, float a[][LPC_LEN], int m)
{
	mat x = matrix_new(m, n);
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			x->v[i][j] = a[i][j];
	return x;
}

mat matrix_mul(mat x, mat y)
{
	if (x->n != y->m) return 0;
	mat r = matrix_new(x->m, y->n);
	for (int i = 0; i < x->m; i++)
		for (int j = 0; j < y->n; j++)
			for (int k = 0; k < x->n; k++)
				r->v[i][j] += x->v[i][k] * y->v[k][j];
	return r;
}

void gram_schmidt(mat m, mat *R, mat *Q)
{
	*Q = matrix_new(m->m, m->n);
	for (int col = 0 ; col < m->m; ++col){
		// Assign vector V_i
		for (int row = 0; row < m->n; ++row){
			(*Q)->v[row][col] = m->v[row][col];
		}
		// Subtract the projections of V_i onto the previous vectors V_i-1.. V_i-2 ... V_1
		for (int proj_col = col; proj_col > 0; --proj_col){
			// compute the dot product (u_proj_col . v) and l2 norm (||u_proj_col||^2)
			float dot_prod = 0;
			float l2_norm = 0;
			for(int row = 0; row < m->n; ++row){
				float val = ((*Q)->v[row][proj_col - 1]);
				l2_norm += val * val;
				dot_prod += m->v[row][col] * ((*Q)->v[row][proj_col - 1]);
			}
			for(int row = 0; row < m->n; ++row){
				(*Q)->v[row][col] -= (((*Q)->v[row][proj_col - 1] * dot_prod) / l2_norm);
			}
		}
		// normalize the basis column vectors
		float norms[LPC_LEN] = {0};
		for (int row = 0; row < m->m; ++row){
			for(int col = 0; col < m->n; ++col){
				float val = (*Q)->v[row][col];
				norms[col] += val * val;
			}
		}
		for (int row = 0; row < m->m; ++row){
			for(int col = 0; col < m->n; ++col){
				(*Q)->v[row][col] /= sqrt(norms[col]);
			}
		}
	}
	matrix_transpose(*Q);
	*R = matrix_mul(*Q, m);
	matrix_transpose(*Q);
}
void eigenvals(mat m, float eigen[][2]){
	float epsilon = 0.0001;
	for(int i = 0; i < m->m; ++i){
		// check the entries off the diagonals for the complex eigenvals
		if(i + 1 < m->m) {
			if(fabs(m->v[i + 1][i]) > epsilon){
				float b = (m->v[i][i] + m->v[i + 1][i + 1]);
				float c =  (m->v[i][i] * m->v[i + 1][i + 1]) - (m->v[i + 1][i] * m->v[i][i + 1]);
				float complex_val = sqrt(fabs(b * b - 4 * c)) / 2;
				eigen[i][0] = eigen[i + 1][0] = b / 2;
				eigen[i][1] = complex_val;
				eigen[i + 1][1] = -complex_val;
				++i; // skip the next diagonal since it is a complex conjugate of the previous
			}
			else{
				eigen[i][0] = m->v[i][i];
				eigen[i][1] = 0;
			}
		}
		else{
			// If it isn't a complex eigenval, it must be real
			eigen[i][0] = m->v[i][i];
			eigen[i][1] = 0;
		}
	}
}

void swap(float *xp[2], float *yp[2])
{
    float real = *xp[0];
    float imag = *xp[1];
    *xp[0] = *yp[0];
    *xp[1] = *yp[1];
    *yp[0] = real;
    *yp[1] = imag;
}

void complex_mul(float xp[2], float yp[2], float out[2]){
	float a = xp[0];
	float b = xp[1];
	float c = yp[0];
	float d = yp[1];
	out[0] += a * c + -(b * d);
	out[1] += a * d + b * c;
}

void filter1(const float *b, const float *a, size_t filterLength, const float *in, float *out, size_t length) {
    const float a0 = a[0];
    const float *a_end = &a[filterLength-1];
    const float *out_start = out;
    a++;
    out--;
    size_t m;
    for (m = 0; m < length; m++) {
        const float *b_macc = b;
        const float *in_macc = in;
        const float *a_macc = a;
        const float *out_macc = out;
        float b_acc = (*in_macc--) * (*b_macc++);
        float a_acc = 0;
        while (a_macc <= a_end && out_macc >= out_start) {
            b_acc += (*in_macc--) * (*b_macc++);
            a_acc += (*out_macc--) * (*a_macc++);
        }
        *++out = (b_acc - a_acc) / a0;
        in++;
    }
}

void voice_obfuscation(int16_t *input, int16_t *output) {
	   float PI = 2 * acos(0.0);
	    float frame[WIN_LENGTH];
		float input_max = 0;
		float sum = 0;
		for(int i = 0; i < WIN_LENGTH; ++i){
			if(input_max < abs(input[i])){
				input_max = abs(input[i]);
			}
			sum += input[i];
		}
		sum /= WIN_LENGTH;

		for(int i = 0; i < WIN_LENGTH; ++i){
			frame[i] = (float)((float)input[i] - sum) / (input_max);
		}

	    // Linear predictive coding
	    // First calulate the autocovariance
	    float r_x[LPC_LEN + 1];
	    for (size_t i = 0; i < LPC_LEN + 1; ++i){
	        r_x[i] = 0;
	        for (size_t j = 0; j < WIN_LENGTH - i; ++j){
	            r_x[i] += frame[j] * frame[j + i];
	        }
	    }

	    // given the autocorrelation vector, solve for the coefficients
	    // Set up the autocovariance matrix
	    float R_x[LPC_LEN + 1][LPC_LEN + 2] = {0};
	    for (int32_t i = 0; i < LPC_LEN + 1; ++i){
	        for (int32_t j = 0; j < LPC_LEN + 1; ++j){
	            if(i + j < LPC_LEN + 1){
	                R_x[i][i + j] = r_x[j];
	            }
	            if(i - j >= 0){
	                R_x[i][i - j] = r_x[j];
	            }
	        }
	    }
	    R_x[0][LPC_LEN + 1] = 1;
	    float c = 0;
	    /* Now finding the elements of diagonal matrix */
	    for(int j = 0; j < LPC_LEN + 1; j++) {
	        for(int i = 0; i < LPC_LEN + 1; i++) {
	            if(i != j) {
	                c= R_x[i][j] / R_x[j][j];
	                for(int k = 0; k < LPC_LEN + 2; k++) {
	                    R_x[i][k] = R_x[i][k] - c * R_x[j][k];
	                }
	            }
	        }
	    }
	    float a[LPC_LEN + 1] = {0};
	    float scale = 0;
	    for(int i = 0; i < LPC_LEN + 1; ++i){
	        a[i] = R_x[i][LPC_LEN + 1] / R_x[i][i];
	    }
	    scale = a[0];
	    for(int i = 0; i < LPC_LEN + 1; ++i){
	        a[i] /= scale;
	    }

	    //build the companion matrix
	    float companion[LPC_LEN][LPC_LEN] = {0};
	    for(uint32_t j = 0; j < LPC_LEN; ++j){
	        companion[0][j] = -a[j + 1];
	    }
	    for(uint32_t i = 1; i < LPC_LEN; ++i){
	        companion[i][i - 1] = 1;
	    }

		mat m = matrix_copy(LPC_LEN, companion, LPC_LEN);
		for (size_t i = 0; i < 28; ++i){
			mat R, Q;
			gram_schmidt(m, &R, &Q);
			matrix_delete(m);
			m = matrix_mul(R, Q);
			matrix_delete(R);
			matrix_delete(Q);
		}
		float eigen[LPC_LEN][2] = {0};
		eigenvals(m, eigen);
		for(int i = 0; i < LPC_LEN; ++i){
			if(eigen[i][1] != 0){
				float angle = atan(eigen[i][1] / eigen[i][0]);
				if(angle < 0)
					angle += PI;
				angle = pow(angle, 0.9);
				if(angle > PI)
					angle = PI;
				if(angle < 0)
					angle = 0;
				float mag = sqrt(eigen[i][1] * eigen[i][1] + eigen[i][0] * eigen[i][0]);
				eigen[i][0] = mag * cos(angle);
				eigen[i][1] = mag * sin(angle);
				eigen[i + 1][0] = mag * cos(angle);
				eigen[i + 1][1] = -mag * sin(angle);
				++i;
			}
		}
		matrix_delete(m);

		float poly_coefficients[LPC_LEN + 1][2] = {0};
		poly_coefficients[0][0] = 1;
		poly_coefficients[0][1] = 0;
		poly_coefficients[1][0] = -eigen[0][0];
		poly_coefficients[1][1] = -eigen[0][1];
		for(int i = 1; i < LPC_LEN; ++i){
			float window[2][2] = {{-eigen[i][0],-eigen[i][1]}, {1,0}};
			float temp_coefficients[LPC_LEN + 1][2] = {0};
			temp_coefficients[0][0] = 1;
			temp_coefficients[0][1] = 0;
			for(int j = 0; j < i; ++j){
				complex_mul(poly_coefficients[j], window[0], temp_coefficients[j + 1]);
				complex_mul(poly_coefficients[j + 1], window[1], temp_coefficients[j + 1]);
			}
			complex_mul(poly_coefficients[i], window[0], temp_coefficients[i + 1]);
			for(int j = 0; j < i + 2; ++ j){
				poly_coefficients[j][0] = temp_coefficients[j][0];
				poly_coefficients[j][1] = temp_coefficients[j][1];
			}
		}

		float a_lpc_new[LPC_LEN + 1];
		for(int i = 0; i < LPC_LEN + 1; ++i){
			a_lpc_new[i] = poly_coefficients[i][0];
		}
		float out[WIN_LENGTH] = {0};
		filter1(a, a_lpc_new, LPC_LEN + 1, frame, out, WIN_LENGTH);
		float max_peak = 0;
		for(int i = 0; i < WIN_LENGTH; ++i){
			// out[i] *= hann_coef[i];
			if(max_peak < abs(out[i])){
				max_peak = abs(out[i]);
			}
		}
		for(int i = 0; i < WIN_LENGTH; ++i){
			output[i] = (int16_t)((out[i]) * INT16_MAX - 1) + sum;
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

  /* USER CODE BEGIN SysInit */
//	float PI = 2 * acos(0.0);
//	float *hann_coef = malloc(sizeof(float) * WIN_LENGTH);
//	float maximum_val = 0;
//	float K = 0;
//	for (size_t i = 0; i <= (WIN_LENGTH / 2); ++i){
//		float temp = 0.5 * (1 - cos((PI * 2 * i) / WIN_LENGTH));
//		hann_coef[i] = temp;
//		hann_coef[WIN_LENGTH - 1 - i] = temp;
//		K += 2 * temp;
//		maximum_val = fmax(temp, maximum_val);
//	}
//	for(size_t i = 0; i < WIN_LENGTH; ++i){
//		hann_coef[i] = sqrt((hann_coef[i] * WIN_SHIFT ) / (maximum_val * K));
//	}
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
  uint8_t prev_rxBuf_status = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  int16_t temp[WIN_LENGTH];
//  int16_t temp_out[WIN_LENGTH];
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int dataBuf_ptr = 0;
	  int txBuf_ptr = 0;

	  if(rxBuf_status == 1){
//		  if(prev_rxBuf_status == 1) exit(1); // DEBUG DOUBLING
//		  printf("rxBuf half\n");  // DEBUG PRINTMSG

		  /* READ IN MICROPHONE DATA */
		  int uter_trigger = 0;
		  for(int i = 0; i < SAMPLE_SIZE/2; i = i+2){
				dataBuf[dataBuf_ptr] = i2s_rxBuf[i];
				if(abs(dataBuf[dataBuf_ptr]) > TRIGGER_TRESH)
					uter_trigger = 1;
		  		++dataBuf_ptr;
//				debug_sample_i2s = i2s_rxBuf[i]; // DEBUG READ DATA
//				i2s_txBuf[i] = i2s_rxBuf[i]; // DIRECT DATA TRANSFER
		  }

		  /* PERFORM VOICE OBFUSCATION */
//		  HAL_Delay(15);
		  int16_t i2s_temp_buf[WIN_LENGTH];
		  txBuf_ptr = 0;
		  if(uter_trigger)
			  voice_obfuscation(dataBuf, i2s_temp_buf);
		  else{
			  for(int i = 0; i < WIN_LENGTH; ++i){
				  i2s_temp_buf[i] = 0;
			  }
		  }

		  /* TRANSFER THE OBFUSCATED AUDIO TO TX BUFFER */
		  for(int j = 0; j < SAMPLE_SIZE/4; ++j){
			  i2s_txBuf[txBuf_ptr] = i2s_temp_buf[j];
			  txBuf_ptr = txBuf_ptr + 2;
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // DEBUG TIMING

		  /* SET FLAGS */
		  rxBuf_status = 0;
		  prev_rxBuf_status = 1;
	  }

	  else if (rxBuf_status == 2){
//		  if(prev_rxBuf_status == 2) exit(1); // DEBUG DOUBLING
//		  printf("rxBuf full\n"); // DEBUG PRINTMSG

		  /* READ IN MICROPHONE DATA */
		  int uter_trigger = 0;
		  for(int i = SAMPLE_SIZE/2; i < SAMPLE_SIZE; i = i+2){
			    dataBuf[dataBuf_ptr] = i2s_rxBuf[i];
				if(abs(dataBuf[dataBuf_ptr]) > TRIGGER_TRESH)
					uter_trigger = 1;
				++dataBuf_ptr;
//				debug_sample_i2s = i2s_rxBuf[i]; // DEBUG READ DATA
//				i2s_txBuf[i] = i2s_rxBuf[i]; // DIRECT DATA TRANSFER
		  }

		  /* PERFORM VOICE OBFUSCATION */
//		  HAL_Delay(15);
		  int16_t i2s_temp_buf[WIN_LENGTH];
		  txBuf_ptr = SAMPLE_SIZE/2; // STARTS FROM THE HALF-FILLED POSITION
		  if(uter_trigger)
			  voice_obfuscation(dataBuf, i2s_temp_buf);
		  else{
			  for(int i = 0; i < WIN_LENGTH; ++i){
				  i2s_temp_buf[i] = 0;
			  }
		  }
		  /* TRANSFER THE OBFUSCATED AUDIO TO TX BUFFER */
		  for(int j = 0; j < SAMPLE_SIZE/4; ++j){
			  i2s_txBuf[txBuf_ptr] = i2s_temp_buf[j];
			  txBuf_ptr = txBuf_ptr + 2;
		  }
//		  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)i2s_txBuf, SAMPLE_SIZE); // DEBUG NON-CIRCULAR TRANSMIT
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // DEBUG TIMING

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
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
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
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
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
  hlpuart1.Init.BaudRate = 115200;
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
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	//	 printf("half callback\n");  // DEBUG PRINTMSG
		rxBuf_status = 1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // DEBUG TIMING
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	//	printf("full callback\n");  // DEBUG PRINTMSG
		rxBuf_status = 2;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // DEBUG TIMING
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
