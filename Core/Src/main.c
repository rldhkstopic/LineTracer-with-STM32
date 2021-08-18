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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char* p, int len)
{
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC1_SIZE 8
#define POS_SIZE 8
#define CLEAN_SIZE 10
#define ZERO_SIZE 10

#define vINIT 1000

#define WEIGHT4 350
#define WEIGHT3 250
#define WEIGHT2 150
#define WEIGHT1 50

#define kP1 6
#define kP2 kP1*1.7
#define kI 0
#define kD 5

int INDEX=0;
int reading[CLEAN_SIZE];


uint8_t Position;

uint16_t ADC1Result[ADC1_SIZE-1];
uint16_t ADC1Max[ADC1_SIZE-1];
uint16_t ADC1cMax[ADC1_SIZE-1];

uint16_t ADC1Min[ADC1_SIZE-1];
uint16_t ADC1cMin[ADC1_SIZE-1];

uint16_t ADC1Current[ADC1_SIZE-1];
uint16_t ADC1Norm[ADC1_SIZE-1];

double avg, sum;
int CleanUp(double val, int tries){
	while(tries){
		sum = sum - reading[INDEX];
		reading[INDEX] = val;
		sum = sum + val;
		INDEX = (INDEX+1) % CLEAN_SIZE;

		avg = sum / CLEAN_SIZE;
		tries--;
	}
	return avg;

}
uint32_t pressed=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){	// K0 Button Active Low (psh 0)
	pressed = !pressed;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, pressed);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_ADC_Init(&hadc1);
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

  HAL_ADC_Start_DMA(&hadc1, &ADC1Result[0], 8);
  for(int i=0;i<8;i++){ADC1Min[i] = 4095;}

  int32_t Err; 			// Current Error
  int32_t pErr;			// Previous Error
  int32_t ErrDif;		// Error Difference

  int32_t dV1;
  int32_t dV2;
  int32_t weight, wsum;
  int32_t vRight, vLeft;

  uint32_t cTime;
  uint32_t pTime;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	for(int i=0;i<8;i++){
		if(ADC1Result[i] >= ADC1Max[i]) ADC1Max[i] = ADC1Result[i];
		if(ADC1Result[i] <= ADC1Min[i] && ADC1Result[i] > 100) ADC1Min[i] = ADC1Result[i];

		ADC1Current[i] = ((ADC1Result[i]-ADC1Min[i]) * 100)/(ADC1Max[i] - ADC1Min[i]); ADC1Norm[i] = ADC1Current[i];
	}

	cTime = HAL_GetTick();
	double delay_Time = (double)(cTime - pTime);

	weight = (ADC1Norm[0]-ADC1Norm[7])*WEIGHT4 + (ADC1Norm[1]-ADC1Norm[6])*WEIGHT3 + (ADC1Norm[2]-ADC1Norm[5])*WEIGHT2 + (ADC1Norm[3]-ADC1Norm[4])*WEIGHT1;
	wsum = (ADC1Norm[0]+ADC1Norm[1]+ADC1Norm[2]+ADC1Norm[3]+ADC1Norm[4]+ADC1Norm[5]+ADC1Norm[6]+ADC1Norm[7]);

	if((ADC1Norm[3]+ADC1Norm[4]) <= 8) // Middle Point
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

	if(weight<0) Err = -(int)(- weight / wsum);
	else Err =  (int)(weight / wsum);

	ErrDif = (Err - pErr);
	dV1 = kP1 * Err + (kD / delay_Time) * ErrDif ;
	dV2 = kP2 * Err + (kD / delay_Time) * ErrDif ;

	int crMax = 13; // Cross Line Maximum Value
	if(wsum <= crMax*8)
	{
		dV1 = 0; dV2 = 0;
		printf("Cross Line!");
	}

	// if pressed==false, it can not drive out
	vRight = pressed&(vINIT - CleanUp(dV2, CLEAN_SIZE));
	vLeft  = pressed&(vINIT + CleanUp(dV1, CLEAN_SIZE));

	TIM1->CCR3 = vLeft;  // A
	TIM1->CCR4 = vRight; // B


	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  printf("\n  ");
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  pErr = Err;
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  pTime = cTime;
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  HAL_Delay(10);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
