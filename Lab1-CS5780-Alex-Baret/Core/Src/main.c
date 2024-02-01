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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
//HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	
SystemClock_Config(); //Configure the system clock
	
/* This example uses HAL library calls to control
the GPIOC peripheral. You’ll be redoing this code
with hardware register access. */
	
//__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC //GIVEN

// Enable the system clock for the C peripheral	//MY CODE
RCC->AHBENR |= (1 << 19);

// Set up a configuration struct to pass to the initialization function

	
//GPIO_InitTypeDef initStr = {GPIO_PIN_7 | GPIO_PIN_6,
//GPIO_MODE_OUTPUT_PP,
//GPIO_SPEED_FREQ_LOW,
//GPIO_NOPULL};

//HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9

//configure the LEDs Pins
GPIOC->MODER |= (1 <<12); //setting PC6 to general output 
GPIOC->MODER |= (1 <<14); //setting PC7 to general output 
GPIOC->MODER |= (1 <<16); //setting PC8 to general output
GPIOC->MODER |= (1 <<18); //setting PC9 to general output

GPIOC->OTYPER |= (0 << 7);//setting PC6 to push/pull output 
GPIOC->OTYPER |= (0 << 8);//setting PC7 to push/pull output 
GPIOC->OTYPER |= (0 << 9);//setting PC8 to push/pull output
GPIOC->OTYPER |= (0 << 10);//setting PC9 to push/pull output

GPIOC->OSPEEDR |= (0 <<12); //setting PC6 to low speed
GPIOC->OSPEEDR |= (0 <<14); //setting PC7 to low speed
GPIOC->OSPEEDR |= (0 <<16); //setting PC8 to low speed
GPIOC->OSPEEDR |= (0 <<18); //setting PC9 to low speed

GPIOC->PUPDR |= (0 <<12); //setting PC6 to to no pull-up/down resistors
GPIOC->PUPDR |= (0 <<14); //setting PC7 to to no pull-up/down resistors
GPIOC->PUPDR |= (0 <<16); //setting PC8 to to no pull-up/down resistors
GPIOC->PUPDR |= (0 <<18); //setting PC9 to to no pull-up/down resistors

//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Start PC8 high
GPIOC->ODR |= (1 << 6); //setting pin 6 to high
GPIOC->ODR |= (0 << 7); //setting pin 7 to low



while (1) {
	HAL_Delay(200); // Delay 200ms
	
	// Toggle the output state of both PC8 and PC9
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_6);
	
	
	GPIOC->ODR ^= 0b001000000; // Inverts the 6th
	GPIOC->ODR ^= 0b010000000; // Inverts the 7th
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
