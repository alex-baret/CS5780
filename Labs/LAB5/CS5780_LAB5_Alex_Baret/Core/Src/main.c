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

void SystemClock_Config(void);
void setUp();




/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
setUp();

while (1){

  
	}
}






// /**
// *@brief This function handles the USART3_4_IRQn
// */
// void USART3_4_IRQHandler (void){
// 	hasData = 1;
//   dataBuffer = (uint8_t)(USART3->RDR);
// }

/**
 * @brief Performs setup needed to run the program such as initializing peripheral clocks, setting GPIO modes, etc.
*/
void setUp(){

HAL_Init(); // Reset of all peripherals, Initializes the Flash interface and the Systick.

/* === Clock Settings === */

SystemClock_Config(); // Configure the system clock

RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //Enable the system clock for the GPIOC peripheral
RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the system clock for the GPIOC peripheral
RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable system clock for I2C2EN peripheral


/* === GPIO Settings === */
/* This sequence selects AF1 for GPIOB11, AF5 for GPIOB13, and general output for PB14 and PC0  */

GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13)) | GPIO_MODER_MODER11_1
| GPIO_MODER_MODER13_1; // Select alternate function mode on GPIOB pins PC11 and PC13

GPIOB->AFR[1] |= 0x01 << GPIO_AFRH_AFSEL11_Pos; // Select AF1 on PB11 in AFRH for I2C2_SDA
GPIOB->AFR[1] |= 0x05 << GPIO_AFRH_AFSEL13_Pos; // Select AF5 on PB13 in AFRH for I2C2_SCL

GPIOB->OTYPER |= (1 << 11);//setting PB11 to open-drain [1]
GPIOB->OTYPER |= (1 << 13);//setting PB13 to open-drain [1]

//setting PB14 to general output [29-28] = [01]
GPIOB->MODER |= (1 <<28); //setting 28th
GPIOB->MODER &= ~(1 << 29) ; //clearing 29th

GPIOB->OTYPER &= ~(1 << 14);//setting PB14 to push/pull output (clearing 14th bit for PB14 in OTYPER)

GPIOB->ODR |= (1 << 14); //setting pin 14 to high

//setting PC0 to general output [29-28] = [01]
GPIOC->MODER |= (1 << 1); //setting 1st
GPIOC->MODER &= ~(1 << 0) ; //clearing 0th

GPIOC->OTYPER &= ~(1 << 0);//setting PC0 to push/pull output (clearing 0th bit for PC0 in OTYPER)

GPIOC->ODR |= (1 << 0); //setting pin 0 to high


/* I2C Settings */

//Setting TIMINGR register parameters to 100kHz standard-mode I2C
I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos); 
I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);

I2C2->CR1 |= I2C_CR1_PE; //enable I2C2 peripheral

//I2C2->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (I2C2_OWN_ADDRESS << 1); /* (3) */

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