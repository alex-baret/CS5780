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

#define THRESHOLD1 0
#define THRESHOLD2 10
#define THRESHOLD3 20
#define THRESHOLD4 30

void SystemClock_Config(void);
void setUp();
void parseData(short data);


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  setUp();

  // Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
  int index = 0;
	
  while (1)
  {
		if(index == 31){
			index = 0;
		}
		// parseData(ADC1->DR);
    DAC1->DHR8R1 = sine_table[index];
    index++;
		HAL_Delay(100);
  }
}

/**
 * Turns LEDs on/off depending on current x and y values.
 */
void parseData(short data)
{

  if (data <= THRESHOLD1)
  {
    GPIOC->ODR &= ~(1 << 6); // setting pin 6 RED to low
    GPIOC->ODR &= ~(1 << 7); // setting pin 7 BLUE to low
    GPIOC->ODR &= ~(1 << 9); // setting pin 9 GREEN to low
    GPIOC->ODR &= ~(1 << 8); // setting pin 8 ORANGE to low
  }

  else if (data > THRESHOLD1 && data < THRESHOLD2) // red threshold
  {
    GPIOC->ODR |= (1 << 6); // setting pin 6 RED to high

    GPIOC->ODR &= ~(1 << 7); // setting pin 6 BLUE to low
    GPIOC->ODR &= ~(1 << 9); // setting pin 9 GREEN to low
    GPIOC->ODR &= ~(1 << 8); // setting pin 8 ORANGE to low
  }
  else if (data > THRESHOLD2 && data < THRESHOLD3) // blue threshold
  {
    GPIOC->ODR |= (1 << 6); // setting pin 6 RED to high
    GPIOC->ODR |= (1 << 7); // setting pin 7 BLUE to high

    GPIOC->ODR &= ~(1 << 8); // setting pin 7 ORANGE to low
    GPIOC->ODR &= ~(1 << 9); // setting pin 9 GREEN to low
  }

  else if (data > THRESHOLD3 && data < THRESHOLD4) // green threshold
  {

    GPIOC->ODR |= (1 << 6); // setting pin 6 RED to high
    GPIOC->ODR |= (1 << 7); // setting pin 7 BLUE to high
    GPIOC->ODR |= (1 << 8); // setting pin 8 ORANGE to high

    GPIOC->ODR &= ~(1 << 9); // setting pin 9 GREEN to low
  }
  else if (data > THRESHOLD4) // orange threshold
  {
    GPIOC->ODR |= (1 << 6); // setting pin 6 RED to high
    GPIOC->ODR |= (1 << 7); // setting pin 7 BLUE to high
    GPIOC->ODR |= (1 << 8); // setting pin 8 ORANGE to high
    GPIOC->ODR |= (1 << 9); // setting pin 9 GREEN to high
  }
}


/**
 * @brief Performs setup needed to run the program such as initializing peripheral clocks, setting GPIO modes, etc.
 */
void setUp()
{
  HAL_Init(); // Reset of all peripherals, Initializes the Flash interface and the Systick.

  /* === Clock Settings === */
  SystemClock_Config(); // Configure the system clock

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the system clock for the GPIOC peripheral

 

  // /* === GPIO Settings === */
  // configure the LEDs Pins
  GPIOC->MODER |= (1 << 12); // setting PC6 to general output RED
  GPIOC->MODER |= (1 << 14); // setting PC7 to general output BLUE
  GPIOC->MODER |= (1 << 16); // setting PC8 to general output ORANGE
  GPIOC->MODER |= (1 << 18); // setting PC9 to general output GREEN

  GPIOC->OTYPER &= ~(0 << 7);  // setting PC6 to push/pull output
  GPIOC->OTYPER &= ~(0 << 8);  // setting PC7 to push/pull output
  GPIOC->OTYPER &= ~(0 << 9);  // setting PC8 to push/pull output
  GPIOC->OTYPER &= ~(0 << 10); // setting PC9 to push/pull output

  GPIOC->OSPEEDR &= ~(0 << 12); // setting PC6 to low speed
  GPIOC->OSPEEDR &= ~(0 << 14); // setting PC7 to low speed
  GPIOC->OSPEEDR &= ~(0 << 16); // setting PC8 to low speed
  GPIOC->OSPEEDR &= ~(0 << 18); // setting PC9 to low speed

  GPIOC->PUPDR &= ~(0 << 12); // setting PC6 to to no pull-up/down resistors
  GPIOC->PUPDR &= ~(0 << 14); // setting PC7 to to no pull-up/down resistors
  GPIOC->PUPDR &= ~(0 << 16); // setting PC8 to to no pull-up/down resistors
  GPIOC->PUPDR &= ~(0 << 18); // setting PC9 to to no pull-up/down resistors

  // Setting Pins initial states
  GPIOC->ODR &= ~(1 << 6); // setting pin 6 to low
  GPIOC->ODR &= ~(1 << 7); // setting pin 7 to low
  GPIOC->ODR &= ~(1 << 8); // setting pin 8 to low
  GPIOC->ODR &= ~(1 << 9); // setting pin 9 to low

  /* === ADC Settings === */
  // setting PC0 to analog
  GPIOC->MODER |= (1 << 0); // setting 0th
  GPIOC->MODER |= (1 << 1); // setting 1st

  // setting PC0 to to no pull-up/down resistors
  GPIOC->PUPDR &= ~(1 << 0); // clearing 0th bit
  GPIOC->PUPDR &= ~(1 << 1); // clearing 1st bit
	
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable system clock for ADCEN peripheral
  
  // 8-bit
  ADC1->CFGR1 |= (1 << 4);
  ADC1->CFGR1 &= ~(1 << 3);
	
  // continuous conversion
  ADC1->CFGR1 |= (1 << 13);
	
  // hardware triggers disabled
  ADC1->CFGR1 &= ~(1 << 10);
	ADC1->CFGR1 &= ~(1 << 11);
	
  // configuring the channel 0
  ADC1->CHSELR |= ADC_CHSELR_CHSEL10;

  /* === ADC Calibration === */
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN by setting ADDIS*/
  /* (3) Clear DMAEN */
  /* (4) Launch the calibration by setting ADCAL */
  /* (5) Wait until ADCAL=0 */

  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR |= ADC_CR_ADDIS; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0)
  {
    /* For robust implementation, add here time-out management */
  }
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;       /* (3) */
  ADC1->CR |= ADC_CR_ADCAL;              /* (4) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
  {
    /* For robust implementation, add here time-out management */
  }
	

  /* (1) Ensure that ADRDY = 0 */
  /* (2) Clear ADRDY */
  /* (3) Enable the ADC */
  /* (4) Wait until ADC ready */
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
  {
    ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
  }
  ADC1->CR |= ADC_CR_ADEN;                 /* (3) */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }
	
	//start
	ADC1->CR |= ADC_CR_ADSTART;


  /* Initializing the DAC*/
 /* === DAC Settings === */

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable the system clock for the GPIOC peripheral

  // setting PA4 to analog
  GPIOA->MODER |= (1 << 8); // setting 9th
  GPIOA->MODER |= (1 << 9); // setting 10th

  // setting PA4 to to no pull-up/down resistors
  GPIOA->PUPDR &= ~(1 << 9); // clearing 9th bit
  GPIOA->PUPDR &= ~(1 << 10); // clearing 10th bit
	
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable system clock for DAC peripheral

  // set channel 1 to software trigger
  //DAC1->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; //setting 0th for channel 1 software trigger
	
	DAC1->CR |= DAC_CR_TSEL1;

  //enable DAC1
  DAC1->CR |= DAC_CR_EN1;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
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

#ifdef USE_FULL_ASSERT
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