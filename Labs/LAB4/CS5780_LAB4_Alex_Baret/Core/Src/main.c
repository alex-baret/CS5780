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

void transmitChar(char c);
void transmitString(char string[]);
void receiveChar();




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


//Enable peripheral clock for USART3
RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
/* This sequence select AF4 for GPIOA10 and 11.  */
/* (1) Enable the peripheral clock of GPIOB */
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	
	/* (2) Select alternate function mode on GPIOB pins PC10 and PC11 */
GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11)) | GPIO_MODER_MODER10_1
| GPIO_MODER_MODER11_1; /* (2) */


/* (3) Select AF4 on PB10 in AFRH for USART3_TX*/
GPIOB->AFR[1] |= 0x04 << GPIO_AFRH_AFSEL10_Pos;
/* (4) Select AF4 on PB11 in AFRH for USART3_RX*/
GPIOB->AFR[1] |= 0x04 << GPIO_AFRH_AFSEL11_Pos;



USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
USART3->CR1 = USART_CR1_TE | USART_CR1_UE; /* (2) */
USART3->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (2) */


char c; 
int numItrs = 0;
char hello[] = "Hello World! "; 
char errorMessage[] = "does not correspond to an LED.  Choose one of the following: 'R','G','B','O'.";

int toggleCount = 0;
/* LED Pin configuration */

// Enable the system clock for the C peripheral
RCC->AHBENR |= (1 << 19);

	
//configure the LEDs Pins 
GPIOC->MODER |= (1 <<12); //setting PC6 to general output 
GPIOC->MODER |= (1 <<14); //setting PC7 to general output 
GPIOC->MODER |= (1 <<16); //setting PC8 to general output
GPIOC->MODER |= (1 <<18); //setting PC9 to general output

GPIOC->OTYPER &= ~(0 << 7);//setting PC6 to push/pull output 
GPIOC->OTYPER &= ~(0 << 8);//setting PC7 to push/pull output 
GPIOC->OTYPER &= ~(0 << 9);//setting PC8 to push/pull output
GPIOC->OTYPER &= ~(0 << 10);//setting PC9 to push/pull output

GPIOC->OSPEEDR &= ~(0 <<12); //setting PC6 to low speed
GPIOC->OSPEEDR &= ~(0 <<14); //setting PC7 to low speed
GPIOC->OSPEEDR &= ~(0 <<16); //setting PC8 to low speed
GPIOC->OSPEEDR &= ~(0 <<18); //setting PC9 to low speed

GPIOC->PUPDR &= ~(0 <<12); //setting PC6 to to no pull-up/down resistors
GPIOC->PUPDR &= ~(0 <<14); //setting PC7 to to no pull-up/down resistors
GPIOC->PUPDR &= ~(0 <<16); //setting PC8 to to no pull-up/down resistors
GPIOC->PUPDR &= ~(0 <<18); //setting PC9 to to no pull-up/down resistors

// Setting Pins initial states
GPIOC->ODR &= ~(1 << 6); //setting pin 6 to low
GPIOC->ODR &= ~(1 << 7); //setting pin 7 to low
GPIOC->ODR &= ~(1 << 8); //setting pin 8 to low
GPIOC->ODR &= ~(1 << 9); //setting pin 9 to low

  while (1)
  {

		receiveChar();

	}
}
	
void receiveChar(){
	char errorMessage[] = "does not correspond to an LED.  Choose one of the following: 'R','G','B','O'.\0";
	char c = 'x';
	
		while(1){
			if(USART_ISR_RXNE){
				break;
			}
		}
					//when not empty, receive data
		//c = (uint8_t)(USART3->RDR);
		//transmitChar(c);
		
			switch(USART3->RDR){
				case 'r': //PC6
					// Toggle the output state of PC6
					GPIOC->ODR ^= 0b001000000; // Inverts the 6th
					GPIOC->ODR &= ~((1 << 7) | (1 << 8) | (1 << 9)); //clears all the other LEDs
					break;
				case 'g': //PC9
					// Toggle the output state of PC9
					GPIOC->ODR ^= 0b01000000000; // Inverts the 9th
					GPIOC->ODR &= ~((1 << 7) | (1 << 8) | (1 << 6)); //clears all the other LEDs
					break;
				case 'b': //PC7
					// Toggle the output state of PC7
					GPIOC->ODR ^= 0b010000000; // Inverts the 7th
					GPIOC->ODR &= ~((1 << 6) | (1 << 8) | (1 << 9)); //clears all the other LEDs
					break;
				case 'o': //PC8
					// Toggle the output state of PC8
					GPIOC->ODR ^= 0b0100000000; // Inverts the 8th
					GPIOC->ODR &= ~((1 << 7) | (1 << 6) | (1 << 9)); //clears all the other LEDs
					break;
				default:
					transmitString(errorMessage);
					break;
			}
}


/**
* @brief transmits a char through USART3
* @retval None
*/
void transmitChar(char c){
	
	while(!(USART3->ISR & USART_ISR_TXE)){
	}
	USART3->TDR = c;
}

/**
* @brief transmits a string through USART3
* @retval None
*/
void transmitString(char string[]){
		int idx = 0;
    while(string[idx] != '\0'){
			transmitChar(string[idx]);
			idx++;
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
