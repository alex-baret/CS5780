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
void transmitChar(char c);
void transmitString(char string[]);
void receiveChar();
void setUp();
void parseData();
void printInstruction(char message[]);
int evalData();

volatile char color;
volatile int ledSetting = 0; 
volatile int hasData = 0;
volatile int printPrompt = 1;
volatile char multiStepErrorMessage[] = "Does not correspond to a command.  Enter one of the following: 'r0','r1','r2','g0','g1','g2','b0','b1','b2','o0','o1','o2'\0";
volatile int step = 0;
volatile char dataBuffer;


/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
setUp();

int numItrs = 0;
char prompt[] = "CMD> \0"; 
char errorMessage[] = "does not correspond to an LED.  Choose one of the following: 'R','G','B','O'.";
int toggleCount = 0;

while (1){
	if(printPrompt == 1){
     printInstruction(prompt);
  }
  printPrompt = 0;
  if (hasData){
    parseData();
    hasData = 0;
  }
  
	}
}

/**
 * @brief sets the LED color and setting based off user input
*/
void parseData(){
  int valid;
    if(step == 0){
      color = (uint8_t)(USART3->RDR);
		  transmitChar(color);
    } else if (step == 1){
      ledSetting = (uint8_t)(USART3->RDR);
		  transmitChar(ledSetting);
      valid = evalData();
      if(valid){
        transmitString("Correct info provided \0");
        printPrompt = 1;
      } 
    }
  //if it's not the last step increment it, last step reset it
  step == 0 ? step++ : (step = 0); 
}

int evalData(){
  switch(color){
  case 'r': //PC6
    // Toggle the output state of PC6U
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6); // Inverts the 6th
    break;
  case 'g': //PC9
    // Toggle the output state of PC9
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9); // Inverts the 9th
    break;
  case 'b': //PC7
    // Toggle the output state of PC7
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7); // Inverts the 7th
    break;
  case 'o': //PC8
    // Toggle the output state of PC8
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8); // Inverts the 8th
    break;
  default: //empty default
    break;
}
  return 1;
}

void printInstruction(char message[]){
  transmitString(message);
}

	
void receiveChar(){
	char errorMessage[] = "Does not correspond to an LED.  Choose one of the following: 'R','G','B','O'.\0";
	char c;	
  if(USART3->ISR & USART_ISR_RXNE){
	//when not empty, receive data
	c = (uint8_t)(USART3->RDR);
  switch(c){
    case 'r': //PC6
      // Toggle the output state of PC6U
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6); // Inverts the 6th
      break;
    case 'g': //PC9
      // Toggle the output state of PC9
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9); // Inverts the 9th
      break;
    case 'b': //PC7
      // Toggle the output state of PC7
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7); // Inverts the 7th
      break;
    case 'o': //PC8
      // Toggle the output state of PC8
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8); // Inverts the 8th
      break;
    default:
            transmitString(errorMessage);
      break;
  }
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
*@brief This function handles the USART3_4_IRQn
*/
void USART3_4_IRQHandler (void){
	hasData = 1;
  dataBuffer = (uint8_t)(USART3->RDR);
}

/**
 * @brief Performs setup needed to run the program such as initializing peripheral clocks, setting GPIO modes, etc.
*/
void setUp(){

// Reset of all peripherals, Initializes the Flash interface and the Systick.
HAL_Init();

// Configure the system clock
SystemClock_Config();

// Enable peripheral clock for USART3
RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

/* This sequence select AF4 for GPIOA10 and 11.  */
//(1) Enable the peripheral clock of GPIOB
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//(2) Select alternate function mode on GPIOB pins PC10 and PC11
GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11)) | GPIO_MODER_MODER10_1
| GPIO_MODER_MODER11_1; 

//(3) Select AF4 on PB10 in AFRH for USART3_TX
GPIOB->AFR[1] |= 0x04 << GPIO_AFRH_AFSEL10_Pos;

// (4) Select AF4 on PB11 in AFRH for USART3_RX
GPIOB->AFR[1] |= 0x04 << GPIO_AFRH_AFSEL11_Pos;

USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
USART3->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE; /* (2) */

//Enable USART interrupt
NVIC_EnableIRQ(USART3_4_IRQn);


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