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

#define WRITE 1
#define READ 0
#define DEVICE_ADDR 0x69
#define WHOAMI_VALUE 0xD3
#define WHOAMI_ADDR 0x0F
#define CTRL_REG1_ADDR	0x20
#define CNRL_REG1_CONFIG_VALS 0x0B  // normal op mode, x & y axes enabled, default low-speed mode
#define X_AXIS_COMBINED_ADDR	0xA8
#define OUT_X_L_ADDR	0x28
#define OUT_X_H_ADDR	0x29
#define Y_AXIS_COMBINED_ADDR	0xAA
#define OUT_Y_L_ADDR	0x2A
#define OUT_Y_H_ADDR	0x2B
#define THRESHOLD 50

void SystemClock_Config(void);
void setUp();
void reloadCR2Params(int readOrWrite, int slaveAddr);
void checkBreakpoint(int step);
void errorLed(int red, int orange);
int read();
void write();
void initGyro();
void parseData(char axis, int data);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  setUp();
  initGyro();

  while (1)
  {
		checkBreakpoint(1);
    // Read from x-axis
    write(1,X_AXIS_COMBINED_ADDR);
    int dataX = read(1);
    parseData('x',dataX);

    // Read from y-axis
    write(1,Y_AXIS_COMBINED_ADDR);
    int dataY = read(1);
    parseData('y',dataY);

    HAL_Delay(100);

  }

}

/**
 * Initializes Gyroscope
*/
void initGyro(){
  write(1,CTRL_REG1_ADDR); //setup write to CTRL_REG1
  write(0,CNRL_REG1_CONFIG_VALS); //writing config values into CTRL_REG1
	checkBreakpoint(1);
}

/**
 * Configures the CR2 register for I2C read/write
 */
void reloadCR2Params(int readOrWrite, int slaveAddr)
{
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // Clear the NBYTES and SADD bit field. The NBYTES field begins at bit 16, the SADD at bit 0
  I2C2->CR2 |= (1 << 16) | (slaveAddr << 1);   // Set NBYTES = 1 and SADD = 0x69
  if (readOrWrite)
  {                          // if true writing
    I2C2->CR2 &= ~(1 << 10); // clearing the RD_WRN bit to indicate a write operation
  }
  else
  {
    I2C2->CR2 |= (1 << 10); // setting the RD_WRN bit to indicate a read operation
  }
}

void parseData(char axis, int data){
  //counter-clockwise = positive, clockwise = negative
    if (axis == 'x') 
    {
      if(data > THRESHOLD){
          // Blue
          GPIOC->ODR |= (1 << 7); // setting pin 7 BLUE to high
          GPIOC->ODR &= ~(1 << 6); // setting pin 6 RED to low
      }
      else if(data < -THRESHOLD){
          // Red
          GPIOC->ODR |= (1 << 6); // setting pin 6 RED to high
          GPIOC->ODR &= ~(1 << 7); // setting pin 7 BLUE to low
      }
    }
    else if (axis == 'y')
    {
      if(data > THRESHOLD){
          // Green
          GPIOC->ODR |= (1 << 9); // setting pin 9 GREEN to high
          GPIOC->ODR &= ~(1 << 8); // setting pin 8 ORANGE to low
      }
      else if(data < -THRESHOLD){
          // Orange
          GPIOC->ODR |= (1 << 8); // setting pin 8 ORANGE to high
          GPIOC->ODR &= ~(1 << 9); // setting pin 9 GREEN to low
      }
    }
}

int read(int restartNeeded)
{
  //reloadCR2Params(READ, DEVICE_ADDR);
  if (restartNeeded)
  {
	reloadCR2Params(READ, DEVICE_ADDR);
  I2C2->CR2 |= I2C_CR2_START; // perform a restart condition
  }
  

  while (!(I2C2->ISR & I2C_ISR_NACKF) && !((I2C2->ISR >> 2) & 1))
  {
  } // #6 waiting for NACKF or RXNE
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    errorLed(1, 0);
  }
  else if (I2C2->ISR & I2C_ISR_RXNE) // continue if RXNE is set
  {
    checkBreakpoint(4);
    while (!((I2C2->ISR >> 6) & 1))
    {                         // waiting for TC flag #4
      GPIOC->ODR |= (1 << 8); // transmitting
      HAL_Delay(200);         // leave it on for 0.2 seconds
      GPIOC->ODR &= ~(1 << 8);
    }
    I2C2->CR2 |= I2C_CR2_STOP;
    return I2C2->RXDR;
  }
	return 0;
}

void write(int restartNeeded, int data)
{
  //reloadCR2Params(WRITE, DEVICE_ADDR);
  if (restartNeeded)
  {
	reloadCR2Params(WRITE, DEVICE_ADDR);
  I2C2->CR2 |= I2C_CR2_START; // perform a restart condition
  }

  while(!(I2C2->ISR & I2C_ISR_NACKF || I2C2->ISR & I2C_ISR_TXIS)){}
  if (I2C2->ISR & I2C_ISR_NACKF)
  { // Slave did not respond to the address frame. Maybe a wiring or configuration error.
    errorLed(1, 0);
  }
  else if (I2C2->ISR & I2C_ISR_TXIS)
  {
    // write data into TXDR
    checkBreakpoint(2);
    I2C2->TXDR = data; // write the data

    while (!(I2C2->ISR & I2C_ISR_TC))
    {                         // waiting for TC flag #4
      GPIOC->ODR |= (1 << 8); // transmitting
      HAL_Delay(200);         // leave it on for 0.2 seconds
      GPIOC->ODR &= ~(1 << 8);
    }
  }
}

/**
 * Blinks blue LED to signify breakpoint step (whether code reaches that step or not)
 */
void checkBreakpoint(int step)
{
  HAL_Delay(500);
  for (int i = 0; i < step; i++)
  {
    GPIOC->ODR |= (1 << 7); // blue LED debugging
    HAL_Delay(200);
    GPIOC->ODR &= ~(1 << 7);
    HAL_Delay(200);
  }
  HAL_Delay(500);
}

void errorLed(int red, int orange)
{
  if (red)
  {
    GPIOC->ODR |= (1 << 6); // error, turn on red LED
    HAL_Delay(200);         // leave it on for 0.2 seconds
    GPIOC->ODR &= ~(1 << 6);
  }
  if (orange)
  {
    GPIOC->ODR |= (1 << 8); // error, turn on red LED
    HAL_Delay(200);         // leave it on for 0.2 seconds
    GPIOC->ODR &= ~(1 << 8);
  }
}

/**
 * @brief Performs setup needed to run the program such as initializing peripheral clocks, setting GPIO modes, etc.
 */
void setUp()
{

  int slaveAddr = 0x69;

  HAL_Init(); // Reset of all peripherals, Initializes the Flash interface and the Systick.

  /* === Clock Settings === */

  SystemClock_Config(); // Configure the system clock

  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable the system clock for the GPIOC peripheral
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  // Enable the system clock for the GPIOC peripheral
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable system clock for I2C2EN peripheral

  /* === GPIO Settings === */
  /* This sequence selects AF1 for GPIOB11, AF5 for GPIOB13, and general output for PB14 and PC0  */

  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13)) | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1; // Select alternate function mode on GPIOB pins PC11 and PC13

  GPIOB->AFR[1] |= 0x01 << GPIO_AFRH_AFSEL11_Pos; // Select AF1 on PB11 in AFRH for I2C2_SDA
  GPIOB->AFR[1] |= 0x05 << GPIO_AFRH_AFSEL13_Pos; // Select AF5 on PB13 in AFRH for I2C2_SCL

  GPIOB->OTYPER |= (1 << 11); // setting PB11 to open-drain [1]
  GPIOB->OTYPER |= (1 << 13); // setting PB13 to open-drain [1]

  // setting PB14 to general output [29-28] = [01]
  GPIOB->MODER |= (1 << 28);  // setting 28th
  GPIOB->MODER &= ~(1 << 29); // clearing 29th

  GPIOB->OTYPER &= ~(1 << 14); // setting PB14 to push/pull output (clearing 14th bit for PB14 in OTYPER)

  GPIOB->ODR |= (1 << 14); // setting pin 14 to high

  // setting PC0 to general output [29-28] = [01]
  GPIOC->MODER |= (1 << 0);  // clearing 0th
  GPIOC->MODER &= ~(1 << 1); // setting 1st

  GPIOC->OTYPER &= ~(1 << 0); // setting PC0 to push/pull output (clearing 0th bit for PC0 in OTYPER)

  GPIOC->ODR |= (1 << 0); // setting pin 0 to high

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

  /* === I2C Settings === */

  // Setting TIMINGR register parameters to 100kHz standard-mode I2C
  // I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
  // I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
  // I2C2->TIMINGR |= (0x0F << I2C_TIMINGR_SCLH_Pos);
  // I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
  // I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);

  I2C2->TIMINGR = 0x1042F013;

  I2C2->CR1 |= I2C_CR1_PE; // enable I2C2 peripheral
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