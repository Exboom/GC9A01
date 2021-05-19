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
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GC9A01.h"
#include "math.h"
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
uint16_t pwm_value = 0;

// uint8_t completed = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setPWM(int8_t mode);

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
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  GC9A01_init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  TIM3->CCR2 = pwm_value;
  struct GC9A01_frame frame = {{0,0},{239,239}};
  GC9A01_set_frame(frame);
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  

    uint8_t color[3];
    // Triangle
    color[0] = 0xFF;
    color[1] = 0xFF;
    for (int x = 0; x < 240; x++) {
        for (int y = 0; y < 240; y++) {
            if (x < y) {
                color[2] = 0xFF;
            } else {
                color[2] = 0x00;
            }
            if (x == 0 && y == 0) {
                GC9A01_write(color, sizeof(color));
            } else {
                GC9A01_write_continue(color, sizeof(color));
            }
        }
    }
    setPWM(1);
    LL_mDelay(1000);
    // setPWM(0);
    // Rainbow
    float frequency = 0.026;
    for (int x = 0; x < 240; x++) {
        color[0] = sin(frequency*x + 0) * 127 + 128;
        color[1] = sin(frequency*x + 2) * 127 + 128;
        color[2] = sin(frequency*x + 4) * 127 + 128;
        for (int y = 0; y < 240; y++) {
            if (x == 0 && y == 0) {
                GC9A01_write(color, sizeof(color));
            } else {
                GC9A01_write_continue(color, sizeof(color));
            }
        }
    }
    // setPWM(1);
    LL_mDelay(1000);
    // setPWM(0);
    // Checkerboard
    for (int x = 0; x < 240; x++) {
        for (int y = 0; y < 240; y++) {
            if ((x / 10) % 2 ==  (y / 10) % 2) {
                color[0] = 0xFF;
                color[1] = 0xFF;
                color[2] = 0xFF;
            } else {
                color[0] = 0x00;
                color[1] = 0x00;
                color[2] = 0x00;
            }
            if (x == 0 && y == 0) {
                GC9A01_write(color, sizeof(color));
            } else {
                GC9A01_write_continue(color, sizeof(color));
            }
        }
    }
    // setPWM(1);
    LL_mDelay(1000);
    // setPWM(0);
    // Swiss flag
    color[0] = 0xFF;
    for (int x = 0; x < 240; x++) {
        for (int y = 0; y < 240; y++) {
            if ((x >= 1*48 && x < 4*48 && y >= 2*48 && y < 3*48) ||
                (x >= 2*48 && x < 3*48 && y >= 1*48 && y < 4*48)) {
                color[1] = 0xFF;
                color[2] = 0xFF;
            } else {
                color[1] = 0x00;
                color[2] = 0x00;
            }
            if (x == 0 && y == 0) {
                GC9A01_write(color, sizeof(color));
            } else {
                GC9A01_write_continue(color, sizeof(color));
            }
        }
    }
    // setPWM(1);
    GC9A01_write_command(0x20);
    LL_mDelay(1000);
    // setPWM(0);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1µs transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */
void setPWM(int8_t mode) {
  if (mode==0) {
    while (pwm_value>0) {
      pwm_value += -1;
      TIM3->CCR2 = pwm_value;
      HAL_Delay(1);
    }
  } else {
      while (pwm_value<500) {
        pwm_value += 1;
        TIM3->CCR2 = pwm_value;
        HAL_Delay(1);
    }
  }
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
