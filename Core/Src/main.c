/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32_hal_legacy.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  IR_WAIT_IDLE,  // 等待信号恢复空闲，避免从一个帧的中间开始
  IR_IDLE,       // 等待第一个下降沿
  IR_RECORDING,  // 逐段记录时长
  IR_READY       // 数组已冻结，等待主循环发送
} IR_State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IR_CAPACITY 512u
#define IR_GAP_MS   30u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t wave[IR_CAPACITY];
volatile uint16_t count = 0;
static volatile IR_State irState = IR_WAIT_IDLE;
static volatile uint32_t lastEdgeMs = 0;
static uint16_t lastEdge;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_0 || irState == IR_READY)
    return;

  uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  uint32_t nowMs = HAL_GetTick();
  GPIO_PinState level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

  if (irState == IR_RECORDING)
  {
    // 主循环若没及时处理帧间空闲，也不把长间隔写进数组。
    if (level == GPIO_PIN_RESET && nowMs - lastEdgeMs >= IR_GAP_MS)
    {
      irState = IR_READY;
      return;
    }

    wave[count++] = (uint16_t)(now - lastEdge);
    lastEdge = now;
    if (count == IR_CAPACITY)
      irState = IR_READY;  // 数组满：停止写入，禁止越界
  }
  else if (irState == IR_IDLE && level == GPIO_PIN_RESET)
  {
    // 第一个下降沿只记起点，不保存前面的空闲高电平。
    count = 0;
    lastEdge = now;
    irState = IR_RECORDING;
  }

  lastEdgeMs = nowMs;
}

static void IR_Poll(void)
{
  // 检查超时和切换状态必须与边沿中断互斥，发送时则保持中断开启。
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET &&
      HAL_GetTick() - lastEdgeMs >= IR_GAP_MS)
  {
    if (irState == IR_RECORDING)
      irState = IR_READY;  // 帧结束，尾部的空闲高电平不写入数组
    else if (irState == IR_WAIT_IDLE)
      irState = IR_IDLE;
  }
  __set_PRIMASK(primask);

  if (irState != IR_READY)
    return;

  char text[80];
  int length = snprintf(text, sizeof(text),
                        "IR_RAW_US LOW_FIRST count=%u end=%s ",
                        (unsigned)count,
                        count == IR_CAPACITY ? "FULL" : "GAP");
  if (HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)length, 100) != HAL_OK)
    Error_Handler();

  for (uint16_t i = 0; i < count; ++i)
  {
    length = snprintf(text, sizeof(text), "%s%u",
                      i == 0 ? "" : ",", (unsigned)wave[i]);
    if (HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)length, 100) != HAL_OK)
      Error_Handler();
  }
  if (HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100) != HAL_OK)
    Error_Handler();

  primask = __get_PRIMASK();
  __disable_irq();
  count = 0;
  lastEdgeMs = HAL_GetTick();
  irState = IR_WAIT_IDLE;
  __set_PRIMASK(primask);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    Error_Handler();
  lastEdgeMs = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IR_Poll();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
