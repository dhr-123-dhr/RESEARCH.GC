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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "path_plan.h"          /* 路网节点图 A* 路径规划(本工程新增) */
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化路网 A* 模块(清空黑柱封锁表, 复位串口收包状态机) */
  PathPlan_Init();

#if PATHPLAN_MANUAL_TEST
  /* ----------------------------------------------------------------------
   * 手动测试示例(把 path_plan.h 里 PATHPLAN_MANUAL_TEST 改成 1 即可, 不需要串口屏):
   *   场景: 起点/终点都是启停区1(2250,2250), 两个黑柱在 (2100,1600) / (2100,700)
   *   期望结果(标准地图, 与上位机"路网点顺序"逐点一致):
   *     屏 t1        : 7-8-5-4-5-8-5-2-3-6-5-8-5-2-3-6-5-8-7,16800
   *                    (全路径节点号, 路网总长mm)
   *     各步本段(1~9): 7-8-5-4 / 4-5-8 / 8-5-2 / 2-3-6 / 6-5-8
   *                    / 8-5-2 / 2-3-6 / 6-5-8-7
   *     (把下面两行 ObsAdd 注释掉再编译 -> t1 = 7-4-7-8-5-2-3-6-5-8-5-2-3-6-5-8-7,15000)
   * -------------------------------------------------------------------- */
  PathPlan_SetStart(2250, 2250);      /* 起点启停区中心 */
  PathPlan_SetGoal (2250, 2250);      /* 终点启停区中心 */
  PathPlan_ObsAdd  (2100, 1600);      /* 黑柱1 -> 封锁边 E7 (3,6) */
  PathPlan_ObsAdd  (2100,  700);      /* 黑柱2 -> 封锁边 E6 (0,3) */
  PathPlan_RunTask();                 /* 生成整任务, 并把停靠节点号串发到屏 t1 */
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t rx;             /* 方案B: 轮询接收; 声明放代码块最前面(C90 兼容) */

    /* 串口屏 -> STM32: 每次都试着收 1 个字节(超时 0 = 非阻塞, 不会卡住主循环),
       收到就喂给路网 A* 解析器; 解析器内部负责"关键字切分 + 200ms 空闲收尾" */
    if (HAL_UART_Receive(&huart1, &rx, 1, 0) == HAL_OK)
    {
        PathPlan_OnRxByte(rx);
    }
    PathPlan_Tick();        /* 整包边界判定(收到空行 或 最后字节后200ms空闲 -> 重算并刷屏) */
    //Wheels_RunPath();     /* 底盘执行占位: 自行按 steps[] 逐步走, 帧用 PathPlan_GetStepFrame(i) 取 */
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
