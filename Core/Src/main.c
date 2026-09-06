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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno085.h"
#include "bno085_port_stm32.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO085_REPORT_INTERVAL_US  10000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static BNO085_ProductInfo_t product_info;
static uint32_t last_rotation_ms;
static float yaw_deg;
static float roll_deg;
static float pitch_deg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void BNO085_Start(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Configure, identify and start both required SH-2 reports.
  *        配置平台层、读取产品信息，并启动姿态和加速度报告。
  *
  * This function retries the complete sequence after any startup failure.
  * 任一步启动失败都会延时后重新执行完整初始化流程。
  */
static void BNO085_Start(void)
{
  BNO085_STM32_PortConfig_t port;
  BNO085_Status_t status;

  /* Only this application/port boundary knows STM32 handles and GPIO pins.
     只有应用与适配层边界知道 STM32 句柄和具体引脚。 */
  port.spi = &hspi1;
  port.cs_port = BNO085_CS_GPIO_Port;
  port.cs_pin = BNO085_CS_Pin;
  port.wake_port = BNO085_WAKE_GPIO_Port;
  port.wake_pin = BNO085_WAKE_Pin;
  port.reset_port = BNO085_RST_GPIO_Port;
  port.reset_pin = BNO085_RST_Pin;
  port.interrupt_port = BNO085_INT_GPIO_Port;
  port.interrupt_pin = BNO085_INT_Pin;

  if (!BNO085_STM32_Port_Init(&port))
  {
    printf("BNO085 platform configuration is invalid\r\n");
    Error_Handler();
  }

  for (;;)
  {
    /* Hardware reset -> SHTP advertisement -> executable reset-complete.
       硬复位 -> SHTP 广告包 -> executable 通道启动完成。 */
    status = BNO085_Init();
    if (status != BNO085_OK)
    {
      printf("BNO085 init failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    /* Product ID proves channel 2 host-to-device commands work correctly.
       Product ID 成功说明 channel 2 的双向控制通信正常。 */
    status = BNO085_GetProductInfo(&product_info);
    if (status != BNO085_OK)
    {
      printf("BNO085 product ID failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    printf("BNO085 part %lu, FW %u.%u.%u, build %lu, reset %u\r\n",
           product_info.sw_part_number,
           product_info.sw_major,
           product_info.sw_minor,
           product_info.sw_patch,
           product_info.build_number,
           product_info.reset_cause);

    /* Set Feature interval is in microseconds: 10000 us = 100 Hz.
       Set Feature 周期单位为微秒：10000 us 即 100 Hz。 */
    status = BNO085_EnableAccelerometer(BNO085_REPORT_INTERVAL_US);
    if (status != BNO085_OK)
    {
      printf("BNO085 accelerometer enable failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    status = BNO085_EnableRotationVector(BNO085_REPORT_INTERVAL_US);
    if (status != BNO085_OK)
    {
      printf("BNO085 enable command failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    /* Do not regard a successful Set Feature write as proof of sensor output.
       Wait until both report types have actually been decoded and cached.
       Set Feature 写成功不等于已有数据，必须真正收到两种报告后才算启动成功。 */
    {
      uint32_t start_ms = HAL_GetTick();
      uint32_t events = 0U;
      uint32_t seen_events = 0U;
      float accel_x;
      float accel_y;
      float accel_z;

      do
      {
        status = BNO085_Poll(1000U, &events);
        if (status == BNO085_OK)
        {
          /* A cargo can contain one or both enabled reports. / 一个包可含一种或两种报告。 */
          seen_events |= events;
        }
      } while ((status == BNO085_OK) &&
               ((seen_events & (BNO085_EVENT_ROTATION_VECTOR |
                                BNO085_EVENT_ACCELEROMETER)) !=
                (BNO085_EVENT_ROTATION_VECTOR |
                 BNO085_EVENT_ACCELEROMETER)) &&
               ((uint32_t)(HAL_GetTick() - start_ms) < 1000U));

      /* These six calls read cache only; no additional SPI transactions.
         这六个 Getter 只读缓存，不会再次访问 SPI。 */
      if ((status == BNO085_OK) &&
          (BNO085_GetYaw(&yaw_deg) == BNO085_OK) &&
          (BNO085_GetRoll(&roll_deg) == BNO085_OK) &&
          (BNO085_GetPitch(&pitch_deg) == BNO085_OK) &&
          (BNO085_GetAccelerationX(&accel_x) == BNO085_OK) &&
          (BNO085_GetAccelerationY(&accel_y) == BNO085_OK) &&
          (BNO085_GetAccelerationZ(&accel_z) == BNO085_OK))
      {
        printf("ACC: x=%6.2f y=%6.2f z=%6.2f m/s^2\r\n",
               accel_x, accel_y, accel_z);
      }
      else if (status == BNO085_OK)
      {
        status = BNO085_ERR_TIMEOUT;
      }
    }

    if (status == BNO085_OK)
    {
      printf("BNO085 rotation vector and accelerometer running at 100 Hz\r\n");
      last_rotation_ms = HAL_GetTick();
      return;
    }

    printf("BNO085 produced no rotation data: %s (%d)\r\n",
           BNO085_StatusString(status), status);
    HAL_Delay(1000U);
  }
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BNO085_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t events = 0U;
    /* Poll is the sole streaming I/O entry.  It drains one useful cargo,
       parses every sub-report, updates caches, and returns event bits.
       Poll 是流式阶段唯一 I/O 入口：收包、遍历子报告、更新缓存并返回事件位。 */
    BNO085_Status_t status = BNO085_Poll(200U, &events);

    if ((status == BNO085_OK) &&
        ((events & BNO085_EVENT_ROTATION_VECTOR) != 0U) &&
        (BNO085_GetYaw(&yaw_deg) == BNO085_OK) &&
        (BNO085_GetRoll(&roll_deg) == BNO085_OK) &&
        (BNO085_GetPitch(&pitch_deg) == BNO085_OK))
    {
      /* The three values belong to the same cached rotation-vector frame.
         三个角度均来自同一帧缓存。 */
      last_rotation_ms = HAL_GetTick();
      printf("YPR: yaw=%7.2f roll=%7.2f pitch=%7.2f deg\r\n",
             yaw_deg, roll_deg, pitch_deg);
    }
    else if ((status != BNO085_OK) && (status != BNO085_ERR_TIMEOUT))
    {
      printf("BNO085 read failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
    }

    /* Acceleration may continue while rotation stalls, so recovery is based
       specifically on the last rotation event. / 仅以最后姿态事件判断恢复。 */
    if ((uint32_t)(HAL_GetTick() - last_rotation_ms) >= 1000U)
    {
      printf("BNO085: no rotation data for 1 second, restarting\r\n");
      BNO085_Start();
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: source line number
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
