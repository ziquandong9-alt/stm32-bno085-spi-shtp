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
#define BNO085_MAG_INTERVAL_US     40000U
#define BNO085_YPR_PRINT_DIVIDER       4U
#define BNO085_SOFT_ERROR_LIMIT        3U
/* Set to 1 only when validating every optional report; normal demo stays lean.
   仅在验证全部可选报告时设为 1；日常示例保持较低总线负载。 */
#define BNO085_EXTENDED_VALIDATION     0U
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
static uint32_t rv_events_per_second;
static uint32_t game_events_per_second;
static uint32_t accel_events_per_second;
static uint32_t gyro_events_per_second;
static uint32_t mag_events_per_second;
static uint32_t stats_start_ms;
static uint32_t ypr_print_divider;
static uint32_t consecutive_transport_errors;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void BNO085_Start(void);
static void bno085_process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Configure, identify and start all demo SH-2 reports.
 *        配置平台层、读取产品信息，并启动示例所需的全部报告。
  *
  * This function retries the complete sequence after any startup failure.
  * 任一步启动失败都会延时后重新执行完整初始化流程。
  */
static void BNO085_Start(void)
{
  BNO085_STM32_PortConfig_t port;
  BNO085_Config_t driver_config;
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
    /* Start from portable defaults; applications may tune policy here without
       editing the protocol core. / 从默认策略开始，应用可集中调整超时与重试。 */
    BNO085_GetDefaultConfig(&driver_config);
    status = BNO085_InitWithConfig(&driver_config);
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

    /* Explicitly enable the three normal dynamic calibrators. The command
       response is matched before startup continues. / 显式启用三类常规动态
       校准，并等待匹配的命令响应后再继续启动。 */
    status = BNO085_SetCalibration(BNO085_CAL_ACCELEROMETER |
                                   BNO085_CAL_GYROSCOPE |
                                   BNO085_CAL_MAGNETOMETER |
                                   BNO085_CAL_ON_TABLE);
    if (status != BNO085_OK)
    {
      printf("BNO085 calibration configuration failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    /* Enable the lower-rate magnetic report first.  Subsequent Set Feature
       writes can then fit into gaps between sensor packets more reliably.
       先启用低速磁场报告，让后续配置命令更容易落在报告间隙中。 */
    status = BNO085_EnableMagnetometer(BNO085_MAG_INTERVAL_US);
    if (status != BNO085_OK)
    {
      BNO085_Diagnostics_t diagnostics;
      printf("BNO085 magnetometer enable failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      if (BNO085_GetDiagnostics(&diagnostics) == BNO085_OK)
      {
        printf("Feature 0x%02X req/actual: int=%lu/%lu batch=%lu/%lu "
               "sens=%u/%u spec=%lu/%lu flags=0x%02X/0x%02X\r\n",
               diagnostics.last_feature_id,
               diagnostics.requested_interval_us,
               diagnostics.effective_interval_us,
               diagnostics.requested_batch_interval_us,
               diagnostics.effective_batch_interval_us,
               diagnostics.requested_change_sensitivity,
               diagnostics.effective_change_sensitivity,
               diagnostics.requested_sensor_specific,
               diagnostics.effective_sensor_specific,
               diagnostics.requested_feature_flags,
               diagnostics.effective_feature_flags);
      }
      HAL_Delay(1000U);
      continue;
    }

    status = BNO085_EnableGyroscope(BNO085_REPORT_INTERVAL_US);
    if (status != BNO085_OK)
    {
      printf("BNO085 gyroscope enable failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    status = BNO085_EnableGameRotationVector(BNO085_REPORT_INTERVAL_US);
    if (status != BNO085_OK)
    {
      printf("BNO085 game rotation vector enable failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }

    /* Set Feature interval is in microseconds: 10000 us = 100 Hz.
       Set Feature 周期单位为微秒：10000 us 即 100 Hz。 */
    status = BNO085_EnableAccelerometer(BNO085_REPORT_INTERVAL_US);
    if (status != BNO085_OK)
    {
      BNO085_Diagnostics_t diagnostics;
      printf("BNO085 accelerometer enable failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      if (BNO085_GetDiagnostics(&diagnostics) == BNO085_OK)
      {
        printf("Feature 0x%02X interval requested=%lu actual=%lu us\r\n",
               diagnostics.last_feature_id,
               diagnostics.requested_interval_us,
               diagnostics.effective_interval_us);
      }
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

#if BNO085_EXTENDED_VALIDATION
    status = BNO085_EnableLinearAcceleration(BNO085_MAG_INTERVAL_US);
    if (status == BNO085_OK) status = BNO085_EnableGravity(BNO085_MAG_INTERVAL_US);
    if (status == BNO085_OK) status = BNO085_EnableUncalibratedGyroscope(BNO085_MAG_INTERVAL_US);
    if (status == BNO085_OK) status = BNO085_EnableUncalibratedMagnetometer(BNO085_MAG_INTERVAL_US);
    if (status == BNO085_OK) status = BNO085_EnableRawAccelerometer(BNO085_MAG_INTERVAL_US);
    if (status == BNO085_OK) status = BNO085_EnableRawGyroscope(BNO085_MAG_INTERVAL_US);
    if (status == BNO085_OK) status = BNO085_EnableRawMagnetometer(BNO085_MAG_INTERVAL_US);
    if (status != BNO085_OK)
    {
      printf("BNO085 extended report enable failed: %s (%d)\r\n",
             BNO085_StatusString(status), status);
      HAL_Delay(1000U);
      continue;
    }
#endif

    /* Do not regard a successful Set Feature write as proof of sensor output.
       Wait until every enabled report has actually been decoded and cached.
       Set Feature 写成功不等于已有数据，所有已启用报告都真正到达后才算启动成功。 */
    {
      uint32_t start_ms = HAL_GetTick();
      uint32_t events = 0U;
      uint32_t seen_events = 0U;
      float accel_x;
      float accel_y;
      float accel_z;
      BNO085_Gyroscope_t gyro;
      BNO085_Magnetometer_t mag;
      BNO085_Euler_t game_euler;
      uint32_t required_events =
          BNO085_EVENT_ROTATION_VECTOR |
          BNO085_EVENT_GAME_ROTATION_VECTOR |
          BNO085_EVENT_ACCELEROMETER |
          BNO085_EVENT_GYROSCOPE |
          BNO085_EVENT_MAGNETOMETER;
#if BNO085_EXTENDED_VALIDATION
      required_events |= BNO085_EVENT_LINEAR_ACCELERATION |
                         BNO085_EVENT_GRAVITY |
                         BNO085_EVENT_GYROSCOPE_UNCAL |
                         BNO085_EVENT_MAGNETOMETER_UNCAL |
                         BNO085_EVENT_RAW_ACCELEROMETER |
                         BNO085_EVENT_RAW_GYROSCOPE |
                         BNO085_EVENT_RAW_MAGNETOMETER;
#endif

      do
      {
        status = BNO085_Poll(1000U, &events);
        if (status == BNO085_OK)
        {
          /* A cargo can contain one or both enabled reports. / 一个包可含一种或两种报告。 */
          seen_events |= events;
        }
      } while ((status == BNO085_OK) &&
               ((seen_events & required_events) != required_events) &&
               ((uint32_t)(HAL_GetTick() - start_ms) < 2000U));

      /* These six calls read cache only; no additional SPI transactions.
         这六个 Getter 只读缓存，不会再次访问 SPI。 */
      if ((status == BNO085_OK) &&
          (BNO085_GetYaw(&yaw_deg) == BNO085_OK) &&
          (BNO085_GetRoll(&roll_deg) == BNO085_OK) &&
          (BNO085_GetPitch(&pitch_deg) == BNO085_OK) &&
          (BNO085_GetAccelerationX(&accel_x) == BNO085_OK) &&
          (BNO085_GetAccelerationY(&accel_y) == BNO085_OK) &&
          (BNO085_GetAccelerationZ(&accel_z) == BNO085_OK) &&
          (BNO085_GetGyroscope(&gyro) == BNO085_OK) &&
          (BNO085_GetMagnetometer(&mag) == BNO085_OK) &&
          (BNO085_GetGameEuler(&game_euler) == BNO085_OK))
      {
        printf("ACC: x=%6.2f y=%6.2f z=%6.2f m/s^2\r\n",
               accel_x, accel_y, accel_z);
        printf("GYRO: x=%6.3f y=%6.3f z=%6.3f rad/s acc=%u\r\n",
               gyro.x_rps, gyro.y_rps, gyro.z_rps, gyro.accuracy);
        printf("MAG: x=%6.2f y=%6.2f z=%6.2f uT acc=%u\r\n",
               mag.x_uT, mag.y_uT, mag.z_uT, mag.accuracy);
        printf("GAME YPR: yaw=%7.2f roll=%7.2f pitch=%7.2f deg\r\n",
               game_euler.yaw_deg, game_euler.roll_deg,
               game_euler.pitch_deg);
#if BNO085_EXTENDED_VALIDATION
        {
          BNO085_LinearAcceleration_t linear;
          BNO085_Gravity_t gravity;
          BNO085_UncalibratedGyroscope_t uncal_gyro;
          BNO085_UncalibratedMagnetometer_t uncal_mag;
          BNO085_RawVector_t raw_accel, raw_mag;
          BNO085_RawGyroscope_t raw_gyro;
          if ((BNO085_GetLinearAcceleration(&linear) == BNO085_OK) &&
              (BNO085_GetGravity(&gravity) == BNO085_OK) &&
              (BNO085_GetUncalibratedGyroscope(&uncal_gyro) == BNO085_OK) &&
              (BNO085_GetUncalibratedMagnetometer(&uncal_mag) == BNO085_OK) &&
              (BNO085_GetRawAccelerometer(&raw_accel) == BNO085_OK) &&
              (BNO085_GetRawGyroscope(&raw_gyro) == BNO085_OK) &&
              (BNO085_GetRawMagnetometer(&raw_mag) == BNO085_OK))
          {
            printf("EXT OK: lin=%.2f grav=%.2f gyro_bias=%.3f mag_bias=%.2f raw=%d/%d/%d\r\n",
                   linear.x_mps2, gravity.z_mps2,
                   uncal_gyro.bias_x_rps, uncal_mag.bias_x_uT,
                   raw_accel.x_counts, raw_gyro.x_counts, raw_mag.x_counts);
          }
          else
          {
            status = BNO085_ERR_NO_DATA;
          }
        }
#endif
      }
      else if (status == BNO085_OK)
      {
        status = BNO085_ERR_TIMEOUT;
      }
    }

    if (status == BNO085_OK)
    {
      printf("BNO085 DMA stream: RV/GAME/ACC/GYRO=100 Hz, MAG=25 Hz\r\n");
      last_rotation_ms = HAL_GetTick();
      stats_start_ms = last_rotation_ms;
      rv_events_per_second = 0U;
      game_events_per_second = 0U;
      accel_events_per_second = 0U;
      gyro_events_per_second = 0U;
      mag_events_per_second = 0U;
      ypr_print_divider = 0U;
      consecutive_transport_errors = 0U;
      return;
    }

    printf("BNO085 produced no required sensor data: %s (%d)\r\n",
           BNO085_StatusString(status), status);
    HAL_Delay(1000U);
  }
}

/**
 * @brief Advance BNO085 DMA I/O and handle all application-level events.
 *        推进 BNO085 DMA 收包，并集中处理数据、统计和故障恢复。
 *
 * This function never waits for SPI completion.  When DMA is not ready,
 * BNO085_PollAsync() returns BNO085_PENDING and the caller can immediately
 * continue with other application work.
 * 本函数不等待 SPI；DMA 尚未完成时立即返回，主循环可继续处理其他任务。
 */
static void bno085_process(void)
{
  uint32_t events = 0U;
  BNO085_Status_t status = BNO085_Process(&events);

  if ((status != BNO085_OK) && (status != BNO085_PENDING))
  {
    BNO085_Diagnostics_t diagnostics;

    printf("BNO085 read failed: %s (%d)\r\n",
           BNO085_StatusString(status), status);
    consecutive_transport_errors++;
    if (consecutive_transport_errors >= BNO085_SOFT_ERROR_LIMIT)
    {
      /* Tier 2: rebuild only SPI/DMA. SH-2 keeps its enabled reports, making
         this much cheaper than a sensor reset. / 二级恢复：只重建 SPI/DMA。 */
      BNO085_Status_t recovery = BNO085_RecoverTransport();
      if (BNO085_GetDiagnostics(&diagnostics) == BNO085_OK)
      {
        printf("BNO085 transport recovery: %s, HAL=0x%08lX\r\n",
               BNO085_StatusString(recovery), diagnostics.port_raw_error);
      }
      consecutive_transport_errors = 0U;
    }
  }
  else if (status == BNO085_OK)
  {
    /* One good complete packet proves framing and DMA are synchronized again.
       一个完整好包说明总线和 DMA 已恢复同步。 */
    consecutive_transport_errors = 0U;
  }

  /* Count decoded reports for a simple one-second hardware sanity check.
     统计每秒报告数，直接验证 Set Feature 与 DMA 收包是否正常。 */
  if ((events & BNO085_EVENT_ACCELEROMETER) != 0U) accel_events_per_second++;
  if ((events & BNO085_EVENT_GYROSCOPE) != 0U) gyro_events_per_second++;
  if ((events & BNO085_EVENT_MAGNETOMETER) != 0U) mag_events_per_second++;
  if ((events & BNO085_EVENT_GAME_ROTATION_VECTOR) != 0U) game_events_per_second++;

  if ((status == BNO085_OK) &&
      ((events & BNO085_EVENT_ROTATION_VECTOR) != 0U) &&
      (BNO085_GetYaw(&yaw_deg) == BNO085_OK) &&
      (BNO085_GetRoll(&roll_deg) == BNO085_OK) &&
      (BNO085_GetPitch(&pitch_deg) == BNO085_OK))
  {
    /* All three angles belong to one cached rotation-vector frame.
       三个角度均来自同一帧缓存。 */
    last_rotation_ms = HAL_GetTick();
    rv_events_per_second++;
    ypr_print_divider++;
    if (ypr_print_divider >= BNO085_YPR_PRINT_DIVIDER)
    {
      ypr_print_divider = 0U;
      printf("YPR: yaw=%7.2f roll=%7.2f pitch=%7.2f deg\r\n",
             yaw_deg, roll_deg, pitch_deg);
    }
  }

  if ((uint32_t)(HAL_GetTick() - stats_start_ms) >= 1000U)
  {
    BNO085_Gyroscope_t gyro;
    BNO085_Magnetometer_t mag;
    BNO085_Euler_t game_euler;
    BNO085_Diagnostics_t diagnostics;

    printf("RATE/s: rv=%lu game=%lu acc=%lu gyro=%lu mag=%lu\r\n",
           rv_events_per_second, game_events_per_second,
           accel_events_per_second, gyro_events_per_second,
           mag_events_per_second);
    if ((BNO085_GetGyroscope(&gyro) == BNO085_OK) &&
        (BNO085_GetMagnetometer(&mag) == BNO085_OK) &&
        (BNO085_GetGameEuler(&game_euler) == BNO085_OK))
    {
      printf("GYRO[%u]: %6.3f %6.3f %6.3f rad/s  MAG[%u]: %6.2f %6.2f %6.2f uT\r\n",
             gyro.accuracy, gyro.x_rps, gyro.y_rps, gyro.z_rps,
             mag.accuracy, mag.x_uT, mag.y_uT, mag.z_uT);
      printf("GAME YPR: yaw=%7.2f roll=%7.2f pitch=%7.2f deg\r\n",
             game_euler.yaw_deg, game_euler.roll_deg,
             game_euler.pitch_deg);
    }
    if (BNO085_GetDiagnostics(&diagnostics) == BNO085_OK)
    {
      printf("DIAG: pkt=%lu shtp_gap=%lu sensor_gap=%lu bad=%lu cont=%lu io=%lu dma_to=%lu recover=%lu reset=%lu\r\n",
             diagnostics.shtp_packets, diagnostics.shtp_sequence_gaps,
             diagnostics.sensor_sequence_gaps,
             diagnostics.invalid_packets, diagnostics.continuation_packets,
             diagnostics.port_errors, diagnostics.dma_timeouts,
             diagnostics.transport_recoveries, diagnostics.hard_resets);
    }
    stats_start_ms = HAL_GetTick();
    rv_events_per_second = 0U;
    game_events_per_second = 0U;
    accel_events_per_second = 0U;
    gyro_events_per_second = 0U;
    mag_events_per_second = 0U;
  }

  /* Other reports may continue while Rotation Vector stalls, so recovery is
     based specifically on the last Rotation Vector event.
     其他报告仍可能到达，故恢复仅以最后姿态帧为准。 */
  if ((uint32_t)(HAL_GetTick() - last_rotation_ms) >= 1000U)
  {
    printf("BNO085: no rotation data for 1 second, restarting\r\n");
    BNO085_Start();
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  printf("BNO085 demo boot (SPI DMA)\r\n");
  /* USER CODE BEGIN 2 */
  BNO085_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Non-blocking service: unfinished DMA returns immediately.
       非阻塞服务：DMA 未完成就立即返回。 */
    bno085_process();
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
