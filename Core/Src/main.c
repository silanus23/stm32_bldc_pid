/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file              : main.c
 * @brief             : STM32F407G-DISC1 PID fan controller: init, RTOS tasks, ISRs
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "app_config.h"
#include "pid.h"
#include "tacho.h"
#include "command.h"
#include "settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Consistent snapshot of everything the USB ISR can change */
typedef struct {
    bool manual;
    bool bumpless;
    float setpoint;
    float manual_pwm;
    pid_gains_t gains;
} control_inputs_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_FLAG_SAVE_REQUEST   0x0001U
#define USB_FLAG_CMD_ERROR      0x0002U
#define USB_FLAGS_ALL           (USB_FLAG_SAVE_REQUEST | USB_FLAG_CMD_ERROR)

/* STM32F407G-DISC1 user LEDs (PD12 / green is used as the PWM output) */
#define LED_GPIO_PORT           GPIOD
#define LED_ORANGE_PIN          GPIO_PIN_13     /* on after a watchdog reset */
#define LED_RED_PIN             GPIO_PIN_14     /* on in Error_Handler */
#define LED_BLUE_PIN            GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
IWDG_HandleTypeDef hiwdg;

osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t tachoTaskHandle;
const osThreadAttr_t tachoTask_attributes = {
  .name = "tachoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
osSemaphoreId_t tachoSemaphoreHandle;
const osSemaphoreAttr_t tachoSemaphore_attributes = {
  .name = "tachoSemaphore"
};
/* USER CODE BEGIN PV */

/* Written by the USB ISR, read by tasks through read_control_inputs() */
static volatile float setpoint_rpm = DEFAULT_SETPOINT_RPM;
static volatile float manual_pwm = 0.0f;
static volatile bool  manual_mode = false;
static volatile bool  pending_bumpless_transfer = false;
static volatile float Kp = DEFAULT_KP;
static volatile float Ki = DEFAULT_KI;
static volatile float Kd = DEFAULT_KD;
static volatile float max_rpm = DEFAULT_MAX_RPM;

/* Written by one task, read by others (single-word accesses are atomic) */
static volatile float measured_rpm = 0.0f;
static volatile float pid_output = 0.0f;
static volatile bool  tacho_timeout = false;
static volatile bool  flash_busy = false;
static volatile uint32_t heartbeat_tacho_ms = 0;
static volatile uint32_t heartbeat_usb_ms = 0;

/* Written by the EXTI0 ISR, read by tachoTask */
static volatile uint32_t tacho_capture_us = 0;

static bool watchdog_reset_occurred = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
void StartPidTask(void *argument);
void StartUsbTask(void *argument);
void StartTachoTask(void *argument);

/* USER CODE BEGIN PFP */
static void set_fan_speed(float duty_cycle);
static void watchdog_service(void);
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
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  SystemClock_Config();

  /* USER CODE BEGIN Init */
  watchdog_reset_occurred = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET);
  __HAL_RCC_CLEAR_RESET_FLAGS();

  settings_t settings;
  if (settings_load(&settings))
  {
    Kp = settings.kp;
    Ki = settings.ki;
    Kd = settings.kd;
    max_rpm = settings.max_rpm;
    if (setpoint_rpm > max_rpm) setpoint_rpm = max_rpm;
  }
  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  if (watchdog_reset_occurred)
  {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
  }
  // Start the microsecond timer
  HAL_TIM_Base_Start(&htim2);
  // Start the PWM timer channel
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  tachoSemaphoreHandle = osSemaphoreNew(1, 0, &tachoSemaphore_attributes);

  pidTaskHandle = osThreadNew(StartPidTask, NULL, &pidTask_attributes);
  usbTaskHandle = osThreadNew(StartUsbTask, NULL, &usbTask_attributes);
  tachoTaskHandle = osThreadNew(StartTachoTask, NULL, &tachoTask_attributes);

  if (tachoSemaphoreHandle == NULL || pidTaskHandle == NULL ||
      usbTaskHandle == NULL || tachoTaskHandle == NULL)
  {
    Error_Handler();
  }

  /* Started last so the init above cannot trip it */
  MX_IWDG_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
 * @brief TIM2 Initialization Function (Microsecond Timer)
 * @param None
 * @retval None
 * @note Configured as a 32-bit free-running microsecond counter
 * @note Prescaler set to 83 for 1MHz counting (assuming 84MHz APB1)
 * @note Used for tachometer pulse timing measurements
 */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM4 Initialization Function (PWM Generation)
 * @param None
 * @note Configured for PWM output on Channel 1 (PD12)
 * @note PWM frequency ~2.5kHz (prescaler 83, period 399)
 * @note Used to control fan motor speed
 */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief IWDG Initialization Function (~8 s timeout, longer than a flash erase)
 */
static void MX_IWDG_Init(void)
{
  __HAL_DBGMCU_FREEZE_IWDG();

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 2000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @note PA0 tach input (falling-edge EXTI0), PD13-PD15 LEDs
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN|LED_RED_PIN|LED_BLUE_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_ORANGE_PIN|LED_RED_PIN|LED_BLUE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Reads the ISR-written inputs with interrupts off, so they are consistent.
 */
static control_inputs_t read_control_inputs(void)
{
  control_inputs_t in;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  in.manual = manual_mode;
  in.bumpless = pending_bumpless_transfer;
  pending_bumpless_transfer = false;
  in.setpoint = setpoint_rpm;
  in.manual_pwm = manual_pwm;
  in.gains.kp = Kp;
  in.gains.ki = Ki;
  in.gains.kd = Kd;
  __set_PRIMASK(primask);

  return in;
}

/**
 * @brief Refreshes the IWDG only while tachoTask and usbTask are still alive.
 */
static void watchdog_service(void)
{
  uint32_t now = osKernelGetTickCount();
  if ((now - heartbeat_tacho_ms) < WATCHDOG_TASK_TIMEOUT_MS &&
      (now - heartbeat_usb_ms) < WATCHDOG_TASK_TIMEOUT_MS)
  {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/**
 * @brief Function implementing the tachoTask thread.
 * @param argument Unused thread argument (required by RTOS)
 * @note Calculates RPM from the time between tachometer pulses
 */
void StartTachoTask(void *argument)
{
  bool is_first_pulse = true;
  uint32_t last_capture_us = 0;

  for (;;)
  {
    heartbeat_tacho_ms = osKernelGetTickCount();

    if (osSemaphoreAcquire(tachoSemaphoreHandle, TACHO_TIMEOUT_MS) == osOK)
    {
      uint32_t capture_us = tacho_capture_us;

      if (flash_busy)
      {
        is_first_pulse = true;
        continue;
      }

      if (is_first_pulse)
      {
        is_first_pulse = false;
        last_capture_us = capture_us;
        continue;
      }

      uint32_t period_us = capture_us - last_capture_us;   // wrap-safe
      last_capture_us = capture_us;

      float rpm;
      if (tacho_rpm_from_period(period_us, &rpm))
      {
        measured_rpm = rpm;
      }
    }
    else
    {
      measured_rpm = 0.0f;
      tacho_timeout = true;
      is_first_pulse = true;
    }
  }
}

/**
 * @brief Function implementing the PID controller task thread.
 * @param argument Unused thread argument (required by RTOS)
 * @note Runs every PID_SAMPLE_TIME_MS and services the watchdog
 */
void StartPidTask(void *argument)
{
  pid_state_t pid;
  pid_reset(&pid);

  osDelay(2000);

  uint32_t tick = osKernelGetTickCount();

  for (;;)
  {
    tick += PID_SAMPLE_TIME_MS;
    if (osDelayUntil(tick) != osOK)
    {
      // Deadline missed (flash erase stall): resync
      tick = osKernelGetTickCount();
    }

    watchdog_service();

    control_inputs_t in = read_control_inputs();
    float measured = measured_rpm;

    if (tacho_timeout)
    {
      tacho_timeout = false;
      pid_reset(&pid);
    }

    if (in.bumpless && !in.manual)
    {
      pid_bumpless_transfer(&pid, &in.gains, in.setpoint, measured, pid_output);
    }

    float output;
    if (in.manual)
    {
      output = in.manual_pwm;
    }
    else
    {
      output = pid_update(&pid, &in.gains, in.setpoint, measured, PID_SAMPLE_TIME_S);
    }

    pid_output = output;
    set_fan_speed(output);
  }
}

/**
 * @brief Sends a string over USB CDC, retrying while the endpoint is busy.
 */
static void usb_transmit_retry(const char *str)
{
  uint8_t retries = 3;
  while (retries > 0)
  {
    if (CDC_Transmit_FS((uint8_t *)str, strlen(str)) == USBD_OK)
    {
      break;
    }
    retries--;
    osDelay(10);
  }
}

/**
 * @brief Function implementing the USB telemetry task thread.
 * @param argument Unused thread argument (required by RTOS)
 * @note Sends telemetry and replies, and runs flash saves requested by the USB ISR
 */
void StartUsbTask(void *argument)
{
  // Static: CDC_Transmit_FS keeps the pointer until the transfer completes
  static char tx_buffer[96];

  heartbeat_usb_ms = osKernelGetTickCount();
  osDelay(2500);

  if (watchdog_reset_occurred)
  {
    usb_transmit_retry("WARN: previous reset was caused by the watchdog\r\n");
  }

  uint32_t next_ms = osKernelGetTickCount();

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();
    heartbeat_usb_ms = now;

    uint32_t wait = ((int32_t)(next_ms - now) > 0) ? (next_ms - now) : 0;
    uint32_t flags = osThreadFlagsWait(USB_FLAGS_ALL, osFlagsWaitAny, wait);

    if (!(flags & osFlagsError))
    {
      if (flags & USB_FLAG_CMD_ERROR)
      {
        usb_transmit_retry("ERR: invalid command\r\n");
      }
      if (flags & USB_FLAG_SAVE_REQUEST)
      {
        control_inputs_t in = read_control_inputs();
        settings_t s = settings_make(in.gains.kp, in.gains.ki, in.gains.kd, max_rpm);

        // Tach pulses delayed by the erase stall are discarded by tachoTask
        flash_busy = true;
        bool ok = settings_save(&s);
        flash_busy = false;
        usb_transmit_retry(ok ? "OK: saved\r\n" : "ERR: save failed\r\n");
      }
      continue;
    }

    next_ms += TELEMETRY_PERIOD_MS;
    if ((int32_t)(osKernelGetTickCount() - next_ms) > 0)
    {
      next_ms = osKernelGetTickCount() + TELEMETRY_PERIOD_MS;
    }

    uint32_t now_ms = HAL_GetTick();
    snprintf(tx_buffer, sizeof(tx_buffer),
             "Time: %4lu.%02lu, Set: %.1f, Meas: %.1f, PWM: %.1f\r\n",
             (unsigned long)(now_ms / 1000), (unsigned long)((now_ms % 1000) / 10),
             (double)setpoint_rpm, (double)measured_rpm, (double)pid_output);

    usb_transmit_retry(tx_buffer);
  }
}

/**
 * @brief Sets the fan's PWM duty cycle.
 * @param duty_cycle The desired duty cycle from 0.0f to 100.0f (clamped)
 */
static void set_fan_speed(float duty_cycle)
{
  if (duty_cycle < 0.0f) duty_cycle = 0.0f;
  if (duty_cycle > 100.0f) duty_cycle = 100.0f;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
  // CCR > ARR gives 100% duty
  uint32_t ccr = (uint32_t)((duty_cycle / 100.0f) * (arr + 1));

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

/**
 * @brief External Interrupt ISR callback for tachometer input (PA0).
 * @param GPIO_Pin The GPIO pin that triggered the interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    tacho_capture_us = __HAL_TIM_GET_COUNTER(&htim2);
    osSemaphoreRelease(tachoSemaphoreHandle);
  }
}

/**
 * @brief Applies one parsed command to the shared state (USB ISR context).
 */
static void apply_command(command_t cmd)
{
  switch (cmd.type)
  {
  case CMD_SETPOINT:
    if (manual_mode)
    {
      pending_bumpless_transfer = true;
    }
    manual_mode = false;
    setpoint_rpm = (cmd.value > max_rpm) ? max_rpm : cmd.value;
    break;
  case CMD_MANUAL:
    manual_pwm = cmd.value;
    manual_mode = true;
    break;
  case CMD_KP:
    Kp = cmd.value;
    break;
  case CMD_KI:
    Ki = cmd.value;
    break;
  case CMD_KD:
    Kd = cmd.value;
    break;
  case CMD_MAX_RPM:
    max_rpm = cmd.value;
    if (setpoint_rpm > max_rpm)
    {
      setpoint_rpm = max_rpm;
    }
    break;
  case CMD_SAVE:
    // Flash erase must not run in the ISR: defer to usbTask
    osThreadFlagsSet(usbTaskHandle, USB_FLAG_SAVE_REQUEST);
    break;
  case CMD_INVALID:
  default:
    osThreadFlagsSet(usbTaskHandle, USB_FLAG_CMD_ERROR);
    break;
  }
}

/**
 * @brief Processes received USB data (USB ISR context, called from CDC_Receive_FS).
 */
void process_usb_command(const uint8_t *Buf, uint32_t Len)
{
  static command_line_t line_buf;

  for (uint32_t i = 0; i < Len; i++)
  {
    const char *line;
    if (command_line_feed(&line_buf, Buf[i], &line))
    {
      apply_command(command_parse(line));
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @note   Turns on the red LED and halts; the IWDG (if started) resets the MCU.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitStruct.Pin = LED_RED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
