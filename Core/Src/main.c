/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file              : main.c
 * @brief             : Main program body for STM32F407G-DISC1 PID Fan Controller
 * @attention
 *
 *
 * - Fixes critical race condition on the 'integral' variable using a
 * critical section (osKernelLock/Unlock).
 * - Implements "derivative on measurement" to prevent derivative kick.
 * - Provides non-zero starting PID gains (Kp, Ki, Kd) for actual control.
 * - Includes robust tachometer logic with a timeout for stalled fans and
 * first-pulse rejection.
 * - Uses snprintf for safer string formatting in the USB task.
 * - MODIFIED: Implements a 10% minimum PWM output, while allowing the fan
 * to be fully turned off (0% PWM) when the setpoint is 0 RPM.
 * - Atomic reads for shared volatile variables.
 * - Improved anti-windup accounting for feedforward.
 * - Flash write verification.
 * - Bumpless transfer from manual to PID mode.
 * - USB transmit busy handling.
 * - Command parsing validation.
 *
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
#include <ctype.h>
#include <stdlib.h>
#include <errno.h>
#include "stm32f4xx_hal_iwdg.h"
#include "stm32f4xx_ll_iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Packed struct to avoid padding issues in flash storage */
typedef struct __attribute__((packed)) {
    uint32_t magic_number;
    float Kp;
    float Ki;
    float Kd;
    float max_rpm;
} Flash_Settings;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PULSES_PER_REVOLUTION 2.0f
#define TACHO_TIMEOUT_MS 1000
#define PID_SAMPLE_TIME_S 0.025f
#define PID_OUTPUT_MAX 95.0f
#define PID_OUTPUT_MIN 10.0f
#define FLASH_SETTINGS_ADDRESS  0x080E0000  
#define FLASH_MAGIC_NUMBER      0xDEADBEEF 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tachoTask */
osThreadId_t tachoTaskHandle;
const osThreadAttr_t tachoTask_attributes = {
  .name = "tachoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for tachoSemaphore */
osSemaphoreId_t tachoSemaphoreHandle;
const osSemaphoreAttr_t tachoSemaphore_attributes = {
  .name = "tachoSemaphore"
};
/* USER CODE BEGIN PV */

// PID Configuration
volatile float Kp = 0.1f;
volatile float Ki = 0.7f;
volatile float Kd = 0.001f;

// Shared State Variables
volatile float setpoint_rpm = 1600.0f;
volatile float measured_rpm = 0.0f;
volatile float pid_output = 0.0f;
volatile float integral = 0.0f;
volatile float max_rpm = 3100.0f;

// RPM Measurement Variables
volatile uint32_t last_capture_time_us = 0;
volatile uint32_t current_capture_time_us = 0;

// Manual Mode Variables
volatile bool manual_mode = false;
volatile float manual_pwm = 0.0f;

// Bumpless Transfer State
volatile bool pending_bumpless_transfer = false;

// Debugging
char tx_buffer[128];

const int num_ff_points = 9;
const float ff_rpm_points[] = { 740, 1250, 1600, 1860, 2100, 2325, 2515, 2675, 2830 };
const float ff_pwm_points[] = { 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0 };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void StartPidTask(void *argument);
void StartUsbTask(void *argument);
void StartTachoTask(void *argument);

/* USER CODE BEGIN PFP */
void set_fan_speed(float duty_cycle);
void process_usb_command(uint8_t* Buf, uint32_t Len);
bool Save_Settings(void);
void Load_Settings(void);
float calculate_feedforward_pwm(float rpm);

/* Atomic read helpers for multi-byte volatile variables */
static inline float atomic_read_float(volatile float *ptr)
{
    float val;
    osKernelLock();
    val = *ptr;
    osKernelUnlock();
    return val;
}

static inline bool atomic_read_bool(volatile bool *ptr)
{
    bool val;
    osKernelLock();
    val = *ptr;
    osKernelUnlock();
    return val;
}

static inline void atomic_write_float(volatile float *ptr, float val)
{
    osKernelLock();
    *ptr = val;
    osKernelUnlock();
}

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
  Load_Settings(); 

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  // Start the microsecond timer
  HAL_TIM_Base_Start(&htim2);
  // Start the PWM timer channel
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the semaphores */
  tachoSemaphoreHandle = osSemaphoreNew(1, 0, &tachoSemaphore_attributes);

  /* Create the threads */
  pidTaskHandle = osThreadNew(StartPidTask, NULL, &pidTask_attributes);
  usbTaskHandle = osThreadNew(StartUsbTask, NULL, &usbTask_attributes);
  tachoTaskHandle = osThreadNew(StartTachoTask, NULL, &tachoTask_attributes);

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
  * @retval None
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /* PA0 (Tachometer Input) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIO pins : PD12 PD13 PD14 PD15 (On-board LEDs) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Calculates the feedforward PWM value using linear interpolation.
 * @param rpm: The target RPM.
 * @retval The estimated PWM value required to achieve that RPM.
 */
float calculate_feedforward_pwm(float rpm)
{
    if (rpm <= ff_rpm_points[0])
    {
        return ff_pwm_points[0];
    }
    if (rpm >= ff_rpm_points[num_ff_points - 1])
    {
        return ff_pwm_points[num_ff_points - 1];
    }

    for (int i = 0; i < num_ff_points - 1; i++)
    {
        if (rpm >= ff_rpm_points[i] && rpm <= ff_rpm_points[i + 1])
        {
            float x0 = ff_rpm_points[i];
            float y0 = ff_pwm_points[i];
            float x1 = ff_rpm_points[i + 1];
            float y1 = ff_pwm_points[i + 1];

            return y0 + (rpm - x0) * (y1 - y0) / (x1 - x0);
        }
    }

    return 0.0f;
}

/**
 * @brief Saves settings to flash with verification.
 * @retval true if successful, false otherwise.
 */
bool Save_Settings(void)
{
    Flash_Settings settings;
    settings.magic_number = FLASH_MAGIC_NUMBER;
    settings.Kp = Kp;
    settings.Ki = Ki;
    settings.Kd = Kd;
    settings.max_rpm = max_rpm;

    bool success = true;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_11;
    EraseInitStruct.NbSectors = 1;
    uint32_t SectorError = 0;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
        uint32_t* p_settings = (uint32_t*)&settings;
        uint32_t num_words = (sizeof(Flash_Settings) + 3) / 4; // Round up to word boundary
        
        for (uint32_t i = 0; i < num_words; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SETTINGS_ADDRESS + (i * 4), p_settings[i]) != HAL_OK)
            {
                success = false;
                break;
            }
        }

        // Verify written data
        if (success)
        {
            uint32_t* flash_data = (uint32_t*)FLASH_SETTINGS_ADDRESS;
            for (uint32_t i = 0; i < num_words; i++)
            {
                if (flash_data[i] != p_settings[i])
                {
                    success = false;
                    break;
                }
            }
        }
    }
    else
    {
        success = false;
    }

    HAL_FLASH_Lock();
    return success;
}

/**
 * @brief Loads settings from flash if valid.
 */
void Load_Settings(void)
{
    uint32_t magic_check = *(__IO uint32_t*)FLASH_SETTINGS_ADDRESS;

    if (magic_check == FLASH_MAGIC_NUMBER)
    {
        Kp = *(__IO float*)(FLASH_SETTINGS_ADDRESS + 4);
        Ki = *(__IO float*)(FLASH_SETTINGS_ADDRESS + 8);
        Kd = *(__IO float*)(FLASH_SETTINGS_ADDRESS + 12);
        max_rpm = *(__IO float*)(FLASH_SETTINGS_ADDRESS + 16);
    }
}


/**
 * @brief  Function implementing the tachoTask thread.
 */
void StartTachoTask(void *argument)
{
  bool is_first_pulse = true;
  float local_max_rpm;

  for (;;)
  {
    if (osSemaphoreAcquire(tachoSemaphoreHandle, TACHO_TIMEOUT_MS) == osOK)
    {
      if (is_first_pulse) {
        is_first_pulse = false;
        last_capture_time_us = current_capture_time_us;
        continue;
      }

      uint32_t dt_us = current_capture_time_us - last_capture_time_us;
      last_capture_time_us = current_capture_time_us;

      if (dt_us > 0) {
        float new_rpm = (60000000.0f) / (dt_us * PULSES_PER_REVOLUTION);

        local_max_rpm = atomic_read_float(&max_rpm);
        if (new_rpm <= local_max_rpm)
        {
            atomic_write_float(&measured_rpm, new_rpm);
        }
      }
    }
    else
    {
      atomic_write_float(&measured_rpm, 0.0f);

      osKernelLock();
      integral = 0.0f;
      osKernelUnlock();

      is_first_pulse = true;
    }
  }
}

/**
 * @brief Function implementing the pidTask thread.
 */
void StartPidTask(void *argument)
{
  float last_measured_rpm = 0.0f;
  uint32_t tick = osKernelGetTickCount();

  osDelay(2000);

  for(;;)
  {
    tick += (uint32_t)(PID_SAMPLE_TIME_S * 1000.0f);
    osDelayUntil(tick);

    // Read shared variables atomically
    bool is_manual = atomic_read_bool(&manual_mode);
    float local_setpoint = atomic_read_float(&setpoint_rpm);
    float local_measured = atomic_read_float(&measured_rpm);
    float local_manual_pwm = atomic_read_float(&manual_pwm);
    float local_Kp = atomic_read_float(&Kp);
    float local_Ki = atomic_read_float(&Ki);
    float local_Kd = atomic_read_float(&Kd);

    // Check for bumpless transfer request
    osKernelLock();
    bool do_bumpless = pending_bumpless_transfer;
    pending_bumpless_transfer = false;
    osKernelUnlock();

    if (do_bumpless && !is_manual)
    {
        // Initialize integral to produce current output smoothly
        float ff_term = calculate_feedforward_pwm(local_setpoint);
        float current_output = atomic_read_float(&pid_output);
        float error = local_setpoint - local_measured;
        float p_term = local_Kp * error;
        
        // Back-calculate integral: output = ff + p + Ki*integral
        // So integral = (output - ff - p) / Ki
        if (local_Ki > 0.001f)
        {
            float new_integral = (current_output - ff_term - p_term) / local_Ki;
            osKernelLock();
            integral = new_integral;
            osKernelUnlock();
        }
        last_measured_rpm = local_measured;
    }

    // Check for manual override first
    if (is_manual)
    {
        set_fan_speed(local_manual_pwm);
        atomic_write_float(&pid_output, local_manual_pwm);
        continue;
    }

    // If the setpoint is zero, turn the fan off and reset the controller.
    if (local_setpoint <= 0.0f)
    {
      osKernelLock();
      integral = 0.0f;
      osKernelUnlock();

      last_measured_rpm = 0.0f;
      atomic_write_float(&pid_output, 0.0f);
      set_fan_speed(0.0f);
      continue; 
    }

    // PID Calculation 
    float error = local_setpoint - local_measured;

    // Proportional Term 
    float p_term = local_Kp * error;

    // Feedforward Term 
    float ff_term = calculate_feedforward_pwm(local_setpoint);

    // Integral term
    osKernelLock();
    integral += error * PID_SAMPLE_TIME_S;
    
    // Anti-windup: clamp integral considering feedforward contribution
    // The integral term alone should not exceed what's left after feedforward
    float integral_headroom = PID_OUTPUT_MAX - ff_term;
    float integral_floor = PID_OUTPUT_MIN - ff_term;
    
    if (local_Ki > 0.001f)
    {
        float max_integral = integral_headroom / local_Ki;
        float min_integral = integral_floor / local_Ki;
        if (integral > max_integral) integral = max_integral;
        else if (integral < min_integral) integral = min_integral;
    }
    float current_integral = integral;
    osKernelUnlock();

    // Integral Term
    float i_term = local_Ki * current_integral;

    // Derivative Term
    float derivative = -(local_measured - last_measured_rpm) / PID_SAMPLE_TIME_S;
    float d_term = local_Kd * derivative;

    last_measured_rpm = local_measured;

    // Total PID Output + Feedforward
    float output = ff_term + p_term + i_term + d_term;

    // Clamp the final output
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;

    atomic_write_float(&pid_output, output);
    set_fan_speed(output);
  }
}

/**
* @brief Function implementing the usbTask thread.
*/
void StartUsbTask(void *argument)
{
  osDelay(2500);

  for(;;)
  {
    uint32_t now_ms = HAL_GetTick();

    // Read shared variables atomically for telemetry
    float local_setpoint = atomic_read_float(&setpoint_rpm);
    float local_measured = atomic_read_float(&measured_rpm);
    float local_output = atomic_read_float(&pid_output);

    snprintf(tx_buffer, sizeof(tx_buffer),
             "Time: %7.2f, Set: %.1f, Meas: %.1f, PWM: %.1f\r\n",
             (float)now_ms / 1000.0f,
             local_setpoint,
             local_measured,
             local_output);

    // Retry USB transmit if busy
    uint8_t retries = 3;
    while (retries > 0)
    {
        if (CDC_Transmit_FS((uint8_t*)tx_buffer, strlen(tx_buffer)) == USBD_OK)
        {
            break;
        }
        retries--;
        osDelay(10);
    }

    osDelay(250);
  }
}

/**
  * @brief  Sets the fan's PWM duty cycle.
  * @param  duty_cycle: The desired duty cycle from 0.0f to 100.0f.
  * @retval None
  */
void set_fan_speed(float duty_cycle)
{
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t ccr = (uint32_t)((duty_cycle / 100.0f) * arr);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

/**
  * @brief  External Interrupt ISR for tachometer input (PA0).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        current_capture_time_us = __HAL_TIM_GET_COUNTER(&htim2);
        osSemaphoreRelease(tachoSemaphoreHandle);
    }
}

/**
 * @brief  Parses a float from string with validation.
 * @param  str: Input string.
 * @param  out: Pointer to store result.
 * @retval true if valid number, false otherwise.
 */
static bool parse_float(const char* str, float* out)
{
    if (str == NULL || *str == '\0')
    {
        return false;
    }

    // Skip leading whitespace
    while (*str == ' ' || *str == '\t') str++;

    if (*str == '\0')
    {
        return false;
    }

    char* endptr;
    errno = 0;
    float val = strtof(str, &endptr);

    // Check for conversion errors
    if (errno != 0 || endptr == str)
    {
        return false;
    }

    // Skip trailing whitespace
    while (*endptr == ' ' || *endptr == '\t') endptr++;

    // Should have consumed entire string (or only whitespace remains)
    if (*endptr != '\0')
    {
        return false;
    }

    *out = val;
    return true;
}

/**
 * @brief  Processes received USB data to set RPM and PID gains. (Robust Version)
 * @param  Buf: Buffer of received data.
 * @param  Len: Length of the data.
 * @retval None
 */
void process_usb_command(uint8_t* Buf, uint32_t Len)
{
    static char command_buffer[64];
    static uint32_t buffer_idx = 0;

    for(uint32_t i = 0; i < Len; i++)
    {
        if (Buf[i] == '\n' || Buf[i] == '\r')
        {
            command_buffer[buffer_idx] = '\0';

            if (buffer_idx > 0)
            {
                char command_char = tolower(command_buffer[0]);
                char* value_str = &command_buffer[1];
                float new_val = 0.0f;

                // Check for multi-character commands first
                if (strncmp(command_buffer, "save", 4) == 0)
                {
                    if (Save_Settings())
                    {
                        // Optional: send success message
                    }
                    else
                    {
                        // Optional: send failure message
                    }
                }
                else if (command_char == 's')
                {
                    if (parse_float(value_str, &new_val))
                    {
                        // Switching from manual to setpoint triggers bumpless transfer
                        if (atomic_read_bool(&manual_mode))
                        {
                            osKernelLock();
                            pending_bumpless_transfer = true;
                            osKernelUnlock();
                        }
                        manual_mode = false;
                        if (new_val < 0.0f) new_val = 0.0f;
                        if (new_val > 4000.0f) new_val = 4000.0f;
                        atomic_write_float(&setpoint_rpm, new_val);
                    }
                }
                else if (command_char == 'm')
                {
                    if (parse_float(value_str, &new_val))
                    {
                        manual_mode = true;
                        if (new_val < 0.0f) new_val = 0.0f;
                        if (new_val > 100.0f) new_val = 100.0f;
                        atomic_write_float(&manual_pwm, new_val);
                    }
                }
                else if (command_char == 'p')
                {
                    if (parse_float(value_str, &new_val))
                    {
                        if (new_val < 0.0f) new_val = 0.0f;
                        atomic_write_float(&Kp, new_val);
                    }
                }
                else if (command_char == 'i')
                {
                    if (parse_float(value_str, &new_val))
                    {
                        if (new_val < 0.0f) new_val = 0.0f;
                        atomic_write_float(&Ki, new_val);
                    }
                }
                else if (command_char == 'd')
                {
                    if (parse_float(value_str, &new_val))
                    {
                        if (new_val < 0.0f) new_val = 0.0f;
                        atomic_write_float(&Kd, new_val);
                    }
                }
                else if (command_char == 'r')
                {
                    // New command: set max_rpm
                    if (parse_float(value_str, &new_val))
                    {
                        if (new_val < 100.0f) new_val = 100.0f;
                        if (new_val > 10000.0f) new_val = 10000.0f;
                        atomic_write_float(&max_rpm, new_val);
                    }
                }
            }

            buffer_idx = 0;
            memset(command_buffer, 0, sizeof(command_buffer));
        }
        else
        {
            if (buffer_idx < sizeof(command_buffer) - 1)
            {
                command_buffer[buffer_idx++] = Buf[i];
            }
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
  __disable_irq();
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