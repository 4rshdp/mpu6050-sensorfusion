/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <math.h>

#include "mpu6000.h"
#include "imu_fusion.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_TIME_MS_USB  50





// Noise and covariance parameters
_float_t Pdiag[EKF_N] = {0.01f, 0.01f};  // Initial covariance for roll and pitch
_float_t Q[EKF_N * EKF_N] = {
    0.001f, 0.0f,
    0.0f,   0.001f
};
// Measurement noise covariance for the 3-axis accelerometer
_float_t R[EKF_M * EKF_M] = {
    0.03f, 0.0f,  0.0f,
    0.0f,  0.03f, 0.0f,
    0.0f,  0.0f,  0.03f
};

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

ekf_t ekf;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

/* USER CODE BEGIN PV */

// Declare a persistent buffer for USB transmission
char usb_tx_buffer[150];  // Adjust size if necessary

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief Sensor model function.
 *
 * This function computes the predicted accelerometer measurements (hx) due to gravity
 * given the current roll (phi) and pitch (theta) estimates, and calculates the Jacobian matrix (H)
 * of the measurement function.
 *
 * The model used is:
 *    ax = -g * sin(theta)
 *    ay =  g * sin(phi) * cos(theta)
 *    az =  g * cos(phi) * cos(theta)
 *
 * The Jacobian matrix H is computed as:
 *
 *  [  0,             -g*cos(theta)             ]
 *  [ g*cos(phi)*cos(theta),   -g*sin(phi)*sin(theta) ]
 *  [ -g*sin(phi)*cos(theta),  -g*cos(phi)*sin(theta) ]
 *
 * @param ekf Pointer to the current EKF state.
 * @param hx  Output predicted measurement vector (length EKF_M).
 * @param H   Output Jacobian matrix (EKF_M x EKF_N).
 */
static void sensor_model(const ekf_t * ekf, _float_t hx[EKF_M], _float_t H[EKF_M * EKF_N])
{
    // Extract current state estimates (roll and pitch in radians)
    _float_t phi   = ekf->x[0];  // roll
    _float_t theta = ekf->x[1];  // pitch
    const _float_t g = 9.81f;

    // Predicted accelerometer measurements (gravity components)
    hx[0] = -g * sin(theta);            // ax
    hx[1] =  g * sin(phi) * cos(theta);   // ay
    hx[2] =  g * cos(phi) * cos(theta);   // az

    // Jacobian matrix H = d[hx]/d[x]
    // For ax = -g*sin(theta)
    H[0 * EKF_N + 0] = 0.0f;               // ∂ax/∂phi
    H[0 * EKF_N + 1] = -g * cos(theta);     // ∂ax/∂theta

    // For ay = g*sin(phi)*cos(theta)
    H[1 * EKF_N + 0] = g * cos(phi) * cos(theta);   // ∂ay/∂phi
    H[1 * EKF_N + 1] = -g * sin(phi) * sin(theta);    // ∂ay/∂theta

    // For az = g*cos(phi)*cos(theta)
    H[2 * EKF_N + 0] = -g * sin(phi) * cos(theta);    // ∂az/∂phi
    H[2 * EKF_N + 1] = -g * cos(phi) * sin(theta);    // ∂az/∂theta
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  mpu6050Config();
  mpu6050Read_DMA();

  imu_fusion_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t timerUSB = 0;





  while (1)
  {
	  /* Log data via USB */
	  if ((HAL_GetTick() - timerUSB) >= SAMPLE_TIME_MS_USB) {


		  float dt = SAMPLE_TIME_MS_USB / 1000.0f;
		  float roll_estimate_deg, pitch_estimate_deg;

		  // Update fusion algorithm with sensor data
		  imu_fusion_update(Gx, Gy, Gz, Ax, Ay, Az, dt,
		                    &roll_estimate_deg, &pitch_estimate_deg);

	      snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
	               "%.3f, %.3f\r\n",
				   roll_estimate_deg,
				   pitch_estimate_deg);


	      CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
	      timerUSB = HAL_GetTick();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
