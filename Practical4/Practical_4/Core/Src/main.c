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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS        128       // Number of samples in LUT
#define TIM2CLK   16000000UL  // STM Clock frequency: Hint You might want to check the ioc file
#define F_SIGNAL  440        // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
// more efficient for 12-bit data
// variables generated using python
// Wave selection state
typedef enum { W_SINE = 0, W_SAW, W_TRI, W_PIANO, W_GUITAR, W_DRUM, W__N } wave_t;
volatile wave_t g_wave = W_SINE;    // start on Sine
volatile uint32_t press_count = 0; // in PV

// Map enum -> LUT pointer
static inline uint16_t* wave_ptr(wave_t w){
	extern uint16_t Sine_LUT[NS];
	extern uint16_t Saw_LUT[NS];
	extern uint16_t Triangle_LUT[NS];
	extern uint16_t Piano_LUT[NS];
	extern uint16_t Guitar_LUT[NS];
	extern uint16_t Drum_LUT[NS];
  switch (w){
    case W_SINE:   return (uint16_t*)Sine_LUT;
    case W_SAW:    return (uint16_t*)Saw_LUT;
    case W_TRI:    return (uint16_t*)Triangle_LUT;
    case W_PIANO:  return (uint16_t*)Piano_LUT;
    case W_GUITAR: return (uint16_t*)Guitar_LUT;
    case W_DRUM:   return (uint16_t*)Drum_LUT;
    default:       return (uint16_t*)Sine_LUT;
  }
}

// Map enum -> label for LCD
static inline const char* wave_name(wave_t w){
  static const char* N[] = {"Sine","Sawtooth","Triangular","Piano","Guitar","Drum"};
  return N[(int)w];
}
uint16_t Sine_LUT[NS] = {
  2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737, 2831, 2923, 3013, 3100, 3185, 3267, 3346, 3423, 3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898, 3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093, 4095, 4093, 4085, 4073, 4056, 4034, 4007, 3975, 3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565, 3495, 3423, 3346, 3267, 3185, 3100, 3013, 2923, 2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148, 2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358, 1264, 1172, 1082, 995, 910, 828, 749, 672, 600, 530, 465, 403, 345, 291, 242, 197, 156, 120, 88, 61, 39, 22, 10, 2, 0, 2, 10, 22, 39, 61, 88, 120, 156, 197, 242, 291, 345, 403, 465, 530, 600, 672, 749, 828, 910, 995, 1082, 1172, 1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947
};

uint16_t Saw_LUT[NS] = {
  0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1024, 1056, 1088, 1120, 1152, 1184, 1216, 1248, 1280, 1312, 1344, 1376, 1408, 1440, 1472, 1504, 1536, 1568, 1600, 1632, 1664, 1696, 1728, 1760, 1792, 1824, 1856, 1888, 1920, 1952, 1984, 2016, 2048, 2079, 2111, 2143, 2175, 2207, 2239, 2271, 2303, 2335, 2367, 2399, 2431, 2463, 2495, 2527, 2559, 2591, 2623, 2655, 2687, 2719, 2751, 2783, 2815, 2847, 2879, 2911, 2943, 2975, 3007, 3039, 3071, 3103, 3135, 3167, 3199, 3231, 3263, 3295, 3327, 3359, 3391, 3423, 3455, 3487, 3519, 3551, 3583, 3615, 3647, 3679, 3711, 3743, 3775, 3807, 3839, 3871, 3903, 3935, 3967, 3999, 4031, 4063
};

uint16_t Triangle_LUT[NS] = {
  0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856, 1920, 1984, 2048, 2111, 2175, 2239, 2303, 2367, 2431, 2495, 2559, 2623, 2687, 2751, 2815, 2879, 2943, 3007, 3071, 3135, 3199, 3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711, 3775, 3839, 3903, 3967, 4031, 4095, 4031, 3967, 3903, 3839, 3775, 3711, 3647, 3583, 3519, 3455, 3391, 3327, 3263, 3199, 3135, 3071, 3007, 2943, 2879, 2815, 2751, 2687, 2623, 2559, 2495, 2431, 2367, 2303, 2239, 2175, 2111, 2048, 1984, 1920, 1856, 1792, 1728, 1664, 1600, 1536, 1472, 1408, 1344, 1280, 1216, 1152, 1088, 1024, 960, 896, 832, 768, 704, 640, 576, 512, 448, 384, 320, 256, 192, 128, 64
};

uint16_t Piano_LUT[NS] = {
  1396, 1332, 1275, 1217, 1175, 1149, 1144, 1147, 1161, 1185, 1213, 1255, 1295, 1334, 1370, 1403, 1426, 1445, 1487, 1536, 1582, 1620, 1672, 1739, 1800, 1876, 1965, 2061, 2172, 2296, 2429, 2568, 2694, 2787, 2876, 2970, 3067, 3166, 3253, 3320, 3371, 3408, 3442, 3493, 3550, 3611, 3688, 3774, 3861, 3926, 3972, 4016, 4063, 4095, 4095, 4081, 4063, 4036, 3994, 3934, 3858, 3761, 3649, 3528, 3420, 3311, 3188, 3059, 2926, 2809, 2709, 2616, 2532, 2448, 2377, 2314, 2245, 2172, 2096, 2004, 1901, 1801, 1702, 1606, 1508, 1401, 1295, 1195, 1100, 1002, 911, 824, 739, 674, 623, 568, 511, 460, 417, 380, 343, 313, 291, 269, 248, 235, 237, 246, 252, 254, 271, 313, 376, 454, 528, 597, 661, 722, 786, 850, 917, 987, 1062, 1141, 1222, 1300, 1370, 1446
};

uint16_t Guitar_LUT[NS] = {
  1406, 1072, 1545, 1634, 1547, 1850, 2049, 1785, 1494, 1740, 1845, 1503, 1440, 1755, 1980, 1885, 1791, 1908, 2032, 2020, 1897, 1922, 2088, 2103, 2013, 2167, 2662, 2923, 2657, 2622, 3070, 3198, 2977, 2975, 3139, 3263, 3195, 3225, 3517, 3466, 3227, 3230, 3382, 3544, 3422, 3212, 3032, 2876, 3074, 3246, 3341, 3642, 3689, 3589, 3739, 3912, 3904, 3693, 3584, 3852, 3926, 3653, 3778, 4023, 3898, 3790, 3808, 3969, 4095, 3976, 3937, 3951, 3990, 4039, 3883, 3895, 4072, 3964, 3856, 4000, 3966, 3621, 3362, 3197, 3110, 3188, 3035, 2876, 3201, 3496, 3430, 3215, 2896, 2690, 2811, 3018, 2971, 2833, 2913, 2993, 2890, 2736, 2713, 2932, 3034, 2877, 2825, 2826, 2854, 3011, 3092, 3073, 3073, 3213, 3397, 3377, 3340, 3303, 3160, 3194, 3272, 3155, 3062, 3074, 3128, 3188, 3189, 2946
};

uint16_t Drum_LUT[NS] = {
  5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1
};

// TODO: Equation to calculate TIM2_Ticks
// uint32_t TIM2_Ticks = 0; // How often to write new LUT value
uint32_t TIM2_Ticks = (uint32_t)(TIM2CLK / (NS * (uint32_t)F_SIGNAL));
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle
volatile uint32_t dma_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  init_LCD();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TIM2_Ticks - 1);  // prime CCR1
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  //uint16_t *src = Sine_LUT;  // (no cast needed)
  uint16_t *src = wave_ptr(g_wave);
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)src, DestAddress, NS);
  // TODO: Write current waveform to LCD(Sine is the first waveform)
  lcd_command(CLEAR);
  HAL_Delay(30);
  lcd_putstring((char*)"Sine");
  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief TIM2 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1; // was 4294967295
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

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = TIM2_Ticks - 1; // was 0
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;          // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;     // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;         // Memory address increments
  //hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  //hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // was WORD
  hdma_tim2_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // was WORD
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;              // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */
}

/**
* @brief TIM3 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095;

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

  HAL_TIM_MspPostInit(&htim3);
}

/**
* Enable DMA controller clock
*/
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Button0_Pin */
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){
	press_count++;   // proves ISR is running
	HAL_GPIO_EXTI_IRQHandler(Button0_Pin);   // clear EXTI flag first

	  static volatile uint8_t swapping = 0;
	  if (swapping) return;
	  swapping = 1;

	  static uint32_t last = 0;
	  uint32_t now = HAL_GetTick();
	  if (now - last < 150) { swapping = 0; return; }  // debounce early-exit
	  last = now;

	  // advance waveform
	  g_wave = (g_wave + 1) % W__N;

	  // stop → swap → restart DMA
	  __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	  HAL_DMA_Abort_IT(&hdma_tim2_ch1);

	  uint16_t *src = wave_ptr(g_wave);
	  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)src, DestAddress, NS);
	  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

	  // update LCD
	  lcd_command(CLEAR);
	  lcd_putstring((char*)wave_name(g_wave));

	  swapping = 0;
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
