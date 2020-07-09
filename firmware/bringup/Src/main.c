/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum CLI_COMMAND
{
  CLI_ERR,
  CLI_REF,
  CLI_DAC,
  CLI_VOUT,
  CLI_CALDAC,
  CLI_CALADC,
  CLI_SAVE,
  CLI_LOAD,
  CLI_SIZE,
  CLI_RESET,
  CLI_HELP
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRAWCSV 0
#define FLASH_USER_ADDR 0x08010000 /* Start @ of user Flash area */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define log(M, ...)                                                                   \
  sprintf(uartMessage, "[LOG] (%s:%d) - " M "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
  HAL_UART_Transmit(&huart1, (uint8_t *)uartMessage, strlen(uartMessage), HAL_MAX_DELAY);

#define printcl(M, ...)                               \
  sprintf(uartMessage, "\033[K" M "", ##__VA_ARGS__); \
  HAL_UART_Transmit(&huart1, (uint8_t *)uartMessage, strlen(uartMessage), HAL_MAX_DELAY);

#define print(M, ...)                           \
  sprintf(uartMessage, "" M "", ##__VA_ARGS__); \
  HAL_UART_Transmit(&huart1, (uint8_t *)uartMessage, strlen(uartMessage), HAL_MAX_DELAY);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

volatile GPIO_PinState switchState;
volatile GPIO_PinState triggerState;
volatile GPIO_PinState holdState;
volatile uint8_t outLedVal;
volatile uint16_t adcValues[7];
volatile uint16_t dacValue[1];
uint16_t targetDac;
char uartMessage[255];
char inString[32];
volatile char uartRXBuffer[10];

volatile uint32_t printTime = 0;

// Callibration Defines
const int ADC_RESOLUTION = 12;
uint16_t SAMPLES = 50; // 5000

uint16_t MAXADC;                 // maximum possible reading from ADC
float VREF = 3.3;                // ADC reference voltage (= power supply)
float VINPUT = 1.65;             // ADC input voltage from Normalized Jack
int16_t EXPECTED;                // expected ADC reading
int16_t adcCalibration[7] = {0}; // calibration table;
int16_t dacCalibration = 0;
int16_t adcOffset[7] = {0}; // offset table;

volatile ITStatus UartRXReady = RESET;
volatile ITStatus UartTXReady = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LED_introSequence(void);
static GPIO_PinState read_Switch(void);
static void setPWM(uint8_t);
static void moveCursor(uint8_t, uint8_t);

static uint16_t read_ADC_Raw(ADC_Inputs);
static float read_ADC_Normalized(ADC_Inputs);
static float read_ADC_Voltage(ADC_Inputs);
static void Write_Flash(uint32_t, uint32_t *, uint32_t);
static uint32_t Read_Flash(uint32_t);
static uint32_t Get_Page(uint32_t);
static void save_Calibration(void);
static void load_Calibration(void);
static void read_User_Input(void);
static void parse_Command(void);
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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  dacValue[0] = 0;
  MAXADC = (1 << ADC_RESOLUTION) - 1;  // maximum possible reading from ADC
  EXPECTED = MAXADC * (VINPUT / VREF); // expected ADC reading

  inString[0] = '\0';

  HAL_DAC_Init(&hdac1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcValues, 7);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dacValue, 1, DAC_ALIGN_12B_R);

  LED_introSequence();
  outLedVal = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    read_Switch();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ADC statistics

    if ((HAL_GetTick() - printTime) > 100)
    {
      printTime = HAL_GetTick();
      setPWM(outLedVal);
      outLedVal++;

      print("\033[?6h \033[H"); // [2J clear entire screen
      print("flash: %lx\n", Read_Flash(0x08010000));
      printcl("LED: %d\n", outLedVal); // [2J clear entire screen
      printcl("DAC: %d \t OFF: %d\n", dacValue[0], dacCalibration);
      print("TRIG: %d\n", triggerState);
      print("HOLD: %d\n", holdState);
      print("CYCLE: %d\n", switchState);

      int tab = 3;
      moveCursor(7, tab);
      if (!DRAWCSV)
      {
        print("PIN");
        moveCursor(7, tab += 8);
        print("ADC");
        moveCursor(7, tab += 8);
        print("ADC V");
        moveCursor(7, tab += 8);
        print("VIN");
        moveCursor(7, tab += 8);
        print("VAR");
        moveCursor(7, tab += 8);
        print("STD");
        moveCursor(7, tab += 8);
        print("AVG");
        moveCursor(7, tab += 8);
        print("PPN");
        moveCursor(7, tab += 8);
        print("OFF");
      }
      else
      {
        moveCursor(7, tab += 8);
        print("PIN, ");
        moveCursor(7, tab += 8);
        print("ADC, ");
        moveCursor(7, tab += 8);
        print("VAR, ");
        moveCursor(7, tab += 8);
        print("STD, ");
        moveCursor(7, tab += 8);
        print("AVG, ");
        moveCursor(7, tab += 8);
        print("PPN");
      }

      float maxVariance = 0;
      float maxSTD = 0;
      int maxPPN = 0;
      float avrVariance = 0;
      float avrSTD = 0;
      int avrPPN = 0;

      for (int i = 0; i < 7; i++)
      {
        long datSum = 0; // reset our accumulated sum of input values to zero
        int sMax = 0;
        int sMin = MAXADC;
        long n;                                     // count of how many readings so far
        double x, mean, delta, m2, variance, stdev; // to calculate standard deviation

        n = 0;    // have not made any ADC readings yet
        mean = 0; // start off with running mean at zero
        m2 = 0;

        tab = 3;
        moveCursor(8 + i, tab);

        for (int j = 0; j < SAMPLES; j++)
        {
          x = adcValues[i] - adcCalibration[i]; // callibration offset
          datSum += x;
          if (x > sMax)
            sMax = x;
          if (x < sMin)
            sMin = x;
          n++;
          delta = x - mean;
          mean += delta / n;
          m2 += (delta * (x - mean));
        }

        variance = m2 / (n - 1); // (n-1):Sample Variance	(n): Population Variance
        stdev = sqrt(variance);  // Calculate standard deviation
        float datAvg = (1.0 * datSum) / n;
        int ppNoise = sMax - sMin;

        if (ppNoise > maxPPN)
          maxPPN = ppNoise;
        if (stdev > maxSTD)
          maxSTD = stdev;
        if (variance > maxVariance)
          maxVariance = variance;
        avrPPN += ppNoise;
        avrSTD += stdev;
        avrVariance += variance;

        int sOffset;

        sOffset = datAvg - EXPECTED;

        adcOffset[i] = sOffset;

        moveCursor(8 + i, tab);
        printcl("A%d:", i);
        moveCursor(8 + i, tab += 8);
        print("%d", adcValues[i]);
        moveCursor(8 + i, tab += 8);
        print("%.2f", (adcValues[i] / (float)MAXADC) * 3.3);
        moveCursor(8 + i, tab += 8);
        if (i > 2)
        {
          print("%.2f", ((adcValues[i] / (float)MAXADC) * 3.3) / -0.33 + 5);
        }
        moveCursor(8 + i, tab += 8);
        print("%.2f", variance);
        moveCursor(8 + i, tab += 8);
        print("%.2f", stdev);
        moveCursor(8 + i, tab += 8);
        print("%.1f", datAvg);
        moveCursor(8 + i, tab += 8);
        print("%d", ppNoise);
        moveCursor(8 + i, tab += 8);
        print("%d", sOffset);
      }
      // print stats for all ADC pins
      avrPPN /= 12;
      avrSTD /= 12.0;
      avrVariance /= 12.0;
      tab = 3 + 8 * 3;
      moveCursor(16, tab);
      printcl("Avr:");
      moveCursor(17, tab);
      printcl("Max:");

      moveCursor(16, tab + 8);
      print("%.2f", maxVariance);
      moveCursor(17, tab + 8);
      print("%.2f", avrVariance);
      moveCursor(16, tab + 16);
      print("%.2f", maxSTD);
      moveCursor(17, tab + 16);
      print("%.2f", avrSTD);
      moveCursor(16, tab + 32);
      print("%d", maxPPN);
      moveCursor(17, tab + 32);
      print("%d", avrPPN);

      print("\n %.2f \t\t %.2f", read_ADC_Voltage(ADC_ATTACK_POT), read_ADC_Voltage(ADC_ATTACK_CV));
    }

    read_User_Input();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 984;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1450;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EOC_LED_Pin | CYC_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EOC_Pin | HOLD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EOC_LED_Pin CYC_LED_Pin */
  GPIO_InitStruct.Pin = EOC_LED_Pin | CYC_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_A_Pin */
  GPIO_InitStruct.Pin = SW_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOLD_Pin TRIGGER_Pin */
  GPIO_InitStruct.Pin = HOLD_Pin | TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EOC_Pin HOLD_LED_Pin */
  GPIO_InitStruct.Pin = EOC_Pin | HOLD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void LED_introSequence()
{
  HAL_GPIO_WritePin(CYC_LED_GPIO_Port, CYC_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(EOC_LED_GPIO_Port, EOC_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  setPWM(255);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CYC_LED_GPIO_Port, CYC_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(EOC_LED_GPIO_Port, EOC_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  setPWM(0);
  HAL_Delay(100);
}

GPIO_PinState read_Switch()
{
  GPIO_PinState t = HAL_GPIO_ReadPin(SW_A_GPIO_Port, SW_A_Pin);
  if (switchState != t)
  {
    switchState = t;
    if (switchState == GPIO_PIN_RESET)
      HAL_GPIO_TogglePin(CYC_LED_GPIO_Port, CYC_LED_Pin);
  }
  return t;
}

// Set PWM output value for OUT LED
void setPWM(uint8_t val)
{
  htim1.Instance->CCR4 = val;
}

// update function for DSP
void update()
{
  //HAL_GPIO_WritePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin, GPIO_PIN_SET);
  //dacValue[0] = adcValues[0]; //adcValues[3]; //4095; //rand();
  dacValue[0] = targetDac;
  //HAL_GPIO_WritePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin, GPIO_PIN_RESET);
}

// trigger input callback handler
void triggerHandler(GPIO_PinState state)
{
  triggerState = state;
  HAL_GPIO_WritePin(EOC_LED_GPIO_Port, EOC_LED_Pin, state);
}

// hold input callback handler
void holdHandler(GPIO_PinState state)
{
  holdState = state;
  HAL_GPIO_WritePin(EOC_LED_GPIO_Port, EOC_LED_Pin, state);
}

// ADC conversion complete callback handler
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  update();
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dacValue, 1, DAC_ALIGN_12B_R);
}

void moveCursor(uint8_t l, uint8_t c)
{
  print("\033[%d;%dH", l, c);
}

uint16_t read_ADC_Raw(ADC_Inputs pin)
{
  if (pin < 0)
    pin = 0;
  if (pin > 6)
    pin = 6;

  if (pin < 3) // potentiometers
    return adcValues[pin];
  else // CV
    return MAXADC - (adcValues[pin] - adcCalibration[pin]);
}

float read_ADC_Normalized(ADC_Inputs pin)
{
  if (pin < 0)
    pin = 0;
  if (pin > 6)
    pin = 6;

  if (pin < 3) // potentiometers
    return (float)adcValues[pin] / MAXADC;
  else // CV
    return (float)(adcValues[pin] - adcCalibration[pin]) / MAXADC * -2.0 + 1;
}

float read_ADC_Voltage(ADC_Inputs pin)
{
  if (pin < 0)
    pin = 0;
  if (pin > 6)
    pin = 6;

  if (pin < 3) // potentiometers
    return (float)adcValues[pin] / MAXADC * 3.3;
  else // CV
    return ((float)(adcValues[pin] - adcCalibration[pin]) / (float)MAXADC * 3.3) / -0.33 + 5;
}

void Write_Flash(uint32_t address, uint32_t *data, uint32_t size)
{
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = FLASH_BANK_1;
  EraseInitStruct.Page = Get_Page(address);
  EraseInitStruct.NbPages = 1;
  HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
  __disable_irq();
  for (int i = 0; i < size; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data[i]);
    address += 8;
  }
  __enable_irq();
  HAL_FLASH_Lock();
}

uint32_t Read_Flash(uint32_t address)
{
  return *(__IO uint32_t *)address;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t Get_Page(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

void load_Calibration(void)
{
  if (Read_Flash(FLASH_USER_ADDR) == 0xDEAD)
  {
    uint32_t address = FLASH_USER_ADDR + 8;
    for (int i = 0; i < 7; i++)
    {
      adcCalibration[i] = (int32_t)Read_Flash(address);
      address += 8;
    }
    dacCalibration = (int32_t)Read_Flash(address);
  }
  else
  {
    log("No Calibration Data");
  }
}

void save_Calibration(void)
{
  uint32_t flashData[9] = {0};
  flashData[0] = 0xDEAD;
  for (int i = 0; i < 7; i++)
  {
    flashData[i + 1] = adcCalibration[i];
  }
  flashData[8] = dacCalibration;
  Write_Flash(FLASH_USER_ADDR, flashData, 9);
}

void read_User_Input()
{
  //log("user input");
  if (UartRXReady != SET)
  {
    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)uartRXBuffer, 1) != HAL_OK)
    {
      Error_Handler();
    }
  }
  //while (UartRXReady != SET)
  //{
  //}

  if (UartRXReady == SET)
  {
    UartRXReady = RESET;

    //log("user: %c - 0x%X", uartRXBuffer[0], uartRXBuffer[0]);
    if (uartRXBuffer[0] == '\n')
    {
      moveCursor(30, 2);
      printcl(""); // print cl

      parse_Command();
      inString[0] = '\0';
    }
    else if (uartRXBuffer[0] == 0x8)
    {
      if (strlen(inString) > 0)
      {
        inString[strlen(inString) - 1] = '\0';
      }
      //log("BS");
    }
    else if ((uartRXBuffer[0] >= 0x20) && (uartRXBuffer[0] <= 0x126))
    {
      sprintf(inString, "%s%c", inString, uartRXBuffer[0]);
    }

    if (strlen(inString) > 0)
    {
      moveCursor(30, 2);
      printcl(">>> %s", inString); // print cl
    }
  }
}

void parse_Command()
{
  char *pch;
  pch = strtok(inString, " ");
  uint8_t command = CLI_ERR;
  uint8_t arguments = 0;
  moveCursor(29, 2);

  while (pch != NULL)
  {

    if (strncmp(pch, "ref", 3) == 0)
    {
      printcl(" %s", pch);
      command = CLI_REF;
    }
    else if (strncmp(pch, "dac", 3) == 0)
    {
      printcl(" %s", pch);
      command = CLI_DAC;
    }
    else if (strncmp(pch, "vout", 4) == 0)
    {
      printcl(" %s", pch);
      command = CLI_VOUT;
    }
    else if (strncmp(pch, "caladc", 6) == 0)
    {
      printcl(" %s", pch);
      command = CLI_CALADC;
      // callibrate
      for (int i = 3; i < 7; i++)
      {
        adcCalibration[i] = adcOffset[i];
      }
    }
    else if (strncmp(pch, "caldac", 6) == 0)
    {
      printcl(" %s", pch);
      command = CLI_CALDAC;
      dacCalibration = dacValue[0];
    }
    else if (strncmp(pch, "load", 4) == 0)
    {
      printcl(" %s", pch);
      command = CLI_LOAD;
      load_Calibration();
    }
    else if (strncmp(pch, "save", 4) == 0)
    {
      printcl(" %s", pch);
      command = CLI_SAVE;
      save_Calibration();
    }
    else if (strncmp(pch, "reset", 5) == 0)
    {
      printcl(" %s", pch);
      command = CLI_RESET;
      for (int i = 0; i < 7; i++)
      {
        adcCalibration[i] = 0;
      }
      dacCalibration = 0;
    }
    else if (strncmp(pch, "size", 3) == 0)
    {
      printcl(" %s", pch);
      command = CLI_SIZE;
    }
    else if (strncmp(pch, "help", 4) == 0)
    {
      printcl(" commands: ref {float}, dac {float}, cal, load, save, reset, size {int}, help");
      command = CLI_HELP;
    }
    else
    {
      if (arguments == 0)
        printcl(" command not found");
    }

    if (arguments == 1)
    {
      float arg = atof(pch);

      switch (command)
      {
      case CLI_REF:
        if (arg > 12.0)
          arg = 12.0;
        if (arg < -12.0)
          arg = -12.0;
        VINPUT = (arg * -0.33) + 1.65;
        EXPECTED = MAXADC * (VINPUT / VREF); // expected ADC reading
        break;
      case CLI_DAC:
        if (arg > (float)MAXADC)
          arg = (float)MAXADC;
        if (arg < 0.0)
          arg = 0.0;
        targetDac = arg;
        print(" val: %s", pch);
        break;
      case CLI_VOUT:
        if (arg > 10.0)
          arg = 10.0;
        if (arg < 0.0)
          arg = 0.0;
        targetDac = (arg / 10.0) * MAXADC;
        print("arg: %f dac: %d", arg, targetDac);
        break;
      case CLI_SIZE:
        if (arg > 5000.0)
          arg = 5000.0;
        if (arg < 1.0)
          arg = 1.0;
        SAMPLES = arg;
        break;
      default:
        break;
      }
    }

    pch = strtok(NULL, " ");
    arguments++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UartRXReady = SET;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UartTXReady = SET;
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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
