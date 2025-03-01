/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ci24r1.h"
#include "stdio.h"
#include "SEGGER_RTT.h"
#include "motor.h"

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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void print_buf(uint8_t* buf,uint8_t buflen)
{ 
  for(int i = 0; i<buflen; i++)
  {
    SEGGER_RTT_printf(0, "%#x ",buf[i]);
  }
  SEGGER_RTT_printf(0, "\r\n");
  
}

//#define TXMODE
#define ADC_CH_CNT 2
#define ADC_AVG_CNT 10
uint8_t adc_value[ADC_CH_CNT * ADC_AVG_CNT] = {0}; // adc dma buffer

u8 adc_get_val(u8 idx)
{
	int sum = 0;
  for(int i=0; i<ADC_AVG_CNT; i++)
  {
    sum += adc_value[idx + i * ADC_CH_CNT];
  }
  return sum/ADC_AVG_CNT;
}



#define SOFTPWMCHN 4

int SoftPwmDuty[SOFTPWMCHN] = {0,0,0,0};
GPIO_TypeDef* SoftPwmPort[SOFTPWMCHN] = {T2CH1_GPIO_Port,T2CH2_GPIO_Port,T2CH3_GPIO_Port,T2CH4_GPIO_Port};
uint16_t SoftPwmPin[SOFTPWMCHN] = {T2CH1_Pin,T2CH2_Pin,T2CH3_Pin,T2CH4_Pin};

void SoftPwmInit(void)
{
  //nothing to do
}
static u8 counter = 0;
// put in tim17 interrupt
void SoftPwmService(void)
{
    
    int i;


    if(counter == 0)
    {
      for(i=0; i<SOFTPWMCHN; i++)
      {
          if(SoftPwmDuty[i] == 0)
          {
            HAL_GPIO_WritePin(SoftPwmPort[i],SoftPwmPin[i],GPIO_PIN_RESET);
          }   
          else
          {
            HAL_GPIO_WritePin(SoftPwmPort[i],SoftPwmPin[i],GPIO_PIN_SET);
          }
           
      }
      counter++;
    }
    else if(counter >= 10)   
    {
      counter = 0;
      for(i=0; i<SOFTPWMCHN; i++)
      {
					if(SoftPwmDuty[i] == 10)
          {
            HAL_GPIO_WritePin(SoftPwmPort[i],SoftPwmPin[i],GPIO_PIN_SET);
          }   
          else
          {
            HAL_GPIO_WritePin(SoftPwmPort[i],SoftPwmPin[i],GPIO_PIN_RESET);
          }
      }
    }
    else// 1~9   
    {
      for(i=0; i<SOFTPWMCHN; i++)
      {
        if(SoftPwmDuty[i] == counter)
          HAL_GPIO_WritePin(SoftPwmPort[i], SoftPwmPin[i], GPIO_PIN_RESET);
      }
      counter++;
    }

    

}
//ch 0-3 duty 0-1000  
void SoftPwmSetDuty(u8 ch,u16 dutyin)
{
    u8 duty = dutyin / 100;
    
    if(dutyin >= 910) 
      duty = 10;

    if(duty > 10)
        duty = 10;
    if(ch >= 4)
        return;
    SoftPwmDuty[ch] = duty;
}




void limit_init(void)
{
  //cubemax already inital
}

void debounce(u8 originInput,u8 *output,u32 mask,int* bounceTimer)
{
  if (originInput & mask)
  {
    //SEGGER_RTT_printf(0, "max1 trig 1 %d\r\n", HAL_GetTick());
    (*bounceTimer)++;
    if (*bounceTimer >= 5)
    {
      //SEGGER_RTT_printf(0, "max1 trig 2 %d\r\n", HAL_GetTick());
      *bounceTimer = 5; // 一直锁定在这里
      *output |= mask;
    }
    else
    {
      *output &= ~mask;
    }
  }
  else
  {
    *output &= ~mask;
    //SEGGER_RTT_printf(0, "max1 trig 0 %d\r\n", HAL_GetTick());
    *bounceTimer = 0;
  }
}

int Max1timer = 0;
int Max2timer = 0;
int Max3timer = 0;

void limit_check(void)
{
  
  u8 tmp_limit_state = 0;

  // MAX 输入高为trig
  // MIN 输入低为trig

  if (HAL_GPIO_ReadPin(MAX1_GPIO_Port, MAX1_Pin) == SET)
    tmp_limit_state |= LIMIT_MAX1_TRIG;
  if (HAL_GPIO_ReadPin(MIN1_GPIO_Port, MIN1_Pin) == RESET)
    tmp_limit_state |= LIMIT_MIN1_TRIG;
  if (HAL_GPIO_ReadPin(MAX2_GPIO_Port, MAX2_Pin) == SET)
    tmp_limit_state |= LIMIT_MAX2_TRIG;
  if (HAL_GPIO_ReadPin(MIN2_GPIO_Port, MIN2_Pin) == RESET)
    tmp_limit_state |= LIMIT_MIN2_TRIG;
  if (HAL_GPIO_ReadPin(MAX3_GPIO_Port, MAX3_Pin) == SET)
    tmp_limit_state |= LIMIT_MAX3_TRIG;
  if (HAL_GPIO_ReadPin(MIN3_GPIO_Port, MIN3_Pin) == RESET)
    tmp_limit_state |= LIMIT_MIN3_TRIG;

  g_limit_state = tmp_limit_state;

#if 1
  // max trig信号必须要添加防抖功能,否则电机会出现断断续续现象
	debounce(tmp_limit_state, &g_limit_state, LIMIT_MAX1_TRIG, &Max1timer);
	debounce(tmp_limit_state, &g_limit_state, LIMIT_MAX2_TRIG, &Max2timer);
	debounce(tmp_limit_state, &g_limit_state, LIMIT_MAX3_TRIG, &Max3timer);

  
#endif
}

//0 灭  1亮  2快闪  3慢闪  4toggle
uint32_t ledOnTime = 5;
uint32_t ledOffTime = 10;

void ledSetMode(int mode)
{
  if(mode == 0)
  {
    ledOnTime = 100;
    ledOffTime = 100;
  }
  else if(mode == 1)
  {
    ledOnTime = 0;
    ledOffTime = 100;
  }
  else if(mode == 2)
  {
    ledOnTime = 20;
    ledOffTime = 40;
  }
  else if(mode == 3)
  {
    ledOnTime = 100;
    ledOffTime = 200;
  }
  else if(mode == 4)
  {
    ledOnTime = 0;
    ledOffTime = 0;
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
  }
}
void ledservice(void)
{
  static uint32_t ledTimerCounter = 0;
  if (ledOnTime != 0 || ledOffTime != 0)
  {
    if (++ledTimerCounter >= ledOnTime)
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET); // led on

    if (ledTimerCounter >= ledOffTime)
    {
      ledTimerCounter = 0;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET); // led off
    }
  }
}

uint8_t connectStatus = 0;
uint32_t lastRevFrame = 0;
void connectService(void)
{
    if (connectStatus)
    {
      uint32_t now = HAL_GetTick();
      if (abs(now - lastRevFrame) >= 50)
      {
        connectStatus = 0;
        ledSetMode(3);
      }
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

  HAL_Delay(100);
  SEGGER_RTT_printf(0, "board init\r\n");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_ERROR == HAL_ADCEx_Calibration_Start(&hadc))
  {
    SEGGER_RTT_printf(0, "ADC Calibration error\r\n");
  }
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_value, ADC_CH_CNT * ADC_AVG_CNT);

  motorinit();
  
  uint8_t TX_ADDRESS[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
  uint8_t RX_ADDRESS[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
  ledSetMode(1);
  while (CI24R1_SPI_Test() == HAL_ERROR)
  {
    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
    //HAL_Delay(30);
    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1000);

  }
  SEGGER_RTT_printf(0, "ci24r1 test succ\r\n");
  ledSetMode(3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  CI24R1_Init();
  CI24R1_SetChannel(122);
#ifdef	TXMODE
	CI24R1_SetTxMode();
#else
  CI24R1_SetRxMode();
#endif
  CI24R1_SetTxAddress(TX_ADDRESS);
  CI24R1_SetRxAddress(RX_ADDRESS);



  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);

  uint8_t rxbuf[32];
  uint8_t rxbuflen;
	uint8_t ackpayload[3] = {0};


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // TX
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
    
    #ifdef	TXMODE
    rxbuf[0]++;
		rxbuf[1] = rxbuf[0]+1;
		rxbuf[2] = rxbuf[0]+2;
		rxbuf[3] = rxbuf[0]+4;
    rxbuf[4]= rxbuf[0];
    uint8_t rc = CI24R1_Tx(rxbuf, 0);
    
    #else
 
    CI24R1_WriteACKPload(ackpayload, 3);

    uint8_t rc = CI24R1_Rx(rxbuf, &rxbuflen);
    //print_buf(adc_value,20);
    if(rc & CI24R1_FLAG_RX_READY)
    {
       if(rxbuflen > 0)
      {
        ledSetMode(4);
        connectStatus = 1;
        lastRevFrame = HAL_GetTick();
        motor_set_exp(MOTOR_SMALL_ARM, rxbuf[0], 1);
        motor_set_exp(MOTOR_TURN, rxbuf[1], 1);
        motor_set_exp(MOTOR_BIG_ARM, rxbuf[2], 1);
        motor_set_exp(MOTOR_DIG_BUCKET, rxbuf[3], 1);
        motor_set_exp(MOTOR_LEFT_TRACK, rxbuf[4], 0);
        motor_set_exp(MOTOR_RIGHT_TRACK, rxbuf[6], 1);
        //SEGGER_RTT_printf(0, "rxbuf:%d %d %d %d %d %d\r\n", rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3],rxbuf[4],rxbuf[6]);
      }
    }

    float vcc = adc_value[1];
    vcc = vcc / 256 * 33.3 * (4.9/1.0);
    //float current = adc_value[0];
    //current = current * 3.33 * 100 / 256 / 0.05 / 20; //10mA per unit
    //SEGGER_RTT_printf(0, "%d %d\r\n", get_adc_val(0),get_adc_val(1));
    ackpayload[0] = adc_get_val(0); 
    ackpayload[1] = (int)vcc; 
    ackpayload[2] = g_limit_state;
    //ackpayload[3] = motor_limit_flagh;
    
    //motor_input_current((u8)current);
    
    
    //motor idx  info
    // 0 Digging buckets 1 small arm 2 big arm 3 turn 4 left track 5 right track



#endif
    
    HAL_Delay(2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, T2CH3_Pin|T2CH4_Pin|T2CH1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|T2CH2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYSLED_GPIO_Port, SYSLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : T2CH3_Pin T2CH4_Pin T2CH1_Pin */
  GPIO_InitStruct.Pin = T2CH3_Pin|T2CH4_Pin|T2CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MIN3_Pin MAX3_Pin MAX2_Pin MIN2_Pin
                           MAX1_Pin MIN1_Pin */
  GPIO_InitStruct.Pin = MIN3_Pin|MAX3_Pin|MAX2_Pin|MIN2_Pin
                          |MAX1_Pin|MIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 T2CH2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|T2CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SYSLED_Pin */
  GPIO_InitStruct.Pin = SYSLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYSLED_GPIO_Port, &GPIO_InitStruct);

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
