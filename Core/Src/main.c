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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX_VALUE 255
#define RX_BUFFER_SIZE 128
#define DISTANCE_PER_PULSE 0.01
/*#define CKp_D 170
#define CKp_G 210
#define CKi_D 0
#define CKi_G 0*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

volatile uint8_t T_batt = 0;
volatile uint8_t adcValue;
volatile float batteryVoltage = 0.0f;
volatile uint8_t ADC_on = 0;
unsigned char rxData;
volatile uint32_t pulse_count = 0;
volatile uint32_t last_capture = 0;
volatile uint32_t current_capture = 0;
volatile float speed = 0.0;
volatile int32_t encoder_value_right = 0;
volatile int32_t encoder_value_left = 0;
volatile uint8_t T_enc = 0;
volatile uint32_t encoder_value_right_minus200 = 0;
volatile uint32_t encoder_value_left_minus200 = 0;

volatile uint8_t target10_top = 27;//35
volatile uint8_t target20_top = 68;
volatile uint8_t target30_top = 100;//85
volatile int16_t somme_erreursR = 0;
volatile int16_t somme_erreursL = 0;
volatile uint32_t VG = 0, VD=0;
volatile uint8_t vitesse;
volatile int32_t test;
volatile uint8_t buttonPressed = 0;

typedef enum {
    STATE_NEUTRAL,
    STATE_AV1,
    STATE_AV2,
    STATE_AV3,
    STATE_R1,
    STATE_R2,
    STATE_R3,
    STATE_D1,
    STATE_D2,
    STATE_D3,
    STATE_G1,
    STATE_G2,
    STATE_G3,
    STATE_END,
} State_t;

typedef enum {
    EVENT_AV,
    EVENT_R,
    EVENT_D,
    EVENT_G,
    EVENT_END,
    EVENT_NEUTRAL,
} Event_t;

State_t currentState = STATE_NEUTRAL;
Event_t currentEvent = EVENT_NEUTRAL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void handleEvent(Event_t event);
void executeStateActions(void);
uint32_t calculCommande(int mode, int cote);
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
    MX_ADC1_Init();
    MX_TIM6_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART3_UART_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_UART_Receive_IT(&huart3, &rxData, sizeof(rxData));
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Exécution de la première boucle while
        if (T_batt >= 50)
        {
            T_batt = 0;
            HAL_ADC_Start_IT(&hadc1);
            if (ADC_on == 1)
            {
                ADC_on = 0;
                adcValue = HAL_ADC_GetValue(&hadc1);

                if (adcValue < 231)
                {
                    HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, GPIO_PIN_SET); // Allumer la LED
                }
                else
                {
                    HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, GPIO_PIN_RESET); // Éteindre la LED
                }
            }
        }

        // Exécution de la deuxième boucle while seulement si buttonPressed == 1
        if (buttonPressed == 1)
        {
            if (T_enc >= 2)
            {
                T_enc = 0;
                encoder_value_right_minus200 = encoder_value_right;
                encoder_value_left_minus200 = encoder_value_left;
                encoder_value_left = (TIM4->CNT);
                encoder_value_right = (TIM3->CNT);
                VD = calculCommande(vitesse, 0);
                VG = calculCommande(vitesse, 1);
                handleEvent(currentEvent);
                executeStateActions();
                test = encoder_value_left - encoder_value_left_minus200;
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_8B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
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
    htim2.Init.Prescaler = 4-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 40000-1;
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
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
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

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
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
    htim6.Init.Prescaler = 122-1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535-1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : Alert_batt_Pin */
    GPIO_InitStruct.Pin = Alert_batt_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Alert_batt_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DIR2_Pin */
    GPIO_InitStruct.Pin = DIR2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIR2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DIR1_Pin */
    GPIO_InitStruct.Pin = DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIR1_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        T_batt++;
        T_enc++;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    ADC_on = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if(rxData == 'F'){
            currentEvent = EVENT_AV;
            somme_erreursL = 0;
            somme_erreursR = 0;
        }
        else if(rxData == 'B'){
            currentEvent = EVENT_R;
            somme_erreursL = 0;
            somme_erreursR = 0;
        }
        else if(rxData == 'L'){
            currentEvent = EVENT_G;
            somme_erreursL = 0;
            somme_erreursR = 0;
        }
        else if(rxData == 'R'){
            currentEvent = EVENT_D;
            somme_erreursL = 0;
            somme_erreursR = 0;
        }
        else if(rxData == 'X'){
            currentEvent = EVENT_END;
            somme_erreursL = 0;
            somme_erreursR = 0;
        }
        else
            currentEvent = EVENT_NEUTRAL;
        HAL_UART_Receive_IT(&huart3, &rxData, sizeof(rxData));
    }
}

void handleEvent(Event_t event) {
    switch (currentState) {
    case STATE_NEUTRAL:
        if (event == EVENT_AV) {
            currentState = STATE_AV1;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R1;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D1;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G1;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_AV1:
        if (event == EVENT_AV) {
            currentState = STATE_AV2;
        }
        else if (event == EVENT_R) {
            currentState = STATE_NEUTRAL;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D1;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G1;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_AV2:
        if (event == EVENT_AV) {
            currentState = STATE_AV3;
        }
        else if (event == EVENT_R) {
            currentState = STATE_AV1;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D2;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G2;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_AV3:
        if (event == EVENT_AV) {
            currentState = STATE_AV3;
        }
        else if (event == EVENT_R) {
            currentState = STATE_AV2;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D3;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G3;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_R1:
        if (event == EVENT_AV) {
            currentState = STATE_NEUTRAL;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R2;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D1;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G1;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_R2:
        if (event == EVENT_AV) {
            currentState = STATE_R1;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R3;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D2;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G2;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_R3:
        if (event == EVENT_AV) {
            currentState = STATE_R2;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R3;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D3;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G3;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_D1:
        if (event == EVENT_AV) {
            currentState = STATE_AV1;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R1;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D2;
        }
        else if (event == EVENT_G) {
            currentState = STATE_NEUTRAL;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_D2:
        if (event == EVENT_AV) {
            currentState = STATE_AV2;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R2;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D3;
        }
        else if (event == EVENT_G) {
            currentState = STATE_D1;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_D3:
        if (event == EVENT_AV) {
            currentState = STATE_AV3;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R3;
        }
        else if (event == EVENT_D) {
            currentState = STATE_D3;
        }
        else if (event == EVENT_G) {
            currentState = STATE_D2;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_G1:
        if (event == EVENT_AV) {
            currentState = STATE_AV1;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R1;
        }
        else if (event == EVENT_D) {
            currentState = STATE_NEUTRAL;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G2;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_G2:
        if (event == EVENT_AV) {
            currentState = STATE_AV2;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R2;
        }
        else if (event == EVENT_D) {
            currentState = STATE_G1;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G3;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    case STATE_G3:
        if (event == EVENT_AV) {
            currentState = STATE_AV3;
        }
        else if (event == EVENT_R) {
            currentState = STATE_R3;
        }
        else if (event == EVENT_D) {
            currentState = STATE_G2;
        }
        else if (event == EVENT_G) {
            currentState = STATE_G3;
        }
        else if (event == EVENT_END) {
            currentState = STATE_END;
        }
        else if (event == EVENT_NEUTRAL){
            currentState = currentState;
        }
        break;

    default:
        currentState = STATE_NEUTRAL;
        break;
    }
}

uint32_t calculCommande(int mode, int cote)
{
    int32_t epsilon = 0;
    int32_t commande = 0;
    int32_t targetTop = 0;
    int32_t encoder_value_current = 0;
    int32_t encoder_value_previous = 0;
    float CKp_D = 0.0;
    float CKi_D = 0.0;
    float CKp_G = 0.0;
    float CKi_G = 0.0;

    // Sélectionner les cibles en fonction du mode
    if (mode == 1)
    {
        targetTop = target10_top;
        CKp_D = 190;
        CKi_D = 0;
        CKp_G = 220;
        CKi_G = 0;
    }
    else if (mode == 2)
    {
        targetTop = target20_top;
        CKp_D = 170;
        CKi_D = 0;
        CKp_G = 210;
        CKi_G = 0;
    }
    else if (mode == 3)
    {
        targetTop = target30_top;
        CKp_D = 170;
        CKi_D = 0;
        CKp_G = 210;
        CKi_G = 0;
    }
    if (mode == 4)
    {
        targetTop = target10_top;
        CKp_D = 350;
        CKi_D = 0;
        CKp_G = 350;
        CKi_G = 0;
    }
    else if (mode == 5)
    {
        targetTop = target20_top;
        CKp_D = 400;
        CKi_D = 0;
        CKp_G = 400;
        CKi_G = 0;
    }
    else if (mode == 6)
    {
        targetTop = target30_top;
        CKp_D = 600;
        CKi_D = 0;
        CKp_G = 600;
        CKi_G = 0;
    }

    // Sélectionner les valeurs du codeur et les coefficients en fonction du côté
    if (cote == 0)
    {
        encoder_value_current = encoder_value_right;
        encoder_value_previous = encoder_value_right_minus200;
        epsilon = targetTop - encoder_value_previous + encoder_value_current;
        somme_erreursR += epsilon;
        commande = CKp_D * epsilon + CKi_D * somme_erreursR;
    }
    else if (cote == 1)
    {
        encoder_value_current = encoder_value_left;
        encoder_value_previous = encoder_value_left_minus200;
        epsilon = targetTop - encoder_value_previous + encoder_value_current;
        somme_erreursL += epsilon;
        commande = CKp_G * epsilon + CKi_G * somme_erreursL;
    }

    if (commande > 40000)
        commande = 40000;
    else if(mode==0)
        commande = 0;
    return commande;
}

void executeStateActions(void) {
    switch (currentState) {
    case STATE_NEUTRAL:
        vitesse = 0;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    case STATE_AV1:
        vitesse = 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    case STATE_AV2:
        vitesse = 2;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    case STATE_AV3:
        vitesse = 3;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    case STATE_R1:
        vitesse = 4;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        break;

    case STATE_R2:
        vitesse = 5;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        break;

    case STATE_R3:
        vitesse = 6;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        break;

    case STATE_D1:
        vitesse = 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        break;

    case STATE_D2:
        vitesse = 2;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        break;

    case STATE_D3:
        vitesse = 3;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        break;

    case STATE_G1:
        vitesse = 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    case STATE_G2:
        vitesse = 2;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    case STATE_G3:
        vitesse = 3;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,VG);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,VD);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
        break;

    default:
        vitesse = 0;
        currentState = STATE_NEUTRAL;
        break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ENC1A_Pin)
    {
        current_capture = __HAL_TIM_GET_COUNTER(&htim6);
        uint32_t elapsed_time = current_capture - last_capture;
        last_capture = current_capture;

        if (elapsed_time > 0)
        {
            speed = (DISTANCE_PER_PULSE / elapsed_time) * 1000;
        }

        pulse_count++;
    }
    if(GPIO_Pin == GPIO_PIN_13)  // Vérifiez si l'interruption provient de PC13
    {
        buttonPressed = !buttonPressed;  // Inverser la valeur du flag
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
