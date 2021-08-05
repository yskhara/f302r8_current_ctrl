/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "arm_math.h"
#include "adc.h"
#include "dac.h"
#include <stdio.h>

#include "svpwm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void
SystemClock_Config (void);
static void
MX_GPIO_Init (void);
static void
MX_USART2_UART_Init (void);
static void
ConfigureTIM1 (void);
static void
ConfigureTIM2 (void);
static void
ConfigureTIM6 (void);
static void
ConfigureDMA (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t Tick_ms = 0;
void Delay_ms (uint32_t delay_ms)
{
    Tick_ms = 0;
    while (Tick_ms < delay_ms)
        ;
}

float sinePeriodFiltered = 0;
uint32_t hallCounter = 0;
uint8_t hallCounterChanged = 0;
//float theta_hall = 0.0f;
float32_t Omega = 0.0f;  // in rad/s
//float32_t gfOmegaDt = 0;
float32_t Theta = 0;		// in range [-pi:pi]
float32_t ThetaHall = 0;
uint8_t hall_pattern = 0x00;
//float theta_at_edge[] = { 0, 2.0 * M_PI / 3.0, M_PI / 3.0, -2.0 * M_PI / 3.0, -M_PI / 3.0, M_PI };
float32_t ThetaAtEdge[] =
    { 0.0f, 0.0f, 2.0f * M_PI / 3.0f, M_PI / 3.0f, -2.0f * M_PI / 3.0f, -M_PI / 3.0f, -M_PI };
uint8_t gaubHallPatternSector[] =
    { 0, 0, 2, 1, 4, 5, 3 };
int8_t gabHallDirectionTable[6][6] =
    {
        { 0, 1, 0, 0, 0, -1 },
        { -1, 0, 1, 0, 0, 0 },
        { 0, -1, 0, 1, 0, 0 },
        { 0, 0, -1, 0, 1, 0 },
        { 0, 0, 0, -1, 0, 1 },
        { 1, 0, 0, 0, -1, 0 }, };
uint8_t gubHallPatternPrev = 0x00;
uint8_t gubHallPatternCurrent = 0x00;
float k_pll = 0.1;

float adv = M_PI;

volatile uint16_t aADC1ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
volatile uint8_t ubAdc1DmaTransferCplt = 0;

float32_t PhaseCurrent[2] =
    { 0, 0 };
AB CurrentAB =
    { 0, 0 };
DQ CurrentDQ =
    { 0, 0 };

uint16_t uhVDDA = 0;
uint16_t aAdcOffset[3] =
    { 0, 0, 0 };

char text[256];
void prints (const char *const str)
{
    auto ptr = str;
    LL_USART_ClearFlag_TC (USART2);
    while (*ptr != '\0')
    {
        while (LL_USART_IsActiveFlag_TXE (USART2) == 0)
            ;
        LL_USART_TransmitData8 (USART2, *ptr++);
    }
}

void
Adc1DmaTransferComplete_Callback (void);
void
Adc1InjConvComplete_Callback (void);
void
Tim1UpdateCallback (void);
void
Tim2UpdateCallback (void);
void
Tim6UpdateCallback (void);

void Tim2CC1Callback (void)
{
    hallCounter = TIM2->CCR1;
    hallCounterChanged = 1;
    gubHallPatternPrev = gubHallPatternCurrent;
    hall_pattern = ((GPIOA->IDR & GPIO_IDR_15) >> 13) | ((GPIOB->IDR & GPIO_IDR_3) >> 2) | ((GPIOB->IDR & GPIO_IDR_10) >> 10);
    gubHallPatternCurrent = gaubHallPatternSector[hall_pattern];
    Omega = (72e6 * M_PI / 3.0f) / hallCounter;
    //float32_t fOmegaDt = 1e-3 / 3.0f * 72e6 / hallCounter;

    //gfOmega *= gabHallDirectionTable[gubHallPatternPrev][gubHallPatternCurrent];
    Omega *= gabHallDirectionTable[gubHallPatternPrev][gubHallPatternCurrent];

    //arm_float_to_q31(&fOmegaDt, &gqwOmegaDt, 1);
    ThetaHall = ThetaAtEdge[hall_pattern];
    //gqwTheta = gqwThetaHall;
}

void Tim6UpdateCallback (void)
{
    return;
    GPIOB->BSRR = GPIO_BSRR_BS_2;
    //q31_t qwOmegaDt = 0;
    //float fOmegaDt = omega / M_PI * 1e-3;
    //arm_float_to_q31(&fOmegaDt, &qwOmegaDt, 1);
    //gqwTheta += gqwOmegaDt;
    //LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (gqwTheta >> 20) ^ 0x800);
    //q31_t qwThetaRT = gqwThetaHall + Q31(gfOmega * TIM2->CNT);
    Theta += 0.03f * M_PI;

    if (Theta < -M_PI)
        Theta += 2 * M_PI;
    else if (Theta > M_PI)
        Theta -= 2 * M_PI;

    float32_t ThetaRT = Theta;
    //LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (qwThetaRT >> 20) ^ 0x800);

    //float32_t qwAdv = 0.7 * M_PI;
    //q31_t qwSines[3];
    //q31_t dummy;

    //arm_sin_cos_q31(qwThetaRT + qwAdv, &qwSines[0], &dummy);
    //arm_sin_cos_q31(qwThetaRT + Q31(2.0f / 3.0f) + qwAdv, &qwSines[1], &dummy);
    //arm_sin_cos_q31(qwThetaRT - Q31(2.0f / 3.0f) + qwAdv, &qwSines[2], &dummy);

    UVW uvw;
    AB ab;
    ab.A = arm_cos_f32 (Theta);
    ab.B = arm_sin_f32 (Theta);
    SVPWM_Calc (&ab, 5.0f, &uvw);

    //LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (qwSines[0] >> 20) ^ 0x800);
    LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, uvw.V * 16000.0f);

    //if (GPIOC->IDR & GPIO_IDR_13 != 0) {
//			TIM1->CCR1 = 1800.0 * sin(theta + adv);
//			TIM1->CCR2 = 1800.0 * sin(theta + 2.0 * M_PI / 3.0 + adv);
//			TIM1->CCR3 = 1800.0 * sin(theta - 2.0 * M_PI / 3.0 + adv);
    //TIM1->CCR1 = 500.0 + (int32_t)qwSines[0] * (100.0/0x80000000);
    //TIM1->CCR2 = 500.0 + (int32_t)qwSines[1] * (100.0/0x80000000);
    //TIM1->CCR3 = 500.0 + (int32_t)qwSines[2] * (100.0/0x80000000);
    TIM1->CCR1 = uvw.U * 1000.0f;
    TIM1->CCR2 = uvw.V * 1000.0f;
    TIM1->CCR3 = uvw.W * 1000.0f;
    //}
    GPIOB->BSRR = GPIO_BSRR_BR_2;
}

void Adc1DmaTransferComplete_Callback (void)
{
    ubAdc1DmaTransferCplt = 1;
}

void Adc1InjConvComplete_Callback (void)
{
//	qwPhaseCurrent[0] = (uint32_t)(ADC1->JDR1) << 16;
//	qwPhaseCurrent[1] = ADC1->JDR2 << 16;
//	qwPhaseCurrent[2] = ADC1->JDR3 << 16;

    PhaseCurrent[0] = (float32_t)uhVDDA * (int16_t)ADC1->JDR1 / 4096.0f;
    PhaseCurrent[1] = (float32_t)uhVDDA * (int16_t)ADC1->JDR2 / 4096.0f;
    //PhaseCurrent[1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(uhVDDA, (int16_t)ADC1->JDR2, LL_ADC_RESOLUTION_12B);
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, PhaseCurrent[0] + 2048);


    Theta += 0.008f * M_PI;

    if (Theta < -M_PI)
        Theta += 2 * M_PI;
    else if (Theta > M_PI)
        Theta -= 2 * M_PI;
    //q31_t qwThetaRT = gqwThetaHall + Q31(gfOmega * TIM2->CNT);

    //q31_t qwSin, qwCos;
    //arm_sin_cos_f32(gqwTheta, &qwSin, &qwCos);
    //arm_clarke_f32 (PhaseCurrent[0], PhaseCurrent[1], &(CurrentAB.A), &(CurrentAB.B));
    //arm_park_f32 (CurrentAB.A, CurrentAB.B, &(CurrentDQ.D), &(CurrentDQ.Q), arm_sin_f32 (Theta), arm_cos_f32 (Theta));

    UVW uvw;
    AB ab;
    ab.A = arm_cos_f32 (Theta);
    ab.B = arm_sin_f32 (Theta);
    SVPWM_Calc (&ab, 5.0f, &uvw);

    //LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, uvw.V * 16000.0f);

    TIM1->CCR1 = uvw.U * 1000.0f;
    TIM1->CCR2 = uvw.V * 1000.0f;
    TIM1->CCR3 = uvw.W * 1000.0f;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main (void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_2);

    /* System interrupt init*/

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config ();

    LL_SYSTICK_EnableIT ();
    NVIC_SetPriority (SysTick_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 0, 0));
    NVIC_EnableIRQ (SysTick_IRQn);

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init ();
    ConfigureDMA ();
    MX_USART2_UART_Init ();
    ConfigureADC1 ();

    ConfigureTIM1 ();
    ConfigureTIM2 ();
    ConfigureTIM6 ();

    ConfigureDAC ();
    ActivateDAC ();
    /* USER CODE BEGIN 2 */

    /* DMA1_Channel1_IRQn interrupt configuration */
    NVIC_SetPriority (DMA1_Channel1_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 1, 0));
    NVIC_EnableIRQ (DMA1_Channel1_IRQn);

    prints ("Starting Self-Cal...");

    ADC_SelfCal ();

    sprintf (text, "done. VDDA: %04d; offsets: %04d, %04d, %04d.\r\n", uhVDDA, (short) aAdcOffset[0], (short) aAdcOffset[1], (short) aAdcOffset[2]);
    prints (text);

    // ADC interrupt @fsw
    NVIC_SetPriority (ADC1_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 0, 1));
    NVIC_EnableIRQ (ADC1_IRQn);

    // TIM1 interrupt not needed
    //HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    LL_TIM_EnableCounter (TIM2);
    // Hall input interrupt
    NVIC_SetPriority (TIM2_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 1, 0));
    NVIC_EnableIRQ (TIM2_IRQn);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    LL_TIM_EnableCounter (TIM1);
    LL_TIM_EnableAllOutputs (TIM1);
    GPIOC->BSRR = GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;

    //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    //TIM1->CCR4 = 995;

    // @1kHz
    NVIC_SetPriority (TIM6_DAC_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 1, 0));
    NVIC_EnableIRQ (TIM6_DAC_IRQn);
    LL_TIM_EnableCounter (TIM6);

    ConfigureADCNormal ();
    //LL_ADC_REG_StartConversion(ADC1);
    LL_ADC_INJ_StartConversion (ADC1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        sprintf (text, "%07.4f, %07.3f, %07.3f\r\n", Omega, PhaseCurrent[0], PhaseCurrent[1]);
        prints (text);
        Delay_ms (1);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config (void)
{
    LL_FLASH_SetLatency (LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency () != LL_FLASH_LATENCY_2)
    {
    }
    LL_RCC_HSE_EnableBypass ();
    LL_RCC_HSE_Enable ();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady () != 1)
    {

    }
    LL_RCC_PLL_ConfigDomain_SYS (LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable ();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady () != 1)
    {

    }
    LL_RCC_SetAHBPrescaler (LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler (LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler (LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource (LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource () != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {

    }
    LL_Init1msTick (72000000);
    LL_SetSystemCoreClock (72000000);
    LL_RCC_SetTIMClockSource (LL_RCC_TIM1_CLKSOURCE_PCLK2);
    LL_RCC_SetADCClockSource (LL_RCC_ADC1_CLKSRC_PLL_DIV_1);
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void ConfigureTIM1 (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct =
        { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct =
        { 0 };
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct =
        { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct =
        { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_TIM1);

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
    TIM_InitStruct.Autoreload = 1000 - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init (TIM1, &TIM_InitStruct);
    LL_TIM_EnableARRPreload (TIM1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init (TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_EnablePreload (TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_DisableFast (TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init (TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_EnablePreload (TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_DisableFast (TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init (TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_EnablePreload (TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_DisableFast (TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init (TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_EnablePreload (TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_DisableFast (TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_DisableMasterSlaveMode (TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = 0;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init (TIM1, &TIM_BDTRInitStruct);

    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_SetCompareCH4(TIM1, LL_TIM_GetAutoReload(TIM1)-1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

    LL_TIM_SetTriggerOutput (TIM1, LL_TIM_TRGO_OC4REF);
    LL_TIM_SetTriggerOutput2 (TIM1, LL_TIM_TRGO2_OC4);

    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    /**TIM1 GPIO Configuration
     PA8   ------> TIM1_CH1
     PA9   ------> TIM1_CH2
     PA10   ------> TIM1_CH3
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init (GPIOA, &GPIO_InitStruct);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void ConfigureTIM2 (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct =
        { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct =
        { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct =
        { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_TIM2);

    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    /**TIM2 GPIO Configuration
     PB10   ------> TIM2_CH3
     PA15   ------> TIM2_CH1
     PB3   ------> TIM2_CH2
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xffffffff;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init (TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload (TIM2);
    //LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_IC_SetActiveInput (TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_TRC);
    LL_TIM_IC_SetPrescaler (TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter (TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity (TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_EnableXORCombination (TIM2);
    LL_TIM_SetTriggerInput (TIM2, LL_TIM_TS_TI1F_ED);
    LL_TIM_SetSlaveMode (TIM2, LL_TIM_SLAVEMODE_RESET);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init (TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast (TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_SetTriggerOutput (TIM2, LL_TIM_TRGO_OC2REF);
    LL_TIM_DisableMasterSlaveMode (TIM2);

    LL_TIM_CC_EnableChannel (TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1 (TIM2);
}

static void ConfigureTIM6 (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct =
        { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_TIM6);

    TIM_InitStruct.Prescaler = 72 - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 100 - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init (TIM6, &TIM_InitStruct);
    LL_TIM_EnableARRPreload (TIM6);

    LL_TIM_SetTriggerOutput (TIM6, LL_TIM_TRGO_UPDATE);
    LL_TIM_DisableMasterSlaveMode (TIM6);

    LL_TIM_EnableIT_UPDATE (TIM6);
}

void ConfigureDMA (void)
{
    /* Init with LL driver */
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_DMA1);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init (void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    LL_USART_InitTypeDef USART_InitStruct =
        { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct =
        { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_USART2);

    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    /**USART2 GPIO Configuration
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
     */
    GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    USART_InitStruct.BaudRate = 921600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init (USART2, &USART_InitStruct);
    LL_USART_DisableIT_CTS (USART2);
    LL_USART_ConfigAsyncMode (USART2);
    LL_USART_Enable (USART2);
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init (void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct =
        { 0 };
    LL_GPIO_InitTypeDef GPIO_InitStruct =
        { 0 };

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOF);
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_2 | LD2_Pin);

    /**/
    LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12);

    /**/
    LL_SYSCFG_SetEXTISource (LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

    /**/
    LL_GPIO_SetPinPull (B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinMode (B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
    EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init (&EXTI_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LD2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init (GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler (void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq ();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
