/*
 * adc.cpp
 *
 *  Created on: Aug 4, 2021
 *      Author: yusaku
 */

#include "adc.h"

void ConfigureADC1 (void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct =
        { 0 };

    /* Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_ADC1);

    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB);
    /**ADC1 GPIO Configuration
     PC0   ------> ADC1_IN6
     PC1   ------> ADC1_IN7
     PC2   ------> ADC1_IN8
     PC3   ------> ADC1_IN9
     PA0   ------> ADC1_IN1
     PA1   ------> ADC1_IN2
     PA7   ------> ADC1_IN15
     PB0   ------> ADC1_IN11
     PB1   ------> ADC1_IN12
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init (GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */

    /* ADC1 Init */
    LL_DMA_SetDataTransferDirection (DMA1, LL_DMA_CHANNEL_1,
    LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel (DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

    LL_DMA_SetMode (DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode (DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode (DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize (DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize (DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    LL_ADC_SetResolution (ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetLowPowerMode (ADC1, LL_ADC_LP_MODE_NONE);

    LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
    LL_ADC_REG_SetSequencerLength (ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);
    LL_ADC_REG_SetSequencerDiscont (ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    LL_ADC_REG_SetContinuousMode (ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer (ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun (ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    LL_ADC_SetCommonClock (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV1);
    LL_ADC_REG_SetTriggerEdge (ADC1, LL_ADC_REG_TRIG_EXT_RISING);

    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator (ADC1);
    /* Delay for ADC internal voltage regulator stabilization. */
    /* Compute number of CPU cycles to wait for, from delay in us. */
    /* Note: Variable divided by 2 to compensate partially */
    /* CPU processing cycles (depends on compilation optimization). */
    /* Note: If system core clock frequency is below 200kHz, wait time */
    /* is only a few CPU processing cycles. */
    uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0)
    {
        wait_loop_index--;
    }
    /** Configure injected channel ADC_JSQR register
     */
    LL_ADC_INJ_ConfigQueueContext (ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
                                   LL_ADC_INJ_TRIG_EXT_RISING,
                                   LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                   LL_ADC_CHANNEL_7,
                                   LL_ADC_CHANNEL_6, 0x0000, 0x0000);
    LL_ADC_INJ_SetTrigAuto (ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);
    LL_ADC_INJ_SetSequencerDiscont (ADC1, LL_ADC_INJ_SEQ_DISCONT_DISABLE);
    /** Configure Injected Channel
     */
    LL_ADC_INJ_SetQueueMode (ADC1, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
    LL_ADC_INJ_SetTriggerEdge (ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    LL_ADC_SetCommonPathInternalCh (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff (ADC1, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);

    LL_ADC_DisableIT_JEOC(ADC1);
    LL_ADC_EnableIT_JEOS(ADC1);
}

uint16_t ADC_Self_Cal_Channel (uint32_t channel)
{
    const uint16_t ubPowNbConvSelfCal = 2;
    uint16_t ubConvIdx = 0;
    int uwAdcAccum = 0;

//	// configure channel
    LL_ADC_REG_SetSequencerLength (ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, channel);
    LL_ADC_SetChannelSamplingTime (ADC1, channel, LL_ADC_SAMPLINGTIME_601CYCLES_5);

    ubAdc1DmaTransferCplt = 0;
    for (ubConvIdx = 0; ubConvIdx < (1u << ubPowNbConvSelfCal); ubConvIdx++)
    {
        LL_ADC_REG_StartConversion (ADC1);
        while (ubAdc1DmaTransferCplt == 0)
            ;
        ubAdc1DmaTransferCplt = 0;
        uwAdcAccum += aADC1ConvertedData[0];
    }
    return uwAdcAccum >> ubPowNbConvSelfCal;
}

void ADC_SelfCal (void)
{
    // disable ADC first to configure
    if (LL_ADC_IsEnabled (ADC1))
    {
        LL_ADC_Disable (ADC1);
        while (LL_ADC_IsEnabled (ADC1) != 0)
            ;
    }

    // configure ADC
    LL_ADC_SetCommonClock (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);
    LL_ADC_SetCommonPathInternalCh (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT);			// this line requires ADC disabled
    //LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    //LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);

    LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode (ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer (ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun (ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    // configure DMA
    LL_DMA_ConfigAddresses (DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr (ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t) &aADC1ConvertedData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength (DMA1, LL_DMA_CHANNEL_1, 1);
    LL_DMA_EnableIT_TC (DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE (DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel (DMA1, LL_DMA_CHANNEL_1);

    LL_ADC_StartCalibration (ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing (ADC1) != 0)
        ; // wait for calibration completion

    LL_ADC_Enable (ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY (ADC1) == 0)
        ; // wait until adc is ready

    Delay_ms (100);

    // Calibrate VDDA
    uhVDDA = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Self_Cal_Channel(LL_ADC_CHANNEL_VREFINT), LL_ADC_RESOLUTION_12B);
    //uhVDDA = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, ADC_Self_Cal_Channel(LL_ADC_CHANNEL_VREFINT), LL_ADC_RESOLUTION_12B);
    //uhVDDA = ADC_Self_Cal_Channel(LL_ADC_CHANNEL_VREFINT);

    // Calibrate I_u
    aAdcOffset[0] = ADC_Self_Cal_Channel (LL_ADC_CHANNEL_1);
    // Calibrate I_v
    aAdcOffset[1] = ADC_Self_Cal_Channel (LL_ADC_CHANNEL_7);
    // Calibrate I_w
    aAdcOffset[2] = ADC_Self_Cal_Channel (LL_ADC_CHANNEL_6);
}

void ConfigureADCNormal (void)
{
    // disable ADC first to configure
    if (LL_ADC_IsEnabled (ADC1))
    {
        LL_ADC_Disable (ADC1);
        while (LL_ADC_IsEnabled (ADC1) != 0)
            ;
    }

    LL_ADC_SetCommonClock (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV1);
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetOffset (ADC1, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_1, aAdcOffset[0]);
    LL_ADC_SetOffset (ADC1, LL_ADC_OFFSET_2, LL_ADC_CHANNEL_7, aAdcOffset[1]);
    LL_ADC_SetOffset (ADC1, LL_ADC_OFFSET_3, LL_ADC_CHANNEL_6, aAdcOffset[2]);

    LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
    LL_ADC_REG_SetContinuousMode (ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer (ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun (ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    // configure DMA
    LL_DMA_DisableChannel (DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ConfigAddresses (DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr (ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t) &aADC1ConvertedData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength (DMA1, LL_DMA_CHANNEL_1, ADC_CONVERTED_DATA_BUFFER_SIZE);
    LL_DMA_EnableIT_TC (DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE (DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel (DMA1, LL_DMA_CHANNEL_1);

    // Enable ADC
    LL_ADC_StartCalibration (ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing (ADC1) != 0)
        ; // wait for calibration completion

    LL_ADC_Enable (ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY (ADC1) == 0)
        ; // wait until adc is ready

    LL_ADC_REG_SetSequencerLength (ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);

    LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_8);
    LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
    LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_12);

    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_181CYCLES_5);
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_181CYCLES_5);

    // for injected channels
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_181CYCLES_5);
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_19CYCLES_5);

    ubAdc1DmaTransferCplt = 0;
}

