/*
 * dac.cpp
 *
 *  Created on: Aug 4, 2021
 *      Author: yusaku
 */

#include "dac.h"

void ConfigureDAC(void)
{
	/*## Configuration of GPIO used by DAC channels ############################*/

	/* Enable GPIO Clock */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* Configure GPIO in analog mode to be used as DAC output */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);

	/*## Configuration of DAC ##################################################*/

	/* Enable DAC clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);

	LL_DAC_SetTriggerSource(DAC, LL_DAC_CHANNEL_1, LL_DAC_TRIG_SOFTWARE);
	LL_DAC_SetOutputBuffer(DAC, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_ENABLE);
	LL_DAC_SetWaveAutoGeneration(DAC, LL_DAC_CHANNEL_1, LL_DAC_WAVE_AUTO_GENERATION_NONE);
	LL_DAC_DisableTrigger(DAC, LL_DAC_CHANNEL_1);
	LL_DAC_DisableDMAReq(DAC, LL_DAC_CHANNEL_1);
	LL_DAC_ConvertData12RightAligned(DAC, LL_DAC_CHANNEL_1, 0x000);
	//LL_DAC_EnableIT_DMAUDR1(DAC);
}

void ActivateDAC(void)
{
  __IO uint32_t wait_loop_index = 0;

  /* Enable DAC channel */
  LL_DAC_Enable(DAC, LL_DAC_CHANNEL_1);

  /* Delay for DAC channel voltage settling time from DAC channel startup.    */
  /* Compute number of CPU cycles to wait for, from delay in us.              */
  /* Note: Variable divided by 2 to compensate partially                      */
  /*       CPU processing cycles (depends on compilation optimization).       */
  /* Note: If system core clock frequency is below 200kHz, wait time          */
  /*       is only a few CPU processing cycles.                               */
  wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
}


