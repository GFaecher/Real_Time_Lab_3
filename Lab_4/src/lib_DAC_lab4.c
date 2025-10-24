// The standard lib_DAC.c, but missing the HAL DAC driver (which they will
// write themselves).

#include "lib_ee152.h"		// for delay_ms().
#include "stm32l432xx.h"
#include <stdint.h>
//#define USE_HAL

////////////////////////////////////////////////////
// DAC
////////////////////////////////////////////////////

#ifndef USE_HAL

////////////////////////////////////////////////////
// DAC Initialization
////////////////////////////////////////////////////
void DAC1_Init(void){
    // Set GPIO pin PA4 (the DAC1 output, Nano A3) to be an analog output.
    //	... first enable the clock of GPIO Port A so we can write CSRs.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    //	... configure PA4 (DAC1_OUT1) as analog
    GPIOA->MODER |=   3U<<(2*4);  // 2 bits of mode per pin; 11b = Analog
    // GPIO port pup/pulldown register. It has 2 bits per pin, and we set 
    // 00=>No pull-up or pull-down (after all, it's an analog output).
    GPIOA->PUPDR &= ~(3U<<(2*4));

    // Turn on the DAC clocks, set DAC1 to drive PA4 via the DAC
    // buffer, and use software triggering.

    // APB1 Peripheral clock Enable Register 1 (APB1ENR1)
    // It looks like this one bit enables the clock for both DACs.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // DAC mode control register (DAC_MCR). Each of the two DACs has three bits
    // of mode. We set 000, or DAC1 driving its external pin (PA4) via a buffer.
    // The buffer allows higher drive current.
    // This value of 000 also turns off sample-and-hold mode.
    DAC->MCR &= ~DAC_MCR_MODE1;

    // DAC channel1 trigger enable. Disable it, so that DAC1 cannot trigger --
    // (which means that writes to the DAC data reg take effect immediately).
    // This also means that writing the trigger-select field TSEL1 is moot.
    DAC->CR &=  ~DAC_CR_TEN1;       // Disable the trigger.

    // Same register again: enable DAC #1.
    DAC->CR |=  DAC_CR_EN1;       // Enable DAC Channel 2

    delay(1);	// ms.
}

void DAC2_Init(void){
    // Set GPIO pin PA5 (the DAC2 output, Nano A4) to be an analog output.
    //	... first enable the clock of GPIO Port A so we can write CSRs.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    //	... configure PA5 (DAC1_OUT2) as analog
    GPIOA->MODER |=   3U<<(2*5);  // 2 bits of mode per pin; 11b = Analog
    // GPIO port pup/pulldown register. It has 2 bits per pin, and we set 
    // 00=>No pull-up or pull-down (after all, it's an analog output).
    GPIOA->PUPDR &= ~(3U<<(2*5));

    // Turn on the DAC clocks, set DAC2 to drive PA5 via the DAC
    // buffer, and use software triggering.

    // APB1 Peripheral clock Enable Register 1 (APB1ENR1)
    // It looks like this one bit enables the clock for both DACs.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // DAC mode control register (DAC_MCR). Each of the two DACs has three bits
    // of mode. We set 000, or DAC2 driving its external pin (PA5) via a buffer.
    // The buffer allows higher drive current.
    // This value of 000 also turns off sample-and-hold mode.
    DAC->MCR &= ~DAC_MCR_MODE2;

    // DAC channel2 trigger enable. Disable it, so that DAC2 cannot trigger --
    // (which means that writes to the DAC data reg take effect immediately).
    // This also means that writing the trigger-select field TSEL2 is moot.
    DAC->CR &=  ~DAC_CR_TEN2;       // Trigger enable 

    // Same register again: enable DAC #2.
    DAC->CR |=  DAC_CR_EN2;       // Enable DAC Channel 2

    delay(1);	// ms.
}

// Write 12-bit unsigned data to DAC 1, which drives PA3.
void DAC1_write (uint32_t data) {
    // 12-bit right-aligned holding register.
    DAC->DHR12R1 = data;
}

// Write 12-bit unsigned data to DAC 2, which drives pin PA5.
void DAC2_write (uint32_t data) {
    DAC->DHR12R2 = data;
}

// This is the Arduino API.
// Most Arduino boards don't have a DAC; so on those boards, this function
// actually does a PWM on a digital GPIO pin. But it's true analog on the few
// Arduinos that have a DAC, and that's what we do too. Also, it defaults to
// 8-bit writes; a few Arduinos support AnalogWriteResolution(12 bits). We
// could do that easily, but I haven't bothered.
// - 'Pin' can only be A3 (PA4, for DAC 1) or A4 (PA5, for DAC 2).
// - 'Value' is in [0,255] to write in [0,3.3V].
void analogWrite (enum Pin pin, uint32_t value) {
    static bool DAC1_enabled=0, DAC2_enabled=0;
    if (pin==A3){		// DAC #1
        if (!DAC1_enabled)
            DAC1_Init();
        DAC1_enabled = 1;
        DAC->DHR8R1 = value;
    } else if (pin==A4){	// DAC #2
        if (!DAC2_enabled)
            DAC2_Init();
        DAC2_enabled = 1;
        DAC->DHR8R2 = value;
    } else
        error ("Called analogWrite() on a non-DAC pin");
}
#endif


////////////////////////////////////////////////////////////////////
// HAL version.
////////////////////////////////////////////////////////////////////

#ifdef USE_HAL

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_dac.h"

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

void Error_Handler(void);

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void) {

    __HAL_RCC_DAC1_CLK_ENABLE();

    if (HAL_Init()) Error_Handler();

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
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
    Error_Handler();
    }

    /** DAC channel OUT2 config
     */
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
    Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

// This is the Arduino API.
// Most Arduino boards don't have a DAC; so on those boards, this function
// actually does a PWM on a digital GPIO pin. But it's true analog on the few
// Arduinos that have a DAC, and that's what we do too. Also, it defaults to
// 8-bit writes; a few Arduinos support AnalogWriteResolution(12 bits). We
// could do that easily, but I haven't bothered.
// - 'Pin' can only be A3 (PA4, for DAC 1) or A4 (PA5, for DAC 2).
// - 'Value' is in [0,255] to write in [0,3.3V].
void analogWrite (enum Pin pin, uint32_t value) {
    static bool DAC1_enabled=0, DAC2_enabled=0;
    if (pin==A3) {		// DAC #1
        if (!DAC1_enabled) {
            MX_DAC1_Init();
            MX_GPIO_Init();
            HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
        }
        DAC1_enabled = 1;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, value);
    } else if (pin==A4) {	// DAC #2
        if (!DAC2_enabled) {
            MX_DAC1_Init();
            MX_GPIO_Init();
            HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
        }
        DAC2_enabled = 1;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, value);
    } else
        error ("Called analogWrite() on a non-DAC pin");
}

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

#endif