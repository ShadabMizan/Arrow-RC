/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

#include <stdio.h>

#define TIM1_CLK_MHZ            170
#define TIM1_CLK_NS_PER_TICK      ((1/(TIM1_CLK_MHZ * 1e6)) * 1e9)

static uint8_t set_deadtime(uint32_t deadtime_ns);
static uint16_t set_period(uint32_t pwm_freq_hz, uint16_t psc);

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
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
  htim1.Init.Prescaler = 170 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_COMBINED_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  htim1.Init.Period = set_period(PWM_FREQUENCY_HZ, htim1.Init.Prescaler);
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sBreakDeadTimeConfig.DeadTime = set_deadtime(DEADTIME_NS);
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PC13     ------> TIM1_CH1N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA12     ------> TIM1_CH2N
    PB9     ------> TIM1_CH3N
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF12_TIM1_COMP1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t set_deadtime(uint32_t deadtime_ns) {
    // Bits 7:0 DTG[7:0]: Dead-time generator setup
    // This bitfield defines the duration of the dead-time inserted between the complementary
    // outputs. DT correspond to this duration.
    // DTG[7:5] = 0xx => DT = DTG[7:0]x tdtg with tdtg = tDTS.
    // DTG[7:5] = 10x => DT = (64+DTG[5:0])xtdtg with Tdtg = 2xtDTS.
    // DTG[7:5] = 110 => DT = (32+DTG[4:0])xtdtg with Tdtg = 8xtDTS.
    // DTG[7:5] = 111 => DT = (32+DTG[4:0])xtdtg with Tdtg = 16xtDTS.
    // Example if TDTS = 125 ns (8 MHz), dead-time possible values are:
    // 0 to 15875 ns by 125 ns steps,
    // 16 μs to 31750 ns by 250 ns steps,
    // 32 μs to 63 μs by 1 μs steps,
    // 64 μs to 126 μs by 2 μs steps

    uint8_t dtg = 0x00; // deadtime generator register
    uint32_t step;

    if (deadtime_ns <= (uint32_t)(0x7F * TIM1_CLK_NS_PER_TICK)) {
        step = TIM1_CLK_NS_PER_TICK;
        dtg = (uint8_t)(0x7F & (deadtime_ns + step/2)/step);
    } else if (deadtime_ns <= (uint32_t)((64 + 0x3F) * 2*TIM1_CLK_NS_PER_TICK)) {
        step = 2 * TIM1_CLK_NS_PER_TICK;
        dtg = (uint8_t)(0x80 | (0x3F & (deadtime_ns - 64 + step/2)/step));
    } else if (deadtime_ns <= (uint32_t)((32 + 0x1F) * 8*TIM1_CLK_NS_PER_TICK)) {
        step = 8 * TIM1_CLK_NS_PER_TICK;
        dtg = (uint8_t)(0xC0 | (0x1F & (deadtime_ns - 32 + step/2)/step));
    } else if (deadtime_ns <= (uint32_t)((32 + 0x1F) * 16*TIM1_CLK_NS_PER_TICK)) {
        step = 16 * TIM1_CLK_NS_PER_TICK;
        dtg = (uint8_t)(0xE0 | (0x1F & (deadtime_ns - 32 + step/2)/step));
    } else {
        printf("Deadtime too long: %lu\nDeadtime set to %lu\r\n", deadtime_ns, (uint32_t)((32 + 0x1F) * 16*TIM1_CLK_NS_PER_TICK));
        dtg = 0xFF;
    }

    return dtg;
}

// Auto Reload Register setting. This controls the PWM frequency
uint16_t set_period(uint32_t pwm_freq_hz, uint16_t psc) {

    // pwm_freq = (clk/psc+1)/(arr + 1)
    if (pwm_freq_hz == 0) return 0;

    uint32_t arr = ((TIM1_CLK_MHZ * 1e6)/(psc + 1))/pwm_freq_hz - 1;

    if (arr > 0xFFFF) {
        arr = 0xFFFF;
        printf("PWM Frequency (%lu) too high. Set to %lu\r\n", pwm_freq_hz, (TIM1_CLK_MHZ/(psc + 1))/(arr + 1));
    }

    return (0xFFFF & arr);
}
/* USER CODE END 1 */
