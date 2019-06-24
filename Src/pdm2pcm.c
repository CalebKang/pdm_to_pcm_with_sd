/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : pdm2pcm.c
  * Description        : This file provides code for the configuration
  *                      of the pdm2pcm instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pdm2pcm.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Global variables ---------------------------------------------------------*/
PDM_Filter_Handler_t PDM1_filter_handler;
PDM_Filter_Config_t PDM1_filter_config;

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/* PDM2PCM init function */
void MX_PDM2PCM_Init(void)
{
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

   /** 
  */
  PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_LSB;
  PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_LE;
  PDM1_filter_handler.high_pass_tap = 2122358088;
  PDM1_filter_handler.in_ptr_channels = 1;
  PDM1_filter_handler.out_ptr_channels = 2; 
  PDM_Filter_Init(&PDM1_filter_handler);

  PDM1_filter_config.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
  PDM1_filter_config.output_samples_number = 16;
  PDM1_filter_config.mic_gain = 24; 
  PDM_Filter_setConfig(&PDM1_filter_handler, &PDM1_filter_config);

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */

/*  process function */
#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))
uint8_t MX_PDM2PCM_Process(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];
  uint32_t index = 0;

  /* PDM Demux */
  for(index = 0; index<INTERNAL_BUFF_SIZE/2; index++)
  {
    AppPDM[index] = HTONS(PDMBuf[index]);
  }

  for(index = 0; index < DEFAULT_AUDIO_IN_CHANNEL_NBR; index++)
  {
    /* PDM to PCM filter */
    PDM_Filter((uint8_t*)&AppPDM[index], (uint16_t*)PCMBuf, &PDM1_filter_handler);
  }
  /* Duplicate samples since a single microphone in mounted on STM32F4-Discovery */
  for(index = 0; index < PCM_OUT_SIZE; index++)
  {
    PCMBuf[(index<<1)+1] = PCMBuf[index<<1];
  }

  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/* USER CODE END 4 */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
