/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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
#include "cordic.h"

/* USER CODE BEGIN 0 */

/* Define PI */
/* Define Q31 */
#define PI 3.1415926536f
#define Q31 0x80000000
/* Define Q31 to float unit in RADIAN = Q31/PI */
#define RADIAN_Q31_f 683565275.6f
/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief Calculate sin/cos value in float unit
 * @param angle in, sin/cos value out
 * @retval None
 * @补充说明：计算sin和cos的值，如输入的值为30，40，转换成的值为sin30和cos30
 */

void Calculate_Float_Sin_Cos(float angle,float *sin, float *cos)
{
  /* Q31,two write, two read, sine calculate, 6 precision */
  CORDIC->CSR = 0x00180061;
  /* Write data into WDATA */
  if(angle > PI)
  {
    angle = angle - 2*PI;
  }
  else if(angle < -PI)
  {
    angle = angle + 2*PI;
  }
  float wt = 2*PI;
  angle = angle - ((int32_t)(angle / wt) * wt);
  //angle = Fmod(angle,2*PI);
  CORDIC->WDATA = (int32_t )(angle * RADIAN_Q31_f);
  /* Modulus is m=1 */
  CORDIC->WDATA = 0x7FFFFFFF;
  /* Get sin value in float */
  *sin = ((int32_t)CORDIC->RDATA)*1.0f/Q31;
  /* Get cos value in float */
  *cos = ((int32_t)CORDIC->RDATA)*1.0f/Q31;
}
/* USER CODE END 1 */
