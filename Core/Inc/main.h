/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_ACS_X_PH_Pin GPIO_PIN_2
#define GPIO_ACS_X_PH_GPIO_Port GPIOE
#define GPIO_ACS_Y_SLEEP_Pin GPIO_PIN_3
#define GPIO_ACS_Y_SLEEP_GPIO_Port GPIOE
#define GPIO_ACS_Y_PH_Pin GPIO_PIN_4
#define GPIO_ACS_Y_PH_GPIO_Port GPIOE
#define GPIO_ACS_Z_SLEEP_Pin GPIO_PIN_5
#define GPIO_ACS_Z_SLEEP_GPIO_Port GPIOE
#define GPIO_ACS_Z_PH_Pin GPIO_PIN_6
#define GPIO_ACS_Z_PH_GPIO_Port GPIOE
#define GPIO_WATCHDOG_WAKE_EXTI1_Pin GPIO_PIN_1
#define GPIO_WATCHDOG_WAKE_EXTI1_GPIO_Port GPIOF
#define GPIO_WATCHDOG_WAKE_EXTI1_EXTI_IRQn EXTI1_IRQn
#define TIM15_CHi1_ACS_Z_Pin GPIO_PIN_9
#define TIM15_CHi1_ACS_Z_GPIO_Port GPIOF
#define GPIO_10MHZ_OE_Pin GPIO_PIN_10
#define GPIO_10MHZ_OE_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOF
#define LED5_Pin GPIO_PIN_12
#define LED5_GPIO_Port GPIOF
#define MAG_INT_EXTI0_Pin GPIO_PIN_0
#define MAG_INT_EXTI0_GPIO_Port GPIOG
#define MAG_INT_EXTI0_EXTI_IRQn EXTI0_IRQn
#define GPIO_RST_SUN_Pin GPIO_PIN_9
#define GPIO_RST_SUN_GPIO_Port GPIOE
#define GPIO_SPI1_MAG_CS_Pin GPIO_PIN_12
#define GPIO_SPI1_MAG_CS_GPIO_Port GPIOE
#define GPIO_GPS_FIX_EXTI8_Pin GPIO_PIN_8
#define GPIO_GPS_FIX_EXTI8_GPIO_Port GPIOD
#define GPIO_GPS_FIX_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_GPS_PPS_EXTI9_Pin GPIO_PIN_9
#define GPIO_GPS_PPS_EXTI9_GPIO_Port GPIOD
#define GPIO_GPS_PPS_EXTI9_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_GPS_RST_Pin GPIO_PIN_10
#define GPIO_GPS_RST_GPIO_Port GPIOD
#define GPIO_5VEN_Pin GPIO_PIN_11
#define GPIO_5VEN_GPIO_Port GPIOD
#define GPIO_SPI2_GYRO_CS_Pin GPIO_PIN_12
#define GPIO_SPI2_GYRO_CS_GPIO_Port GPIOD
#define GYRO_INT2_EXTI2_Pin GPIO_PIN_2
#define GYRO_INT2_EXTI2_GPIO_Port GPIOG
#define GYRO_INT2_EXTI2_EXTI_IRQn EXTI2_IRQn
#define GYRO_INT1_EXTI3_Pin GPIO_PIN_3
#define GYRO_INT1_EXTI3_GPIO_Port GPIOG
#define GYRO_INT1_EXTI3_EXTI_IRQn EXTI3_IRQn
#define GPIO_INTA_EXTI4_Pin GPIO_PIN_4
#define GPIO_INTA_EXTI4_GPIO_Port GPIOG
#define GPIO_INTA_EXTI4_EXTI_IRQn EXTI4_IRQn
#define GPIO_POWER_RST_Pin GPIO_PIN_6
#define GPIO_POWER_RST_GPIO_Port GPIOC
#define GPIO_POWER_INT_EXTI7_Pin GPIO_PIN_7
#define GPIO_POWER_INT_EXTI7_GPIO_Port GPIOC
#define GPIO_POWER_INT_EXTI7_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_EPS_RST_Pin GPIO_PIN_8
#define GPIO_EPS_RST_GPIO_Port GPIOC
#define GPIO_SUN_INT_EXTI10_Pin GPIO_PIN_10
#define GPIO_SUN_INT_EXTI10_GPIO_Port GPIOC
#define GPIO_SUN_INT_EXTI10_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_ACS_INT_EXTI11_Pin GPIO_PIN_11
#define GPIO_ACS_INT_EXTI11_GPIO_Port GPIOC
#define GPIO_ACS_INT_EXTI11_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_ATTENTION_EXTI12_Pin GPIO_PIN_12
#define GPIO_ATTENTION_EXTI12_GPIO_Port GPIOC
#define GPIO_ATTENTION_EXTI12_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_CAN1_SHDN_Pin GPIO_PIN_2
#define GPIO_CAN1_SHDN_GPIO_Port GPIOD
#define GPIO_CAN1_STB_Pin GPIO_PIN_3
#define GPIO_CAN1_STB_GPIO_Port GPIOD
#define GPIO_SPI3_MRAM_CS_Pin GPIO_PIN_12
#define GPIO_SPI3_MRAM_CS_GPIO_Port GPIOG
#define GPIO_CAN2_STB_Pin GPIO_PIN_3
#define GPIO_CAN2_STB_GPIO_Port GPIOB
#define GPIO_CAN2_SHDN_Pin GPIO_PIN_4
#define GPIO_CAN2_SHDN_GPIO_Port GPIOB
#define TIM16_CH1_ACS_X_Pin GPIO_PIN_8
#define TIM16_CH1_ACS_X_GPIO_Port GPIOB
#define TIM17_CH1_ACS_Y_Pin GPIO_PIN_9
#define TIM17_CH1_ACS_Y_GPIO_Port GPIOB
#define GPIO_WATCHDOG_DONE_Pin GPIO_PIN_0
#define GPIO_WATCHDOG_DONE_GPIO_Port GPIOE
#define GPIO_ACS_X_SLEEP_Pin GPIO_PIN_1
#define GPIO_ACS_X_SLEEP_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
