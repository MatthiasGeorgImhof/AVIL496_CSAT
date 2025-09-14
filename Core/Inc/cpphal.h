/*
 * cpphal.h
 *
 *  Created on: Sep 14, 2025
 *      Author: mgi
 */

#ifndef INC_CPPHAL_H_
#define INC_CPPHAL_H_

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;

extern RTC_HandleTypeDef hrtc;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern GPIO_TypeDef GPIOA_object;
extern GPIO_TypeDef GPIOB_object;
extern GPIO_TypeDef GPIOC_object;
extern GPIO_TypeDef GPIOD_object;
extern GPIO_TypeDef GPIOE_object;
extern GPIO_TypeDef GPIOF_object;
extern GPIO_TypeDef GPIOG_object;

#endif /* INC_CPPHAL_H_ */
