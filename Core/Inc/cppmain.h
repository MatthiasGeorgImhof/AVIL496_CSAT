/*
 * cppmain.hpp
 */

#ifndef INC_CPPMAIN_H_
#define INC_CPPMAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32l4xx_hal.h"

struct HAL_Handles {
	CAN_HandleTypeDef *hcan1;
	CAN_HandleTypeDef *hcan2;

	I2C_HandleTypeDef *hi2c1;
	I2C_HandleTypeDef *hi2c2;
	I2C_HandleTypeDef *hi2c4;

	RTC_HandleTypeDef *hrtc;

	SPI_HandleTypeDef *hspi1;
	SPI_HandleTypeDef *hspi2;
	SPI_HandleTypeDef *hspi3;

	UART_HandleTypeDef *huart1;
	UART_HandleTypeDef *huart2;
};

void cppmain(struct HAL_Handles handles);

#ifdef __cplusplus
}
#endif

#endif /* INC_CPPMAIN_H_ */
