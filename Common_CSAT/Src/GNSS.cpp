/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *  Copyright 2020 SimpleMethod
 *
 *  adapted for C++ and streamlined
 *  Matthias G Imhof
 *  01.06.2025
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#include "GNSS.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

GNSS::GNSS(UART_HandleTypeDef *huart) : huart(huart) {}

// https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf

/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
uint8_t *GNSS::findHeader(UBXClass classID, UBXMessageID messageID)
{
	constexpr uint16_t SIZE = sizeof(uart_buffer) / 2;
	for (uint16_t var = 0; var <= SIZE; ++var)
	{
		if (uart_buffer[var] == 0xB5 && uart_buffer[var + 1] == 0x62)
		{
			uint16_t length = GNSSCore::getUShort(uart_buffer, static_cast<uint16_t>(var + 4));
			uint8_t cka_message = GNSSCore::getUByte(uart_buffer, static_cast<uint16_t>(var + length + UBLOX_HEADER_SIZE));
			uint8_t ckb_message = GNSSCore::getUByte(uart_buffer, static_cast<uint16_t>(var + length + UBLOX_HEADER_SIZE + 1));
			uint8_t cka, ckb;
			GNSSCore::checksum(length + 4U, &uart_buffer[var + 2], &cka, &ckb);
			if (cka != cka_message && ckb != ckb_message)
			{
				return nullptr;
			}
			if (uart_buffer[var + 2] == static_cast<uint8_t>(classID) && uart_buffer[var + 3] == static_cast<uint8_t>(messageID))
			{
				return uart_buffer + var + UBLOX_HEADER_SIZE;
			}
		}
	}
	return nullptr;
}

uint8_t *GNSS::request(const uint8_t *request, uint16_t size, UBXClass classID, UBXMessageID messageID)
{
	HAL_UART_Transmit(huart, const_cast<uint8_t *>(request), size, 50);
	HAL_UART_Receive(huart, uart_buffer, sizeof(uart_buffer), 500);
	return findHeader(classID, messageID);
}

std::optional<UniqueID> GNSS::getUniqID()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_UNIQUE_ID, sizeof(GNSSCore::GET_UNIQUE_ID), UBXClass::MON, UBXMessageID::UNIQ_ID);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseUniqID(messageBuffer);
}

/*!
 * Make request for UTC time solution data.
 */
std::optional<UTCTime> GNSS::getNavTimeUTC()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_TIME_UTC, sizeof(GNSSCore::GET_NAV_TIME_UTC), UBXClass::NAV, UBXMessageID::UTC_TIME);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavTimeUTC(messageBuffer);
}

/*!
 * Make request for geodetic position solution data.
 */
std::optional<PositionLLH> GNSS::getNavPosLLH()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_POS_LLH, sizeof(GNSSCore::GET_NAV_POS_LLH), UBXClass::NAV, UBXMessageID::POS_LLH);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPosLLH(messageBuffer);
}

/*!
 * Make request for earth centric position solution data.
 */
std::optional<PositionECEF> GNSS::getNavPosECEF()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_POS_ECEF, sizeof(GNSSCore::GET_NAV_POS_ECEF), UBXClass::NAV, UBXMessageID::POS_ECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPosECEF(messageBuffer);
}

/*!
 * Make request for navigation position velocity time solution data.
 */
std::optional<NavigationPVT> GNSS::getNavPVT()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_PVT, sizeof(GNSSCore::GET_NAV_PVT), UBXClass::NAV, UBXMessageID::PVT);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPVT(messageBuffer);
}

/*!
 * Make request for geocentric navigation velocity solution data.
 */
std::optional<VelocityNED> GNSS::getNavVelNED()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_VEL_NED, sizeof(GNSSCore::GET_NAV_VEL_NED), UBXClass::NAV, UBXMessageID::VEL_NED);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavVelNED(messageBuffer);
}

/*!
 * Make request for earth centric navigation velocity solution data.
 */
std::optional<VelocityECEF> GNSS::getNavVelECEF()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_VEL_ECEF, sizeof(GNSSCore::GET_NAV_VEL_ECEF), UBXClass::NAV, UBXMessageID::VEL_ECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavVelECEF(messageBuffer);
}

/*!
 * Changing the GNSS mode.
 */
void GNSS::setMode(GNSSMode gnssMode)
{
	uint8_t *dataToSend = nullptr;
	size_t dataSize = 0;

	switch (gnssMode)
	{
	case GNSSMode::Portable:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_PORTABLE_MODE);
		dataSize = sizeof(GNSSCore::SET_PORTABLE_MODE);
		break;
	case GNSSMode::Stationary:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_STATIONARY_MODE);
		dataSize = sizeof(GNSSCore::SET_STATIONARY_MODE);
		break;
	case GNSSMode::Pedestrian:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_PEDESTRIAN_MODE);
		dataSize = sizeof(GNSSCore::SET_PEDESTRIAN_MODE);
		break;
	case GNSSMode::Automotive:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AUTOMOTIVE_MODE);
		dataSize = sizeof(GNSSCore::SET_AUTOMOTIVE_MODE);
		break;
	case GNSSMode::Sea:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_SEA_MODE);
		dataSize = sizeof(GNSSCore::SET_SEA_MODE);
		break;
	case GNSSMode::Airborne1G:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AIRBORNE_1G_MODE);
		dataSize = sizeof(GNSSCore::SET_AIRBORNE_1G_MODE);
		break;
	case GNSSMode::Airborne2G:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AIRBORNE_2G_MODE);
		dataSize = sizeof(GNSSCore::SET_AIRBORNE_2G_MODE);
		break;
	case GNSSMode::Airborne4G:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AIRBORNE_4G_MODE);
		dataSize = sizeof(GNSSCore::SET_AIRBORNE_4G_MODE);
		break;
	case GNSSMode::Wrist:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_WRIST_MODE);
		dataSize = sizeof(GNSSCore::SET_WRIST_MODE);
		break;
	case GNSSMode::Bike:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_BIKE_MODE);
		dataSize = sizeof(GNSSCore::SET_BIKE_MODE);
		break;
	default:
		return;
	}

	if (dataToSend != nullptr)
	{
		HAL_UART_Transmit_DMA(huart, dataToSend, static_cast<uint16_t>(dataSize));
	}
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 */
void GNSS::loadConfig()
{
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(GNSSCore::CONFIG_UBX), sizeof(GNSSCore::CONFIG_UBX));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(GNSSCore::SET_NMEA_410), sizeof(GNSSCore::SET_NMEA_410));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(GNSSCore::SET_GNSS), sizeof(GNSSCore::SET_GNSS));
	HAL_Delay(250);
}

SimulatedGNSS::SimulatedGNSS(int32_t error_meters) : error_meters(error_meters) {}

int32_t SimulatedGNSS::noise() const
{
	int32_t error_cm_meters = error_meters * 100;
	return std::rand() % (2*error_cm_meters + 1) - error_cm_meters;
}

std::optional<PositionECEF> SimulatedGNSS::getNavPosECEF()
{
    // Approximate ECEF coordinates for Katy, TX (WGS84)
    constexpr int32_t baseX = -1489000 * 100; // meters
    constexpr int32_t baseY = -4743000 * 100;
    constexpr int32_t baseZ = 3980000 * 100;
    constexpr uint32_t baseAcc = 100 * 100; // simulated accuracy

    return PositionECEF{
        .ecefX = baseX + noise(),
        .ecefY = baseY + noise(),
        .ecefZ = baseZ + noise(),
        .pAcc = baseAcc};
}
