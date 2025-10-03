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

union u_Short
{
	uint8_t bytes[2];
	uint16_t uShort;
} uShort;

union i_Short
{
	uint8_t bytes[2];
	int16_t iShort;
} iShort;

union u_Long
{
	uint8_t bytes[4];
	uint32_t uLong;
} uLong;

union i_Long
{
	uint8_t bytes[4];
	int32_t iLong;
} iLong;

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
	uint8_t *messageBuffer = request(GET_UNIQUE_ID, sizeof(GET_UNIQUE_ID), UBXClass::MON, UBXMessageID::UNIQ_ID);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseUniqID(messageBuffer);
}

/*!
 * Make request for UTC time solution data.
 */
std::optional<UTCTime> GNSS::getNavTimeUTC()
{
	uint8_t *messageBuffer = request(GET_NAV_TIME_UTC, sizeof(GET_NAV_TIME_UTC), UBXClass::NAV, UBXMessageID::UTC_TIME);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavTimeUTC(messageBuffer);
}

/*!
 * Make request for geodetic position solution data.
 */
std::optional<PositionLLH> GNSS::getNavPosLLH()
{
	uint8_t *messageBuffer = request(GET_NAV_POS_LLH, sizeof(GET_NAV_POS_LLH), UBXClass::NAV, UBXMessageID::POS_LLH);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPosLLH(messageBuffer);
}

/*!
 * Make request for earth centric position solution data.
 */
std::optional<PositionECEF> GNSS::getNavPosECEF()
{
	uint8_t *messageBuffer = request(GET_NAV_POS_ECEF, sizeof(GET_NAV_POS_ECEF), UBXClass::NAV, UBXMessageID::POS_ECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPosECEF(messageBuffer);
}

/*!
 * Make request for navigation position velocity time solution data.
 */
std::optional<NavigationPVT> GNSS::getNavPVT()
{
	uint8_t *messageBuffer = request(GET_NAV_PVT, sizeof(GET_NAV_PVT), UBXClass::NAV, UBXMessageID::PVT);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPVT(messageBuffer);
}

/*!
 * Make request for geocentric navigation velocity solution data.
 */
std::optional<VelocityNED> GNSS::getNavVelNED()
{
	uint8_t *messageBuffer = request(GET_NAV_VEL_NED, sizeof(GET_NAV_VEL_NED), UBXClass::NAV, UBXMessageID::VEL_NED);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavVelNED(messageBuffer);
}

/*!
 * Make request for earth centric navigation velocity solution data.
 */
std::optional<VelocityECEF> GNSS::getNavVelECEF()
{
	uint8_t *messageBuffer = request(GET_NAV_VEL_ECEF, sizeof(GET_NAV_VEL_ECEF), UBXClass::NAV, UBXMessageID::VEL_ECEF);
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
		dataToSend = const_cast<uint8_t *>(SET_PORTABLE_MODE);
		dataSize = sizeof(SET_PORTABLE_MODE);
		break;
	case GNSSMode::Stationary:
		dataToSend = const_cast<uint8_t *>(SET_STATIONARY_MODE);
		dataSize = sizeof(SET_STATIONARY_MODE);
		break;
	case GNSSMode::Pedestrian:
		dataToSend = const_cast<uint8_t *>(SET_PEDESTRIAN_MODE);
		dataSize = sizeof(SET_PEDESTRIAN_MODE);
		break;
	case GNSSMode::Automotive:
		dataToSend = const_cast<uint8_t *>(SET_AUTOMOTIVE_MODE);
		dataSize = sizeof(SET_AUTOMOTIVE_MODE);
		break;
	case GNSSMode::Sea:
		dataToSend = const_cast<uint8_t *>(SET_SEA_MODE);
		dataSize = sizeof(SET_SEA_MODE);
		break;
	case GNSSMode::Airborne1G:
		dataToSend = const_cast<uint8_t *>(SET_AIRBORNE_1G_MODE);
		dataSize = sizeof(SET_AIRBORNE_1G_MODE);
		break;
	case GNSSMode::Airborne2G:
		dataToSend = const_cast<uint8_t *>(SET_AIRBORNE_2G_MODE);
		dataSize = sizeof(SET_AIRBORNE_2G_MODE);
		break;
	case GNSSMode::Airborne4G:
		dataToSend = const_cast<uint8_t *>(SET_AIRBORNE_4G_MODE);
		dataSize = sizeof(SET_AIRBORNE_4G_MODE);
		break;
	case GNSSMode::Wrist:
		dataToSend = const_cast<uint8_t *>(SET_WRIST_MODE);
		dataSize = sizeof(SET_WRIST_MODE);
		break;
	case GNSSMode::Bike:
		dataToSend = const_cast<uint8_t *>(SET_BIKE_MODE);
		dataSize = sizeof(SET_BIKE_MODE);
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
 * Parse data to unique chip ID standard.
 */
UniqueID GNSSCore::parseUniqID(uint8_t *messageBuffer)
{
	return UniqueID{
		.id = {getUByte(messageBuffer, 4),
			   getUByte(messageBuffer, 5),
			   getUByte(messageBuffer, 6),
			   getUByte(messageBuffer, 7),
			   getUByte(messageBuffer, 8),
			   getUByte(messageBuffer, 9)}};
}

/*!
 * Parse data to navigation position velocity time solution standard.
 */
NavigationPVT GNSSCore::parseNavPVT(uint8_t *messageBuffer)
{
	return NavigationPVT{
		.utcTime = {
			.year = getUShort(messageBuffer, 4),							  // year
			.month = getUByte(messageBuffer, 6),							  // month
			.day = getUByte(messageBuffer, 7),								  // day
			.hour = getUByte(messageBuffer, 8),								  // hour
			.min = getUByte(messageBuffer, 9),								  // min
			.sec = getUByte(messageBuffer, 10),								  // sec
			.nano = getILong(messageBuffer, 16),							  // nano seconds
			.tAcc = getULong(messageBuffer, 12),							  // time accuracy
			.valid = static_cast<uint8_t>(getUByte(messageBuffer, 11) & 0x0f) // valid flags
		},
		.position = {
			// position are in mm, so we divide by 10 to get cm
			.lon = getILong(messageBuffer, 24),			// longitude
			.lat = getILong(messageBuffer, 28),			// latitude
			.height = getILong(messageBuffer, 32) / 10, // height above ellipsoid
			.hMSL = getILong(messageBuffer, 36) / 10,	// height above mean sea level
			.hAcc = getULong(messageBuffer, 40) / 10,	// horizontal accuracy estimate
			.vAcc = getULong(messageBuffer, 44) / 10	// vertical accuracy estimate
		},
		.velocity = {
			// velocities are in mm/s, so we divide by 10 to get cm/s
			.velN = getILong(messageBuffer, 48) / 10, // north velocity component
			.velE = getILong(messageBuffer, 52) / 10, // east velocity component
			.velD = getILong(messageBuffer, 56) / 10, // down velocity component
			.headMot = getILong(messageBuffer, 64),	  // heading of motion (2-D)
			.speed = 0,
			.gSpeed = getULong(messageBuffer, 60) / 10, // ground speed estimate
			.sAcc = getULong(messageBuffer, 68),		// speed accuracy estimate
			.headAcc = getULong(messageBuffer, 72)		// heading accuracy estimate
		},
		.fixType = getIByte(messageBuffer, 20), // fix type
		.numSV = getUByte(messageBuffer, 23)	// number of satellites used
	};
}

/*!
 * Parse data to UTC time solution standard.
 */
UTCTime GNSSCore::parseNavTimeUTC(uint8_t *messageBuffer)
{
	return UTCTime{
		.year = getUShort(messageBuffer, 12),							  // year
		.month = getUByte(messageBuffer, 14),							  // month
		.day = getUByte(messageBuffer, 15),								  // day
		.hour = getUByte(messageBuffer, 16),							  // hour
		.min = getUByte(messageBuffer, 17),								  // min
		.sec = getUByte(messageBuffer, 18),								  // sec
		.nano = getILong(messageBuffer, 8),								  // nano seconds
		.tAcc = getULong(messageBuffer, 4),								  // time accuracy
		.valid = static_cast<uint8_t>(getUByte(messageBuffer, 19) & 0x0f) // valid flags
	};
}

/*!
 * Parse data to geodetic position solution standard.
 */
PositionLLH GNSSCore::parseNavPosLLH(uint8_t *messageBuffer)
{
	return PositionLLH{
		.lon = getILong(messageBuffer, 4),	   // lon
		.lat = getILong(messageBuffer, 8),	   // lat
		.height = getILong(messageBuffer, 12), // height
		.hMSL = getILong(messageBuffer, 16),   // hMSL
		.hAcc = getULong(messageBuffer, 20),   // hAcc
		.vAcc = getULong(messageBuffer, 24)	   // vAcc
	};
}

/*!
 * Parse data to earth centric position solution standard.
 */
PositionECEF GNSSCore::parseNavPosECEF(uint8_t *messageBuffer)
{
	return PositionECEF{
		.ecefX = getILong(messageBuffer, 4),  // ecefX
		.ecefY = getILong(messageBuffer, 8),  // ecefY
		.ecefZ = getILong(messageBuffer, 12), // ecefZ
		.pAcc = getULong(messageBuffer, 16)	  // pAcc
	};
}

/*!
 * Parse data to geodetic velocity solution standard.
 */
VelocityNED GNSSCore::parseNavVelNED(uint8_t *messageBuffer)
{
	return VelocityNED{
		.velN = getILong(messageBuffer, 4),		// velN
		.velE = getILong(messageBuffer, 8),		// velE
		.velD = getILong(messageBuffer, 12),	// velD
		.headMot = getILong(messageBuffer, 24), // headMot
		.speed = getULong(messageBuffer, 16),	// gSpeed
		.gSpeed = getULong(messageBuffer, 20),	// gSpeed
		.sAcc = getULong(messageBuffer, 28),	// sAcc
		.headAcc = getULong(messageBuffer, 32)	// headAcc
	};
}

/*!
 * Parse data to earth centric velocity solution standard.
 */
VelocityECEF GNSSCore::parseNavVelECEF(uint8_t *messageBuffer)
{
	return VelocityECEF{
		.ecefVX = getILong(messageBuffer, 4),  // velX
		.ecefVY = getILong(messageBuffer, 8),  // velY
		.ecefVZ = getILong(messageBuffer, 12), // velZ
		.sAcc = getULong(messageBuffer, 16)	   // sAcc
	};
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 */
void GNSS::loadConfig()
{
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(CONFIG_UBX), sizeof(CONFIG_UBX));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(SET_NMEA_410), sizeof(SET_NMEA_410));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(SET_GNSS), sizeof(SET_GNSS));
	HAL_Delay(250);
}

/*!
 *  Creates a checksum based on UBX standard.
 */
void GNSSCore::checksum(uint16_t length, const uint8_t *payload, uint8_t *cka, uint8_t *ckb)
{
	uint8_t cka_ = 0;
	uint8_t ckb_ = 0;

	for (int i = 0; i < length; ++i)
	{
		cka_ += payload[i];
		ckb_ += cka_;
	}
	*cka = cka_;
	*ckb = ckb_;
}

uint8_t GNSSCore::getUByte(const uint8_t *messageBuffer, uint16_t offset)
{
	return static_cast<uint8_t>(messageBuffer[offset]);
}

int8_t GNSSCore::getIByte(const uint8_t *messageBuffer, uint16_t offset)
{
	return static_cast<int8_t>(messageBuffer[offset]);
}

uint16_t GNSSCore::getUShort(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 2; ++var)
	{
		uShort.bytes[var] = messageBuffer[var + offset];
	}
	return uShort.uShort;
}

int16_t GNSSCore::getIShort(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 2; ++var)
	{
		iShort.bytes[var] = messageBuffer[var + offset];
	}
	return iShort.iShort;
}

uint32_t GNSSCore::getULong(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 4; ++var)
	{
		uLong.bytes[var] = messageBuffer[var + offset];
	}
	return uLong.uLong;
}

int32_t GNSSCore::getILong(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 4; ++var)
	{
		iLong.bytes[var] = messageBuffer[var + offset];
	}
	return iLong.iLong;
}

PositionECEF_AU ConvertPositionECEF(const PositionECEF &pos)
{
	return PositionECEF_AU{
		.x = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.ecefX)),
		.y = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.ecefY)),
		.z = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.ecefZ)),
		.acc = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.pAcc))};
}

VelocityECEF_AU ConvertVelocityECEF(const VelocityECEF &vel)
{
	return VelocityECEF_AU{
		.x = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.ecefVX)),
		.y = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.ecefVY)),
		.z = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.ecefVZ)),
		.acc = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.sAcc))};
}
