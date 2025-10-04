/*
 * GNSS.h
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
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

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include <cstdint>
#include <optional>

#include "GNSSCore.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include "au.hpp"

struct PositionECEF_AU
{
	au::QuantityF<au::MetersInEcefFrame> x, y, z;
	au::QuantityF<au::MetersInEcefFrame> acc;
};

struct VelocityECEF_AU
{
	au::QuantityF<au::MetersPerSecondInEcefFrame> x, y, z;
	au::QuantityF<au::MetersPerSecondInEcefFrame> acc;
};

PositionECEF_AU ConvertPositionECEF(const PositionECEF &pos);
VelocityECEF_AU ConvertVelocityECEF(const VelocityECEF &vel);

enum GNSSMode : uint8_t
{
	Portable = 0,
	Stationary = 1,
	Pedestrian = 2,
	Automotive = 3,
	Sea = 4,
	Airborne1G = 5,
	Airborne2G = 6,
	Airborne4G = 7,
	Wrist = 8,
	Bike = 9
};

enum UBXClass : uint8_t
{
	MON = 0x27,
	NAV = 0x01
};

enum UBXMessageID : uint8_t
{
	UNIQ_ID = 0x03,
	UTC_TIME = 0x21,
	POS_LLH = 0x02,
	POS_ECEF = 0x01,
	PVT = 0x07,
	VEL_ECEF = 0x11,
	VEL_NED = 0x12
};

class GNSS
{
public:
	GNSS() = delete;
	GNSS(UART_HandleTypeDef *huart);

	void setMode(GNSSMode gnssMode);
	std::optional<UniqueID> getUniqID();
	std::optional<UTCTime> getNavTimeUTC();
	std::optional<PositionLLH> getNavPosLLH();
	std::optional<PositionECEF> getNavPosECEF();
	std::optional<NavigationPVT> getNavPVT();
	std::optional<VelocityECEF> getNavVelECEF();
	std::optional<VelocityNED> getNavVelNED();

private:
	void loadConfig();
	uint8_t *findHeader(UBXClass classID, UBXMessageID messageID);
	uint8_t *request(const uint8_t *request, uint16_t size, UBXClass classID, UBXMessageID messageID);

private:
	static constexpr uint16_t GNSS_BUFFER_SIZE = 201U;
	static constexpr uint16_t UBLOX_HEADER_SIZE = 6U;

private:
	UART_HandleTypeDef *huart;
	uint8_t uart_buffer[GNSS_BUFFER_SIZE];

};

class SimulatedGNSS
{
public:
    SimulatedGNSS() = delete;
    explicit SimulatedGNSS(int32_t error_meters = 100);

    std::optional<PositionECEF> getNavPosECEF();

private:
	int32_t noise() const;

private:
    int32_t error_meters;
};

#endif /* INC_GNSS_H_ */
