
#ifndef INC_MMC5983_H_
#define INC_MMC5983_H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"
#include "Drivers.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

enum class MMC5983_REGISTERS : uint8_t
{
	MMC5983_XOUT0 = 0x00,
	MMC5983_XOUT1 = 0x01,
	MMC5983_YOUT0 = 0x02,
	MMC5983_YOUT1 = 0x03,
	MMC5983_ZOUT0 = 0x04,
	MMC5983_ZOUT1 = 0x05,
	MMC5983_XYZOUT2 = 0x06,
	MMC5983_TOUT = 0x07,
	MMC5983_STATUS = 0x08,
	MMC5983_CONTROL0 = 0x09,
	MMC5983_CONTROL1 = 0x0a,
	MMC5983_CONTROL2 = 0x0b,
	MMC5983_CONTROL3 = 0x0c,
	MMC5983_PRODUCTID = 0x2F,
};

template <typename Transport>
	requires RegisterModeTransport<Transport>
class MMC5983
{
public:
	MMC5983() = delete;
	explicit MMC5983(const Transport &transport) : transport_(transport) {}

	std::optional<ChipID> readChipID() const;
//	std::optional<MagneticFieldInBodyFrame> readMagnetometer() const;
	std::optional<Temperature> readThermometer() const;

private:
	int16_t toInt16(const uint8_t lsb, const uint8_t msb) const
	{
		return (msb << 8) | lsb;
	}

	uint16_t toUInt16(const uint8_t lsb, const uint8_t msb) const
	{
		return (msb << 8) | lsb;
	}

//	au::QuantityF<au::MetersPerSecondSquaredInBodyFrame> convertAcc(const uint8_t lsb, const uint8_t msb) const
//	{
//		constexpr float LSB_PER_G = 16384.0f;
//		constexpr float G = 9.80665f;
//		return au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(toInt16(lsb, msb) * G / LSB_PER_G);
//	}
//
//	au::QuantityF<au::TeslaInBodyFrame> convertMag(const uint8_t lsb, const uint8_t msb) const
//	{
//		constexpr float LSB_PER_DPS = 16.4f;  // ±2000°/s range
//		return au::make_quantity<au::DegreesPerSecondInBodyFrame>(toInt16(lsb, msb) / LSB_PER_DPS);
//	}

	au::QuantityF<au::Celsius> convertTmp(const uint8_t value) const
	{
		constexpr float LSB_PER_TMP = 0.8f;
		constexpr float TMP_SHIFT = -75.0f;
		return au::make_quantity<au::Celsius>(TMP_SHIFT + value * LSB_PER_TMP);
	}

private:
	const Transport &transport_;

	static constexpr uint8_t MMC5983_READ_BIT = 0x80;
};

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<ChipID> MMC5983<Transport>::readChipID() const
{
	uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_PRODUCTID) | MMC5983_READ_BIT;
	uint8_t rx_buf[1];

	if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf)))
	{
		return std::nullopt;
	}

	return rx_buf[0];
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<Temperature> MMC5983<Transport>::readThermometer() const
{
    uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_TOUT) | MMC5983_READ_BIT;
    uint8_t rx_buf[1] = {};

    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
        return std::nullopt;
    }

    return Temperature{ convertTmp(rx_buf[0]) };
}

#endif /* INC_MMC5983_H_ */
