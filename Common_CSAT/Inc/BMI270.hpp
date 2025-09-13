
#ifndef INC_BMI270_H_
#define INC_BMI270_H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"
#include "Drivers.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

template <typename Transport>
	requires RegisterModeTransport<Transport>
class BMI270
{
public:
	BMI270() = delete;
	explicit BMI270(const Transport &transport) : transport_(transport) {}

	std::optional<ChipID> readChipID() const;
	std::optional<AccelerationInBodyFrame> readAccelerometer() const;
	std::optional<AngularVelocityInBodyFrame> readGyroscope() const;
	std::optional<MagneticFieldInBodyFrame> readMagnetometer() const;
	std::optional<Temperature> readThermometer() const;

private:
	int16_t toInt16(const uint8_t lsb, const uint8_t msb) const
	{
		return (msb << 8) | lsb;
	}

	au::QuantityF<au::MetersPerSecondSquaredInBodyFrame> convertAcc(const uint8_t lsb, const uint8_t msb) const
	{
		constexpr float LSB_PER_G = 16384.0f;
		constexpr float G = 9.80665f;
		return au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(toInt16(lsb, msb) * G / LSB_PER_G);
	}

	au::QuantityF<au::DegreesPerSecondInBodyFrame> convertGyr(const uint8_t lsb, const uint8_t msb) const
	{
		constexpr float LSB_PER_DPS = 16.4f;  // ±2000°/s range
		return au::make_quantity<au::DegreesPerSecondInBodyFrame>(toInt16(lsb, msb) / LSB_PER_DPS);
	}

	au::QuantityF<au::Celsius> convertTmp(const uint8_t lsb, const uint8_t msb) const
	{
		constexpr float LSB_PER_TMP = 1.0/512.0f;  // 0.001953125 °C/LSB
		constexpr float TMP_SHIFT = 23.0f;
		return au::make_quantity<au::Celsius>(TMP_SHIFT + toInt16(lsb, msb) * LSB_PER_TMP);
	}

private:
	const Transport &transport_;

	static constexpr uint8_t BMI270_READ_BIT = 0x80;

	// Common Registers
	static constexpr uint8_t BMI270_REG_CHIP_ID = 0x00;
	
	static constexpr uint8_t BMI270_REG_ACC_DATA_X_LSB = 0x0C;
	static constexpr uint8_t BMI270_REG_ACC_DATA_X_MSB = 0x0D;
	static constexpr uint8_t BMI270_REG_ACC_DATA_Y_LSB = 0x0E;
	static constexpr uint8_t BMI270_REG_ACC_DATA_Y_MSB = 0x0F;
	static constexpr uint8_t BMI270_REG_ACC_DATA_Z_LSB = 0x10;
	static constexpr uint8_t BMI270_REG_ACC_DATA_Z_MSB = 0x11;
	
	static constexpr uint8_t BMI270_REG_GYR_DATA_X_LSB = 0x12;
	static constexpr uint8_t BMI270_REG_GYR_DATA_X_MSB = 0x13;
	static constexpr uint8_t BMI270_REG_GYR_DATA_Y_LSB = 0x14;
	static constexpr uint8_t BMI270_REG_GYR_DATA_Y_MSB = 0x15;
	static constexpr uint8_t BMI270_REG_GYR_DATA_Z_LSB = 0x16;
	static constexpr uint8_t BMI270_REG_GYR_DATA_Z_MSB = 0x17;

	static constexpr uint8_t BMI270_REG_TMP_DATA_LSB =  0x22;
	static constexpr uint8_t BMI270_REG_TMP_DATA_MSB =  0x23;

	static constexpr uint8_t BMI270_REG_STATUS = 0x21;
	static constexpr uint8_t BMI270_REG_FEAT_PAGE = 0x2F;
	static constexpr uint8_t BMI270_REG_FEATURES_START = 0x30;
	static constexpr uint8_t BMI270_REG_PWR_CONF = 0x7C;
	static constexpr uint8_t BMI270_REG_PWR_CTRL = 0x7D;
	static constexpr uint8_t BMI270_REG_CMD = 0x7E;

	// Common Command Register (0x7E) values
	static constexpr uint8_t BMI270_CMD_SOFT_RESET = 0xB6;
	static constexpr uint8_t BMI270_CMD_ACCEL_ENABLE = 0x04;
	static constexpr uint8_t BMI270_CMD_GYRO_ENABLE = 0x08;
	static constexpr uint8_t BMI270_CMD_AUX_ENABLE = 0x02;
};

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<ChipID> BMI270<Transport>::readChipID() const
{
	uint8_t tx_buf = 0x00 | BMI270_READ_BIT;
	uint8_t rx_buf{};

	if (!transport_.write_then_read(&tx_buf, 1, &rx_buf, 1))
	{
		return std::nullopt;
	}

	return rx_buf;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<AccelerationInBodyFrame> BMI270<Transport>::readAccelerometer() const
{
    uint8_t tx_buf = BMI270_REG_ACC_DATA_X_LSB | BMI270_READ_BIT;
    uint8_t rx_buf[6] = {};

    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
        return std::nullopt;
    }

    return AccelerationInBodyFrame{
        convertAcc(rx_buf[0], rx_buf[1]),
        convertAcc(rx_buf[2], rx_buf[3]),
        convertAcc(rx_buf[4], rx_buf[5])
    };
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<AngularVelocityInBodyFrame> BMI270<Transport>::readGyroscope() const
{
    uint8_t tx_buf = BMI270_REG_GYR_DATA_X_LSB | BMI270_READ_BIT;
    uint8_t rx_buf[6] = {};

    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
        return std::nullopt;
    }

    return AngularVelocityInBodyFrame{
        convertGyr(rx_buf[0], rx_buf[1]),
        convertGyr(rx_buf[2], rx_buf[3]),
        convertGyr(rx_buf[4], rx_buf[5])
    };
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<Temperature> BMI270<Transport>::readThermometer() const
{
    uint8_t tx_buf = BMI270_REG_TMP_DATA_LSB | BMI270_READ_BIT;
    uint8_t rx_buf[2] = {};

    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
        return std::nullopt;
    }

    return Temperature{ convertTmp(rx_buf[0], rx_buf[1]) };
}

#endif /* INC_BMI270_H_ */
