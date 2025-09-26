
#ifndef _BMI270_MMC9583H_
#define _BMI270_MMC9583H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"
#include "BMI270.hpp"
#include "MMC5983.hpp"
#include "Transport.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

template <typename BMITransport>
class BMI270AuxTransport {
public:
	using config_type = struct { using mode_tag = register_mode_tag; };

	explicit BMI270AuxTransport(const BMITransport& bmi) : bmi_(bmi) {}

    bool write(const uint8_t* tx_buf, uint16_t tx_len) const;
    bool write_then_read(const uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf, uint16_t rx_len) const;

private:
    const BMITransport& bmi_;
};

template <typename BMITransport>
bool BMI270AuxTransport<BMITransport>::write(const uint8_t* tx_buf, uint16_t tx_len) const {
    if (tx_len != 2) return false;

    // Write value to AUX_WR_DATA, then address to AUX_WR_ADDR
    return bmi_.auxWriteRegister(BMI270_REGISTERS::AUX_WR_DATA, tx_buf[1]) &&
           bmi_.auxWriteRegister(BMI270_REGISTERS::AUX_WR_ADDR, tx_buf[0]);
}

template <typename BMITransport>
bool BMI270AuxTransport<BMITransport>::write_then_read(const uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf, uint16_t rx_len) const {
    if (tx_len != 1 || rx_len < 2) return false;

    // Set read address
    if (!bmi_.auxWriteRegister(BMI270_REGISTERS::AUX_RD_ADDR, tx_buf[0])) return false;

    // Wait for BMI270 to fetch from AUX device
    HAL_Delay(1); // or use a smarter wait if you have AUX status polling

    // Read from AUX_DATA_X_LSB
    uint8_t dummy = static_cast<uint8_t>(BMI270_REGISTERS::AUX_DATA_X_LSB) | bmi_.auxReadBit();
    bool ok = bmi_.transport().write_then_read(&dummy, 1, rx_buf, rx_len);

    // Shift buffer left to remove dummy
    memcpy(rx_buf, rx_buf + 1, rx_len - 1);

    return ok;
}

#
#
#

template <typename Transport>
    requires RegisterModeTransport<Transport>
class BMI270_MMC5983 : public BMI270<Transport>
{
public:
    using BMI270Type = BMI270<Transport>;
    using AuxTransport = BMI270AuxTransport<BMI270Type>;
    using Magnetometer = MMC5983<AuxTransport>;

    explicit BMI270_MMC5983(const Transport& transport, const MagnetometerCalibration& calibration = DefaultMMC5983Calibration)
        : BMI270Type(transport), aux_(*this), mag_(aux_, calibration) {}

    bool configure() const;
    std::optional<MagneticFieldInBodyFrame> readMagnetometer() const { return mag_.readMagnetometer(); }
    std::array<int32_t, 3> readRawMagnetometer() const { return mag_.readRawMagnetometer(); }

private:
    bool configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const;

    AuxTransport aux_;
    Magnetometer mag_;

    static constexpr uint8_t MMC5983_I2C = 0x30;
    static constexpr uint8_t MMC5983_ID = 0x30;
};

template <typename Transport>
requires RegisterModeTransport<Transport>
bool BMI270_MMC5983<Transport>::configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const
{
    uint8_t ctrl1 = auto_set ? 0x80 : 0x00;
    uint8_t ctrl2 = (auto_set ? 0x80 : 0x00) | (set_interval_code << 4) | (1 << 3) | freq_code;

	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_WR_DATA, ctrl1);
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_WR_ADDR, static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_CONTROL1));

	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_WR_DATA, ctrl2);
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_WR_ADDR, static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_CONTROL2));
	return true;
}

template <typename Transport>
requires RegisterModeTransport<Transport>
bool BMI270_MMC5983<Transport>::configure() const
{
	BMI270<Transport>::configure();

	uint8_t mag_id;

	BMI270<Transport>::writeRegister(BMI270_REGISTERS::IF_CONF, 0b00100000); // 1. Enable AUX routing
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_IF_CONF, 0x80); // 2. Set AUX interface to manual mode, 1-byte read
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::PWR_CONF, 0x00); // 3. Power on AUX engine
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::PWR_CTRL, 0b00001110);
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_DEV_ID, MMC5983_I2C<<1); // 4. Set MMC5983 I2C address
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_RD_ADDR, 0x2F); // 5. Set read address to PRODUCTID
	HAL_Delay(1);
	BMI270<Transport>::readRegister(BMI270_REGISTERS::AUX_DATA_X_LSB, mag_id); // 6. Read result

	if (mag_id != MMC5983_ID) {
	    log(LOG_LEVEL_ERROR, "BMI270_MMC5983 ID mismatch: got %02x\r\n", mag_id);
	    return false;
	}

	configureContinuousMode(/*freq_code=*/0b101, /*set_interval_code=*/0b011, /*auto_set=*/true);

	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_RD_ADDR, 0x00);
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::IF_CONF, 0b00100000);
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::AUX_IF_CONF, 0x03);
	BMI270<Transport>::writeRegister(BMI270_REGISTERS::PWR_CTRL, 0b00001111);

	return true;
}

#endif /* _BMI270_MMC9583H_ */
