
#ifndef _MMC5983_H_
#define _MMC5983_H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"
#include "Transport.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

enum class MMC5983_REGISTERS : uint8_t
{
<<<<<<< HEAD
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
=======
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
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
};

#include <array>

<<<<<<< HEAD
struct MagnetometerCalibration {
	MagneticFieldInBodyFrame bias;
=======
struct MagnetometerCalibration
{
    MagneticFieldInBodyFrame bias;
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
    std::array<std::array<float, 3>, 3> scale;
};

constexpr MagnetometerCalibration DefaultMMC5983Calibration = {
<<<<<<< HEAD
    { au::make_quantity<au::TeslaInBodyFrame>(0.0f), au::make_quantity<au::TeslaInBodyFrame>(0.0f), au::make_quantity<au::TeslaInBodyFrame>(0.0f) },
    {{
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}
    }}
};

class MMC5983Core {
public:
    static constexpr int32_t NULL_VALUE = 131072;

    static int32_t toInt32(uint8_t lsb, uint8_t isb, uint8_t msb) {
        return ((msb << 10) | (isb << 2) | lsb) - NULL_VALUE;
    }

    static au::QuantityF<au::TeslaInBodyFrame> convertMag(uint8_t lsb, uint8_t isb, uint8_t msb) {
        constexpr float COUNT_PER_GAUSS = 16384.f;
        constexpr float GAUSS_PER_TESLA = 10000.f;
        constexpr float TESLA_PER_COUNT = 1.f / (COUNT_PER_GAUSS * GAUSS_PER_TESLA);
        return au::make_quantity<au::TeslaInBodyFrame>(toInt32(lsb, isb, msb) * TESLA_PER_COUNT);
    }

	static au::QuantityF<au::Celsius> convertTmp(const uint8_t value)
	{
		constexpr float LSB_PER_TMP = 0.8f;
		constexpr float TMP_SHIFT = -75.0f;
		return au::make_quantity<au::Celsius>(TMP_SHIFT + value * LSB_PER_TMP);
	}

    static MagneticFieldInBodyFrame parseMagnetometerData(const uint8_t* buf) {
        return MagneticFieldInBodyFrame{
            convertMag((buf[6] >> 6) & 0b11, buf[1], buf[0]),
            convertMag((buf[6] >> 4) & 0b11, buf[3], buf[2]),
            convertMag((buf[6] >> 2) & 0b11, buf[5], buf[4])
        };
    }

    static std::array<int32_t, 3> parseRawMagnetometerData(const uint8_t* buf) {
        return {
            toInt32((buf[6] >> 6) & 0b11, buf[1], buf[0]),
            toInt32((buf[6] >> 4) & 0b11, buf[3], buf[2]),
            toInt32((buf[6] >> 2) & 0b11, buf[5], buf[4])
        };
=======
    {au::make_quantity<au::TeslaInBodyFrame>(0.0f), au::make_quantity<au::TeslaInBodyFrame>(0.0f), au::make_quantity<au::TeslaInBodyFrame>(0.0f)},
    {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}}};

class MMC5983Core
{
public:
    static constexpr int32_t NULL_VALUE = 131072;

    static int32_t toInt32(uint8_t lsb, uint8_t isb, uint8_t msb)
    {
        return ((msb << 10) | (isb << 2) | lsb) - NULL_VALUE;
    }

    static au::QuantityF<au::TeslaInBodyFrame> convertMag(uint8_t lsb, uint8_t isb, uint8_t msb)
    {
        constexpr float COUNT_PER_GAUSS = 16384.f;
        constexpr float GAUSS_PER_TESLA = 10000.f;
        constexpr float TESLA_PER_COUNT = 1.f / (COUNT_PER_GAUSS * GAUSS_PER_TESLA);
        return au::make_quantity<au::TeslaInBodyFrame>(static_cast<float>(toInt32(lsb, isb, msb)) * TESLA_PER_COUNT);
    }

    static au::QuantityF<au::Celsius> convertTmp(const uint8_t value)
    {
        constexpr float LSB_PER_TMP = 0.8f;
        constexpr float TMP_SHIFT = -75.0f;
        return au::make_quantity<au::Celsius>(TMP_SHIFT + value * LSB_PER_TMP);
    }

    static MagneticFieldInBodyFrame parseMagnetometerData(const uint8_t *buf)
    {
        return MagneticFieldInBodyFrame{
            convertMag((buf[6] >> 6) & 0b11, buf[1], buf[0]),
            convertMag((buf[6] >> 4) & 0b11, buf[3], buf[2]),
            convertMag((buf[6] >> 2) & 0b11, buf[5], buf[4])};
    }

    static std::array<int32_t, 3> parseRawMagnetometerData(const uint8_t *buf)
    {
        return {
            toInt32((buf[6] >> 6) & 0b11, buf[1], buf[0]),
            toInt32((buf[6] >> 4) & 0b11, buf[3], buf[2]),
            toInt32((buf[6] >> 2) & 0b11, buf[5], buf[4])};
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
    }
};

template <typename Transport>
<<<<<<< HEAD
	requires RegisterModeTransport<Transport>
class MMC5983
{
public:
	MMC5983() = delete;
	explicit MMC5983(const Transport &transport, const MagnetometerCalibration& calibration = DefaultMMC5983Calibration) : transport_(transport), calibration_(calibration) {}
	bool initialize() const;
	bool configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const;

	uint8_t readStatus() const;
	std::optional<ChipID> readChipID() const;

	std::optional<MagneticFieldInBodyFrame> readMagnetometer() const;
	std::optional<Temperature> readThermometer() const;

	std::array<int32_t, 3> readRawMagnetometer() const;
	uint8_t readRawThermometer() const;

	bool performSet() const { return writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL0, 0x08); }

private:
	bool writeRegister(MMC5983_REGISTERS reg, uint8_t value) const;
	bool readRegister(MMC5983_REGISTERS reg, uint8_t &value) const;

private:
	const Transport &transport_;
	const MagnetometerCalibration& calibration_;
	static constexpr uint8_t MMC5983_READ_BIT = 0x80;
};

template <typename Transport>
	requires RegisterModeTransport<Transport>
=======
    requires RegisterModeTransport<Transport>
class MMC5983
{
public:
    MMC5983() = delete;
    explicit MMC5983(const Transport &transport, const MagnetometerCalibration &calibration = DefaultMMC5983Calibration) : transport_(transport), calibration_(calibration) {}
    bool initialize() const;
    bool configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const;

    uint8_t readStatus() const;
    std::optional<ChipID> readChipID() const;

    std::optional<MagneticFieldInBodyFrame> readMagnetometer() const;
    std::optional<Temperature> readThermometer() const;

    std::array<int32_t, 3> readRawMagnetometer() const;
    uint8_t readRawThermometer() const;

    bool performSet() const { return writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL0, 0x08); }

private:
    bool writeRegister(MMC5983_REGISTERS reg, uint8_t value) const;
    bool readRegister(MMC5983_REGISTERS reg, uint8_t &value) const;

private:
    const Transport &transport_;
    const MagnetometerCalibration &calibration_;
    static constexpr uint8_t MMC5983_READ_BIT = 0x80;
};

template <typename Transport>
    requires RegisterModeTransport<Transport>
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
bool MMC5983<Transport>::writeRegister(MMC5983_REGISTERS reg, uint8_t value) const
{
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg), value};
    return transport_.write(tx_buf, sizeof(tx_buf));
}

template <typename Transport>
<<<<<<< HEAD
	requires RegisterModeTransport<Transport>
=======
    requires RegisterModeTransport<Transport>
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
bool MMC5983<Transport>::readRegister(MMC5983_REGISTERS reg, uint8_t &value) const
{
    uint8_t tx_buf[1] = {static_cast<uint8_t>(reg) | MMC5983_READ_BIT};
    uint8_t rx_buf[2]{};
    bool ok = transport_.write_then_read(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));
<<<<<<< HEAD
    if (! ok) return false;
=======
    if (!ok)
        return false;
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
    value = rx_buf[1];
    return true;
}

template <typename Transport>
<<<<<<< HEAD
	requires RegisterModeTransport<Transport>
=======
    requires RegisterModeTransport<Transport>
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
bool MMC5983<Transport>::configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const
{
    uint8_t ctrl1 = auto_set ? 0x80 : 0x00;
    uint8_t ctrl2 = (auto_set ? 0x80 : 0x00) | (set_interval_code << 4) | (1 << 3) | freq_code;
    return writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL1, ctrl1) &&
           writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL2, ctrl2);
}

template <typename Transport>
<<<<<<< HEAD
	requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::initialize() const
{
	return configureContinuousMode(/*freq_code=*/0b101, /*set_interval_code=*/0b011, /*auto_set=*/true);
}

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
=======
    requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::initialize() const
{
    return configureContinuousMode(/*freq_code=*/0b101, /*set_interval_code=*/0b011, /*auto_set=*/true);
}

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
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
uint8_t MMC5983<Transport>::readStatus() const
{
    uint8_t value = 0;
    readRegister(MMC5983_REGISTERS::MMC5983_STATUS, value);
    return value;
}

template <typename Transport>
<<<<<<< HEAD
	requires RegisterModeTransport<Transport>
std::optional<MagneticFieldInBodyFrame> MMC5983<Transport>::readMagnetometer() const
{
	uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_XOUT0) | MMC5983_READ_BIT;
    uint8_t rx_buf[8]{};

    if (!MMC5983<Transport>::transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
    	return std::nullopt;
=======
    requires RegisterModeTransport<Transport>
std::optional<MagneticFieldInBodyFrame> MMC5983<Transport>::readMagnetometer() const
{
    uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_XOUT0) | MMC5983_READ_BIT;
    uint8_t rx_buf[8]{};

    if (!MMC5983<Transport>::transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf)))
    {
        return std::nullopt;
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
    }

    auto uncalibrated = MMC5983Core::parseMagnetometerData(rx_buf);
    uncalibrated[0] -= calibration_.bias[0];
    uncalibrated[1] -= calibration_.bias[1];
    uncalibrated[2] -= calibration_.bias[2];

<<<<<<< HEAD
    return MagneticFieldInBodyFrame {
    	uncalibrated[0]*calibration_.scale[0][0] + uncalibrated[1]*calibration_.scale[0][1] + uncalibrated[2]*calibration_.scale[0][2],
    	uncalibrated[0]*calibration_.scale[1][0] + uncalibrated[1]*calibration_.scale[1][1] + uncalibrated[2]*calibration_.scale[1][2],
    	uncalibrated[0]*calibration_.scale[2][0] + uncalibrated[1]*calibration_.scale[2][1] + uncalibrated[2]*calibration_.scale[2][2]
    };
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
=======
    return MagneticFieldInBodyFrame{
        uncalibrated[0] * calibration_.scale[0][0] + uncalibrated[1] * calibration_.scale[0][1] + uncalibrated[2] * calibration_.scale[0][2],
        uncalibrated[0] * calibration_.scale[1][0] + uncalibrated[1] * calibration_.scale[1][1] + uncalibrated[2] * calibration_.scale[1][2],
        uncalibrated[0] * calibration_.scale[2][0] + uncalibrated[1] * calibration_.scale[2][1] + uncalibrated[2] * calibration_.scale[2][2]};
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
std::optional<Temperature> MMC5983<Transport>::readThermometer() const
{
    uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_TOUT) | MMC5983_READ_BIT;
    uint8_t rx_buf[1] = {};

<<<<<<< HEAD
    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
        return std::nullopt;
    }

    return Temperature{ MMC5983Core::convertTmp(rx_buf[0]) };
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::array<int32_t, 3> MMC5983<Transport>::readRawMagnetometer() const
{
	uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_XOUT0) | MMC5983_READ_BIT;
    uint8_t rx_buf[8]{};

    if (!MMC5983<Transport>::transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
    	memset(rx_buf, 0, sizeof(rx_buf));
=======
    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf)))
    {
        return std::nullopt;
    }

    return Temperature{MMC5983Core::convertTmp(rx_buf[0])};
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
std::array<int32_t, 3> MMC5983<Transport>::readRawMagnetometer() const
{
    uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_XOUT0) | MMC5983_READ_BIT;
    uint8_t rx_buf[8]{};

    if (!MMC5983<Transport>::transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf)))
    {
        memset(rx_buf, 0, sizeof(rx_buf));
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
    }

    return MMC5983Core::parseRawMagnetometerData(rx_buf);
}

template <typename Transport>
<<<<<<< HEAD
	requires RegisterModeTransport<Transport>
=======
    requires RegisterModeTransport<Transport>
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
uint8_t MMC5983<Transport>::readRawThermometer() const
{
    writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL0, 0b10);
    HAL_Delay(5);

<<<<<<< HEAD
	uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_TOUT) | MMC5983_READ_BIT;
    uint8_t rx_buf[1]{};

    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf))) {
    	rx_buf[0] = 255;
=======
    uint8_t tx_buf = static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_TOUT) | MMC5983_READ_BIT;
    uint8_t rx_buf[1]{};

    if (!transport_.write_then_read(&tx_buf, 1, rx_buf, sizeof(rx_buf)))
    {
        rx_buf[0] = 255;
>>>>>>> 253e199ab4359115119738472e9c8f2e21082cbb
    }

    return rx_buf[0];
}

#endif /* _MMC5983_H_ */
