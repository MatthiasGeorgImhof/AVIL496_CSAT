#ifndef _POWER_SWITCH_H_
#define _POWER_SWITCH_H_

#include "Drivers.hpp"
#include <cstdint>
#include <array>

// MCP23008 Register Definitions
enum class MCP23008_REGISTERS : uint8_t
{
    MCP23008_IODIR = 0x00,
    MCP23008_IPOL = 0x01,
    MCP23008_GPINTEN  = 0x02,
    MCP23008_DEFVAL = 0x03,
    MCP23008_INTCON = 0x04,
    MCP23008_IOCON = 0x05,
    MCP23008_GPPU = 0x06,
    MCP23008_INTF = 0x07,
    MCP23008_INTCAP = 0x08,
    MCP23008_GPIO = 0x09,
    MCP23008_OLAT = 0x0A,
};

template <typename Transport>
    requires RegisterModeTransport<Transport>
class PowerSwitch
{
public:
    PowerSwitch() = delete;

    explicit PowerSwitch(const Transport &transport)
        : transport_(transport), register_value_(0)
    {
        uint8_t reset[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        writeRegister(MCP23008_REGISTERS::MCP23008_IODIR, reset, sizeof(reset));
    }

    bool on(uint8_t slot)
    {
        if (invalidSlot(slot))
            return false;
        register_value_ |= static_cast<uint8_t>(1 << slot);
        return writeRegister(MCP23008_REGISTERS::MCP23008_GPIO, &register_value_, 1);
    }

    bool off(uint8_t slot)
    {
        if (invalidSlot(slot))
            return false;
        register_value_ &= static_cast<uint8_t>(~(1 << slot));
        return writeRegister(MCP23008_REGISTERS::MCP23008_GPIO, &register_value_, 1);
    }

    bool status(uint8_t slot) const
    {
        if (invalidSlot(slot))
            return false;
        return (register_value_ & (1 << slot)) != 0;
    }

    bool setState(uint8_t mask)
    {
        register_value_ = mask;
        return writeRegister(MCP23008_REGISTERS::MCP23008_GPIO, &register_value_, 1);
    }

    uint8_t getState() const
    {
        return register_value_;
    }

private:
    bool writeRegister(MCP23008_REGISTERS reg, uint8_t *data, uint16_t size)
    {
        constexpr size_t MaxSize = 16;
        static_assert(MaxSize >= 1, "MaxSize must accommodate register + data");

        if (size > MaxSize - 1)
        {
            return false; // prevent overflow
        }

        std::array<uint8_t, MaxSize> tx{};
        tx[0] = static_cast<uint8_t>(reg);
        std::memcpy(&tx[1], data, size);

        return transport_.write(tx.data(), size + 1);
    }

    static constexpr bool invalidSlot(uint8_t slot)
    {
        return slot >= 8;
    }

private:
    const Transport &transport_;
    uint8_t register_value_;
};

#endif /* _POWER_SWITCH_H_ */
