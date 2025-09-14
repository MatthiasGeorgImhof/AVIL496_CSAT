#ifndef _POWER_SWITCH_H_
#define _POWER_SWITCH_H_

#include "Drivers.hpp"
#include <cstdint>
#include <array>


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
        writeRegister(IODIR, reset, sizeof(reset));
    }

    bool on(uint8_t slot)
    {
        if (invalidSlot(slot))
            return false;
        register_value_ |= static_cast<uint8_t>(1 << (2 * slot));
        return writeRegister(GPIO, &register_value_, 1);
    }

    bool off(uint8_t slot)
    {
        if (invalidSlot(slot))
            return false;
        register_value_ &= static_cast<uint8_t>(~(1 << (2 * slot)));
        return writeRegister(GPIO, &register_value_, 1);
    }

    bool status(uint8_t slot) const
    {
        if (invalidSlot(slot))
            return false;
        return (register_value_ & (1 << (2 * slot))) != 0;
    }

    bool setState(uint8_t mask)
    {
        register_value_ = mask;
        return writeRegister(GPIO, &register_value_, 1);
    }

    uint8_t getState() const
    {
        return register_value_;
    }

private:
    static constexpr uint8_t IODIR = 0x00;
    static constexpr uint8_t GPIO = 0x09;
    static constexpr uint8_t OLAT = 0x0A;

    bool writeRegister(uint8_t reg, uint8_t *data, uint16_t size)
    {
        constexpr size_t MaxSize = 16;
        static_assert(MaxSize >= 1, "MaxSize must accommodate register + data");

        if (size > MaxSize - 1)
        {
            return false; // prevent overflow
        }

        std::array<uint8_t, MaxSize> tx{};
        tx[0] = reg;
        std::memcpy(&tx[1], data, size);

        return transport_.write(tx.data(), size + 1);
    }

    static constexpr bool invalidSlot(uint8_t slot)
    {
        return slot >= 4;
    }

private:
    const Transport &transport_;
    uint8_t register_value_;
};

#endif /* _POWER_SWITCH_H_ */
