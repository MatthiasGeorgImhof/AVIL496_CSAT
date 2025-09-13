#ifndef __TRANSPORTS_HPP__
#define __TRANSPORTS_HPP__

#ifdef __arm__
#include "stm32l4xx_hal.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include <cstdint>
#include <cstddef>
#include <type_traits>

// Mode tags
struct register_mode_tag {};
struct stream_mode_tag {};

// Transport tags
struct i2c_tag {};
struct spi_tag {};
struct uart_tag {};

// ─────────────────────────────────────────────
// I2C Transport (Register Mode)
// ─────────────────────────────────────────────

template<I2C_HandleTypeDef& HandleRef, uint16_t Address, uint32_t Timeout = 100>
struct I2C_Config {
    using transport_tag = i2c_tag;
    using mode_tag = register_mode_tag;

    static I2C_HandleTypeDef& handle() { return HandleRef; }
    static constexpr uint16_t address = Address;
    static constexpr uint32_t timeout = Timeout;

    static_assert(std::is_same_v<decltype(HandleRef), I2C_HandleTypeDef&>, "Handle must be I2C_HandleTypeDef&");
    static_assert(Address <= 0x7F, "I2C address must be 7-bit");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
};

template<typename Config>
requires std::is_same_v<typename Config::transport_tag, i2c_tag>
class I2CTransport {
public:
    using config_type = Config;
    
    bool write_then_read(uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf, uint16_t rx_len) const {
        return HAL_I2C_Master_Transmit(&Config::handle(), Config::address, tx_buf, tx_len, Config::timeout) == HAL_OK &&
               HAL_I2C_Master_Receive(&Config::handle(), Config::address, rx_buf, rx_len, Config::timeout) == HAL_OK;
    }
};

// ─────────────────────────────────────────────
// SPI Transport (Register Mode)
// ─────────────────────────────────────────────

template<SPI_HandleTypeDef& HandleRef, GPIO_TypeDef* PortRef, uint16_t Pin, uint32_t Timeout = 100>
struct SPI_Config {
    using transport_tag = spi_tag;
    using mode_tag = register_mode_tag;

    static SPI_HandleTypeDef& handle() { return HandleRef; }
    static GPIO_TypeDef* csPort() { return PortRef; }
    static constexpr uint16_t csPin = Pin;
    static constexpr uint32_t timeout = Timeout;

    static_assert(std::is_same_v<decltype(HandleRef), SPI_HandleTypeDef&>, "Handle must be SPI_HandleTypeDef&");
    static_assert(std::is_pointer_v<decltype(PortRef)>, "Port must be a GPIO_TypeDef*");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
};

template<typename Config>
requires std::is_same_v<typename Config::transport_tag, spi_tag>
class SPITransport {
public:
    using config_type = Config;
    
    bool write_then_read(uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf, uint16_t rx_len) const {
        select();
        bool ok = HAL_SPI_Transmit(&Config::handle(), tx_buf, tx_len, Config::timeout) == HAL_OK;
        if (ok) {
            ok = HAL_SPI_Receive(&Config::handle(), rx_buf, rx_len, Config::timeout) == HAL_OK;
        }
        deselect();
        return ok;
    }

private:
    void select() const { HAL_GPIO_WritePin(Config::csPort(), Config::csPin, GPIO_PIN_RESET); }
    void deselect() const { HAL_GPIO_WritePin(Config::csPort(), Config::csPin, GPIO_PIN_SET); }
};

// ─────────────────────────────────────────────
// UART Transport (Stream Mode)
// ─────────────────────────────────────────────

template<UART_HandleTypeDef& HandleRef, uint32_t Timeout = 100>
struct UART_Config {
    using transport_tag = uart_tag;
    using mode_tag = stream_mode_tag;

    static UART_HandleTypeDef& handle() { return HandleRef; }
    static constexpr uint32_t timeout = Timeout;

    static_assert(std::is_same_v<decltype(HandleRef), UART_HandleTypeDef&>, "Handle must be UART_HandleTypeDef&");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
};

template<typename Config>
requires std::is_same_v<typename Config::transport_tag, uart_tag>
class UARTTransport {
public:
    using config_type = Config;
    
    bool send(uint8_t* buf, uint16_t len) const {
        return HAL_UART_Transmit(&Config::handle(), buf, len, Config::timeout) == HAL_OK;
    }

    bool receive(uint8_t* buf, uint16_t len) const {
        return HAL_UART_Receive(&Config::handle(), buf, len, Config::timeout) == HAL_OK;
    }
};

// ─────────────────────────────────────────────
// Transport Concepts
// ─────────────────────────────────────────────

template<typename T>
concept RegisterModeTransport = std::is_same_v<typename T::config_type::mode_tag, register_mode_tag>;

template<typename T>
concept StreamModeTransport = std::is_same_v<typename T::config_type::mode_tag, stream_mode_tag>;

template<typename T>
concept TransportProtocol = requires(T t, uint8_t* buf, uint16_t len) {
    { t.write_then_read(buf, len, buf, len) } -> std::same_as<bool>;
} || requires(T t, uint8_t* buf, uint16_t len) {
    { t.send(buf, len) } -> std::same_as<bool>;
    { t.receive(buf, len) } -> std::same_as<bool>;
};

// ─────────────────────────────────────────────
// Transport Kind Traits
// ─────────────────────────────────────────────

enum class TransportKind { I2C, SPI, UART };

template<typename T>
struct TransportTraits;

template<typename Config>
struct TransportTraits<I2CTransport<Config>> {
    static constexpr TransportKind kind = TransportKind::I2C;
};

template<typename Config>
struct TransportTraits<SPITransport<Config>> {
    static constexpr TransportKind kind = TransportKind::SPI;
};

template<typename Config>
struct TransportTraits<UARTTransport<Config>> {
    static constexpr TransportKind kind = TransportKind::UART;
};

#endif // __TRANSPORTS_HPP__
