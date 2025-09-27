#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "PowerSwitch.hpp"
#include "mock_hal.h"
#include "Transport.hpp"

#include <iostream> // For debugging output (remove in production tests)
#include <bitset>   // For visualizing bit patterns

TEST_CASE("PowerSwitch On/Off/Status")
{
    static I2C_HandleTypeDef hi2c;
    constexpr uint8_t address = 0x40;
    constexpr uint8_t ps_register = 0x0a;

    using SwitchConfig = I2C_Config<hi2c, address>;
    using SwitchTransport = I2CTransport<SwitchConfig>;
    SwitchTransport transport;

    PowerSwitch<SwitchTransport> pm(transport);

    // Reset the I2C buffer before each test case
    clear_i2c_mem_data();

    SUBCASE("Turn on slot 0")
    {
        pm.on(0);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00000001); // Bit 0 should be set
        CHECK(pm.status(0) == true);
    }

    SUBCASE("Turn off slot 0")
    {
        // First, turn it on
        pm.on(0);
        pm.off(0);

        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00000000); // Bit 0 should be cleared
        CHECK(pm.status(0) == false);
    }

    SUBCASE("Turn on slot 1")
    {
        pm.on(1);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00000010); // Bit 1 should be set
        CHECK(pm.status(1) == true);
    }

    SUBCASE("Turn on slot 2")
    {
        pm.on(2);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00000100); // Bit 2 should be set
        CHECK(pm.status(2) == true);
    }

    SUBCASE("Turn on slot 3")
    {
        pm.on(3);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00001000); // Bit 3 should be set
        CHECK(pm.status(3) == true);
    }

    SUBCASE("Turn off slot 4")
    {
        pm.on(4);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00010000); // Bit 4 should be cleared
        CHECK(pm.status(4) == true);
    }

    SUBCASE("Turn on slot 5")
    {
        pm.on(5);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00100000); // Bit 5 should be set
        CHECK(pm.status(5) == true);
    }

    SUBCASE("Turn on slot 6")
    {
        pm.on(6);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b01000000); // Bit 6 should be set
        CHECK(pm.status(6) == true);
    }

    SUBCASE("Turn on slot 7")
    {
        pm.on(7);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b10000000); // Bit 7 should be set
        CHECK(pm.status(7) == true);
    }
    SUBCASE("Turn on and off multiple slots")
    {
        pm.on(0);
        pm.on(2);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer()[1] == 0b00000101);
        CHECK(pm.status(0) == true);
        CHECK(pm.status(2) == true);

        pm.off(0);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[1] == 0b00000100);
        CHECK(pm.status(0) == false);
        CHECK(pm.status(2) == true);

        pm.off(2);
        CHECK(get_i2c_buffer()[0] == ps_register);
        CHECK(get_i2c_buffer_count() == 2);
        CHECK(get_i2c_buffer()[1] == 0b00000000);
        CHECK(pm.status(0) == false);
        CHECK(pm.status(2) == false);
    }

    SUBCASE("Invalid slot check")
    {
        CHECK(get_i2c_buffer_count() == 0);       // I2C transactions cancelled
        pm.on(8);                                 // Invalid slot
        CHECK(get_i2c_buffer()[0] == 0b00000000); // register_value_ should still be 0
        CHECK(pm.status(8) == false);
    }

    SUBCASE("Initial status is off")
    {
        CHECK(pm.status(0) == false);
        CHECK(pm.status(1) == false);
        CHECK(pm.status(2) == false);
        CHECK(pm.status(3) == false);
    }

    SUBCASE("Set state with bitmask")
    {
        pm.setState(0b10101010);
        CHECK(pm.status(0) == false);
        CHECK(pm.status(1) == true);
        CHECK(pm.status(2) == false);
        CHECK(pm.status(3) == true);
        CHECK(pm.status(4) == false);
        CHECK(pm.status(5) == true);
        CHECK(pm.status(6) == false);
        CHECK(pm.status(7) == true);
    }

    SUBCASE("Get state reads from OLAT")
    {
        constexpr uint16_t dev_addr = 0x40;
        uint8_t response = 0b11001100;

        inject_i2c_rx_data(dev_addr, &response, 1); // Correct injection

        auto state = pm.getState();
        CHECK(state == 0b11001100);
    }
}