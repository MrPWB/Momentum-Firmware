#pragma once

#include <furi_hal_i2c.h>
#include <furi.h>
#include <furi_hal.h>

// Minimal BME280 / BMP280 driver, pressure only.
// Used to feed the SCD30 with the current ambient pressure for CO2 compensation.
class FlipperBME280 {
public:
    FlipperBME280() = default;
    virtual ~FlipperBME280() = default;

    // Probes 0x76 and 0x77, verifies the chip id, configures the sensor and
    // reads the factory calibration. Returns false if no sensor was found.
    bool init();

    bool is_present() {
        return present;
    }

    // Reads a compensated pressure and rounds it to mbar (= hPa).
    // Returns false if the sensor is absent, the transfer failed or the value
    // is outside the range the SCD30 accepts.
    bool read_pressure_mbar(uint16_t* result);

private:
    // Already shifted 8 bit address, as expected by furi_hal_i2c
    uint8_t addr = 0;
    bool present = false;

    // Set by the temperature compensation, needed by the pressure compensation
    int32_t t_fine = 0;

    uint16_t dig_T1 = 0;
    int16_t dig_T2 = 0;
    int16_t dig_T3 = 0;

    uint16_t dig_P1 = 0;
    int16_t dig_P2 = 0;
    int16_t dig_P3 = 0;
    int16_t dig_P4 = 0;
    int16_t dig_P5 = 0;
    int16_t dig_P6 = 0;
    int16_t dig_P7 = 0;
    int16_t dig_P8 = 0;
    int16_t dig_P9 = 0;

    bool probe(uint8_t address);
    bool read_calibration();

    float compensate_temperature(int32_t adc_T);
    float compensate_pressure(int32_t adc_P);
};
