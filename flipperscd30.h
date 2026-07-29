#pragma once

#include <furi_hal_i2c.h>
#include <furi.h>
#include <furi_hal.h>
#include <string>
#include <vector>

#include "flipperbme280.h"

struct SCD30Data {
    float co2_ppm = 0;
    float temperature = 0;
    float humidity = 0;

    // Ambient pressure from an optional BME280, only set if pressure_valid
    float pressure_mbar = 0;
    bool pressure_valid = false;

    std::string ts;

    bool result_valid = false;
};

class FlipperSCD30 {
public:
    FlipperSCD30() = default;
    virtual ~FlipperSCD30() = default;

    bool send_command(std::vector<uint8_t> cmd);
    bool send_command(std::vector<uint8_t> cmd, std::vector<uint8_t> data);
    bool send_command_and_read(std::vector<uint8_t> cmd, uint8_t* result, int len);

    // pressure_mbar of 0 disables the ambient pressure compensation
    bool start_measurement(uint16_t pressure_mbar = 0);
    // Same command as start_measurement, named for the intent of updating the
    // compensation while the sensor is already measuring
    bool set_ambient_pressure(uint16_t pressure_mbar);
    bool set_interval(uint16_t interval);
    bool calibrate(uint16_t calibration);

    SCD30Data read_measurements();
};

class FlipperSCD30WorkerThread {
public:
    explicit FlipperSCD30WorkerThread(int interval);
    virtual ~FlipperSCD30WorkerThread();

    void stop();
    void start();

    void calibrate_to(uint16_t ppm);

    SCD30Data get_data();
    bool has_data();

    int interval;
    uint16_t next_calibration = 0;
    bool running = false;
    bool data_available = false;
    FuriThread* thread;
    FlipperSCD30 scd30;
    SCD30Data last_data;

    // Optional ambient pressure source for the SCD30 compensation
    FlipperBME280 bme280;
    bool bme280_present = false;
    uint16_t last_pressure_mbar = 0;
    uint16_t last_sent_pressure = 0;
    uint32_t last_bme280_read_tick = 0;
    uint32_t last_pressure_sent_tick = 0;
};
