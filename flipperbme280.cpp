#include "flipperbme280.h"

#define BME280_ADDR_PRIMARY   (0x76 << 1)
#define BME280_ADDR_SECONDARY (0x77 << 1)

#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60

#define REG_CHIP_ID   0xD0
#define REG_RESET     0xE0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG    0xF5
#define REG_PRESS_MSB 0xF7
#define REG_TEMP_CAL  0x88
#define REG_PRESS_CAL 0x8E

#define RESET_MAGIC 0xB6

// Temperature oversampling x2, pressure oversampling x4, normal mode
#define CTRL_MEAS_VALUE 0x4F
// Standby 1000ms, IIR filter 16, 3 wire SPI off. Ambient pressure changes
// slowly, so the heavy filtering is exactly what we want here.
#define CONFIG_VALUE    0xB0

#define I2C_TIMEOUT 10

// SCD30 accepts 700..1400 mBar, anything outside that is a bad reading
#define PRESSURE_MIN_MBAR 700
#define PRESSURE_MAX_MBAR 1400

static bool read_reg(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    furi_hal_i2c_acquire(&furi_hal_i2c_handle_external);
    bool result =
        furi_hal_i2c_read_mem(&furi_hal_i2c_handle_external, addr, reg, data, len, I2C_TIMEOUT);
    furi_hal_i2c_release(&furi_hal_i2c_handle_external);
    return result;
}

static bool write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    furi_hal_i2c_acquire(&furi_hal_i2c_handle_external);
    bool result =
        furi_hal_i2c_write_mem(&furi_hal_i2c_handle_external, addr, reg, &value, 1, I2C_TIMEOUT);
    furi_hal_i2c_release(&furi_hal_i2c_handle_external);
    return result;
}

static bool is_ready(uint8_t addr) {
    furi_hal_i2c_acquire(&furi_hal_i2c_handle_external);
    bool result = furi_hal_i2c_is_device_ready(&furi_hal_i2c_handle_external, addr, I2C_TIMEOUT);
    furi_hal_i2c_release(&furi_hal_i2c_handle_external);
    return result;
}

static inline uint16_t u16_le(const uint8_t* buffer, int offset) {
    return static_cast<uint16_t>(buffer[offset] | (buffer[offset + 1] << 8));
}

static inline int16_t s16_le(const uint8_t* buffer, int offset) {
    return static_cast<int16_t>(u16_le(buffer, offset));
}

// 20 bit ADC value spread over three registers
static inline int32_t adc20(const uint8_t* buffer, int offset) {
    return (static_cast<int32_t>(buffer[offset]) << 12) |
           (static_cast<int32_t>(buffer[offset + 1]) << 4) |
           (static_cast<int32_t>(buffer[offset + 2]) >> 4);
}

bool FlipperBME280::probe(uint8_t address) {
    if(!is_ready(address)) {
        return false;
    }

    // Start from a known state. The chip needs a moment before it answers again.
    write_reg(address, REG_RESET, RESET_MAGIC);
    furi_delay_ms(5);

    uint8_t chip_id = 0;
    if(!read_reg(address, REG_CHIP_ID, &chip_id, 1)) {
        return false;
    }

    return chip_id == BMP280_CHIP_ID || chip_id == BME280_CHIP_ID;
}

bool FlipperBME280::init() {
    present = false;

    const uint8_t addresses[] = {BME280_ADDR_PRIMARY, BME280_ADDR_SECONDARY};
    for(uint32_t i = 0; i < COUNT_OF(addresses); i++) {
        if(probe(addresses[i])) {
            addr = addresses[i];
            present = true;
            break;
        }
    }

    if(!present) {
        return false;
    }

    // ctrl_hum (0xF2) is deliberately left alone: we never read humidity, and
    // on the BMP280 that register is reserved.
    if(!write_reg(addr, REG_CTRL_MEAS, CTRL_MEAS_VALUE) ||
       !write_reg(addr, REG_CONFIG, CONFIG_VALUE) || !read_calibration()) {
        present = false;
        return false;
    }

    return true;
}

bool FlipperBME280::read_calibration() {
    uint8_t buffer[18];

    if(!read_reg(addr, REG_TEMP_CAL, buffer, 6)) {
        return false;
    }
    dig_T1 = u16_le(buffer, 0);
    dig_T2 = s16_le(buffer, 2);
    dig_T3 = s16_le(buffer, 4);

    if(!read_reg(addr, REG_PRESS_CAL, buffer, 18)) {
        return false;
    }
    dig_P1 = u16_le(buffer, 0);
    dig_P2 = s16_le(buffer, 2);
    dig_P3 = s16_le(buffer, 4);
    dig_P4 = s16_le(buffer, 6);
    dig_P5 = s16_le(buffer, 8);
    dig_P6 = s16_le(buffer, 10);
    dig_P7 = s16_le(buffer, 12);
    dig_P8 = s16_le(buffer, 14);
    dig_P9 = s16_le(buffer, 16);

    return true;
}

// Compensation formulas from the Bosch BME280 datasheet, same integer variant
// as used by applications/external/unitemp/sensors/BMx280.c
float FlipperBME280::compensate_temperature(int32_t adc_T) {
    int32_t var1, var2;

    var1 =
        ((((adc_T >> 3) - (static_cast<int32_t>(dig_T1) << 1))) * static_cast<int32_t>(dig_T2)) >>
        11;
    var2 = (((((adc_T >> 4) - static_cast<int32_t>(dig_T1)) *
              ((adc_T >> 4) - static_cast<int32_t>(dig_T1))) >>
             12) *
            static_cast<int32_t>(dig_T3)) >>
           14;

    t_fine = var1 + var2;
    return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

// Returns pascal
float FlipperBME280::compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;

    var1 = (t_fine >> 1) - static_cast<int32_t>(64000);
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * static_cast<int32_t>(dig_P6);
    var2 = var2 + ((var1 * static_cast<int32_t>(dig_P5)) << 1);
    var2 = (var2 >> 2) + (static_cast<int32_t>(dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((static_cast<int32_t>(dig_P2) * var1) >> 1)) >>
           18;
    var1 = (((32768 + var1)) * static_cast<int32_t>(dig_P1)) >> 15;

    if(var1 == 0) {
        return 0; // avoid division by zero
    }

    p = (static_cast<uint32_t>((static_cast<int32_t>(1048576) - adc_P) - (var2 >> 12))) * 3125;
    if(p < 0x80000000) {
        p = (p << 1) / static_cast<uint32_t>(var1);
    } else {
        p = (p / static_cast<uint32_t>(var1)) * 2;
    }

    var1 = (static_cast<int32_t>(dig_P9) * static_cast<int32_t>(((p >> 3) * (p >> 3)) >> 13)) >>
           12;
    var2 = (static_cast<int32_t>(p >> 2) * static_cast<int32_t>(dig_P8)) >> 13;
    p = static_cast<uint32_t>(static_cast<int32_t>(p) + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

bool FlipperBME280::read_pressure_mbar(uint16_t* result) {
    if(!present) {
        return false;
    }

    // 0xF7..0xFC in one burst: pressure first, then temperature
    uint8_t buffer[6];
    if(!read_reg(addr, REG_PRESS_MSB, buffer, sizeof(buffer))) {
        return false;
    }

    // Temperature has to be compensated first, it sets t_fine
    compensate_temperature(adc20(buffer, 3));
    float pascal = compensate_pressure(adc20(buffer, 0));

    uint32_t mbar = static_cast<uint32_t>((pascal + 50.0f) / 100.0f);
    if(mbar < PRESSURE_MIN_MBAR || mbar > PRESSURE_MAX_MBAR) {
        return false;
    }

    *result = static_cast<uint16_t>(mbar);
    return true;
}
