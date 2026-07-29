# Flipper Zero SCD30 CO2 Monitor

![Screenshot](docs/screenshot.jpg)

CO2 monitor and data logger for Flipper Zero. This started as a port of https://github.com/thzinc/flipperzero-firmware to a recent version and evolved to a completetly new project.

## Wiring
- SCD30 VIN <--> 3.3V
- SCD30 GND <--> GND
- SCD30 SCL <--> PC0
- SCD30 SDA <--> PC1

### Optional: BME280 / BMP280

A BME280 (or BMP280) can be attached to the same I2C bus to compensate the CO2
reading for the current ambient pressure. It is detected automatically at
address 0x76 or 0x77, no configuration needed.

- BME280 VIN <--> 3.3V
- BME280 GND <--> GND
- BME280 SCL <--> PC0
- BME280 SDA <--> PC1

## Features

- Displays CO2, Temperature, Humidity
- Automatic ambient pressure compensation if a BME280/BMP280 is connected
- Continous logging to a CSV file
- LED Color based on current CO2 level (500 = green ... 2500 = red)
- Bar graph (0 ... 3000ppm)
- Calibration

## CSV format

`timestamp,co2_ppm,temperature,humidity,pressure_hpa`

The pressure column is empty when no BME280/BMP280 is connected.

## Calibrate sensor

Place the sensor outside for at least 5 minutes - push and hold the "UP" button. The CO2 sensor will calibrate to 420 ppm.