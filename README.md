# I2C EEPROM driver for the ESP32 microcontroller
Driver for reading and writing data to external I2C EEPROMs.  Written in pure C (For use with ESP-IDF)

### Attention
I2C initialization settings may have to be changed in the `init_i2c_master` function located in the `eeprom.c` file.
Most compiler definitions are located in `include/eeprom.h`.

This driver was written and tested with specifically with the 24LC256 EEPROM, other EEPROM types are not guaranteed to work.
