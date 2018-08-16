#include "eeprom.h"

void init_i2c_master() {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t eeprom_write_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, eeaddress>>8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);
    i2c_master_write_byte(cmd, byte, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t eeprom_write(uint8_t deviceaddress, uint16_t eeaddress, uint8_t* data, size_t size) {
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, eeaddress>>8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);

    int bytes_remaining = size;
    int current_address = eeaddress;
    int first_write_size = ((EEPROM_PAGE_SIZE-1) - eeaddress % (EEPROM_PAGE_SIZE-1))+1;
    if (eeaddress % (EEPROM_PAGE_SIZE-1) == 0 && eeaddress != 0) first_write_size = 1;
    if (bytes_remaining <= first_write_size) {
        i2c_master_write(cmd, data, bytes_remaining, 1);
    } else {
        i2c_master_write(cmd, data, first_write_size, 1);
        bytes_remaining -= first_write_size;
        current_address += first_write_size;
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) return ret;
        while (bytes_remaining > 0) {
            cmd = i2c_cmd_link_create();

            // 2ms delay period to allow EEPROM to write the page
            // buffer to memory.
            vTaskDelay(20/portTICK_PERIOD_MS);

            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, 1);
            i2c_master_write_byte(cmd, current_address>>8, 1);
            i2c_master_write_byte(cmd, current_address&0xFF, 1);
            if (bytes_remaining <= EEPROM_PAGE_SIZE) {
                i2c_master_write(cmd, data+(size-bytes_remaining), bytes_remaining, 1);
                bytes_remaining = 0;
            } else {
                i2c_master_write(cmd, data+(size-bytes_remaining), EEPROM_PAGE_SIZE, 1);
                bytes_remaining -= EEPROM_PAGE_SIZE;
                current_address += EEPROM_PAGE_SIZE;
            }
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            if (ret != ESP_OK) return ret;
        }
    }

    return ret;
}

uint8_t eeprom_read_byte(uint8_t deviceaddress, uint16_t eeaddress) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, eeaddress<<8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_READ_ADDR, 1);

    uint8_t data;
    i2c_master_read_byte(cmd, &data, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

esp_err_t eeprom_read(uint8_t deviceaddress, uint16_t eeaddress, uint8_t* data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, eeaddress<<8, 1);
    i2c_master_write_byte(cmd, eeaddress&0xFF, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress<<1)|EEPROM_READ_ADDR, 1);

    if (size > 1) {
        i2c_master_read(cmd, data, size-1, 0);
    }
    i2c_master_read_byte(cmd, data+size-1, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}