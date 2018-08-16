#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "eeprom.h"

void eeprom_task(void *arg) {
    const uint8_t eeprom_address = 0x50;
    const uint16_t starting_address = 0x0000;

    // EEPROM single byte write example
    uint8_t single_write_byte = 0x40;
    eeprom_write_byte(eeprom_address, starting_address, 0x40);
    printf("Wrote byte 0x%02X to address 0x%04X\n", single_write_byte, starting_address);
    vTaskDelay(20/portTICK_PERIOD_MS);

    // EEPROM random read example
    uint8_t random_read_byte = eeprom_read_byte(eeprom_address, starting_address);
    printf("Read byte 0x%02X at address 0x%04X\n", random_read_byte, starting_address);
    vTaskDelay(20/portTICK_PERIOD_MS);

    // EEPROM page write example
    char* page_write_data = "EEPROM page writing allows long strings of text to be written quickly, and with a lot less write cycles!\0";
    eeprom_write(eeprom_address, starting_address, (uint8_t*)page_write_data, strlen(page_write_data));
    printf("Wrote the following string to EEPROM: %s\n", page_write_data);

    vTaskDelay(20/portTICK_PERIOD_MS);

    // EEPROM sequential read example and error checking
    uint8_t* sequential_read_data = (uint8_t*) malloc(strlen(page_write_data)+1);
    esp_err_t ret = eeprom_read(eeprom_address, starting_address, sequential_read_data, strlen(page_write_data)+1);

    if (ret == ESP_ERR_TIMEOUT) printf("I2C timeout...\n");
    if (ret == ESP_OK) printf("The read operation was successful!\n");
    else printf("The read operation was not successful, no ACK recieved.  Is the device connected properly?\n");

    printf("Read the following string from EEPROM: %s\n", (char*)sequential_read_data);

    free(sequential_read_data);

    // We're done with this task now.  Deallocate so the computer doesn't complain
    vTaskDelete(NULL);
}

// Run on APP CPU of ESP32 microcontroller
void app_main() {
    init_i2c_master();
    xTaskCreate(&eeprom_task, "eeprom_read_write_demo", 1024 * 2, NULL, 5, NULL);
}