/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"

// We're going to erase and reprogram a region 2044k from the start of flash.
// Once done, we can access this at XIP_BASE + 2044k.
#define FLASH_TARGET_OFFSET (2044 * 1024)

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

int main() {
    // Init the LED's GPIO so we can use it
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    uint8_t data_to_be_written[FLASH_SECTOR_SIZE];

    // Read the target region
    memcpy(data_to_be_written, flash_target_contents, FLASH_SECTOR_SIZE);

    if (data_to_be_written[0] > 10) {
        data_to_be_written[0] = 1;
    }

    // Blink the LED as often as the first byte tells us to
    for (uint8_t i = 0; i < data_to_be_written[0]; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(250);
    }

    // Increase the first byte's value
    data_to_be_written[0] = data_to_be_written[0] + 1;

    // Erase the flash sector
    // Note that a whole number of sectors must be erased at a time.
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);

    // Program the flash sector with the new values
    flash_range_program(FLASH_TARGET_OFFSET, data_to_be_written, FLASH_SECTOR_SIZE);

    // Don't do anything else - wait for a reset
    return 0;
}
