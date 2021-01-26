/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Show how to reconfigure and restart a channel in a channel completion
// interrupt handler. Plus prepare a DMX-512 buffer for 16 univereses
// before triggering each DMA transfer
//
// Our DMA channel will transfer data to a PIO state machine, which is
// configured to serialise the raw bits that we push, one by one, 16 bits in
// parallel to 16 GPIOs (16 DMX universes).
//
// Once the channel has sent a predetermined amount of data (1 DMX packet), it
// will halt, and raise an interrupt flag. The processor will enter the 
// interrupt handler in response to this, where it will:
// - Toggle GP28 HIGH
// - Zero the complete wave table
// - Prepare the next DMX packet to be sent in the wavetable
// - Sets GP28 LOW (so we can trigger a scope on it)
// - Restarts the DMA channel
// This repeats.

#include <stdio.h>
#include "hardware/clocks.h"    // To derive our 250000bit/s from sys_clk
#include "hardware/dma.h"       // To control the data transfer from mem to pio
#include "hardware/gpio.h"      // To "manually" control the trigger pin
#include "hardware/irq.h"       // To control the data transfer from mem to pio
#include "dmx16.pio.h"          // Header file for the PIO program

int dma_chan;                     // The DMA channel we use to push data around
uint8_t dmx_values[16][512];      // 16 universes with 512 byte each
static uint16_t wavetable[5672];  // 16 universes (data type) with 5672 bit each

// Appends one bit to the wavetable for universe "universe" at the position
// bitoffset. The offset will be increased by 1!
void wavetable_write_bit(int universe, uint16_t* bitoffset, uint8_t value) {
    if (!value) {
        // Since initial value is 0, just increment the offset
        (*bitoffset)++;
        return;
    }

    wavetable[(*bitoffset)++] |= (1 << universe);
};

// Appends one byte (including on start and two stop bits) to the wavetable for
// given universe at the given bit offset. This offset will be increased!
void wavetable_write_byte(int universe, uint16_t* bitoffset, uint8_t value) {
    // Start bit is 0
    wavetable_write_bit(universe, bitoffset, 0);
    // I assume LSB is first? At least it works :)
    wavetable_write_bit(universe, bitoffset, (value >> 0) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 1) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 2) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 3) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 4) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 5) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 6) & 0x01);
    wavetable_write_bit(universe, bitoffset, (value >> 7) & 0x01);

    // Write two stop bits
    wavetable_write_bit(universe, bitoffset, 1);
    wavetable_write_bit(universe, bitoffset, 1);
};

// One transfer has finished, prepare the next DMX packet and restart the
// DMA transfer
void dma_handler() {
    uint8_t universe;   // Loop over the 16 universes
    uint16_t bitoffset; // Current bit offset inside current universe
    uint16_t chan;      // Current channel in universe

    // Drive the TRIGGER GPIO to HIGH
    gpio_put(28, 1);

    // Zero the wavetable
    memset(wavetable, 0x00, 5672*2);

    // Loop through all 16 universes
    for (universe = 0; universe < 16; universe++) {

        // TESTING
        /*
        if (!universe) {
            // TEMPORARY: INcreasing counter on chan 0 of each universe
            dmx_values[universe][509]++;
            // TEMPORARY: DEcreasing counter on chan 1 of each universe
            dmx_values[universe][510]--;
        }
        */

        // The first 24 bit (96µs) are BREAK = LOW level
        // Nothing to do since wavetable has been cleared to 0
        // TODO: This could be shortened since we are on LOW already
        //       and need around 3ms anyways to prepare the wavetable
        bitoffset = 23;

        // Write 4 bit MARK-AFTER-BREAK (16µs)
        wavetable_write_bit(universe, &bitoffset, 1);
        wavetable_write_bit(universe, &bitoffset, 1);
        wavetable_write_bit(universe, &bitoffset, 1);
        wavetable_write_bit(universe, &bitoffset, 1);

        // Write the startbyte
        wavetable_write_byte(universe, &bitoffset, 0);

        // Write the data (channel values) from the buffer
        for (chan = 0; chan < 512; chan++) {
            wavetable_write_byte(universe, &bitoffset, dmx_values[universe][chan]);
        }

        // Go to a defined LOW at the end
        wavetable_write_bit(universe, &bitoffset, 0);
    }

    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan;
    // Give the channel a new wave table entry to read from, and re-trigger it
    dma_channel_set_read_addr(dma_chan, wavetable, true);

    // Drive the TRIGGER GPIO to LOW
    gpio_put(28, 0);
};

int main() {
    stdio_init_all();

    // Set up our TRIGGER GPIO on GP28 and init it to LOW
    gpio_init(28);
    gpio_set_dir(28, GPIO_OUT);
    gpio_put(28, 0);

    // Set up a PIO state machine to serialise our bits at 250000 bit/s
    uint offset = pio_add_program(pio0, &dmx16_program);
    float div = (float)clock_get_hz(clk_sys) / 250000;
    dmx16_program_init(pio0, 0, offset, div);

    // Configure a channel to write the wavetable to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true); // TODO: is by default. Line needed?
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);

    dma_channel_configure(
        dma_chan,
        &c,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        NULL,             // Don't provide a read address yet
        2836,             // Write one complete DMX packet, then halt and interrupt
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Manually call the handler once, to trigger the first transfer
    dma_handler();

    // Everything else from this point is interrupt-driven. The processor has
    // time to sit and think about its early retirement -- maybe open a bakery?
    while (true) {
        tight_loop_contents();
    }
};
