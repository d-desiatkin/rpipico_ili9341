#ifndef _PIO_LOGIC_ANALYSER_H
#define _PIO_LOGIC_ANALYSER_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"


const uint CAPTURE_PIN_BASE = 5;
const uint CAPTURE_PIN_COUNT = 8;
const uint CAPTURE_N_SAMPLES = 96;

#ifdef __cplusplus
extern "C" {
#endif

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}


void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div);

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level);

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples);

#ifdef __cplusplus
}
#endif

#endif // _PIO_LOGIC_ANALYSER_H