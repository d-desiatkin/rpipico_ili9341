/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_DISPLAY_SPI_H
#define _PIO_DISPLAY_SPI_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "display_spi.pio.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
} pio_spi_inst_t;

// Three functions bellow is for
// 8 bit meaningfull data tx transfers (+ 16 bit of service data) from host
// 8 bit meaningfull data rx transfers (+ 0 bit of service data) to host

void pio_spi_write8_blocking(const pio_spi_inst_t *spi, const uint8_t *src, size_t len);

void pio_spi_read8_blocking(const pio_spi_inst_t *spi, uint8_t *dst, size_t len);

void pio_spi_write8_read8_blocking(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst, size_t len);


// Rest of the functions is for
// 16 bit meaningfull data tx transfers (+ 16 bit of service data)
// 16 bit meaningfull data rx transfers (+ 0 bit of service data) to host

void pio_spi_write16_blocking(const pio_spi_inst_t *spi, const uint16_t *src, size_t len);

void pio_spi_read16_blocking(const pio_spi_inst_t *spi, uint16_t *dst, size_t len);

void pio_spi_write16_read16_blocking(const pio_spi_inst_t *spi, uint16_t *src, uint16_t *dst, size_t len);

#ifdef __cplusplus
}
#endif

#endif // _PIO_DISPLAY_SPI_H
