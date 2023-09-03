/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pio_display_spi.h"

// Just 8 bit functions provided here. The PIO program supports any frame size
// 1...32, but the software to do the necessary FIFO shuffling is left as an
// exercise for the reader :)
//
// Likewise we only provide MSB-first here. To do LSB-first, you need to
// - Do shifts when reading from the FIFO, for general case n != 8, 16, 32
// - Do a narrow read at a one halfword or 3 byte offset for n == 16, 8
// in order to get the read data correctly justified. 

#pragma pack(push, 1)
typedef struct {
    uint16_t cmd;
    uint8_t data;
} packet_8bit_data_t;

typedef struct {
    uint16_t cmd;
    uint16_t data;
} packet_16bit_data_t;
#pragma pack(pop)

typedef volatile packet_8bit_data_t io_rw_p8b;
typedef volatile packet_16bit_data_t io_rw_p16b;

void __time_critical_func(pio_spi_write8_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, size_t len) {
    size_t tx_remain = len / 3, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_p8b *txfifo = (io_rw_p8b *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *(io_rw_p8b *)src;
            src+=sizeof(io_rw_p8b);
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            (void) *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_write8_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst,
                                                         size_t len) {
    size_t tx_remain = len / 3, rx_remain = len;
    io_rw_p8b *txfifo = (io_rw_p8b *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *(io_rw_p8b *)src;
            src += sizeof(io_rw_p8b);
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}



void __time_critical_func(pio_spi_write16_blocking)(const pio_spi_inst_t *spi, const uint16_t *src, size_t len) {
    size_t tx_remain = len / 2, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_p16b *txfifo = (io_rw_p16b *) &spi->pio->txf[spi->sm];
    io_rw_16 *rxfifo = (io_rw_16 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *(io_rw_p16b *)src;
            src+=sizeof(io_rw_p16b);
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            (void) *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_read16_blocking)(const pio_spi_inst_t *spi, uint16_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) &spi->pio->txf[spi->sm];
    io_rw_16 *rxfifo = (io_rw_16 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_write16_read16_blocking)(const pio_spi_inst_t *spi, uint16_t *src, uint16_t *dst,
                                                         size_t len) {
    size_t tx_remain = len / 2, rx_remain = len;
    io_rw_p16b *txfifo = (io_rw_p16b *) &spi->pio->txf[spi->sm];
    io_rw_16 *rxfifo = (io_rw_16 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *(io_rw_p16b *)src;
            src += sizeof(io_rw_p16b);
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}