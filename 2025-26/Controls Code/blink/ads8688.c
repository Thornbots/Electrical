/**
 * ADS8688 / ADS8684 SPI ADC driver for Raspberry Pi Pico SDK
 *
 * Converted from Linux kernel driver:
 *   SPDX-License-Identifier: GPL-2.0-only
 *   Copyright (C) 2015 Prevas A/S
 *
 * Pico SDK port: BSD-3-Clause
 */

#include "ads8688.h"
#include "hardware/gpio.h"
#include <string.h>

/* ── Byte-order helpers ──────────────────────────────────────────────── */

static inline uint32_t cpu_to_be32(uint32_t x) {
    return ((x & 0x000000FFu) << 24) |
           ((x & 0x0000FF00u) <<  8) |
           ((x & 0x00FF0000u) >>  8) |
           ((x & 0xFF000000u) >> 24);
}

static inline uint32_t be32_to_cpu(uint32_t x) {
    return cpu_to_be32(x);   /* symmetric */
}

/* ── Range definition table (mirrors Linux ads8688_range_def[]) ───────
 *
 * scale is in nano-Volts per LSB when multiplied by vref_mv.
 * Full-scale voltage = scale × vref_mv × 10⁻⁹ × 65536
 *
 *   ±2.5 V_REF  →  76295 × 4096 = 312,504,320 nV/LSB  ≈ 312.5 µV/LSB
 *   ±1.25 V_REF →  38148 × 4096 = 156,253,608 nV/LSB  ≈ 156.2 µV/LSB
 *   ±0.625 V_REF→  19074 × 4096 =  78,126,804 nV/LSB  ≈  78.1 µV/LSB
 *    0-2.5 V_REF →  38148 × 4096 = 156,253,608 nV/LSB (unipolar)
 *    0-1.25 V_REF →  19074 × 4096 =  78,126,804 nV/LSB (unipolar)
 * ─────────────────────────────────────────────────────────────────────*/
static const ads8688_range_def_t range_table[] = {
    { ADS8688_PLUSMINUS25VREF,    76295, -(1 << 15), ADS8688_REG_PLUSMINUS25VREF   },
    { ADS8688_PLUSMINUS125VREF,   38148, -(1 << 15), ADS8688_REG_PLUSMINUS125VREF  },
    { ADS8688_PLUSMINUS0625VREF,  19074, -(1 << 15), ADS8688_REG_PLUSMINUS0625VREF },
    { ADS8688_PLUS25VREF,         38148,           0, ADS8688_REG_PLUS25VREF        },
    { ADS8688_PLUS125VREF,        19074,           0, ADS8688_REG_PLUS125VREF       },
};
#define RANGE_TABLE_LEN  (sizeof(range_table) / sizeof(range_table[0]))

/* ── Low-level SPI helpers ───────────────────────────────────────────── */

static inline void cs_select(ads8688_t *dev) {
    gpio_put(dev->pin_cs, 0);
}

static inline void cs_deselect(ads8688_t *dev) {
    gpio_put(dev->pin_cs, 1);
}

/**
 * spi_write_be32 - Transmit a big-endian 32-bit word over SPI.
 *
 * The Linux driver stores the word in a __be32 union and then writes
 * d8[0..3] (4 bytes).  We replicate that exactly.
 */
static void spi_write_be32(ads8688_t *dev, uint32_t word_be) {
    uint8_t buf[4];
    buf[0] = (word_be >> 24) & 0xFF;
    buf[1] = (word_be >> 16) & 0xFF;
    buf[2] = (word_be >>  8) & 0xFF;
    buf[3] = (word_be >>  0) & 0xFF;

    cs_select(dev);
    spi_write_blocking(dev->spi, buf, 4);
    cs_deselect(dev);
}

/**
 * spi_write_be32_24 - Transmit only the upper 3 bytes of a big-endian 32-bit word.
 *
 * The Linux prog_write sends &d8[1] with len=3, i.e. bytes [1],[2],[3]
 * of the big-endian 32-bit value (the MSB [0] is skipped).
 */
static void spi_write_be32_24(ads8688_t *dev, uint32_t word_be) {
    uint8_t buf[3];
    buf[0] = (word_be >> 16) & 0xFF;   /* d8[1] */
    buf[1] = (word_be >>  8) & 0xFF;   /* d8[2] */
    buf[2] = (word_be >>  0) & 0xFF;   /* d8[3] */

    cs_select(dev);
    spi_write_blocking(dev->spi, buf, 3);
    cs_deselect(dev);
}

/* ── Device-level register operations ───────────────────────────────── */

/**
 * prog_write - Write to a programming register (3-byte transaction).
 *
 *  Mirrors: ads8688_prog_write() in the Linux driver.
 *
 *  Frame layout (32-bit, big-endian, MSB first):
 *    [31:17] PROG_REG address  (addr << 9 → shifted into top bits)
 *    [16]    Write bit         (1)
 *    [15:8]  Value
 *    [7:0]   Don't-care        (shifted out by PROG_DONT_CARE_BITS=8)
 *
 *  Only bytes [1],[2],[3] of the resulting BE32 are actually sent (24 bits).
 */
static void prog_write(ads8688_t *dev, uint32_t addr, uint32_t val) {
    uint32_t tmp = ADS8688_PROG_REG(addr) | ADS8688_PROG_WR_BIT | val;
    tmp <<= ADS8688_PROG_DONT_CARE_BITS;
    // removed cpu_to_be32
    spi_write_be32_24(dev, tmp);
}

/* ── Public API implementation ───────────────────────────────────────── */

void ads8688_reset(ads8688_t *dev) {
    uint32_t tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_RST);
    tmp <<= ADS8688_CMD_DONT_CARE_BITS;
    // removed cpu_to_be32
    spi_write_be32(dev, tmp);
}

int32_t ads8688_read_raw(ads8688_t *dev, uint8_t channel) {
    /*
     * Mirrors: ads8688_read() in the Linux driver.
     *
     * Two SPI transactions separated by a CS toggle (cs_change=1):
     *
     *   TX1: CMD_REG_MAN_CH(channel) << 16   — select channel for next conv.
     *   TX2: CMD_REG_NOOP             << 16   — clock out result while sending
     *   RX2: 32-bit big-endian result; lower 16 bits are the ADC code.
     * %%%%%%%%% NOT BIG ENDIAN %%%%%%%%%%%%%
     */
    if (channel >= dev->num_channels) return -1;

    uint8_t tx[4], rx[4];
    uint32_t tmp;

    /* ── First transfer: manual channel select ── */
    tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_MAN_CH(channel));
    tmp <<= ADS8688_CMD_DONT_CARE_BITS;
    tx[0] = (tmp >> 24) & 0xFF;
    tx[1] = (tmp >> 16) & 0xFF;
    tx[2] = (tmp >>  8) & 0xFF;
    tx[3] = (tmp >>  0) & 0xFF;

    cs_select(dev);
    spi_write_blocking(dev->spi, tx, 4);
    cs_deselect(dev);   /* cs_change between transfers */

    /* ── Second transfer: NOOP → receive conversion result ── */
    tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_NOOP);
    tmp <<= ADS8688_CMD_DONT_CARE_BITS;
    tx[0] = (tmp >> 24) & 0xFF;
    tx[1] = (tmp >> 16) & 0xFF;
    tx[2] = (tmp >>  8) & 0xFF;
    tx[3] = (tmp >>  0) & 0xFF;

    cs_select(dev);
    spi_write_read_blocking(dev->spi, tx, rx, 4);
    cs_deselect(dev);

    uint32_t result = ((uint32_t)rx[0] << 24) |
                      ((uint32_t)rx[1] << 16) |
                      ((uint32_t)rx[2] <<  8) |
                      ((uint32_t)rx[3]);

    return (int32_t)(result & 0xFFFF);
}

float ads8688_read_voltage_mv(ads8688_t *dev, uint8_t channel) {
    int32_t raw = ads8688_read_raw(dev, channel);
    if (raw < 0) return 0.0f;

    ads8688_range_t r   = dev->range[channel];
    int32_t  offset     = range_table[r].offset;
    uint32_t scale_nano = range_table[r].scale;

    /*
     * Linux IIO formula:
     *   voltage (V) = (raw + offset) × scale_nano × vref_mv × 10⁻⁹
     *
     * We return millivolts:
     *   voltage (mV) = (raw + offset) × scale_nano × vref_mv × 10⁻⁶
     */
    double val = (double)(raw + offset) *
                 (double)scale_nano *
                 (double)dev->vref_mv *
                 1e-6;

    return (float)val;
}

bool ads8688_set_range(ads8688_t *dev, uint8_t channel, ads8688_range_t range) {
    if (channel >= dev->num_channels) return false;

    for (uint32_t i = 0; i < RANGE_TABLE_LEN; i++) {
        if (range_table[i].range == range) {
            uint32_t addr = ADS8688_PROG_REG_RANGE_CH(channel);
            prog_write(dev, addr, range_table[i].reg);
            dev->range[channel] = range;
            return true;
        }
    }
    return false;
}

ads8688_range_t ads8688_get_range(ads8688_t *dev, uint8_t channel) {
    return dev->range[channel];
}

void ads8688_init(ads8688_t *dev,
                  spi_inst_t *spi,
                  uint pin_cs, uint pin_sck,
                  uint pin_mosi, uint pin_miso,
                  uint32_t baudrate,
                  uint8_t num_channels,
                  uint32_t vref_mv) {

    memset(dev, 0, sizeof(*dev));

    dev->spi          = spi;
    dev->pin_cs       = pin_cs;
    dev->pin_sck      = pin_sck;
    dev->pin_mosi     = pin_mosi;
    dev->pin_miso     = pin_miso;
    dev->spi_baudrate = baudrate;
    dev->num_channels = (num_channels <= ADS8688_MAX_CHANNELS)
                            ? num_channels : ADS8688_MAX_CHANNELS;
    dev->vref_mv      = (vref_mv > 0) ? vref_mv : ADS8688_VREF_MV;

    /* Default every channel to ±2.5 × VREF (power-on default) */
    for (int i = 0; i < ADS8688_MAX_CHANNELS; i++)
        dev->range[i] = ADS8688_PLUSMINUS25VREF;

    /* ── SPI peripheral init ──
     * ADS8688 uses SPI Mode 1: CPOL=0, CPHA=1
     * The Linux driver sets spi->mode = SPI_MODE_1 in ads8688_probe().
     */
    spi_init(dev->spi, dev->spi_baudrate);
    spi_set_format(dev->spi,
                   8,            /* bits per transfer */
                   SPI_CPOL_0,   /* CPOL = 0  ┐ SPI Mode 1 */
                   SPI_CPHA_1,   /* CPHA = 1  ┘            */
                   SPI_MSB_FIRST);

    gpio_set_function(pin_sck,  GPIO_FUNC_SPI);
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);

    /* Manual chip-select — active LOW */
    gpio_init(pin_cs);
    gpio_set_dir(pin_cs, GPIO_OUT);
    gpio_put(pin_cs, 1);   /* deassert */

    sleep_ms(1);   /* Allow power rail and SPI to settle */

    ads8688_reset(dev);
    sleep_ms(1);   /* Datasheet: reset pulse must complete before next cmd */
}
