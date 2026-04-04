#ifndef ADS8688_H
#define ADS8688_H

/**
 * ADS8688 / ADS8684 SPI ADC driver for Raspberry Pi Pico SDK
 *
 * Converted from Linux kernel driver (GPL-2.0-only, Prevas A/S / Sean Nyekjaer)
 * Pico SDK port: BSD-3-Clause
 *
 * Wiring (default pins — change in ads8688_init if needed):
 *   MOSI → GP19  (SPI0 TX)
 *   MISO → GP16  (SPI0 RX)
 *   SCK  → GP18  (SPI0 SCK)
 *   CS   → GP17  (manual GPIO)
 */

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Register / command encoding ─────────────────────────────────────── */
#define ADS8688_CMD_REG(x)           ((x) << 8)
#define ADS8688_CMD_REG_NOOP         0x00
#define ADS8688_CMD_REG_RST          0x85
#define ADS8688_CMD_REG_MAN_CH(ch)   (0xC0 | (4 * (ch)))
#define ADS8688_CMD_DONT_CARE_BITS   16

#define ADS8688_PROG_REG(x)          ((x) << 9)
#define ADS8688_PROG_REG_RANGE_CH(ch)(0x05 + (ch))
#define ADS8688_PROG_WR_BIT          (1u << 8)
#define ADS8688_PROG_DONT_CARE_BITS  8

/* Range register values */
#define ADS8688_REG_PLUSMINUS25VREF   0
#define ADS8688_REG_PLUSMINUS125VREF  1
#define ADS8688_REG_PLUSMINUS0625VREF 2
#define ADS8688_REG_PLUS25VREF        5
#define ADS8688_REG_PLUS125VREF       6

#define ADS8688_VREF_MV   4096   /* Internal reference (mV) */
#define ADS8688_MAX_CHANNELS 8

/* ── Voltage range enum ───────────────────────────────────────────────── */
typedef enum {
    ADS8688_PLUSMINUS25VREF,    /* ±2.5 × VREF  */
    ADS8688_PLUSMINUS125VREF,   /* ±1.25 × VREF */
    ADS8688_PLUSMINUS0625VREF,  /* ±0.625 × VREF */
    ADS8688_PLUS25VREF,         /* 0 – 2.5 × VREF */
    ADS8688_PLUS125VREF,        /* 0 – 1.25 × VREF */
} ads8688_range_t;

/* ── Per-range scale / offset table entry ────────────────────────────── */
typedef struct {
    ads8688_range_t range;
    uint32_t        scale;   /* nano-Volts per LSB divisor (×10⁻⁹ V/LSB) */
    int32_t         offset;  /* raw offset applied to reading             */
    uint8_t         reg;     /* value written to the range register       */
} ads8688_range_def_t;

/* ── Device handle ────────────────────────────────────────────────────── */
typedef struct {
    spi_inst_t     *spi;
    uint            pin_cs;
    uint            pin_sck;
    uint            pin_mosi;
    uint            pin_miso;
    uint32_t        spi_baudrate;
    uint32_t        vref_mv;
    uint8_t         num_channels;        /* 4 (ADS8684) or 8 (ADS8688) */
    ads8688_range_t range[ADS8688_MAX_CHANNELS];
} ads8688_t;

/* ── Public API ──────────────────────────────────────────────────────── */

/**
 * ads8688_init - Initialise SPI and the ADS8688/ADS8684.
 *
 * @dev           Pointer to an ads8688_t struct to populate.
 * @spi           SPI instance (spi0 or spi1).
 * @pin_cs        GPIO for manual chip-select.
 * @pin_sck       GPIO for SPI clock.
 * @pin_mosi      GPIO for MOSI.
 * @pin_miso      GPIO for MISO.
 * @baudrate      SPI clock rate in Hz (≤17 MHz per datasheet).
 * @num_channels  4 for ADS8684, 8 for ADS8688.
 * @vref_mv       External VREF in mV, or 0 to use internal 4096 mV reference.
 */
void ads8688_init(ads8688_t *dev,
                  spi_inst_t *spi,
                  uint pin_cs, uint pin_sck,
                  uint pin_mosi, uint pin_miso,
                  uint32_t baudrate,
                  uint8_t num_channels,
                  uint32_t vref_mv);

/** ads8688_reset - Send a full device reset command. */
void ads8688_reset(ads8688_t *dev);

/**
 * ads8688_read_raw - Read a single channel (returns raw 16-bit unsigned value).
 * Returns -1 on error.
 */
int32_t ads8688_read_raw(ads8688_t *dev, uint8_t channel);

/**
 * ads8688_read_voltage_mv - Convert a raw reading to millivolts.
 *
 * Takes the current range setting for that channel into account.
 */
float ads8688_read_voltage_mv(ads8688_t *dev, uint8_t channel);

/**
 * ads8688_set_range - Set the input voltage range for one channel.
 *
 * Returns true on success.
 */
bool ads8688_set_range(ads8688_t *dev, uint8_t channel, ads8688_range_t range);

/**
 * ads8688_get_range - Return the current range for a channel.
 */
ads8688_range_t ads8688_get_range(ads8688_t *dev, uint8_t channel);

#endif /* ADS8688_H */
