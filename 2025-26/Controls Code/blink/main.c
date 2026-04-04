/**
 * ADS8688 / ADS8684 — Pico SDK usage example
 *
 * Reads all channels once per second and prints the raw ADC code
 * and the computed voltage (mV) over USB serial (UART0).
 *
 * Default wiring:
 *   GP16 → MISO   (SPI0 RX)
 *   GP17 → CS     (manual GPIO)
 *   GP18 → SCK    (SPI0 SCK)
 *   GP19 → MOSI   (SPI0 TX)
 *
 * Our Pinout:
 *  GP11 → MOSI    (SPI1 TX)
 *  GP12 → MISO    (SPI1 RX)
 *  GP13 → CSn     (SPI1 CS)
 *  <GND pin in between>
 *  GP14 → SCK     (SPI1 SCK)
 * 
 * CMakeLists.txt dependencies:
 *   target_link_libraries(your_target pico_stdlib hardware_spi)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ads8688.h"

/* ── Pin assignment ──────────────────────────────────────────────────── */
#define PIN_MISO   12
#define PIN_CS     13
#define PIN_SCK    14
#define PIN_MOSI   11

/* ── ADS8688 = 8 channels, ADS8684 = 4 channels ─────────────────────── */
#define NUM_CHANNELS  4        /* Change to 4 for ADS8684 */

/* ── SPI clock: max 17 MHz per ADS8688 datasheet ────────────────────── */
#define SPI_BAUDRATE  (1000 * 1000)   /* 1 MHz — safe default */

int main(void) {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(100);

     // 1. Init the LED pin
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    sleep_ms(2000);   /* Wait for USB serial to enumerate */

    printf("ADS8688 Pico SDK driver example\n");

    /* Initialise driver — 0 for vref_mv uses internal 4096 mV reference */
    ads8688_t adc;
    ads8688_init(&adc,
                 spi1,
                 PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO,
                 SPI_BAUDRATE,
                 NUM_CHANNELS,
                 0 /* internal VREF = 4096 mV */);

    /*
     * Optional: override the input range on specific channels.
     *
     * Power-on default is ADS8688_PLUSMINUS25VREF (±2.5 × VREF = ±10.24 V
     * with the internal 4096 mV reference).
     *
     * Example — set channel 0 to unipolar 0-2.5×VREF (0 – 10.24 V):
     *   ads8688_set_range(&adc, 0, ADS8688_PLUS25VREF);
     *
     * Example — set channel 1 to ±1.25×VREF (±5.12 V):
     *   ads8688_set_range(&adc, 1, ADS8688_PLUSMINUS125VREF);
     */

    printf("Sampling every 1 s.  Press Ctrl-C to stop.\n\n");

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            int32_t raw = ads8688_read_raw(&adc, ch);
            float   mv  = ads8688_read_voltage_mv(&adc, ch);
            printf("CH%d: raw=%5d  voltage=%+9.3f mV\n", ch, raw, mv);
        }
        printf("──────────────────────────────────\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(1000);
    }

    return 0;
}