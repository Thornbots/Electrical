#pragma once

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// PIN ASSIGNMENTS — fill in per board before building
// ============================================================

// UART (core 1 — comms with main compute)
#define UART_PIN_TX         24   // UART0 TX TODO
#define UART_PIN_RX         25   // UART0 RX TODO

// SPI0 — drivetrain ADC
#define SPI0_PIN_SCK        2
#define SPI0_PIN_MOSI       3
#define SPI0_PIN_MISO       0
#define SPI0_PIN_CS         1    // GP1 — PWM slice 0B, inverted, CS driven by PWM

// SPI1 — capbank ADC
#define SPI1_PIN_SCK        14
#define SPI1_PIN_MOSI       11
#define SPI1_PIN_MISO       12
#define SPI1_PIN_CS         13   // GP13 — PWM slice 6B, inverted, CS driven by PWM

// Half-bridge PWM (slice determined by pin — fill in GPIOs)
#define HB_PIN_A            20   // TODO — high-side gate drive
#define HB_PIN_B            21   // TODO — low-side gate drive

// ============================================================
// TIMING CONSTANTS
// ============================================================

// Phase-correct PWM: electrical period = 2 * (PWM_TOP + 1) system cycles
// PWM_PERIOD = 1248 cycles @ 125 MHz = 100.16 kHz
// 3 SPI frames tile exactly: 3 * 416 = 1248
#define PWM_PERIOD          1248
#define PWM_TOP             ((PWM_PERIOD / 2) - 1)      // 623

// Symmetric deadtime applied to both HB channels (counter ticks)
#define PWM_DEADTIME        3    // TODO — tune via scope

// Delay in system clock cycles between buck_control_start_spi()
// and buck_control_start_hb().
#define BUCK_DELAY          100  // cycles — tune via scope

// ============================================================
// CS PWM CONSTANTS
// ============================================================

#define CS_PERIOD           416
#define CS_TOP              (CS_PERIOD - 1)              // 415

// CS output is INVERTED so the pin falls at counter wrap (= 0),
// which is the same cycle the PWM DREQ fires.
#define CS_CC               300

// ============================================================
// ADS8684 COMMAND WORDS
// 16-bit command word — upper 16 bits of 32-bit TX word.
// TX is 32-bit: command << 16 | 0x0000 (NOP in lower 16 bits).
// Result comes back in lower 16 bits of 32-bit RX word.
// Format:
//   [15:14] = 11   manual channel select
//   [13:10] = channel index (0-3 for ADS8684)
//   [ 9: 0] = 0x000 reserved
// ============================================================

#define ADS8684_CMD_MANUAL_CH(ch)   ((uint16_t)(0xC000u | ((ch) << 10)))

// Drivetrain ADC channel indices (TODO — fill from schematic)
#define CMD_DT_F1           ADS8684_CMD_MANUAL_CH(2)    // TODO
#define CMD_DT_F2           ADS8684_CMD_MANUAL_CH(2)    // TODO
#define CMD_DT_F3           ADS8684_CMD_MANUAL_CH(2)    // TODO

// Capbank ADC channel indices (TODO — fill from schematic)
#define CMD_CAP_F1          ADS8684_CMD_MANUAL_CH(2)    // TODO
#define CMD_CAP_F2          ADS8684_CMD_MANUAL_CH(2)    // TODO
#define CMD_CAP_F3          ADS8684_CMD_MANUAL_CH(2)    // TODO

// ============================================================
// ADC CHANNEL ENCODING for adc_read()
// bits[7:4] = SPI peripheral (0x0 = spi0, 0x1 = spi1)
// bits[3:0] = ADC channel index (0-3 for ADS8684)
// ============================================================

#define ADC_CH(spi, ch)     ((uint8_t)(((spi) << 4) | ((ch) & 0x0F)))

// ============================================================
// COMMAND + RESULT BUFFERS
// TX: 32-bit word, command in upper 16 bits, NOP in lower 16 bits.
// RX: 32-bit word, result in lower 16 bits (upper 16 bits = zeros).
// i1 is always written/read before i2.
// ============================================================

// SPI0 — drivetrain: TX command slots (written by CPU each period)
extern volatile uint32_t adc0_tx_f1;
extern volatile uint32_t adc0_tx_f2;
extern volatile uint32_t adc0_tx_f3;

// SPI0 — drivetrain: RX result slots (written by DMA, read by CPU)
// Result is in lower 16 bits: (uint16_t)(adc0_rx_f1 & 0xFFFF)
extern volatile uint32_t adc0_rx_f1;
extern volatile uint32_t adc0_rx_f2;
extern volatile uint32_t adc0_rx_f3;

// SPI1 — capbank: TX command slots
extern volatile uint32_t adc1_tx_f1;
extern volatile uint32_t adc1_tx_f2;
extern volatile uint32_t adc1_tx_f3;

// SPI1 — capbank: RX result slots
extern volatile uint32_t adc1_rx_f1;
extern volatile uint32_t adc1_rx_f2;
extern volatile uint32_t adc1_rx_f3;

// CS PWM slice numbers — exposed so main can poll the counter
// for BUCK_DELAY between buck_control_start_spi() and buck_control_start_hb()
extern uint cs_pwm_slice_spi0;
extern uint cs_pwm_slice_spi1;

// ============================================================
// PUBLIC API
// ============================================================

// Configure all SPI, PWM, and DMA hardware.
// CS pins left as plain GPIO (deasserted high) after this call.
// Nothing starts until buck_control_start_spi() is called.
void buck_control_init(void);

// Transfer CS pins to PWM function, start CS-PWM slices and arm
// kickoff DMA. SPI frames begin on the first CS DREQ.
// Call buck_control_start_hb() separately after BUCK_DELAY cycles.
void buck_control_start_spi(void);

// Enable HB-PWM slice. Phase-locked to CS-PWM permanently from
// this point. Call after counter-poll delay in main:
//   while (pwm_get_counter(cs_pwm_slice_spi0) < BUCK_DELAY);
//   buck_control_start_hb();
void buck_control_start_hb(void);

// Graceful stop: waits for HB-PWM counter to complete current period,
// disables all PWM slices, aborts DMA, returns CS pins to plain GPIO
// (deasserted high).
void buck_control_stop(void);

// Manual single-channel ADC read (blocking, 2-frame pipeline).
// Uses plain GPIO CS — call only before buck_control_start_spi()
// or after buck_control_stop().
// ch: ADC_CH(spi, channel) — e.g. ADC_CH(0, 2) = spi0 channel 2
uint16_t adc_read(uint8_t ch);

// tells all ADCs to configure all channels to measure in the range of 0-5.12 instead of +-10.24
// allows for 4x more precise readings of the voltage and current sensors
void adc_set_range(void);

#ifdef __cplusplus
}
#endif