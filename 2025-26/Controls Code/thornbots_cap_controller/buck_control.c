#include "buck_control.h"

#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

// ============================================================
// COMMAND + RESULT BUFFERS
// TX: 32-bit word, command in upper 16 bits, NOP (0x0000) in lower 16 bits.
// RX: 32-bit word, result in lower 16 bits.
// Extract result: (uint16_t)(adc0_rx_f1 & 0xFFFF)
// ============================================================

volatile uint32_t adc0_tx_f1 = (uint32_t)CMD_DT_F1;
volatile uint32_t adc0_tx_f2 = (uint32_t)CMD_DT_F2;
volatile uint32_t adc0_tx_f3 = (uint32_t)CMD_DT_F3;
 
volatile uint32_t adc0_rx_f1 = 0;
volatile uint32_t adc0_rx_f2 = 0;
volatile uint32_t adc0_rx_f3 = 0;
 
volatile uint32_t adc1_tx_f1 = (uint32_t)CMD_CAP_F1;
volatile uint32_t adc1_tx_f2 = (uint32_t)CMD_CAP_F2;
volatile uint32_t adc1_tx_f3 = (uint32_t)CMD_CAP_F3;
 
volatile uint32_t adc1_rx_f1 = 0;
volatile uint32_t adc1_rx_f2 = 0;
volatile uint32_t adc1_rx_f3 = 0;

// ============================================================
// PWM SLICE HANDLES (exported for counter-poll in main)
// ============================================================

uint cs_pwm_slice_spi0;
uint cs_pwm_slice_spi1;
static uint hb_pwm_slice;

static uint32_t cs_pwm_mask;
static uint32_t hb_pwm_mask;

// ============================================================
// DMA CHANNEL HANDLES
// ============================================================

// Two kickoff channels — one per SPI, chained together.
// kickoff_a wakes on CS-PWM DREQ, triggers spi0_tx, chains to kickoff_b.
// kickoff_b fires immediately after, triggers spi1_tx, chains to kickoff_a.
static int dma_kickoff_a;
static int dma_kickoff_b;

static int dma_spi0_tx;
static int dma_spi0_rx;
static int dma_spi1_tx;
static int dma_spi1_rx;

// ============================================================
// FRAME POINTER TABLES
// ISR walks these each frame to reload TX source / RX dest.
// ============================================================

static uint32_t * const spi0_tx_frames[3] = {
    (uint32_t *)&adc0_tx_f1,
    (uint32_t *)&adc0_tx_f2,
    (uint32_t *)&adc0_tx_f3,
};
static uint32_t * const spi0_rx_frames[3] = {
    (uint32_t *)&adc0_rx_f1,
    (uint32_t *)&adc0_rx_f2,
    (uint32_t *)&adc0_rx_f3,
};
static uint32_t * const spi1_tx_frames[3] = {
    (uint32_t *)&adc1_tx_f1,
    (uint32_t *)&adc1_tx_f2,
    (uint32_t *)&adc1_tx_f3,
};
static uint32_t * const spi1_rx_frames[3] = {
    (uint32_t *)&adc1_rx_f1,
    (uint32_t *)&adc1_rx_f2,
    (uint32_t *)&adc1_rx_f3,
};

// ============================================================
// FRAME INDEX
// Incremented by ISR each time a frame completes.
// Wraps 0 -> 1 -> 2 -> 0.
// ============================================================

volatile uint frame_index = 0;

// ============================================================
// INTERNAL: SPI SETUP
// 16-bit SPI mode. DMA sends 2 x 16-bit transfers per frame:
//   transfer 1: lower 16 bits of tx word = command (sent first)
//   transfer 2: upper 16 bits of tx word = NOP 0x0000 (sent second)
// Result received:
//   transfer 1 RX: zeros (lower 16 bits of rx word)
//   transfer 2 RX: ADC result (upper 16 bits of rx word)
// ============================================================

static void setup_spi(spi_inst_t *spi,
                      uint pin_sck, uint pin_mosi, uint pin_miso)
{
    // 15.625 MHz SCLK: 125 MHz / (CPSDVSR=8, SCR=0)
    // Mode 1: CPOL=0, CPHA=1    
    // 16-bit transfers — DMA sends 2 per CS period for full 32-clock frame

    spi_init(spi, 15625000);
    spi_set_format(spi, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(pin_sck,  GPIO_FUNC_SPI);
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);
    // CS pin configured separately as GPIO in buck_control_init(),
    // transferred to PWM in buck_control_start_spi()
}

// ============================================================
// INTERNAL: CS PWM SETUP
// Inverted output: pin falls at counter wrap (cycle 0) = same
// cycle as DREQ fires. Pin rises at cycle CS_CC.
// CS low  duration = CS_CC cycles
// CS high duration = CS_TOP + 1 - CS_CC cycles
// ============================================================

static void setup_cs_pwms(void)
{
    pwm_config cfg0 = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg0, 1);
    pwm_config_set_wrap(&cfg0, CS_TOP);

    uint slice0 = pwm_gpio_to_slice_num(SPI0_PIN_CS);
    uint chan0   = pwm_gpio_to_channel(SPI0_PIN_CS);
    cs_pwm_slice_spi0 = slice0;
    pwm_config_set_output_polarity(&cfg0,
        chan0 == PWM_CHAN_A,
        chan0 == PWM_CHAN_B);
    pwm_init(slice0, &cfg0, false);
    pwm_set_chan_level(slice0, chan0, CS_CC);

    pwm_config cfg1 = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg1, 1);
    pwm_config_set_wrap(&cfg1, CS_TOP);

    uint slice1 = pwm_gpio_to_slice_num(SPI1_PIN_CS);
    uint chan1   = pwm_gpio_to_channel(SPI1_PIN_CS);
    cs_pwm_slice_spi1 = slice1;
    pwm_config_set_output_polarity(&cfg1,
        chan1 == PWM_CHAN_A,
        chan1 == PWM_CHAN_B);
    pwm_init(slice1, &cfg1, false);
    pwm_set_chan_level(slice1, chan1, CS_CC);
}

// ============================================================
// INTERNAL: HB PWM SETUP
// Phase-correct, period = PWM_PERIOD, not started here.
// ============================================================

static void setup_hb_pwm(void)
{
    hb_pwm_slice = pwm_gpio_to_slice_num(HB_PIN_A);

    gpio_set_function(HB_PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(HB_PIN_B, GPIO_FUNC_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, PWM_TOP);
    pwm_config_set_phase_correct(&cfg, true);
    pwm_config_set_output_polarity(&cfg, false, true);  // B_INV=1
    pwm_init(hb_pwm_slice, &cfg, false);
}

// ============================================================
// INTERNAL: DMA SETUP
// Kickoff channels trigger SPI TX channels once per CS period.
// Each SPI TX/RX transfers 1 x 32-bit word per frame.
// ============================================================

static const uint32_t tx_trigger_word = 2u;

static void setup_dma(void)
{
    dma_kickoff_a = dma_claim_unused_channel(true);
    dma_kickoff_b = dma_claim_unused_channel(true);
    dma_spi0_tx   = dma_claim_unused_channel(true);
    dma_spi0_rx   = dma_claim_unused_channel(true);
    dma_spi1_tx   = dma_claim_unused_channel(true);
    dma_spi1_rx   = dma_claim_unused_channel(true);

    // ----------------------------------------------------------
    // kickoff_a — paced by CS-PWM DREQ
    // Writes 2 to spi0_tx TRANS_COUNT_TRIG (triggers spi0_tx)
    // then chains to kickoff_b
    // ----------------------------------------------------------
    {
        dma_channel_config c = dma_channel_get_default_config(dma_kickoff_a);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pwm_get_dreq(cs_pwm_slice_spi0));
        channel_config_set_chain_to(&c, dma_kickoff_b);

        dma_channel_configure(
            dma_kickoff_a, &c,
            &dma_hw->ch[dma_spi0_tx].al1_transfer_count_trig,
            &tx_trigger_word,
            1, false
        );
    }

    // ----------------------------------------------------------
    // kickoff_b — no DREQ, fires immediately after kickoff_a
    // Writes 2 to spi1_tx TRANS_COUNT_TRIG (triggers spi1_tx)
    // then chains back to kickoff_a (re-arms for next CS period)
    // ----------------------------------------------------------
    {
        dma_channel_config c = dma_channel_get_default_config(dma_kickoff_b);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, DREQ_FORCE);
        channel_config_set_chain_to(&c, dma_kickoff_a);

        dma_channel_configure(
            dma_kickoff_b, &c,
            &dma_hw->ch[dma_spi1_tx].al1_transfer_count_trig,
            &tx_trigger_word,
            1, false
        );
    }

    // ----------------------------------------------------------
    // SPI0 TX — drivetrain
    // Transfers 2 x 16-bit words (command | NOP) per frame.
    // Source reloaded each frame by frame_advance_isr.
    // DMA_SIZE_32 matches SPI 32-bit mode — sends upper 16 bits
    // (command) first, then lower 16 bits (NOP).
    // ----------------------------------------------------------
    {
        dma_channel_config c = dma_channel_get_default_config(dma_spi0_tx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, spi_get_dreq(spi0, true));
        channel_config_set_chain_to(&c, dma_spi0_rx);

        dma_channel_configure(
            dma_spi0_tx, &c,
            &spi_get_hw(spi0)->dr,
            (void *)&adc0_tx_f1,
            2, false
        );
    }

    // ----------------------------------------------------------
    // SPI0 RX — drivetrain
    // Receives 1 x 32-bit word per frame.
    // Result is in lower 16 bits: (uint16_t)(adc0_rx_f1 & 0xFFFF)
    // Dest reloaded each frame by frame_advance_isr.
    // IRQ0 fires on completion to advance frame pointer.
    // ----------------------------------------------------------
    {
        dma_channel_config c = dma_channel_get_default_config(dma_spi0_rx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_dreq(&c, spi_get_dreq(spi0, false));

        dma_channel_configure(
            dma_spi0_rx, &c,
            (void *)&adc0_rx_f1,
            &spi_get_hw(spi0)->dr,
            2, false
        );

        dma_channel_set_irq0_enabled(dma_spi0_rx, true);
    }

    // ----------------------------------------------------------
    // SPI1 TX — capbank
    // ----------------------------------------------------------
    {
        dma_channel_config c = dma_channel_get_default_config(dma_spi1_tx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, spi_get_dreq(spi1, true));
        channel_config_set_chain_to(&c, dma_spi1_rx);

        dma_channel_configure(
            dma_spi1_tx, &c,
            &spi_get_hw(spi1)->dr,
            (void *)&adc1_tx_f1,
            2, false
        );
    }

    // ----------------------------------------------------------
    // SPI1 RX — capbank
    // ----------------------------------------------------------
    {
        dma_channel_config c = dma_channel_get_default_config(dma_spi1_rx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_dreq(&c, spi_get_dreq(spi1, false));

        dma_channel_configure(
            dma_spi1_rx, &c,
            (void *)&adc1_rx_f1,
            &spi_get_hw(spi1)->dr,
            2, false
        );
    }
}

// ============================================================
// FRAME ADVANCE ISR
// Fired by SPI0 RX DMA complete — once per 416-cycle CS period.
// Reloads TX source and RX dest for next frame on both SPI buses.
// ============================================================

static void __isr frame_advance_isr(void)
{
    dma_hw->ints0 = (1u << dma_spi0_rx);

    uint next = (frame_index + 1) % 3;

    dma_channel_set_read_addr(dma_spi0_tx,  spi0_tx_frames[next], false);
    dma_channel_set_write_addr(dma_spi0_rx, spi0_rx_frames[next], false);
    dma_channel_set_read_addr(dma_spi1_tx,  spi1_tx_frames[next], false);
    dma_channel_set_write_addr(dma_spi1_rx, spi1_rx_frames[next], false);

    frame_index = next;
}

// ============================================================
// PUBLIC: INIT
// ============================================================

void buck_control_init(void)
{
    // SPI peripherals
    setup_spi(spi0, SPI0_PIN_SCK, SPI0_PIN_MOSI, SPI0_PIN_MISO);
    setup_spi(spi1, SPI1_PIN_SCK, SPI1_PIN_MOSI, SPI1_PIN_MISO);

    // CS pins as plain GPIO, deasserted high
    gpio_init(SPI0_PIN_CS);
    gpio_set_dir(SPI0_PIN_CS, GPIO_OUT);
    gpio_put(SPI0_PIN_CS, 1);

    gpio_init(SPI1_PIN_CS);
    gpio_set_dir(SPI1_PIN_CS, GPIO_OUT);
    gpio_put(SPI1_PIN_CS, 1);

    // CS PWM slices (configured but pin stays GPIO until start_spi)
    setup_cs_pwms();

    // HB PWM slice
    setup_hb_pwm();

    // Precompute masks
    cs_pwm_mask = (1u << cs_pwm_slice_spi0) | (1u << cs_pwm_slice_spi1);
    hb_pwm_mask = (1u << hb_pwm_slice);

    // DMA channels
    setup_dma();

    // Frame advance IRQ
    irq_set_exclusive_handler(DMA_IRQ_0, frame_advance_isr);
    irq_set_enabled(DMA_IRQ_0, true);
}

// ============================================================
// PUBLIC: START SPI
// Transfer CS pins to PWM, start CS-PWM slices, arm kickoff DMA.
// ============================================================

void buck_control_start_spi(void)
{
    frame_index = 0;

    // Transfer CS pins from GPIO to PWM function
    gpio_set_function(SPI0_PIN_CS, GPIO_FUNC_PWM);
    gpio_set_function(SPI1_PIN_CS, GPIO_FUNC_PWM);

    // Start both CS-PWM slices simultaneously — single register write
    // guarantees both counters start from 0 on the same cycle
    pwm_set_mask_enabled(cs_pwm_mask);

    // Arm kickoff_a — fires on first CS DREQ (416 cycles from now)
    dma_channel_start(dma_kickoff_a);
}

// ============================================================
// PUBLIC: START HB
// Enable HB-PWM only — OR into pwm_hw->en to avoid
// resetting already-running CS PWM slices.
// ============================================================

void buck_control_start_hb(void)
{
    pwm_hw->en |= hb_pwm_mask;
}

// ============================================================
// PUBLIC: STOP
// Graceful stop — waits for HB period to complete, disables all
// PWM, aborts DMA, returns CS pins to plain GPIO (deasserted high).
// ============================================================

void buck_control_stop(void)
{
    // Wait for phase-correct counter to reach TOP then return to 0
    while (pwm_get_counter(hb_pwm_slice) < PWM_TOP);
    while (pwm_get_counter(hb_pwm_slice) > 0);

    // Disable all PWM slices
    pwm_set_mask_enabled(0);

    // Abort DMA
    dma_channel_abort(dma_kickoff_a);
    dma_channel_abort(dma_kickoff_b);
    dma_channel_abort(dma_spi0_tx);
    dma_channel_abort(dma_spi0_rx);
    dma_channel_abort(dma_spi1_tx);
    dma_channel_abort(dma_spi1_rx);

    // Return CS pins to plain GPIO, deasserted high
    gpio_set_function(SPI0_PIN_CS, GPIO_FUNC_SIO);
    gpio_set_dir(SPI0_PIN_CS, GPIO_OUT);
    gpio_put(SPI0_PIN_CS, 1);

    gpio_set_function(SPI1_PIN_CS, GPIO_FUNC_SIO);
    gpio_set_dir(SPI1_PIN_CS, GPIO_OUT);
    gpio_put(SPI1_PIN_CS, 1);

    frame_index = 0;
}

// ============================================================
// PUBLIC: ADC READ
// Manual single-channel blocking read. CS driven as plain GPIO.
// Call only before buck_control_start_spi() or after buck_control_stop().
// ch: ADC_CH(spi, channel) — e.g. ADC_CH(0, 2) = spi0 channel 2
// ADS8684 pipeline: frame N TX selects channel for frame N+1 RX.
// Sends 2 complete 32-bit frames. Result returned from frame 2 RX.
// ============================================================

uint16_t adc_read(uint8_t ch)
{
    // ch encoding: ADC_CH(spi, channel)
    //   bits[7:4] = SPI (0 = spi0, 1 = spi1)
    //   bits[3:0] = ADC channel (0-3)
    spi_inst_t *spi    = (ch >> 4) ? spi1 : spi0;
    uint        cs_pin = (ch >> 4) ? SPI1_PIN_CS : SPI0_PIN_CS;
    uint8_t     adc_ch = ch & 0x0F;
    // TX: command in lower 16 bits (sent first), NOP in upper 16 bits
    uint32_t tx = (uint32_t)ADS8684_CMD_MANUAL_CH(adc_ch);
    uint32_t rx = 0;

    gpio_put(cs_pin, 0);
    spi_write_read_blocking(spi, (uint8_t *)&tx, (uint8_t *)&rx, 4);
    gpio_put(cs_pin, 1);

    return (uint16_t)(rx >> 16);
}

// ============================================================
// PUBLIC: ADC SET RANGE
// tells all ADCs to configure all channels to measure in the range of 0-5.12 instead of +-10.24
// allows for 4x more precise readings of the voltage and current sensors
// ============================================================

void adc_set_range(void)
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        uint8_t  reg_addr = 0x05 + ch;
        uint16_t cmd      = ((uint16_t)reg_addr << 9) | (1u << 8) | 0x05u;
        uint32_t tx       = (uint32_t)cmd;
        uint32_t rx       = 0;

        gpio_put(SPI0_PIN_CS, 0);
        spi_write_read_blocking(spi0, (uint8_t *)&tx, (uint8_t *)&rx, 4);
        gpio_put(SPI0_PIN_CS, 1);

        gpio_put(SPI1_PIN_CS, 0);
        spi_write_read_blocking(spi1, (uint8_t *)&tx, (uint8_t *)&rx, 4);
        gpio_put(SPI1_PIN_CS, 1);
    }
}