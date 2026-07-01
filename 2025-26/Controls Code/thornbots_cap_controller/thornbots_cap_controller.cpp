#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

extern "C" {
    #include "pico/bootrom.h"
}

#include "buck_control.h"
#include "calculations.h"

// ============================================================
// COMMS MODE
// 0 = USB Serial (stdio over USB, no extra pins needed)
// 1 = UART       (GP0 TX, GP1 RX)
// Also update CMakeLists.txt:
//   COMMS_MODE 0 -> pico_enable_stdio_usb  1, pico_enable_stdio_uart 0
//   COMMS_MODE 1 -> pico_enable_stdio_usb  0, pico_enable_stdio_uart 1
// ============================================================
#define COMMS_MODE 0

// ============================================================
// COMMS HELPERS
// ============================================================

static void comms_init(void)
{
#if COMMS_MODE == 1
    stdio_uart_init_full(uart0, 115200, UART_PIN_TX, UART_PIN_RX);
#else
    stdio_usb_init();
#endif
}

// Accumulates characters across calls, returns true when a full line arrives.
static bool comms_readline(char *buf, size_t len)
{
    static size_t pos = 0;
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            buf[pos] = '\0';
            pos = 0;
            return true;
        }
        if (pos < len - 1) {
            buf[pos++] = (char)c;
        }
    }
    return false;
}

// ============================================================
// SHARED VOLATILES — written by core 0, read by core 1
// ============================================================

static volatile float    energy_stored        = 0.0f;
static volatile float    max_charge_power     = 0.0f;
static volatile float    max_discharge_power  = 0.0f;

// Written by core 1, read by core 0
static volatile float    target_power         = 0.0f;
static volatile uint16_t target_duty          = 0;
static volatile bool     use_power_mode       = true;
// ============================================================
// CORE 1 — COMMS
// Input:  "P <watts>\n"  — power setpoint (signed)
//         "D <counts>\n" — raw duty cycle (signed, clamped to ±PWM_TOP)
//         "S\n"          — start signal to core 0
//         "X\n"          — stop signal to core 0
// Output: "E <joules> <max_charge_W> <max_discharge_W>\n" every 100 ms
// ============================================================
 
static void core1_comms_entry(void)
{
    comms_init();
 
#if COMMS_MODE == 0
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
#endif
 
    printf("READY\n");
    printf("Commands: S=start X=stop B=bootloader D=duty P=power\n");
 
    absolute_time_t next_output = make_timeout_time_ms(100);
    char line[32];
 
    while (true) {
 
        if (comms_readline(line, sizeof(line))) {
            char cmd   = line[0];
            int  value = (strlen(line) > 2) ? atoi(&line[2]) : 0;
 
            switch (cmd) {
 
            case 'S':
                // Start — core 0 will ignore if already running
                multicore_fifo_push_blocking(1);
                printf("start\n");
                break;
 
            case 'X':
                // Stop
                multicore_fifo_push_blocking(0);
                printf("stop\n");
                break;
 
            case 'B':
                // Reboot to BOOTSEL — stop first if running
                multicore_fifo_push_blocking(0);
                sleep_ms(50);   // give core 0 time to stop cleanly
                printf("rebooting to bootloader\n");
                sleep_ms(10);   // flush printf before reset
                reset_usb_boot(0, 0);
                break;          // never reached
 
            case 'D':
                // Set raw duty cycle counts (signed, clamped to ±PWM_TOP)
                if (value >  (int)PWM_TOP) value =  (int)PWM_TOP;
                if (value < -(int)PWM_TOP) value = -(int)PWM_TOP;
                target_duty    = (uint16_t)value;
                use_power_mode = false;
                printf("duty: %d\n", value);
                break;
 
            case 'P':
                // Set power setpoint in watts (used once control law is implemented)
                target_power   = (float)value;
                use_power_mode = true;
                printf("power: %d W\n", value);
                break;
 
            default:
                printf("?\n");
                break;
            }
        }
 
        // Equivalent to:
        //   Serial.print("E ");
        //   Serial.print(energy_stored); ...
        //   Serial.println();
        if (absolute_time_diff_us(get_absolute_time(), next_output) <= 0) {
            printf("E %.2f %.2f %.2f\n",
                   (double)energy_stored,
                   (double)max_charge_power,
                   (double)max_discharge_power);
            next_output = make_timeout_time_ms(100);
        }
 
        sleep_us(100);
    }
}
 
// ============================================================
// CONTROL UPDATE
// Called in main loop after each full 3-frame rotation completes.
// Reads DMA RX results, updates shared output values for core 1,
// computes new duty cycle, writes it to HB PWM,
// updates command slots for next rotation.
// ============================================================

static void control_update(void)
{    
    float v_dt  = adc_to_voltage((uint16_t)(adc0_rx_f1 >> 16));
    float i_dt  = adc_to_current((uint16_t)(adc0_rx_f2 >> 16));
    float v_cap = adc_to_voltage((uint16_t)(adc1_rx_f1 >> 16));

    // Update shared values for core 1 to transmit
    energy_stored       = calc_energy_stored(0.0f, v_cap);         // TODO: fill capacitance
    max_charge_power    = calc_max_charge_power(v_cap, 0.0f, 0.0f);   // TODO: limits
    max_discharge_power = calc_max_discharge_power(v_cap, 0.0f, 0.0f);

    // TODO: implement control law using v_dt, i_dt, target_power/target_duty
    uint16_t new_duty = PWM_DEADTIME;
    (void)v_dt; (void)i_dt;

    // if (!use_power_mode) {
    //     new_duty = target_duty;
    // }    
    new_duty = (uint16_t)(v_cap * 150.0f);

    pwm_set_both_levels(pwm_gpio_to_slice_num(HB_PIN_A),
                        new_duty,
                        PWM_TOP - new_duty);
}

// ============================================================
// MAIN — core 0
// ============================================================

int main(void)
{
    comms_init(); 
    buck_control_init();
    multicore_launch_core1(core1_comms_entry);

    // Wait for start signal from core 1 (S, P, or D command)
    //multicore_fifo_pop_blocking();

    // Pre-measurement: read V and I before starting the half-bridge.
    // adc_read() uses plain GPIO CS (safe before start_spi).
    // ADS8684 pipeline handled internally — returns valid result.
    // uint16_t v_dt_raw  = adc_read(ADC_CH(0, 0));   // TODO: correct channels
    // uint16_t i_dt_raw  = adc_read(ADC_CH(0, 1));
    // uint16_t v_cap_raw = adc_read(ADC_CH(1, 0));

    // float v_dt  = adc_to_voltage(v_dt_raw);
    // float i_dt  = adc_to_current(i_dt_raw);
    // float v_cap = adc_to_voltage(v_cap_raw);
    // (void)v_dt; (void)i_dt; (void)v_cap;

    // TODO: calculate safe initial duty from pre-measurements
    uint16_t duty_cycle = 6;
    pwm_set_both_levels(pwm_gpio_to_slice_num(HB_PIN_A),
                        duty_cycle,
                        duty_cycle + PWM_DEADTIME);

    // Load initial command slots
    adc0_tx_f1 = (uint32_t)CMD_DT_F1;
    adc0_tx_f2 = (uint32_t)CMD_DT_F2;
    adc0_tx_f3 = (uint32_t)CMD_DT_F3;

    adc1_tx_f1 = (uint32_t)CMD_CAP_F1;
    adc1_tx_f2 = (uint32_t)CMD_CAP_F2;
    adc1_tx_f3 = (uint32_t)CMD_CAP_F3;


    // Start SPI — CS pins transfer to PWM, DMA chain arms
    buck_control_start_spi();

    // Counter-poll: wait BUCK_DELAY cycles into the CS period,
    // then start HB-PWM. Phase offset is locked permanently from here.
    while (pwm_get_counter(cs_pwm_slice_spi0) < BUCK_DELAY);
    buck_control_start_hb();

    // Main control loop
    while (true) {

        // TODO: gate on frame-complete flag from frame_advance_isr
        // (set when frame_index wraps 2->0, cleared here after reading)
        //control_update();

        // adc_read(ADC_CH(1, 2));
        // sleep_us(50);
        // Check for stop signal from core 1
        // if (multicore_fifo_rvalid()) {
        //     uint32_t msg = multicore_fifo_pop_blocking();
        //     if (msg == 0) {
        //         buck_control_stop();
        //         // TODO: handle restart or halt after stop
        //     }
        // }
    }
}