#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// ADC CONVERSION
// All functions take a raw 16-bit ADS8684 result and return
// a physical quantity as a float.
// Scaling factors are determined by external circuitry
// (resistor dividers, current sense amplifier gain, etc.)
// and must be filled in by the implementer.
// ============================================================

// Convert raw ADC reading to voltage in volts.
float adc_to_voltage(uint16_t raw);

// Convert raw ADC reading to current in amps.
// Direction is determined by the caller via the command buffer —
// this function returns a magnitude with a positive sign always.
float adc_to_current(uint16_t raw);

// ============================================================
// ENERGY
// ============================================================

// Calculate energy stored in a capacitor in joules: E = 0.5 * C * V^2
// C is capacitance in farads, voltage in volts.
float calc_energy_stored(float capacitance, float voltage);

// ============================================================
// POWER LIMITS
// Returns maximum allowable input power in watts at the given
// capacitor voltage and current limit.
// Both limits are signed: positive = charging, negative = discharging.
// ============================================================

// Maximum charge power at the given capacitor voltage.
// Near V_max, this will be reduced to protect the capacitor.
float calc_max_charge_power(float cap_voltage, float v_max, float i_max);

// Maximum discharge power at the given capacitor voltage.
// Near V_min, this will be reduced due to current limits.
float calc_max_discharge_power(float cap_voltage, float v_min, float i_max);

#ifdef __cplusplus
}
#endif
