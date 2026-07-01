#include "calculations.h"

// ============================================================
// ADC CONVERSION
// ============================================================

float adc_to_voltage(uint16_t raw)
{
    // TODO: apply scaling for voltage sensor circuit
    // Example: return (float)raw * LSB_SIZE * DIVIDER_RATIO;
    (void)raw;
    return (float)raw * 0.25f / 65536.0f;
}

float adc_to_current(uint16_t raw)
{
    // TODO: apply scaling for current sense amplifier
    // Example: return (float)raw * LSB_SIZE / (SENSE_RESISTANCE * AMP_GAIN);
    (void)raw;
    return 0.0f;
}

// ============================================================
// ENERGY
// ============================================================

float calc_energy_stored(float capacitance, float voltage)
{
    // TODO: implement E = 0.5 * C * V^2
    (void)capacitance;
    (void)voltage;
    return 0.0f;
}

// ============================================================
// POWER LIMITS
// ============================================================

float calc_max_charge_power(float cap_voltage, float v_max, float i_max)
{
    // TODO: implement voltage-dependent charge power limit
    // At cap_voltage near v_max, reduce max power to protect capacitor.
    // Example: linear derating over the top N volts of the range.
    (void)cap_voltage;
    (void)v_max;
    (void)i_max;
    return 0.0f;
}

float calc_max_discharge_power(float cap_voltage, float v_min, float i_max)
{
    // TODO: implement voltage-dependent discharge power limit
    // At cap_voltage near v_min, reduce max power due to current limits.
    // Example: P_max = cap_voltage * i_max, clamped to system minimum.
    (void)cap_voltage;
    (void)v_min;
    (void)i_max;
    return 0.0f;
}
