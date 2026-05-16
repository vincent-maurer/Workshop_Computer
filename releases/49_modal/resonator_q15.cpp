// =============================================================================
// resonator_q15.cpp — Fixed-point Modal Resonator for Elements port
//
// Port of elements::Resonator to Q15 integer arithmetic.
// Based on original code by Émilie Gillet (MIT license).
// =============================================================================

#include "resonator_q15.h"

// ── Fast tan(π·f) approximation ────────────────────────────────────────────
// 5th-order polynomial: g = f·(π + a·f² + b·f⁴)
// Accurate to <0.5% for f < 0.35 (8.4kHz at 24kHz SR), good enough for SVFs.
// Input: f_q16 = normalized frequency (0..32768 = 0..0.5). Output: Q14.
static inline int32_t __attribute__((section(".time_critical.FastTan"))) FastTanQ14(int32_t f_q16) {
    if (f_q16 > 32112) f_q16 = 32112; // cap at ~0.49
    int64_t f2 = ((int64_t)f_q16 * f_q16) >> 16;
    int64_t f4 = (f2 * f2) >> 16;
    
    // Coefficients from Elements' FREQUENCY_FAST approximation
    // a = 3.260e-01 * π³ ≈ 10.108,  b = 1.823e-01 * π⁵ ≈ 55.787
    int32_t a_q16 = 662438;  // 10.108 * 65536
    int32_t b_q16 = 3656057; // 55.787 * 65536
    
    int32_t term1 = (int32_t)(((int64_t)a_q16 * (int32_t)f2) >> 16);
    int32_t term2 = (int32_t)(((int64_t)b_q16 * (int32_t)f4) >> 16);
    int32_t sum = 205887 + term1 + term2; // π * 65536 = 205887
    int32_t g_q16 = (int32_t)(((int64_t)f_q16 * sum) >> 16);
    return g_q16 >> 2; // Q16 → Q14
}

void ResonatorQ15::Init() {
    for (size_t i = 0; i < kMaxModesQ15; ++i) {
        f_[i].Init();
    }
    for (size_t i = 0; i < kMaxBowedModesQ15; ++i) {
        f_bow_[i].Init();
        d_bow_[i].Init();
    }
    
    frequency_q15 = 300;     // ~10Hz / 24kHz = 0.0004 → Q15 ≈ 13
    geometry_q15 = 8192;     // 0.25
    brightness_q15 = 16384;  // 0.5
    damping_q15 = 9830;      // 0.3
    position_q15 = 32734;    // 0.999
    
    previous_position_q15 = position_q15;
    // ~0.5Hz / 24kHz = 2.08e-5 → Q15 ≈ 1
    modulation_frequency_q15 = 1;
    modulation_offset_q15 = 3276; // 0.1
    lfo_phase_q15 = 0;
    
    resolution = kMaxModesQ15;
    clock_divider = 0;
    num_modes_cached = 0;
    bow_signal_q15 = 0;
}

size_t __attribute__((section(".time_critical.ComputeFilters"))) ResonatorQ15::ComputeFilters() {
    // clock_divider is incremented in Process1 (per-sample),
    // used here only for staggering higher-mode updates.
    
    // stiffness = Interpolate(lut_stiffness, geometry_, 256.0f)
    // geometry_q15 is 0..32767. LUT has 257 entries indexed 0..256.
    // Scale: index_q8 = geometry_q15 * 256 / 32767 ≈ geometry_q15 >> 7
    int32_t stiffness_idx = (geometry_q15 * 256) >> 7;
    int32_t stiffness = InterpolateQ15(lut_stiffness_q15, stiffness_idx);
    
    int32_t harmonic = frequency_q15;
    int32_t stretch_factor = 32767; // 1.0 in Q15
    
    // q = 500.0f * Interpolate(lut_4_decades, damping_ * 0.8f, 256.0f)
    int32_t damping_adj = mul_q15(damping_q15, 26214); // * 0.8
    int32_t q_idx = (damping_adj * 256) >> 7;
    // lut_4_decades_q16 gives Q16. We want q = 500 * table_value.
    // Keep as int64 to avoid overflow during the mode loop.
    int64_t q_base = (int64_t)InterpolateQ16(lut_4_decades_q16, q_idx) * 500;
    
    // Brightness attenuation = (1 - geometry)^8
    // Original does ^8 but we'll use ^4 (close enough, saves cycles)
    int32_t ba = 32767 - geometry_q15;
    int64_t ba2 = ((int64_t)ba * ba) >> 15;
    int32_t ba4 = (int32_t)((ba2 * ba2) >> 15);
    
    // brightness = brightness_ * (1.0f - 0.2f * brightness_attenuation^8)
    // Using ba4 (^4) instead of ba8 — close enough
    int32_t bright_mod = 32767 - (int32_t)(((int64_t)6553 * ba4) >> 15);
    if (bright_mod < 0) bright_mod = 0;
    int32_t brightness = mul_q15(brightness_q15, bright_mod);
    
    // q_loss = brightness * (2.0 - brightness) * 0.85 + 0.15
    int32_t two_minus_b = 65534 - brightness; // 2.0 - brightness in Q15
    int32_t q_loss_base = mul_q15(brightness, two_minus_b);
    int32_t q_loss = mul_q15(q_loss_base, 27852) + 4915; // *0.85 + 0.15
    
    // q_loss_damping_rate = geometry_ * (2.0 - geometry_) * 0.1
    int32_t two_minus_g = 65534 - geometry_q15;
    int32_t q_loss_damping_rate = mul_q15(mul_q15(geometry_q15, two_minus_g), 3277);
    
    size_t num_modes = 0;
    
    for (size_t i = 0; i < resolution; ++i) {
        // Update first 16 modes every call, higher modes every other call
        bool update = (i <= 16) || ((i & 1) == (clock_divider & 1));
        
        // partial_frequency = harmonic * stretch_factor
        // Both Q15, multiply → Q30, shift back to Q15
        int32_t partial_frequency = (int32_t)(((int64_t)harmonic * stretch_factor) >> 15);
        
        // Cap at Nyquist (0.49 * 32768 = 16056)
        if (partial_frequency >= 16056) {
            partial_frequency = 16056;
        } else {
            num_modes = i + 1;
        }
        
        if (update) {
            // Convert Q15 normalized freq to Q16 for FastTan
            int32_t f_q16 = partial_frequency << 1;
            int32_t g_q14 = FastTanQ14(f_q16);
            
            // resonance = 1.0 + partial_frequency * q
            // q_base = table_Q16 * 500 (int64). partial_frequency is Q15.
            // Product = Q15 * Q16 * 500 = Q31 * 500.
            // We want the result in Q15 to add to Q15 base, so shift >> 16.
            int64_t res_term = ((int64_t)partial_frequency * q_base) >> 16;
            int32_t resonance_q15 = 32767 + (int32_t)res_term;
            if (resonance_q15 < 328) resonance_q15 = 328;
            
            f_[i].SetGQ(g_q14, resonance_q15);
            
            if (i < kMaxBowedModesQ15) {
                // Delay line period = sr / frequency = 1.0 / partial_freq_norm
                // partial_frequency is Q15 (32768 = 1.0).
                // period in samples = 1.0 / (partial_frequency / 32768)
                //                   = 32768 / partial_frequency
                int32_t period = (partial_frequency > 0)
                    ? 32768 / partial_frequency
                    : (int32_t)kMaxDelayLineSizeQ15 - 1;
                while (period >= (int32_t)kMaxDelayLineSizeQ15 && period > 1)
                    period >>= 1;
                d_bow_[i].set_delay((uint16_t)period);
                
                // Bow SVF: higher Q for sustained oscillation
                // resonance = 1.0 + partial_freq * 1500
                // partial_freq is Q15. 1500 is raw. Result should be Q15.
                // partial_freq * 1500 >> 0 is already ~Q15 scale
                // (e.g. 357 * 1500 = 535500 / 32768 = 16.3 → Q = 17.3)
                int32_t bow_res = 32767 + partial_frequency * 1500;
                if (bow_res < 328) bow_res = 328;
                f_bow_[i].SetGQ(g_q14, bow_res);
            }
        }
        
        stretch_factor += stiffness;
        if (stiffness < 0) {
            stiffness = mul_q15(stiffness, 30474); // * 0.93
        } else {
            stiffness = mul_q15(stiffness, 32112); // * 0.98
        }
        
        // q_loss accumulates toward 1.0 over higher partials
        int32_t q_loss_inv = 32767 - q_loss;
        q_loss += mul_q15(q_loss_damping_rate, q_loss_inv);
        
        harmonic += frequency_q15;
        q_base = (q_base * q_loss) >> 15;
    }
    
    return num_modes;
}

void __attribute__((section(".time_critical.resonator"))) ResonatorQ15::Process1(int32_t bow_strength, int32_t in,
                            int32_t &center, int32_t &sides) {
    // Throttle filter coefficient updates: recompute every 24 samples
    // (original does once per 32-sample block at 32kHz ≈ 1kHz update rate)
    // At 24kHz, every 24 samples = 1kHz update — matches original.
    ++clock_divider;
    if ((clock_divider % 24) == 0) {
        num_modes_cached = ComputeFilters();
    }
    size_t num_modes = num_modes_cached;
    if (num_modes == 0) num_modes = 1; // safety
    
    size_t num_banded_wg = (kMaxBowedModesQ15 < num_modes)
        ? kMaxBowedModesQ15 : num_modes;
    
    // Smooth position to prevent zipper noise on amplitude distribution (Issue 7)
    previous_position_q15 += (position_q15 - previous_position_q15) >> 6;
    // LFO for stereo modulation (~0.5Hz triangle)
    // Position-dependent amplitude distribution (CosineOscillator pattern)
    CosineOscQ15 amplitudes;
    amplitudes.Init(previous_position_q15);

    // ── Render normal modes ──────────────────────────────────────────────
    // Input scaling: original uses 0.125 (>>3)
    int32_t input = in >> 3;
    int32_t sum_center = 0;
    int32_t sum_side = 0;
    
    amplitudes.Start();
    for (size_t i = 0; i < num_modes; ++i) {
        int32_t s = f_[i].Process(input, FILT_BP);
        int32_t amp_c, amp_s;
        amplitudes.NextQuadrature(amp_c, amp_s);
        sum_center += mul_q15(s, amp_c);
        sum_side   += mul_q15(s, amp_s);
    }
    
    // ── Render bowed modes ───────────────────────────────────────────────
    int32_t bow_signal = 0;
    int32_t input_bow = input + bow_signal_q15;
    
    amplitudes.Start();
    for (size_t i = 0; i < num_banded_wg; ++i) {
        int32_t s = mul_q15(d_bow_[i].Read(), 32440); // * 0.99
        bow_signal += s;
        s = f_bow_[i].Process(input_bow + s, FILT_BPN);
        d_bow_[i].Write(s);
        
        int32_t amp_c, amp_s;
        amplitudes.NextQuadrature(amp_c, amp_s);
        sum_center += mul_q15(s, amp_c) << 3;  // ×8 (matches original's * 8.0f)
        sum_side   += mul_q15(s, amp_s) << 3;
    }
    
    // ── Bow Table (friction model) ───────────────────────────────────────
    int32_t velocity = bow_strength;
    int32_t x = mul_q15(4259, velocity) - bow_signal;
    int32_t six_x = x * 6;
    int32_t abs_six_x = (six_x < 0) ? -six_x : six_x;
    int32_t denom = abs_six_x + 24576; // + 0.75
    
    int64_t d2 = ((int64_t)denom * denom) >> 15;
    int32_t d4 = (int32_t)((d2 * d2) >> 15);
    
    int32_t bow_gain;
    if (d4 < 1) {
        bow_gain = 8028; // 0.245
    } else {
        bow_gain = (int32_t)(268435456LL / d4); // 8192 * 32768
    }
    if (bow_gain < 82)   bow_gain = 82;   // 0.0025
    if (bow_gain > 8028) bow_gain = 8028; // 0.245
    
    bow_signal_q15 = mul_q15(x, bow_gain);
    
    // Output scaling and stereo separation
    // Scale down significantly (>>7) to provide headroom for the 24-mode stack.
    center = sum_center >> 7;
    sides = (sum_side - sum_center) >> 7;
}
