#include "string_q15.h"

void StringQ15::Init() {
    string_.Init();
    stretch_.Init();
    lp_state_ = 0;
    hp_state_ = 0;
    dc_px = 0;
    dc_py = 0;
    dispersion_noise_ = 0;
    curved_bridge_ = 0;
    out_sample_[0] = 0;
    out_sample_[1] = 0;
    aux_sample_[0] = 0;
    aux_sample_[1] = 0;
}

void __attribute__((section(".time_critical.string_process"))) StringQ15::Process(int32_t in, int32_t& out, int32_t& aux) {
    // delay_samples = sr / f
    // f_inc (48kHz) = f / 48000 * 2^32
    // f = f_inc * 48000 / 2^32
    // delay_samples (24kHz) = 24000 / f = 24000 * 2^32 / (f_inc * 48000) = 2^31 / f_inc
    // delay_q15 = delay_samples * 32768 = 2^46 / f_inc
    
    int32_t delay_q15;
    if (frequency_inc_ < 100) {
        delay_q15 = (STRING_DELAY_SIZE - 4) << 15;
    } else {
        delay_q15 = (int32_t)(((int64_t)1 << 46) / (int64_t)frequency_inc_);
    }
    
    // Clamp delay
    if (delay_q15 > ((STRING_DELAY_SIZE - 4) << 15)) {
        delay_q15 = (STRING_DELAY_SIZE - 4) << 15;
    }
    if (delay_q15 < (4 << 15)) {
        delay_q15 = (4 << 15);
    }
    
    // ── Rock-Solid 1-Pole Damping Filters ──
    int32_t lp_coef = (int32_t)(frequency_inc_ >> 13) + brightness_q15_; // Adjusted scaling
    if (lp_coef > 30000) lp_coef = 30000;
    if (lp_coef < 100) lp_coef = 100;
    
    int32_t hp_coef = 100 + mul_q15(damping_q15_, 500); // 100 to 600
    
    // Feedback amount
    int32_t feedback = 32700 - mul_q15(damping_q15_, 4000); // 32700 to 28700
    
    // Comb delay (pickup position)
    int32_t clamped_pos = 16384 - mul_q15(32112, (position_q15_ > 16384 ? position_q15_ - 16384 : 16384 - position_q15_));
    if (clamped_pos < 0) clamped_pos = 0;
    int32_t comb_delay_q15 = mul_q15(delay_q15, clamped_pos);
    
    // ── Filter Delay Compensation ──
    // The 1-pole LP filter adds group delay: d = (1-a)/a.
    // lp_coef is 'a' in Q15. Result in Q15.
    int32_t filter_delay_q15 = 0;
    if (lp_coef > 400) {
        filter_delay_q15 = (int32_t)((((int64_t)32768 - lp_coef) << 15) / lp_coef);
    } else {
        filter_delay_q15 = 80 << 15; // Cap at 80 samples
    }
    
    // Read from string, compensating for filter delay and the implicit 1-sample loop delay.
    // Implicit loop delay is 32768 (1 sample).
    int32_t read_delay = delay_q15 - 32768 - filter_delay_q15; 
    
    // Read from string
    int32_t s = string_.ReadHermite(read_delay);
    
    // ── Dispersion Delay Compensation ──
    // The allpass filter adds group delay: d = D * (1-g)/(1+g)
    // where D is ap_delay and g is ap_gain.
    int32_t ap_delay_q15 = 0;
    int32_t ap_gain = 0;
    int32_t dispersion_compensation_q15 = 0;

    if (dispersion_q15_ > 1000) {
        ap_gain = -mul_q15(20000, dispersion_q15_); // max -0.61
        int32_t stretch_point = mul_q15(dispersion_q15_, 16384); // max 0.5
        ap_delay_q15 = mul_q15(read_delay, stretch_point);
        
        // group delay multiplier: (1-g)/(1+g). 
        // Since g (ap_gain) is negative, let G = -g. Mult = (1+G)/(1-G).
        int32_t g_abs = -ap_gain;
        int32_t multiplier_q12 = (int32_t)((((int64_t)32768 + g_abs) << 12) / (32768 - g_abs));
        dispersion_compensation_q15 = (int32_t)(((int64_t)ap_delay_q15 * multiplier_q12) >> 12);
        
        int32_t main_delay = read_delay - dispersion_compensation_q15;
        if (ap_delay_q15 >= (4 << 15) && main_delay >= (4 << 15)) {
            s = string_.ReadHermite(main_delay);
            s = stretch_.Allpass(s, ap_delay_q15, ap_gain);
        }
    }
    
    // Lowpass filter
    lp_state_ += mul_q15(lp_coef, s - lp_state_);
    s = lp_state_;
    
    // Highpass filter (DC block)
    hp_state_ += mul_q15(hp_coef, s - hp_state_);
    s = s - hp_state_;
    
    // Apply feedback
    s = mul_q15(s, feedback);
    
    // Inject input
    s += in;
    
    // Soft clip to prevent blow-ups
    s = SoftClipQ15(s);
    
    string_.Write((int16_t)s);
    
    out_sample_[1] = out_sample_[0];
    aux_sample_[1] = aux_sample_[0];
    out_sample_[0] = s;
    aux_sample_[0] = string_.ReadHermite(comb_delay_q15);
    
    out = out_sample_[0];
    aux = aux_sample_[0];
}
