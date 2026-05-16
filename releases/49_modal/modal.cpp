// =============================================================================
//  49_modal — Mutable Instruments Elements port for Workshop Computer
//
//  A modal synthesis voice inspired by Émilie Gillet's Elements module.
//  Ported from the mi-UGens SuperCollider adaptation by Volker Böhm.
//
//  Original code: Émilie Gillet (Mutable Instruments) — MIT License
//  SC port: Volker Böhm — GPL3
//  Workshop Computer port: Vincent Maurer — GPL3
//
//  Target: RP2040 (Cortex-M0+, no FPU, 264KB RAM)
//  All DSP uses Q15 fixed-point arithmetic (int32_t with >>15 scaling).
//  Audio runs at 48kHz on Core 0, DSP engine at 24kHz on Core 1.
// =============================================================================

// ── ComputerCard setup ──────────────────────────────────────────────────────
// Use 144kHz system clock for reduced ADC tonal artifacts.
// ProcessSample runs at 48kHz; we decimate to 24kHz for the DSP engine.
#define COMPUTERCARD_SAMPLE_RATE_DIV 1
#include "ComputerCard.h"

#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <string.h>

// ── Fixed-point DSP infrastructure ──────────────────────────────────────────
#include "dsp_q15.h"
#include "envelope_q15.h"
#include "exciter_q15.h"
#include "resonator_q15.h"
#include "plate_reverb_q15.h"
#include "svf_q15.h"
#include "string_q15.h"
#include "diffuser_q15.h"
#include "tube_q15.h"

// ── Constants ───────────────────────────────────────────────────────────────

#define SAMPLE_RATE        48000
#define DSP_RATE           24000   // Effective DSP rate (every 2nd sample)
#define NUM_PAGES          6       // Number of parameter pages
#define BOOT_SILENCE_MS    1000    // Startup mute duration in samples
#define BOOT_SILENCE_SAMP  (SAMPLE_RATE * BOOT_SILENCE_MS / 1000)

// ── Fixed-point helpers ─────────────────────────────────────────────────────
// mul_q15() and other core Q15 ops are in dsp_q15.h.
// Knob scaling is project-specific:

/// Scale a raw knob value (0–4095) to Q15 (0–32767), with dead zones at edges.
/// The pots don't reliably reach 0, so we remap ~14–4095 → 0–32767.
static inline int32_t knob_to_q15(int32_t raw) {
    int32_t v = (raw - 50) * 32767 / (4095 - 100);
    if (v < 0) v = 0;
    if (v > 32767) v = 32767;
    return v;
}

// ── DC Blocker ──────────────────────────────────────────────────────────────
// Simple first-order high-pass filter to remove DC offset from audio signals.

static inline int32_t dc_block(int32_t x, int32_t &px, int32_t &py) {
    int32_t y = x - px + py - (py >> 8);
    px = x;
    py = y;
    return y;
}

// ── PRNG ────────────────────────────────────────────────────────────────────
// Fast linear congruential generator, one seed per core to avoid contention.

static uint32_t rand_seed0 = 12345, rand_seed1 = 67890;

uint32_t __not_in_flash_func(fast_rand)(int core) {
    uint32_t &s = (core == 0) ? rand_seed0 : rand_seed1;
    s = (1103515245u * s + 12345u);
    return s;
}

// ── Knob Lock ───────────────────────────────────────────────────────────────
// When switching pages, knobs are "locked" to their last value. They only
// start controlling the new page's parameter once the user moves them past
// a threshold (~40% of range), preventing parameter jumps on page change.

struct KnobLock {
    bool locked = true;
    int32_t ref = 0;

    /// Lock the knob at its current value (called on page change)
    void engage(int32_t v) {
        locked = true;
        ref = v;
    }

    /// Returns true if the knob is "unlocked" and should update the parameter.
    /// Unlocks when the knob moves >1638 units (~5% of Q15 range) from ref.
    bool update(int32_t v) {
        if (locked) {
            int32_t d = v - ref;
            if (d < 0) d = -d;
            if (d > 1638) locked = false;
        }
        return !locked;
    }
};

// ── Page Parameter Storage ──────────────────────────────────────────────────
// Each page stores 3 parameters (one per knob: Main, X, Y) in Q15 range.
// Pages are logically grouped to match Elements' control layout.
//
// Page 0: STRIKE         — Level / Timbre / Model (Meta)
// Page 1: BLOW           — Level / Timbre / Texture (Meta)
// Page 2: BOW & ENV      — Bow Level / Bow Timbre / Env Shape
// Page 3: RESONATOR 1    — Geometry / Brightness / Damping
// Page 4: RESONATOR 2    — Position / Space / [Unused]
// Page 5: PERFORMANCE    — Pitch Coarse / Fine Tune / Exciter Strength

struct PageParams {
    int32_t pMain, pX, pY;
};

// ── Resonator Model ─────────────────────────────────────────────────────────
enum ResonatorModel {
    MODEL_MODAL   = 0,   // Bank of bandpass filters (SVF) simulating modes
    MODEL_STRING  = 1,   // Single Karplus-Strong string
    MODEL_STRINGS = 2,   // Chord of 3 strings
    MODEL_COUNT   = 3
};

// ── Core 1 Communication ────────────────────────────────────────────────────
// We use the RP2040 multicore FIFO to send parameters from Core 0 (audio IRQ)
// to Core 1 (DSP engine) and receive processed audio samples back.
//
// Protocol (every 2nd ProcessSample call):
//   Core 0 → Core 1:  word 0 = trigger/gate flags
//   Core 1 → Core 0:  word 0 = left sample (Q15)
//                      word 1 = right sample (Q15)

// Trigger flags packed into the FIFO word
#define FIFO_FLAG_ACTIVE    0x00000001   // "I'm sending you work"
#define FIFO_FLAG_GATE      0x00000002   // Gate is high (Pulse In 1)
#define FIFO_FLAG_RISING    0x00000004   // Gate just went high
#define FIFO_FLAG_FALLING   0x00000008   // Gate just went low
#define FIFO_FLAG_MODEL_CHG 0x00000010   // Resonator model changed

// ── Shared State (volatile for cross-core access) ───────────────────────────

// Parameters are written by Core 0 and read by Core 1.
// Cross-core access is safe because writes are atomic at the word level
// and we only write from Core 0 / read from Core 1.
PageParams params[NUM_PAGES];
volatile int32_t    cv1_pitch_q8   = 0;   // V/Oct pitch from CV In 1 (Q8)
volatile int32_t    cv2_strength   = 0;   // Strength from CV In 2 (Q15)
volatile int32_t    audio_in1_q15  = 0;   // Blow external input (Q15)
volatile int32_t    audio_in2_q15  = 0;   // Strike external input (Q15)
volatile int32_t    currentModel   = MODEL_MODAL;

// ── Core 1 DSP Engine ──────────────────────────────────────────────────────
// This function runs on Core 1 in an infinite loop. It waits for trigger
// words from Core 0, processes one sample of DSP, and sends audio back.
//
// Signal flow: Gate → Envelope → Exciters → Resonator → Stereo Output

// Core 1 persistent state — lives outside the function to avoid stack pressure
static ExciterQ15 bow_exciter;
static ExciterQ15 blow_exciter;
static ExciterQ15 strike_exciter;
static EnvelopeQ15 envelope;
static ResonatorQ15 resonator;
static StringQ15 strings[3];
static PlateReverbQ15 reverb;
static DiffuserQ15 diffuser;
static TubeQ15 tube;

static int32_t dc_ex_x = 0;
static int32_t dc_ex_y = 0;

// Smoothed parameters for zipper-free interpolation
static int32_t smooth_strength = 0;
static int32_t smooth_env_value = 0;

static void __not_in_flash_func(core1_dsp_loop)() {
    // Initialize exciters with different PRNG seeds
    bow_exciter.Init(12345);
    blow_exciter.Init(67890);
    strike_exciter.Init(24680);
    envelope.Init();
    resonator.Init();
    for (int i=0; i<3; i++) strings[i].Init();
    reverb.Init();
    diffuser.Init();
    tube.Init();
    
    // Set default exciter models
    bow_exciter.model = EXCITER_Q15_FLOW;
    bow_exciter.parameter = 22938;  // ~0.7
    bow_exciter.timbre = 16384;     // ~0.5
    
    blow_exciter.model = EXCITER_Q15_NOISE;
    
    strike_exciter.model = EXCITER_Q15_MALLET;
    
    while (true) {
        // Wait for work from Core 0
        uint32_t flags = multicore_fifo_pop_blocking();

        if (!(flags & FIFO_FLAG_ACTIVE)) {
            multicore_fifo_push_blocking(0);
            multicore_fifo_push_blocking(0);
            continue;
        }

        // ── Read Parameters ─────────────────────────────────────────────
        // Page 0: Strike
        int32_t strike_level  = params[0].pMain;
        int32_t strike_timbre = params[0].pX;
        int32_t strike_meta   = params[0].pY;

        // Page 1: Blow
        int32_t blow_level  = params[1].pMain;
        int32_t blow_timbre = params[1].pX;
        int32_t blow_meta   = params[1].pY;

        // Page 2: Bow & Envelope
        int32_t bow_level  = params[2].pMain;
        int32_t bow_timbre = params[2].pX;
        int32_t env_shape  = params[2].pY;

        // Page 3: Resonator Core
        int32_t geometry   = params[3].pMain;
        int32_t brightness = params[3].pX;
        int32_t damping    = params[3].pY;

        // Page 4: Resonator Space
        int32_t position   = params[4].pMain;
        int32_t space      = params[4].pX;    // Controls stereo spread
        int32_t reverb_amt = params[4].pY;    // Controls reverb mix and decay

        // Page 5: Performance
        int32_t pitch_coarse = params[5].pMain;
        int32_t fine_tune    = params[5].pX;
        int32_t strength     = params[5].pY;

        // External inputs
        int32_t ext_audio  = audio_in1_q15;  // Mono audio input for excitation
        int32_t cv_damping = audio_in2_q15;  // Use Audio 2 as CV for Damping
        int32_t pitch_cv   = cv1_pitch_q8;
        int32_t cv_bright  = cv2_strength;   // Use CV 2 as CV for Brightness modulation

        // Gate state
        bool gate    = (flags & FIFO_FLAG_GATE)    != 0;
        bool rising  = (flags & FIFO_FLAG_RISING)  != 0;
        bool falling = (flags & FIFO_FLAG_FALLING) != 0;

        // Suppress unused warnings for parameters not yet wired

        // ── Build gate flags for exciters/envelope ──────────────────────
        uint8_t env_flags = 0;
        if (gate)    env_flags |= ENV_FLAG_GATE;
        if (rising)  env_flags |= ENV_FLAG_RISING;
        if (falling) env_flags |= ENV_FLAG_FALLING;

        // ── Configure Envelope ──────────────────────────────────────────
        // env_shape (Q15): 0..13107 = AD, 13107..19660 = ADSR, 19660..32767 = AR
        // Maps the original Elements envelope shape parameter
        if (env_shape < 13107) {
            // Short AD shapes (0..0.4): attack + decay, no sustain
            // a = shape*0.75 + 0.15 → Q15: shape*24576/32767 + 4915
            int32_t a = mul_q15(env_shape, 24576) + 4915;
            int32_t dr = mul_q15(a, 29491) << 1;  // a * 1.8
            if (dr > 32767) dr = 32767;
            envelope.SetADR(a, dr, 0, dr);
        } else if (env_shape < 19660) {
            // ADSR with increasing sustain (0.4..0.6)
            int32_t s = (env_shape - 13107) * 5;  // scale to 0..32767
            if (s > 32767) s = 32767;
            envelope.SetADSR(14746, 26542, s, 26542);  // a=0.45, dr=0.81
        } else {
            // Long AR shapes (0.6..1.0): full sustain
            int32_t a = mul_q15(32767 - env_shape, 24576) + 4915;
            int32_t dr = mul_q15(a, 29491) << 1;
            if (dr > 32767) dr = 32767;
            envelope.SetADSR(a, dr, 32767, dr);
        }

        // Process envelope
        int32_t env_value = envelope.Process(env_flags);
        
        // Smooth envelope to avoid zipper noise
        smooth_env_value += (env_value - smooth_env_value) >> 3;

        // ── CV Modulations ──────────────────────────────────────────────
        // Map CV2 to Brightness and Audio2 to Damping
        // Shift right by 1 to make the modulation depth sensible (50%)
        int32_t total_brightness = brightness + (cv_bright >> 1);
        if (total_brightness < 0) total_brightness = 0;
        if (total_brightness > 32767) total_brightness = 32767;

        int32_t total_damping_cv = damping + (cv_damping >> 1);
        if (total_damping_cv < 0) total_damping_cv = 0;
        if (total_damping_cv > 32767) total_damping_cv = 32767;

        // ── Configure Resonator / String ────────────────────────────────
        
        resonator.geometry_q15 = geometry;
        resonator.brightness_q15 = total_brightness;
        resonator.position_q15 = position;
        
        for (int i = 0; i < 3; i++) {
            strings[i].SetDispersion(geometry);
            strings[i].SetBrightness(total_brightness);
            strings[i].SetPosition(position);
        }
        
        // Model toggle: changes the number of active modes for Modal
        int32_t model = currentModel;
        if (model == MODEL_MODAL) {
            resonator.resolution = kMaxModesQ15;  // 24
        } else if (model == MODEL_STRING) {
            resonator.resolution = 6;
        } else {
            resonator.resolution = 12;
        }
        
        // Stereo modulation LFO: ~0.5Hz / 24kHz = 2.08e-5
        // In Q15: 0.5/24000 * 32768 ≈ 0.68 → round to 1
        resonator.modulation_frequency_q15 = 1;
        resonator.modulation_offset_q15 = 3277; // 0.1 in Q15
        
        // ── Pitch Mapping ────────────────────────────────────────────────
        // pitch_coarse (Q15: 0..32767) → MIDI 24..96 (72 semitones)
        // Center (16384) = MIDI 60 (C4 = 261.6Hz) — musically useful range
        // In Q8: MIDI 60 = 15360.
        // Range: 24..96 (C1..C7) — good for both bass drones and bells
        // Mapping: midi = 24 + (pitch_coarse * 72 / 32767)
        //        = 24*256 + (pitch_coarse * 72*256 / 32767)
        //        ≈ 6144 + (pitch_coarse * 18432 / 32767)
        //        ≈ 6144 + (pitch_coarse * 18432 >> 15)
        int32_t midi_q8 = 6144 + (int32_t)(((int64_t)pitch_coarse * 18432) >> 15);
        
        // Fine tune: ±2 semitones (Page 5 X knob)
        // fine_tune 0..32767 → -2..+2 semitones → -512..+512 in Q8
        midi_q8 += ((fine_tune - 16384) >> 5);
        
        // CV1 V/Oct pitch tracking
        // pitch_cv is already scaled correctly to Q8 (1V = 1 octave = 3072)
        midi_q8 += pitch_cv;
        
        // Clamp to valid MIDI range
        if (midi_q8 < 0) midi_q8 = 0;           // MIDI 0 (~8Hz)
        if (midi_q8 > 30720) midi_q8 = 30720;   // MIDI 120 (C9)
        
        // Convert MIDI pitch to normalized frequency via phase increment.
        // MidiToIncrementU32() internally adds +38<<8 to the index,
        // so we pass the raw MIDI Q8 value directly — no external offset.
        uint32_t inc = MidiToIncrementU32(midi_q8);
        // MidiToIncrementU32 returns inc = (f / 48000) * 2^32.
        // We need f_norm = f / sr_dsp, where sr_dsp = 24kHz, in Q15.
        //   f_norm_q15 = (f / 24000) * 32768
        //              = (f / 48000) * 2 * 32768
        //              = (f / 48000) * 65536
        //              = (f / 48000) * 2^16
        //              = inc / 2^32 * 2^16
        //              = inc >> 16
        int32_t freq_q15 = (int32_t)(inc >> 16);
        if (freq_q15 > 16056) freq_q15 = 16056; // cap at 0.49 Nyquist
        if (freq_q15 < 1) freq_q15 = 1;
        resonator.frequency_q15 = freq_q15;
        
        // Apply frequencies to strings, including chord offsets
        // chords_table offsets are in semitones. 1 semitone = 256 in Q8.
        int32_t num_strings = (currentModel == MODEL_STRINGS) ? 3 : 1;
        for (int i = 0; i < num_strings; i++) {
            int32_t string_midi = midi_q8;
            if (num_strings == 3) {
                // Determine chord based on geometry knob (0..32767 -> 0..10 chords)
                int32_t chord_idx = (geometry * 11) >> 15;
                if (chord_idx > 10) chord_idx = 10;
                
                // Read from elements chord table (first string is usually 0, but use table directly)
                // Let's implement a simple integer chord table locally
                static const int16_t chord_offsets[11][3] = {
                    {0, 12, 24}, // Octaves
                    {0, 7, 12},  // Fifth
                    {0, 4, 7},   // Major
                    {0, 3, 7},   // Minor
                    {0, 4, 11},  // Maj7
                    {0, 3, 10},  // Min7
                    {0, 4, 10},  // Dom7
                    {0, 2, 7},   // Sus2
                    {0, 5, 7},   // Sus4
                    {0, 3, 6},   // Dim
                    {0, 4, 8}    // Aug
                };
                string_midi += chord_offsets[chord_idx][i] * 256;
            }
            if (string_midi < 0) string_midi = 0;
            if (string_midi > 30720) string_midi = 30720;
            // MidiToIncrementU32 handles the LUT offset internally
            uint32_t s_inc = MidiToIncrementU32(string_midi);
            strings[i].SetFrequency(s_inc);
        }

        // ── Configure Exciters ──────────────────────────────────────────
        
        // Brightness factor: resonator brightness modulates exciter timbre
        // brightness_factor = 0.4 + 0.6 * brightness → Q15: 13107 + brightness * 0.6
        int32_t brightness_factor = 13107 + mul_q15(brightness, 19661);
        
        // Bow: Flow model, timbre controlled by bow_timbre * brightness
        bow_exciter.timbre = mul_q15(bow_timbre, brightness_factor);
        bow_exciter.model = EXCITER_Q15_FLOW;
        bow_exciter.parameter = 22938;  // ~0.7 turbulence
        
        // Blow: Granular sample player (matching original)
        blow_exciter.parameter = blow_meta;
        blow_exciter.timbre = blow_timbre;
        blow_exciter.signature = blow_meta;  // Tie signature to meta for texture variation
        blow_exciter.model = EXCITER_Q15_GRANULAR;
        
        // Strike: Use meta to select model (Sample→Mallet→Plectrum→Particles)
        // strike_meta <= 0.4: scale to 0..0.25 range (sample player region)
        // strike_meta > 0.4: scale to 0.25..1.0 range (synth models)
        int32_t adjusted_meta;
        if (strike_meta <= 13107) {
            adjusted_meta = mul_q15(strike_meta, 20480);  // * 0.625
        } else {
            adjusted_meta = mul_q15(strike_meta, 40960) - 8192;  // * 1.25 - 0.25
        }
        if (adjusted_meta < 0) adjusted_meta = 0;
        if (adjusted_meta > 32767) adjusted_meta = 32767;
        strike_exciter.SetMeta(adjusted_meta, EXCITER_Q15_SAMPLE, EXCITER_Q15_PARTICLES);
        strike_exciter.timbre = strike_timbre;
        strike_exciter.signature = strike_meta;  // Tie signature to meta

        // ── Process Exciters ────────────────────────────────────────────
        
        int32_t bow_out    = bow_exciter.Process(env_flags);
        int32_t blow_out   = blow_exciter.Process(env_flags);
        int32_t strike_out = strike_exciter.Process(env_flags);

        // ── Smooth Strength ─────────────────────────────────────────────
        // Strength from knob + CV2
        int32_t total_strength = strength + cv2_strength;
        if (total_strength < 0) total_strength = 0;
        if (total_strength > 32767) total_strength = 32767;
        smooth_strength += (total_strength - smooth_strength) >> 4;
        
        // Accent gain from strength
        int32_t accent = AccentGainQ14(smooth_strength);

        // ── Mix Excitation ──────────────────────────────────────────────
        // Replicate the original mix logic from voice.cc:
        //
        // blow: level < 1.0 → blow * 0.4, level ≥ 1.0 → 0.4
        //       (tube level for values > 1.0, but tube is omitted for now)
        // strike: level < 1.0 → strike * level * 1.5, level ≥ 1.0 → strike * 1.5
        //         bleed: level > 1.0 → raw strike bleeds to output
        // bow: bow * bow_level * envelope * accent * 0.125

        int32_t e = mul_q15(smooth_env_value, accent);

        // Bow contribution: bow * bow_level * e * 2.5
        int32_t bow_mix = mul_q15(mul_q15(bow_out, bow_level), e);
        bow_mix = (bow_mix * 5) >> 1;

        // Blow contribution: blow * blow_level_scaled * e + tube body
        // blow_level < 0.5 (16384): scale to 0..1.0 for noise level
        // blow_level >= 0.5: noise level stays at 0.4, and tube level increases
        int32_t blow_noise_lvl;
        int32_t tube_amt = 0;
        if (blow_level < 16384) {
            blow_noise_lvl = blow_level << 1; // 0..32767
            tube_amt = 0;
        } else {
            blow_noise_lvl = 13107; // 0.4 fixed
            tube_amt = (blow_level - 16384) << 1; // 0..32767
        }
        
        int32_t b_noise = mul_q15(blow_out, blow_noise_lvl);
        b_noise = mul_q15(b_noise, e);
        
        // Process Tube (Flute Body)
        // Gain is reduced (tube_amt >> 3) to keep the Blow section from overpowering
        int32_t tube_out = tube.Process(freq_q15, smooth_env_value, damping, blow_timbre, b_noise);
        int32_t b_mix = b_noise + mul_q15(tube_out, tube_amt >> 3);

        // Strike contribution: strike * accent * strike_level_scaled + external
        // strike_level_scaled = min(strike_level * 1.25, 1.0) * 1.5
        int32_t strike_lvl_adj = mul_q15(strike_level, 40960);  // * 1.25
        if (strike_lvl_adj > 32767) strike_lvl_adj = 32767;
        int32_t strike_scaled = mul_q15(strike_lvl_adj, 49152);  // * 1.5
        int32_t strike_mix = mul_q15(mul_q15(strike_out, accent), strike_scaled);

        // Strike bleed: raw strike signal bleeds to output at high levels
        int32_t strike_bleed = 0;
        if (strike_level > 26214) {  // > 0.8 in Q15
            strike_bleed = mul_q15(strike_out, (strike_level - 26214) * 5);
        }

        // Sum all components that are always diffused (Blow, External)
        int32_t to_diffuse = b_mix + (ext_audio >> 2);
        
        // Split strike component: diffusion depends on Strike Timbre
        // Low Timbre = soft mallet (diffused), High Timbre = hard stick (direct)
        // fade = 0 (low) -> strike_to_diffuse = strike_mix
        // fade = 32767 (high) -> strike_to_diffuse = 0
        int32_t strike_to_diffuse = strike_mix - mul_q15(strike_mix, strike_timbre);
        int32_t strike_direct = strike_mix - strike_to_diffuse;
        
        to_diffuse += strike_to_diffuse;
        
        // Process the diffuser stage (advances pointers once)
        int32_t diffused_excitation = diffuser.Process(to_diffuse);
        
        // Sum all parts: Bow (direct) + Diffused (Blow/Ext/SoftStrike) + Direct Strike
        int32_t excitation = bow_mix + diffused_excitation + strike_direct;

        // DC block the excitation signal (critical for strings because strike is a positive impulse)
        excitation = DCBlockQ15(excitation, dc_ex_x, dc_ex_y);

        // ── Damping from Exciters ───────────────────────────────────────
        // Strike exciter provides damping feedback (palm mute on release)
        int32_t final_damping = total_damping_cv;
        final_damping -= mul_q15(strike_exciter.damping, strike_lvl_adj) >> 3;
        // Bow damping: when bow is not pressed, it damps
        int32_t bow_strength_inv = 32767 - mul_q15(bow_level, smooth_env_value);
        final_damping -= mul_q15(bow_strength_inv, bow_level) >> 4;
        if (final_damping < 0) final_damping = 0;

        // ── Process Resonator / String ──────────────────────────────────
        int32_t outL = 0;
        int32_t outR = 0;
        
        if (currentModel == MODEL_MODAL) {
            resonator.damping_q15 = final_damping; // use exciter-damped value
            // Calculate stereo spread from the Space parameter (matches Elements)
            // space_adj = max(0, space - 0.1)
            int32_t space_adj = (space > 3277) ? (space - 3277) : 0;
            int32_t spread = space_adj;
            if (spread > 22938) spread = 22938; // Max spread is 0.7
            
            int32_t bow_strength_q15 = mul_q15(bow_level, smooth_env_value);
            int32_t res_center = 0;
            int32_t res_sides = 0;
            resonator.Process1(bow_strength_q15, excitation, res_center, res_sides);
            
            // Stereo output matches original part.cc:
            //   L = center + sides * spread
            //   R = center - sides * spread
            int32_t side_signal = mul_q15(res_sides, spread);
            outL = res_center + side_signal;
            outR = res_center - side_signal;
        } else {
            // String models
            int32_t s_center = 0;
            int32_t s_sides = 0;
            int32_t num_strings = (currentModel == MODEL_STRINGS) ? 3 : 1;
            
            // Normalize input
            int32_t string_in = (num_strings == 3) ? (excitation >> 2) : excitation;
            
            for (int i = 0; i < num_strings; i++) {
                int32_t c = 0, s = 0;
                strings[i].Process(string_in, c, s);
                s_center += c;
                s_sides += s;
            }
            
            // Calculate stereo spread from Space
            int32_t space_adj = (space > 3277) ? (space - 3277) : 0;
            int32_t spread = space_adj;
            if (spread > 22938) spread = 22938;
            
            int32_t side_signal = mul_q15(s_sides, spread);
            outL = s_center - side_signal;
            outR = s_center + side_signal;
        }
        
        // Add strike bleed to both channels
        outL += strike_bleed;
        outR += strike_bleed;
        
        // Soft-limit before reverb to prevent digital harshness
        outL = SoftLimitQ15(outL);
        outR = SoftLimitQ15(outR);
        
        // ── Process Reverb ──────────────────────────────────────────────
        // Reverb Macro (Page 4 Y):
        // As the knob turns, it increases both the wet mix and the tail length.
        
        // Mix: 0 to 100% (allows fully washed-out drone textures)
        int32_t rev_mix = reverb_amt;
        
        // Decay (Tail Length): Ramps from a tight room (0.3) to a massive cavern (0.99)
        // 0.3 = 9830. 0.99 = 32440. Range = 22610.
        int32_t rev_decay = 9830 + mul_q15(reverb_amt, 22610);
        
        // LP damping: static 0.5 (16384) provides a good balance of shimmer and warmth,
        // avoiding the muddy buildup of darker plates.
        int32_t rev_damp = 16384; 
        
        reverb.Process(outL, outR, rev_mix, rev_decay, rev_damp);

        // Soft clip
        outL = SoftClipQ15(outL);
        outR = SoftClipQ15(outR);

        // ── Send Results Back ───────────────────────────────────────────
        multicore_fifo_push_blocking((uint32_t)outL);
        multicore_fifo_push_blocking((uint32_t)outR);
    }
}

// ── Main Application ────────────────────────────────────────────────────────

class Modal : public ComputerCard {
public:
    // ── UI State ────────────────────────────────────────────────────────
    int currentPage = 0;            // Currently active parameter page (0–5)
    KnobLock lockMain, lockX, lockY;  // Knob locks for page switching
    int32_t smoothMain = 0, smoothX = 0, smoothY = 0;  // Smoothed knob values

    // ── Switch State ────────────────────────────────────────────────────
    uint32_t switchDownTimer = 0;   // How long switch has been held down
    bool switchHandled = false;     // Has the current press been handled?
    bool switchInit = false;        // Has switch been initialized?
    bool lastSwitchUp = false;      // Previous state of switch-up
    Switch lastSwitch = Switch::Middle;  // Debounced switch state
    uint32_t debounceTimer = 0;     // Switch debounce counter

    // ── Gate State ──────────────────────────────────────────────────────
    bool previousGate = false;      // Gate state from last DSP cycle
    bool triggerBuffered = false;   // Buffered trigger from Pulse In 1

    // ── Audio State ─────────────────────────────────────────────────────
    int32_t dspOutL = 0, dspOutR = 0;  // Latest output from Core 1
    int gPhase = 0;                    // Phase counter for 2:1 decimation
    uint32_t bootSilence = BOOT_SILENCE_SAMP;  // Startup mute countdown

    // ── CV Smoothing ────────────────────────────────────────────────────
    int32_t cv1_acc = 0;            // Smoothed CV1 accumulator
    int32_t cv2_acc = 0;            // Smoothed CV2 accumulator

    // ── DC Blockers ─────────────────────────────────────────────────────
    int32_t dc_bxL = 0, dc_byL = 0, dc_bxR = 0, dc_byR = 0;  // Input
    int32_t dc_oxL = 0, dc_oyL = 0, dc_oxR = 0, dc_oyR = 0;  // Output

    // ── Page Display Timer ──────────────────────────────────────────────
    uint32_t pageDisplayTimer = 0;  // Countdown for page display LED flash
    static const uint32_t PAGE_DISPLAY_DURATION = 24000;  // ~500ms at 48kHz

    // ────────────────────────────────────────────────────────────────────
    //  Constructor — Initialize all parameters to musical defaults
    // ────────────────────────────────────────────────────────────────────

    Modal() {
        // Page 0: Strike (Level, Timbre, Meta) — Mallet, mid timbre, 50% level
        params[0] = {16384, 16384, 16384};

        // Page 1: Blow (Level, Timbre, Meta) — Off by default
        params[1] = {0, 16384, 16384};

        // Page 2: Bow & Env (Level, Timbre, Shape) — AD envelope
        params[2] = {0, 16384, 8192};

        // Page 3: Resonator Core (Geometry, Brightness, Damping)
        // Matching original Elements defaults: geometry=0.2, brightness=0.5, damping=0.25
        params[3] = {6554, 16384, 8192};

        // Page 4: Resonator Space (Position, Space)
        params[4] = {9830, 8192, 0};  // Position=0.3, Space=0.25 (light reverb)

        // Page 5: Performance (Pitch Coarse, Fine Tune, Strength)
        params[5] = {16384, 16384, 16384};
    }

    // ────────────────────────────────────────────────────────────────────
    //  BackgroundLoop — runs on Core 0 when not in ProcessSample ISR
    //  Used for slow UI updates (LED display, flash operations, etc.)
    //  ~5000 iterations per meaningful update (matches grains pattern).
    // ────────────────────────────────────────────────────────────────────

    void BackgroundLoop() override {
        static uint32_t loopCount = 0;
        if (++loopCount < 5000) return;
        loopCount = 0;

        // ── LED Display ─────────────────────────────────────────────────
        // Show current page indicator. During the first 500ms after a page
        // change, the page LED is bright. After that, it dims to a subtle
        // indicator and other LEDs show activity meters.

        if (pageDisplayTimer > 0) {
            // Page just changed — show page number prominently
            for (int i = 0; i < 6; i++) {
                LedOn(i, i == currentPage);
            }
        } else {
        // ── Normal Operation LED Display ───────────────────────────────
        if (pageDisplayTimer == 0) {
            // Display only the current page (LED 0-5)
            for (int i = 0; i < 6; i++) {
                int32_t b = (i == currentPage) ? 1800 : 0;
                
                // Pulse Display (Gate activity) on LED 4
                // Adds on top of the page indicator if we are on Page 4
                if (i == 4 && previousGate) {
                    b += 1500; // Slightly stronger pulse
                }
                
                if (b > 4095) b = 4095;
                if (b > 0) {
                    LedBrightness(i, b);
                } else {
                    LedOff(i);
                }
            }
        }
        }
    }

    // ────────────────────────────────────────────────────────────────────
    //  ProcessSample — runs at 48kHz in interrupt context
    //  MUST complete within ~20μs. No allocations, no blocking.
    // ────────────────────────────────────────────────────────────────────

    void __not_in_flash_func(ProcessSample)() override {
        // ── Boot Silence ────────────────────────────────────────────────
        // Mute outputs for the first second to let hardware settle.
        // Show a chasing LED animation during boot.
        if (bootSilence > 0) {
            bootSilence--;
            AudioOut1(0);
            AudioOut2(0);

            int chaseIdx = (BOOT_SILENCE_SAMP - bootSilence) / 4000;
            for (int i = 0; i < 6; i++) {
                LedOn(i, i == (chaseIdx % 6));
            }
            if (bootSilence == 0) {
                for (int i = 0; i < 6; i++) LedOff(i);
            }
            return;
        }

        // ── Poll Triggers ───────────────────────────────────────────────
        // Check pulse inputs at full 48kHz rate to avoid missing triggers.
        if (PulseIn1RisingEdge()) {
            triggerBuffered = true;
        }

        // ── Read Raw Inputs ─────────────────────────────────────────────
        Switch sw = SwitchVal();
        int32_t rawMain = KnobVal(Knob::Main);
        int32_t rawX    = KnobVal(Knob::X);
        int32_t rawY    = KnobVal(Knob::Y);

        // Scale knobs to Q15 range
        int32_t kMain = knob_to_q15(rawMain);
        int32_t kX    = knob_to_q15(rawX);
        int32_t kY    = knob_to_q15(rawY);

        // Smooth knobs with simple IIR (>>2 = 0.25 coefficient)
        smoothMain += (kMain - smoothMain) >> 2;
        smoothX    += (kX    - smoothX)    >> 2;
        smoothY    += (kY    - smoothY)    >> 2;

        // Smooth CV1 for V/Oct pitch tracking
        // CV is ±2048 (12-bit signed). Check if connected to avoid noise drift.
        int32_t cv1_raw = Connected(ComputerCard::CV1) ? CVIn1() : 0;
        
        // Heavy IIR smoothing for stable pitch. Steady state cv1_acc ≈ cv1_raw * 256.
        cv1_acc = cv1_acc - (cv1_acc >> 8) + cv1_raw;
        int32_t cv1_smoothed = cv1_acc >> 8;
        
        // 1 Volt ≈ 410 raw. 1 Octave = 3072 in Q8.
        // Scale = 3072 / 410 = 7.49.
        // Multiply by 7.5: cv1_pitch_q8 = (cv1_smoothed * 15) / 2
        cv1_pitch_q8 = (cv1_smoothed * 15) >> 1;

        // CV2 for strength/accent modulation
        int32_t cv2_raw = Connected(ComputerCard::CV2) ? CVIn2() : 0;
        cv2_acc = cv2_acc - (cv2_acc >> 4) + cv2_raw;
        cv2_strength = (cv2_acc >> 4) << 3;  // Scale 12-bit to ~Q15

        // Buffer external audio inputs for Core 1 (scale 12-bit to Q15)
        audio_in1_q15 = AudioIn1() << 4;  // ±2047 → ±32752
        audio_in2_q15 = AudioIn2() << 4;

        // ── Switch Debouncing & Page Navigation ─────────────────────────
        // Debounce the switch with a 500-sample (~10ms) window.
        Switch effectiveSwitch = sw;
        if (sw != lastSwitch) {
            if (++debounceTimer < 500) {
                effectiveSwitch = lastSwitch;
            } else {
                lastSwitch = sw;
                debounceTimer = 0;
            }
        } else {
            debounceTimer = 0;
        }

        // ── Switch Down: Page Cycling ───────────────────────────────────
        // Tap (<312ms) = next page. Hold (>312ms) = previous page.
        if (effectiveSwitch == Switch::Down) {
            switchDownTimer++;
        } else {
            if (switchDownTimer > 500) {  // Minimum 10ms to count as press
                if (!switchHandled) {
                    if (switchDownTimer < 15000) {
                        // Short tap: advance to next page
                        currentPage = (currentPage + 1) % NUM_PAGES;
                    } else {
                        // Long press: go to previous page
                        currentPage = (currentPage + NUM_PAGES - 1) % NUM_PAGES;
                    }
                    // Lock all knobs to prevent parameter jumps
                    lockMain.engage(smoothMain);
                    lockX.engage(smoothX);
                    lockY.engage(smoothY);
                    // Start page display timer for visual feedback
                    pageDisplayTimer = PAGE_DISPLAY_DURATION;
                }
                switchHandled = true;
            }
            if (effectiveSwitch == Switch::Middle) {
                switchDownTimer = 0;
                switchHandled = false;
            }
        }

        // ── Switch Up: Model Toggle ─────────────────────────────────────
        // Toggle up cycles through resonator models: Modal → String → Strings
        bool swUp = (effectiveSwitch == Switch::Up);
        if (!switchInit) {
            lastSwitchUp = swUp;
            switchInit = true;
        } else if (swUp && !lastSwitchUp) {
            currentModel = (currentModel + 1) % MODEL_COUNT;
        }
        lastSwitchUp = swUp;

        // Also allow Pulse In 2 to toggle model
        if (PulseIn2RisingEdge()) {
            currentModel = (currentModel + 1) % MODEL_COUNT;
        }

        // ── Page Display Timer ──────────────────────────────────────────
        if (pageDisplayTimer > 0) pageDisplayTimer--;

        // ── Parameter Updates with Knob Locking ─────────────────────────
        // Only update the current page's parameters if the knob has been
        // moved past the lock threshold. This prevents parameter jumps
        // when switching between pages.
        if (lockMain.update(smoothMain)) {
            params[currentPage].pMain = smoothMain;
        }
        if (lockX.update(smoothX)) {
            params[currentPage].pX = smoothX;
        }
        if (lockY.update(smoothY)) {
            params[currentPage].pY = smoothY;
        }

        // ── DSP Engine Communication (24kHz) ────────────────────────────
        // Every 2nd sample, exchange data with Core 1 via multicore FIFO.
        // This effectively runs the DSP engine at 24kHz while maintaining
        // the 48kHz sample rate for input polling and output.

        if (++gPhase >= 2) {
            gPhase = 0;

            // Build gate flags
            uint32_t flags = FIFO_FLAG_ACTIVE;
            bool gateNow = PulseIn1() || triggerBuffered;

            if (gateNow)               flags |= FIFO_FLAG_GATE;
            if (gateNow && !previousGate)  flags |= FIFO_FLAG_RISING;
            if (!gateNow && previousGate)  flags |= FIFO_FLAG_FALLING;

            previousGate = gateNow;
            triggerBuffered = false;

            // 1. Receive processed audio from Core 1 (from the PREVIOUS 24kHz period)
            // By popping before pushing, we don't force Core 1 to finish within this 20.8us ISR.
            // It gets a full 41.6us to compute. If it's not ready, we reuse the old output (preventing a hard drop).
            if (multicore_fifo_rvalid()) {
                int32_t rawL = (int32_t)multicore_fifo_pop_blocking();
                int32_t rawR = (int32_t)multicore_fifo_pop_blocking();
                // DC block the output to prevent drift
                dspOutL = dc_block(rawL, dc_oxL, dc_oyL);
                dspOutR = dc_block(rawR, dc_oxR, dc_oyR);
            }

            // 2. Send work to Core 1 to start processing the NEXT period
            multicore_fifo_push_blocking(flags);
        }

        // ── Audio Output ────────────────────────────────────────────────
        // Scale from Q15 (±32767) to 12-bit DAC range (±2047) and clamp.

        int32_t outL = dspOutL >> 4;
        int32_t outR = dspOutR >> 4;

        if (outL >  2047) outL =  2047;
        if (outL < -2048) outL = -2048;
        if (outR >  2047) outR =  2047;
        if (outR < -2048) outR = -2048;

        AudioOut1((int16_t)outL);
        AudioOut2((int16_t)outR);

        // ── CV Outputs (meters) ─────────────────────────────────────────
        // TODO Phase 5: Output exciter/resonator levels as CV
        CVOut1(0);
        CVOut2(0);

        // ── Pulse Outputs ───────────────────────────────────────────────
        // Pulse Out 1: Gate passthrough
        PulseOut1(previousGate);
        // Pulse Out 2: Reserved for envelope EOC (Phase 5)
        PulseOut2(false);
    }
};

// ── Entry Point ─────────────────────────────────────────────────────────────

int main() {
    // Overclock to 240MHz for maximum DSP headroom
    // (matching 51_grains pattern)
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);
    set_sys_clock_khz(240000, true);

    // Launch Core 1 DSP engine before starting the audio interrupt
    multicore_launch_core1(core1_dsp_loop);

    // Create and run the main application
    Modal modal;
    modal.EnableNormalisationProbe();
    modal.Run();  // Never returns — ProcessSample runs in ISR at 48kHz
}
