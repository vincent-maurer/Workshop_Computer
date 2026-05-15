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
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <string.h>

// ── Fixed-point DSP infrastructure ──────────────────────────────────────────
#include "dsp_q15.h"
#include "svf_q15.h"
#include "envelope_q15.h"

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
// Page 0: Exciter Levels     — Strike Level / Blow Level / Bow Level
// Page 1: Exciter Timbre     — Strike Timbre / Blow Timbre / Bow Timbre
// Page 2: Exciter Shape      — Strike Meta / Blow Meta / Envelope Shape
// Page 3: Resonator Core     — Geometry / Brightness / Damping
// Page 4: Resonator Space    — Position / Space (reverb) / Model Select
// Page 5: Performance        — Pitch (coarse) / Strength / Fine Tune

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
volatile int32_t    cv1_pitch_q16  = 0;   // V/Oct pitch from CV In 1 (Q16)
volatile int32_t    cv2_strength   = 0;   // Strength from CV In 2 (Q15)
volatile int32_t    audio_in1_q15  = 0;   // Blow external input (Q15)
volatile int32_t    audio_in2_q15  = 0;   // Strike external input (Q15)
volatile int32_t    currentModel   = MODEL_MODAL;

// ── Core 1 DSP Engine ──────────────────────────────────────────────────────
// This function runs on Core 1 in an infinite loop. It waits for trigger
// words from Core 0, processes one block of DSP, and sends audio back.
//
// Currently a placeholder that generates silence. The actual DSP will be
// added in Phases 2-4 (exciter, resonator, reverb).

static void __not_in_flash_func(core1_dsp_loop)() {
    while (true) {
        // Wait for work from Core 0
        uint32_t flags = multicore_fifo_pop_blocking();

        if (!(flags & FIFO_FLAG_ACTIVE)) {
            // Not a real work packet — just echo silence
            multicore_fifo_push_blocking(0);
            multicore_fifo_push_blocking(0);
            continue;
        }

        // ── Read Parameters ─────────────────────────────────────────────
        // Page 0: Exciter Levels
        int32_t strike_level = params[0].pMain;
        int32_t blow_level   = params[0].pX;
        int32_t bow_level    = params[0].pY;

        // Page 1: Exciter Timbre
        int32_t strike_timbre = params[1].pMain;
        int32_t blow_timbre   = params[1].pX;
        int32_t bow_timbre    = params[1].pY;

        // Page 2: Exciter Shape
        int32_t strike_meta  = params[2].pMain;
        int32_t blow_meta    = params[2].pX;
        int32_t env_shape    = params[2].pY;

        // Page 3: Resonator Core
        int32_t geometry     = params[3].pMain;
        int32_t brightness   = params[3].pX;
        int32_t damping      = params[3].pY;

        // Page 4: Resonator Space
        int32_t position     = params[4].pMain;
        int32_t space        = params[4].pX;
        // int32_t model_sel = params[4].pY;  // Read via currentModel

        // Page 5: Performance
        int32_t pitch_coarse = params[5].pMain;
        int32_t strength     = params[5].pX;
        int32_t fine_tune    = params[5].pY;

        // External inputs
        int32_t ext_blow   = audio_in1_q15;
        int32_t ext_strike = audio_in2_q15;
        int32_t pitch_cv   = cv1_pitch_q16;

        // Gate state
        bool gate    = (flags & FIFO_FLAG_GATE)    != 0;
        bool rising  = (flags & FIFO_FLAG_RISING)  != 0;
        bool falling = (flags & FIFO_FLAG_FALLING) != 0;

        // ── Suppress unused variable warnings during skeleton phase ─────
        (void)strike_level; (void)blow_level; (void)bow_level;
        (void)strike_timbre; (void)blow_timbre; (void)bow_timbre;
        (void)strike_meta; (void)blow_meta; (void)env_shape;
        (void)geometry; (void)brightness; (void)damping;
        (void)position; (void)space;
        (void)pitch_coarse; (void)strength; (void)fine_tune;
        (void)ext_blow; (void)ext_strike; (void)pitch_cv;
        (void)gate; (void)rising; (void)falling;

        // ── TODO: DSP Processing ────────────────────────────────────────
        // Phase 2: Exciter → generates excitation signal
        // Phase 3: Resonator → filters excitation into pitched output
        // Phase 4: Space/Reverb → adds stereo width and ambience
        //
        // For now, pass through a scaled version of external inputs
        // so we can verify the audio path works.

        int32_t outL = ext_blow >> 1;    // Attenuated passthrough
        int32_t outR = ext_strike >> 1;

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
        // Page 0: Exciter Levels — Start with strike only (classic mallet)
        params[0] = {16384, 0, 0};  // Strike=50%, Blow=0, Bow=0

        // Page 1: Exciter Timbre — Mid brightness
        params[1] = {16384, 16384, 16384};

        // Page 2: Exciter Shape — Mallet mode, mid envelope
        params[2] = {16384, 0, 8192};

        // Page 3: Resonator Core — Classic modal settings
        params[3] = {6554, 9830, 22938};  // Geometry=0.2, Bright=0.3, Damp=0.7

        // Page 4: Resonator Space — Centered, moderate space
        params[4] = {9830, 16384, 0};  // Position=0.3, Space=0.5, Model=modal

        // Page 5: Performance — Middle pitch, moderate strength
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
            // Normal operation — dim page indicator + activity meters
            for (int i = 0; i < 6; i++) {
                if (i == currentPage) {
                    LedBrightness(i, 512);   // Dim current page indicator
                } else {
                    LedOff(i);
                }
            }

            // LED 4: Gate activity (follows Pulse In 1)
            LedOn(4, previousGate);

            // LED 5: Resonator model indicator
            //   Off     = Modal
            //   On      = String
            //   Blink   = Strings (chord)
            if (currentModel == MODEL_MODAL) {
                if (currentPage != 5) LedOff(5);
            } else if (currentModel == MODEL_STRING) {
                LedOn(5);
            } else {
                // Blink for Strings mode (toggle every background loop)
                static bool blink = false;
                blink = !blink;
                LedOn(5, blink);
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
        // CV is ±2048 (12-bit signed). Scale to Q16 for pitch computation.
        // 160 = ~8 semitones per volt scaling factor
        cv1_acc = (cv1_acc * 127 + (CVIn1() * 160 * 16)) >> 7;
        cv1_pitch_q16 = cv1_acc >> 4;

        // CV2 for strength/accent modulation
        cv2_strength = CVIn2() << 3;  // Scale 12-bit to ~Q15

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

            // Send work to Core 1
            multicore_fifo_push_blocking(flags);

            // Receive processed audio from Core 1
            int32_t rawL = (int32_t)multicore_fifo_pop_blocking();
            int32_t rawR = (int32_t)multicore_fifo_pop_blocking();

            // DC block the output to prevent drift
            dspOutL = dc_block(rawL, dc_oxL, dc_oyL);
            dspOutR = dc_block(rawR, dc_oxR, dc_oyR);
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
    // Overclock to 144MHz for ADC artifact reduction and more DSP headroom
    set_sys_clock_khz(144000, true);

    // Launch Core 1 DSP engine before starting the audio interrupt
    multicore_launch_core1(core1_dsp_loop);

    // Create and run the main application
    Modal modal;
    modal.EnableNormalisationProbe();
    modal.Run();  // Never returns — ProcessSample runs in ISR at 48kHz
}
