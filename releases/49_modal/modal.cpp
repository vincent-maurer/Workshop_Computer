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
#include <cstdlib>
#include <string.h>

#include "tusb.h"
#include "usb_midi_host.h"
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/flash.h>
#include <pico/unique_id.h>
#include <stdio.h>

// ── Fixed-point DSP infrastructure ──────────────────────────────────────────
#include "diffuser_q15.h"
#include "dsp_q15.h"
#include "envelope_q15.h"
#include "exciter_q15.h"
#include "plate_reverb_q15.h"
#include "resonator_q15.h"
#include "string_q15.h"
#include "svf_q15.h"
#include "tube_q15.h"

// ── Constants ───────────────────────────────────────────────────────────────

#define SAMPLE_RATE 48000
#define DSP_RATE 24000       // Effective DSP rate (every 2nd sample)
#define NUM_PAGES 6          // Number of parameter pages
#define BOOT_SILENCE_MS 1000 // Startup mute duration in samples
#define BOOT_SILENCE_SAMP (SAMPLE_RATE * BOOT_SILENCE_MS / 1000)

// ── Fixed-point helpers ─────────────────────────────────────────────────────
// mul_q15() and other core Q15 ops are in dsp_q15.h.
// Knob scaling is project-specific:

/// Scale a raw knob value (0–4095) to Q15 (0–32767), with dead zones at edges.
/// The pots don't reliably reach 0, so we remap ~14–4095 → 0–32767.
static inline int32_t knob_to_q15(int32_t raw) {
  int32_t v = (raw - 50) * 32767 / (4095 - 100);
  if (v < 0)
    v = 0;
  if (v > 32767)
    v = 32767;
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
      if (d < 0)
        d = -d;
      if (d > 1638)
        locked = false;
    }
    return !locked;
  }

  /// Force the knob back into a locked state (used when CC takes over)
  void relock(int32_t v) {
    locked = true;
    ref = v;
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
  volatile int32_t pMain, pX, pY;
};

// Track what we last sent over MIDI to avoid flooding and loops
struct MIDIState {
  int32_t pMain, pX, pY;
};

// ── Resonator Model ─────────────────────────────────────────────────────────
enum ResonatorModel {
  MODEL_MODAL = 0,   // Bank of bandpass filters (SVF) simulating modes
  MODEL_STRING = 1,  // Single Karplus-Strong string
  MODEL_STRINGS = 2, // Chord of 3 strings
  MODEL_BORE = 3,    // Wind style resonator (Bore)
  MODEL_COUNT = 4
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
#define FIFO_FLAG_ACTIVE 0x00000001    // "I'm sending you work"
#define FIFO_FLAG_GATE 0x00000002      // Gate is high (Pulse In 1)
#define FIFO_FLAG_RISING 0x00000004    // Gate just went high
#define FIFO_FLAG_FALLING 0x00000008   // Gate just went low
#define FIFO_FLAG_MODEL_CHG 0x00000010 // Resonator model changed

// ── Shared State (volatile for cross-core access) ───────────────────────────

// Parameters are written by Core 0 and read by Core 1.
// Cross-core access is safe because writes are atomic at the word level
// and we only write from Core 0 / read from Core 1.
PageParams params[NUM_PAGES];
MIDIState last_sent[NUM_PAGES];
volatile int32_t cv1_pitch_q8 = 0;  // V/Oct pitch from CV In 1 (Q8)
volatile int32_t cv2_strength = 0;  // Strength from CV In 2 (Q15)
volatile int32_t audio_in1_q15 = 0; // Blow external input (Q15)
volatile int32_t audio_in2_q15 = 0; // Strike external input (Q15)
volatile int32_t currentModel = MODEL_MODAL;

// ── MIDI State ──────────────────────────────────────────────────────────────
volatile int32_t midi_pitch_q8 = 0; // MIDI note pitch (Q8)
volatile bool midi_trigger = false; // MIDI note-on trigger
volatile bool midi_gate = false;    // MIDI gate state
uint8_t midi_dev_addr = 0;          // Current MIDI device address
bool isUSBMIDIHost = false;         // Are we acting as a host?
ComputerCard::USBPowerState_t usb_power_state = ComputerCard::Unsupported;

// ── Preset System ───────────────────────────────────────────────────────────
#define FLASH_PRESET_OFFSET (1536 * 1024) // 1.5MB offset (plenty of room)
#define PRESET_COUNT 16
#define PRESET_MAGIC 0x4D4F4441 // "MODA"

struct Preset {
  uint32_t magic;
  uint32_t model;
  uint32_t sequence;   // 16-step bitmask
  uint8_t pitches[16]; // 16 pitches
  int32_t bpm_val;
  int32_t density_val;
  int32_t map_val;
  uint8_t scale;
  uint8_t root;
  PageParams pages[NUM_PAGES];
  uint32_t reserved[16];
};

enum UIState { STATE_NORMAL, STATE_LOAD_MENU, STATE_SAVE_MENU, STATE_GEN_SEQ };

volatile UIState currentUIState = STATE_NORMAL;

// ── Generative Sequencer State ──────────────────────────────────────────────
static int gen_step = 0;
static uint32_t gen_sequence = 0xAAAA; // 16-step bitmask
static uint8_t gen_pitches[16];
static int32_t gen_cv2_val = 0;
static bool gen_gate = false;
static bool gen_gate_pending = false;
static bool gen_pulse_clk = false;
static int32_t gen_melody_q8 = 0;
static int32_t seq_bpm_val = 16384;
static int32_t seq_density_val = 16384;
static int32_t seq_map_val = 0;
static int32_t last_seq_dens = -1, last_seq_bpm = -1, last_seq_ent = -1;
static uint32_t last_cc_tx_time[128] = {0};
static uint8_t currentScale =
    0; // 0=PentMinor, 1=PentMajor, 2=Minor, 3=Major, 4=Chromatic
static uint8_t currentRoot = 0; // 0=C, 1=C#, etc.
int32_t presetMenuSlot = 0;

// MIDI CC mappings for each page [Page][Knob]
static const uint8_t PageCCs[6][3] = {
    {12, 13, 14}, // Page 0: Strike
    {15, 16, 17}, // Page 1: Blow
    {18, 19, 20}, // Page 2: Bow
    {70, 74, 71}, // Page 3: Resonator 1
    {10, 21, 23}, // Page 4: Resonator 2 (Pos, Space, Reverb)
    {11, 22, 1}   // Page 5: Perf (Coarse, Fine, Strength)
};

// ── Core 1 DSP Engine ──────────────────────────────────────────────────────
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
static DiffuserQ15 diffuser;
static TubeQ15 tube;
static PlateReverbQ15 reverb; // Moved to Core 0 context logically

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
  for (int i = 0; i < 3; i++)
    strings[i].Init();
  diffuser.Init();
  tube.Init();

  // Set default exciter models
  bow_exciter.model = EXCITER_Q15_FLOW;
  bow_exciter.parameter = 22938; // ~0.7
  bow_exciter.timbre = 16384;    // ~0.5

  blow_exciter.model = EXCITER_Q15_NOISE;

  strike_exciter.model = EXCITER_Q15_MALLET;

  uint32_t prng_seed = 0x12345678;
  int32_t random_pitch = 0;
  int32_t random_timbre = 0;

  while (1) {
    uint32_t flags = multicore_fifo_pop_blocking();
    if (!(flags & FIFO_FLAG_ACTIVE)) {
      multicore_fifo_push_blocking(0);
      multicore_fifo_push_blocking(0);
      continue;
    }

    // Simple PRNG for humanization
    auto get_rand = [&]() {
      prng_seed = prng_seed * 1103515245 + 12345;
      return (int32_t)((prng_seed >> 16) & 0x7FFF);
    };

    // ── Read Parameters ─────────────────────────────────────────────
    // Page 0: Strike
    int32_t strike_level = params[0].pMain;
    int32_t strike_timbre = params[0].pX;
    int32_t strike_meta = params[0].pY;

    // Page 1: Blow
    int32_t blow_level = params[1].pMain;
    int32_t blow_timbre = params[1].pX;
    int32_t blow_meta = params[1].pY;

    // Page 2: Bow & Envelope
    int32_t bow_level = params[2].pMain;
    int32_t bow_timbre = params[2].pX;
    int32_t env_shape = params[2].pY;

    // Page 3: Resonator Core
    int32_t geometry = params[3].pMain;
    int32_t brightness = params[3].pX;
    int32_t damping = params[3].pY;

    // Page 4: Resonator Space
    int32_t position = params[4].pMain;

    // Page 5: Performance
    int32_t pitch_coarse = params[5].pMain;
    int32_t fine_tune = params[5].pX;
    int32_t strength = params[5].pY;

    // External inputs
    int32_t ext_audio = audio_in1_q15;  // Mono audio input for excitation
    int32_t cv_damping = audio_in2_q15; // Use Audio 2 as CV for Damping
    int32_t pitch_cv = cv1_pitch_q8;
    int32_t cv_bright =
        cv2_strength; // Use CV 2 as CV for Brightness modulation

    // Gate state
    bool gate = (flags & FIFO_FLAG_GATE) != 0;
    bool rising = (flags & FIFO_FLAG_RISING) != 0;
    bool falling = (flags & FIFO_FLAG_FALLING) != 0;

    // On every trigger, generate slight deviations for more organic sound
    if (rising) {
      // Pitch: ±8 units (approx ±3 cents)
      random_pitch = (get_rand() & 0x1F) - 16;
      // Timbre: ±400 units (approx 1.2%)
      random_timbre = (get_rand() & 0x3FF) - 512;
    }

    // Suppress unused warnings for parameters not yet wired

    // ── Build gate flags for exciters/envelope ──────────────────────
    uint8_t env_flags = 0;
    if (gate)
      env_flags |= ENV_FLAG_GATE;
    if (rising)
      env_flags |= ENV_FLAG_RISING;
    if (falling)
      env_flags |= ENV_FLAG_FALLING;

    // ── Configure Envelope ──────────────────────────────────────────
    // env_shape (Q15): 0..13107 = AD, 13107..19660 = ADSR, 19660..32767 = AR
    // Maps the original Elements envelope shape parameter
    if (env_shape < 13107) {
      // Short AD shapes (0..0.4): attack + decay, no sustain
      // a = shape*0.75 + 0.15 → Q15: shape*24576/32767 + 4915
      int32_t a = mul_q15(env_shape, 24576) + 4915;
      int32_t dr = mul_q15(a, 29491) << 1; // a * 1.8
      if (dr > 32767)
        dr = 32767;
      envelope.SetADR(a, dr, 0, dr);
    } else if (env_shape < 19660) {
      // ADSR with increasing sustain (0.4..0.6)
      int32_t s = (env_shape - 13107) * 5; // scale to 0..32767
      if (s > 32767)
        s = 32767;
      envelope.SetADSR(14746, 26542, s, 26542); // a=0.45, dr=0.81
    } else {
      // Long AR shapes (0.6..1.0): full sustain
      int32_t a = mul_q15(32767 - env_shape, 24576) + 4915;
      int32_t dr = mul_q15(a, 29491) << 1;
      if (dr > 32767)
        dr = 32767;
      envelope.SetADSR(a, dr, 32767, dr);
    }

    // Process envelope
    int32_t env_value = envelope.Process(env_flags);

    // Smooth envelope to avoid zipper noise
    smooth_env_value += (env_value - smooth_env_value) >> 3;

    // ── CV Modulations ──────────────────────────────────────────────
    // Map CV2 to Brightness and Audio2 to Damping
    // Shift right by 1 to make the modulation depth sensible (50%)
    int32_t total_brightness = brightness + (cv_bright >> 1) + random_timbre;
    if (total_brightness < 0)
      total_brightness = 0;
    if (total_brightness > 32767)
      total_brightness = 32767;

    int32_t total_damping_cv = damping + (cv_damping >> 1);
    if (total_damping_cv < 0)
      total_damping_cv = 0;
    if (total_damping_cv > 32767)
      total_damping_cv = 32767;

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
      resonator.resolution = kMaxModesQ15;
      resonator.structure = ResonatorQ15::STRUC_MODAL;
    } else if (model == MODEL_BORE) {
      resonator.resolution = kMaxModesQ15;
      resonator.structure = ResonatorQ15::STRUC_WIND;
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

    // Embouchure pitch modulation for Wind instruments (Bore mode)
    // Harder blowing (env value) slightly increases pitch, mimicking
    // overblowing tension
    if (currentModel == MODEL_BORE) {
      midi_q8 += (smooth_env_value * 300) >> 15; // Up to ~1.2 semitones sharp
    }

    // Clamp to valid MIDI range
    if (midi_q8 < 0)
      midi_q8 = 0; // MIDI 0 (~8Hz)
    if (midi_q8 > 30720)
      midi_q8 = 30720; // MIDI 120 (C9)

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
    if (freq_q15 > 16056)
      freq_q15 = 16056; // cap at 0.49 Nyquist
    if (freq_q15 < 1)
      freq_q15 = 1;
    resonator.frequency_q15 = freq_q15;

    // Apply frequencies to strings, including chord offsets
    // chords_table offsets are in semitones. 1 semitone = 256 in Q8.
    int32_t num_strings = (currentModel == MODEL_STRINGS) ? 3 : 1;
    for (int i = 0; i < num_strings; i++) {
      int32_t string_midi = midi_q8;
      if (num_strings == 3) {
        // For discrete chord selection, use kMain (raw-ish) to avoid sluggish
        // jumps but keep geometry (smoothed) for continuous SetDispersion
        // below.
        int32_t chord_idx = (geometry * 11) >> 15;
        if (chord_idx > 10)
          chord_idx = 10;

        static const int16_t chord_offsets[11][3] = {
            {0, -12 * 256, 12 * 256}, // Octaves
            {0, -12 * 256, 3 * 256},  // Minor
            {0, -12 * 256, 7 * 256},  // Minor 7
            {0, 3 * 256, 14 * 256},   // Minor 9
            {0, 3 * 256, 17 * 256},   // Minor 11
            {0, -12 * 256, 19 * 256}, // Power chord stack
            {0, 4 * 256, 17 * 256},   // Major 11
            {0, 4 * 256, 14 * 256},   // Major 9
            {0, 4 * 256, 7 * 256},    // Major
            {0, 4 * 256, 11 * 256},   // Major 7
            {0, 5 * 256, 7 * 256}     // Sus4
        };
        string_midi += chord_offsets[chord_idx][i];
      }
      if (string_midi < 0)
        string_midi = 0;
      if (string_midi > 30720)
        string_midi = 30720;
      // MidiToIncrementU32 handles the LUT offset internally
      uint32_t s_inc = MidiToIncrementU32(string_midi);
      strings[i].SetFrequency(s_inc);
    }

    // ── Configure Exciters ──────────────────────────────────────────

    // Brightness factor: resonator brightness modulates exciter timbre
    // brightness_factor = 0.4 + 0.6 * brightness → Q15: 13107 + brightness *
    // 0.6
    int32_t brightness_factor = 13107 + mul_q15(brightness, 19661);

    // Bow: Flow model, timbre controlled by bow_timbre * brightness
    bow_exciter.timbre = mul_q15(bow_timbre, brightness_factor);
    bow_exciter.model = EXCITER_Q15_FLOW;
    bow_exciter.parameter = 22938; // ~0.7 turbulence

    // Blow: Granular sample player (matching original)
    blow_exciter.parameter = blow_meta;
    blow_exciter.timbre = blow_timbre;
    blow_exciter.signature =
        blow_meta; // Tie signature to meta for texture variation
    blow_exciter.model = EXCITER_Q15_GRANULAR;

    // Strike: Use meta to select model (Sample→Mallet→Plectrum→Particles)
    // strike_meta <= 0.4: scale to 0..0.25 range (sample player region)
    // strike_meta > 0.4: scale to 0.25..1.0 range (synth models)
    int32_t adjusted_meta;
    if (strike_meta <= 13107) {
      adjusted_meta = mul_q15(strike_meta, 20480); // * 0.625
    } else {
      adjusted_meta = mul_q15(strike_meta, 40960) - 8192; // * 1.25 - 0.25
    }
    if (adjusted_meta < 0)
      adjusted_meta = 0;
    if (adjusted_meta > 32767)
      adjusted_meta = 32767;
    strike_exciter.SetMeta(adjusted_meta, EXCITER_Q15_SAMPLE,
                           EXCITER_Q15_PARTICLES);
    strike_exciter.timbre = strike_timbre;
    strike_exciter.signature = strike_meta; // Tie signature to meta

    // ── Process Exciters ────────────────────────────────────────────

    int32_t bow_out = bow_exciter.Process(env_flags);
    int32_t blow_out = blow_exciter.Process(env_flags);
    int32_t strike_out = strike_exciter.Process(env_flags);

    // ── Smooth Strength ─────────────────────────────────────────────
    // Strength from knob + CV2
    int32_t total_strength = strength + cv2_strength;
    if (total_strength < 0)
      total_strength = 0;
    if (total_strength > 32767)
      total_strength = 32767;
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

    // Bow contribution: bow * bow_level * e * 1.8 (reduced from 2.5)
    int32_t bow_mix = mul_q15(mul_q15(bow_out, bow_level), e);
    bow_mix = (bow_mix * 9) >> 2;

    // Blow contribution: blow * blow_level_scaled * e + tube body
    // blow_level < 0.5 (16384): scale to 0..1.0 for noise level
    // blow_level >= 0.5: noise level stays at 0.4, and tube level increases
    int32_t blow_noise_lvl;
    int32_t tube_amt = 0;
    if (blow_level < 16384) {
      blow_noise_lvl = blow_level << 1; // 0..32767
      tube_amt = 0;
    } else {
      blow_noise_lvl = 13107;               // 0.4 fixed
      tube_amt = (blow_level - 16384) << 1; // 0..32767
    }

    int32_t b_noise = mul_q15(blow_out, blow_noise_lvl);
    b_noise = mul_q15(b_noise, e);

    // Process Tube (Flute Body)
    // Gain is reduced (tube_amt >> 3) to keep the Blow section from
    // overpowering
    int32_t tube_out =
        tube.Process(freq_q15, smooth_env_value, damping, blow_timbre, b_noise);
    int32_t b_mix = b_noise + mul_q15(tube_out, tube_amt >> 3);

    // Strike contribution: strike * accent * strike_level_scaled + external
    // strike_level_scaled = min(strike_level * 1.25, 1.0) * 0.9 (reduced
    // from 1.1)
    int32_t strike_lvl_adj = mul_q15(strike_level, 40960); // * 1.25
    if (strike_lvl_adj > 32767)
      strike_lvl_adj = 32767;
    int32_t strike_scaled = mul_q15(strike_lvl_adj, 29491); // * 0.9
    int32_t strike_mix = mul_q15(mul_q15(strike_out, accent), strike_scaled);

    // Strike bleed: raw strike signal bleeds to output at high levels
    int32_t strike_bleed = 0;
    if (strike_level > 26214) { // > 0.8 in Q15
      // Bleed gain reduced from 3.0 to 1.5
      strike_bleed = mul_q15(strike_out, ((strike_level - 26214) * 3) >> 1);
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

    // Sum all parts: Bow (direct) + Diffused (Blow/Ext/SoftStrike) + Direct
    // Strike
    int32_t excitation = bow_mix + diffused_excitation + strike_direct;

    // ── Exciter Gain Staging ────────────────────────────────────────
    // Apply soft limiting to the combined excitation signal to prevent
    // the resonator from blowing up / clipping internally.
    excitation = SoftLimitQ15(excitation);

    // DC block the excitation signal (critical for strings because strike is a
    // positive impulse)
    excitation = DCBlockQ15(excitation, dc_ex_x, dc_ex_y);

    // ── Damping from Exciters ───────────────────────────────────────
    // Strike exciter provides damping feedback (palm mute on release)
    int32_t final_damping = total_damping_cv;
    final_damping -= mul_q15(strike_exciter.damping, strike_lvl_adj) >> 3;
    // Bow damping: when bow is not pressed, it damps
    int32_t bow_strength_inv = 32767 - mul_q15(bow_level, smooth_env_value);
    final_damping -= mul_q15(bow_strength_inv, bow_level) >> 4;
    if (final_damping < 0)
      final_damping = 0;

    // ── Process Resonator / String ──────────────────────────────────
    // ── Output Stage ───────────────────────────────────────────────
    // We now send the raw Center and Side signals to Core 0.
    // Core 0 will handle the Stereo Widener (spread) and Delay (reverb).
    // This offloads significant CPU from Core 1.

    int32_t final_center = 0;
    int32_t final_sides = 0;

    if (currentModel == MODEL_MODAL || currentModel == MODEL_BORE) {
      int32_t res_center = 0;
      int32_t res_sides = 0;
      int32_t bow_strength_q15 = mul_q15(bow_level, smooth_env_value);

      resonator.damping_q15 = final_damping;
      resonator.Process1(bow_strength_q15, excitation, res_center, res_sides);

      final_center = res_center + strike_bleed;
      final_sides = res_sides;
    } else {
      // String models
      int32_t num_strings = (currentModel == MODEL_STRINGS) ? 3 : 1;
      // Chord table matching Elements' first 3 strings voicing.
      // Values in Q8 semitones (1 semitone = 256)
      static const int16_t chord_offsets[11][3] = {
          {0, -12 * 256, 12 * 256}, // Octaves
          {0, -12 * 256, 3 * 256},  // Minor
          {0, -12 * 256, 7 * 256},  // Minor 7
          {0, 3 * 256, 14 * 256},   // Minor 9
          {0, 3 * 256, 17 * 256},   // Minor 11
          {0, -12 * 256, 19 * 256}, // Power chord stack
          {0, 4 * 256, 17 * 256},   // Major 11
          {0, 4 * 256, 14 * 256},   // Major 9
          {0, 4 * 256, 7 * 256},    // Major
          {0, 4 * 256, 11 * 256},   // Major 7
          {0, 5 * 256, 7 * 256}     // Sus4
      };

      int32_t s_center = 0;
      int32_t s_sides = 0;
      int32_t sympathetic_bus = 0;

      for (int i = 0; i < num_strings; i++) {
        int32_t string_midi = midi_q8;
        if (num_strings == 3) {
          int32_t chord_idx = (geometry * 11) >> 15;
          if (chord_idx > 10)
            chord_idx = 10;
          string_midi += chord_offsets[chord_idx][i];
        }

        uint32_t s_inc = MidiToIncrementU32(string_midi);
        strings[i].SetFrequency(s_inc);

        // Sympathetic strings ring longer and are darker
        if (i > 0 && num_strings == 3) {
          // Longer decay (higher damping value)
          int32_t sym_damping = 32767 - ((32767 - final_damping) >> 1);
          strings[i].SetDamping(sym_damping);
          strings[i].SetBrightness(mul_q15(total_brightness, 20000));
        } else {
          strings[i].SetDamping(final_damping);
          strings[i].SetBrightness(total_brightness);
        }

        strings[i].SetPosition(position);
        strings[i].SetDispersion(geometry);

        int32_t c = 0, s = 0;
        int32_t current_in = 0;

        if (i == 0) {
          // String 0 is the master
          current_in = (num_strings == 3) ? (excitation >> 1) : excitation;
        } else {
          // Strings 1 and 2 are sympathetic, receiving master's energy
          // 0.25 scaling factor
          current_in = sympathetic_bus >> 2;
        }

        strings[i].Process(current_in, c, s);

        if (i == 0) {
          // Use output difference for sympathetic excitation
          sympathetic_bus = c - s;
        }

        s_center += c;
        s_sides += s;
      }
      final_center = s_center + strike_bleed;
      final_sides = s_sides;
    }

    // ── Send Results Back (Center/Sides) ────────────────────────────
    multicore_fifo_push_blocking((uint32_t)final_center);
    multicore_fifo_push_blocking((uint32_t)final_sides);
  }
}

// ── Main Application Forward Declaration ────────────────────────────────────
class Modal;

// ── MIDI Callbacks ──────────────────────────────────────────────────────────

// Forward declarations for MIDI callbacks used by the USB host driver
extern "C" {
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                       uint8_t num_cables_rx, uint16_t num_cables_tx);
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance);
void handle_midi_message(uint8_t *packet, int size);
void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets);
}

// ── Preset Management Logic ─────────────────────────────────────────────────

void __not_in_flash_func(save_preset)(int slot) {
  if (slot < 0 || slot >= PRESET_COUNT)
    return;

  Preset p;
  p.magic = PRESET_MAGIC;
  p.model = (uint32_t)currentModel;

  // Sequencer Save
  p.sequence = gen_sequence;
  memcpy(p.pitches, gen_pitches, 16);
  p.bpm_val = seq_bpm_val;
  p.density_val = seq_density_val;
  p.map_val = seq_map_val;
  p.scale = currentScale;
  p.root = currentRoot;

  for (int i = 0; i < NUM_PAGES; i++) {
    p.pages[i] = params[i];
  }

  // Static buffer to avoid stack overflow (RP2040 stack is small)
  static uint8_t flash_buffer[4096];

  uint32_t ints = save_and_disable_interrupts();

  // Copy existing presets to RAM buffer
  memcpy(flash_buffer, (const uint8_t *)(XIP_BASE + FLASH_PRESET_OFFSET), 4096);
  // Update the specific slot in our RAM buffer
  memcpy(flash_buffer + (slot * sizeof(Preset)), &p, sizeof(Preset));

  // Erase and reprogram the sector
  flash_range_erase(FLASH_PRESET_OFFSET, 4096);
  flash_range_program(FLASH_PRESET_OFFSET, flash_buffer, 4096);

  restore_interrupts(ints);
}

void load_preset(int slot) {
  if (slot < 0 || slot >= PRESET_COUNT)
    return;

  const Preset *p = (const Preset *)(XIP_BASE + FLASH_PRESET_OFFSET +
                                     (slot * sizeof(Preset)));

  if (p->magic != PRESET_MAGIC)
    return;

  currentModel = (int32_t)p->model;

  // Sequencer Load
  gen_sequence = p->sequence;
  memcpy(gen_pitches, p->pitches, 16);
  seq_bpm_val = p->bpm_val;
  seq_density_val = p->density_val;
  seq_map_val = p->map_val;
  currentScale = p->scale;
  currentRoot = p->root;

  for (int i = 0; i < NUM_PAGES; i++) {
    params[i] = p->pages[i];
  }
}

// ── Factory Defaults ────────────────────────────────────────────────────────

static const Preset factory_presets[8] = {
    {// Slot 0: Wooden Marimba
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{32767, 16384, 0},
      {0, 0, 0},
      {0, 0, 16384},
      {16384, 28000, 8000},
      {16384, 5000, 2000},
      {16384, 16384, 32767}},
     {0}},
    {// Slot 1: Bowed Glass
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{0, 0, 0},
      {0, 0, 0},
      {28000, 16384, 32000},
      {30000, 20000, 30000},
      {4000, 32767, 30000},
      {12000, 16384, 32767}},
     {0}},
    {// Slot 2: Industrial Plate
     PRESET_MAGIC,
     (uint32_t)MODEL_MODAL,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{32767, 32767, 32767},
      {8000, 16384, 16384},
      {0, 0, 0},
      {32000, 10000, 15000},
      {28000, 16384, 10000},
      {8000, 16384, 32767}},
     {0}},
    {// Slot 3: Ambient Strings
     PRESET_MAGIC,
     (uint32_t)MODEL_STRINGS,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{16384, 16384, 0},
      {0, 0, 0},
      {25000, 8000, 28000},
      {20000, 16384, 25000},
      {10000, 30000, 28000},
      {16384, 16384, 32767}},
     {0}},
    {// Slot 4: Wind Flute
     PRESET_MAGIC,
     (uint32_t)MODEL_BORE,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{0, 0, 0},
      {32767, 12000, 16384},
      {0, 0, 16384},
      {30000, 18000, 22000},
      {16384, 12000, 15000},
      {24000, 16384, 32767}},
     {0}},
    {// Slot 5: Plucked Bass
     PRESET_MAGIC,
     (uint32_t)MODEL_STRING,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{32767, 8000, 0},
      {0, 0, 0},
      {0, 0, 0},
      {8000, 8000, 12000},
      {16384, 0, 0},
      {4000, 16384, 32767}},
     {0}},
    {// Slot 6: Searing Bow
     PRESET_MAGIC,
     (uint32_t)MODEL_STRINGS,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{0, 0, 0},
      {0, 0, 0},
      {32767, 30000, 20000},
      {16384, 32767, 30000},
      {16384, 16384, 20000},
      {20000, 16384, 32767}},
     {0}},
    {// Slot 7: Hollow Bottle
     PRESET_MAGIC,
     (uint32_t)MODEL_BORE,
     0xAAAA,
     {48, 51, 53, 55, 58, 60, 63, 65, 67, 70, 72, 75, 77, 79, 82, 84},
     16384,
     16384,
     0,
     0,
     0,
     {{0, 0, 0},
      {20000, 8000, 16384},
      {0, 0, 16384},
      {2000, 10000, 32000},
      {16384, 16384, 32767},
      {16384, 16384, 32767}},
     {0}}};

void init_factory_presets() {
  const Preset *p = (const Preset *)(XIP_BASE + FLASH_PRESET_OFFSET);
  if (p->magic != PRESET_MAGIC) {
    // Flash is empty, write factory defaults
    uint8_t buffer[4096];
    memset(buffer, 0, 4096);
    memcpy(buffer, factory_presets, sizeof(factory_presets));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_PRESET_OFFSET, 4096);
    flash_range_program(FLASH_PRESET_OFFSET, buffer, 4096);
    restore_interrupts(ints);
  }
}

class Modal : public ComputerCard {
public:
  // ── UI State ────────────────────────────────────────────────────────
  int currentPage = 0;             // Currently active parameter page (0–5)
  KnobLock lockMain, lockX, lockY; // Knob locks for page switching
  int32_t smoothMain = 0, smoothX = 0, smoothY = 0; // Smoothed knob values

  // ── Switch State ────────────────────────────────────────────────────
  uint32_t switchDownTimer = 0; // How long switch has been held down
  bool switchHandled = false;   // Has the current press been handled?
  uint32_t switchUpTimer = 0;
  bool switchHandledUp = false;
  bool switchInit = false;            // Has switch been initialized?
  bool lastSwitchUp = false;          // Previous state of switch-up
  Switch lastSwitch = Switch::Middle; // Debounced switch state
  uint32_t debounceTimer = 0;         // Switch debounce counter

  // ── Gate State ──────────────────────────────────────────────────────
  bool previousGate = false;    // Gate state from last DSP cycle
  bool triggerBuffered = false; // Buffered trigger from Pulse In 1

  // ── Audio State ─────────────────────────────────────────────────────
  int32_t dspOutL = 0, dspOutR = 0;         // Latest output from Core 1
  int gPhase = 0;                           // Phase counter for 2:1 decimation
  uint32_t bootSilence = BOOT_SILENCE_SAMP; // Startup mute countdown

  // ── CV Smoothing ────────────────────────────────────────────────────
  int32_t cv1_acc = 0; // Smoothed CV1 accumulator
  int32_t cv2_acc = 0; // Smoothed CV2 accumulator

  // ── DC Blockers ─────────────────────────────────────────────────────
  int32_t dc_bxL = 0, dc_byL = 0, dc_bxR = 0, dc_byR = 0; // Input
  int32_t dc_oxL = 0, dc_oyL = 0, dc_oxR = 0, dc_oyR = 0; // Output

  // ── Page Display Timer ──────────────────────────────────────────────
  uint32_t pageDisplayTimer = 0; // Countdown for page display LED flash
  static const uint32_t PAGE_DISPLAY_DURATION = 24000; // ~500ms at 48kHz

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
    // Matching original Elements defaults: geometry=0.2, brightness=0.5,
    // damping=0.25
    params[3] = {6554, 16384, 8192};

    // Page 4: Resonator Space (Position, Space)
    params[4] = {9830, 8192, 0}; // Position=0.3, Space=0.25 (light reverb)

    // Page 5: Performance (Pitch Coarse, Fine Tune, Strength)
    params[5] = {16384, 16384, 16384};

    // Initialize reverb state
    reverb.Init();

    // Initialize sequencer pitches (Pentatonic Minor)
    for (int i = 0; i < 16; i++) {
      static const uint8_t scale[] = {0, 3, 5, 7, 10};
      gen_pitches[i] = 48 + scale[i % 5];
    }
  }

  // ────────────────────────────────────────────────────────────────────
  //  InitUSB — Initialize TinyUSB based on hardware power state
  // ────────────────────────────────────────────────────────────────────

  void InitUSB() {
    // Wait for USB power state to settle
    sleep_ms(300);
    usb_power_state = USBPowerState();

    // Visual feedback:
    // LED 0 flashes for Host mode, LED 1 for Device mode
    if (usb_power_state == ComputerCard::DFP) {
      isUSBMIDIHost = true;
      LedOn(0);
      tuh_init(TUH_OPT_RHPORT);
    } else {
      isUSBMIDIHost = false;
      LedOn(1);
      tud_init(TUD_OPT_RHPORT);
    }
    sleep_ms(200);
    LedOff(0);
    LedOff(1);
  }

  // ────────────────────────────────────────────────────────────────────
  //  BackgroundLoop — runs on Core 0 when not in ProcessSample ISR
  //  Used for slow UI updates (LED display, flash operations, etc.)
  //  ~5000 iterations per meaningful update (matches grains pattern).
  // ────────────────────────────────────────────────────────────────────

  void BackgroundLoop() override {
    // ── USB / MIDI Tasks ────────────────────────────────────────────
    if (isUSBMIDIHost) {
      tuh_task();
    } else {
      tud_task();
      while (tud_midi_available()) {
        uint8_t packet[4];
        tud_midi_packet_read(packet);
        handle_midi_message(packet + 1, 3);
      }

      // ── MIDI Feedback Sync ──────────────────────────────────────
      // Send current knob values back to UI, but debounced and only if changed.
      // Check one page per loop to save CPU.
      static int syncPage = 0;
      static int32_t last_sent_model = -1;
      syncPage = (syncPage + 1) % 6;

      // Sync Model via CC 102
      if (currentModel != last_sent_model) {
        last_sent_model = currentModel;
        if (tud_midi_mounted()) {
          uint8_t packet[4] = {0x0B, 0xB0, 102, (uint8_t)currentModel};
          tud_midi_packet_write(packet);
        }
      }

      auto syncKnob = [&](int32_t current, int32_t &last, uint8_t cc) {
        // Only send if the difference is significant (>1 CC step) to avoid
        // jitter
        int32_t diff = (current > last) ? (current - last) : (last - current);
        if (diff > 256) {
          last = current;
          if (tud_midi_mounted()) {
            uint8_t packet[4] = {0x0B, 0xB0, cc, (uint8_t)(current >> 8)};
            tud_midi_packet_write(packet);
            last_cc_tx_time[cc & 0x7F] = to_ms_since_boot(get_absolute_time());
          }
        }
      };

      syncKnob(params[syncPage].pMain, last_sent[syncPage].pMain,
               PageCCs[syncPage][0]);
      syncKnob(params[syncPage].pX, last_sent[syncPage].pX,
               PageCCs[syncPage][1]);
      syncKnob(params[syncPage].pY, last_sent[syncPage].pY,
               PageCCs[syncPage][2]);

      // Sync Sequencer Params
      auto syncSeq = [&](int32_t &current, int32_t &last, uint8_t cc) {
        // Apply a 2.5% dead-zone (800 units in Q15)
        if (abs(current - last) > 800) {
          last = current;
          if (tud_midi_mounted()) {
            uint8_t packet[4] = {0x0B, 0xB0, cc, (uint8_t)(current >> 8)};
            tud_midi_packet_write(packet);
            last_cc_tx_time[cc & 0x7F] = to_ms_since_boot(get_absolute_time());
          }
        }
      };
      syncSeq(seq_density_val, last_seq_dens, 103);
      syncSeq(seq_bpm_val, last_seq_bpm, 104);
      syncSeq(seq_map_val, last_seq_ent, 105);

      // Sync Sequence Pattern
      static uint32_t last_gen_sequence = 0;
      if (gen_sequence != last_gen_sequence) {
        last_gen_sequence = gen_sequence;
        if (tud_midi_mounted()) {
          uint8_t p1[4] = {0x0B, 0xB0, 107, (uint8_t)(gen_sequence & 0x7F)};
          uint8_t p2[4] = {0x0B, 0xB0, 108,
                           (uint8_t)((gen_sequence >> 7) & 0x7F)};
          uint8_t p3[4] = {0x0B, 0xB0, 110,
                           (uint8_t)((gen_sequence >> 14) & 0x03)};
          tud_midi_packet_write(p1);
          tud_midi_packet_write(p2);
          tud_midi_packet_write(p3);
        }
      }

      // ── Generative Sequencer Logic ──────────────────────────────────
      // Runs every background loop (~1ms) at all times
      static uint32_t last_gen_ms = 0;
      uint32_t now = to_ms_since_boot(get_absolute_time());

      int bpm = 40 + ((seq_bpm_val * 200) >> 15);
      uint32_t step_ms = (60000 / bpm) / 4; // 16th notes

      if (now - last_gen_ms >= step_ms) {
        last_gen_ms = now;
        gen_step = (gen_step + 1) % 16;
        gen_pulse_clk = true; // Clock pulse start

        // Sync step to Web UI
        if (tud_midi_mounted()) {
          uint8_t packet[4] = {0x0B, 0xB0, 106, (uint8_t)gen_step};
          tud_midi_packet_write(packet);
        }

        // ── Topographic Melodic Engine (Grids Style) ─────────────────
        // 4 Basis Patterns (Corner 0,0: Steady, 1,0: Syncopated, 0,1: Arp, 1,1:
        // Chaos)
        static const uint16_t basis_gates[4] = {
            0x8888, // 1/4 Downbeats
            0x9249, // Euclidean 3/8 (Syncopated)
            0xAAAA, // 1/8 Steady
            0xBEAF  // Broken 16ths
        };
        static const int8_t basis_pitches[4][16] = {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // Steady Root
            {0, 3, 7, 0, 3, 7, 12, 7, 0, 3, 7, 0, 5, 7, 10, 7}, // Rhythmic Arp
            {0, 12, 7, 12, 5, 17, 12, 17, 7, 19, 15, 19, 0, 12, 7,
             12}, // High Arp
            {0, 7, 3, 10, 5, 12, 7, 15, 2, 9, 5, 12, 7, 14, 10, 17}
            // Jumpy Chaos
        };

        // Rhythmic Hierarchy Seeds (Downbeats -> Eighths -> Sixteenths)
        static const uint16_t rh_seeds[16] = {
            0,    16000, 8000,  24000, 4000, 20000, 12000, 28000,
            2000, 18000, 10000, 26000, 6000, 22000, 14000, 30000};

        // 2D Interpolation (X=Density, Y=Map)
        int32_t y = seq_map_val;     // 0..32767
        int32_t x = seq_density_val; // 0..32767

        // Chaos Injection (Full Right on Map)
        bool chaos_mode = (y > 31000);
        if (chaos_mode) {
          y = 31000; // Cap interpolation at the edge of chaos
        }

        // Blend Gates (Bitwise selection based on Y)
        uint16_t gA = (y < 16384) ? basis_gates[0] : basis_gates[2];
        uint16_t gB = (y < 16384) ? basis_gates[1] : basis_gates[3];
        uint16_t blendY = (y < 16384) ? (y << 1) : ((y - 16384) << 1);

        // Select patterns based on Y blend
        uint16_t master_gates = (uint16_t)((((uint32_t)gA * (32768 - blendY)) +
                                            ((uint32_t)gB * blendY)) >>
                                           15);

        // Euclidean-style density thresholding
        bool gate_on = (master_gates & (1 << gen_step)) ||
                       (rh_seeds[gen_step] < (uint32_t)x);

        // Chaos variation
        if (chaos_mode && ((rand() % 100) < 20)) {
          gate_on = !gate_on; // Toggle gate randomly
        }

        gen_sequence =
            (gen_sequence & ~(1 << gen_step)) | (gate_on ? (1 << gen_step) : 0);

        // Interpolate Pitches
        int idxA = (y < 16384) ? 0 : 2;
        int idxB = (y < 16384) ? 1 : 3;
        int32_t pA = basis_pitches[idxA][gen_step];
        int32_t pB = basis_pitches[idxB][gen_step];
        int32_t p_blend = (pA * (32768 - blendY) + pB * blendY) >> 15;

        if (chaos_mode) {
          p_blend += (rand() % 7) - 3; // Slighly shift pitch
        }

        // Map to Scale/Root
        static const uint8_t scales[5][12] = {
            {0, 3, 5, 7, 10, 12, 15, 17, 19, 22, 24, 27}, // Pent Minor
            {0, 2, 4, 7, 9, 12, 14, 16, 19, 21, 24, 26},  // Pent Major
            {0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19},   // Natural Minor
            {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19},   // Major
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}        // Chromatic
        };
        uint8_t scale_idx = currentScale % 5;
        // Quantize interpolated pitch to scale
        int32_t target_note = p_blend + 60; // Base C4
        int32_t octave = (target_note / 12) - 4;
        int32_t note_in_octave = target_note % 12;

        // Find closest note in scale
        int32_t best_dist = 100;
        int32_t best_note = 0;
        for (int i = 0; i < 12; i++) {
          int32_t dist = abs(note_in_octave - scales[scale_idx][i]);
          if (dist < best_dist) {
            best_dist = dist;
            best_note = scales[scale_idx][i];
          }
        }
        gen_pitches[gen_step] = 48 + currentRoot + (octave * 12) + best_note;

        // Trigger Step
        bool stepActive = (gen_sequence & (1 << gen_step));
        if (stepActive) {
          gen_melody_q8 = (gen_pitches[gen_step] - 60) * 256;
          gen_gate_pending = true; // Prepare to fire gate after CV settles
          gen_gate = false;        // Ensure gate is low during CV transition
        } else {
          gen_gate = false;
          gen_gate_pending = false;
        }
      } else if (gen_gate_pending && (now - last_gen_ms >= 2)) {
        // 2ms have passed since CV update — fire the gate
        gen_gate = true;
        gen_gate_pending = false;
      } else if (gen_gate && (now - last_gen_ms >= (step_ms / 2))) {
        // 50% duty cycle reached — end the gate
        gen_gate = false;
      } else if (now - last_gen_ms > 20) {
        gen_pulse_clk = false; // Short clock pulse
      }

      // ── External Sync & Randomization ─────────────────────────────
      static bool last_pulse1 = false, last_pulse2 = false;
      bool p1 = PulseIn1();
      bool p2 = PulseIn2();

      if (p1 && !last_pulse1) {
        // Pulse 1 In: Randomize CV 2
        gen_cv2_val = (rand() % 4096) - 2048;
      }
      if (p2 && !last_pulse2) {
        // Pulse 2 In: Sync Sequencer (Reset to Step 0)
        gen_step = 0;
      }
      last_pulse1 = p1;
      last_pulse2 = p2;
    }

    static uint32_t loopCount = 0;
    if (++loopCount < 1000)
      return;
    loopCount = 0;

    // ── LED Rendering ───────────────────────────────────────────────
    if (currentUIState == STATE_LOAD_MENU ||
        currentUIState == STATE_SAVE_MENU) {
      // Preset Menu Visuals:
      // LEDs 0-3 show slot index in binary (0-15)
      // LED 4 pulses for LOAD, LED 5 pulses for SAVE
      static uint32_t pulseTimer = 0;
      pulseTimer++;

      for (int i = 0; i < 6; i++) {
        if (i < 4) {
          // Binary slot display
          bool on = (presetMenuSlot & (1 << i));
          LedOn(i, on);
        } else if (i == 4 && currentUIState == STATE_LOAD_MENU) {
          // Pulse LED 4 for Load
          int32_t b = (pulseTimer % 50 < 25) ? 4095 : 500;
          LedBrightness(i, b);
        } else if (i == 5 && currentUIState == STATE_SAVE_MENU) {
          // Pulse LED 5 for Save
          int32_t b = (pulseTimer % 50 < 25) ? 4095 : 500;
          LedBrightness(i, b);
        } else {
          LedOff(i);
        }
      }
    } else if (currentUIState == STATE_GEN_SEQ) {
      // Sequencer View: 6 LEDs representing the 16 steps
      // 1. Background bar shows Density
      int density_leds = (seq_density_val * 6) >> 15;
      // 2. Traveling dot shows 16-step position scaled to 6 LEDs
      int pos_led = (gen_step * 6) / 16;

      for (int i = 0; i < 6; i++) {
        int br = 0;
        if (i < density_leds)
          br = 150; // Dim background for density
        if (i == pos_led)
          br = 4095; // Bright dot for position
        LedBrightness(i, br);
      }
    } else if (pageDisplayTimer > 0) {
      // Page just changed — show current page prominently
      for (int i = 0; i < 6; i++) {
        LedOn(i, i == currentPage);
      }
    } else {
      // Normal Operation: Display current page + Gate pulses
      for (int i = 0; i < 6; i++) {
        int32_t b = (i == currentPage) ? 1800 : 0;

        // Pulse LED 4 on gate activity
        if (i == 4 && previousGate)
          b += 2000;

        if (b > 4095)
          b = 4095;
        if (b > 0)
          LedBrightness(i, b);
        else
          LedOff(i);
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
        for (int i = 0; i < 6; i++)
          LedOff(i);
      }
      return;
    }

    // ── Poll Triggers ───────────────────────────────────────────────
    // Check pulse inputs at full 48kHz rate to avoid missing triggers.
    if (PulseIn1RisingEdge() || midi_trigger) {
      triggerBuffered = true;
      // We'll clear midi_trigger below once we've sampled the pitch
    }

    // ── Read Raw Inputs ─────────────────────────────────────────────
    Switch sw = SwitchVal();
    int32_t rawMain = KnobVal(Knob::Main);
    int32_t rawX = KnobVal(Knob::X);
    int32_t rawY = KnobVal(Knob::Y);

    // Scale knobs to Q15 range
    int32_t kMain = knob_to_q15(rawMain);
    int32_t kX = knob_to_q15(rawX);
    int32_t kY = knob_to_q15(rawY);

    // Smooth knobs with heavy IIR for sequencer (>>5) and
    // responsive IIR for normal play (>>3)
    int32_t shift = (currentUIState == STATE_GEN_SEQ) ? 5 : 3;
    smoothMain += (kMain - smoothMain) >> shift;
    smoothX += (kX - smoothX) >> shift;
    smoothY += (kY - smoothY) >> shift;

    // CV1 for V/Oct pitch tracking
    int32_t cv1_raw = Connected(ComputerCard::CV1) ? CVIn1() : 0;
    cv1_acc = cv1_acc - (cv1_acc >> 8) + cv1_raw;
    int32_t cv1_smoothed = cv1_acc >> 8;

    // Sample & Hold logic triggered by Pulse In 1 or MIDI Note On
    static bool last_p1_sh = false;
    bool p1_now = PulseIn1();
    bool midi_trig_local = midi_trigger;
    bool pulse1_connected = Connected(ComputerCard::Pulse1);

    int32_t raw_pitch =
        Connected(ComputerCard::CV1) ? ((cv1_smoothed * 15) >> 1) : 0;

    if (pulse1_connected) {
      // S&H and Quantization active
      if ((p1_now && !last_p1_sh) || midi_trig_local) {
        // Quantize to nearest semitone (256 in Q8) and add MIDI offset
        cv1_pitch_q8 = ((raw_pitch + 128) & ~0xFF) + midi_pitch_q8;
        midi_trigger = false; // Mark handled
      }
    } else {
      // Continuous tracking, no quantization
      cv1_pitch_q8 = raw_pitch + midi_pitch_q8;
      if (midi_trig_local)
        midi_trigger = false;
    }
    last_p1_sh = p1_now;

    // CV2 for strength/accent modulation
    int32_t cv2_raw = Connected(ComputerCard::CV2) ? CVIn2() : 0;
    cv2_acc = cv2_acc - (cv2_acc >> 4) + cv2_raw;
    cv2_strength = (cv2_acc >> 4) << 3; // Scale 12-bit to ~Q15

    // Buffer external audio inputs for Core 1 (scale 12-bit to Q15)
    audio_in1_q15 = AudioIn1() << 4; // ±2047 → ±32752
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

    // ── Switch Down: Page Cycling & Preset Menu ───────────────────────
    // Tap = next page. Hold (2s) = Preset Menu (X Left = Load, X Right = Save).
    if (effectiveSwitch == Switch::Down) {
      switchDownTimer++;
      if (switchDownTimer > 30000 && currentUIState == STATE_NORMAL) {
        if (smoothX < 16384)
          currentUIState = STATE_LOAD_MENU;
        else
          currentUIState = STATE_SAVE_MENU;
        lockMain.relock(smoothMain);
        lockX.relock(smoothX);
        lockY.relock(smoothY);
      }
    } else {
      if (switchDownTimer > 500) {
        if (currentUIState == STATE_LOAD_MENU) {
          load_preset(presetMenuSlot);
          currentUIState = STATE_NORMAL;
        } else if (currentUIState == STATE_SAVE_MENU) {
          save_preset(presetMenuSlot);
          currentUIState = STATE_NORMAL;
        } else if (!switchHandled) {
          currentPage = (currentPage + 1) % NUM_PAGES;
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);
          pageDisplayTimer = PAGE_DISPLAY_DURATION;
        }
        switchHandled = true;
      }
      if (effectiveSwitch == Switch::Middle) {
        switchDownTimer = 0;
        switchHandled = false;
      }
    }

    // ── Switch Up: Model Toggle & Generative Menu ───────────────────
    // Tap = cycle models. Hold (2s) = Generative Sequencer Menu.
    if (effectiveSwitch == Switch::Up) {
      switchUpTimer++;
      if (switchUpTimer > 30000 && currentUIState == STATE_NORMAL) {
        currentUIState = STATE_GEN_SEQ;
        lockMain.relock(smoothMain);
        lockX.relock(smoothX);
        lockY.relock(smoothY);
      }
    } else {
      if (switchUpTimer > 500) {
        if (currentUIState == STATE_GEN_SEQ) {
          currentUIState = STATE_NORMAL;
          lockMain.relock(smoothMain);
          lockX.relock(smoothX);
          lockY.relock(smoothY);
        } else if (!switchHandledUp) {
          currentModel = (currentModel + 1) % MODEL_COUNT;
        }
        switchHandledUp = true;
      }
      if (effectiveSwitch == Switch::Middle) {
        switchUpTimer = 0;
        switchHandledUp = false;
      }
    }

    // ── Generative Sequencer / Preset Menu Controls ──────────────────
    if (currentUIState == STATE_GEN_SEQ) {
      if (lockMain.update(smoothMain))
        seq_density_val = smoothMain;
      if (lockX.update(smoothX))
        seq_bpm_val = smoothX;
      if (lockY.update(smoothY))
        seq_map_val = smoothY;
    } else if (currentUIState == STATE_LOAD_MENU ||
               currentUIState == STATE_SAVE_MENU) {
      // Use Main knob to select slot 0-15
      presetMenuSlot = (smoothMain >> 11); // Q15 -> 0..15
      if (presetMenuSlot > 15)
        presetMenuSlot = 15;
      if (presetMenuSlot < 0)
        presetMenuSlot = 0;
    }

    // Pulse In 2 functionality moved to Sequencer Sync

    // ── Page Display Timer ──────────────────────────────────────────
    if (pageDisplayTimer > 0)
      pageDisplayTimer--;

    // Only update parameters if we are in normal operation mode
    if (currentUIState == STATE_NORMAL) {
      if (lockMain.update(smoothMain))
        params[currentPage].pMain = smoothMain;
      if (lockX.update(smoothX))
        params[currentPage].pX = smoothX;
      if (lockY.update(smoothY))
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
      bool gateNow = PulseIn1() || triggerBuffered || midi_gate;

      if (gateNow)
        flags |= FIFO_FLAG_GATE;
      if (gateNow && !previousGate)
        flags |= FIFO_FLAG_RISING;
      if (!gateNow && previousGate)
        flags |= FIFO_FLAG_FALLING;

      previousGate = gateNow;
      triggerBuffered = false;

      // 1. Receive processed audio from Core 1 (from the PREVIOUS 24kHz period)
      // By popping before pushing, we don't force Core 1 to finish within
      // this 20.8us ISR. It gets a full 41.6us to compute. If it's not ready,
      // we reuse the old output (preventing a hard drop).
      if (multicore_fifo_rvalid()) {
        int32_t center = (int32_t)multicore_fifo_pop_blocking();
        int32_t sides = (int32_t)multicore_fifo_pop_blocking();

        // ── Stereo Widener (Mid-Side Spread) ────────────────────
        // Space parameter (Page 4 X) controls the stereo width.
        int32_t space = params[4].pX;
        int32_t space_adj = (space > 3277) ? (space - 3277) : 0;
        int32_t spread = space_adj;
        if (spread > 22938)
          spread = 22938; // Max spread 0.7

        int32_t side_signal = mul_q15(sides, spread);
        int32_t outL = center + side_signal;
        int32_t outR = center - side_signal;

        // ── Soft Limiter ────────────────────────────────────────
        // Applied before reverb to prevent harsh resonance peaks.
        outL = SoftLimitQ15(outL);
        outR = SoftLimitQ15(outR);

        // ── Stereo Delay / Reverb ───────────────────────────────
        // Space parameter (Page 4 Y) also controls reverb mix/decay.
        int32_t reverb_amt = params[4].pY;
        int32_t rev_decay = 9830 + mul_q15(reverb_amt, 22610);
        int32_t rev_damp = 16384;

        reverb.Process(outL, outR, reverb_amt, rev_decay, rev_damp);

        // ── Soft Clip ───────────────────────────────────────────
        outL = SoftClipQ15(outL);
        outR = SoftClipQ15(outR);

        // ── DC Block ────────────────────────────────────────────
        dspOutL = dc_block(outL, dc_oxL, dc_oyL);
        dspOutR = dc_block(outR, dc_oxR, dc_oyR);
      }

      // 2. Send work to Core 1 to start processing the NEXT period
      multicore_fifo_push_blocking(flags);
    }

    // ── Audio Output ────────────────────────────────────────────────
    // Scale from Q15 (±32767) to 12-bit DAC range (±2047) and clamp.

    int32_t outL = dspOutL >> 4;
    int32_t outR = dspOutR >> 4;

    if (outL > 2047)
      outL = 2047;
    if (outL < -2048)
      outL = -2048;
    if (outR > 2047)
      outR = 2047;
    if (outR < -2048)
      outR = -2048;

    AudioOut1((int16_t)outL);
    AudioOut2((int16_t)outR);

    // ── CV Outputs ──────────────────────────────────────────────────
    // Always output the current melody/mod, but they only update
    // in the background loop (with a 2ms lead time before gen_gate).
    int32_t melody_dac = (gen_melody_q8 * 410) / 3072;
    CVOut1(melody_dac);
    CVOut2(gen_cv2_val);

    // ── Pulse Outputs ───────────────────────────────────────────────
    PulseOut1(gen_gate);
    PulseOut2(gen_pulse_clk);
  }
};

int main() {
  // ── System Overclock ────────────────────────────────────────────────
  // 240MHz for maximum DSP headroom
  vreg_set_voltage(VREG_VOLTAGE_1_25);
  sleep_ms(10);
  set_sys_clock_khz(240000, true);

  // Create the application
  Modal *modal = new Modal();
  modal->EnableNormalisationProbe();

  // ── USB / MIDI Initialization ───────────────────────────────────────
  modal->InitUSB();

  // Initialize factory presets if flash is empty
  init_factory_presets();

  // Launch Core 1 DSP engine
  multicore_launch_core1(core1_dsp_loop);

  // Run the main application (never returns)
  modal->Run();
}

// ── MIDI Callback Implementations ───────────────────────────────────────────

extern "C" {
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                       uint8_t num_cables_rx, uint16_t num_cables_tx) {
  (void)in_ep;
  (void)out_ep;
  (void)num_cables_rx;
  (void)num_cables_tx;
  if (midi_dev_addr == 0)
    midi_dev_addr = dev_addr;
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance) {
  (void)instance;
  if (dev_addr == midi_dev_addr)
    midi_dev_addr = 0;
}

void handle_midi_message(uint8_t *packet, int size) {
  if (size < 3)
    return;
  uint8_t status = packet[0];
  uint8_t type = status & 0xF0;

  Modal *app = (Modal *)Modal::ThisPtr();

  if (type == 0x90 && packet[2] > 0) { // Note On
    midi_pitch_q8 = (packet[1] - 60) * 256;
    midi_gate = true;
    midi_trigger = true;
    printf("MIDI Note On: %d\n", packet[1]);
  } else if (type == 0x80 || (type == 0x90 && packet[2] == 0)) { // Note Off
    if (midi_pitch_q8 == (packet[1] - 60) * 256) {
      midi_gate = false;
    }
  } else if (type == 0xB0) { // CC
    uint8_t cc = packet[1];
    int32_t val = packet[2] << 8;

    // Hardware priority: ignore incoming CCs for 500ms after we just sent one
    // to prevent MIDI echo loops from fighting the physical knobs.
    if (to_ms_since_boot(get_absolute_time()) - last_cc_tx_time[cc & 0x7F] <
        500) {
      return;
    }

    int targetPage = -1;
    int targetKnob = -1;

    switch (cc) {
    case 12:
      targetPage = 0;
      targetKnob = 0;
      break;
    case 13:
      targetPage = 0;
      targetKnob = 1;
      break;
    case 14:
      targetPage = 0;
      targetKnob = 2;
      break;
    case 15:
      targetPage = 1;
      targetKnob = 0;
      break;
    case 16:
      targetPage = 1;
      targetKnob = 1;
      break;
    case 17:
      targetPage = 1;
      targetKnob = 2;
      break;
    case 18:
      targetPage = 2;
      targetKnob = 0;
      break;
    case 19:
      targetPage = 2;
      targetKnob = 1;
      break;
    case 20:
      targetPage = 2;
      targetKnob = 2;
      break;
    case 70:
      targetPage = 3;
      targetKnob = 0;
      break;
    case 74:
      targetPage = 3;
      targetKnob = 1;
      break;
    case 71:
      targetPage = 3;
      targetKnob = 2;
      break;
    case 10:
      targetPage = 4;
      targetKnob = 0;
      break;
    case 21:
      targetPage = 4;
      targetKnob = 1;
      break;
    case 23:
      targetPage = 4;
      targetKnob = 2;
      break; // Reverb (Page 4 Y)
    case 1:
      targetPage = 5;
      targetKnob = 2;
      break; // Strength (Page 5 Y)
    case 11:
      targetPage = 5;
      targetKnob = 0;
      break;
    case 22:
      targetPage = 5;
      targetKnob = 1;
      break;
    case 102: // Model Switch
      currentModel = packet[2] % MODEL_COUNT;
      printf("Model Change: %ld\n", currentModel);
      break;

    // ── Remote Preset Management ─────────────────────────────
    case 118: // Save to Slot X
      save_preset(packet[2] % PRESET_COUNT);
      break;
    case 119: // Load from Slot X
      load_preset(packet[2] % PRESET_COUNT);
      break;
    case 103:
      if (packet[2] != (last_seq_dens >> 8))
        seq_density_val = (packet[2] << 8);
      break;
    case 104:
      if (packet[2] != (last_seq_bpm >> 8))
        seq_bpm_val = (packet[2] << 8);
      break;
    case 105:
      if (packet[2] != (last_seq_ent >> 8))
        seq_map_val = (packet[2] << 8);
      break;

    case 109: // Toggle Step
      gen_sequence ^= (1 << (packet[2] % 16));
      break;

    case 111: // Request Pattern & Parameter Sync
      if (tud_midi_mounted()) {
        // Sync current sequence pattern
        uint8_t p1[4] = {0x0B, 0xB0, 107, (uint8_t)(gen_sequence & 0x7F)};
        uint8_t p2[4] = {0x0B, 0xB0, 108,
                         (uint8_t)((gen_sequence >> 7) & 0x7F)};
        uint8_t p3[4] = {0x0B, 0xB0, 110,
                         (uint8_t)((gen_sequence >> 14) & 0x03)};
        tud_midi_packet_write(p1);
        tud_midi_packet_write(p2);
        tud_midi_packet_write(p3);
        // Also sync Scale/Root
        uint8_t p4[4] = {0x0B, 0xB0, 112, currentRoot};
        uint8_t p5[4] = {0x0B, 0xB0, 113, currentScale};
        tud_midi_packet_write(p4);
        tud_midi_packet_write(p5);

        // Sync Model
        uint8_t pm[4] = {0x0B, 0xB0, 102, (uint8_t)currentModel};
        tud_midi_packet_write(pm);

        // Sync Sequencer Params
        uint8_t sd[4] = {0x0B, 0xB0, 103, (uint8_t)(seq_density_val >> 8)};
        uint8_t sb[4] = {0x0B, 0xB0, 104, (uint8_t)(seq_bpm_val >> 8)};
        uint8_t sm[4] = {0x0B, 0xB0, 105, (uint8_t)(seq_map_val >> 8)};
        tud_midi_packet_write(sd);
        tud_midi_packet_write(sb);
        tud_midi_packet_write(sm);

        // Sync all parameter pages
        for (int page = 0; page < NUM_PAGES; page++) {
          uint8_t ccM = PageCCs[page][0];
          uint8_t ccX = PageCCs[page][1];
          uint8_t ccY = PageCCs[page][2];
          uint8_t pkt1[4] = {0x0B, 0xB0, ccM,
                             (uint8_t)(params[page].pMain >> 8)};
          uint8_t pkt2[4] = {0x0B, 0xB0, ccX, (uint8_t)(params[page].pX >> 8)};
          uint8_t pkt3[4] = {0x0B, 0xB0, ccY, (uint8_t)(params[page].pY >> 8)};
          tud_midi_packet_write(pkt1);
          tud_midi_packet_write(pkt2);
          tud_midi_packet_write(pkt3);
        }
      }
      break;

    case 112:
      currentRoot = packet[2] % 12;
      break;
    case 113:
      currentScale = packet[2] % 5;
      break;
    }

    if (targetPage != -1) {
      // Ignore the CC if it's identical to the one we just sent out (MIDI
      // echo/loopback prevention). This prevents the hardware knobs from
      // snapping/locking when turning them while connected to Web UI.
      uint8_t last_cc = 0;
      if (targetKnob == 0)
        last_cc = last_sent[targetPage].pMain >> 8;
      else if (targetKnob == 1)
        last_cc = last_sent[targetPage].pX >> 8;
      else if (targetKnob == 2)
        last_cc = last_sent[targetPage].pY >> 8;

      if (packet[2] != last_cc) {
        if (targetKnob == 0) {
          params[targetPage].pMain = val;
          last_sent[targetPage].pMain = val;
        } else if (targetKnob == 1) {
          params[targetPage].pX = val;
          last_sent[targetPage].pX = val;
        } else if (targetKnob == 2) {
          params[targetPage].pY = val;
          last_sent[targetPage].pY = val;
        }

        if (app && targetPage == app->currentPage) {
          if (targetKnob == 0)
            app->lockMain.relock(app->smoothMain);
          else if (targetKnob == 1)
            app->lockX.relock(app->smoothX);
          else if (targetKnob == 2)
            app->lockY.relock(app->smoothY);
        }
      }
    }
  }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets) {
  if (midi_dev_addr != dev_addr || num_packets == 0)
    return;
  uint8_t cable_num;
  uint8_t buffer[48];
  while (true) {
    int32_t bytesRead =
        tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
    if (bytesRead <= 0)
      break;
    for (int i = 0; i < bytesRead; i += 3) {
      handle_midi_message(buffer + i, bytesRead - i);
    }
  }
}

void tuh_midi_tx_cb(uint8_t dev_addr) { (void)dev_addr; }
}
