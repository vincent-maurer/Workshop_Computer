# README.md

# Wild Pebble

Still very much in Beta testing

A generative rhythm and melody organism for the
Music Thing Modular Workshop Computer.

Inspired by the spirit of Jonah Senzel's Pet Rock.

Wild Pebble creates evolving rhythmic structures, quantised melodic patterns, drum voices, and modulation voltages that slowly transform over time while remaining musically connected.

The goal is not precise repeatability, but constrained musical evolution.

---

# Features

* Dual probabilistic trigger streams
* Quantised melodic CV generation
* Internal kick and snare percussion voices
* Self-mutating sequence behaviour
* Phrase replication and variation
* Slowly evolving scale randomisation
* Internal tension modulation
* External or internal clocking
* Swing timing modes
* Low CPU usage

---

# Outputs

## Pulse Output 1

Primary rhythm stream.

Used internally to drive:

* melodic progression
* kick drum voice

---

## Pulse Output 2

Derived companion rhythm stream.

Used internally to:

* trigger the snare voice
* create correlated rhythmic variation

---

## CV Output 1

Quantised melodic pitch output.

Generated from evolving scale-constrained note sequences.

Wild Pebble slowly changes between related scales over time depending on mutation amount and switch mode, creating harmonic drift without fully losing musical coherence.

---

## CV Output 2

Energy/tension modulation output.

A smoothed evolving modulation source derived from:

* step energy
* accent
* internal tension state

---

## Audio Output 1

Kick drum voice driven from the primary trigger stream.

Produces deep evolving bass drum patterns with long decaying tails and pitch movement tied to sequence accents.

---

## Audio Output 2

Snare and percussion voice driven from the companion trigger stream.

Combines tonal percussion with noise elements for shifting rhythmic textures ranging from soft clicks to noisy snare bursts.

---

# Controls

## Main Knob

Controls internal clock speed.

Ignored when external clock is present.

---

## Clock Range

Wild Pebble’s internal clock ranges from approximately:

* **~90 BPM** at the slowest setting
* to fast audio-rate adjacent rhythmic speeds at the highest setting

The clock is designed to move smoothly from slow generative sequencing into dense evolving percussion patterns.

Swing amount varies depending on switch mode and affects every second step of the internal clock.

---

## X Knob

Controls rhythmic density.

Higher values increase trigger probability.
The range of probability of a trigger is from 5% to almost 100% chance. I recommend starting with the knob at 12 o'clock

CV Input 1 modulates density.

---

## Y Knob

Controls mutation intensity.

Higher values increase:

* sequence mutation
* melodic movement
* scale changes
* structural instability

CV Input 2 modulates mutation amount.

---

## Switch Modes

### Up

* stable melodic motion
* restrained mutation
* slower harmonic movement
* tighter rhythms

### Middle

* balanced mutation
* moderate swing
* evolving melodic variation
* gradual harmonic drift

### Down

* aggressive mutation
* strongest swing
* wider melodic jumps
* more active scale changes
* denser companion rhythms

---

# External Inputs

## Pulse Input 1

External clock input.

Automatically overrides the internal clock while active.

---

## Pulse Input 2

Freeze input.

While held high:

* mutation is disabled
* current structure is preserved

Clocking and playback continue normally.

---

# LED Behaviour

Wild Pebble uses the Workshop Computer’s 6 LEDs as a live visualisation of rhythm, density, mutation, and system state.

| LED   | Function     | Behaviour                                           |
| ----- | ------------ | --------------------------------------------------- |
| LED 1 | Main Trigger | Flashes when `PulseOut1` fires                      |
| LED 2 | Density      | Brightness follows Density control (`Knob X + CV1`) |
| LED 3 | Mutation     | Brightness follows Mutation amount (`Knob Y + CV2`) |
| LED 4 | Energy       | Displays smoothed internal energy/modulation level  |
| LED 5 | Clock Source | Fully lit when external clock is active             |
| LED 6 | Tension      | Displays evolving internal tension state             |

---

# Sequencing Behaviour

Wild Pebble continuously evolves using:

* probabilistic trigger generation
* constrained mutation resistance
* phrase copying
* harmonic rotation
* evolving scale selection
* tension cycling

The interaction between rhythm mutation, melodic drift, repeating phrases, and gradual scale movement creates patterns that evolve organically while remaining playable and musically usable.

---

# Building

Requires:

* Raspberry Pi Pico SDK
* Workshop Computer framework

Typical build flow:

```bash
mkdir build
cd build
cmake ..
make
```

---

# Philosophy

Wild Pebble is intended to feel less like a fixed sequencer and more like a small autonomous musical system.

The interaction between:

* mutation
* repetition
* probability
* tension
* harmonic drift
* evolving drum voices
* slow scale randomisation

creates continuously shifting patterns that remain musically coherent and performance-friendly.
Tension evolves slowly over time with one full cycle taking 1440 sequencer steps. 

Wild Pebble contains an internal evolving energy system.

Each step in the sequence carries its own energy state which slowly changes over time through mutation, repetition, and interaction with the global tension cycle.

Energy influences:

* modulation movement
* rhythmic intensity
* dynamic behaviour

Some parts of the sequence gradually become:

* more active
* more unstable
* more forceful

while others become:

* restrained
* sparse
* quieter

This creates long-term phrasing and shifting musical behaviour even when rhythmic structures repeat.

The smoothed Energy CV output exposes this evolving internal state to the outside patch, allowing Wild Pebble to animate filters, VCAs, LPGs, reverbs, FM depth, and other modulation destinations with continuously changing expressive movement.

Together with the slow tension cycle, the energy system helps Wild Pebble behave less like a traditional sequencer and more like a small autonomous musical organism.

LED 4 and 6 now display their respective states more clearly
