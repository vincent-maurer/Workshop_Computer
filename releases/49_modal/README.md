# 49 Modal — Elements for Workshop Computer

A port of Mutable Instruments **Elements** (modal synthesis) to the Music Thing Modular Workshop Computer.

## Status: Phase 0 — Skeleton

Currently implements:
- ✅ 6-page parameter system with LED indicators
- ✅ Knob locking across page switches
- ✅ Switch debouncing (tap = next page, hold = prev page)
- ✅ Switch up = cycle resonator model
- ✅ Dual-core architecture (Core 0: audio ISR, Core 1: DSP engine)
- ✅ 24kHz DSP decimation via multicore FIFO
- ✅ Audio passthrough for testing
- ⬜ Exciter (Phase 2)
- ⬜ Resonator (Phase 3)
- ⬜ Reverb/Space (Phase 4)

## Controls

### Pages (cycle with switch down)

| Page | LED | Main | X | Y |
|------|-----|------|---|---|
| 0 | LED 0 | Strike Level | Blow Level | Bow Level |
| 1 | LED 1 | Strike Timbre | Blow Timbre | Bow Timbre |
| 2 | LED 2 | Strike Meta | Blow Meta | Envelope Shape |
| 3 | LED 3 | Geometry | Brightness | Damping |
| 4 | LED 4 | Position | Space | Model Select |
| 5 | LED 5 | Pitch | Strength | Fine Tune |

### I/O

| Jack | Function |
|------|----------|
| Audio In 1 | Blow external input |
| Audio In 2 | Strike external input |
| Audio Out 1 | Main (center) |
| Audio Out 2 | Aux (sides/reverb) |
| CV In 1 | V/Oct pitch |
| CV In 2 | Strength/accent |
| Pulse In 1 | Gate |
| Pulse In 2 | Model toggle |

## Credits

- **Émilie Gillet** — Original Elements DSP code (Mutable Instruments)
- **Volker Böhm** — SuperCollider mi-UGens port
- **Music Thing Modular** — Workshop Computer platform
