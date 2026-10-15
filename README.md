# mcuno-32

A lightweight, high-performance polyphonic synthesizer engine implemented in standard C99, modeled on the architecture of classic Roland analog polyphonic synthesizers (Juno-60 / Juno-106).

Designed for 32-bit microcontrollers with minimal, single-precision FPUs; specifically the JieLi AC791N series found in M-VAVE's SMK-37 and FM-1 MIDI controllers/synths.

## Development

Primarily vibe-coded using Google Gemini. Prodded until it ran within the available CPU cycles and still sounded more-or-less as it should.

## Features

- **6-Voice Polyphony**: Discrete voice cards featuring LRU dynamic voice allocation, Poly 1, Poly 2, and Unison modes.
- **PolyBLEP DCO**: Antialiased Sawtooth and Pulse/PWM waveforms with sub-oscillator and pink noise generator.
- **IR3109 4-Pole ZDF VCF**: Topology-Preserving Transform / Zero-Delay Feedback OTA ladder low-pass filter with nonlinear soft-clipping and resonance self-oscillation.
- **Master HPF**: 4-position high-pass filter with analog bass boost and DC blocking coupling capacitors.
- **MN3009 BBD Stereo Chorus**: Bucket Brigade Device delay line modeling with dual triangle/sine LFO modulation modes.
- **Analog Quirks**: Power rail sag emulation, component tolerance dispersion, and analog clock phase jitter.
- **Microcontroller-Optimized DSP**: Block-rate processing (32-sample sub-blocks) with fast IEEE-754 polynomial approximations for transcendental functions (`exp`, `log`, `pow`).

## Basic Usage

```c
#include "juno_engine.h"

JunoEngine engine;
const float sampleRate = 48000.0f;

// 1. Initialize engine
juno_engine_init(&engine, sampleRate);

// 2. Play a note (MIDI note 60, velocity 0.8)
juno_engine_note_on(&engine, 60, 0.8f);

// 3. Render audio in blocks (up to 32 samples)
float outL[32];
float outR[32];
juno_engine_process_block(&engine, outL, outR, 32);

// 4. Release note
juno_engine_note_off(&engine, 60);
```

## Building & Testing

To build the standalone library and run tests:

```bash
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## License

Copyright (c) 2026 Ed Coffey.

This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
