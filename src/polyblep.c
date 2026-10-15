/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file polyblep.c
 * @brief Analog jitter generator and saw-wave AC droop filter
 *
 * The PolyBLEP residual itself lives in polyblep.h as static inline functions
 * (poly_blep), used directly by the voice renderer.
 */

#include "polyblep.h"

void jitter_init(AnalogJitter* j, uint32_t seed) {
    j->b0 = 0.0f;
    j->b1 = 0.0f;
    j->b2 = 0.0f;
    j->seed = seed;
}

float jitter_step(AnalogJitter* j) {
    // 32-bit linear congruential generator (Knuth / Numerical Recipes parameters)
    j->seed = (1664525u * j->seed + 1013904223u);
    float white = ((float)j->seed / 4294967296.0f) * 2.0f - 1.0f;

    // 3-pole IIR filter yielding -10dB/decade (pink noise) power spectral slope
    j->b0 = 0.99765f * j->b0 + white * 0.099046f;
    j->b1 = 0.963f * j->b1 + white * 0.2965164f;
    j->b2 = 0.57f * j->b2 + white * 1.0526913f;

    float pink = (j->b0 + j->b1 + j->b2 + white * 0.1848f) * 0.05f;
    if (pink < -1.0f) return -1.0f;
    if (pink > 1.0f) return 1.0f;
    return pink;
}

void droop_init(DroopFilter* d, float sample_rate) {
    d->x1 = 0.0f;
    d->y1 = 0.0f;
    float fc = 2.0f; // Timing cap AC discharge corner frequency (retains sub-bass)
    float dt = 1.0f / sample_rate;
    float rc = 1.0f / (2.0f * (float)M_PI * fc);
    d->alpha = rc / (rc + dt);
}

float droop_process(DroopFilter* d, float x) {
    float y = d->alpha * (d->y1 + x - d->x1);
    d->x1 = x;
    d->y1 = fabsf(y) < 1e-15f ? 0.0f : y;
    return y;
}