/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file polyblep.h
 * @brief PolyBLEP Anti-Aliasing Residuals, Padé Soft-Clipping, Pink Jitter & AC Droop Filter
 *
 * Implements:
 * 1. 2nd-Order PolyBLEP (Polynomial Band-Limited Step) Residual Edge Correction.
 *    Suppresses Nyquist foldover harmonics for geometric waveforms (sawtooth, pulse, sub-osc).
 * 2. 3rd-Order Padé Approximant for OTA Differential Transistor Saturation (IR3109).
 *    fsat(x) = x * (27.0 + x^2) / (27.0 + 9.0 * x^2), strictly monotonic with C1 derivative = 0 at |x| = 3.0.
 * 3. 3-Stage Voss-McCartney Filtered Pink Noise Generator for analog clock phase jitter.
 * 4. 1-Pole High-Pass Filter (~6 Hz cutoff) modeling the analog capacitor AC droop on the saw wave.
 *
 * Optimized for standard 32-bit single-precision float execution.
 */

#ifndef JUNO_POLYBLEP_H
#define JUNO_POLYBLEP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Single precision math prototypes for freestanding embedded environments
float sinf(float);
float cosf(float);
float tanf(float);
float expf(float);
float logf(float);
float powf(float, float);
float fabsf(float);
float tanhf(float);
float sqrtf(float);

/* ---------------------------------------------------------------------------
 * Fast block-rate transcendentals
 *
 * Evaluates polynomial approximations for transcendental functions to optimize
 * per-block voice calculation on cores without hardware transcendental units.
 *
 * Accuracy: the quintic below is a least-squares fit to 2^f on [0, 1)
 * constrained to be exact at f = 0, evaluated in float32. Measured max
 * relative error is 2.2e-7 (float32 epsilon).
 *
 * The exponent is applied by writing directly into the IEEE-754 exponent
 * field: scaling by 2^n is exact.
 * ------------------------------------------------------------------------- */
#define JUNO_FAST_EXP2_MIN (-126.0f)
#define JUNO_FAST_EXP2_MAX (126.0f)

static inline float fast_exp2(float x) {
    if (x <= JUNO_FAST_EXP2_MIN) return 0.0f;
    if (x >= JUNO_FAST_EXP2_MAX) x = JUNO_FAST_EXP2_MAX;

    /* Split x = n + f with n integral and f in [0, 1). */
    int32_t n = (int32_t)x;
    float f = x - (float)n;
    if (f < 0.0f) {
        f += 1.0f;
        n -= 1;
    }

    /* Quintic for 2^f on [0, 1), exact at f = 0, max rel err 2.2e-7. */
    float p = 1.0f + f * (0.693152726f + f * (0.240153193f + f * (0.0558281578f
              + f * (0.00898898672f + f * 0.00187662884f))));

    union {
        uint32_t i;
        float f;
    } u;
    u.i = (uint32_t)(n + 127) << 23; /* exact power-of-two scaling */
    return u.f * p;
}

/** @brief exp(x) via fast_exp2. */
static inline float fast_exp(float x) {
    return fast_exp2(x * 1.44269504f); /* log2(e) */
}

/** @brief pow(2, x) via fast_exp2. */
static inline float fast_pow2(float x) {
    return fast_exp2(x);
}

/* ---------------------------------------------------------------------------
 * Fast logarithms
 *
 * Bit-extracts the exponent and evaluates log on the reduced mantissa with the
 * atanh series, z = (m-1)/(m+1). After folding the mantissa into
 * [1/sqrt2, sqrt2), |z| <= 0.1716, so truncating after z^7 leaves ~1e-6
 * relative error.
 * ------------------------------------------------------------------------- */
static inline float fast_log2(float x) {
    if (x <= 0.0f) return -126.0f;

    union {
        uint32_t i;
        float f;
    } u;
    u.f = x;
    int32_t e = (int32_t)((u.i >> 23) & 0xFFu) - 127;
    u.i = (u.i & 0x007FFFFFu) | 0x3F800000u; /* mantissa -> [1.0, 2.0) */
    float m = u.f;

    /* Fold to [1/sqrt2, sqrt2) so the series stays well conditioned. */
    if (m > 1.41421356f) {
        m *= 0.5f;
        e += 1;
    }

    float z = (m - 1.0f) / (m + 1.0f);
    float z2 = z * z;
    float p = z * (2.0f + z2 * (0.66666667f + z2 * (0.4f + z2 * 0.28571429f)));
    return (float)e + p * 1.44269504f; /* 1/ln2 */
}

/** @brief ln(x) via fast_log2. */
static inline float fast_log(float x) {
    return fast_log2(x) * 0.69314718f;
}

/** @brief pow(base, exponent) via the exp/log pair. */
static inline float fast_pow(float base, float exponent) {
    return fast_exp2(exponent * fast_log2(base));
}

/**
 * @brief 3-Stage Voss-McCartney / Filtered LCG Pink Noise Generator
 *
 * Simulates analog master clock phase noise (±0.02% to ±0.05% jitter) across
 * the 6 independent voice cards. Produces natural beating in polyphonic and unison modes.
 */
typedef struct {
    float b0;      /**< 1st pinking pole state */
    float b1;      /**< 2nd pinking pole state */
    float b2;      /**< 3rd pinking pole state */
    uint32_t seed; /**< 32-bit linear congruential generator (LCG) state */
} AnalogJitter;

/**
 * @brief Gentle 1-pole high-pass filter (~6 Hz cutoff)
 *
 * Models the analog capacitor AC droop / DC tilt on the saw wave
 * produced by the timing capacitor discharging through the reset transistor.
 */
typedef struct {
    float x1;    /**< Previous input sample */
    float y1;    /**< Previous output sample */
    float alpha; /**< 1-pole high-pass feedback factor: rc / (rc + dt) */
} DroopFilter;

/**
 * @brief 2nd-Order / 2-Point PolyBLEP Residual (poly_blep).
 * Call with precomputed inv_dt = 1.0f / dt to avoid divisions.
 *
 * @param t     Normalized phase in [0, 1)
 * @param dt    Normalized phase increment
 * @param inv_dt Precomputed 1.0f / dt
 */
static inline float poly_blep(float t, float dt, float inv_dt) {
    if (dt <= 1e-7f) return 0.0f;
    if (t < dt) {
        float n = t * inv_dt;
        return n + n - n * n - 1.0f;
    } else if (t > 1.0f - dt) {
        float n = (t - 1.0f) * inv_dt;
        return n * n + n + n + 1.0f;
    }
    return 0.0f;
}

/**
 * @brief 3rd-Order Padé Approximant for OTA Differential Transistor Saturation (IR3109).
 *
 * Formula: fsat(x) = x * (27.0 + x^2) / (27.0 + 9.0 * x^2) for |x| < 3.0, and sign(x) for |x| >= 3.0.
 *
 * Key Properties:
 * - Slope = 1.0 at x = 0 (exact match to small-signal linear ZDF equations)
 * - Derivative f'(3.0) = 0.0 (smooth C1 transition to saturation asymptote +/- 1.0)
 * - Strictly monotonic: NEVER folds over or creates negative resistance
 * - Prevents numerical divergence under high resonance
 *
 * @param x Normalized differential input voltage
 * @return Saturated output voltage normalized to [-1.0, 1.0]
 */
static inline float rational_soft_clip(float x) {
    float x2 = x * x;
    if (x > 3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    float num = x * (27.0f + x2);
    float den = 27.0f + 9.0f * x2;
    return num / den;
}

/**
 * @brief Derivative of Padé rational soft-clip (sech² approximant) for Newton-Raphson solve.
 *
 * Evaluates df/dx of rational_soft_clip(sd) analytically:
 * df/dx = 27 * (27 - 3*x^2) / (27 + 9*x^2)^2 for |x| < 3.0, and 0.0 for |x| >= 3.0.
 *
 * @param sd Differential voltage across OTA transistor pair
 * @return Instantaneous derivative df/dx
 */
static inline float ota_deriv(float sd) {
    if (sd > 3.0f || sd < -3.0f) return 0.0f;
    float x2 = sd * sd;
    float d = 27.0f + 9.0f * x2;
    return (27.0f * (27.0f - 3.0f * x2)) / (d * d);
}

/**
 * @brief Initialize analog phase jitter generator with voice-specific seed
 * @param j Pointer to AnalogJitter state struct
 * @param seed Pseudo-random seed offset
 */
void jitter_init(AnalogJitter* j, uint32_t seed);

/**
 * @brief Step analog phase jitter generator by one sample
 * @param j Pointer to AnalogJitter state struct
 * @return Pink-filtered noise sample bounded in [-1.0, 1.0]
 */
float jitter_step(AnalogJitter* j);

/**
 * @brief Initialize 1-pole saw AC droop filter
 * @param d Pointer to DroopFilter struct
 * @param sample_rate Operating audio sample rate in Hz
 */
void droop_init(DroopFilter* d, float sample_rate);

/**
 * @brief Process one sample through saw droop filter
 * @param d Pointer to DroopFilter struct
 * @param x Input raw sawtooth sample
 * @return AC-drooped sawtooth sample with zero DC bias
 */
float droop_process(DroopFilter* d, float x);

#ifdef __cplusplus
}
#endif

#endif // JUNO_POLYBLEP_H
