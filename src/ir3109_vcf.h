/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file ir3109_vcf.h
 * @brief Roland IR3109 4-Pole Operational Transconductance Amplifier (OTA) Ladder Filter
 *
 * Implements:
 * - Topology-Preserving Transform (TPT) / Zero-Delay Feedback (ZDF) 4-pole low-pass ladder.
 * - Hardware IR3109 custom Roland quad-OTA IC modeling.
 * - Per-stage local Padé rational soft-clipping for differential BJT input stages.
 * - Single-step Newton-Raphson nonlinear solver for implicit ODE integrator stages:
 *     y = s + g * tanh(x - y)
 * - Resonance mapping:
 *     k = 0.811 * (exp(2.128 * res) - 1.0) with high-resonance soft-limiting.
 * - Resonance peak-pull compensation.
 * - High-resonance feedback scaling preventing numerical instability while preserving
 *   analog self-oscillation and pitch-tracking sine tone.
 */

#ifndef JUNO_IR3109_VCF_H
#define JUNO_IR3109_VCF_H

#include "polyblep.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Roland IR3109 4-Pole ZDF Low-Pass Filter Card
 */
typedef struct {
    // Integrator state registers for the 4 cascaded OTA stages
    float s0;           /**< State of 1st OTA integrator stage */
    float s1;           /**< State of 2nd OTA integrator stage */
    float s2;           /**< State of 3rd OTA integrator stage */
    float s3;           /**< State of 4th OTA integrator stage */

    // Precomputed block-rate coefficients (updated at 32-sample DMA block intervals)
    float g;            /**< Bilinear warped cutoff frequency: tan(pi * fc / fs) */
    float g1;           /**< One-pole gain coefficient: g / (1.0 + g) */
    float g1_sq;         /**< g1^2 */
    float g1_cu;         /**< g1^3 */
    float ladder_gain;         /**< Total forward ladder gain: g1^4 */
    float k;            /**< Resonance feedback coefficient */
    float inv_den;       /**< ZDF denominator inverse: 1.0 / (1.0 + k * G) */
    float k_fb_scale;     /**< Dynamic feedback loop saturation scaling */
    float inv_k_fb_scale;  /**< Inverse feedback saturation scaling */
    float ota_scale;     /**< Per-stage differential BJT input drive scale (default 0.35) */
    float inv_ota_scale;  /**< Cached 1/ota_scale. ota_scale is patch-constant, avoiding recomputing it */

    float sample_rate;   /**< Audio sample rate in Hz */
    float nyquist;      /**< Guard-banded Nyquist limit (0.495 * sample_rate) */
} IR3109Filter;

/**
 * @brief Initialize IR3109 filter state registers and defaults
 * @param vcf Pointer to IR3109Filter struct
 * @param sample_rate Operating audio sample rate in Hz
 */
void vcf_init(IR3109Filter* vcf, float sample_rate);

/**
 * @brief Update filter tuning and resonance coefficients at block boundaries (e.g. every 32 samples)
 *
 * Implements bilinear pre-warping, resonance curve fitting,
 * and peak-pull compensation.
 *
 * @param vcf Pointer to IR3109Filter struct
 * @param cutoff_hz Cutoff frequency in Hz (clamped between 15 Hz and Nyquist)
 * @param resonance Resonance parameter in range [0.0, 1.0]
 */
void vcf_update_coefficients(IR3109Filter* vcf, float cutoff_hz, float resonance);

/**
 * @brief Process one single audio sample through the 4-pole IR3109 OTA ladder filter
 *
 * Solves the implicit Zero-Delay Feedback loop using the precomputed linear matrix estimate,
 * then processes each of the 4 OTA integrator stages with local Newton-Raphson Padé rational
 * non-linear soft-clipping.
 *
 * @param vcf Pointer to IR3109Filter struct
 * @param x Input audio sample from DCO mixer
 * @return Filtered audio sample
 */
float vcf_process(IR3109Filter* vcf, float x);

#ifdef __cplusplus
}
#endif

#endif // JUNO_IR3109_VCF_H
