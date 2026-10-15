/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file master_hpf.h
 * @brief Master High-Pass Filter (HPF) & Bass-Boost Circuit
 *
 * Implements:
 * - Master circuit topology modeling:
 *     Position 0: Direct wire bypass (flat response, unity gain, zero phase shift).
 *     Position 1: 122 Hz 2nd-order high-pass Butterworth.
 *     Position 2: 269 Hz 2nd-order high-pass Butterworth.
 *     Position 3: 571 Hz 2nd-order high-pass Butterworth.
 * - AC coupling DC blocker cap (~1.59 Hz).
 */

#ifndef JUNO_MASTER_HPF_H
#define JUNO_MASTER_HPF_H

#include "polyblep.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JUNO_HPF_POS_0 = 0,
    JUNO_HPF_POS_1 = 1,
    JUNO_HPF_POS_2 = 2,
    JUNO_HPF_POS_3 = 3
} JunoHpfPos;

typedef enum {
    JUNO_MODEL_60 = 0,
    JUNO_MODEL_106 = 1
} JunoModel;

/**
 * @brief Master HPF Biquad State
 */
typedef struct {
    int32_t position; /**< 0, 1, 2, or 3 */
    int32_t model;    /**< 0 = Model 60, 1 = Model 106 */
    float sample_rate; /**< Audio sample rate in Hz */

    // Direct Form I biquad delay registers
    float x1_l, x2_l, y1_l, y2_l;
    float x1_r, x2_r, y1_r, y2_r;

    // Normalized biquad coefficients
    float b0, b1, b2, a1, a2;

    // AC coupling DC blocker capacitor (~1.59 Hz)
    float dc_g;
    float dc_state_l;
    float dc_state_r;

    float out_l;
    float out_r;
} JunoMasterHpf;

/**
 * @brief Initialize master HPF biquad state and DC blocking filter
 * @param hpf Pointer to JunoMasterHpf struct
 * @param sample_rate Operating audio sample rate in Hz
 */
void hpf_init(JunoMasterHpf* hpf, float sample_rate);

/**
 * @brief Compute biquad coefficients for specified switch position and synthesizer model
 * @param hpf Pointer to JunoMasterHpf struct
 * @param pos Switch position (0, 1, 2, or 3)
 */
void hpf_compute_coefficients(JunoMasterHpf* hpf, int32_t pos);

/**
 * @brief Process one stereo sample pair through master HPF and DC blocker
 * @param hpf Pointer to JunoMasterHpf struct
 * @param in_l Left input audio sample
 * @param in_r Right input audio sample
 */
void hpf_process(JunoMasterHpf* hpf, float in_l, float in_r);

#ifdef __cplusplus
}
#endif

#endif // JUNO_MASTER_HPF_H
