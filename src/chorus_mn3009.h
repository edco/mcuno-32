/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file chorus_mn3009.h
 * @brief Matsushita MN3009 Bucket Brigade Device (BBD) Chorus Engine
 *
 * Implements:
 * - Hardware delay geometry: 3.30 ms center delay with ±1.5% clock trim between lines
 * - Mode I: 0.393 Hz rate, 2.13 ms depth, analog triangle LFO
 * - Mode II: 0.842 Hz rate, 1.71 ms depth, analog triangle LFO
 * - Mode I+II: 7.85 Hz rate, 0.236 ms depth, rotary sine LFO
 * - Matched 9,661 Hz 2nd-order Butterworth pre-filter and reconstruction post-filter
 * - BBD soft-knee saturation: tanhf(x * drive) / drive
 * - 4-Point Hermite fractional delay interpolation in Horner form
 * - IC6 summer gains: kDry = 0.863, kWet = 1.257
 * - Clock-dependent charge-transfer efficiency (CTE) high-frequency attenuation modeling
 */

#ifndef JUNO_CHORUS_MN3009_H
#define JUNO_CHORUS_MN3009_H

#include "polyblep.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JUNO_CHORUS_BUFFER_SIZE 4096
#define JUNO_CHORUS_BUFFER_MASK (JUNO_CHORUS_BUFFER_SIZE - 1)

typedef enum {
    JUNO_CHORUS_OFF = 0,
    JUNO_CHORUS_I = 1,
    JUNO_CHORUS_II = 2,
    JUNO_CHORUS_I_II = 3
} JunoChorusMode;

/**
 * @brief Matsushita MN3009 BBD Stereo Chorus Circuit State
 */
typedef struct {
    float sample_rate;
    float buffer_l[JUNO_CHORUS_BUFFER_SIZE];
    float buffer_r[JUNO_CHORUS_BUFFER_SIZE];
    uint32_t write_idx;

    float lfo_phase;

    // Pre-filter state registers (Left & Right)
    float pre_ic1_l, pre_ic2_l, pre_pole_l;
    float pre_ic1_r, pre_ic2_r, pre_pole_r;

    // Post-filter state registers (Left & Right)
    float post_ic1_l, post_ic2_l;
    float post_ic1_r, post_ic2_r;

    // Filter coefficients computed at initialization
    float svf_g;
    float svf_a1;
    float pole_g;

    int32_t mode;

    float out_l;
    float out_r;
} JunoMn3009Chorus;
typedef JunoMn3009Chorus JunoMN3009Chorus;

/**
 * @brief Initialize MN3009 chorus state and pre-compute 9,661 Hz anti-aliasing / reconstruction filters
 * @param c Pointer to JunoMN3009Chorus struct
 * @param sample_rate Operating audio sample rate in Hz
 */
void chorus_init(JunoMn3009Chorus* c, float sample_rate);

/**
 * @brief Process one stereo sample pair through the MN3009 BBD chorus engine
 * @param c Pointer to JunoMN3009Chorus struct
 * @param in_l Left input audio sample
 * @param in_r Right input audio sample
 */
void chorus_process(JunoMn3009Chorus* c, float in_l, float in_r);

#ifdef __cplusplus
}
#endif

#endif // JUNO_CHORUS_MN3009_H
