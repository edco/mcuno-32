/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file chorus_mn3009.c
 * @brief Matsushita MN3009 BBD Chorus Implementation
 */

#include "chorus_mn3009.h"

/*
 * Saturating transfer polynomial for the BBD input stage.
 *
 * Odd polynomial approximation for tanh(x) over [-0.41, 0.41]:
 *   tanh(x) ~ x - x^3/3 + 2x^5/15 - 17x^7/315
 */
static inline float chorus_bbd_sat(float x) {
    float x2 = x * x;
    float p = -0.33333334f + x2 * (0.13333334f + x2 * -0.053968254f);
    return x + x * (x2 * p);
}

void chorus_init(JunoMn3009Chorus* c, float sample_rate) {
    c->sample_rate = sample_rate;
    for (int i = 0; i < JUNO_CHORUS_BUFFER_SIZE; i++) {
        c->buffer_l[i] = 0.0f;
        c->buffer_r[i] = 0.0f;
    }
    c->write_idx = 0;
    c->lfo_phase = 0.0f;
    c->pre_ic1_l = c->pre_ic2_l = c->pre_pole_l = 0.0f;
    c->pre_ic1_r = c->pre_ic2_r = c->pre_pole_r = 0.0f;
    c->post_ic1_l = c->post_ic2_l = 0.0f;
    c->post_ic1_r = c->post_ic2_r = 0.0f;
    c->mode = JUNO_CHORUS_OFF;
    c->out_l = c->out_r = 0.0f;

    // Matched 9,661 Hz 2nd-order Butterworth Sallen-Key low-pass filter (Q = 0.7071)
    float fc = 9661.0f;
    float q = 0.7071f;
    c->svf_g = tanf(((float)M_PI * fc) / sample_rate);
    float k = 1.0f / q;
    c->svf_a1 = 1.0f / (1.0f + c->svf_g * (c->svf_g + k));

    // 1-pole high-frequency tilt at 20 kHz
    c->pole_g = tanf(((float)M_PI * 20000.0f) / sample_rate);
}

/**
 * 4-Point Hermite fractional delay interpolation in Horner form:
 * ((c3 * f + c2) * f + c1) * f + c0
 */
static inline float chorus_hermite(const float* buffer, float read_pos) {
    int32_t idx_int = (int32_t)read_pos;
    float f = read_pos - (float)idx_int;

    uint32_t mask = JUNO_CHORUS_BUFFER_MASK;
    uint32_t i0 = (idx_int - 1) & mask;
    uint32_t i1 = idx_int & mask;
    uint32_t i2 = (idx_int + 1) & mask;
    uint32_t i3 = (idx_int + 2) & mask;

    float y0 = buffer[i0];
    float y1 = buffer[i1];
    float y2 = buffer[i2];
    float y3 = buffer[i3];

    float c0 = y1;
    float c1 = 0.5f * (y2 - y0);
    float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);

    return ((c3 * f + c2) * f + c1) * f + c0;
}

static inline float chorus_filter_pre_l(JunoMn3009Chorus* c, float x) {
    float g = c->svf_g;
    float a1 = c->svf_a1;
    float v3 = x - c->pre_ic2_l;
    float v1 = a1 * c->pre_ic1_l + a1 * g * v3;
    float v2 = c->pre_ic2_l + g * v1;
    c->pre_ic1_l = 2.0f * v1 - c->pre_ic1_l;
    c->pre_ic2_l = 2.0f * v2 - c->pre_ic2_l;

    float gp = c->pole_g;
    float vp = (v2 - c->pre_pole_l) * gp / (1.0f + gp);
    float lp = c->pre_pole_l + vp;
    c->pre_pole_l = lp + vp;
    return lp;
}

static inline float chorus_filter_pre_r(JunoMn3009Chorus* c, float x) {
    float g = c->svf_g;
    float a1 = c->svf_a1;
    float v3 = x - c->pre_ic2_r;
    float v1 = a1 * c->pre_ic1_r + a1 * g * v3;
    float v2 = c->pre_ic2_r + g * v1;
    c->pre_ic1_r = 2.0f * v1 - c->pre_ic1_r;
    c->pre_ic2_r = 2.0f * v2 - c->pre_ic2_r;

    float gp = c->pole_g;
    float vp = (v2 - c->pre_pole_r) * gp / (1.0f + gp);
    float lp = c->pre_pole_r + vp;
    c->pre_pole_r = lp + vp;
    return lp;
}

static inline float chorus_filter_post_l(JunoMn3009Chorus* c, float x) {
    float g = c->svf_g;
    float a1 = c->svf_a1;
    float v3 = x - c->post_ic2_l;
    float v1 = a1 * c->post_ic1_l + a1 * g * v3;
    float v2 = c->post_ic2_l + g * v1;
    c->post_ic1_l = 2.0f * v1 - c->post_ic1_l;
    c->post_ic2_l = 2.0f * v2 - c->post_ic2_l;
    return v2;
}

static inline float chorus_filter_post_r(JunoMn3009Chorus* c, float x) {
    float g = c->svf_g;
    float a1 = c->svf_a1;
    float v3 = x - c->post_ic2_r;
    float v1 = a1 * c->post_ic1_r + a1 * g * v3;
    float v2 = c->post_ic2_r + g * v1;
    c->post_ic1_r = 2.0f * v1 - c->post_ic1_r;
    c->post_ic2_r = 2.0f * v2 - c->post_ic2_r;
    return v2;
}

void chorus_process(JunoMn3009Chorus* c, float in_l, float in_r) {
    if (c->mode == JUNO_CHORUS_OFF) {
        c->out_l = in_l;
        c->out_r = in_r;
        return;
    }

    float fs = c->sample_rate;
    const float CENTER_DELAY_MS = 3.30f;
    const float CLOCK_TRIM = 0.015f; // ±1.5% clock offset between lines

    float rate = 0.393f;
    float depth_ms = 2.13f;
    float lfo = 0.0f;

    // LFO frequencies and waveforms
    if (c->mode == JUNO_CHORUS_I) {
        rate = 0.393f;
        depth_ms = 2.13f;
        c->lfo_phase += (2.0f * (float)M_PI * rate) / fs;
        if (c->lfo_phase >= 2.0f * (float)M_PI) c->lfo_phase -= 2.0f * (float)M_PI;
        float raw_tri = 2.0f * fabsf((c->lfo_phase / (float)M_PI) - 1.0f) - 1.0f;
        lfo = raw_tri * (1.5f - 0.5f * raw_tri * raw_tri); // Parabolic rounded triangle
    } else if (c->mode == JUNO_CHORUS_II) {
        rate = 0.842f;
        depth_ms = 1.71f;
        c->lfo_phase += (2.0f * (float)M_PI * rate) / fs;
        if (c->lfo_phase >= 2.0f * (float)M_PI) c->lfo_phase -= 2.0f * (float)M_PI;
        float raw_tri = 2.0f * fabsf((c->lfo_phase / (float)M_PI) - 1.0f) - 1.0f;
        lfo = raw_tri * (1.5f - 0.5f * raw_tri * raw_tri);
    } else {
        // Mode I+II: Leslie/rotary fast vibrato-chorus
        rate = 7.85f;
        depth_ms = 0.236f;
        c->lfo_phase += (2.0f * (float)M_PI * rate) / fs;
        if (c->lfo_phase >= 2.0f * (float)M_PI) c->lfo_phase -= 2.0f * (float)M_PI;
        lfo = sinf(c->lfo_phase);
    }

    float delay0_ms = (CENTER_DELAY_MS + depth_ms * lfo) * (1.0f - CLOCK_TRIM);
    if (delay0_ms < 0.1f) delay0_ms = 0.1f;
    float delay1_ms = (CENTER_DELAY_MS - depth_ms * lfo) * (1.0f + CLOCK_TRIM);
    if (delay1_ms < 0.1f) delay1_ms = 0.1f;

    float delay_samp0 = (delay0_ms * 0.001f) * fs;
    float delay_samp1 = (delay1_ms * 0.001f) * fs;

    // 1. Anti-aliasing pre-filtering (9.66 kHz Butterworth SVF + 20 kHz pole)
    float filtered_l = chorus_filter_pre_l(c, in_l);
    float filtered_r = chorus_filter_pre_r(c, in_r);

    // 2. BBD soft-knee input stage saturation
    const float SAT_DRIVE = 0.12f;
    float sat_l = chorus_bbd_sat(filtered_l * SAT_DRIVE) / SAT_DRIVE;
    float sat_r = chorus_bbd_sat(filtered_r * SAT_DRIVE) / SAT_DRIVE;

    // 3. Write into circular delay buffer
    uint32_t w_idx = c->write_idx;
    c->buffer_l[w_idx] = sat_l;
    c->buffer_r[w_idx] = sat_r;
    c->write_idx = (w_idx + 1) & JUNO_CHORUS_BUFFER_MASK;

    // 4. Fractional delay readout with 4-point Hermite interpolation
    float read_pos0 = (float)w_idx - delay_samp0 + (float)(JUNO_CHORUS_BUFFER_SIZE * 4);
    while (read_pos0 >= (float)JUNO_CHORUS_BUFFER_SIZE) read_pos0 -= (float)JUNO_CHORUS_BUFFER_SIZE;
    float read_pos1 = (float)w_idx - delay_samp1 + (float)(JUNO_CHORUS_BUFFER_SIZE * 4);
    while (read_pos1 >= (float)JUNO_CHORUS_BUFFER_SIZE) read_pos1 -= (float)JUNO_CHORUS_BUFFER_SIZE;

    float wet0 = chorus_hermite(c->buffer_l, read_pos0);
    float wet1 = chorus_hermite(c->buffer_r, read_pos1);

    // 5. Reconstruction post-filtering
    wet0 = chorus_filter_post_l(c, wet0);
    wet1 = chorus_filter_post_r(c, wet1);

    // 6. Charge-Transfer Efficiency (CTE) clock-dependent attenuation
    float clock0 = 256.0f / (2.0f * delay0_ms * 0.001f);
    if (clock0 < 5000.0f) clock0 = 5000.0f;
    float clock1 = 256.0f / (2.0f * delay1_ms * 0.001f);
    if (clock1 < 5000.0f) clock1 = 5000.0f;

    const float INV_CENTER = 1.0f / 40000.0f;
    const float CTE_COEFF = 4468.0f;
    const float GAIN_TRIM = 0.04f;

    float gain0 = (1.0f + GAIN_TRIM) * (1.0f - CTE_COEFF * (1.0f / clock0 - INV_CENTER));
    float gain1 = (1.0f - GAIN_TRIM) * (1.0f - CTE_COEFF * (1.0f / clock1 - INV_CENTER));
    wet0 *= gain0;
    wet1 *= gain1;

    // 7. IC6 summer mixing gains
    const float DRY_GAIN = 0.863f;
    const float WET_GAIN = 1.257f;

    c->out_l = in_l * DRY_GAIN + wet0 * WET_GAIN;
    c->out_r = in_r * DRY_GAIN + wet1 * WET_GAIN;
}
