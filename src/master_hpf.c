/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file master_hpf.c
 * @brief Master HPF Implementation
 */

#include "master_hpf.h"

void hpf_compute_coefficients(JunoMasterHpf* hpf, int32_t pos) {
    float fs = hpf->sample_rate;
    hpf->position = pos;

    if (hpf->model == JUNO_MODEL_106) {
        // Model 106 Master HPF Circuit:
        // Position 0: Bass Boost (+4.5 dB low shelf around 120 Hz)
        // Position 1: Direct wire bypass (flat response, unity gain, zero phase shift)
        // Position 2: 225 Hz 1-pole high-pass (-6 dB/oct)
        // Position 3: 700 Hz 1-pole high-pass (-6 dB/oct)
        if (pos == 0) {
            // Bass boost: +4.5 dB low shelf at 120 Hz
            float gain_db = 4.5f;
            float a_gain = powf(10.0f, gain_db / 40.0f);
            float w0 = (2.0f * (float)M_PI * 120.0f) / fs;
            float cosw0 = cosf(w0);
            float sinw0 = sinf(w0);
            float alpha = sinw0 / (2.0f * 0.7071f);
            float sqrt_a = sqrtf(a_gain);
            float a0 = (a_gain + 1.0f) + (a_gain - 1.0f) * cosw0 + 2.0f * sqrt_a * alpha;

            hpf->b0 = (a_gain * ((a_gain + 1.0f) - (a_gain - 1.0f) * cosw0 + 2.0f * sqrt_a * alpha)) / a0;
            hpf->b1 = (2.0f * a_gain * ((a_gain - 1.0f) - (a_gain + 1.0f) * cosw0)) / a0;
            hpf->b2 = (a_gain * ((a_gain + 1.0f) - (a_gain - 1.0f) * cosw0 - 2.0f * sqrt_a * alpha)) / a0;
            hpf->a1 = (-2.0f * ((a_gain - 1.0f) + (a_gain + 1.0f) * cosw0)) / a0;
            hpf->a2 = ((a_gain + 1.0f) + (a_gain - 1.0f) * cosw0 - 2.0f * sqrt_a * alpha) / a0;
        } else if (pos == 1) {
            // Position 1 is flat wire bypass
            hpf->b0 = 1.0f;
            hpf->b1 = 0.0f;
            hpf->b2 = 0.0f;
            hpf->a1 = 0.0f;
            hpf->a2 = 0.0f;
        } else {
            // Positions 2 & 3: 1-pole high-pass (-6 dB/oct)
            float fc = (pos == 2) ? 225.0f : 700.0f;
            float k_tan = tanf(((float)M_PI * fc) / fs);
            float norm = 1.0f / (1.0f + k_tan);
            hpf->b0 = norm;
            hpf->b1 = -norm;
            hpf->b2 = 0.0f;
            hpf->a1 = (k_tan - 1.0f) * norm;
            hpf->a2 = 0.0f;
        }
    } else {
        // Model 60 Master HPF (Passive 1-Pole RC Network, -6 dB/oct):
        // Position 0: Direct wire bypass (flat response, unity gain, zero phase shift)
        // Position 1: 122 Hz 1-pole high-pass (-6 dB/oct)
        // Position 2: 269 Hz 1-pole high-pass (-6 dB/oct)
        // Position 3: 571 Hz 1-pole high-pass (-6 dB/oct)
        if (pos == 0) {
            hpf->b0 = 1.0f;
            hpf->b1 = 0.0f;
            hpf->b2 = 0.0f;
            hpf->a1 = 0.0f;
            hpf->a2 = 0.0f;
        } else {
            float fc = (pos == 1) ? 122.0f : ((pos == 2) ? 269.0f : 571.0f);
            float k_tan = tanf(((float)M_PI * fc) / fs);
            float norm = 1.0f / (1.0f + k_tan);
            hpf->b0 = norm;
            hpf->b1 = -norm;
            hpf->b2 = 0.0f;
            hpf->a1 = (k_tan - 1.0f) * norm;
            hpf->a2 = 0.0f;
        }
    }
}

void hpf_init(JunoMasterHpf* hpf, float sample_rate) {
    hpf->sample_rate = sample_rate;
    hpf->model = JUNO_MODEL_60;
    hpf->x1_l = hpf->x2_l = hpf->y1_l = hpf->y2_l = 0.0f;
    hpf->x1_r = hpf->x2_r = hpf->y1_r = hpf->y2_r = 0.0f;
    hpf->dc_g = tanf(((float)M_PI * 1.59f) / sample_rate);
    hpf->dc_state_l = hpf->dc_state_r = 0.0f;
    hpf->out_l = hpf->out_r = 0.0f;
    hpf_compute_coefficients(hpf, 0);
}

void hpf_process(JunoMasterHpf* hpf, float in_l, float in_r) {
    // 1. Direct Form I Biquad filtering (Left Channel)
    float y_l = hpf->b0 * in_l + hpf->b1 * hpf->x1_l + hpf->b2 * hpf->x2_l - hpf->a1 * hpf->y1_l - hpf->a2 * hpf->y2_l;
    hpf->x2_l = hpf->x1_l;
    hpf->x1_l = in_l;
    hpf->y2_l = hpf->y1_l;
    hpf->y1_l = y_l;

    // Direct Form I Biquad filtering (Right Channel)
    float y_r = hpf->b0 * in_r + hpf->b1 * hpf->x1_r + hpf->b2 * hpf->x2_r - hpf->a1 * hpf->y1_r - hpf->a2 * hpf->y2_r;
    hpf->x2_r = hpf->x1_r;
    hpf->x1_r = in_r;
    hpf->y2_r = hpf->y1_r;
    hpf->y1_r = y_r;

    // 2. AC coupling DC blocker (~1.59 Hz)
    float dc_v_l = (y_l - hpf->dc_state_l) * hpf->dc_g / (1.0f + hpf->dc_g);
    float dc_lp_l = hpf->dc_state_l + dc_v_l;
    hpf->dc_state_l = dc_lp_l + dc_v_l;
    hpf->out_l = y_l - dc_lp_l;

    float dc_v_r = (y_r - hpf->dc_state_r) * hpf->dc_g / (1.0f + hpf->dc_g);
    float dc_lp_r = hpf->dc_state_r + dc_v_r;
    hpf->dc_state_r = dc_lp_r + dc_v_r;
    hpf->out_r = y_r - dc_lp_r;
}
