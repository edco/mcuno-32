/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file ir3109_vcf.c
 * @brief Roland IR3109 4-Pole OTA Ladder Filter Implementation
 */

#include "ir3109_vcf.h"

void vcf_init(IR3109Filter* vcf, float sample_rate) {
    vcf->sample_rate = sample_rate;
    vcf->nyquist = sample_rate * 0.495f;
    vcf->s0 = vcf->s1 = vcf->s2 = vcf->s3 = 0.0f;
    vcf->ota_scale = 0.35f;
    vcf->inv_ota_scale = 1.0f / vcf->ota_scale;
    vcf_update_coefficients(vcf, 1000.0f, 0.0f);
}

void vcf_update_coefficients(IR3109Filter* vcf, float cutoff_hz, float resonance) {
    // 1. Frequency pre-warping with Nyquist guard-band protection
    float fc = cutoff_hz < 15.0f ? 15.0f : (cutoff_hz > vcf->nyquist ? vcf->nyquist : cutoff_hz);
    float frq_norm = fc / (vcf->sample_rate * 0.5f);
    float frq_over = frq_norm < 0.85f ? frq_norm : 0.85f;

    vcf->g = tanf(frq_over * (float)M_PI * 0.5f);

    // 2. Resonance curve equation:
    // ResK(res) = 0.811 * (exp(2.128 * res) - 1.0)
    float k = 0.811f * (expf(2.128f * resonance) - 1.0f);
    if (k > 3.0f) {
        float excess = k - 3.0f;
        k = 3.0f + excess / (1.0f + excess * 0.2f);
    }
    vcf->k = k < 5.0f ? k : 5.0f;

    // 3. Resonance peak-pull compensation:
    // Counteracts downward frequency pulling that occurs in analog OTA ladders
    // when resonance feedback approaches self-oscillation.
    float k_fit = vcf->k < 0.0f ? 0.0f : (vcf->k > 5.0f ? 5.0f : vcf->k);
    float log_m = 0.20646f + k_fit * (-0.0414f + k_fit * (0.00602f - k_fit * 0.00127f));
    if (vcf->k < 1.0f) log_m *= vcf->k;
    if (vcf->k > 3.7f) {
        float fade = (4.4f - vcf->k) / 0.7f;
        if (fade < 0.0f) fade = 0.0f;
        if (fade > 1.0f) fade = 1.0f;
        log_m *= fade;
    }
    float new_g = vcf->g * expf(log_m);
    vcf->g = new_g < 10.0f ? new_g : 10.0f;

    // 4. Compute cascaded stage gains
    vcf->g1 = vcf->g / (1.0f + vcf->g);
    vcf->g1_sq = vcf->g1 * vcf->g1;
    vcf->g1_cu = vcf->g1_sq * vcf->g1;
    vcf->ladder_gain = vcf->g1_sq * vcf->g1_sq;

    // 5. Zero-Delay Feedback denominator inversion
    vcf->inv_den = 1.0f / (1.0f + vcf->k * vcf->ladder_gain);
    float k_diff = vcf->k - 2.5f;
    float k_min = k_diff < 0.3f ? 0.3f : (k_diff > 1.0f ? 1.0f : k_diff);
    vcf->k_fb_scale = 4.2f * k_min;
    vcf->inv_k_fb_scale = 1.0f / vcf->k_fb_scale;
}

static inline float vcf_linear_stage(float s, float x, float g1, float* state_out) {
    float y = s + g1 * (x - s);
    *state_out = 2.0f * y - s;
    return y;
}

float vcf_process(IR3109Filter* vcf, float x) {
    /* Cache integrator states in local variables during stage evaluation */
    float s0 = vcf->s0;
    float s1 = vcf->s1;
    float s2 = vcf->s2;
    float s3 = vcf->s3;

    // 1. Zero-Delay Feedback instantaneous linear state predictor
    float s = (1.0f - vcf->g1) * (s0 * vcf->g1_cu + s1 * vcf->g1_sq + s2 * vcf->g1 + s3);
    const float INPUT_RATIO = 0.065f;
    const float DIRECT_GAIN = 0.142f;

    // 2. Feedback OTA differential stage with Pade saturation.
    float ota_diff = x * INPUT_RATIO - s;
    float ota_out = rational_soft_clip(ota_diff * vcf->k_fb_scale) * vcf->inv_k_fb_scale;
    float u = (x * DIRECT_GAIN + vcf->k * ota_out) * vcf->inv_den;

    // 3. 4-Pole OTA-C Integration Cascade, stages inlined.
    //    Each stage returns its output y and writes its new trapezoidal state
    //    through s_out; the next stage is fed y.
    float y = u;
    y = vcf_linear_stage(s0, y, vcf->g1, &s0);
    y = vcf_linear_stage(s1, y, vcf->g1, &s1);
    y = vcf_linear_stage(s2, y, vcf->g1, &s2);
    y = vcf_linear_stage(s3, y, vcf->g1, &s3);

    // 4. Micro-damping to guarantee numerical stability across denormal/subnormal ranges
    const float DAMP = 0.99998f;
    vcf->s0 = s0 * DAMP;
    vcf->s1 = s1 * DAMP;
    vcf->s2 = s2 * DAMP;
    vcf->s3 = s3 * DAMP;

    // 5. Output headroom gain
    return y * 8.59f;
}
