/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file voice.c
 * @brief MCUno-32 Single Voice Card Implementation
 */

#include "voice.h"

// Attack tau table (seconds):
static const float J60_ATTACK_TAU[11] = {
    0.000558f, 0.001674f, 0.008762f, 0.029468f, 0.064015f, 0.120998f,
    0.238481f, 0.495993f, 0.607950f, 1.392486f, 1.674332f
};

// DCO LFO slider response curve to semitones (0 to ~3.95 semitones):
// Sliders 0.0..0.85 provide musical vibrato (0..1 semitone), with the top 15% expanding up to 3.95 semitones.
static const float J60_DCO_LFO_SEMITONES[65] = {
    0.0f, 0.0f, 0.0227f, 0.04496f, 0.06493f, 0.08306f, 0.09971f, 0.11515f,
    0.12961f, 0.14326f, 0.15623f, 0.16867f, 0.18065f, 0.19228f, 0.20362f, 0.21473f,
    0.22568f, 0.23651f, 0.24728f, 0.25801f, 0.26876f, 0.27956f, 0.29044f, 0.30144f,
    0.3126f, 0.32393f, 0.33549f, 0.34729f, 0.35938f, 0.37179f, 0.38456f, 0.39773f,
    0.41133f, 0.42541f, 0.44002f, 0.45521f, 0.47103f, 0.48754f, 0.50481f, 0.5229f,
    0.54191f, 0.56191f, 0.58301f, 0.6053f, 0.62893f, 0.65402f, 0.68073f, 0.70925f,
    0.73978f, 0.77257f, 0.80787f, 0.84603f, 0.8874f, 0.93246f, 0.99087f, 1.09905f,
    1.21846f, 1.35098f, 1.49894f, 1.66526f, 1.85363f, 2.19256f, 2.68905f, 3.26826f,
    3.95294f
};

float get_j60_dco_lfo_semitones(float slider) {
    if (slider <= 0.0f) return 0.0f;
    if (slider >= 1.0f) return 3.95294f;
    float s = slider * 64.0f;
    int32_t idx = (int32_t)s;
    if (idx > 63) idx = 63;
    float frac = s - (float)idx;
    return J60_DCO_LFO_SEMITONES[idx] + frac * (J60_DCO_LFO_SEMITONES[idx + 1] - J60_DCO_LFO_SEMITONES[idx]);
}

float get_j60_attack_tau(float slider) {
    float s = slider < 0.0f ? 0.0f : (slider > 1.0f ? 1.0f : slider);
    s *= 10.0f;
    int32_t idx = (int32_t)s;
    if (idx > 9) idx = 9;
    float frac = s - (float)idx;
    /* Log-interpolated attack time constant */
    float log_tau0 = fast_log(J60_ATTACK_TAU[idx]);
    float log_tau1 = fast_log(J60_ATTACK_TAU[idx + 1]);
    float l = log_tau0 + frac * (log_tau1 - log_tau0);
    return fast_exp(l);
}

float get_j60_dec_rel_tau(float slider) {
    float s = slider < 0.0f ? 0.0f : (slider > 1.0f ? 1.0f : slider);
    float a = 12.9460f * s - 5.0638f * s * s;
    return 0.003577f * fast_exp(a);
}

/* One-pole envelope coefficient, shared by ATTACK / DECAY / RELEASE.
 * The division by tau*fs_block stays: tau is not a constant, so there is nothing
 * to precompute. Only the transcendental is substituted. */
static inline float env_alpha(float tau, float fs_block) {
    float a = -1.0f / (tau * fs_block);
    return 1.0f - fast_exp(a);
}

/* The BLEP residual is evaluated in exactly one place per oscillator, with
 * the saw residual shared, so there is no per-call-site dispatch. poly_blep
 * is static inline in polyblep.h and is called with a cached 1/dt, avoiding
 * the per-edge division. */

void voice_init(JunoVoiceCard* v, int32_t voice_index, const VoiceTolerance* tol, float sample_rate) {
    v->voice_index = voice_index;
    v->tolerance = *tol;
    v->sample_rate = sample_rate;
    v->inv_sample_rate = 1.0f / sample_rate;
    // Distribute initial phase to avoid artificial transient summing
    v->phase = ((float)voice_index / 6.0f);
    while (v->phase >= 1.0f) v->phase -= 1.0f;

    vcf_init(&v->vcf, sample_rate);
    jitter_init(&v->jitter_gen, 10007u + (uint32_t)voice_index * 9973u);
    droop_init(&v->saw_droop, sample_rate);
    v->noise_seed = 54321u + (uint32_t)voice_index * 12345u;
    v->noise_b0 = 0.0f;
    v->noise_b1 = 0.0f;
    v->noise_b2 = 0.0f;

    // Post-DCO (0.16 Hz) and Post-VCF (1.59 Hz) DC blocking filters.
    // post_osc_dc_a and post_vcf_dc_a hold the precomputed g / (1 + g) coefficients.
    v->post_osc_dc_g = tanf(((float)M_PI * 0.16f) / sample_rate);
    v->post_vcf_dc_g = tanf(((float)M_PI * 1.59f) / sample_rate);
    v->post_osc_dc_a = v->post_osc_dc_g / (1.0f + v->post_osc_dc_g);
    v->post_vcf_dc_a = v->post_vcf_dc_g / (1.0f + v->post_vcf_dc_g);
    v->post_osc_dc_s = 0.0f;
    v->post_vcf_dc_s = 0.0f;
    v->jitter_factor = 1.0f;

    v->current_pwm = 0.5f;
    v->env_stage = ENV_STAGE_IDLE;
    v->env_level = 0.0f;
    v->current_env_level = 0.0f;
    v->env_level_step = 0.0f;
    v->gate_gain = 0.0f;
    v->env_target = 0.0f;
    // Matches the factory-default dco_sub_level; refreshed every block by
    // voice_update_control_block().
    v->sub_taper = (expf(3.0f * 0.65f) - 1.0f) / 19.08553692f;

    v->note = -1;
    v->note_freq = 440.0f;
    v->active = false;
    v->age = 0;
    v->key_held = false;

    v->current_vibrato_ratio = 1.0f;
    v->lfo_delay_gain = 1.0f;

    v->current_cutoff_hz = 1000.0f;
    v->current_jitter_cents = 0.0f;
    v->last_output_level = 0.0f;
}

void voice_note_on(JunoVoiceCard* v, int32_t note, float velocity, float detune_cents) {
    (void)velocity;
    v->note = note;
    float base_freq = 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
    v->note_freq = base_freq * powf(2.0f, detune_cents / 1200.0f);
    v->active = true;
    v->key_held = true;
    v->age = 0;
    v->lfo_delay_gain = 0.0f;
    v->current_vibrato_ratio = 1.0f;

    v->env_stage = ENV_STAGE_ATTACK;
    v->env_target = 1.0f;
    v->env_level = v->current_env_level; // Seamless legato pickup
    v->env_level_step = 0.0f;

    // Prime filter state slightly to avoid dead zone on high resonance self-oscillation
    if (fabsf(v->vcf.s0) < 1e-4f) {
        v->vcf.s0 = 0.005f;
    }
}

void voice_note_off(JunoVoiceCard* v) {
    v->key_held = false;
    if (v->active && v->env_stage != ENV_STAGE_IDLE) {
        v->env_stage = ENV_STAGE_RELEASE;
        v->env_target = 0.0f;
    }
}

void voice_silence(JunoVoiceCard* v) {
    v->active = false;
    v->key_held = false;
    v->env_stage = ENV_STAGE_IDLE;
    v->env_level = 0.0f;
    v->current_env_level = 0.0f;
    v->env_level_step = 0.0f;
    v->gate_gain = 0.0f;
    v->note = -1;
    v->age = 0;
    v->lfo_delay_gain = 0.0f;
    v->current_vibrato_ratio = 1.0f;
    v->phase = ((float)v->voice_index / 6.0f);
    v->sub_step = 0;
    v->smoothed_jitter = 0.0f;
    v->jitter_factor = 1.0f;
    v->current_jitter_cents = 0.0f;
    v->post_osc_dc_s = 0.0f;
    v->post_vcf_dc_s = 0.0f;
    v->vcf.s0 = v->vcf.s1 = v->vcf.s2 = v->vcf.s3 = 0.0f;
    v->saw_droop.x1 = v->saw_droop.y1 = 0.0f;
}

void voice_update_control_block(
    JunoVoiceCard* v,
    const JunoEngineParams* params,
    float global_lfo,
    float pitch_bend_ratio,
    float power_sag_deflection
) {
    (void)pitch_bend_ratio;

    // Analog master-clock phase jitter, stepped once per block
    float raw_jitter = jitter_step(&v->jitter_gen);
    v->smoothed_jitter += 0.0004f * (raw_jitter - v->smoothed_jitter);
    v->jitter_factor = 1.0f + v->smoothed_jitter * 0.0004f;
    v->current_jitter_cents = v->smoothed_jitter * 0.69f;

    // Sub-oscillator analog log level taper precomputed at block rate
    {
        float sub3 = fast_exp(3.0f * params->dco_sub_level);
        v->sub_taper = (sub3 - 1.0f) / 19.08553692f;
    }

    if (!v->active && v->env_stage == ENV_STAGE_IDLE) {
        v->env_level = 0.0f;
        v->current_env_level = 0.0f;
        v->env_level_step = 0.0f;
        return;
    }

    v->age += JUNO_BLOCK_SIZE;

    // 1. ADSR Analog RC Envelope Progress
    float fs_block = v->sample_rate / 32.0f;
    switch (v->env_stage) {
        case ENV_STAGE_ATTACK: {
            float tau = get_j60_attack_tau(params->env_attack) * v->tolerance.attack_scale;
            float alpha = env_alpha(tau, fs_block);
            v->env_level += alpha * (1.2f - v->env_level); // Target 1.2V for fast logarithmic curvature
            if (v->env_level >= 1.0f) {
                v->env_level = 1.0f;
                v->env_stage = ENV_STAGE_DECAY;
            }
            break;
        }
        case ENV_STAGE_DECAY: {
            float tau = get_j60_dec_rel_tau(params->env_decay) * v->tolerance.decay_scale;
            float alpha = env_alpha(tau, fs_block);
            float sus = params->env_sustain < 0.0f ? 0.0f : (params->env_sustain > 1.0f ? 1.0f : params->env_sustain);
            v->env_level += alpha * (sus - v->env_level);
            if (v->env_level <= sus + 0.001f) {
                v->env_level = sus;
                v->env_stage = ENV_STAGE_SUSTAIN;
            }
            break;
        }
        case ENV_STAGE_SUSTAIN: {
            float sus = params->env_sustain < 0.0f ? 0.0f : (params->env_sustain > 1.0f ? 1.0f : params->env_sustain);
            v->env_level = sus;
            break;
        }
        case ENV_STAGE_RELEASE: {
            float tau = get_j60_dec_rel_tau(params->env_release) * v->tolerance.decay_scale;
            float alpha = env_alpha(tau, fs_block);
            v->env_level += alpha * (-0.05f - v->env_level); // Asymptotic drop below 0V
            if (v->env_level <= 0.0005f) {
                v->env_level = 0.0f;
                v->env_stage = ENV_STAGE_IDLE;
                v->active = false;
            }
            break;
        }
        default:
            v->env_level = 0.0f;
            v->active = false;
            break;
    }

    v->env_level_step = (v->env_level - v->current_env_level) / 32.0f;

    // 2. LFO Delay & Vibrato Modulation
    float lfo_delay_gain = 1.0f;
    if (params->lfo_delay > 0.001f) {
        float note_time_sec = (float)v->age / v->sample_rate;
        float delay_sec = params->lfo_delay;
        float ramp = note_time_sec / delay_sec;
        if (ramp < 0.0f) ramp = 0.0f;
        if (ramp > 1.0f) ramp = 1.0f;
        lfo_delay_gain = ramp * ramp * (3.0f - 2.0f * ramp); // Hermite S-curve
    }
    v->lfo_delay_gain = lfo_delay_gain;

    float lfo_max_st = get_j60_dco_lfo_semitones(params->dco_lfo_mod);
    float lfo_st = global_lfo * v->lfo_delay_gain * (lfo_max_st * 0.5f);
    float mod_wheel_st = params->mod_wheel * 2.0f * global_lfo;
    float total_vibrato_st = lfo_st + mod_wheel_st;
    v->current_vibrato_ratio = fast_pow2(total_vibrato_st / 12.0f);

    // 3. Cutoff Frequency Calculation
    float raw_cutoff = params->vcf_cutoff;
    float base_freq = 28.0f * fast_exp(7.262f * raw_cutoff); // Exponential V/Oct slider map

    float env_val = v->env_level * (params->vcf_env_polarity == 1 ? 1.0f : -1.0f);
    float env_octaves = env_val * params->vcf_env_mod * 10.0f;
    float lfo_mod_pow = fast_pow(params->vcf_lfo_mod, 1.2f);
    float lfo_octaves = global_lfo * v->lfo_delay_gain * lfo_mod_pow * 3.0f;
    float kbd_octaves = params->vcf_kbd_track * (((float)v->note - 60.0f + params->master_tune) / 12.0f);

    float tolerance_mult = 1.0f + v->tolerance.cutoff_drift;
    float sag_depression = 1.0f - power_sag_deflection * 0.015f;

    float octaves = env_octaves + lfo_octaves + kbd_octaves;
    float oct_pow = fast_pow2(octaves);
    float target_cutoff = base_freq * oct_pow * tolerance_mult * sag_depression;
    if (target_cutoff > 20000.0f) {
        target_cutoff = 20000.0f + 20000.0f * tanhf((target_cutoff - 20000.0f) / 20000.0f);
    }
    if (target_cutoff < 15.0f) target_cutoff = 15.0f;
    if (target_cutoff > 38000.0f) target_cutoff = 38000.0f;
    v->current_cutoff_hz = target_cutoff;

    float res_effective = params->vcf_res * (1.0f + v->tolerance.res_drift);
    if (res_effective < 0.0f) res_effective = 0.0f;
    if (res_effective > 0.999f) res_effective = 0.999f;

    vcf_update_coefficients(&v->vcf, target_cutoff, res_effective);
}

float voice_process_sample(
    JunoVoiceCard* v,
    const JunoEngineParams* params,
    float global_lfo,
    float pitch_bend_ratio,
    float pwm_val
) {
    (void)global_lfo;
    /* No liveness check here. This used to early-out on an idle card, but the
     * guard was re-tested on every sample of every block: four field loads and
     * a branch per voice per sample, on the innermost loop, to protect a
     * condition that cannot change within a sub-block. juno_engine_process_block()
     * now applies the same predicate once per block and skips the card's 32
     * iterations outright, which is strictly better than paying for the test
     * each time. The Juno engine is this function's only consumer, so there is
     * no untrusted caller left to validate. */

    // Interpolate envelope voltage across the 32-sample sub-block
    v->current_env_level += v->env_level_step;
    if (v->env_level_step > 0.0f && v->current_env_level > v->env_level) {
        v->current_env_level = v->env_level;
    } else if (v->env_level_step < 0.0f && v->current_env_level < v->env_level) {
        v->current_env_level = v->env_level;
    }

    // 1. Master Phase Jitter
    /* Jitter was already stepped for this block in voice_update_control_block(),
     * so the held factor is reused and no per-sample work happens here. */
    float jitter_factor = v->jitter_factor;

    float eff_freq = v->note_freq * pitch_bend_ratio * v->current_vibrato_ratio * jitter_factor;
    /* sample_rate is constant for the life of the voice, so the reciprocal is
     * folded at init. Not bit-exact: 1/48000 is not representable, so this is a
     * sub-ulp difference in dt, far below the noise floor of the oscillator. */
    float dt = eff_freq * v->inv_sample_rate;
    if (dt < 1e-7f) dt = 1e-7f;
    if (dt > 0.49f) dt = 0.49f;

    // 2. Master Phase Accumulator
    v->phase += dt;
    if (v->phase >= 1.0f) {
        v->phase -= 1.0f;
        if (params->dco_sub_octave == -1) {
            v->sub_step = (v->sub_step + 1) & 1;
        } else {
            v->sub_step = (v->sub_step + 1) & 3;
        }
    }

    float t = v->phase;

    /* Precompute inv_dt once per sample for all BLEP calls.
     * This replaces multiple divides with a single divide + multiplications.
     * Using 2nd-order poly_blep for all edges; anti-aliasing is slightly
     * degraded (no 2-sample tail) but faster on the critical path. */
    float d = dt < 0.49f ? dt : 0.49f;
    float inv_dt = 1.0f / d;

    bool any_dco = params->dco_saw || params->dco_pulse || params->dco_sub;
    float blep0 = 0.0f;
    if (any_dco) {
        blep0 = poly_blep(t, dt, inv_dt);
    }

    // 3. Sawtooth Wave Generation
    float saw_sample = 0.0f;
    if (params->dco_saw) {
        float raw_saw = 2.0f * t - 1.0f;
        raw_saw -= blep0;
        saw_sample = droop_process(&v->saw_droop, raw_saw) * 0.5f;
    }

    // 4. Pulse / PWM Wave Generation
    float pulse_sample = 0.0f;
    if (params->dco_pulse) {
        v->current_pwm += 0.08f * (pwm_val - v->current_pwm);
        float duty = v->current_pwm;
        if (duty < 0.01f) duty = 0.01f;
        if (duty > 0.99f) duty = 0.99f;

        float raw_pulse = (t < duty) ? -1.0f : 1.0f;
        raw_pulse -= blep0;

        float t_duty = t - duty;
        if (t_duty < 0.0f) t_duty += 1.0f;
        raw_pulse += poly_blep(t_duty, dt, inv_dt);

        pulse_sample = raw_pulse * 0.5f;
    }

    // 5. Sub-Oscillator Wave Generation
    float sub_sample = 0.0f;
    if (params->dco_sub) {
        /* Original nested branches */
        float raw_sub = 0.0f;
        if (params->dco_sub_octave == -1) {
            if (v->sub_step == 0) {
                raw_sub = 1.0f;
                if (t < dt) raw_sub += blep0;
                else if (t > 1.0f - dt) raw_sub -= blep0;
            } else {
                raw_sub = -1.0f;
                if (t < dt) raw_sub -= blep0;
                else if (t > 1.0f - dt) raw_sub += blep0;
            }
        } else {
            if (v->sub_step == 0) {
                raw_sub = 1.0f;
                if (t < dt) raw_sub += blep0;
            } else if (v->sub_step == 1) {
                raw_sub = 1.0f;
                if (t > 1.0f - dt) raw_sub -= blep0;
            } else if (v->sub_step == 2) {
                raw_sub = -1.0f;
                if (t > 1.0f - dt) raw_sub -= blep0;
            } else {
                raw_sub = -1.0f;
                if (t < dt) raw_sub += blep0;
            }
        }
        // Sub-oscillator analog log level taper (precomputed at block rate)
        sub_sample = raw_sub * (0.5942f * v->sub_taper);
    }

    // 6. Analog Pink Noise Generator (3-pole pinking filter)
    float noise_sample = 0.0f;
    if (params->dco_noise > 0.001f) {
        v->noise_seed = (1664525u * v->noise_seed + 1013904223u);
        /* Fast PRNG conversion to [-1.0, 1.0) float */
        union { uint32_t i; float f; } w;
        w.i = (v->noise_seed >> 8) | 0x3F800000u;
        float white = (w.f - 1.5f) * 2.0f;
        v->noise_b0 = 0.99765f * v->noise_b0 + white * 0.099046f;
        v->noise_b1 = 0.96300f * v->noise_b1 + white * 0.2965164f;
        v->noise_b2 = 0.57000f * v->noise_b2 + white * 1.0526913f;
        float pink = (v->noise_b0 + v->noise_b1 + v->noise_b2 + white * 0.1848f) * 0.22f;
        noise_sample = pink * params->dco_noise;
    }

    // 7. Oscillator Mixer & Post-DCO DC Blocker (0.16 Hz)
    float raw_dco_mix = saw_sample + pulse_sample + sub_sample + noise_sample;
    float dc_v;
    dc_v = (raw_dco_mix - v->post_osc_dc_s) * v->post_osc_dc_a;
    float dc_lp = v->post_osc_dc_s + dc_v;
    v->post_osc_dc_s = dc_lp + dc_v;
    float dco_mix = raw_dco_mix - dc_lp;

    // 8. Roland IR3109 4-Pole OTA Ladder Filter & Post-VCF DC Blocker (1.59 Hz)
    float raw_vcf_out = vcf_process(&v->vcf, dco_mix);
    float vcf_dc_v;
    vcf_dc_v = (raw_vcf_out - v->post_vcf_dc_s) * v->post_vcf_dc_a;
    float vcf_dc_lp = v->post_vcf_dc_s + vcf_dc_v;
    v->post_vcf_dc_s = vcf_dc_lp + vcf_dc_v;
    float vcf_out = raw_vcf_out - vcf_dc_lp;

    // 9. VCA (Voltage Controlled Amplifier)
    float vca_gain = 0.0f;
    if (params->vca_mode == JUNO_VCA_ENV) {
        float env = v->current_env_level;
        if (env < 0.0f) env = 0.0f;
        if (env > 1.0f) env = 1.0f;
        // Piecewise exponential VCA curve matching Roland BA662 / IR3109 VCA response
        vca_gain = (env < 0.25f) ? (env * env * 2.0f) : (0.125f + (env - 0.25f) * 1.1667f);
    } else {
        // Gate mode: 0.08 rate smoothing simulating analog RC gate filtering
        float target_gate = v->key_held ? 1.0f : 0.0f;
        v->gate_gain += 0.08f * (target_gate - v->gate_gain);
        vca_gain = v->gate_gain;
        if (!v->key_held && v->gate_gain < 0.0005f) {
            v->gate_gain = 0.0f;
            v->active = false;
            v->env_stage = ENV_STAGE_IDLE;
        }
    }

    float vca_out = (vcf_out + v->tolerance.vca_bleed) * vca_gain;
    v->last_output_level = fabsf(vca_out);
    return vca_out;
}
