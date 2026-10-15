/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file juno_engine.c
 * @brief MCUno-32 6-Voice Polyphonic Synthesizer Engine Implementation
 *
 * Implements:
 * - 6 Independent Voice Cards with static analog tolerance profiles
 * - Block-rate processing (DMA double-buffered block size = 32 samples)
 * - Dynamic Power Rail "Sag" tracking loop (PSU deflection under multi-voice load)
 * - Poly 1, Poly 2, and Unison voice allocation with LRU voice stealing and release priority
 * - Master 4-position HPF (Position 0 Flat bypass, Pos 1 122Hz, Pos 2 269Hz, Pos 3 571Hz)
 * - Matsushita MN3009 BBD Stereo Chorus with compander and Hermite fractional delay
 * - Arpeggiator (Up, Down, Up/Down, 1-3 octaves) with insertion sort note tracking
 */

#include "juno_engine.h"
#include "polyblep.h"

// Static tolerance offset table assigning distinct analog variance to each of the 6 voices
static const VoiceTolerance VOICE_TOLERANCE_TABLE[JUNO_NUM_VOICES] = {
    { +0.011f, -0.021f, 1.025f, 0.980f, +0.0018f },
    { -0.008f, +0.015f, 0.972f, 1.031f, -0.0012f },
    { +0.014f, +0.028f, 1.038f, 1.022f, +0.0024f },
    { -0.013f, -0.018f, 0.962f, 0.970f, -0.0021f },
    { +0.005f, -0.029f, 1.012f, 1.040f, +0.0011f },
    { -0.012f, +0.022f, 0.981f, 0.964f, -0.0015f },
};

// Subtle analog micro-detune (cents) spread across the 6 voices in UNISON mode
// Matches authentic Juno-60 voice-card component tolerances. Because frequencies are micro-detuned,
// the continuously running DCO phases naturally drift and beat, producing a lush, thick ensemble
// without synthetic phase resets or abrupt discontinuities.
static const float UNISON_DETUNE_CENTS[JUNO_NUM_VOICES] = {
    -3.8f, +2.4f, -1.9f, +4.2f, -4.5f, +1.8f
};

void juno_engine_init(JunoEngine* engine, float sample_rate) {
    engine->sample_rate = sample_rate;

    // Default synthesis parameters matching Roland Juno-60 Factory Default
    JunoEngineParams* p = &engine->params;
    p->lfo_rate = 2.5f;
    p->lfo_delay = 0.0f;
    p->dco_lfo_mod = 0.0f;
    p->dco_pwm = 0.5f;
    p->dco_pwm_mode = JUNO_PWM_MAN;
    p->dco_saw = true;
    p->dco_pulse = false;
    p->dco_sub = true;
    p->dco_sub_octave = -1;
    p->dco_sub_level = 0.65f;
    p->dco_noise = 0.0f;
    p->hpf_position = 0; // Position 0: Flat bypass (unity gain, no bass boost)
    p->vcf_cutoff = 0.42f;
    p->vcf_res = 0.28f;
    p->vcf_env_mod = 0.65f;
    p->vcf_env_polarity = 1;
    p->vcf_lfo_mod = 0.0f;
    p->vcf_kbd_track = 0.4f;
    p->vca_mode = JUNO_VCA_ENV;
    p->vca_level = 0.85f;
    p->env_attack = 0.08f;
    p->env_decay = 0.45f;
    p->env_sustain = 0.35f;
    p->env_release = 0.4f;
    p->chorus_mode = JUNO_CHORUS_I;
    p->master_volume = 0.8f;
    p->master_tune = 0.0f;
    p->bender = 0.0f;
    p->mod_wheel = 0.0f;
    p->poly_mode = JUNO_POLY_1;
    p->hold = false;
    p->arp_on = false;
    p->arp_rate = 6.0f;
    p->arp_range = 1;
    p->arp_mode = JUNO_ARP_UP;
    p->model = JUNO_MODEL_60;

    // Initialize 6 discrete voice cards
    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        voice_init(&engine->voices[i], i, &VOICE_TOLERANCE_TABLE[i], sample_rate);
    }

    hpf_init(&engine->master_hpf, sample_rate);
    chorus_init(&engine->chorus, sample_rate);
    engine->chorus.mode = p->chorus_mode;

    engine->global_lfo_phase = 0.0f;
    engine->global_lfo_value = 0.0f;
    engine->power_sag_detector = 0.0f;
    engine->power_sag_deflection = 0.0f;
    engine->v_rail = 1.0f;

    engine->round_robin_idx = 0;
    engine->held_notes_count = 0;
    engine->arp_phase = 0.0f;
    engine->arp_step_idx = 0;
    engine->arp_current_note = -1;

    engine->dma_block_counter = 0;
    engine->vca_trim_linear = 1.0f;
}

static void release_latched_hold_notes(JunoEngine* engine) {
    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        if (engine->voices[i].active) {
            bool is_held = false;
            for (int h = 0; h < engine->held_notes_count; h++) {
                if (engine->held_notes[h] == engine->voices[i].note) {
                    is_held = true;
                    break;
                }
            }
            if (!is_held) {
                voice_note_off(&engine->voices[i]);
            }
        }
    }
}

/**
 * Polyphonic voice allocator with LRU stealing and release priority
 */
static void allocate_voice(JunoEngine* engine, int32_t note, float velocity) {
    // 1. UNISON Mode: Assign note to all 6 voices with analog detune table
    if (engine->params.poly_mode == JUNO_POLY_UNISON) {
        for (int i = 0; i < JUNO_NUM_VOICES; i++) {
            voice_note_on(&engine->voices[i], note, velocity, UNISON_DETUNE_CENTS[i]);
        }
        return;
    }

    // 2. POLY 2 Mode: Re-use voice already playing the same note if available
    if (engine->params.poly_mode == JUNO_POLY_2) {
        for (int i = 0; i < JUNO_NUM_VOICES; i++) {
            if (engine->voices[i].note == note && engine->voices[i].active) {
                voice_note_on(&engine->voices[i], note, velocity, 0.0f);
                return;
            }
        }
    }

    // 3. Find an idle voice using round-robin scan
    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        int32_t v_idx = (engine->round_robin_idx + i) % JUNO_NUM_VOICES;
        if (!engine->voices[v_idx].active) {
            engine->round_robin_idx = (v_idx + 1) % JUNO_NUM_VOICES;
            voice_note_on(&engine->voices[v_idx], note, velocity, 0.0f);
            return;
        }
    }

    // 4. Steal voice with release-stage priority:
    // Prefer stealing voices in RELEASE over actively sustained voices
    int32_t steal_voice_idx = -1;
    int32_t max_release_age = -1;
    int32_t oldest_voice_idx = 0;
    int32_t max_age = -1;

    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        JunoVoiceCard* v = &engine->voices[i];
        if (v->age > max_age) {
            max_age = v->age;
            oldest_voice_idx = i;
        }
        bool is_held = false;
        for (int h = 0; h < engine->held_notes_count; h++) {
            if (engine->held_notes[h] == v->note) {
                is_held = true;
                break;
            }
        }
        if (!is_held) {
            if (v->age > max_release_age) {
                max_release_age = v->age;
                steal_voice_idx = i;
            }
        }
    }

    if (steal_voice_idx == -1) {
        steal_voice_idx = oldest_voice_idx;
    }

    voice_note_on(&engine->voices[steal_voice_idx], note, velocity, 0.0f);
    engine->round_robin_idx = (steal_voice_idx + 1) % JUNO_NUM_VOICES;
}

void juno_engine_note_on(JunoEngine* engine, int32_t note, float velocity) {
    bool already_held = false;
    for (int i = 0; i < engine->held_notes_count; i++) {
        if (engine->held_notes[i] == note) {
            already_held = true;
            break;
        }
    }
    if (!already_held && engine->held_notes_count < 16) {
        engine->held_notes[engine->held_notes_count++] = note;
    }

    if (engine->params.arp_on) return;
    allocate_voice(engine, note, velocity);
}

void juno_engine_note_off(JunoEngine* engine, int32_t note) {
    for (int i = 0; i < engine->held_notes_count; i++) {
        if (engine->held_notes[i] == note) {
            for (int j = i; j < engine->held_notes_count - 1; j++) {
                engine->held_notes[j] = engine->held_notes[j + 1];
            }
            engine->held_notes_count--;
            break;
        }
    }

    if (engine->params.hold) return;

    if (engine->params.arp_on) {
        if (engine->held_notes_count == 0 && engine->arp_current_note != -1) {
            for (int i = 0; i < JUNO_NUM_VOICES; i++) {
                if (engine->voices[i].note == engine->arp_current_note && engine->voices[i].active) {
                    voice_note_off(&engine->voices[i]);
                }
            }
            engine->arp_current_note = -1;
        }
        return;
    }

    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        if (engine->voices[i].note == note && engine->voices[i].active) {
            voice_note_off(&engine->voices[i]);
        }
    }
}

void juno_engine_all_notes_off(JunoEngine* engine) {
    engine->held_notes_count = 0;
    engine->arp_current_note = -1;
    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        voice_silence(&engine->voices[i]);
    }
}

void juno_engine_set_param(JunoEngine* engine, int32_t param_id, float value) {
    JunoEngineParams* p = &engine->params;
    bool old_hold = p->hold;

    switch (param_id) {
        case JUNO_PARAM_LFO_RATE: p->lfo_rate = value; break;
        case JUNO_PARAM_LFO_DELAY: p->lfo_delay = value; break;
        case JUNO_PARAM_DCO_LFO: p->dco_lfo_mod = value; break;
        case JUNO_PARAM_DCO_PWM: p->dco_pwm = value; break;
        case JUNO_PARAM_DCO_PWM_MODE: p->dco_pwm_mode = (int32_t)value; break;
        case JUNO_PARAM_DCO_SAW: p->dco_saw = (value >= 0.5f); break;
        case JUNO_PARAM_DCO_PULSE: p->dco_pulse = (value >= 0.5f); break;
        case JUNO_PARAM_DCO_SUB: p->dco_sub = (value >= 0.5f); break;
        case JUNO_PARAM_DCO_SUB_OCT: p->dco_sub_octave = (int32_t)value; break;
        case JUNO_PARAM_DCO_SUB_LEVEL: p->dco_sub_level = value; break;
        case JUNO_PARAM_DCO_NOISE: p->dco_noise = value; break;
        case JUNO_PARAM_HPF_POS:
            p->hpf_position = (int32_t)value;
            hpf_compute_coefficients(&engine->master_hpf, p->hpf_position);
            break;
        case JUNO_PARAM_VCF_CUTOFF: p->vcf_cutoff = value; break;
        case JUNO_PARAM_VCF_RES: p->vcf_res = value; break;
        case JUNO_PARAM_VCF_ENV_MOD: p->vcf_env_mod = value; break;
        case JUNO_PARAM_VCF_ENV_POL: p->vcf_env_polarity = (value < 0.0f) ? -1 : 1; break;
        case JUNO_PARAM_VCF_LFO_MOD: p->vcf_lfo_mod = value; break;
        case JUNO_PARAM_VCF_KBD: p->vcf_kbd_track = value; break;
        case JUNO_PARAM_VCA_MODE: p->vca_mode = (int32_t)value; break;
        case JUNO_PARAM_VCA_LEVEL: p->vca_level = value; break;
        case JUNO_PARAM_ENV_ATTACK: p->env_attack = value; break;
        case JUNO_PARAM_ENV_DECAY: p->env_decay = value; break;
        case JUNO_PARAM_ENV_SUSTAIN: p->env_sustain = value; break;
        case JUNO_PARAM_ENV_RELEASE: p->env_release = value; break;
        case JUNO_PARAM_CHORUS_MODE:
            p->chorus_mode = (int32_t)value;
            engine->chorus.mode = p->chorus_mode;
            break;
        case JUNO_PARAM_MASTER_VOL: p->master_volume = value; break;
        case JUNO_PARAM_MASTER_TUNE: p->master_tune = value; break;
        case JUNO_PARAM_BENDER: p->bender = value; break;
        case JUNO_PARAM_MOD_WHEEL: p->mod_wheel = value; break;
        case JUNO_PARAM_POLY_MODE: p->poly_mode = (int32_t)value; break;
        case JUNO_PARAM_HOLD:
            p->hold = (value >= 0.5f);
            if (old_hold && !p->hold) release_latched_hold_notes(engine);
            break;
        case JUNO_PARAM_ARP_ON: p->arp_on = (value >= 0.5f); break;
        case JUNO_PARAM_ARP_RATE: p->arp_rate = value; break;
        case JUNO_PARAM_ARP_RANGE: p->arp_range = (int32_t)value; break;
        case JUNO_PARAM_ARP_MODE: p->arp_mode = (int32_t)value; break;
        case JUNO_PARAM_MODEL:
            p->model = (int32_t)value;
            engine->master_hpf.model = p->model;
            hpf_compute_coefficients(&engine->master_hpf, p->hpf_position);
            break;
        default: break;
    }
}

static void update_arpeggiator(JunoEngine* engine) {
    if (!engine->params.arp_on || engine->held_notes_count == 0) return;

    float fs_block = engine->sample_rate / 32.0f;
    float rate = engine->params.arp_rate < 1.0f ? 1.0f : engine->params.arp_rate;
    float step_duration = fs_block / rate;

    engine->arp_phase += 1.0f;
    if (engine->arp_phase >= step_duration) {
        engine->arp_phase = 0.0f;

        int32_t count = engine->held_notes_count < 16 ? engine->held_notes_count : 16;
        for (int i = 0; i < count; i++) {
            engine->arp_sorted_buffer[i] = engine->held_notes[i];
        }

        // Insertion sort held notes in ascending pitch order
        for (int i = 1; i < count; i++) {
            int32_t key = engine->arp_sorted_buffer[i];
            int j = i - 1;
            while (j >= 0 && engine->arp_sorted_buffer[j] > key) {
                engine->arp_sorted_buffer[j + 1] = engine->arp_sorted_buffer[j];
                j--;
            }
            engine->arp_sorted_buffer[j + 1] = key;
        }

        // Expand pattern across 1 to 3 octaves
        int32_t range = engine->params.arp_range < 1 ? 1 : (engine->params.arp_range > 3 ? 3 : engine->params.arp_range);
        int32_t pattern_len = 0;
        for (int oct = 0; oct < range; oct++) {
            int32_t oct_ofs = oct * 12;
            for (int i = 0; i < count; i++) {
                if (pattern_len < 64) {
                    engine->arp_pattern_buffer[pattern_len++] = engine->arp_sorted_buffer[i] + oct_ofs;
                }
            }
        }

        if (pattern_len == 0) return;

        int32_t target_note = engine->arp_pattern_buffer[0];
        if (engine->params.arp_mode == JUNO_ARP_UP) {
            target_note = engine->arp_pattern_buffer[engine->arp_step_idx % pattern_len];
            engine->arp_step_idx = (engine->arp_step_idx + 1) % pattern_len;
        } else if (engine->params.arp_mode == JUNO_ARP_DOWN) {
            int32_t rev_idx = (pattern_len - 1) - (engine->arp_step_idx % pattern_len);
            target_note = engine->arp_pattern_buffer[rev_idx];
            engine->arp_step_idx = (engine->arp_step_idx + 1) % pattern_len;
        } else {
            // Up/Down alternating pattern
            int32_t cycle_len = (pattern_len > 1) ? (pattern_len * 2 - 2) : 1;
            int32_t pos = engine->arp_step_idx % cycle_len;
            if (pos < pattern_len) target_note = engine->arp_pattern_buffer[pos];
            else target_note = engine->arp_pattern_buffer[cycle_len - pos];
            engine->arp_step_idx = (engine->arp_step_idx + 1) % cycle_len;
        }

        if (engine->arp_current_note != -1) {
            for (int i = 0; i < JUNO_NUM_VOICES; i++) {
                if (engine->voices[i].note == engine->arp_current_note) {
                    voice_note_off(&engine->voices[i]);
                }
            }
        }

        engine->arp_current_note = target_note;
        allocate_voice(engine, target_note, 1.0f);
    }
}

/**
 * Control block updates: executed once per 32 audio samples
 */
static void process_control_block(JunoEngine* engine) {
    engine->dma_block_counter++;

    // 1. Advance Master Analog Triangle LFO
    float fs_block = engine->sample_rate / 32.0f;
    float lfo_inc = (2.0f * (float)M_PI * engine->params.lfo_rate) / fs_block;
    engine->global_lfo_phase += lfo_inc;
    if (engine->global_lfo_phase >= 2.0f * (float)M_PI) {
        engine->global_lfo_phase -= 2.0f * (float)M_PI;
    }
    float tri = 2.0f * fabsf((engine->global_lfo_phase / (float)M_PI) - 1.0f) - 1.0f;
    engine->global_lfo_value = tri * (1.5f - 0.5f * tri * tri); // Parabolic curved triangle

    // 2. Arpeggiator Clock
    update_arpeggiator(engine);

    // 3. Pitch bend ratio via fast_pow2
    float semitones = engine->params.bender * 2.0f + engine->params.master_tune;
    float pitch_bend_ratio = fast_pow2(semitones / 12.0f);

    // 4. Update Voice Cards
    for (int i = 0; i < JUNO_NUM_VOICES; i++) {
        voice_update_control_block(
            &engine->voices[i],
            &engine->params,
            engine->global_lfo_value,
            pitch_bend_ratio,
            engine->power_sag_deflection
        );
    }

    // 5. Chorus & HPF Mode Sync
    if (engine->chorus.mode != engine->params.chorus_mode) {
        engine->chorus.mode = engine->params.chorus_mode;
    }
    if (engine->master_hpf.position != engine->params.hpf_position) {
        hpf_compute_coefficients(&engine->master_hpf, engine->params.hpf_position);
    }

    // 6. Master Voice VCA Slider Trim (±10 dB)
    float vca_clamp = engine->params.vca_level;
    if (vca_clamp < 0.0f) vca_clamp = 0.0f;
    if (vca_clamp > 1.0f) vca_clamp = 1.0f;
    float vca_trim_db = -10.0f + 20.0f * vca_clamp;
    engine->vca_trim_linear = powf(10.0f, vca_trim_db / 20.0f);
}

/* Identify active voice cards capable of producing audio in the current block */
static int32_t build_live_list(JunoEngine* engine, JunoVoiceCard** live) {
    int32_t live_count = 0;
    for (int v = 0; v < JUNO_NUM_VOICES; v++) {
        JunoVoiceCard* card = &engine->voices[v];
        if (!card->active && card->env_stage == ENV_STAGE_IDLE &&
            card->current_env_level < 0.0001f && card->gate_gain < 0.0001f) {
            card->current_env_level = 0.0f;
            card->gate_gain = 0.0f;
            continue;
        }
        live[live_count++] = card;
    }
    return live_count;
}

void juno_engine_process_block(JunoEngine* engine, float* out_l, float* out_r, int32_t block_size) {
    process_control_block(engine);

    // Effective PWM duty cycle
    float effective_duty = 0.5f;
    if (engine->params.dco_pwm_mode == JUNO_PWM_MAN) {
        effective_duty = 0.5f - 0.472f * engine->params.dco_pwm;
    } else {
        effective_duty = 0.5f - 0.472f * engine->params.dco_pwm * engine->global_lfo_value;
    }
    if (effective_duty < 0.025f) effective_duty = 0.025f;
    if (effective_duty > 0.975f) effective_duty = 0.975f;

    float semitones = engine->params.bender * 2.0f + engine->params.master_tune;
    float pitch_bend_ratio = fast_pow2(semitones / 12.0f);

    /* Build the list of active voice cards to process for this block */
    if (block_size > JUNO_BLOCK_SIZE) {
        block_size = JUNO_BLOCK_SIZE; /* contract is <= JUNO_BLOCK_SIZE; clamp defensively */
    }

    const JunoEngineParams* params = &engine->params;
    float lfo_val = engine->global_lfo_value;
    float vca_trim = engine->vca_trim_linear;

    // Loop-invariant master coefficients
    float unison_scale = (engine->params.poly_mode == JUNO_POLY_UNISON) ? 0.65f : 1.0f;

    // Master Output Volume: square-law log response (40 dB range at 0.01)
    float master_vol = engine->params.master_volume;
    if (master_vol < 0.0f) master_vol = 0.0f;
    if (master_vol > 1.0f) master_vol = 1.0f;
    float master_gain = master_vol * master_vol;

    JunoVoiceCard* live[JUNO_NUM_VOICES];
    int32_t live_count = build_live_list(engine, live);

    for (int n = 0; n < block_size; n++) {
        float voice_sum = 0.0f;
        for (int v = 0; v < live_count; v++) {
            voice_sum += voice_process_sample(
                live[v],
                params,
                lfo_val,
                pitch_bend_ratio,
                effective_duty
            );
        }

        float bus_signal = voice_sum * unison_scale * vca_trim;

        // Dynamic PSU rail sag detector
        float abs_sum = fabsf(bus_signal);
        if (abs_sum > engine->power_sag_detector) {
            engine->power_sag_detector += 0.035f * (abs_sum - engine->power_sag_detector);
        } else {
            engine->power_sag_detector -= 0.0003f * engine->power_sag_detector;
        }

        engine->power_sag_deflection = engine->power_sag_detector * 0.012f;
        if (engine->power_sag_deflection > 0.04f) engine->power_sag_deflection = 0.04f;
        engine->v_rail = 1.0f - engine->power_sag_deflection;

        // Master High-Pass Filter
        hpf_process(&engine->master_hpf, bus_signal, bus_signal);

        // Matsushita MN3009 BBD Stereo Chorus
        chorus_process(&engine->chorus, engine->master_hpf.out_l, engine->master_hpf.out_r);

        float final_l = engine->chorus.out_l * master_gain;
        float final_r = engine->chorus.out_r * master_gain;

        out_l[n] = final_l;
        out_r[n] = final_r;
    }
}
