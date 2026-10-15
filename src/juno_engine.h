/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file juno_engine.h
 * @brief MCUno-32 6-Voice Polyphonic Synthesizer Engine
 *
 * Implements:
 * - 6 Independent Voice Cards with static analog tolerance profiles
 * - Poly 1, Poly 2, and Unison voice allocation with LRU voice stealing and release priority
 * - Master 4-position HPF (Position 0 Flat bypass, Pos 1 122Hz, Pos 2 269Hz, Pos 3 571Hz)
 * - Matsushita MN3009 BBD Stereo Chorus with compander and Hermite fractional delay
 * - Dynamic Power Rail "Sag" tracking loop (PSU deflection under multi-voice load)
 * - Arpeggiator (Up, Down, Up/Down, 1-3 octaves) with insertion sort note tracking
 * - DMA double-buffered 32-sample sub-block rendering
 */

#ifndef JUNO_ENGINE_H
#define JUNO_ENGINE_H

#include "chorus_mn3009.h"
#include "ir3109_vcf.h"
#include "master_hpf.h"
#include "polyblep.h"
#include "voice.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JUNO_NUM_VOICES 6

typedef enum {
    JUNO_POLY_1 = 0,
    JUNO_POLY_2 = 1,
    JUNO_POLY_UNISON = 2
} JunoPolyMode;

typedef enum {
    JUNO_ARP_UP = 0,
    JUNO_ARP_DOWN = 1,
    JUNO_ARP_UP_DOWN = 2
} JunoArpMode;

// Parameter IDs
enum {
    JUNO_PARAM_LFO_RATE = 0,
    JUNO_PARAM_LFO_DELAY = 1,
    JUNO_PARAM_DCO_LFO = 2,
    JUNO_PARAM_DCO_PWM = 3,
    JUNO_PARAM_DCO_PWM_MODE = 4,
    JUNO_PARAM_DCO_SAW = 5,
    JUNO_PARAM_DCO_PULSE = 6,
    JUNO_PARAM_DCO_SUB = 7,
    JUNO_PARAM_DCO_SUB_OCT = 8,
    JUNO_PARAM_DCO_SUB_LEVEL = 9,
    JUNO_PARAM_DCO_NOISE = 10,
    JUNO_PARAM_HPF_POS = 11,
    JUNO_PARAM_VCF_CUTOFF = 12,
    JUNO_PARAM_VCF_RES = 13,
    JUNO_PARAM_VCF_ENV_MOD = 14,
    JUNO_PARAM_VCF_ENV_POL = 15,
    JUNO_PARAM_VCF_LFO_MOD = 16,
    JUNO_PARAM_VCF_KBD = 17,
    JUNO_PARAM_VCA_MODE = 18,
    JUNO_PARAM_VCA_LEVEL = 19,
    JUNO_PARAM_ENV_ATTACK = 20,
    JUNO_PARAM_ENV_DECAY = 21,
    JUNO_PARAM_ENV_SUSTAIN = 22,
    JUNO_PARAM_ENV_RELEASE = 23,
    JUNO_PARAM_CHORUS_MODE = 24,
    JUNO_PARAM_MASTER_VOL = 25,
    JUNO_PARAM_MASTER_TUNE = 26,
    JUNO_PARAM_BENDER = 27,
    JUNO_PARAM_MOD_WHEEL = 28,
    JUNO_PARAM_POLY_MODE = 29,
    JUNO_PARAM_HOLD = 30,
    JUNO_PARAM_ARP_ON = 31,
    JUNO_PARAM_ARP_RATE = 32,
    JUNO_PARAM_ARP_RANGE = 33,
    JUNO_PARAM_ARP_MODE = 34,
    JUNO_PARAM_MODEL = 35
};

/**
 * @brief Master Synthesizer Engine State
 */
typedef struct {
    JunoEngineParams params; /**< Current synth parameter settings */
    float sample_rate;        /**< Audio sample rate in Hz */

    JunoVoiceCard voices[JUNO_NUM_VOICES]; /**< 6 discrete voice cards */

    JunoMasterHpf master_hpf;   /**< Master high-pass filter */
    JunoMN3009Chorus chorus;   /**< Stereo BBD chorus */

    float global_lfo_phase;      /**< Global LFO phase accumulator */
    float global_lfo_value;      /**< Global LFO instantaneous amplitude */

    // Power Rail Sag Emulation
    float power_sag_detector;    /**< Power supply rectifier current draw detector */
    float power_sag_deflection;  /**< Instantaneous V-Rail sag deflection [0.0, 0.04] */
    float v_rail;               /**< Effective power supply rail voltage factor [0.96, 1.0] */

    // Voice Allocation State
    int32_t round_robin_idx;     /**< Next round-robin voice candidate */
    int32_t held_notes[16];     /**< Currently depressed MIDI keys buffer */
    int32_t held_notes_count;    /**< Number of active keys in held_notes buffer */

    // Arpeggiator
    float arp_phase;            /**< Arpeggiator clock phase */
    int32_t arp_step_idx;        /**< Step counter within the current pattern */
    int32_t arp_current_note;    /**< Currently sounding arpeggiator note */
    int32_t arp_sorted_buffer[16];  /**< Note pitch sort scratch buffer */
    int32_t arp_pattern_buffer[64]; /**< Expanded arpeggio pattern buffer (up to 3 octaves) */

    // DMA Block Counter & VCA Trim
    uint32_t dma_block_counter;  /**< Total 32-sample DMA blocks rendered */
    float vca_trim_linear;       /**< Linear master voice VCA slider trim factor */
} JunoEngine;

/**
 * @brief Initialize master engine, voice cards, HPF, and chorus
 * @param engine Pointer to JunoEngine struct
 * @param sample_rate Operating audio sample rate in Hz
 */
void juno_engine_init(JunoEngine* engine, float sample_rate);

/**
 * @brief Trigger a Note On event across the polyphonic allocation pool
 */
void juno_engine_note_on(JunoEngine* engine, int32_t note, float velocity);

/**
 * @brief Trigger a Note Off event for a released MIDI key
 */
void juno_engine_note_off(JunoEngine* engine, int32_t note);

/**
 * @brief Silence all currently sounding voices and reset arpeggiator
 */
void juno_engine_all_notes_off(JunoEngine* engine);

/**
 * @brief Update a synthesis parameter by parameter ID
 */
void juno_engine_set_param(JunoEngine* engine, int32_t param_id, float value);

/**
 * @brief Render a block of stereo audio samples (typically 32 samples per DMA block)
 * @param engine Pointer to JunoEngine struct
 * @param out_l Left channel output buffer
 * @param out_r Right channel output buffer
 * @param block_size Number of samples to render (<= 32)
 */
void juno_engine_process_block(JunoEngine* engine, float* out_l, float* out_r, int32_t block_size);

#ifdef __cplusplus
}
#endif

#endif // JUNO_ENGINE_H
