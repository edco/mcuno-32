/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file voice.h
 * @brief MCUno-32 Single Voice Card
 *
 * Implements:
 * - Single DCO with phase noise, PolyBLEP saw/pulse/sub, and analog droop quirks
 * - Sub-oscillator f0/2 (-1 oct) and f0/4 (-2 oct) square division with analog log-taper
 * - 4-Pole ZDF IR3109 OTA ladder filter with per-stage local Padé saturation
 * - Analog-modeled ADSR envelope with exponential curves (J60_ATTACK_TAU table)
 * - VCA with ENV non-linear response and GATE analog RC filtering, plus DC bleed
 * - Individual voice card component tolerance offsets (cutoff drift, res drift, attack/decay scale, VCA bleed)
 * - DCO LFO slider response curve to semitones lookup table (J60_DCO_LFO_SEMITONES)
 * - Post-DCO (0.16 Hz) and Post-VCF (1.59 Hz) coupling capacitor DC blockers
 */

#ifndef JUNO_VOICE_H
#define JUNO_VOICE_H

#include "ir3109_vcf.h"
#include "polyblep.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JUNO_BLOCK_SIZE 32

typedef enum {
    JUNO_VCA_ENV = 0,
    JUNO_VCA_GATE = 1
} JunoVcaMode;

typedef enum {
    JUNO_PWM_MAN = 0,
    JUNO_PWM_LFO = 1
} JunoPwmMode;

typedef enum {
    ENV_STAGE_IDLE = 0,
    ENV_STAGE_ATTACK = 1,
    ENV_STAGE_DECAY = 2,
    ENV_STAGE_SUSTAIN = 3,
    ENV_STAGE_RELEASE = 4
} JunoEnvStage;

/**
 * @brief Static analog component tolerance offsets for an individual voice card
 */
typedef struct {
    float cutoff_drift; /**< Cutoff frequency offset (e.g. ±1.5%) */
    float res_drift;    /**< Resonance feedback offset (e.g. ±3.0%) */
    float attack_scale; /**< Attack timing scale (e.g. 0.96 to 1.04) */
    float decay_scale;  /**< Decay/release timing scale (e.g. 0.96 to 1.04) */
    float vca_bleed;    /**< VCA DC bias / leakage voltage in millivolts */
} VoiceTolerance;

/**
 * @brief Synthesizer Engine Parameter Set
 */
typedef struct {
    // LFO
    float lfo_rate;          /**< LFO rate in Hz (0.1 - 30.0) */
    float lfo_delay;         /**< LFO delay in seconds (0.0 - 3.0) */

    // DCO
    float dco_lfo_mod;        /**< DCO pitch vibrato mod depth (0.0 - 1.0) */
    float dco_pwm;           /**< Manual pulse width or LFO PWM depth (0.0 - 1.0) */
    int32_t dco_pwm_mode;     /**< 0: MAN, 1: LFO */
    bool dco_saw;            /**< Sawtooth wave enabled */
    bool dco_pulse;          /**< Pulse wave enabled */
    bool dco_sub;            /**< Sub-oscillator enabled */
    int32_t dco_sub_octave;   /**< -1 (f0/2) or -2 (f0/4) */
    float dco_sub_level;      /**< Sub-oscillator level (0.0 - 1.0) */
    float dco_noise;         /**< Noise generator level (0.0 - 1.0) */

    // HPF
    int32_t hpf_position;    /**< 0, 1, 2, or 3 */

    // VCF (IR3109)
    float vcf_cutoff;        /**< Filter cutoff slider (0.0 - 1.0) */
    float vcf_res;           /**< Filter resonance slider (0.0 - 1.0) */
    float vcf_env_mod;        /**< Envelope mod depth (0.0 - 1.0) */
    int32_t vcf_env_polarity; /**< +1 or -1 */
    float vcf_lfo_mod;        /**< Filter LFO mod depth (0.0 - 1.0) */
    float vcf_kbd_track;      /**< Keyboard tracking amount (0.0 - 1.0) */

    // VCA
    int32_t vca_mode;        /**< 0: ENV, 1: GATE */
    float vca_level;         /**< Master voice VCA level (0.0 - 1.0) */

    // ENV (ADSR)
    float env_attack;        /**< Attack time (0.0 - 1.0) */
    float env_decay;         /**< Decay time (0.0 - 1.0) */
    float env_sustain;       /**< Sustain level (0.0 - 1.0) */
    float env_release;       /**< Release time (0.0 - 1.0) */

    // Chorus
    int32_t chorus_mode;     /**< 0: Off, 1: I, 2: II, 3: I+II */

    // Master / Performance
    float master_volume;     /**< Master volume output gain (0.0 - 1.0) */
    float master_tune;       /**< Master fine tuning in semitones (-1.0 to +1.0) */
    float bender;           /**< Pitch bender lever (-1.0 to +1.0) */
    float mod_wheel;         /**< Modulation wheel (0.0 to 1.0) */
    int32_t poly_mode;       /**< 0: POLY1, 1: POLY2, 2: UNISON */
    bool hold;              /**< Chord hold latch */

    // Arpeggiator
    bool arp_on;             /**< Arpeggiator enabled */
    float arp_rate;          /**< Arpeggiator rate in Hz (1.0 - 25.0) */
    int32_t arp_range;       /**< 1, 2, or 3 octaves */
    int32_t arp_mode;        /**< 0: UP, 1: DOWN, 2: UP_DOWN */

    // Model Architecture
    int32_t model;          /**< 0: Juno-60, 1: Juno-106 */
} JunoEngineParams;

/**
 * @brief Discrete Juno Voice Card State
 */
typedef struct {
    int32_t voice_index;             /**< Voice card index (0 to 5) */
    VoiceTolerance tolerance;       /**< Hardware tolerance offsets */

    // DCO States
    float phase;                    /**< Normalized master oscillator phase [0.0, 1.0) */
    int32_t sub_step;                /**< Sub-oscillator flip-flop counter (0..1 or 0..3) */
    float smoothed_jitter;           /**< Low-pass filtered master clock phase drift */
    uint32_t noise_seed;             /**< White noise 32-bit generator seed */
    float noise_b0, noise_b1, noise_b2; /**< 3-pole pinking filter states for analog pink noise */

    AnalogJitter jitter_gen;         /**< Voss-McCartney pink noise generator */
    float jitter_factor;             /**< Stepped once per block and reused for all samples */
    DroopFilter saw_droop;           /**< 1-pole saw wave AC droop filter */

    // DC Blocking Filters with precalculated coefficients
    float post_osc_dc_g;               /**< Post-DCO DC blocker integrator factor (~0.16 Hz) */
    float post_osc_dc_a;               /**< post_osc_dc_g / (1 + post_osc_dc_g), folded at init */
    float post_osc_dc_s;               /**< Post-DCO DC blocker integrator state */
    float post_vcf_dc_g;               /**< Post-VCF DC blocker integrator factor (~1.59 Hz) */
    float post_vcf_dc_a;               /**< post_vcf_dc_g / (1 + post_vcf_dc_g), folded at init */
    float post_vcf_dc_s;               /**< Post-VCF DC blocker integrator state */

    float current_pwm;               /**< Interpolated pulse-width duty cycle */

    // VCF
    IR3109Filter vcf;               /**< 4-Pole ZDF IR3109 OTA ladder filter */

    // ADSR Envelope
    int32_t env_stage;               /**< ENV_STAGE_IDLE, ATTACK, DECAY, SUSTAIN, RELEASE */
    float env_level;                 /**< Block-rate target envelope voltage [0.0, 1.0] */
    float current_env_level;          /**< Sample-interpolated continuous envelope voltage */
    float env_level_step;             /**< Linear ramp increment across 32-sample DMA block */
    float gate_gain;                 /**< Continuous analog RC-filtered gate control voltage */
    float env_target;                /**< Destination envelope voltage for current stage */
    float sub_taper;                 /**< Sub-osc log taper; block-rate, patch-constant */

    // Note allocation & voice tracking
    int32_t note;                   /**< MIDI note number currently assigned (-1 if idle) */
    float note_freq;                 /**< Fundamental note frequency in Hz */
    bool active;                    /**< True if currently sounding */
    int32_t age;                    /**< Age counter in samples for LRU voice stealing */
    bool key_held;                   /**< True if key is currently physically pressed down */

    float current_vibrato_ratio;      /**< Instantaneous pitch vibrato modulation factor */
    float lfo_delay_gain;             /**< LFO initial delay ramp factor [0.0, 1.0] */

    // Telemetry output registers
    float current_cutoff_hz;          /**< Instantaneous VCF cutoff frequency in Hz */
    float current_jitter_cents;       /**< Instantaneous pitch jitter offset in cents */
    float last_output_level;          /**< Instantaneous voice output amplitude */

    float sample_rate;               /**< Audio sample rate in Hz */
    float inv_sample_rate;            /**< Precomputed 1 / sample_rate */
} JunoVoiceCard;

// Curve lookup utilities
float get_j60_dco_lfo_semitones(float slider);
float get_j60_attack_tau(float slider);
float get_j60_dec_rel_tau(float slider);

/**
 * @brief Initialize voice card state, tolerances, and sub-circuits
 */
void voice_init(JunoVoiceCard* v, int32_t voice_index, const VoiceTolerance* tol, float sample_rate);

/**
 * @brief Trigger a Note On event on this voice card
 */
void voice_note_on(JunoVoiceCard* v, int32_t note, float velocity, float detune_cents);

/**
 * @brief Release the held key on this voice card (advances to RELEASE envelope stage)
 */
void voice_note_off(JunoVoiceCard* v);

/**
 * @brief Immediately silence and reset this voice card to IDLE
 */
void voice_silence(JunoVoiceCard* v);

/**
 * @brief Update voice control parameters at 32-sample block boundaries (e.g. 1500 Hz at 48kHz)
 */
void voice_update_control_block(
    JunoVoiceCard* v,
    const JunoEngineParams* params,
    float global_lfo,
    float pitch_bend_ratio,
    float power_sag_deflection
);

/**
 * @brief Process one single audio sample through the voice card (DCO -> VCF -> VCA)
 */
float voice_process_sample(
    JunoVoiceCard* v,
    const JunoEngineParams* params,
    float global_lfo,
    float pitch_bend_ratio,
    float pwm_val
);

#ifdef __cplusplus
}
#endif

#endif // JUNO_VOICE_H
