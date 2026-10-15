/*
 * Copyright (c) 2026 Ed Coffey
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "juno_engine.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    printf("Running mcuno-32 smoke test...\n");

    JunoEngine engine;
    const float SAMPLE_RATE = 48000.0f;
    juno_engine_init(&engine, SAMPLE_RATE);

    // Play MIDI note 60 (Middle C)
    juno_engine_note_on(&engine, 60, 0.9f);

    float out_l[32];
    float out_r[32];
    float max_abs_sample = 0.0f;

    // Render 10 blocks (320 samples)
    for (int block = 0; block < 10; block++) {
        juno_engine_process_block(&engine, out_l, out_r, 32);

        for (int i = 0; i < 32; i++) {
            if (isnan(out_l[i]) || isinf(out_l[i]) || isnan(out_r[i]) || isinf(out_r[i])) {
                fprintf(stderr, "Error: NaN or Inf detected in audio output at block %d sample %d\n",
                        block, i);
                return 1;
            }
            float abs_l = fabsf(out_l[i]);
            float abs_r = fabsf(out_r[i]);
            if (abs_l > max_abs_sample) max_abs_sample = abs_l;
            if (abs_r > max_abs_sample) max_abs_sample = abs_r;
        }
    }

    if (max_abs_sample <= 0.0001f) {
        fprintf(stderr, "Error: Engine produced silent output (max amplitude %f)\n", max_abs_sample);
        return 1;
    }

    // Release note
    juno_engine_note_off(&engine, 60);

    // Render 5 more blocks
    for (int block = 0; block < 5; block++) {
        juno_engine_process_block(&engine, out_l, out_r, 32);
        for (int i = 0; i < 32; i++) {
            if (isnan(out_l[i]) || isinf(out_l[i]) || isnan(out_r[i]) || isinf(out_r[i])) {
                fprintf(stderr, "Error: NaN or Inf detected during release\n");
                return 1;
            }
        }
    }

    juno_engine_all_notes_off(&engine);
    printf("mcuno-32 smoke test passed successfully (peak amplitude: %f).\n", max_abs_sample);
    return 0;
}
