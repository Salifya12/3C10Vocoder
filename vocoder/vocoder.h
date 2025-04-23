
#pragma once

#include "synth.h"

struct Filter {
	float a1, a2, b0, b2;
};

struct FilterState {
	float x1, x2, y1, y2;
};

struct Vocoder {
	static constexpr int   filter_count      = 4;
	static constexpr int   cascade_count     = 4;
	static constexpr float sample_rate       = 8000.0f;
	static constexpr float dt                = 1.0f/sample_rate;
	static constexpr float f_l = 500.0f, f_u = 2000.0f;
	static constexpr float env_a             = 0.1f;

	Filter filters[filter_count];
	FilterState input_state[filter_count * cascade_count];
	FilterState carrier_state[filter_count * cascade_count];

	float envelope_state[filter_count];
};

void vocoder_init(Vocoder *vocoder);
float vocoder_update(Vocoder *vocoder, float input, float synth_input);
