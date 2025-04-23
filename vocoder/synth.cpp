
#include <math.h>
#include <cstring>

#include "synth.h"

void synth_init(Synth *synth) {
	memset(synth->t, 0, sizeof(synth->t));

	for (int i = 0; i < 4; ++i) 
		synth->f[i] = 400.0f;

	synth->type = SynthType::Square;
}

float synth_run(Synth *synth, float dt) {
	float out = 0.0f;

	int scale = 0;

	for (int i = 0; i < 4; ++i) {
		synth->t[i] = fmodf(synth->t[i] + dt * synth->f[i], 1.0f);
	}

	if ((uint32_t)synth->type & (uint32_t)SynthType::Square) {
		out += (synth->t[0] < 0.5f) * 2.0f - 1.0f;
		++scale;
	}
	if ((uint32_t)synth->type & (uint32_t)SynthType::Sawtooth) {
		out += synth->t[1] * 2.0f - 1.0f;
		++scale;
	}
	if ((uint32_t)synth->type & (uint32_t)SynthType::Triangle) {
		out += (synth->t[2] + 2.0f * (0.5f - synth->t[2]) * (synth->t[2] > 0.5f)) * 4.0f - 1.0f;
		++scale;
	}
	if ((uint32_t)synth->type & (uint32_t)SynthType::Sine) {
		out += sinf(synth->t[3] * 2 * (float)M_PI);
		++scale;
	}

	if (scale != 0)
		out /= scale;

	return out;
}
