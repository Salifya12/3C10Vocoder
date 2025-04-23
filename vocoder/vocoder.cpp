
#include "vocoder.h"

#include <cstring>
#include <cmath>

static void configure_filter(Filter *filter, float f_l, float f_u, float gain, float sample_rate) {
	float f_0 = 0.5f * (f_u + f_l);
	float Q = f_0 / (f_u - f_l);

	float omega = 2.0f * (float)M_PI * f_0 / sample_rate;
	float alpha = sinf(omega) / (2.0f * Q);

	float a0 = 1.0f + alpha;
	filter->b0 = (alpha * gain) / a0;
	filter->b2 = (-filter->b0) / a0;
	filter->a1 = -2.0f * cosf(omega) / a0;
	filter->a2 = (1.0f - alpha) / a0;
}

static float filter_step(Filter *filter, FilterState *state, float x) {
	float y =
		filter->b0 *        x  +
		filter->b2 * state->x2 -
		filter->a1 * state->y1 -
		filter->a2 * state->y2;

	state->x2 = state->x1;
	state->x1 = x;
	state->y2 = state->y1;
	state->y1 = y;

	return y;
}

static float cascade_step(Filter *filter, FilterState *states, int state_count, float x) {
	float out = filter_step(filter, states, x);
	for (int i = 1; i < state_count; ++i)
		out = filter_step(filter, states + i, out);
	return out;
}

static void envelope_step(float *state, float a, float x) {
	*state = (1 - a) * *state + a * std::abs(x);
}

void vocoder_init(Vocoder *vocoder) {
	float step = (vocoder->f_u - vocoder->f_l) / vocoder->filter_count;
	for (int i = 0; i < vocoder->filter_count; ++i)
		configure_filter(
			vocoder->filters + i,
			vocoder->f_l + step * i,
			vocoder->f_l + step * (i + 1),
			1.0f, vocoder->sample_rate
		);
	
	memset(vocoder->input_state   , 0, sizeof(vocoder->   input_state));
	memset(vocoder->carrier_state , 0, sizeof(vocoder-> carrier_state));
	memset(vocoder->envelope_state, 0, sizeof(vocoder->envelope_state));
}

float vocoder_update(Vocoder *vocoder, float input, float synth_input) {
	float out[vocoder->filter_count];
	float synth_out[vocoder->filter_count];

	for (int i = 0; i < vocoder->filter_count; ++i) {
		out[i] = cascade_step(
			vocoder->filters + i,
			vocoder->input_state + i * vocoder->cascade_count,
			vocoder->cascade_count,
			input
		);

		synth_out[i] = cascade_step(
			vocoder->filters + i,
			vocoder->carrier_state + i * vocoder->cascade_count,
			vocoder->cascade_count,
			synth_input
		);
	}

	for (int i = 0; i < vocoder->filter_count; ++i)
		envelope_step(vocoder->envelope_state + i, vocoder->env_a, out[i]);

	float signal = 0.0f;
	for (int i = 0; i < vocoder->filter_count; ++i) {
		signal += synth_out[i] * vocoder->envelope_state[i];
	}
	
	return signal;
}







































