
#pragma once

enum class SynthType {
	Square   = 1<<0,
	Sawtooth = 1<<1,
	Triangle = 1<<2,
	Sine     = 1<<3
};

struct Synth {
	SynthType type;
	float f[4];
	float t[4];
};

void synth_init(Synth *synth);
float synth_run(Synth *synth, float dt);
