

#include "dac.h"
#include "vocoder.h"

#include <cmath>

static constexpr int mic_pin = 34;

static Vocoder vocoder;
static Synth synth;

void setup() {
	Serial.begin(9600);

	pinMode(mic_pin, INPUT);

	pinMode(25, INPUT);
	pinMode(33, INPUT);
	pinMode(32, INPUT);
	pinMode(35, INPUT);
	pinMode(21, INPUT);
	pinMode( 3, INPUT);
	pinMode( 1, INPUT);
	pinMode(22, INPUT);
	pinMode(19, INPUT);
	
	dac_init();
	synth_init(&synth);
	vocoder_init(&vocoder);
}

void loop() {
	unsigned long start_time = micros();

	int run_vocoder = digitalRead(19);

	synth.type = (SynthType)(
		  (digitalRead(21) << 0)
		| (digitalRead( 3) << 1)
		| (digitalRead( 1) << 2)
		| (digitalRead(22) << 3)
	);

	synth.f[0] = 200.0f + 400.0f * (analogRead(25) / 4095.0f);
	synth.f[1] = 200.0f + 400.0f * (analogRead(33) / 4095.0f);
	synth.f[2] = 200.0f + 400.0f * (analogRead(32) / 4095.0f);
	synth.f[3] = 200.0f + 400.0f * (analogRead(35) / 4095.0f);

	float input = analogRead(mic_pin) / 4095.0f;
	float synth_input = synth_run(&synth, 1.0f/8000.0f);

	float output = 0.0f;
	if (run_vocoder) {
		static constexpr float gain = 100.0f;
		output = vocoder_update(&vocoder, input, synth_input) * gain;
		output = max(-1.0f, min(1.0f, output));
	}
	else {
		output = synth_input;
	}

	output = (output + 1.0f) * 0.5f;
	dac_write_float(output);

	unsigned long end_time = start_time + int(125);
	unsigned long time = micros();
	if (time > end_time)
		Serial.println("Running behind\n");
	else
		delayMicroseconds(end_time - time);
}
