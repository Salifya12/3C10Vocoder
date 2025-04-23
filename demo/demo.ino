
#include <Adafruit_SSD1306.h>
#include <SPI.h>

#define PI 3.141592653589793238

#define CS_PIN 15

SPIClass spi(HSPI);

void dac_write(int value) {
	value &= 0x0FFF;
	value |= 0x5000;

	spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS_PIN, LOW);

	spi.transfer16(value);

	digitalWrite(CS_PIN, HIGH);
	spi.endTransaction();
}

void dac_write_float(float value) {
	dac_write(int(min(2047.0f, value * 2047.0f)));
}

float sawtooth(float t) {
	return t * (1.0f / (2.0f * PI));
}

int prev_time;
int n;

#define BAND_COUNT 1

#define INPUT_PIN 35

float t[] = { 0.0f, 0.0f };
float f_mean[] = { 450, 650 };
float f_range[] = { 100, 100 };

const int vr_pins[] = { 25, 26 };
const float vr_range = 650.0f;

void setup() {
	Serial.begin(115200);

	pinMode(CS_PIN, OUTPUT);
	pinMode(INPUT_PIN, INPUT);

	for (int i = 0; i < BAND_COUNT; ++i) {
		pinMode(      vr_pins[i], INPUT);
	}

	digitalWrite(CS_PIN, HIGH);
	spi.begin(14, 12, 13, CS_PIN);

	prev_time = micros();
	n = 0;
}

void loop() {
	generate_signal();
	//sample_input();
}

void generate_signal() {
	int time = micros();
	int delta = time - prev_time;
	prev_time = time;
	double dt = delta / 1000000.0f;

	float output = 0.0f;
	for (int i = 0; i < 1; ++i) {
		float vr_input = analogRead(vr_pins[i]) / vr_range - 0.5f;
		float f = f_mean[i] + f_range[i] * vr_input;

		t[i] += 2.0f * PI * f * dt;
		t[i] = fmod(t[i], 2 * PI);

		//float output = t[i] * (1.0f / (2.0f * PI)); // Sawtooth
		float output = float(t[i] < PI);              // Square

		dac_write_float(output);
	}
}

void sample_input() {
	//int time = micros();
	int input = analogRead(INPUT_PIN);

	//Serial.printf("%d, %f\n", micros, input / 2047.0);
	Serial.println(input);
}
