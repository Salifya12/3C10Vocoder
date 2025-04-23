
#include <Arduino.h>
#include <SPI.h>
#include <cmath>

#define CS_PIN 15
static SPIClass spi(HSPI);

void dac_init() {
	pinMode(CS_PIN, OUTPUT);
	delay(1);
	digitalWrite(CS_PIN, HIGH);
	spi.begin(14, 12, 13, CS_PIN);
}

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
