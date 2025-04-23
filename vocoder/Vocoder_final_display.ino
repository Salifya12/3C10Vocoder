
#include <Adafruit_SSD1306.h> // Library for the OLED display 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// SPI connections for the display
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// input pins for the wave switch latches 
// when equal to 1 the wave is ON, when equal to 0 it is OFF 
const int SINE = 4;
const int TRIANGLE = 3;
const int SAWTOOTH = 2;
const int SQUARE = 1;

// input pins for the wave frequency potentiometers 
const int SINE_FREQ = A2;
const int TRIANGLE_FREQ = A3;
const int SAWTOOTH_FREQ = A4;
const int SQUARE_FREQ = A5;

// prints all of the wave type names on the display
void print_wave_names()
{
	display.setCursor(0, 0); // set the cursor to the right position. This ensures the text is printed where wated on the screen.
  display.println("Sine");

	display.setCursor(0, 16); // set the cursor to the right position. This ensures the text is printed where wated on the screen.
  display.println("Triangle");

	display.setCursor(0, 32); // set the cursor to the right position. This ensures the text is printed where wated on the screen.
  display.println("Sawtooth");

	display.setCursor(0, 48); // set the cursor to the right position. This ensures the text is printed where wated on the screen.
  display.println("Square");

}

// prints if each wave is set to ON or OFF in the correct position on the screen
void print_on_or_off(int x, int y, const int WAVE)
{
	display.setCursor(x,y); // set the cursor to the right position. This ensures the text is printed where wated on the screen.

	if (digitalRead(WAVE) == 1) // check if the switch latch is pressed
		display.println("ON");
	else
		display.println("OFF");

}

// prints the frequency of each wave in the correct position on the screen
void print_freq(int x, int y, const int FREQ)
{
	display.setCursor(x,y); // set the cursor to the right position. This ensures the text is printed where wated on the screen.

	float freq = 200.0f + 400.0f * (analogRead(FREQ) / 1023.0f); // calculate the frequency from the potentiometer
	char str[64];

	itoa((int)freq, str, 10); // convert frequency to a string 
	display.println(str);
}

void setup() 
{
  Serial.begin(9600);

	// initialise all pins as inputs
	pinMode( SINE, INPUT);
	pinMode( TRIANGLE, INPUT); 
	pinMode( SAWTOOTH, INPUT);
	pinMode( SQUARE, INPUT);
	pinMode( SINE_FREQ, INPUT);
	pinMode( TRIANGLE_FREQ, INPUT); 
	pinMode( SAWTOOTH_FREQ, INPUT);
	pinMode( SQUARE_FREQ, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Loop forever if the display doesn't start correctly
  }

	display.setTextSize(0.5); // set the size of the text printed on the display
  display.setTextColor(WHITE); // set the colour of the text to white 

	// Clear the display screen
  display.clearDisplay();
  display.display();
}

void loop() 
{
  display.clearDisplay(); // clear the display

	// print the name of all the waves
	print_wave_names();

	// print if each wave is ON or OFF
	print_on_or_off(60, 0, SINE);
	print_on_or_off(60, 16, TRIANGLE);
	print_on_or_off(60, 32, SAWTOOTH);
	print_on_or_off(60, 48, SQUARE);

	// print the frequency of each wave
	print_freq(90, 0, SINE_FREQ);
	print_freq(90, 16, TRIANGLE_FREQ);
	print_freq(90, 32, SAWTOOTH_FREQ);
	print_freq(90, 48, SQUARE_FREQ);

	display.display(); // Display the new info on the screen 
}
