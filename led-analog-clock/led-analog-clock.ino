/*******************************************************************************
 * LED Analog Clock Project
 * 
 * Description *
 * The LED Analog Clock Project is a small idea I had to digitize an analog
 * clock received as a gift. The issue with some analog clocks is that they can
 * be quite loud. This isn't helpful when you are trying to sleep or meditate as
 * it can be very distracting. Not that LEDs would be much better but at least
 * they are silent.
 * 
 * This project was not intended to store time on an external flash when powered
 * down. As a result time should be reset to 12:00 when power is cut. Every time
 * the clock is powered up, a user will need to adjust the hour and minute.
 * 
 * Features *
 * Lighting of time via neopixel led strip and individual leds
 * Ability to change hour and minute via external buttons
 * Ability to increment time by holding down respective hour/minute buttons
 * 
 * Author *
 * Juan Caraballo
 * ****************************************************************************/

//includes
#include <Adafruit_NeoPixel.h>

//defines
#define NUM_LEDS 5
#define NUM_PIXELS 12

#define PIXEL_PIN 10
#define LED_0_PIN 5
#define LED_1_PIN 6
#define LED_2_PIN 7
#define LED_3_PIN 8
#define LED_4_PIN 9

#define HOUR_BTN_PIN 2
#define MIN_BTN_PIN  3
#define AUX_BTN_PIN  4

//globals
//led pins
const int ledPinArr[NUM_LEDS] = {LED_0_PIN, LED_1_PIN, LED_2_PIN, LED_3_PIN, LED_4_PIN};
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  // put your setup code here, to run once:

  //configure neopixel strip output
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  //configure led outputs
  for (int i = 0; i < NUM_LEDS; i++)
    pinMode(ledPinArr[i], OUTPUT);

  //configure button inputs
  pinMode(HOUR_BTN_PIN, INPUT_PULLUP);
  pinMode(MIN_BTN_PIN, INPUT_PULLUP);
  pinMode(AUX_BTN_PIN, INPUT_PULLUP);
}

void loop()
{
  // here is where you'd put code that needs to be running all the time.
}