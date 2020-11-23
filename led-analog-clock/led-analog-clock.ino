/*******************************************************************************
 * led-analog-clock.ino
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

/*******************************************************************************
 * Notes *
 * None
 * ****************************************************************************/

//includes
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
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
#define MIN_BTN_PIN 3
#define AUX_BTN_PIN 4

//globals
//led pins
const int ledPinArr[NUM_LEDS] = {LED_0_PIN, LED_1_PIN, LED_2_PIN, LED_3_PIN, LED_4_PIN};
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
//time
#define HOUR 0                     //hour idx in time array
#define MIN 1                      //minute idx in time array
#define SEC 2                      //second idx in time array
volatile int time[3] = {12, 0, 0}; //always start at 12:00:00

//HW Diagnostics
#define PERFORM_DIAG 0
int ledCtr = 5;

//semaphore handles
SemaphoreHandle_t xTimer1Sem;

//task handles
TaskHandle_t xTimingTask;

//ISRs
ISR(TIMER1_OVF_vect)
{
  //give the semaphore for the timing task
  xSemaphoreGiveFromISR(xTimer1Sem);
}

void setup()
{
  //configure 16-bit timer 1 to generate 1Hz interrupt
  TCCR1A = 0;             //Normal-Mode on Timer1
  TCNT1 = 3036;           //Set Start cnt to 3036 to force 1Hz overflow
  TIMSK1 |= (1 << TOIE1); //Enable Timer1 interrupts
  TCCR1B |= (1 << CS12);  //Enable i/o clk + 256 prescaler

  //create the binary semaphore for taskTiming
  xTimer1Sem = xSemaphoreCreateBinary();

  //create tasks
  (void)xTaskCreate(
      taskTiming,   //function that performs the task
      "timing",     //task description
      128,          //task stack size
      NULL,         //parameter passed to task
      2,            //task priority
      xTimingTask); //task handle

#if PERFORM_DIAG
  Serial.begin(9600);
#endif

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
#if PERFORM_DIAG
  performDiagTests();
#endif

  //do nothing, everything is done in tasks
}

//***tasks***
/*******************************************************************************
 * taskTiming
 * 
 * Description *
 * Update global time values when timer overflows
 * 
 * Arguments *
 * 
 * Returns *
 * ****************************************************************************/
void taskTiming(void *pvParameters __attribute__((unused)))
{
  //a task should never return
  while (1)
  {
    //block until we are freed by the 1Hz Timer 1 ISR
    xSemaphoreTake(xTimer1Sem, portMAX_DELAY);

    if (time[SEC] >= 59)
    {
      if (time[MIN] >= 59)
      {
        if (time[HOUR] >= 12)
        {
          time[HOUR] = 1;
          time[MIN] = 0;
          time[SEC] = 0;
        }
        else
        {
          time[HOUR]++;
          time[MIN] = 0;
          time[SEC] = 0;
        }
      }
      else
      {
        time[MIN]++;
        time[SEC] = 0;
      }
    }
    else
    {
      time[SEC]++;
    }
  } //end of while(1)
}

/*******************************************************************************
 * taskLighting
 * 
 * Description *
 * Use global animation mode value to determine how to illuminate the time
 * Use global hours and min to illuminate/animate current hour and minutes
 * 
 * Arguments *
 * 
 * Returns *
 * ****************************************************************************/
void taskLighting(void *pvParameters __attribute__((unused)));

/*******************************************************************************
 * taskButtons
 * 
 * Description *
 * Increment global hours or mins value if either the hour or minute button is
 * pressed.
 * Increment global animation mode value if the auxiliary button is pressed
 * 
 * Arguments *
 * 
 * Returns *
 * ****************************************************************************/
void taskButtons(void *pvParameters __attribute__((unused)));

/*******************************************************************************
 * taskAnimationMode
 * 
 * Description *
 * Illuminate the auxiliary LED to indicate current animation mode when signaled
 * every 30 seconds.
 * 
 * Arguments *
 * 
 * Returns *
 * ****************************************************************************/
void taskAnimationMode(void *pvParameters __attribute__((unused)));

//***functions***
/*******************************************************************************
 * colorWipe
 * 
 * Description *
 * Wipe a color across LEDs of the neopixel strip
 * 
 * Arguments *
 * color - 32-bit RGB color value, use neopixel strip.color() to generate
 * wait  - Number of ms to wait before terminating
 * 
 * Returns *
 * void
 * ****************************************************************************/
void colorWipe(uint32_t color, int wait)
{
  for (int i = 0; i < strip.numPixels(); i++)
  {                                // For each pixel in strip...
    strip.setPixelColor(i, color); //  Set pixel's color (in RAM)
    strip.show();                  //  Update strip to match
    delay(wait);                   //  Pause for a moment
  }
}

/*******************************************************************************
 * performDiagTests
 * 
 * Description *
 * Test all buttons, leds and the neopixel strip. Test will turn on each LED for
 * one second each. Then wipe red, green, and blue, across the neopixel strip. 
 * Button presses will be confirmed via the serial monitor (serial must be 
 * enabled). All must be pressed when checked. Each test happens only once.
 * 
 * Arguments *
 * None
 * 
 * Returns *
 * void
 * ****************************************************************************/
void performDiagTests(void)
{
  //test leds
  for (int i = 0; i < NUM_LEDS; i++)
  {
    digitalWrite(ledPinArr[i], HIGH);
    delay(1000);
    digitalWrite(ledPinArr[i], LOW);
  }

  //test led strip
  colorWipe(strip.Color(255, 0, 0), 50); // Red
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  colorWipe(strip.Color(0, 0, 255), 50); // Blue

  //collect button states and print
  if (digitalRead(HOUR_BTN_PIN) == LOW)
    Serial.print("Hour Pressed\n");
  if (digitalRead(MIN_BTN_PIN) == LOW)
    Serial.print("Minute Pressed\n");
  if (digitalRead(AUX_BTN_PIN) == LOW)
    Serial.print("Aux Pressed\n");
}