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
#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>

//defines
#define NUM_LEDS 5
#define NUM_MIN_LEDS (NUM_LEDS-1)
#define NUM_PIXELS 12

#define PIXEL_PIN 10
#define LED_0_PIN 5
#define LED_1_PIN 6
#define LED_2_PIN 7
#define LED_3_PIN 8
#define LED_AUX_PIN 9

#define HOUR_BTN_PIN 2
#define MIN_BTN_PIN 3
#define AUX_BTN_PIN 4

#define RTC_SQR_PIN 11
// GLOBALS
//led pins
const int ledPinArr[NUM_LEDS] = {LED_0_PIN, LED_1_PIN, LED_2_PIN, LED_3_PIN, LED_AUX_PIN};
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
//time
#define HOUR 0                     //hour idx in time array
#define MIN 1                      //minute idx in time array
#define SEC 2                      //second idx in time array
volatile int time[3] = {12, 0, 0}; //always start at 12:00:00

//button debounce
unsigned long debounceDelay = 300;
unsigned long lastHourDTime = 0;
unsigned long lastMinDTime  = 0;
unsigned long lastAuxDTime   = 0;

// HW Diagnostics
// #define PERFORM_DIAG

// ISRs
ISR(TIMER1_OVF_vect) //Timer1
{
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
}

void hourBtnInterruptHandler(void)
{
  handleButtonPress(HOUR_BTN_PIN);
}

void minBtnInterruptHandler(void)
{
  handleButtonPress(MIN_BTN_PIN);
}

void auxBtnInterruptHandler(void)
{
  handleButtonPress(AUX_BTN_PIN);
}

void setup()
{
  #ifdef PERFORM_DIAG
  Serial.begin(9600);
  #else
  noInterrupts(); //disable interrupts while configuring interrupt related items
    //configure 16-bit timer 1 to generate 1Hz interrupt
  TCCR1A = 0;             //Normal-Mode on Timer1
  TCCR1B = 0;             //Temporarily disable timer clock for configuration
  TCNT1 = 3036;           //Set Start cnt to 3036 to force 1Hz overflow
  TCCR1B |= (1 << CS12);  //Enable i/o clk + 256 prescaler
  TIMSK1 |= (1 << TOIE1); //Enable Timer1 interrupts

//configure pin change interrupts for the hour, min, and aux btns
  attachInterrupt(digitalPinToInterrupt(HOUR_BTN_PIN), hourBtnInterruptHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(MIN_BTN_PIN), minBtnInterruptHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUX_BTN_PIN), auxBtnInterruptHandler, FALLING);
  interrupts(); //reenable interrupts since we are done with related configs
  #endif //#ifdef PERFORM_DIAG

  //configure neopixel strip output
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)

  //configure led outputs
  for (int i = 0; i < NUM_LEDS; i++)
    pinMode(ledPinArr[i], OUTPUT);
  clearMinLeds();

  //configure button inputs
  pinMode(HOUR_BTN_PIN, INPUT_PULLUP);
  pinMode(MIN_BTN_PIN, INPUT_PULLUP);
  pinMode(AUX_BTN_PIN, INPUT_PULLUP);
}

void loop()
{
  #ifdef PERFORM_DIAG
  performDiagTests();
  #else
  taskLighting();
  #endif
}

// TASKS
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
void taskLighting(void)
{
  uint8_t hourPixelIdx = 0;
  uint8_t minPixelIdx = 0;
  uint8_t minLedIdx = 255; //255 used to indicate that we should not light any
  uint8_t tempHourPixelIdx = 0;
  uint8_t tempMinPixelIdx = 0;
  uint8_t tempMinLedIdx = 255;

  //do initial illumination based on global time
  getPixelAndLedIndices(&hourPixelIdx, &minPixelIdx, &minLedIdx);

  if (hourPixelIdx == minPixelIdx)
  {
    strip.setPixelColor(hourPixelIdx, strip.Color(255,255,255)/*white*/);
  }
  else
  {
    strip.setPixelColor(hourPixelIdx, strip.Color(0,255,255)/*cyan*/);
    strip.setPixelColor(minPixelIdx, strip.Color(255,0,255)/*pink*/);
  }
  strip.show();

  if (minLedIdx < NUM_MIN_LEDS)
  {
    digitalWrite(ledPinArr[minLedIdx], HIGH);
  }

  
  //tasks should never return
  while(1)
  {
    getPixelAndLedIndices(&tempHourPixelIdx, &tempMinPixelIdx, &tempMinLedIdx);
    
    //if hour or minute indices changed, update the neopixels
    if ((hourPixelIdx != tempHourPixelIdx)
      || minPixelIdx != tempMinPixelIdx)
    {
      hourPixelIdx = tempHourPixelIdx;
      minPixelIdx = tempMinPixelIdx;

      strip.clear();
      if (hourPixelIdx == minPixelIdx)
      {
        strip.setPixelColor(hourPixelIdx, strip.Color(255,255,255)/*white*/);
      }
      else
      {
        strip.setPixelColor(hourPixelIdx, strip.Color(0,255,255)/*cyan*/);
        strip.setPixelColor(minPixelIdx, strip.Color(255,0,255)/*pink*/);
      }
      strip.show();
    }

    if(minLedIdx != tempMinLedIdx)
    {
      minLedIdx = tempMinLedIdx;
      if (minLedIdx < NUM_MIN_LEDS)
      {
        digitalWrite(ledPinArr[minLedIdx], HIGH);
      }
      else
      {
        clearMinLeds();
      }
    }
  } //end of while(1)
}

// FUNCTIONS
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

/*******************************************************************************
 * handleButtonPress
 * 
 * Description *
 * When the hour, minute, or aux button has been pressed, respond accordingly.
 * 
 * Arguments *
 * btnPressed - An ID for which button was pressed. Uses their pin numbers.
 * 
 * Returns *
 * void
 * ****************************************************************************/
void handleButtonPress(uint8_t btnPin)
{
  unsigned long timeReading = millis();

  switch (btnPin)
  {
    case HOUR_BTN_PIN:
    {
      if ((timeReading - lastHourDTime) > debounceDelay)
      {
        time[SEC] = 0; //reset seconds if changing time
        if (time[HOUR] >= 12)
          time[HOUR] = 1;
        else
          time[HOUR]++;

        lastHourDTime = timeReading;
      }
      break;
    }

    case MIN_BTN_PIN:
    {
      if((timeReading - lastMinDTime) > debounceDelay)
      {
        time[SEC] = 0; //reset seconds if changing time
        if (time[MIN] >= 59)
        {
          if (time[HOUR] >= 12)
          {
            time[HOUR] = 1;
            time[MIN] = 0;
          }
          else
          {
            time[HOUR]++;
            time[MIN] = 0;
          }
          
        }
        else
          time[MIN]++;

        lastMinDTime = timeReading;
      }
      break;
    }

    case AUX_BTN_PIN:
    {
      //do nothing for now
      break;
    }
  }
}

/*******************************************************************************
 * getPixelAndLedIndices
 * 
 * Description *
 * Using global time params, determine what neopixel LED to illuminate for the
 * hour and minute, also determine what LED (if any) should be illuminated for
 * the minute.
 * 
 * Arguments *
 * hourPixelIdx - Location to store idx to be used for hour pixel
 * minPixelIdx  - Location to store idx to be used for minute pixel
 * minLedIdx    - Location to store idx to be used for minute led (if any)
 *                This value will be filled with 255 if no LED should be lit.
 * 
 * Returns *
 * void
 * ****************************************************************************/
void getPixelAndLedIndices(uint8_t* hourPixelIdx, uint8_t* minPixelIdx, 
  uint8_t* minLedIdx)
{
  uint8_t currMin = time[MIN];
  uint8_t minModFive = 0;

  //pixel 0 is at the 12 hour mark
  //hours are 1-12
  // H12 : LED0
  // H1  : LED1
  // H2  : LED2
  // H3  : LED3
  // H4  : LED4
  // H5  : LED5
  // H6  : LED6
  // H7  : LED7
  // H8  : LED8
  // H9  : LED9
  // H10 : LED10
  // H11 : LED11
  if (time[HOUR] == 12)
    *hourPixelIdx = 0;
  else
    *hourPixelIdx = time[HOUR];

  minModFive = time[MIN] % 5;
  if(minModFive == 0)
    *minLedIdx = 255; //use a bad val to indicate not to use the leds for min
  else
    *minLedIdx = minModFive - 1;

  if ((currMin >= 0) && (currMin < 5))
  {
    *minPixelIdx = 0;
  }
  else if ((currMin >= 5) && (currMin < 10))
  {
    *minPixelIdx = 1;
  }
  else if ((currMin >= 10) && (currMin < 15))
  {
    *minPixelIdx = 2;
  }
  else if ((currMin >= 15) && (currMin < 20))
  {
    *minPixelIdx = 3;
  }
  else if ((currMin >= 20) && (currMin < 25))
  {
    *minPixelIdx = 4;
  }
  else if ((currMin >= 25) && (currMin < 30))
  {
    *minPixelIdx = 5;
  }
  else if ((currMin >= 30) && (currMin < 35))
  {
    *minPixelIdx = 6;
  }
  else if ((currMin >= 35) && (currMin < 40))
  {
    *minPixelIdx = 7;
  }
  else if ((currMin >= 40) && (currMin < 45))
  {
    *minPixelIdx = 8;
  }
  else if ((currMin >= 45) && (currMin < 50))
  {
    *minPixelIdx = 9;
  }
  else if ((currMin >= 50) && (currMin < 55))
  {
    *minPixelIdx = 10;
  }
  else //((currMin >= 55) && (currMin < 60 /*0*/ ))
  {
    *minPixelIdx = 11;
  }
  return;
}

/*******************************************************************************
 * clearMinLeds
 * 
 * Description *
 * Disable all minute LEDs
 * 
 * Arguments *
 * void
 * 
 * Returns *
 * void
 * ****************************************************************************/
void clearMinLeds(void)
{
  for (int i = 0; i < NUM_MIN_LEDS; i++)
    digitalWrite(ledPinArr[i], LOW);

  return;
}
