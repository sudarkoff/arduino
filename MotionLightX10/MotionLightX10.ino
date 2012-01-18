/**
 * X10 Motion/Light Sensor
 * Sends the ON command if it's dark and motion detected; OFF otherwise.
 */

#include <avr/sleep.h>
#include <avr/wdt.h>

#include <x10.h>
#include <x10constants.h>

//#define DEBUG

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// CONFIG
const int statusLed = 13;       // status LED
const int PIRPin = 5;           // motion sensor
const int photoResPin = 0;      // light sensor

const int zcPin = 9;            // X10 0-xing pin
const int dataPin = 8;          // X10 data pin
const int houseCode = HOUSE_G;  // X10 house code
const int unitCode = UNIT_3;    // X10 unit code
const int repeat = 2;           // X10 command repeat count

const int lightThreshold = 150; // light brightness threshold

const unsigned long delayBy = 90000ul;  // lights OFF delay (in usec)
volatile unsigned long timer = 0ul;     // usec since last motion

// STATE
#define NO_MOTION 0
#define MOTION 1
#define MOTION_NO_LIGHT 2

// GLOBALS
x10 house = x10(zcPin, dataPin);
boolean lightsOn;

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Status LED
  pinMode(statusLed, OUTPUT);

  // Initialize motion sensor
  pinMode(PIRPin, INPUT);
  digitalWrite(PIRPin, HIGH);
  // wait for the sensor to settle
  delay(5000);

  // Initialize light sensor
  pinMode(photoResPin, INPUT);

  // Initialize X10 module
  pinMode(zcPin, INPUT);
  pinMode(dataPin, OUTPUT);
  house.write(houseCode, ALL_UNITS_OFF, repeat);

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)
  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  // 0: 16ms, 1: 32ms, 2: 64ms, 3: 128ms, 4: 250ms,
  // 5: 500ms, 6: 1sec, 7: 2sec, 8: 4sec, 9: 8sec
  setup_watchdog(8);
}

void loop()
{
  switch (state()) {
    case NO_MOTION:
#ifdef DEBUG
      flashLed(statusLed, 5, 25);
      Serial.print("timer = "); Serial.println(timer);
      Serial.print("delayBy = "); Serial.println(delayBy);
      Serial.print("lightsOn = "); Serial.println(lightsOn? "ON": "OFF");
#endif
      if ((timer > delayBy) && lightsOn) {
        flashLed(statusLed, 2, 50);
        turnLights(false);
      }
      break;

    case MOTION:
      flashLed(statusLed, 1, 50);
      break;

    case MOTION_NO_LIGHT:
      flashLed(statusLed, 1, 50);
      if (!lightsOn) {
        flashLed(statusLed, 2, 50);
        turnLights(true);
      }
      break;

    default:
      flashLed(statusLed, 5, 50);
      break;
  }

#ifdef DEBUG
  delay(2);
#endif

  sleep();
}

void flashLed(int pin, int times, int wait) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}

inline int readMotion()
{
  int motion = digitalRead(PIRPin);
  if (motion == LOW) {
    timer = 0;
  }
  return (motion == LOW? HIGH: LOW);
}

int state()
{
  int motion = readMotion();
  int light = analogRead(photoResPin);

  if (motion == LOW)
  {
    return NO_MOTION;
  }
  else if (motion == HIGH)
  {
    if (light > lightThreshold)
    {
      return MOTION;
    }
    else
    {
      return MOTION_NO_LIGHT;
    }
  }
}

void turnLights(boolean state)
{
  lightsOn = state;

  // Send the ON/OFF command
  house.write(houseCode, unitCode, repeat);
  house.write(houseCode, (state == true)? ON: OFF, repeat);
}

void sleep()
{
  // Switch ADC OFF
  cbi(ADCSRA,ADEN);

  // Put ATmega to sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  // zzzzz.....

  // Wake up
  sleep_disable();

  // Switch ADC ON
  sbi(ADCSRA,ADEN);
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  timer += 4000; // increase the timer by 4 seconds
}

