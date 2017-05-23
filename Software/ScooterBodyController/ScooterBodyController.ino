#include <Servo.h>
#include "FastLED.h"

#define NUM_LEDS 8
#define LED_PIN 10

#define THROTTLE_SAMPLES 10
#define THROTTLE_DTIME 2

//determine these two through arbitrary experimentation, will vary from throttle to throttle
#define THROTTLE_MAX 800
#define THROTTLE_MIN 180

//throttle analogue pin
#define THROTTLE_PIN 0

//max percentage of output at max throttle, essentially a "power limit"
#define THROTTLE_MAXP 100

//throttle min and hysteresis value for where the power should start being applied
int throttleMin = 0;
int throttleMinHyst = 8;

int throttleMax = 0;
int throttleMaxHyst = 8;

int currentThrottlePercentage = 0;

Servo ESC;
CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN>(leds, NUM_LEDS);
  for(int i = 0; i < 8; i++) {
    leds[i] = CRGB(0, 0, 0); //off
  }
  FastLED.show();
  ESC.attach(9);
  armESC();
}

void armESC() {
  Serial.println("Arming ESC");
  //1ms pulse for 2s arms the esc properly
  ESC.writeMicroseconds(1000);
  delay(2000);
}

void writeSpeed(uint8_t percent) {
  uint16_t escPosition = 0;
  //map between 1ms and 2ms, any less/more doesn't work and makes esc throw a hissy fit
  escPosition = map(percent, 0, 100, 1000, 2000);

  //print debugging info
  Serial.print("ESC: "); Serial.print(escPosition); Serial.println();
  ESC.writeMicroseconds(escPosition);
}

uint8_t dealWithThrottle() {
  //throttle dealing with code, returns a value between 0-THROTTLE_MAXP% of throttle
  uint32_t average = 0; //large variable because the average could potentially be a large value

  //take an average of THROTTLE_SAMPLES at THROTTLE_DTIME delay
  for(int i = 0; i < (THROTTLE_SAMPLES - 1); i++) {
    average += analogRead(THROTTLE_PIN);
    delay(THROTTLE_DTIME);
  }

  //convert to actual value
  uint16_t currentThrottle = average / THROTTLE_SAMPLES; //up to 1024 which is adc max value
  
  //map throttle to value between 0-THROTTLE_MAXP%
  uint8_t throttlePercentage = 0; //up to 100% but dependant on THROTTLE_MAXP
  if(currentThrottle < THROTTLE_MIN) currentThrottle = THROTTLE_MIN; //if we're below THROTTLE_MIN set to THROTTLE_MIN, eliminates noise based error
  throttlePercentage = map(currentThrottle, THROTTLE_MIN, THROTTLE_MAX, 0, THROTTLE_MAXP); //map to convert to percent output

  //we done, woo!
  return throttlePercentage;
}


void loop() {
  //print current throttle value for debugging
  Serial.print("ThCur%: "); Serial.print(currentThrottlePercentage); Serial.print(", ");

  currentThrottlePercentage = dealWithThrottle();
  
  //set motor output based on throttle processing
  writeSpeed(currentThrottlePercentage);
  /*
  for(int i = 0; i < NUM_LEDS - 1; i++) {
    leds[i].setHue(map(currentThrottlePercentage, 0, 100, 0, 254));
  }
  FastLED.show();
  */
  delay(50);
}
