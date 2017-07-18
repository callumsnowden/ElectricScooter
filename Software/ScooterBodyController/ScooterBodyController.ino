#include <SoftwareSerial.h>
#include <Servo.h>
#include "FastLED.h"

/*
 * D6 - Fan PWM - violet
 * D9 - ESC - pin header
 * D10 - LEDs - green
 * D11 - Status LED - red
 * D12 - Safety switch - brown
 * 
 * RS485 Pins
 * D5 - RS485 Control
 * D7 - RS485 DI - Tx
 * D8 - RS485 DO - Rx
 * 
 * A0 - Throttle - blue
 * A1 - Motor Temp - turquoise
 * A2 - ESC Temp - purple
 * A3 - DC Bus Voltage
 * A4 - 12V Bus Voltage
 * A5 - DC Bus Current
 */

//RS485 Pins
#define RS485_TX 7
#define RS485_RX 8
#define RS485_CONTROL 5

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

//various other sense pins for bus voltages and currents
#define MOTORTEMP_PIN A1
#define ESCTEMP_PIN A2
#define DCBUSV_PIN 3
#define LVBUSV_PIN 4
#define DCBUSC_PIN 5
#define FAN_PIN 6

#define BUS_R1 100000
#define BUS_R2 10000

//status LED pin
#define STATUS_LED 11

//safety switch pin
#define SAFETY_SWITCH 12

//throttle min and hysteresis value for where the power should start being applied
uint8_t throttleMin = 0;
uint8_t throttleMinHyst = 8;

uint8_t throttleMax = 0;
uint8_t throttleMaxHyst = 8;

uint8_t currentThrottlePercentage = 0;

//vcc of arduino, useful in true analogue calculations such as for temperature and voltage measurement
float vcc = 0;

SoftwareSerial RS485(RS485_RX, RS485_TX);

Servo ESC;
CRGB leds[NUM_LEDS];

CRGB statusLed[1];

void setStatusColour(CRGB colour) {
  statusLed[0] = colour;
  FastLED.show();
}

void setup() {
  analogWrite(6, 200);
  FastLED.addLeds<WS2811, STATUS_LED, GRB>(statusLed, 1);
  setStatusColour(CRGB::Red);
  Serial.begin(9600);
  pinMode(RS485_CONTROL, OUTPUT);
  digitalWrite(RS485_CONTROL, HIGH); //put into receive mode
  RS485.begin(57600);
  FastLED.addLeds<WS2812, LED_PIN>(leds, NUM_LEDS);
  for(uint8_t i = 0; i < 8; i++) {
    leds[i] = CRGB(0, 0, 0); //off
  }
  FastLED.show();

  /*
  //start polling for devices
  uint8_t HU_DISC[4] = {0xAA, 0x01, 0xFE, 0x55};
  for(uint8_t count = 0; count < 50; count++) {
    digitalWrite(RS485_CONTROL, HIGH);
    RS485.write(HU_DISC, sizeof(HU_DISC));
    digitalWrite(RS485_CONTROL, LOW);

    if(RS485.available()) {
      
    }
  }
  */
  
  pinMode(SAFETY_SWITCH, INPUT_PULLUP);
  ESC.attach(9);
  setStatusColour(CRGB::Orange);
  armESC();
  setStatusColour(CRGB::Green);
}

void readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  vcc = result / 1000; //convert to volts and store as float
}

void armESC() {
  Serial.println("Arming ESC");
  //1ms pulse for 2s arms the esc properly
  ESC.writeMicroseconds(1000);
  delay(2000);
  readVcc(); //everything has roughly settled at this point, read vcc
  Serial.print("SupplyV: ");
  Serial.println(vcc);
}

uint8_t readBusVoltage(uint16_t analog) {
  //fill in
}

uint8_t readTemp(uint8_t pin) {
  // Calculate the temperature based on the reading and send that value back
  float tempC = analogRead(pin);           //read the value from the sensor
  tempC = (vcc * tempC * 100.0)/1024.0;
  return tempC;
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
  for(uint8_t i = 0; i < (THROTTLE_SAMPLES - 1); i++) {
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
  Serial.print("ThCur%: "); Serial.println(currentThrottlePercentage);

  currentThrottlePercentage = dealWithThrottle();
  
  //set motor output based on throttle processing
  writeSpeed(currentThrottlePercentage);

  uint8_t pkt[10] = {0xAA, currentThrottlePercentage, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x55};

  RS485.write(pkt, sizeof(pkt));
  /*
  for(int i = 0; i < NUM_LEDS - 1; i++) {
    leds[i].setHue(map(currentThrottlePercentage, 0, 100, 0, 254));
  }
  FastLED.show();
  */
  delay(10);
}
