
#include <math.h>
#include "TimerOne.h"

#define TO_FAN_PWM_PIN        9

#define THERM_ENABLE_PIN      A0 // 14
#define THERM_IN_PIN          A6

#define THERM_R_KOHMS         4.7
#define ANALOG_MAX            1024

volatile bool doEverySecond = false;

void timer25kHz() {
  // Counts from 0 to 24999
  static uint16_t timer25kHzCounter = 0;
 
  timer25kHzCounter++;
  if (timer25kHzCounter >= 25000) {
    timer25kHzCounter = 0;
    doEverySecond = true;
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("test");

  pinMode(TO_FAN_PWM_PIN, OUTPUT);
  digitalWrite(TO_FAN_PWM_PIN, 1);

  pinMode(THERM_IN_PIN, INPUT);
  analogRead(THERM_IN_PIN);
  
  pinMode(THERM_ENABLE_PIN, OUTPUT);
  digitalWrite(THERM_ENABLE_PIN, 0);
  
  Timer1.initialize(40); // 25khz
  Timer1.attachInterrupt(timer25kHz);
}

void loop() {
  static float thermC = 0;
  static int analogReadCount = 0;
  
  if (doEverySecond) {
    doEverySecond = false;
    Serial.print("tick:");
    Serial.println(micros(), DEC);
    Serial.print("thermC:");
    Serial.println(thermC, 3);
    Serial.print("analogReadCount:");
    Serial.println(analogReadCount, DEC);
    analogReadCount = 0;
  }
  
  // Read temp ADC.
  digitalWrite(THERM_ENABLE_PIN, 1);
  uint16_t thermRaw = analogRead(THERM_IN_PIN);
  digitalWrite(THERM_ENABLE_PIN, 0);
  float thermKohms = (THERM_R_KOHMS * thermRaw) / (ANALOG_MAX - thermRaw); // Rt / (R + Rt)
    /* Thermister ln curve from
   *  Kohms   temp C
   *  20      0
   *  10.7    22.5
   *  6.74     37
   *  1.1    100
  */
  thermC = 103.21 - 34.37 * (float)log(thermKohms);
  analogReadCount++;
}
