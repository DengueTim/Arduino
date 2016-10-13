
#include <math.h>
#include "TimerOne.h"

const int TO_FAN_PIN = 9;
const int FROM_FAN_PIN = 2;

const int THERM_IN = A6;
const int THERM_ENABLE = A0;

const float THERM_R_KOHMS = 4.7;


int fanTackCount = 0;
int fanRpm = 0;

float thermC = 0;

// Will be called between 0 and 140 times a second.
void fanTackPinIsr() {
  fanTackCount++;
}

int timerIsrCounter = 0;
// Gets call 25,000 times a second!
void fanTackTimerIsr() {
  timerIsrCounter++;
  if (timerIsrCounter >= 25000) {
    timerIsrCounter = 0;
    fanRpm = fanTackCount * (60 / 4); // Two ticks per rev. Two changes per tick
    fanTackCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hola!");
  
  pinMode(FROM_FAN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FROM_FAN_PIN), fanTackPinIsr, CHANGE);
  
  pinMode(TO_FAN_PIN, OUTPUT);
  digitalWrite(TO_FAN_PIN, 0);
  Timer1.initialize(40); // 25khz PWM for fan.
  Timer1.pwm(TO_FAN_PIN, 0);
  Timer1.attachInterrupt(fanTackTimerIsr);

  pinMode(THERM_IN, INPUT);
  pinMode(THERM_ENABLE, OUTPUT);
}

void loop() {
  digitalWrite(THERM_ENABLE, 1);
  int thermRaw = analogRead(THERM_IN);
  digitalWrite(THERM_ENABLE, 0);

  Serial.println(thermRaw, DEC);
  float thermKohms = (THERM_R_KOHMS * thermRaw) / (1024 - thermRaw); // Rt / (R + Rt)
  Serial.println(thermKohms, 3);
  
  /* Thermister ln curve from
   *  Kohms   temp C
   *  20      0
   *  10.7    22.5
   *  6.74     37
   *  1.1    100
  */
  thermC = 103.21 - 34.37 * (float)log(thermKohms);
  Serial.println(thermC, 3);
  delay(1000);
}
