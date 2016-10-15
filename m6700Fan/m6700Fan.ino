
#include <math.h>
#include "TimerOne.h"

#define TO_FAN_PWM_PIN        9
#define FROM_FAN_TACK_PIN     2

#define FROM_EC_PWM_PIN       15 // A0
#define FROM_EC_PWM_ADC_PIN   A3 // 17
#define FROM_EC_PWM_REF_PIN   16 // A2
#define TO_EC_TACK_PIN        7

#define THERM_ENABLE_PIN      A0 // 14
#define THERM_IN_PIN          A6

#define LED_PIN               13
#define SPARE_PWM_PIN         10

#define THERM_R_KOHMS         4.7

#define TACK_PASS_THROUGH     -1
#define TACK_STOP             0

#define TACK_MIN_25K          93
#define TACK_MAX_25K          375

#define PWM_MAX               1024
#define PWM_MIN_CUTOFF        256

#define ANALOG_MAX            1024

volatile bool fanUsPerRevValid = false;
volatile uint32_t fanUsPerRev = UINT32_MAX; // Stopped

// Desired time between transitions of tack to EC in 25,0000th of a second.
// Working values are 93(~4000rpm 100%) to 375(~1000rpm 25%)
// 0 Stopped
// -1 copy tack value from real fan every millisecond  
volatile uint16_t ecTackOutInterval = TACK_PASS_THROUGH;

volatile bool doEveryDecasecond = false;
volatile bool doEcTackOutToggle = false;

void everyDecasecond() {
  uint32_t localFanUsPerRev;
  if (fanUsPerRevValid) {
    fanUsPerRevValid = false;
    localFanUsPerRev = fanUsPerRev;
  } else {
    // No tack transitions, assume stopped.
    localFanUsPerRev = UINT32_MAX;
  }
  
  // Read EC PWM
  uint16_t rateFromEc = analogRead(FROM_EC_PWM_ADC_PIN);
  if (rateFromEc < 256) {
    rateFromEc = 0;
  } else if (rateFromEc > (ANALOG_MAX - 32)) {
    rateFromEc = PWM_MAX;
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
  float thermC = 103.21 - 34.37 * (float)log(thermKohms);

  // Calulate rate from thermC. Linear. 40C -> 25%, 70C -> 100%
  //uint16_t rateFromTherm = (uint16_t)(thermC * 25.6) - 768;
  int16_t signedRateFromTherm = (int16_t)(thermC * 102.4) - 2560;
  uint16_t rateFromTherm;
  if (signedRateFromTherm < 256) {
    rateFromTherm = 0;
  } else if (signedRateFromTherm > 1024) {
    rateFromTherm = 1024;
  } else {
    rateFromTherm = (uint16_t)signedRateFromTherm;
  }

  static bool thermOverride = false;
  if (thermOverride) {
    if (rateFromTherm == 0 || rateFromTherm < rateFromEc) {
      thermOverride = false;
    }
  } else if (rateFromTherm >= 256) {
    if (rateFromTherm > (rateFromEc + 50)) {
      thermOverride = true;
    }
  }

  if (thermOverride) {
    // Set fan speed from thermRequiredPwm and send fake tack values to EC
    Timer1.pwm(TO_FAN_PWM_PIN, rateFromTherm);
    if (rateFromEc == 0) {
      ecTackOutInterval = 0;
    } else {
      const uint32_t C = 24094; // ((1 / 2.72) << 16)
      ecTackOutInterval = TACK_MIN_25K + (uint16_t)(((PWM_MAX - rateFromEc) * C) >> 16);     
    }
    digitalWrite(LED_PIN, 1);
  } else {
    // Set fan speed from ecPwm and send tack from fan to EC.
    Timer1.pwm(TO_FAN_PWM_PIN, rateFromEc);
    ecTackOutInterval = TACK_PASS_THROUGH;
    digitalWrite(LED_PIN, 0);
  }

  Serial.print(rateFromEc, DEC);
  Serial.print('\t');
  Serial.print(thermC, 3);
  Serial.print('\t');
  Serial.print(rateFromTherm, DEC);
  Serial.print('\t');
  Serial.print(ecTackOutInterval, DEC);
  Serial.print('\t');
  Serial.print(localFanUsPerRev, DEC);
  Serial.println();
}

void timer25kHz() {
  // Counts from 0 to 24999
  static uint16_t timer25kHzCounter = 0;
  // Counts from 0 to ecTackOutInterval
  static uint16_t ecTackOutCounter = 0;

  timer25kHzCounter++;
  if (timer25kHzCounter >= 2500) {
    timer25kHzCounter = 0;
    doEveryDecasecond = true;
  }

  if (ecTackOutInterval > 0) {
    ecTackOutCounter++;
    if (ecTackOutCounter >= ecTackOutInterval) {
      ecTackOutCounter = 0;
      doEcTackOutToggle = true;
    } 
  }
}

// Will be called between 0 and 140 times a second.
void fanTackPinIsr() {
  static uint32_t lastMicros = UINT32_MAX;
  uint32_t currentMicros = micros();

  if (currentMicros > lastMicros) {
    fanUsPerRev = (currentMicros - lastMicros) << 2;
    fanUsPerRevValid = true;  
  }
  lastMicros = currentMicros;
  
  if (ecTackOutInterval == TACK_PASS_THROUGH) {
    doEcTackOutToggle = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("M6700 4 Wire Fan Controller Hack v0.1");
  
  pinMode(FROM_FAN_TACK_PIN, INPUT_PULLUP);
  
  pinMode(TO_FAN_PWM_PIN, OUTPUT);
  digitalWrite(TO_FAN_PWM_PIN, 0);

  pinMode(FROM_EC_PWM_PIN, INPUT);
  //pinMode(FROM_EC_PWM_PIN, OUTPUT);
  //digitalWrite(FROM_EC_PWM_PIN, 0);
  pinMode(FROM_EC_PWM_REF_PIN, OUTPUT);
  digitalWrite(FROM_EC_PWM_REF_PIN, 0);
  pinMode(FROM_EC_PWM_ADC_PIN, INPUT);
  
  pinMode(TO_EC_TACK_PIN, OUTPUT);
  digitalWrite(TO_EC_TACK_PIN, 0);

  pinMode(THERM_IN_PIN, INPUT);
  pinMode(THERM_ENABLE_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  Timer1.initialize(40); // 25khz PWM for fan.
  Timer1.pwm(TO_FAN_PWM_PIN, 0);
  Timer1.attachInterrupt(timer25kHz);

  attachInterrupt(digitalPinToInterrupt(FROM_FAN_TACK_PIN), fanTackPinIsr, CHANGE);
}
 
void loop() {
  static bool ecTackOutValue = 0;
  if (doEveryDecasecond) {
    doEveryDecasecond = false;
    everyDecasecond();
  } else if (doEcTackOutToggle) {
    doEcTackOutToggle = false;
    ecTackOutValue = !ecTackOutValue;
    digitalWrite(TO_EC_TACK_PIN, ecTackOutValue);
  } else if (Serial.available()) {
    int c = Serial.read();
    if (c >= '0' && c <= '8') {
        int pwmDuty = (c - '0') * 128;
        Timer1.pwm(SPARE_PWM_PIN, pwmDuty);
    }
  } else {
    delay(1);
  }
}
