
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

#define THERM_R_KOHMS         4.7

#define TACK_PASS_THROUGH     -1
#define TACK_STOP             0

volatile int fanTackCount = 0;

volatile int ecTackOutInterval = TACK_PASS_THROUGH;

volatile bool doEverySecond = false;
volatile bool doEcTackOutToggle = false;

void everySecond() {
  static int seconds = 0;
  static int fanRpm = 0;
  seconds++;
  // Compute fanRpm
  fanRpm = fanTackCount * (60 / 4); // Two ticks per rev. Two changes per tick
  fanTackCount = 0;

  // Read EC PWM
  int ecPwm = analogRead(FROM_EC_PWM_ADC_PIN);
  if (ecPwm < 256) {
    ecPwm = 0;
  } else if (ecPwm > (1024 - 64)) {
    ecPwm = 1024;
  }
  Serial.print("EC PWM:");
  Serial.println(ecPwm, DEC);
  
  // Read temp.
  digitalWrite(THERM_ENABLE_PIN, 1);
  int thermRaw = analogRead(THERM_IN_PIN);
  digitalWrite(THERM_ENABLE_PIN, 0);
  float thermKohms = (THERM_R_KOHMS * thermRaw) / (1024 - thermRaw); // Rt / (R + Rt)
    /* Thermister ln curve from
   *  Kohms   temp C
   *  20      0
   *  10.7    22.5
   *  6.74     37
   *  1.1    100
  */
  float thermC = 103.21 - 34.37 * (float)log(thermKohms);
  Serial.print("thermC:");
  Serial.println(thermC, 3);

  // Calculate the required fan PWM duty(0-1024) given thermC
  int thermRequiredPwm = 0;

  if (thermRequiredPwm > ecPwm) {
    // Set fan speed from thermRequiredPwm and send fake tack values to EC
    
    ecTackOutInterval = 
  } else {
    // Set fan speed from ecPwm and send tack from fan to EC.
    Timer1.pwm(TO_FAN_PWM_PIN, ecPwm);
    ecTackOutInterval = TACK_PASS_THROUGH;
  }
}

void timer25kHz() {
  // Counts from 0 to 24999
  static int timer25kHzCounter = 0;
  // Counts from 0 to ecTackOutInterval
  static int ecTackOutCounter = 0;

  timer25kHzCounter++;
  if (timer25kHzCounter >= 25000) {
    timer25kHzCounter = 0;
    doEverySecond = true;
  }

  if (ecTackOutInterval > 0) {
    ecTackOutCounter++;
    if (ecTackOutCounter > ecTackOutInterval) {
      ecTackOutCounter = 1;
      doEcTackOutToggle = true;
    } 
  }
}

// Will be called between 0 and 140 times a second.
void fanTackPinIsr() {
  fanTackCount++;
  if (ecTackOutInterval == TACK_PASS_THROUGH) {
    doEcTackOutToggle = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("4 Wire Fan Controller Hack v0.1");
  
  pinMode(FROM_FAN_TACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FROM_FAN_TACK_PIN), fanTackPinIsr, CHANGE);
  
  pinMode(TO_FAN_PWM_PIN, OUTPUT);
  digitalWrite(TO_FAN_PWM_PIN, 0);
  Timer1.initialize(40); // 25khz PWM for fan.
  Timer1.pwm(TO_FAN_PWM_PIN, 0);
  Timer1.attachInterrupt(timer25kHz);

  pinMode(FROM_EC_PWM_PIN, INPUT_PULLUP);
  pinMode(FROM_EC_PWM_REF_PIN, OUTPUT);
  digitalWrite(FROM_EC_PWM_REF_PIN, 0);
  pinMode(FROM_EC_PWM_ADC_PIN, INPUT);
  
  pinMode(TO_EC_TACK_PIN, OUTPUT);
  digitalWrite(TO_EC_TACK_PIN, 0);

  pinMode(THERM_IN_PIN, INPUT);
  pinMode(THERM_ENABLE_PIN, OUTPUT);
}
 
void loop() {
  static bool ecTackOutValue = 0;
  if (doEverySecond) {
    doEverySecond = false;
    everySecond();
  } else if (doEcTackOutToggle) {
    doEcTackOutToggle = false;
    ecTackOutValue = !ecTackOutValue;
    digitalWrite(TO_EC_TACK_PIN, ecTackOutValue);
  } else {
    delay(1);
  }
}
