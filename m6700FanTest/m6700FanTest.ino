
#include <math.h>
#include "avr/interrupt.h"
#include "TimerOne.h"

#define FAN_CPU_PWM_PIN       9
#define FAN_CPU_TACK_PIN      2

#define FAN_GPU_PWM_PIN       10
#define FAN_GPU_TACK_PIN      3

#define EC_CPU_PWM_GND_PIN    18 // A4
#define EC_CPU_PWM_ADC_PIN    A5 // 19
#define EC_CPU_TACK_PIN       15 // A1

#define EC_GPU_PWM_GND_PIN    16 // A2
#define EC_GPU_PWM_ADC_PIN    A3 // 17
#define EC_GPU_TACK_PIN       14 // A0

#define THERM_ENABLE_PIN      11
#define THERM_IN_PIN          A6
#define THERM_R_KOHMS         4.7

#define LED_PIN               13

#define TACK_PASS_THROUGH     -1
#define TACK_STOP             0

#define TACK_MIN_25K          93
#define TACK_MAX_25K          375

#define PWM_MAX               1024
#define PWM_MIN_CUTOFF        256

#define ANALOG_MAX            1024

uint16_t adcPinValues[3] = { 0, 0, 0 };
const uint8_t nextAdcPin[3] = { _BV(ADLAR) | 0b0001, _BV(ADLAR) | 0b0010, _BV(ADLAR) | 0b0011 };


volatile bool fanCpuUsPerRevValid = false;
volatile uint32_t fanCpuUsPerRev = UINT32_MAX; // Stopped

volatile bool fanGpuUsPerRevValid = false;
volatile uint32_t fanGpuUsPerRev = UINT32_MAX; // Stopped

// Desired time between transitions of tack to EC in 25,0000th of a second.
// Working values are 93(~4000rpm 100%) to 375(~1000rpm 25%)
// 0 Stopped
// -1 copy tack value from real fan every millisecond  
volatile uint16_t ecCpuTackOutInterval = TACK_PASS_THROUGH;
volatile uint16_t ecGpuTackOutInterval = TACK_PASS_THROUGH;

volatile bool doEveryDecasecond = false;
volatile bool doEcCpuTackOutToggle = false;
volatile bool doEcGpuTackOutToggle = false;

void everyDecasecond() {
  uint32_t localFanCpuUsPerRev;
  if (fanCpuUsPerRevValid) {
    fanCpuUsPerRevValid = false;
    localFanCpuUsPerRev = fanCpuUsPerRev;
  } else {
    // No tack transitions, assume stopped.
    localFanCpuUsPerRev = UINT32_MAX;
  }
  
  // Read EC CPU_PWM
  uint16_t rateFromEcCpu = analogRead(EC_CPU_PWM_ADC_PIN);
  if (rateFromEcCpu < 256) {
    rateFromEcCpu = 0;
  } else if (rateFromEcCpu > (ANALOG_MAX - 32)) {
    rateFromEcCpu = PWM_MAX;
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
    if (rateFromTherm == 0 || rateFromTherm < rateFromEcCpu) {
      thermOverride = false;
    }
  } else if (rateFromTherm >= 256) {
    if (rateFromTherm > (rateFromEcCpu + 50)) {
      thermOverride = true;
    }
  }

  if (thermOverride) {
    // Set fan speed from thermRequiredPwm and send fake tack values to EC
    Timer1.pwm(FAN_CPU_PWM_PIN, rateFromTherm);
    if (rateFromEcCpu == 0) {
      ecCpuTackOutInterval = 0;
    } else {
      const uint32_t C = 24094; // ((1 / 2.72) << 16)
      ecCpuTackOutInterval = TACK_MIN_25K + (uint16_t)(((PWM_MAX - rateFromEcCpu) * C) >> 16);     
    }
    digitalWrite(LED_PIN, 1);
  } else {
    // Set fan speed from ecPwm and send tack from fan to EC.
    Timer1.pwm(FAN_CPU_PWM_PIN, rateFromEcCpu);
    ecCpuTackOutInterval = TACK_PASS_THROUGH;
    digitalWrite(LED_PIN, 0);
  }

  Serial.print(rateFromEcCpu, DEC);
  Serial.print('\t');
  Serial.print(thermC, 3);
  Serial.print('\t');
  Serial.print(rateFromTherm, DEC);
  Serial.print('\t');
  Serial.print(ecCpuTackOutInterval, DEC);
  Serial.print('\t');
  Serial.print(localFanCpuUsPerRev, DEC);
  Serial.println();
}

void timer25kHz() {
  // Counts from 0 to 24999
  static uint16_t timer25kHzCounter = 0;
  // Counts from 0 to ecTackOutInterval
  static uint16_t ecCpuTackOutCounter = 0;

  timer25kHzCounter++;
  if (timer25kHzCounter >= 2500) {
    timer25kHzCounter = 0;
    doEveryDecasecond = true;
  }

  if (ecCpuTackOutInterval > 0) {
    ecCpuTackOutCounter++;
    if (ecCpuTackOutCounter >= ecCpuTackOutInterval) {
      ecCpuTackOutCounter = 0;
      doEcCpuTackOutToggle = true;
    } 
  }
}

// Will be called between 0 and 140 times a second.
void fanCpuTackPinIsr() {
  static uint32_t lastMicros = UINT32_MAX;
  uint32_t currentMicros = micros();

  if (currentMicros > lastMicros) {
    fanCpuUsPerRev = (currentMicros - lastMicros) << 2;
    fanCpuUsPerRevValid = true;  
  }
  lastMicros = currentMicros;
  
  if (ecCpuTackOutInterval == TACK_PASS_THROUGH) {
    doEcCpuTackOutToggle = true;
  }
}

// Will be called between 0 and 140 times a second.
void fanGpuTackPinIsr() {
  static uint32_t lastMicros = UINT32_MAX;
  uint32_t currentMicros = micros();

  if (currentMicros > lastMicros) {
    fanCpuUsPerRev = (currentMicros - lastMicros) << 2;
    fanCpuUsPerRevValid = true;  
  }
  lastMicros = currentMicros;
  
  if (ecCpuTackOutInterval == TACK_PASS_THROUGH) {
    doEcCpuTackOutToggle = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("M6700 4 Wire Fan Controller Hack v0.1");
  
  pinMode(FAN_CPU_TACK_PIN, INPUT_PULLUP);
  pinMode(FAN_CPU_PWM_PIN, OUTPUT);
  digitalWrite(FAN_CPU_PWM_PIN, 0);

  pinMode(FAN_GPU_TACK_PIN, INPUT_PULLUP);
  pinMode(FAN_GPU_PWM_PIN, OUTPUT);
  digitalWrite(FAN_GPU_PWM_PIN, 0);

  pinMode(EC_CPU_PWM_ADC_PIN, INPUT);
  pinMode(EC_CPU_PWM_GND_PIN, OUTPUT);
  digitalWrite(EC_CPU_PWM_GND_PIN, 0);
  pinMode(EC_CPU_TACK_PIN, OUTPUT);
  digitalWrite(EC_CPU_TACK_PIN, 0);

  pinMode(EC_GPU_PWM_ADC_PIN, INPUT);
  pinMode(EC_GPU_PWM_GND_PIN, OUTPUT);
  digitalWrite(EC_GPU_PWM_GND_PIN, 0);
  pinMode(EC_GPU_TACK_PIN, OUTPUT);
  digitalWrite(EC_GPU_TACK_PIN, 0);

  pinMode(THERM_IN_PIN, INPUT);
  pinMode(THERM_ENABLE_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  Timer1.initialize(40); // 25khz PWM for fan.
  Timer1.pwm(FAN_CPU_PWM_PIN, 0);
  Timer1.pwm(FAN_GPU_PWM_PIN, 512); // 50% for test
  Timer1.attachInterrupt(timer25kHz);

  attachInterrupt(digitalPinToInterrupt(FAN_CPU_TACK_PIN), fanCpuTackPinIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FAN_GPU_TACK_PIN), fanGpuTackPinIsr, CHANGE);

}
 
void loop() {
  static bool ecCpuTackOutValue = 0;
  if (doEveryDecasecond) {
    doEveryDecasecond = false;
    everyDecasecond();
  } else if (doEcCpuTackOutToggle) {
    doEcCpuTackOutToggle = false;
    ecCpuTackOutValue = !ecCpuTackOutValue;
    digitalWrite(EC_CPU_TACK_PIN, ecCpuTackOutValue);
  } else if (Serial.available()) {
    int c = Serial.read();
    if (c >= '0' && c <= '8') {
        int pwmDuty = (c - '0') * 128;
        Timer1.pwm(FAN_GPU_PWM_PIN, pwmDuty);
    }
  } else {
    delay(1);
  }
}
