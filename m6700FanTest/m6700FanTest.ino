
#include <math.h>
#include "avr/interrupt.h"
#include "TimerOne.h"

#define FAN_CPU_PWM_PIN       9
#define FAN_CPU_TACK_PIN      2

#define FAN_GPU_PWM_PIN       10
#define FAN_GPU_TACK_PIN      3

#define EC_CPU_PWM_PIN        16 // A2
#define EC_CPU_PWM_ADC        5 // A5 // 19
#define EC_CPU_TACK_PIN       17 // A3

#define EC_GPU_PWM_PIN        15 // A1
#define EC_GPU_PWM_ADC        4 // A4 // 18
#define EC_GPU_TACK_PIN       14 // A0

#define THERM_ENABLE_PIN      11
#define THERM_ADC             6 // A6
#define THERM_R_OHMS         4700

#define LED_PIN               13

#define PWM_MAX               1024
#define PWM_MIN_CUTOFF        256

#define ANALOG_MAX            1024

volatile uint16_t adcEcGpuPwm = 0;
volatile uint16_t adcEcCpuPwm = 0;
volatile uint16_t adcTherm1 = 0;
volatile boolean adcSweepComplete = false;

void timer27kHz() {
  static uint16_t counter = 0;
  if (++counter >= 270) {
    // About 100Hz
    counter = 0;

    // Read the last EC PWM values and update the tack intervals
    ADMUX = _BV(REFS0) | 0b0100;
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS0)| _BV(ADPS1) | _BV(ADPS2);
  } 
}

ISR(ADC_vect) {
  switch(ADMUX & 0b0111) {
    case 0b0100:
      adcEcGpuPwm = (ADCL | (ADCH << 8));
      ADMUX = _BV(REFS0) | 0b0101;
      ADCSRA |= _BV(ADSC);
      digitalWrite(THERM_ENABLE_PIN, 1);
      break;
    case 0b0101:
      adcEcCpuPwm = (ADCL | (ADCH << 8));
      ADMUX = _BV(REFS0) | 0b0110;
      ADCSRA |= _BV(ADSC);
      break;
    case 0b0110:
      adcTherm1 = (ADCL | (ADCH << 8));
      ADMUX = _BV(REFS0) | 0b0100;
      // Stop the ADC
      ADCSRA &= ~_BV(ADEN);
      digitalWrite(THERM_ENABLE_PIN, 0);
      adcSweepComplete = true;
      break;
   }
}

// Will be called between 0 and 140 times a second.
void fanCpuTackPinIsr() {
}

// Will be called between 0 and 140 times a second.
void fanGpuTackPinIsr() {
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

  pinMode(EC_CPU_TACK_PIN, OUTPUT);
  digitalWrite(EC_CPU_TACK_PIN, 0);

  pinMode(EC_GPU_TACK_PIN, OUTPUT);
  digitalWrite(EC_GPU_TACK_PIN, 0);

  //pinMode(THERM_ADC_PIN, INPUT);
  pinMode(THERM_ENABLE_PIN, OUTPUT);
  digitalWrite(THERM_ENABLE_PIN, 0);
    
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1);

  // Disable digital circuits for pins used for analog input
  DIDR0 = 0b01110110;

  Timer1.initialize(37); // 27.027khz for PWM to fans.
  Timer1.pwm(FAN_CPU_PWM_PIN, 1024); // %100 to start
  Timer1.pwm(FAN_GPU_PWM_PIN, 1024); // %100 to start
  Timer1.attachInterrupt(timer27kHz);

  attachInterrupt(digitalPinToInterrupt(FAN_CPU_TACK_PIN), fanCpuTackPinIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FAN_GPU_TACK_PIN), fanGpuTackPinIsr, CHANGE);
}

#define EC_TACK_INTERVAL_STOP   0
#define EC_TACK_START_INTERVAL  16088

#define EC_TACK_ACCEL_RATE 120
#define EC_TACK_FREE_RATE 30

void updateEcTackInterval(uint16_t ecPwmAdc, uint16_t *currentInterval) {
  if (ecPwmAdc >= 256) {
    if (*currentInterval == 0) {
      // Start
      *currentInterval = EC_TACK_START_INTERVAL;
    } else {
      uint16_t targetInterval = EC_TACK_START_INTERVAL - (ecPwmAdc - 256) * 16;     
      if (targetInterval < *currentInterval) {
        // Speed up
        uint16_t di = *currentInterval - targetInterval;
        *currentInterval -= (di > EC_TACK_ACCEL_RATE ? EC_TACK_ACCEL_RATE : di);  
      } else if (targetInterval > *currentInterval) {
        // Slow down
        uint16_t di = targetInterval - *currentInterval;
        *currentInterval += (di > EC_TACK_ACCEL_RATE ? EC_TACK_ACCEL_RATE : di);
      }
    }
  } else if (*currentInterval) {
    // Stop
    *currentInterval += EC_TACK_FREE_RATE;
    if (*currentInterval > EC_TACK_START_INTERVAL) {
      *currentInterval = EC_TACK_INTERVAL_STOP;
    }
  }
}

uint16_t centegrateFromThermAdc(uint32_t thermAdc) {
  uint32_t thermOhms = (THERM_R_OHMS * thermAdc) / (ANALOG_MAX - thermAdc); // Rt / (R + Rt)
  /* 
   *  Kohms   temp C    ADC
   *  20      0         829
   *  10.7    22.5      711
   *  6.74     37       603
   *  1.1    100        194
   *  
   * Function from logorithmic regression (* 64).  
   */
  return (21804 - (uint16_t)(2200 * (float)log(thermOhms))) >> 6;
}

void updateFanSpeeds(uint8_t tempC) {
  // Calulate rate from temp. Linear. 40C -> 25%, 70C -> 100%
  //uint16_t rateFromTemp = (uint16_t)(tempC * 25.6) - 768;
  int16_t signedRateFromTemp = (int16_t)(tempC * 102.4) - 2560;
  uint16_t rateFromTemp;
  if (signedRateFromTemp < 256) {
    rateFromTemp = 0;
  } else if (signedRateFromTemp > 1024) {
    rateFromTemp = 1024;
  } else {
    rateFromTemp = (uint16_t)signedRateFromTemp;
  }
  
  static bool cpuRateOverride = false;
  if (cpuRateOverride) {
    cpuRateOverride = rateFromTemp && rateFromTemp > adcEcCpuPwm;
  } else {
    cpuRateOverride = rateFromTemp && rateFromTemp > (adcEcCpuPwm + 50);
  }
  Timer1.pwm(FAN_CPU_PWM_PIN, cpuRateOverride ? rateFromTemp : adcEcCpuPwm);
   
  static bool gpuRateOverride = false;
  if (gpuRateOverride) {
    gpuRateOverride = rateFromTemp && rateFromTemp > adcEcGpuPwm;
  } else {
    gpuRateOverride = rateFromTemp && rateFromTemp > (adcEcGpuPwm + 50);
  }
  Timer1.pwm(FAN_GPU_PWM_PIN, gpuRateOverride ? rateFromTemp : adcEcGpuPwm);
  
  digitalWrite(LED_PIN, cpuRateOverride | gpuRateOverride);
  
  static uint8_t counter = 0;
  if (counter++ >= 50) {
    counter = 0;
    Serial.print(adcEcCpuPwm, DEC);
    Serial.print(':');
    Serial.print(adcEcGpuPwm, DEC);
    Serial.print('\t');
    Serial.print(tempC, DEC);
    Serial.print(':');
    Serial.print(rateFromTemp, DEC);
    Serial.println(); 
  }
}

void loop() {
  uint32_t t = micros();

  /* 
  Fake the tack output to the EC for CPU and GPU fans
  The values are set depending on the EC PWM inputs updated after every ADC sweep @ ~100hz

  Interval between transitions of tack to EC in microseconds.  4 transitions per rotation.
  Working values are 
    3700 4054rpm 100% 
    16088 923rpm ~ 23%
    Stopped -> 0 
  */
  static uint16_t ecCpuTackInterval = EC_TACK_INTERVAL_STOP;
  static uint32_t ecCpuTackLast = t;
  static bool ecCpuTackOutValue = 0;

  static uint16_t ecGpuTackInterval = EC_TACK_INTERVAL_STOP;
  static uint32_t ecGpuTackLast = t;
  static bool ecGpuTackOutValue = 0;

  if (adcSweepComplete) {
    adcSweepComplete = false;
    updateEcTackInterval(adcEcCpuPwm, &ecCpuTackInterval);
    updateEcTackInterval(adcEcGpuPwm, &ecGpuTackInterval);
    uint8_t gpuTempC = centegrateFromThermAdc(adcTherm1);
    updateFanSpeeds(gpuTempC);
  }

  if (ecCpuTackInterval != EC_TACK_INTERVAL_STOP && (t - ecCpuTackLast > ecCpuTackInterval)) {
    ecCpuTackLast += ecCpuTackInterval;
    ecCpuTackOutValue = !ecCpuTackOutValue;
    digitalWrite(EC_CPU_TACK_PIN, ecCpuTackOutValue);
  } else {
    ecCpuTackLast = t;
  }
  
  if (ecGpuTackInterval != EC_TACK_INTERVAL_STOP && (t - ecGpuTackLast > ecGpuTackInterval)) {
    ecGpuTackLast += ecGpuTackInterval;
    ecGpuTackOutValue = !ecGpuTackOutValue;
    digitalWrite(EC_GPU_TACK_PIN, ecGpuTackOutValue);
  } else {
    ecGpuTackLast = t;
  }

  if (Serial.available()) {
    int c = Serial.read();
    if (c >= '0' && c <= '8') {
        int pwmDuty = (c - '0') * 128;
        Timer1.pwm(FAN_GPU_PWM_PIN, pwmDuty);
    }
  }
}
