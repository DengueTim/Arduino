#include "avr/interrupt.h"

volatile uint16_t adcEcGpuPwm = 0;
volatile uint16_t adcEcCpuPwm = 0;
volatile uint16_t adcTherm1 = 0;

ISR(ADC_vect) {
  switch(ADMUX & 0b0111) {
    case 0b0100:
      adcEcGpuPwm = (ADCL | (ADCH << 8));
      ADMUX = _BV(REFS0) | 0b0101;
      ADCSRA |= _BV(ADSC);
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
      break;
   }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Free rUunning ADC test.");
  pinMode(11, OUTPUT);
  digitalWrite(11, 1);
  
  DIDR0 = 0b01110000;
}

void loop() {
  ADMUX = _BV(REFS0) | 0b0100;
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS0)| _BV(ADPS1) | _BV(ADPS2);
  
  delay(1000);

  Serial.println(adcEcGpuPwm, DEC);
  Serial.println(adcEcCpuPwm, DEC);
  Serial.println(adcTherm1, DEC);
  Serial.println();
}  
