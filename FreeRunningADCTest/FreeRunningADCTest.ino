#include "avr/interrupt.h"

uint16_t scount = 10;
const uint8_t adcInputs[3] = { 0b0101, 0b0100, 0b0110 };
uint16_t adcValues[3] = { 0, 0, 0 };

ISR(ADC_vect) {
  uint16_t reading = ADCL | (ADCH << 8);

  static uint8_t adcInputIndex = 255;
  if (adcInputIndex == 255) {
    // Ignore first read.
    adcInputIndex = 0;
    return;
  }

  // Read each pin 4 times. The impedance of the added components is to high.
  if (adcInputIndex & 0b11 == 0b11) {
    adcValues[adcInputIndex >> 2] = reading;
  }
  adcInputIndex++;
  if (adcInputIndex >= (3 << 2)) {
    adcInputIndex = 0;
  }
  ADMUX &= 0b11111000;
  ADMUX |= adcInputs[adcInputIndex >> 2];
  scount++;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Free rUunning ADC test.");
  pinMode(11, OUTPUT);
  digitalWrite(11, 1);
  
  DIDR0 = 0b01110000;
  ADMUX = _BV(REFS0) | adcInputs[0];
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS0)| _BV(ADPS1) | _BV(ADPS2);
  ADCSRB = 0; // free-running - all ADTS bits cleared
}

void loop() {
  delay(500);

  // put your main code here, to run repeatedly:
  Serial.println(scount);
  Serial.println(adcValues[0]);
  Serial.println(adcValues[1]);
  Serial.println(adcValues[2]);
}  
