#include <Arduino.h>

#define BAUD_RATE (115200)
#define SAMPLE_RATE (4096)

volatile unsigned long last_fetch;
volatile unsigned long elapsed;

void TC0_Handler(void) {
  NVIC_DisableIRQ(TC0_IRQn);
  if (TC0->COUNT16.INTFLAG.bit.MC0) {
    TC0->COUNT16.INTFLAG.bit.MC0 = 1;
    elapsed = micros() - last_fetch;
    last_fetch = micros();
  }
  NVIC_EnableIRQ(TC0_IRQn);
}

void init_GCLK() {
  GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
}

void init_TC() {
  TC0->COUNT16.CTRLA.bit.ENABLE = 0;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
  TC0->COUNT16.CTRLA.bit.ENABLE = 1;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.INTENSET.bit.MC0 = 1;
}

void set_sampling_rate(uint32_t sampling_rate) {
  uint32_t timer_delay = SystemCoreClock / sampling_rate - 1;
  TC0->COUNT16.CTRLA.bit.ENABLE = 0;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.CC[0].reg = timer_delay;
  while(TC0->COUNT16.SYNCBUSY.bit.CC0);
  TC0->COUNT16.CTRLA.bit.ENABLE = 1;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
}

void setup(void) {
  Serial.begin(BAUD_RATE);
  while(!Serial);
  init_GCLK();
  delay(100);
  init_TC();
  set_sampling_rate(SAMPLE_RATE);
  NVIC_EnableIRQ(TC0_IRQn);
}

void loop(void) {
  delay(1000);
  Serial.printf("%lu\n", elapsed);
}