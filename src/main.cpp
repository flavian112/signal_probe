#include <Arduino.h>

#define BAUD_RATE (115200)
#define SAMPLING_RATE (1000)

volatile unsigned long last_fetch;
volatile unsigned long elapsed;

void TC0_Handler(void) {
  NVIC_DisableIRQ(TC0_IRQn);
  if (TC0->COUNT16.INTFLAG.bit.MC0) {
    TC0->COUNT16.INTFLAG.bit.MC0 = 1;
    //elapsed = micros() - last_fetch;
    //last_fetch = micros();
  }
  NVIC_EnableIRQ(TC0_IRQn);
}

void init_GCLK(void) {
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0;
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0;
  GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
  GCLK->PCHCTRL[EVSYS_GCLK_ID_0].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
}

void init_TC(void) {
  TC0->COUNT16.CTRLA.bit.ENABLE = 0;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
  TC0->COUNT16.EVCTRL.bit.MCEO0 = 1;
  TC0->COUNT16.CTRLA.bit.ENABLE = 1;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.INTENSET.bit.MC0 = 1;
  NVIC_EnableIRQ(TC0_IRQn);
}

void set_sampling_rate(uint32_t sampling_rate) {
  uint16_t timer_delay = SystemCoreClock / sampling_rate - 1;
  TC0->COUNT16.CTRLA.bit.ENABLE = 0;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.CC[0].reg = timer_delay;
  while(TC0->COUNT16.SYNCBUSY.bit.CC0);
  TC0->COUNT16.CTRLA.bit.ENABLE = 1;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
}

void init_EVSYS(void) {
  EVSYS->USER[EVSYS_ID_USER_ADC0_START].bit.CHANNEL = 1;
  EVSYS->Channel[0].CHANNEL.bit.PATH = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val;
  EVSYS->Channel[0].CHANNEL.bit.EVGEN = EVSYS_ID_GEN_TC0_MCX_0;
}

void ADC0_1_Handler(void) {
  NVIC_DisableIRQ(ADC0_1_IRQn);
  if (ADC0->INTFLAG.bit.RESRDY) {
    ADC0->INTFLAG.bit.RESRDY = 1;
    elapsed = micros() - last_fetch;
    last_fetch = micros();
  }
  NVIC_EnableIRQ(ADC0_1_IRQn);
}

void init_ADC(void) {
  ADC0->EVCTRL.bit.STARTEI = 1;
  ADC0->EVCTRL.bit.RESRDYEO = 1;
  ADC0->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
  while(ADC0->SYNCBUSY.bit.INPUTCTRL);
  ADC0->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN0_Val;
  while(ADC0->SYNCBUSY.bit.INPUTCTRL);
  ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
  while(ADC0->SYNCBUSY.bit.REFCTRL);
  ADC0->CTRLA.bit.ENABLE = 1;
  while(ADC0->SYNCBUSY.bit.ENABLE);
  ADC0->INTENSET.bit.RESRDY = 1;
  NVIC_EnableIRQ(ADC0_1_IRQn);
}

void setup(void) {
  Serial.begin(BAUD_RATE);
  while(!Serial);
  init_GCLK();
  init_TC();
  set_sampling_rate(SAMPLING_RATE);
  init_EVSYS();
  init_ADC();
}

void loop(void) {
  delay(1000);
  Serial.printf("%lu\n", elapsed);
}