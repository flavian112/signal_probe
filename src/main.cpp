#include <Arduino.h>

#define BAUD_RATE (115200)
#define SAMPLING_RATE (10)
#define SAMPLING_BUFFER_SIZE (16)

DmacDescriptor dmac_section_desc[1] __attribute__ ((aligned (8)));
DmacDescriptor dmac_section_desc_wrb[1] __attribute__ ((aligned (8)));
DmacDescriptor dmac_desc[1] __attribute__ ((aligned (8)));

DmacDescriptor *adc_dmac_desc[2] = { &dmac_section_desc[0], &dmac_desc[0] };

uint16_t adc_buf[SAMPLING_BUFFER_SIZE << 1];
volatile uint16_t *adc_buf_active = &adc_buf[SAMPLING_BUFFER_SIZE];
volatile boolean adc_buf_used = false;


void TC0_Handler(void) {
  NVIC_DisableIRQ(TC0_IRQn);
  if (TC0->COUNT16.INTFLAG.bit.MC0) {
    TC0->COUNT16.INTFLAG.bit.MC0 = 1;
  }
  NVIC_EnableIRQ(TC0_IRQn);
}

void init_CLK(void) {
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0;
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0;
  MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
  GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
  GCLK->PCHCTRL[EVSYS_GCLK_ID_0].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
}

void init_TC(void) {
  TC0->COUNT16.CTRLA.bit.ENABLE = 0;
  while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
  TC0->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV256_Val;
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
  }
  NVIC_EnableIRQ(ADC0_1_IRQn);
}

void init_ADC(void) {
  PORT->Group[PORTA].PMUX[PIN_PA02 >> 1].bit.PMUXE = 0x01;
  PORT->Group[PORTA].PINCFG[PIN_PA02].reg = PORT_PINCFG_PMUXEN;
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

void DMAC_0_Handler(void) {
  NVIC_DisableIRQ(DMAC_0_IRQn);
  if(DMAC->Channel[0].CHINTFLAG.bit.TCMPL) {
    DMAC->Channel[0].CHINTFLAG.bit.TCMPL = 1;
    adc_buf_active = adc_buf_active == adc_buf ? &adc_buf[SAMPLING_BUFFER_SIZE] : adc_buf;
    adc_buf_used = false;
  }
  NVIC_EnableIRQ(DMAC_0_IRQn);
}

void init_DMA(void) {
  DMAC->CTRL.bit.DMAENABLE = 0;
  DMAC->CRCCTRL.bit.CRCSRC = 0;
  while (DMAC->CTRL.bit.DMAENABLE);
  DMAC->CTRL.bit.SWRST = 1;
  while (DMAC->CTRL.bit.SWRST);
  DMAC->BASEADDR.reg = (uint32_t) dmac_section_desc;
  DMAC->WRBADDR.reg = (uint32_t) dmac_section_desc_wrb;
  DMAC->CTRL.bit.LVLEN0 = 1;
  DMAC->CTRL.bit.LVLEN1 = 1;
  DMAC->CTRL.bit.LVLEN2 = 1;
  DMAC->CTRL.bit.LVLEN3 = 1;
  DMAC->CTRL.bit.DMAENABLE = 1;
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 0;
	while(DMAC->Channel[0].CHCTRLA.bit.ENABLE);
	DMAC->Channel[0].CHCTRLA.bit.SWRST = 1;
	while(DMAC->Channel[0].CHCTRLA.bit.SWRST);
  DMAC->Channel[0].CHINTENSET.bit.TCMPL = 1;
  DMAC->Channel[0].CHCTRLA.bit.BURSTLEN = DMAC_CHCTRLA_BURSTLEN_SINGLE_Val;
  DMAC->Channel[0].CHCTRLA.bit.TRIGACT = DMAC_CHCTRLA_TRIGACT_BURST_Val;
  DMAC->Channel[0].CHCTRLA.bit.TRIGSRC = ADC0_DMAC_ID_RESRDY;

  adc_dmac_desc[0]->SRCADDR.reg = (uint32_t)  &ADC0->RESULT.reg;
  adc_dmac_desc[0]->DSTADDR.reg = ((uint32_t) &adc_buf[SAMPLING_BUFFER_SIZE]);
  adc_dmac_desc[0]->DESCADDR.reg = (uint32_t) adc_dmac_desc[1];
  adc_dmac_desc[0]->BTCNT.reg = SAMPLING_BUFFER_SIZE;
  adc_dmac_desc[0]->BTCTRL.reg = DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID;

  adc_dmac_desc[1]->SRCADDR.reg = (uint32_t) &ADC0->RESULT.reg;
  adc_dmac_desc[1]->DSTADDR.reg = ((uint32_t) &adc_buf[SAMPLING_BUFFER_SIZE << 1]);
  adc_dmac_desc[1]->DESCADDR.reg = (uint32_t) adc_dmac_desc[0];
  adc_dmac_desc[1]->BTCNT.reg = SAMPLING_BUFFER_SIZE;
  adc_dmac_desc[1]->BTCTRL.reg = DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID; 

  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;
  NVIC_EnableIRQ(DMAC_0_IRQn);
}


void setup(void) {
  Serial.begin(BAUD_RATE);
  while(!Serial);
  init_CLK();
  init_TC();
  set_sampling_rate(SAMPLING_RATE);
  init_EVSYS();
  init_ADC();
  init_DMA();
}

uint16_t buf[SAMPLING_BUFFER_SIZE];

void loop(void) {
  if (!adc_buf_used) {
    adc_buf_used = true;
    memcpy((void*)buf, (void*)adc_buf_active, SAMPLING_BUFFER_SIZE * sizeof(uint16_t));
    for (unsigned int i = 0; i < SAMPLING_BUFFER_SIZE; ++i) {
      Serial.printf("%hu ", buf[i]);
    }
    Serial.printf("\n");
  }
}
