#include <Arduino.h>
#include <arduinoFFT.h>

#define DATA_BUF_SIZE (32)
#define SAMPLE_RATE (25000) 

DmacDescriptor dmac_section_desc[1] __attribute__ ((aligned (8)));
DmacDescriptor dmac_section_desc_wrb[1] __attribute__ ((aligned (8)));
DmacDescriptor dmac_desc[1] __attribute__ ((aligned (8)));

DmacDescriptor *adc_dmac_desc[2] = { &dmac_section_desc[0], &dmac_desc[0] };

uint16_t adc_buf[DATA_BUF_SIZE << 1];
volatile uint16_t *adc_buf_active = &adc_buf[DATA_BUF_SIZE];


void dmac_handler(void) {
  __disable_irq();
  if (DMAC->Channel[0].CHINTFLAG.bit.TCMPL) {
    DMAC->Channel[0].CHINTFLAG.bit.TCMPL = 1;
    adc_buf_active = adc_buf_active == adc_buf ? &adc_buf[DATA_BUF_SIZE] : adc_buf;
  }
  __enable_irq();
}

void init_PORT() {
  //ADC (AIN0), PIN (PA02), Arduino (A0)
  PORT->Group[PORTA].PMUX[2 >> 1].bit.PMUXE = 1;
  PORT->Group[PORTA].PINCFG[2].bit.PMUXEN = 1;
}















const uint16_t number_of_samples = 1 << 12;
const float sampling_frequency = 5000;
const uint8_t amplitude = 100;

float vReal[number_of_samples];
float vImag[number_of_samples];

ArduinoFFT<float> fft(vReal, vImag, number_of_samples, sampling_frequency);

void fourier(void) {
    time_t start = micros();
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();
    time_t elapsed = micros() - start;
    Serial.println(elapsed);
}

void setup(void) {
    Serial.begin(115200);
    delay(10000);
    fourier();
}

void loop(void) {
    
}