#include "myadc051.h"

// ���� ��������� ADC_SR � ADC_IER
#define ADCSR_OVRIF  0x0010     // ������������ ���
#define ADCSR_EOSEQ  0x0008     // ��������� ������������������
#define ADCSR_EOC    0x0004     // ��������� ���������
#define ADCSR_EOS    0x0002     // ��������� ������� ������������
#define ADCSR_ADCRDY 0x0001     // ��� ����� � ������

// ���� �������� ADC_CR
#define ADCCR_ADCAL  0x80000000 // ���������� (������, ���������)
#define ADCCR_STOP   0x00000010 // �������������� ���������
#define ADCCR_START  0x00000004 // ������
#define ADCCR_ADDIS  0x00000002 // ���������� ���
#define ADCCR_ADEN   0x00000001 // ��������� ���

// ���� �������� ADC_CFGR1
#define ADCCFG1_DISCEN 0x00010000  // ����������� ����� ��� ����, ����������� ��� 1
#define ADCCFG1_AUTOFF 0x00008000  // �������������� ����������
#define ADCCFG1_CONT   0x00002000  // 1 - ����������� ���������, 0 - ����������� ���������
#define ADCCFG1_OVRMOD 0x00001000  // 1 - �� ��������� ��������� ��� ������������
#define ADCCFG1_ALIGN  0x00000020  // ������������ 1-�����
#define ADCCFG1_RES(N) (N<<3)      // ���������� � ����� 0-12, 1-10, 2-8 3-6
#define ADCCFG1_SCANDR 0x00000004  // ����������� ������������
#define ADCCFG1_DMACFG 0x00000002  // 1-DMA circular, 0 - one shot DMA
#define ADCCFG1_DMAEN  0x00000001  // ���������� ������� DMA

// ���� ������ �������� ��� ������� ADC051Common
#define ADCCOM_VBATEN  0x01000000  // ��������� ��������� ���������
#define ADCCOM_TSEN    0x00800000  // ��������� �����������
#define ADCCOM_VREFEN  0x00400000  // ��������� ������� ����������

// ��������� ������� ��������� ����������, �����������, Vref
void ADC051Common(uint32_t flags) {
  ADC->CCR=flags;
}


void ADC051DeInit(void) {
  RCC->APB2RSTR|=RCC_APB2RSTR_ADC1RST;
  RCC->APB2RSTR&=~RCC_APB2RSTR_ADC1RST;
}

// ��������� ����������
// ���� ��� �������� - ������������� ��� 
void ADC051StartCal(ADC_TypeDef * adc) {
  if (adc->CR&ADCCR_ADEN) {
    adc->CR|=ADCCR_ADDIS;
    while (adc->CR&ADCCR_ADEN);
  }
  adc->CR|=ADCCR_ADCAL;
}

// ��������� ���
// �������������� ��������� ��� ���� �������, ����� ��������� ���������� �� �� ���!
// ������ �������� - ������������� ���
// ������ �������� - ������� ������ �� ��������, 0..7
// ������ �������� - �������� �������� ������������
// ��������� �������� - ����������� ���������� (������� ADC_IER)
// ��������� ����������� ��� � ����� ��������� ����������
void ADC051Init(ADC_TypeDef * adc,uint32_t samptime,uint32_t cfgr1,uint32_t intst) {
// ������� ���������� ���������� ���� ����
  while (adc->CR&ADCCR_ADCAL);
  adc->CR|=ADCCR_ADEN;
// ���������
  adc->CFGR1=cfgr1;
  adc->CFGR2=0;
  adc->IER=intst;
  adc->SMPR=samptime;
}

void ADC051SelCh(ADC_TypeDef * adc,uint32_t channels) {
  adc->CHSELR=channels;
}

// ������ ���������
void ADC051Start(ADC_TypeDef * adc) {
  adc->CR|=ADCCR_START;
};

// ���������, ���������� � ������
#define VREFINT_CAL (*((uint16_t*)(0x1FFFF7BA)))    // �������� ADC Vref ��� 30* � 3.3V
#define TS_CAL1     (*((uint16_t*)(0x1FFFF7B8)))    // �������� ADC TS ��� 30* � 3.3V
#define TS_CAL2     (*((uint16_t*)(0x1FFFF7C2)))    // �������� ADC TS ��� 110* � 3.3V

// ������� �������
// ������ �������� ���������� ������� � ������������ �� ����������� VREF
uint32_t GetUpwr(uint16_t vref) {
  uint32_t tmp;
  tmp=33000*VREFINT_CAL;
  tmp/=vref;
  return tmp*100;
}


// ������ ���������� � ���������� � ����������� VREF
// ���������� �������� � ������������
uint32_t CalcU(uint16_t vref,uint16_t adczn) {
  uint32_t upit;
  upit=GetUpwr(vref)/4096;
  return upit*adczn;
};

// ������������ �������� ����������� � 0.1 ��������
int32_t CalcT(uint16_t vref,uint32_t tszn) {
  int32_t A25,KA;
  int32_t tmp;
// ������������� tszn � 30 �������� � 3.3� �� vref
  tszn*=VREFINT_CAL;
  tszn/=vref;
// �������� �������� Asens
  KA=((int32_t)TS_CAL1-TS_CAL2)*100;
  KA/=80;
  A25=5*KA+TS_CAL1*100;
  tmp=(A25*10-1000*tszn);
  tmp=tmp/KA+250;
  return tmp;
};

