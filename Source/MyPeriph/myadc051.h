#ifndef MYADC051_H
#define MYADC051_H
//// ��� ��������� ������������ ���
// 1. ��������� ������������ ��� - RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
// 2. ��������� ���������� ��� - ADC051StartCal. ������� ���������� ��� ������� �����
// 3. ��������� ��� (�� ������������ ���������) - ADC051Init
// 4. ��������� ������ ��� - ADC051SelCh
// 5. ��������� DMA � �������� ��� (���� ������������)
// 6. ��������� ��� - ADC051Start
// 7. ���� ������� ������ ��� ������ - ADC_DeInit � ������������ � ������ 2

#include "mainconfig.h"

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
#define ADCCFG1_EXTSEL(A) (A<<6)   // �������� �������
#define ADCCFG1_EXTEN(A) (A<<10)   // ������ �������


// ���� ������ �������� ��� ������� ADC051Common
#define ADCCOM_VBATEN  0x01000000  // ��������� ��������� ���������
#define ADCCOM_TSEN    0x00800000  // ��������� �����������
#define ADCCOM_VREFEN  0x00400000  // ��������� ������� ����������

// ��������� ������� ��������� ����������, �����������, Vref
void ADC051Common(uint32_t flags);

// ����� ���
void ADC051DeInit(void);
void ADC051StartCal(ADC_TypeDef * adc);
// ��������� ���
// �������������� �������� ���
// ������ �������� - ������������� ���
// ������ �������� - ������� ������
// ������ �������� - �������� �������� ������������
// ��������� �������� - ����������� ���������� (������� ADC_IER)
void ADC051Init(ADC_TypeDef *,uint32_t samptime,uint32_t cfgr1,uint32_t intst);

// ���������� �������, ���������
#define ADCCH_VBAT 0x00040000   // ���������
#define ADCCH_TEMP 0x00010000   // �����������
#define ADCCH_VREF 0x00020000   // ������� ����������
void ADC051SelCh(ADC_TypeDef *,uint32_t channels); 

void ADC051Start(ADC_TypeDef *);// ������ ���������

// ���������, ���������� � ������
#define VREFINT_CAL (*((uint16_t*)(0x1FFFF7BA)))    // �������� ADC Vref ��� 30* � 3.3V
#define TS_CAL1     (*((uint16_t*)(0x1FFFF7B8)))    // �������� ADC TS ��� 30* � 3.3V
#define TS_CAL2     (*((uint16_t*)(0x1FFFF7C2)))    // �������� ADC TS ��� 110* � 3.3V

// ������� �������
// ������� ������� � ���������� � ����������� VREF
// ���������� �������� � ������������
uint32_t GetUpwr(uint16_t vref); // ������ �������� ���������� �������
uint32_t CalcU(uint16_t vref,uint16_t adczn);    // ������ ���������� � ���������� � ����������� VREF
// ������������ �������� ����������� � 0.1 ��������
int32_t CalcT(uint16_t vref,uint32_t tszn); 

#endif
