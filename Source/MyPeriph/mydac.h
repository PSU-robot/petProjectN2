#ifndef MYDAC_H
#define MYDAC_H

#include "mainconfig.h"

////// ���� �������� DAC_CR
#define DAC1CH2_UDRIE    (1<<29)       // ���������� ��� ������ ��������� ������ DAC �� DMA 
#define DAC1CH2_DMAEN    (1<<28)       // ���������� DMA ��� ������ 2 DAC
#define DAC1CH2_TRGSOFT  (0x7<<19)     // ����������� �������
#define DAC1CH2_TEN      (1<<18)       // ���������� �������� ���
#define DAC1CH2_BOFF     (1<<17)       // ���������� ��������� ������
#define DAC1CH2_EN       (1<<16)       // ��������� ���
#define DAC1CH1_UDRIE    (1<<13)       // ���������� ��� ������ ��������� ������ DAC �� DMA 
#define DAC1CH1_DMAEN    (1<<12)       // ���������� DMA ��� ������ 2 DAC
#define DAC1CH1_TRGSOFT  (0x7<<3)     // ����������� �������
#define DAC1CH1_TEN      (1<<2)       // ���������� �������� ���
#define DAC1CH1_BOFF     (1<<1)       // ���������� ��������� ������
#define DAC1CH1_EN       (1<<0)       // ��������� ���

void InitDAC(uint32_t cfg);
void SetDAC1(uint32_t zn);    //  ���������� 16-������ ��������
void SetDAC2(uint32_t zn);    //  ���������� 16-������ ��������

#endif
