// ��� ������������� DMA
// ��������� InitDMACh, ��������� ������� �� ����������� ����. ����� ����� ������� ����������� DMA,
// ���� ��������� StartDMA �������
#ifndef MYDMA_H
#define MYDMA_H

#include "mainconfig.h"

// ���� �������� DAC->CR
#define DAC_DMAEN   (1<<12)
#define DAC_TRGTIM6 (0)
#define DAC_TRGEN   (1<<2)
#define DAC_BUFEN   (1<<1)
#define DAC_ON      (1<<0)

//// ����������� DMA

// ������� DMAx_Channely
#define DMA_M2M             (1<<14)     // ����� "������ � ������"
#define DMA_PL(lev)         ((lev&0x03)<<12)     // ������� ���������� - �� 0 �� 3, 3 - ����� ��������
#define DMA_MEMSIZE8BIT     (0<<10)     // ������������ ����� � ������
#define DMA_MEMSIZE16BIT    (1<<10)     // ����������������� ����� � ������
#define DMA_MEMSIZE32BIT    (2<<10)     // ������������������ ����� � ������
#define DMA_PRHSIZE8BIT     (0<<8)      // ������������ ����� � ���������
#define DMA_PRHSIZE16BIT    (1<<8)      // ����������������� ����� � ���������
#define DMA_PRHSIZE32BIT    (2<<8)      // ������������������ ����� � ���������
#define DMA_MINC            (1<<7)      // ����������� ����� � ������
#define DMA_PINC            (1<<6)      // ����������� ����� � ���������
#define DMA_CIRC            (1<<5)      // ��������� �����
#define DMA_MEMSRC          (1<<4)      // �������� �� ������ � ���������
#define DMA_TEIE            (1<<3)      // ���������� ��� ������ ��������
#define DMA_HCIE            (1<<2)      // ���������� ��� ���������� ��������
#define DMA_TCIE            (1<<1)      // ���������� ��� ��������� ��������
#define DMA_EN              (1<<0)      // ���������� ������ DMA

// ��������� ������ DMA
// ������ �������� - ����� DMA
// ������ �������� - �����������, ���������� �� ����������� ����
// ������ � ��������� ��������� - ������ � ��������� � ������
// ����� �������� - ���������� ������������ ������
void InitDMACh(DMA_Channel_TypeDef * dmach,uint32_t dmacfg,uint32_t periph,uint32_t memory,uint32_t count);
void StartDMA(DMA_Channel_TypeDef * dmach);
void StopDMA(DMA_Channel_TypeDef * dmach);
// ���������� ������ � ������
uint32_t GetCNDTR(DMA_Channel_TypeDef * dmach);
// �������� � ������ ����� ����������
void ResetDMAIF(DMA_TypeDef * dma,uint8_t channel,uint32_t flag);
uint8_t CheckDMAIF(DMA_TypeDef * dma,uint8_t channel,uint32_t flag);

// ���� ������ ���������� - � ����� ��� ���� ������� �������� DMAx->ISR
// � ��������� - ����� ������ �� 1 �� x)
#define DMA_TCIF(IFNUM) ((0x0002)<<(4*(IFNUM-1)))
#define DMA_CGIF(IFNUM) ((0x0001)<<(4*(IFNUM-1)))


#endif
