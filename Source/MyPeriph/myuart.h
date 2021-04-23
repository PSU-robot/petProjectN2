//////////////////////////////////////////////////////////////
// ������ 0.1
//
#ifndef MYUART_H
#define MYUART_H

#include "mainconfig.h"

// ���� �������� CR1
#define UART_BITS_DEAT 0x03E00000 // ���� ��������� DEAT
#define UART_DEAT(A)   (A<<21)    // ��������� ��������� DEAT
#define UART_BITS_DEDT 0x001F0000 // ���� ��������� DEDT
#define UART_DEDT(A)   (A<<16)    // ��������� ��������� DEDT
#define UART_9BIT  0x00001000     // �������� 9 ��� (�������� ��� �������� ��������
#define UART_RTOIE 0x04000000     // ���������� ���������� �� �������� ������ CR1
#define UART_CMIE  0x00004000     // ���������� ���������� �� ������ �����    CR1
#define UART_PCE   0x00000400     // ���������� �������� ��������
#define UART_ODD   0x00000200     // ODD �������� ��������
#define UART_PEIE  0x00000100     // ���������� ���������� �� ��������
#define UART_TXEIE 0x00000080     // ���������� ���������� �� ����������� ������
#define UART_TCIE  0x00000040     // ���������� ���������� �� ��������� ��������
#define UART_RXNEIE 0x00000020    // ���������� ���������� ��� ������������ ��� ������ �����
#define UART_TE    0x00000008     // ���������� �����������
#define UART_RE    0x00000004     // ���������� ���������
#define UART_UE    0x00000001     // ���������� ������ USART

// ���� �������� CR2
#define UART_2STOP  (0x02<<12)    // ��� �������� ����
// ���� �������� CR3
#define UART_OVRDIS 0x00001000    // ���������� ������� ������������ ������
#define UART_DMAT   0x00000080    // ���������� DMA ��� ��������
#define UART_DMAR   0x00000040    // ���������� DMA ��� ������
#define UART_EIE    0x00000001    // ���������� ��������� ���������� ��� ������ (����� ORE, FE, NF)

// ���� �������� CR3
#define UART_ONEBIT 0x00000800    // �������� �������� �� ������ ���������
#define UART_DEP    0x00008000    // ���������� ������� DE
#define UART_DEM    0x00004000    // ���������� ������� DE


// ���� ��������� ISR � ICR
// ����������� ���������� � �������� CR1, ��������� �������� - CR2
//
#define UART_IR_TXE  0x00000080     // ���� ������� ������ ��������
#define UART_IR_TC   0x00000040     // ���� � ���������� ��������� ��������
#define UART_IR_RXNE 0x00000020     // ����� ������ �� ����
#define UART_IR_ORE  0x00000008     // ������������ ������ ������
#define UART_IR_NF   0x00000004     // ���
#define UART_IR_FE   0x00000002     // ������ �����
#define UART_IR_PE   0x00000001     // ������ ��������

// ������������ ����� USART - ��� ������������ �������
// speed - �������� � �����
// parity - ��������, 0-N, 1-E, 2-O, +128 - 2 �������� ����
// USARTFreq - ������� USART, ������� �� ����, ���� ��� ������������ =1, �� ����� ������������ SystemCoreClock
void InitUSART(USART_TypeDef * USARTx,uint32_t speed,uint8_t parity,uint32_t USARTFreq);
void InitHardDE(USART_TypeDef * USARTx);  // ��������� ����������� ������ ������/��������

// �������� ������ � ������, ������������� � CR1
void USART_EnableIRQ(USART_TypeDef * USARTx,uint32_t flg,uint32_t enbit);
void USART_DisableIRQ(USART_TypeDef * USARTx,uint32_t flg,uint32_t enbit);

#endif
