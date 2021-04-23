// ������������ ������������ USART2 ����������� STM32F051
// ������ RA2-RA4
// ������������ CCP2 ������� 3 ��� ��������� ��������� �����������, ������ ������ ���� �������� �� �������
// ������, � ������������� 15 �� ������� 8���
// ������� ������������ - 8���
// ���������� DMA �� ����� � �� ��������

#ifndef FPU2H_CONFIG_H
#define FPU2H_CONFIG_H

#define FPU2H_RXPIN GPIOA,3       // ���� ���������
#define FPU2H_TXPIN GPIOA,2       // ���� �����������
#define FPU2H_DEPIN GPIOA,1       // ���� ������������ �����/��������
#define FPU2H_AFNUM 1             // ����� �������������� �������, ��������������� UART
#define FPU2H_USART USART2        // ����, � ������� ���������� �����
#define FPU2H_FREQ  16000000      // ������� ������ UART � ������� ��������� ��������� �����������
#define FPU2H_TRDMA DMA1_Channel4 // DMA �������� UART
#define FPU2H_TRDMASTATE DMA1,4   // ����� DMA �������� UART (DMA � ����� ������)
#define FPU2H_RXDMA DMA1_Channel5 // DMA ������ UART
#define FPU2H_UARTINTFUNC USART2_IRQHandler // ���������� ���������� USART
#define FPU2H_USART_IRQn USART2_IRQn // ���������� ���������� � NVIC
#define FPU2H_MBSIZE 128          // ������ �������������� ������ ������

// ������, ������������ ����������� ��������� �����������
// ����������� ������ ����� �������� �� 16 ����� ������ ��������� ����������� �������� � BaudRate
#define FPU2H_TIMER TIM3
#define FPU2H_TIMEROC TIM3->CCR2
#define FPU2H_CC_FLAG TIM_CC2IE

#endif
