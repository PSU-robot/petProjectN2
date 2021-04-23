// ������������ ������������ USART1 ����������� STM32F051
// ������ RB6-RB8
// ������������ CCP2 ������� 3 ��� ��������� ��������� �����������, ������ ������ ���� �������� �� �������
// ������, � ������������� 15 �� ������� 8���
// ������� ������������ - 8���
// ���������� DMA �� ����� � �� ��������

#ifndef FPU1H_CONFIG_H
#define FPU1H_CONFIG_H

#define FPU1H_RXPIN GPIOB,7       // ���� ���������
#define FPU1H_TXPIN GPIOB,6       // ���� �����������
#define FPU1H_DEPIN GPIOB,8       // ���� ������������ �����/��������
#define FPU1H_AFNUM 0             // ����� �������������� �������, ��������������� UART
#define FPU1H_USART USART1        // ����, � ������� ���������� �����
#define FPU1H_FREQ  16000000      // ������� ������ UART � ������� ��������� ��������� �����������
#define FPU1H_TRDMA DMA1_Channel2 // DMA �������� UART
#define FPU1H_TRDMASTATE DMA1,2   // ����� DMA �������� UART (DMA � ����� ������)
#define FPU1H_RXDMA DMA1_Channel3 // DMA ������ UART
#define FPU1H_UARTINTFUNC USART1_IRQHandler // ���������� ���������� USART
#define FPU1H_USART_IRQn USART1_IRQn // ���������� ���������� � NVIC
#define FPU1H_MBSIZE 128          // ������ �������������� ������ ������

// ������, ������������ ����������� ��������� �����������
// ����������� ������ ����� �������� �� 16 ����� ������ ��������� ����������� �������� � BaudRate
#define FPU1H_TIMER TIM3
#define FPU1H_TIMEROC TIM3->CCR2
#define FPU1H_CC_FLAG TIM_CC2IE

#endif
