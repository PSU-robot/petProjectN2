// ��� ������ � �������� ��������� InitTimer, ���:
// �������� CR1, ������ ����������. 
// ���������� � �������� PSC ������������ (Fsck/PSC+1)
// ���������� �������� ����� � �������� ARR. ������ �������� ����� ���������, ����� �������� ������ ��������� � ����
// ���������� �������� CR2, DIER, ��������� - CR1 � ���������� �����

#ifndef MYTIMER_H
#define MYTIMER_H

#include "mainconfig.h"

// ���� �������� TIMx->CR1
#define TIM_ON      (1<<0)

// ���� �������� TIMx->CR2
#define TIM_MMS_RESET   (0x00<<4)   // TRGO ��������� � ���� UG
#define TIM_MMS_ENABLE  (0x01<<4)   // TRGO ��������� � ������� ���������� ������ �������
#define TIM_MMS_UPDATE  (0x02<<4)   // TRGO ��������� � ������� ������������ �������

// ���� ��������� TIMx->DIER � TIMx->SR
#define TIM_UDE         (1<<8)      // ������ DMA ��� ������������ �������
#define TIM_CC3IE       (1<<3)      // ���� ���������� ���������� �� CC2  (��������� � ����� � SR)
#define TIM_CC2IE       (1<<2)      // ���� ���������� ���������� �� CC2  (��������� � ����� � SR)
#define TIM_CC1IE       (1<<1)      // ���� ���������� ���������� �� CC1  (��������� � ����� � SR)
#define TIM_UIE         (1<<0)      // ������ ���������� ��� ������������ �������

// ���� ����������� ������� ���������/������� TIMx->CCMRx (������� ��������)
#define TIM_CCMR_OCxCE  (1<<7)      
#define TIM_CCMR_OCxMODE0 (0<<4)   // Frozen - ������ ���������  (����� �� ���������)
#define TIM_CCMR_OCxMODE1 (1<<4)   // ��������� ������ OCREF � 1 ��� ����������
#define TIM_CCMR_OCxMODE2 (2<<4)   // ��������� ������ OCREF � 0 ��� ����������
#define TIM_CCMR_OCxMODE3 (3<<4)   // ������������ OCREF ��� ����������
#define TIM_CCMR_OCxMODE4 (4<<4)   // OCREF=0
#define TIM_CCMR_OCxMODE5 (5<<4)   // OCREF=1
#define TIM_CCMR_OCxMODE6 (6<<4)   // PWM Mode1
#define TIM_CCMR_OCxMODE7 (7<<4)   // PWM Mode2
#define TIM_CCMR_OCxPE    (1<<3)   // ������� ������� ����� CCR
#define TIM_CCMR_OCxFE    (1<<2)   // ������� ����� ����� ���������
#define TIM_CCMR_CCxS0    (0)      // ����� ��������������� ��� �����
#define TIM_CCMR_CCxS1    (1)      // ����� ��������������� ��� ����, � TI1
#define TIM_CCMR_CCxS2    (2)      // ����� ��������������� ��� ����, � TI2
#define TIM_CCMR_CCxS3    (3)      // ����� ��������������� ��� ����, � TRC


// ������������� �������
void InitTimer(TIM_TypeDef*TIMx,uint32_t psc,uint32_t cr1,uint32_t cr2,uint32_t dier);
// ������������� ������ ���������
void InitOC(TIM_TypeDef*TIMx,uint8_t occhnum,uint32_t flags);
// ������ � ��������� �������
void StartTimer(TIM_TypeDef*TIMx);
void StopTimer(TIM_TypeDef*TIMx);

#endif
