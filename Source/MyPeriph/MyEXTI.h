#ifndef MYEXTI_H
#define MYEXTI_H

#include "mainconfig.h"

// ��������� ���� ����������
#define EI_TRIGGER_FALLING       1
#define EI_TRIGGER_RISING        2
#define EI_TRIGGER_BOTH          3

void EXTIDisable(uint8_t num);                       // ��������� �����
void EXTIEnableInterrupt(uint8_t num,uint8_t type);  // ����������� ����� ��� ����������
void EXTIClearPending(uint8_t num);                  // �������� ���� ����������
uint8_t EXTICheckPending(uint8_t num);               // ��������� ���� ����������
void EXTIPortConnect(GPIO_TypeDef* GPIOx,uint16_t Pin); // ������������� SYSCFG ��������� ����� � �����������

#endif
