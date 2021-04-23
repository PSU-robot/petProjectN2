#ifndef MYGPIO_H
#define MYGPIO_H

#include "mainconfig.h"

// ������� ���������������� �����. ����� ������ �������� �������, �� 0 �� 15. 
// ��������������� �� ����� �����
// ����� ���������� �� ��������� � ������������

#define GPIO_SPEED_HI    0x00002000
#define GPIO_SPEED_MED   0x00001000
#define GPIO_SPEED_LO    0x00000000   // ����� �� ���������
#define GPIO_PULLUP      0x00000100
#define GPIO_PULLDOWN    0x00000200
#define GPIO_NOPULL      0x00000000   // ����� �� ���������
#define GPIO_OD          0x00000010
#define GPIO_PUSHPULL    0x00000000   // ����� �� ���������

void GSetPinToInput(GPIO_TypeDef* GPIOx,uint16_t Pin,uint32_t Flags);
void GSetPinToOutput(GPIO_TypeDef* GPIOx,uint16_t Pin,uint8_t StartState,uint32_t Flags);
void GSetPinToAnalog(GPIO_TypeDef* GPIOx,uint16_t Pin);
void GSetPinToAF(GPIO_TypeDef* GPIOx,uint16_t Pin,uint8_t AFnum,uint32_t Flags);

// � �������� ���� ����� �������� �������, �� 0 �� 15
void GSetPin(GPIO_TypeDef* GPIOx,uint16_t Pin);
void GResetPin(GPIO_TypeDef* GPIOx,uint16_t Pin);
void GOutToPin(GPIO_TypeDef* GPIOx,uint16_t Pin,uint8_t State);
uint8_t GGetPin(GPIO_TypeDef* GPIOx,uint16_t Pin);

// � �������� ���� ����� �������� ������, ��� � SDL
void GSetPins(GPIO_TypeDef* GPIOx,uint16_t Pin);
void GResetPins(GPIO_TypeDef* GPIOx,uint16_t Pin);
void GOutToPins(GPIO_TypeDef* GPIOx,uint16_t Pin,uint8_t State);


#endif
