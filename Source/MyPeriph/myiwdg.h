#ifndef MYIWDG_H
#define MYIWDG_H

#include "mainconfig.h"

// �������� - ������� ������ ������������, �� ���� 2 � �� ����� 8
void InitIWDG(uint8_t psc);
void StartIWDG(void);

#define CLRWDT() {IWDG->KR = 0xAAAA;}

#endif
