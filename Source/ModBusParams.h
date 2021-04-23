#ifndef MODBUSPARAMS_H
#define MODBUSPARAMS_H

#include "mainconfig.h"

typedef struct {
  uint16_t adc[10];  // �� ����������� �������� ���������� ������
  uint16_t sig1,sig2;// ������� � ������� ���������������� (A)
  uint16_t cels;     // ����������� ����������
  uint16_t upit;     // ���������� ������� ����������
  uint8_t ledstate;
} RAMInfo;

extern RAMInfo info;

#endif
