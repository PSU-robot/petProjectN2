#ifndef SOURCE_MYSETTINGS_H_
#define SOURCE_MYSETTINGS_H_

#include "mainconfig.h"

extern uint32_t hssavetime;

void ReinitFromHardDSt(uint8_t soft);
uint8_t CheckSaveHardDSt(void);

typedef struct {
  uint8_t mbaddr;     // ����� � ���� modbus (�� ��������� 1)
  uint16_t param1,param2; // ��� ���������� ���������
} MySettings;

extern MySettings HardDSt;    // ����� � Flash - � ��� �������� ��
extern MySettings DSt;        // ����� � RAM - ���� ���� ������ � ������ �� ����������

// ��� ����������� ������� � ModBus
extern uint16_t RescueState;

#endif
