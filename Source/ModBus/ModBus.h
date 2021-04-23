#ifndef SOFTWARE_SOURCE_MODBUS_MODBUS_H_
#define SOFTWARE_SOURCE_MODBUS_MODBUS_H_

#include "mainconfig.h"

// ������� �������� ������ � ModBus
void InitModBus(uint16_t mbaddr,uint16_t mbconn);
// ������� ������ �� modBus
// �������� � �������� �����
// ���������� �� ����, ���� ���� ��������� HardDSt
uint32_t ModBusProc(void);  

// �������� ��������� � ������� ���������� ����������
typedef struct {
  uint16_t reg;  // ����� �������� (� 0)
  void * addr;   // ����� � ������
  uint32_t flag; // �����
} MbStruct;

// ����� ����������, ��������� �� ModBus
// ���� ������
#define MBT_FMASK   0x0F
#define MBT_INT8    0x00  // ����������� �� uint16, ������������ � ����������� �� ������ ��� uint8
#define MBT_INT16   0x01
#define MBT_INT32   0x02
#define MBT_FLOAT   0x04
// ������������ ����������� ������
#define MBT_WRITIBLE  0x0800   // �������� �� ������, ������
#define MBT_AWRITIBLE 0x0400   // �������� �� ������, � ���������������� ������
#define MBT_SWRITIBLE 0x0200   // �������� �� ������, � ������ ������������
#define MBT_WRITEMASK (MBT_SWRITIBLE|MBT_AWRITIBLE|MBT_WRITIBLE)   // ����� ������ �� ������
#define MBT_EESAVE   0x4000   // ���������� EEPROM ����� ������
#define MBT_CHECKSAV 0x2000   // ��������� ������� MBCanWrite ��� ����������� ����������� ������

// ������� ����������. ������ ������������� ������� 0,0,0
extern MbStruct mbs[];

// ���������� ������� ������� �������
// 0 - ������� (MBT_WRITIBLE)
// 1 - ���������������� (MBT_AWRITIBLE)
// 3 - ���������������  (MBT_SWRITIBLE)
uint8_t MBCanWrite(void);

// �������������� ������� - ������ CRC. ������������ ��� �������� ������ ������
uint16_t CalcCRC(uint8_t * curbuf,int16_t cnt);

#endif
