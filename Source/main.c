#include "mainconfig.h"
#include "mygpio.h"
#include "myiwdg.h"
#include "ssec.h"
#include "mysettings.h"
#include "ModBus.h"
#include "ModBusParams.h"
#include "mydac.h"
#include "myadc051.h"
#include "mydma.h"
#include "stdbool.h"

// �������� �� 16��� �� ����������� RC ����������. �������� ������� -3.8..+5.5% �� ���� ��������� ����������
// ��������!!! ������ �������� RCC->CR
void SysInit(void) {
  RCC->CR |= 0x00000001;     // �������� HSI
  while (!(RCC->CR&0x02));  // ������� HSI (�� ������ ������)
// �������� �� ������
  RCC->CR &= 0x0000FFF8;
  RCC->CFGR=0;  // ��������� �� HSI � ��� ������������ - �� ����
// ��������� PLL - �������� ����� �� 16 ���
  RCC->CFGR2=1;  // � ��� ����� ������������ ����� PLL
  RCC->CFGR3=0;
  RCC->CFGR|=0x00080000;  // ����� �� 2, �������� �� 4, �������� - HSI
  RCC->CR |= 0x01000000;  // ��������� PLL
  while (!RCC->CR&0x02000000);        // ������� ��������� PLL. ���� �� ��������� - ���������� ������ ��� ��������
  FLASH->ACR = 0x00000010;  // Enable Prefetch Buffer and set Flash Latency
  RCC->CFGR|=0x00000002;    // �������� PLL ��� �������� ������������
  while (!((RCC->CFGR&0x0C)==(0x02<<2)));  // ������� �������������
// 
  RCC->CIR = 0x00000000;    // ��������� ��� ����������
}

void ConfigHW() {
// ������� ���������� � ���� ��� �����, ����� ��� ����� ������������
  RCC->AHBENR|=RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN|RCC_AHBENR_GPIODEN|RCC_AHBENR_GPIOFEN;
  RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;     // ������ ���������� ��������� ����������� 1�� � ������������ - �������� UART
  RCC->AHBENR|=RCC_AHBENR_DMAEN;        // ������ ������ � ������ - ����� ���� �������� UART
  RCC->APB2ENR|=RCC_APB2ENR_USART1EN;   // UART - ���������������� ����, �� �������� ����� ������ RS485 ModBus
  RCC->APB2ENR|=RCC_APB2ENR_SYSCFGCOMPEN;// ��������� ������� (���� ��� �������� �� �����)
  RCC->APB2ENR|=RCC_APB2ENR_ADCEN;      // ��� ��� ��������� ���������� �������� � �������� � ������� �������� � ��������� ��������
  RCC->APB1ENR|=RCC_APB1ENR_DACEN;      // ��� ��� ������������ �������� ������ ��� �������� �������� � ������
// ������ ����������� ������  
// ������
//
// ����������
  GSetPinToOutput(LED1,0,0);
  GSetPinToOutput(LED2,0,0);
  GSetPinToOutput(LED3,0,0);
	GSetPinToOutput(BLUELED,0,0);
	GSetPinToOutput(REDLED,0,0);
	GSetPinToOutput(GREENLED,0,0);
	GSetPinToInput(BUTTON,GPIO_PULLUP);
// ����� ��� ��������
  GSetPinToAnalog(AN1);
  GSetPinToAnalog(AN2);
  GSetPinToAnalog(AN3);
  GSetPinToAnalog(AN4);
  GSetPinToAnalog(AN5);
  GSetPinToAnalog(AN6);
  GSetPinToAnalog(AN7);
  GSetPinToAnalog(AN8);
  GSetPinToAnalog(DACPIN);
// �������� ��� � ������ ������� ������� - ����� �������� ��� ���� ��������
  InitDAC(DAC1CH1_EN);
  SetDAC1(32768);    //  ������ �������� - 65536, �������� - 32768
// ����������� ���
  ADC051DeInit();
// �������� ������� ���������� ��� � ���������� �����������
  ADC051Common(ADCCOM_TSEN|ADCCOM_VREFEN);
  ADC051StartCal(ADC1);
  ADC051SelCh(ADC1,0x000340EF);  // ����� ��������, ����� ������ ����� ����������
  ADC051Init(ADC1,0x03,ADCCFG1_OVRMOD|ADCCFG1_DMAEN,0);
// ������� ��������� ����������
  while (ADC1->CR&ADCCR_ADCAL) CLRWDT();
// ������ ����� �������� � ��� ��������� DMA
//
// ���������������� ���� RS485 ������������� � ����� fpuart1.c ����� ������ ������� InitModBus -> InitFPU1
// ������ ��������� SSEC
  InitSSEC();
  __enable_irq();
}

// ��� ���������� �� ���� ������ ��������
uint16_t adcdat[10]; 
// ���� ������� ���������� ��������
uint8_t adcrezflg=0;

// ��������� ��������� ���������� ������ ������
void StartIzm(void) {
// �������� � ���
  StopDMA(DMA1_Channel1);
  InitDMACh(DMA1_Channel1,DMA_MEMSIZE16BIT|DMA_MINC|DMA_PRHSIZE16BIT|DMA_CIRC|DMA_TCIE,(uint32_t)(&(ADC1->DR)),(uint32_t)(&(adcdat[0])),10);    
// ��������� ���������� �� DMA
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);    
  StartDMA(DMA1_Channel1);
  adcrezflg=0;
  ADC1->CR|=ADCCR_START;
}        

// ���������� ���������� �� DMA ���
void DMA1_Channel1_IRQHandler(void) {
  DMA1->IFCR=DMA_TCIF(1);  // ���������� ���� ����������
  adcrezflg=1;             // ������������� ���� ��� ��������� �����
}
/*
void sleep(uint32_t milliseconds1){
	uint32_t current = getssec();
	while (tdlt(current)<milliseconds1);
	
	
	
	
	uint32_t current1 = getssec();
	do {
			current1 = getssec();
      
    } while (current1-current<milliseconds1);
} */
////////////////////////////////////////////////////////////
// �������� ���������
// ��� ������� ������� �� �������������, ������ ��� �������� ������ - �� � �����������
int main() {
 uint32_t ledtime1=getssec();
	uint32_t lighttime=getssec();
  uint32_t ledtimeR=getssec();
	uint32_t ledtimeG=getssec();
	uint32_t ledtimeB=getssec();
	int counter = 0 ;
	bool flag = true ; //  ��������
	bool fire = false; // ������ �� ������
	int  litghtType =0; // ����� ������ ���������
////////////////////////////////////////////////////
// ������������� ���������� ����� �����������  
  SysInit();   // ��������� ���������� �� ������ ��� ������� (������ - 16���)
#ifndef MYDEBUG
  InitIWDG(4);
#endif
  ConfigHW();  // ������������� ���������� ����� ����������� (������)
// ��������� ������������ �� ������ �����������. �� ����� ���� ���������� ������ ����������� ���� � ���������
// �� � ����������� ��� ���� ����� ����������� ������. ���� �����-�� �� ����� ���������� - ����������� ��������� ��������������.
  ReinitFromHardDSt(0);
// ��������� �������������� - 
// ���� ��� ���� ������ - ��� ������ � ������
// ����� ������� - ��������� � ������ ��������� ������� �������� ������, � ����� ��������� ����������� ����� ������
  while (hssavetime!=0) {
    CLRWDT();
    CheckSaveHardDSt();
  }
// ����� ������� ����� �������� ��������� �������������, ��� ���� ���� ������� ����������� ����� ��������� � ������ ������ ������
// ��� ��������� ��������� ������������� ������������� ��� ����� (���� �������� ������ �����, � ���������� - ������), ���� ������ 
// (���� ������ ���������� ��� ������ ������ �����) ����� ������.
//
// ������������� ����� � �� - ������ ����� ������������� ������ ������ ���������
// ��������� ��������� ����� ������ ��������� - 115200 ���, 8N1 - �� ���� 115200 ��� � ������� ����������,
// 8 ��� �� ���� ����� ������, ��� �������� ��������, 1 �������� ���.
  InitModBus(HardDSt.mbaddr,32);
//
  StartIzm();   // ��������� ��������� ���������� ������
// ���������� �������� ���� - �� ���� ��������� ������� �� �������
  for (;;) {  
// �������� ��������� ��������� ������� ModBus ��� �������������� � ����� ModBus.h
    if (ModBusProc()) {
      hssavetime=zgetssec();
    }
// ��������� - ���� ����, ���������� ���������������� ������ � ������     
    CheckSaveHardDSt();
// ��������� - ����� ���� ��� ����� ��������� ��������� ���������� ������?
    if (adcrezflg) {
// ��, �����. ������������ � ������������� ���������
      uint32_t pit,prom;
      uint8_t n;
// ������ �������� ���� ������ � ��������� �� ModBus ��������      
      for (n=0;n<10;n++) info.adc[n]=adcdat[n];
// ������ ���������� ������� �� �������� ���������� (��. ������������ �����������)
      pit=GetUpwr(adcdat[9]);
// ���������� �������� ���������� ������� ���������� � ��������, ��������� �� ModBus      
      info.upit=(pit/1000);
// ��� �������� ������������ � �������� ����������� ��������� (���������)      
      info.cels=CalcT(adcdat[9],adcdat[8]);
// ������������� �������� ������������ ��� �������. �������� ������� ����� ���������� ������� �����������
// ������� ������� CalcU �� ������������, ��������� ��� ��������� ����� ������� ��� ������������ ���������� �������
      pit/=4096; ///pit>>=12;
      prom=adcdat[0]*pit;
      info.sig1=prom/1000;
      prom=adcdat[2]*pit;
      info.sig2=prom/1000;
// ������������� ��������
      StartIzm(); 
    }
		
		if (fire){
			switch(litghtType){
				case 0:
			
				if (tdlt(lighttime)<=300){
					if (GGetPin(GREENLED)){
					GResetPin(GREENLED);
				}
				else {
					GSetPin(GREENLED);
				}
				}
				
				if (tdlt(lighttime)>=300 && tdlt(lighttime)<=500){
					if (GGetPin(BLUELED)){
					GResetPin(BLUELED);
				}
				else {
					GSetPin(BLUELED);
				}
			  }
				
				if (tdlt(lighttime)>=800){
 					lighttime = getssec();
					if (GGetPin(REDLED)){
					GResetPin(REDLED);
				}
				else {
					GSetPin(REDLED);
				}
			  }
				break;
				
				case 1:
					if (tdlt(lighttime)<=600){
					if (GGetPin(BLUELED)){
					GResetPin(BLUELED);
				}
				else {
					GSetPin(BLUELED);
				}
			  }
				
				if (tdlt(lighttime)>=1200){
 					lighttime = getssec();
					if (GGetPin(REDLED)){
					GResetPin(REDLED);
				}
				else {
					GSetPin(REDLED);
				}
			  }
				break;
				
				case 2:
					if (tdlt(lighttime)<=800){
					if (GGetPin(GREENLED)){
					GResetPin(GREENLED);
				}
				else {
					GSetPin(GREENLED);
				}
				}
				
				if (tdlt(lighttime)>=800 && tdlt(lighttime)<=2000){
					if (GGetPin(BLUELED)){
					GResetPin(BLUELED);
				}
				else {
					GSetPin(BLUELED);
				}
			  }
				
				if (tdlt(lighttime)>=2000){
 					lighttime = getssec();
					if (GGetPin(REDLED)){
					GResetPin(REDLED);
				}
				else {
					GSetPin(REDLED);
				}
			  }
				break;
				
			}
			
		}
			if (GGetPin(BUTTON) == 0){
				if (flag){
					ledtime1=getssec();
					flag = false;
					fire = true;
				}
			}
			else{
				if (!flag){
					flag = true;
					if (tdlt(ledtime1)>=500){
						GResetPin(GREENLED);
						GResetPin(REDLED);
						GResetPin(BLUELED);
						fire = false;
					}
					else{
					switch (litghtType){
						case 0:
							litghtType++;
						
							break;
						case 1:
							
						litghtType++;
							break;
						case 2:
							
						litghtType = 0;
							break;
					}
						
						
						
					}
				
			}
			}
				
					
		
   
		
		
		
  }
}
