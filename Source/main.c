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

// Работаем на 16МГц от внутреннего RC генератора. Точность обещают -3.8..+5.5% во всем диапазоне температур
// ВНИМАНИЕ!!! Нельзя обнулять RCC->CR
void SysInit(void) {
  RCC->CR |= 0x00000001;     // Включаем HSI
  while (!(RCC->CR&0x02));  // Ожидаем HSI (на всякий случай)
// Обнуляем не нужное
  RCC->CR &= 0x0000FFF8;
  RCC->CFGR=0;  // Переходим на HSI и все предделители - на нули
// Запускаем PLL - работать будем на 16 МГц
  RCC->CFGR2=1;  // В том числе предделитель перед PLL
  RCC->CFGR3=0;
  RCC->CFGR|=0x00080000;  // Делим на 2, умножаем на 4, источник - HSI
  RCC->CR |= 0x01000000;  // Запускаем PLL
  while (!RCC->CR&0x02000000);        // Ожидаем включения PLL. Если не включился - сторожевой таймер нас рассудит
  FLASH->ACR = 0x00000010;  // Enable Prefetch Buffer and set Flash Latency
  RCC->CFGR|=0x00000002;    // Выбираем PLL как источник тактирования
  while (!((RCC->CFGR&0x0C)==(0x02<<2)));  // Ожидаем подтверждения
// 
  RCC->CIR = 0x00000000;    // Запрещаем все прерывания
}

void ConfigHW() {
// Сначала подключаем к шине все блоки, какие нам могут понадобиться
  RCC->AHBENR|=RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN|RCC_AHBENR_GPIODEN|RCC_AHBENR_GPIOFEN;
  RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;     // Таймер генератора временных промежутков 1мс и одновременно - таймаута UART
  RCC->AHBENR|=RCC_AHBENR_DMAEN;        // Прямой доступ к памяти - через него работает UART
  RCC->APB2ENR|=RCC_APB2ENR_USART1EN;   // UART - последовательный порт, из которого будет сделан RS485 ModBus
  RCC->APB2ENR|=RCC_APB2ENR_SYSCFGCOMPEN;// Системный регистр (пока тут наверное не нужен)
  RCC->APB2ENR|=RCC_APB2ENR_ADCEN;      // АЦП для измерения аналоговых сигналов с датчиков и токовых сигналов с мостового драйвера
  RCC->APB1ENR|=RCC_APB1ENR_DACEN;      // ЦАП для формирования среднего уровня для смещения сигналов с мостов
// Теперь настраиваем выводы  
// Выводы
//
// Светодиоды
  GSetPinToOutput(LED1,0,0);
  GSetPinToOutput(LED2,0,0);
  GSetPinToOutput(LED3,0,0);
	GSetPinToOutput(BLUELED,0,0);
	GSetPinToOutput(REDLED,0,0);
	GSetPinToOutput(GREENLED,0,0);
	GSetPinToInput(BUTTON,GPIO_PULLUP);
// Входы АЦП датчиков
  GSetPinToAnalog(AN1);
  GSetPinToAnalog(AN2);
  GSetPinToAnalog(AN3);
  GSetPinToAnalog(AN4);
  GSetPinToAnalog(AN5);
  GSetPinToAnalog(AN6);
  GSetPinToAnalog(AN7);
  GSetPinToAnalog(AN8);
  GSetPinToAnalog(DACPIN);
// Включаем ЦАП и выдаем средний уровень - точку смещения для всех датчиков
  InitDAC(DAC1CH1_EN);
  SetDAC1(32768);    //  Полный диапазон - 65536, половина - 32768
// Настраиваем АЦП
  ADC051DeInit();
// Включаем опорное напряжение АЦП и внутренний термодатчик
  ADC051Common(ADCCOM_TSEN|ADCCOM_VREFEN);
  ADC051StartCal(ADC1);
  ADC051SelCh(ADC1,0x000340EF);  // Здесь задается, какие каналы будут измеряться
  ADC051Init(ADC1,0x03,ADCCFG1_OVRMOD|ADCCFG1_DMAEN,0);
// Ожидаем окончания калибровки
  while (ADC1->CR&ADCCR_ADCAL) CLRWDT();
// Дальше можно работать с АЦП используя DMA
//
// Последовательный порт RS485 настраивается в файле fpuart1.c через связку функций InitModBus -> InitFPU1
// Таймер генерации SSEC
  InitSSEC();
  __enable_irq();
}

// Все измеренные за один проход значения
uint16_t adcdat[10]; 
// Флаг наличия измеренных значений
uint8_t adcrezflg=0;

// Запускаем измерение очередного набора данных
void StartIzm(void) {
// Работаем с АЦП
  StopDMA(DMA1_Channel1);
  InitDMACh(DMA1_Channel1,DMA_MEMSIZE16BIT|DMA_MINC|DMA_PRHSIZE16BIT|DMA_CIRC|DMA_TCIE,(uint32_t)(&(ADC1->DR)),(uint32_t)(&(adcdat[0])),10);    
// Разрешаем прерывания от DMA
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);    
  StartDMA(DMA1_Channel1);
  adcrezflg=0;
  ADC1->CR|=ADCCR_START;
}        

// Обработчик прерываний от DMA АЦП
void DMA1_Channel1_IRQHandler(void) {
  DMA1->IFCR=DMA_TCIF(1);  // Сбрасываем флаг прерывания
  adcrezflg=1;             // Устанавливаем флаг что результат готов
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
// Основная программа
// Эта функция никогда не заканчивается, потому что выходить некуда - мы в контроллере
int main() {
 uint32_t ledtime1=getssec();
	uint32_t lighttime=getssec();
  uint32_t ledtimeR=getssec();
	uint32_t ledtimeG=getssec();
	uint32_t ledtimeB=getssec();
	int counter = 0 ;
	bool flag = true ; //  выключен
	bool fire = false; // мигать не мигать
	int  litghtType =0; // смена режима подсветки
////////////////////////////////////////////////////
// Инициализация аппаратной части контроллера  
  SysInit();   // Запускаем контроллер на нужной нам частоте (сейчас - 16МГц)
#ifndef MYDEBUG
  InitIWDG(4);
#endif
  ConfigHW();  // Конфигурируем аппаратную часть контроллера (выводы)
// Считываем конфигурацию из памяти контроллера. На самом деле происходит расчет контрольных сумм и сравнение
// их с записанными для двух копий настроечных данных. Если какая-то из копий повреждена - запускается процедура восстановления.
  ReinitFromHardDSt(0);
// Процедура восстановления - 
// Если был сбой памяти - ждём записи в память
// Через таймаут - поскольку в момент включения питания возможны помехи, и будет повторное повреждение копии данных
  while (hssavetime!=0) {
    CLRWDT();
    CheckSaveHardDSt();
  }
// Такая сложная схема хранения позволяет гарантировать, что если даже питание контроллера будет выключено в момент записи данных
// при следующем включении автоматически восстановится или новая (если записана первая копия, а повреждена - вторая), либо старая 
// (если запись прервалась при записи первой копии) копия данных.
//
// Конфигурируем связь с ПК - выводы порта настраиваются внутри данной процедуры
// Параметры настройки порта всегда одинаковы - 115200 бод, 8N1 - то есть 115200 бит в секунду передается,
// 8 бит на одно слово данных, без контроля четности, 1 стоповый бит.
  InitModBus(HardDSt.mbaddr,32);
//
  StartIzm();   // Запускаем измерение аналоговых входов
// Начинается основной цикл - из него программа никогда не выходит
  for (;;) {  
// Вызываем процедуру обработки пакетов ModBus как договаривались в файле ModBus.h
    if (ModBusProc()) {
      hssavetime=zgetssec();
    }
// Проверяем - если надо, записываем конфигурационные данные в память     
    CheckSaveHardDSt();
// Проверяем - может быть уже готов результат измерения аналоговых входов?
    if (adcrezflg) {
// Да, готов. Обрабатываем и перезапускаем измерение
      uint32_t pit,prom;
      uint8_t n;
// Просто копируем весь массив в доступные по ModBus регистры      
      for (n=0;n<10;n++) info.adc[n]=adcdat[n];
// Расчет напряжения питания по опорному напряжению (см. документацию контроллера)
      pit=GetUpwr(adcdat[9]);
// Полученное значение напряжения питания записываем в регистры, доступные по ModBus      
      info.upit=(pit/1000);
// Для интереса рассчитываем и копируем температуру кристалла (примерную)      
      info.cels=CalcT(adcdat[9],adcdat[8]);
// Пересчитываем значения интересующих нас каналов. Пересчет ведется через напряжение питания контроллера
// Функция расчета CalcU не используется, поскольку она постоянно будет считать уже подсчитанное напряжение питания
      pit/=4096; ///pit>>=12;
      prom=adcdat[0]*pit;
      info.sig1=prom/1000;
      prom=adcdat[2]*pit;
      info.sig2=prom/1000;
// Перезапускаем измеение
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
