/*
 * Copyright (c) 2019 by Vadim Kulakov vad7@yahoo.com, vad711
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 *
 * Водоподготовка, дозирование, управление насосной станцией.
 */

#include "Arduino.h"
#include "Constant.h"                       // Должен быть первым !!!! кое что берется в стандартных либах для настройки либ, Config.h входит в него

#include <WireSam.h>                        // vad711 доработанная библиотека https://github.com/vad7/Arduino-DUE-WireSam
#include <DS3232.h>                         // vad711 Работа с часами i2c - используются при выключении питания. модифицированная https://github.com/vad7/Arduino-DUE-WireSam
#include <extEEPROM.h>                      // vad711 Работа с eeprom и fram i2c библиотека доработана для DUE!!! https://github.com/vad7/Arduino-DUE-WireSam

#include <SPI.h>                            // SPI - стандартная
#include "SdFat.h"                          // SdFat - модифицированная
#include "FreeRTOS_ARM.h"                   // модифицированная
#include <rtc_clock.h>                      // работа со встроенными часами  Это основные часы!!!
#include <ModbusMaster.h>                   // Используется МОДИФИЦИРОВАННАЯ для омрона мх2 либа ModbusMaster https://github.com/4-20ma/ModbusMaster

// Глубоко переделанные библиотеки
#include "Ethernet.h"                       // Ethernet - модифицированная
#include <Dns.h>                            // DNS - модифицированная
#include <SerialFlash.h>                    // Либа по работе с spi флешом как диском

#include "Hardware.h"
#include "Message.h"
#include "Information.h"
#include "MainClass.h"
#include "Statistics.h"
#include "LiquidCrystal.h"
#include "HX711.h"

// LCD ---------- rs, en, d4, d5, d6, d7
LiquidCrystal lcd(25, 34, 33, 36, 35, 38);
HX711 Weight;

#if defined(W5500_ETHERNET_SHIELD)                  // Задание имени чипа для вывода сообщений
  const char nameWiznet[] ={"W5500"};
#elif defined(W5200_ETHERNET_SHIELD)
  const char nameWiznet[] ={"W5200"};
#else
  const char nameWiznet[] ={"W5100"};
#endif

// Глобальные переменные
extern boolean set_time_NTP(void);   
EthernetServer server1(80);                         // сервер
EthernetUDP Udp;                                    // Для NTP сервера
EthernetClient ethClient(W5200_SOCK_SYS);           // для MQTT

#ifdef RADIO_SENSORS
void check_radio_sensors(void);
void radio_sensor_send(char *cmd);
#endif

#ifdef MQTT                                 // признак использования MQTT
#include <PubSubClient.h>                   // передаланная под многозадачность  http://knolleary.net
PubSubClient w5200_MQTT(ethClient);  	    // клиент MQTT
#endif

// I2C eeprom Размер в килобитах, число чипов, страница в байтах, адрес на шине, тип памяти:
extEEPROM eepromI2C(I2C_SIZE_EEPROM,I2C_MEMORY_TOTAL/I2C_SIZE_EEPROM,I2C_PAGE_EEPROM,I2C_ADR_EEPROM,I2C_FRAM_MEMORY);
//RTC_clock rtcSAM3X8(RC);                                               // Внутренние часы, используется внутренний RC генератор
RTC_clock rtcSAM3X8(XTAL);                                               // Внутренние часы, используется часовой кварц
DS3232  rtcI2C;                                                          // Часы 3231 на шине I2C
static Journal  journal;                                                 // системный журнал, отдельно т.к. должен инициализоваться с начала старта
static MainClass MC;                                                     // Главный класс
static devModbus Modbus;                                                 // Класс модбас - управление инвертором
SdFat card;                                                              // Карта памяти

// Use the Arduino core to set-up the unused USART2 on Serial4 (without serial events)
#ifdef USE_SERIAL4
RingBuffer rx_buffer5;
RingBuffer tx_buffer5;
USARTClass Serial4(USART2, USART2_IRQn, ID_USART2, &rx_buffer5, &tx_buffer5);
//void serialEvent4() __attribute__((weak));
//void serialEvent4() { }
void USART2_Handler(void)   // Interrupt handler for UART2
{
	Serial4.IrqHandler();     // In turn calls on the Serial2 interrupt handler
}
#endif

// Мютексы блокираторы железа
SemaphoreHandle_t xModbusSemaphore;                 // Семафор Modbus, инвертор запас на счетчик
SemaphoreHandle_t xWebThreadSemaphore;              // Семафор потоки вебсервера,  деление сетевой карты
SemaphoreHandle_t xI2CSemaphore;                    // Семафор шины I2C, часы, память, мастер OneWire
SemaphoreHandle_t xSPISemaphore;                    // Семафор шины SPI  сетевая карта, память. SD карта // пока не используется
SemaphoreHandle_t xLoadingWebSemaphore;             // Семафор загрузки веб морды в spi память
uint16_t lastErrorFreeRtosCode;                     // код последней ошибки операционки нужен для отладки
uint32_t startSupcStatusReg;                        // Состояние при старте SUPC Supply Controller Status Register - проверяем что с питание

// Структура для хранения одного сокета, нужна для организации многопотоковой обработки
#define fABORT_SOCK   0                     // флаг прекращения передачи (произошел сброс сети)
#define fUser         1                     // Признак идентификации как обычный пользователь (нужно подменить файл меню)

struct type_socketData
  {
    EthernetClient client;                  // Клиент для работы с потоком
    byte inBuf[W5200_MAX_LEN];              // буфер для работы с w5200 (входной запрос)
    char *inPtr;                            // указатель в входном буфере с именем файла или запросом (копирование не делаем)
    char outBuf[3*W5200_MAX_LEN];           // буфер для формирования ответа на запрос и чтение с карты
    int8_t sock;                            // сокет потока -1 свободный сокет
    uint8_t flags;                          // Флаги состояния потока
    uint16_t http_req_type;                 // Тип запроса
  };
static type_socketData Socket[W5200_THREAD];   // Требует много памяти 4*W5200_MAX_LEN*W5200_THREAD=24 кб

// Установка вачдога таймера вариант vad711 - адаптация для DUE 1.6.11+
// WDT_TIME период Watchdog таймера секунды но не более 16 секунд!!! ЕСЛИ WDT_TIME=0 то Watchdog будет отключен!!!
volatile unsigned long ulHighFrequencyTimerTicks;   // vad711 - адаптация для DUE 1.6.11+
void watchdogSetup(void)
{
#if WDT_TIME == 0
	watchdogDisable();
#else 
	watchdogEnable(WDT_TIME * 1000);
#endif
}

__attribute__((always_inline)) inline void _delay(int t) // Функция задержки (мсек) в зависимости от шедулера задач FreeRtos
{
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(t/portTICK_PERIOD_MS);
	else delay(t);
}

void yield(void)
{
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) taskYIELD();
}

// Захватить семафор с проверкой, что шедуллер работает
BaseType_t SemaphoreTake(QueueHandle_t xSemaphore, TickType_t xBlockTime)
{
	if(xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return pdTRUE;
	else {
		for(;;) {
			if(xSemaphoreTake(xSemaphore, 0) == pdTRUE) return pdTRUE;
			if(!xBlockTime--) break;
			vTaskDelay(1/portTICK_PERIOD_MS);
		}
		return pdFALSE;
	}
}

// Освободить семафор с проверкой, что шедуллер работает
inline void SemaphoreGive(QueueHandle_t xSemaphore)
{
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) xSemaphoreGive(xSemaphore);
}

// Остановить шедулер задач, возврат 1, если получилось
uint8_t TaskSuspendAll(void) {
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
		vTaskSuspendAll(); // запрет других задач
		return 1;
	}
	return 0;
}


void setup() {
	// 1. Инициализация SPI
	// Баг разводки дуе (вероятность). Есть проблема с инициализацией spi.  Ручками прописываем
	// https://groups.google.com/a/arduino.cc/forum/#!topic/developers/0PUzlnr7948
	// http://forum.arduino.cc/index.php?topic=243778.0;nowap
	pinMode(PIN_SPI_SS0,INPUT_PULLUP);          // Eth Pin 77
	pinMode(BOARD_SPI_SS0,INPUT_PULLUP);        // Eth Pin 10
	pinMode(PIN_SPI_SS1,INPUT_PULLUP);          // SD Pin  87
	pinMode(BOARD_SPI_SS1,INPUT_PULLUP);     	// SD Pin  4

#ifdef SPI_FLASH
	pinMode(PIN_SPI_CS_FLASH,INPUT_PULLUP);     // сигнал CS управление чипом флеш памяти
	pinMode(PIN_SPI_CS_FLASH,OUTPUT);           // сигнал CS управление чипом флеш памяти (ВРЕМЕННО, пока нет реализации)
#endif
	SPI_switchAllOFF();                         // Выключить все устройства на SPI

	delay(10);
	DebugToSerialOn = true;

	// Борьба с зависшими устройствами на шине  I2C (в первую очередь часы) неудачный сброс
	Recover_I2C_bus();

	// 2. Инициализация журнала и в нем последовательный порт
	journal.Init();
#ifdef TEST_BOARD
	journal.jprintf("\n---> TEST BOARD!!!\n\n");
#endif
	journal.printf("Firmware version: %s\n",VERSION);
	showID();                                                                  // информация о чипе
	getIDchip((char*)Socket[0].inBuf);
	journal.printf("Chip ID SAM3X8E: %s\n", Socket[0].inBuf);// информация об серийном номере чипа
	if(GPBR->SYS_GPBR[0] & 0x80000000) GPBR->SYS_GPBR[0] = 0; else GPBR->SYS_GPBR[0] |= 0x80000000; // очистка старой причины
	lastErrorFreeRtosCode = GPBR->SYS_GPBR[0] & 0x7FFFFFFF;         // Сохранение кода ошибки при страте (причину перегрузки)
	journal.jprintf("Reset: %s, 0x%04x", ResetCause(), lastErrorFreeRtosCode);
	if(GPBR->SYS_GPBR[4]) journal.jprintf(" %d", GPBR->SYS_GPBR[4]);
	journal.jprintf("\n");

#ifdef PIN_LED1                            // Включение (точнее индикация) питания платы если необходимо
	pinMode(PIN_LED1,OUTPUT);
	digitalWriteDirect(PIN_LED1, HIGH);
#endif

	SupplyMonitorON(SUPC_SMMR_SMTH_3_0V);           // включение монитора питания

#ifdef RADIO_SENSORS
	RADIO_SENSORS_SERIAL.begin(RADIO_SENSORS_PSPEED, RADIO_SENSORS_PCONFIG);
#endif

	// 3. Инициализация и проверка шины i2c
	journal.printf("* Setting and checking I2C devices . . .\n");

	uint8_t eepStatus=0;
#ifndef I2C_JOURNAL_IN_RAM
	if(journal.get_err()) { // I2C память и журнал в ней уже пытались инициализировать
#endif
		Wire.begin();
		for(uint8_t i=0; i<I2C_NUM_INIT; i++ )
		{
			if ((eepStatus=eepromI2C.begin(I2C_SPEED))>0)    // переходим на I2C_SPEED и пытаемся инициализировать
			{
				journal.jprintf("$ERROR - I2C mem failed, status = %d\n", eepStatus);
#ifdef POWER_CONTROL
				digitalWriteDirect(PIN_POWER_ON, HIGH);
				digitalWriteDirect(PIN_LED1, LOW);
				_delay(2000);
				digitalWriteDirect(PIN_POWER_ON, LOW);
				digitalWriteDirect(PIN_LED1, HIGH);
				_delay(500);
				Wire.begin();
#else
				Wire.begin();
				_delay(500);
#endif
				WDT_Restart(WDT);                       // Сбросить вачдог
			}  else break;   // Все хорошо
		} // for
#ifndef I2C_JOURNAL_IN_RAM
	}
#endif
	if(eepStatus)  // если I2C память не инициализирована, делаем попытку запустится на малой частоте один раз!
	{
		if((eepStatus=eepromI2C.begin(twiClock100kHz))>0) {
			journal.printf("$ERROR - I2C mem init failed on speed %d kHz, status = %d\n",twiClock100kHz/1000,eepStatus);
			eepromI2C.begin(I2C_SPEED);
			goto x_I2C_init_std_message;
		} else journal.printf("I2C bus low speed, init on %d kHz - OK\n",twiClock100kHz/1000);
	} else {
		x_I2C_init_std_message:
		journal.printf("I2C init on %d kHz - OK\n",I2C_SPEED/1000);
	}

	// Сканирование шины i2c
	//    if (eepStatus==0)   // есть инициализация
	{
		byte error, address;
		const byte start_address = 8;       // lower addresses are reserved to prevent conflicts with other protocols
		const byte end_address = 119;       // higher addresses unlock other modes, like 10-bit addressing

		for(address = start_address; address < end_address; address++ )
		{
#ifdef ONEWIRE_DS2482         // если есть мост
			if(address == I2C_ADR_DS2482) {
				error = OneWireBus.Init();
#ifdef ONEWIRE_DS2482_SECOND
			} else if(address == I2C_ADR_DS2482_2) {
				error = OneWireBus2.Init();
#endif
#ifdef ONEWIRE_DS2482_THIRD
			} else if(address == I2C_ADR_DS2482_3) {
				error = OneWireBus3.Init();
#endif
#ifdef ONEWIRE_DS2482_FOURTH
			} else if(address == I2C_ADR_DS2482_4) {
				error = OneWireBus4.Init();
#endif
			} else
#endif
			{
				Wire.beginTransmission(address); error = Wire.endTransmission();
			}
			if(error==0)
			{
				journal.printf("I2C device found at address %s",byteToHex(address));
				switch (address)
				{
#ifdef ONEWIRE_DS2482
				case I2C_ADR_DS2482_4:
				case I2C_ADR_DS2482_3:
				case I2C_ADR_DS2482_2:
				case I2C_ADR_DS2482:  		journal.printf(" - OneWire DS2482-100 bus: %d%s\n", address - I2C_ADR_DS2482 + 1, (ONEWIRE_2WAY & (1<<(address - I2C_ADR_DS2482))) ? " (2W)" : ""); break;
#endif
#if I2C_FRAM_MEMORY == 1
				case I2C_ADR_EEPROM:	journal.printf(" - FRAM FM24V%02d\n", I2C_MEMORY_TOTAL*10/1024); break;
#if I2C_MEMORY_TOTAL != I2C_SIZE_EEPROM
				case I2C_ADR_EEPROM+1:	journal.printf(" - FRAM second 64k page\n"); break;
#endif
#else
				case I2C_ADR_EEPROM:	journal.printf(" - EEPROM AT24C%d\n", I2C_SIZE_EEPROM);break; // 0x50 возможны варианты
#if I2C_MEMORY_TOTAL != I2C_SIZE_EEPROM
				case I2C_ADR_EEPROM+1:	journal.printf(" - EEPROM second 64k page\n"); break;
#endif
#endif
				case I2C_ADR_RTC   :		journal.printf(" - RTC DS3231\n"); break; // 0x68
				default            :		journal.printf(" - Unknow\n"); break; // не определенный тип
				}
				_delay(100);
			}
			//    else journal.printf("I2C device bad endTransmission at address %s code %d",byteToHex(address), error);
		} // for
	} //  (eepStatus==0)

#ifndef ONEWIRE_DS2482         // если нет моста
	if(OneWireBus.Init()) journal.jprintf("Error init 1-Wire: %d\n", OneWireBus.GetLastErr());
	else journal.printf("1-Wire init Ok\n");
#endif

#ifdef RADIO_SENSORS
	check_radio_sensors();
	if(rs_serial_flag == RS_SEND_RESPONSE) {
		_delay(5);
		check_radio_sensors();
	}
#endif
#ifdef USE_SERIAL4
	PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB20A_TXD2 | PIO_PB21A_RXD2, PIO_DEFAULT);
	//Serial4.begin(115200);
#endif

	// 4. Инициализировать основной класс
	journal.printf("* Init %s.\n",(char*)nameMainClass);
	MC.init();                           // Основной класс

	// 5. Установка сервисных пинов

	journal.printf("* Read safe Network key.\n");
	pinMode(PIN_KEY_SAFE, INPUT_PULLUP);               // Кнопка 1, Нажатие при включении - режим safeNetwork (настрока сети по умолчанию 192.168.0.177  шлюз 192.168.0.1, не спрашивает пароль на вход в веб морду)
	MC.safeNetwork=!digitalReadDirect(PIN_KEY_SAFE);
	if (MC.safeNetwork)  journal.jprintf("Mode safeNetwork ON \n"); else journal.printf("Mode safeNetwork OFF \n");

	pinMode(PIN_BEEP, OUTPUT);              // Выход на пищалку
	pinMode(PIN_LED_OK, OUTPUT);            // Выход на светодиод мигает 0.5 герца - ОК  с частотой 2 герца ошибка
	digitalWriteDirect(PIN_BEEP,LOW);       // Выключить пищалку
	digitalWriteDirect(PIN_LED_OK,HIGH);    // Выключить светодиод

	// 7. Инициализация СД карты и запоминание результата 3 попытки
	journal.printf("* Init SD card.\n");
	WDT_Restart(WDT);
	MC.set_fSD(initSD());
	WDT_Restart(WDT);                          // Сбросить вачдог  иногда карта долго инициализируется
	digitalWriteDirect(PIN_LED_OK,LOW);        // Включить светодиод - признак того что сд карта инициализирована
	//_delay(100);

	// 8. Инициализация spi флеш диска
#ifdef SPI_FLASH
	journal.printf("* Init SPI flash disk.\n");
	MC.set_fSPIFlash(initSpiDisk(true));  // проверка диска с выводом инфо
#else
	journal.printf("* No SPI flash in config.\n");
#endif

	// 9. Чтение ЕЕПРОМ
	journal.printf("* Load data from I2C memory.\n");
	if(MC.load_WorkStats() == ERR_HEADER2_EEPROM)           // Загрузить счетчики
	{
		journal.printf("I2C memory is empty!\n");
		MC.save_WorkStats();
	} else {
		MC.load((uint8_t *)Socket[0].outBuf, 0);      // Загрузить настройки
	}

	// обновить хеш для пользователей
	MC.set_hashUser();
	MC.set_hashAdmin();
	journal.printf(" Web interface source: ");
	switch (MC.get_SourceWeb())
	{
	case pMIN_WEB:   journal.printf("internal\n"); break;
	case pSD_WEB:    journal.printf("SD card\n"); break;
	case pFLASH_WEB: journal.printf("SPI Flash\n"); break;
	default:         journal.printf("unknown\n"); break;
	}

	journal.printf("* Start read ADC sensors.\n");
	start_ADC(); // после инициализации
	//journal.printf(" Mask ADC_IMR: 0x%08x\n",ADC->ADC_IMR);

	// 10. Сетевые настройки
	journal.printf("* Setting Network.\n");
	if(initW5200(true)) {   // Инициализация сети с выводом инфы в консоль
		W5100.getMACAddress((uint8_t *)Socket[0].outBuf);
		journal.printf(" MAC: %s\n", MAC2String((uint8_t *)Socket[0].outBuf));
	}
	digitalWriteDirect(PIN_BEEP,LOW);          // Выключить пищалку
	WDT_Restart(WDT);

	// 11. Разбираемся со всеми часами и синхронизацией
	journal.printf("* Setting time.\n");
	int16_t s = rtcI2C.readRTC(RTC_STATUS);        //read the status register
	if(s != -1) {
		if(s & (1<<RTC_OSF)) {
			journal.jprintf(" RTC low battery!\n");
			set_Error(ERR_RTC_LOW_BATTERY, (char*)"");
			rtcI2C.writeRTC(RTC_STATUS, 0);  //clear the Oscillator Stop Flag
			NeedSaveRTC = RTC_SaveAll;
			update_RTC_store_memory();
		} else {
			if(rtcI2C.readRTC(RTC_STORE_ADDR, (uint8_t*)&MC.RTC_store, sizeof(MC.RTC_store))) {
				memset(&MC.RTC_store, 0, sizeof(MC.RTC_store));
				journal.jprintf(" Error read RTC store!\n");
			} else journal.printf(" RTC MEM: %02X, D: %d, R: %d\n", MC.RTC_store.Work, MC.RTC_store.UsedToday, MC.RTC_store.UsedRegen);
		}
	} else journal.jprintf(" Error read RTC!\n");
	set_time();

	// 12. Инициалазация уведомлений
	//journal.printf("* Message DNS update.\n");
	//MC.message.dnsUpdate();

	// 13. Инициалазация MQTT
#ifdef MQTT
	journal.printf("* Client MQTT update IP from DNS . . .\n");
	MC.clMQTT.dnsUpdateStart();
#else
	journal.printf("* Client MQTT disabled\n");
#endif

	WDT_Restart(WDT);
	// 14. Инициалазация Statistics
	journal.printf("* Statistics ");
	if(MC.get_fSD()) {
		journal.printf("writing on SD card\n");
		Stats.Init();             // Инициализовать статистику
	} else journal.printf("not available\n");

#ifdef TEST_BOARD
	// Scan oneWire - TEST.
	//MC.scan_OneWire(Socket[0].outBuf);
#endif

	Weight.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	Weight_Clear_Averaging();
	journal.printf("* Scale inited, ADC: %d\n", Weight.read() - MC.Option.WeightZero);

	// Создание задач FreeRTOS  ----------------------
	journal.printf("* Create tasks FreeRTOS.\n");
	MC.mRTOS=236;  //расчет памяти для задач 236 - размер данных шедуллера, каждая задача требует 64 байта+ стек (он в словах!!)
	MC.mRTOS=MC.mRTOS+64+4*configMINIMAL_STACK_SIZE;  // задача бездействия
	//MC.mRTOS=MC.mRTOS+4*configTIMER_TASK_STACK_DEPTH;  // программные таймера (их теперь нет)

	// ПРИОРИТЕТ 4 Высший приоритет
	if(xTaskCreate(vPumps, "Pumps", 100, NULL, 4, &MC.xHandlePumps) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 100;
	//vTaskSuspend(MC.xHandleFeedPump);      // Остановить задачу

	// ПРИОРИТЕТ 3 Очень высокий приоритет
	if(xTaskCreate(vReadSensor, "ReadSensor", 150, NULL, 3, &MC.xHandleReadSensor) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 150;

	// ПРИОРИТЕТ 2 средний
	if(xTaskCreate(vKeysLCD, "KeysLCD", 70, NULL, 4, &MC.xHandleKeysLCD) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 70;
	if(xTaskCreate(vService, "Service", 170, NULL, 2, &MC.xHandleService) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 170;

	// ПРИОРИТЕТ 1 низкий - обслуживание вебморды в несколько потоков
	// ВНИМАНИЕ первый поток должен иметь больший стек для обработки фоновых сетевых задач
	// 1 - поток
	#define STACK_vWebX 180
	if(xTaskCreate(vWeb0,"Web0", STACK_vWebX+10,NULL,1,&MC.xHandleUpdateWeb0)==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4*STACK_vWebX+10;
	if(xTaskCreate(vWeb1,"Web1", STACK_vWebX,NULL,1,&MC.xHandleUpdateWeb1)==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4*STACK_vWebX;
	if(xTaskCreate(vWeb2,"Web2", STACK_vWebX,NULL,1,&MC.xHandleUpdateWeb2)==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4*STACK_vWebX;
	vSemaphoreCreateBinary(xLoadingWebSemaphore);           // Создание семафора загрузки веб морды в spi память
	if(xLoadingWebSemaphore==NULL) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);

	vSemaphoreCreateBinary(xWebThreadSemaphore);               // Создание мютекса
	if (xWebThreadSemaphore==NULL) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	vSemaphoreCreateBinary(xI2CSemaphore);                     // Создание мютекса
	if (xI2CSemaphore==NULL) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	//vSemaphoreCreateBinary(xSPISemaphore);                     // Создание мютекса
	//if (xSPISemaphore==NULL) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	// Дополнительные семафоры (почему то именно здесь) Создается когда есть модбас
	if(Modbus.get_present())
	{
		vSemaphoreCreateBinary(xModbusSemaphore);                       // Создание мютекса
		if (xModbusSemaphore==NULL) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	}

	journal.printf(" Create tasks - OK, size %d bytes\n",MC.mRTOS);

	//journal.printf("* Send a notification . . .\n");
	//MC.message.setMessage(pMESSAGE_RESET,(char*)"Контроллер был перезагружен",0);    // сформировать уведомление о загрузке
	journal.printf("* Information:\n");
	freeRamShow();
	MC.startRAM=freeRam()-MC.mRTOS;   // оценка свободной памяти до пуска шедулера, поправка на 1054 байта
	journal.printf("FREE MEMORY %d bytes\n",MC.startRAM);
	journal.printf("Temperature SAM3X8E: %.2f\n",temp_DUE());
	journal.printf("Temperature DS2331: %.2d\n", getTemp_RtcI2C());
	//MC.Stat.generate_TestData(STAT_POINT); // Сгенерировать статистику STAT_POINT точек только тестирование
	journal.printf("Start FreeRTOS!\n\n");
#ifndef TEST_BOARD
	DebugToSerialOn = false;
#endif
	eepromI2C.use_RTOS_delay = 1;       //vad711
	//
	vTaskStartScheduler();              // СТАРТ !!
	journal.printf("FreeRTOS NOT STARTED!!!\n");
}


void loop() {}

//  З А Д А Ч И -------------------------------------------------
// Это и есть поток с минимальным приоритетом измеряем простой процессора
//extern "C" 
//{
static unsigned long cpu_idle_max_count = 0; // 1566594 // максимальное значение счетчика, вычисляется при калибровке и соответствует 100% CPU idle

extern "C" void vApplicationIdleHook(void)  // FreeRTOS expects C linkage
{
	static boolean ledState = LOW;
	static unsigned long countLastTick = 0;
	static unsigned long countLED = 0;
	static unsigned long ulIdleCycleCount = 0;                                    // наш трудяга счетчик

	// в
	//WDT_Restart(WDT);                                                            // Сбросить вачдог
	ulIdleCycleCount++;                                                          // приращение счетчика

	if(xTaskGetTickCount() - countLastTick >= 3000)		// мсек
	{
		countLastTick = xTaskGetTickCount();                            // расчет нагрузки
		if(ulIdleCycleCount > cpu_idle_max_count) cpu_idle_max_count = ulIdleCycleCount; // это калибровка запоминаем максимальные значения
		MC.CPU_IDLE = (100 * ulIdleCycleCount) / cpu_idle_max_count;              // вычисляем текущую загрузку
		ulIdleCycleCount = 0;
	}

	// Светодиод мигание в зависимости от ошибки и подача звукового сигнала при ошибке
	if(xTaskGetTickCount() - countLED > TIME_LED_ERR) {
		if(MC.get_errcode() != OK) {          // Ошибка
			digitalWriteDirect(PIN_BEEP, MC.get_Beep() ? ledState : LOW); // звукового сигнала
			ledState = !ledState;
			digitalWriteDirect(PIN_LED_OK, ledState);
			countLED = xTaskGetTickCount();
		} else if(xTaskGetTickCount() - countLED > TIME_LED_OK)   // Ошибок нет и время пришло
		{
			digitalWriteDirect(PIN_BEEP, LOW);
			ledState = !ledState;       // ОК
			digitalWriteDirect(PIN_LED_OK, ledState);
			countLED = xTaskGetTickCount();
		}
	}
}

// --------------------------- W E B ------------------------
// Задача обслуживания web сервера
// Сюда надо пихать все что связано с сетью иначе конфликты не избежны
// Здесь также обслуживается посылка уведомлений MQTT
// Первый поток веб сервера - дополнительно нагружен различными сервисами
void vWeb0(void *)
{ //const char *pcTaskName = "Web server is running\r\n";
	static unsigned long timeResetW5200 = 0;
	static unsigned long thisTime = 0;
	static unsigned long resW5200 = 0;
	static unsigned long iniW5200 = 0;
	static unsigned long pingt = 0;
#ifdef MQTT
	static unsigned long narmont=0;
	static unsigned long mqttt=0;
#endif
	static boolean active;  // ФЛАГ Одно дополнительное действие за один цикл - распределяем нагрузку
	static boolean network_last_link = true;

	MC.timeNTP = xTaskGetTickCount();        // В первый момент не обновляем
	for(;;)
	{
		STORE_DEBUG_INFO(1);
		web_server(0);
		STORE_DEBUG_INFO(2);
		active = true;                                                         // Можно работать в этом цикле (дополнительная нагрузка на поток)
		vTaskDelay(TIME_WEB_SERVER / portTICK_PERIOD_MS); // задержка чтения уменьшаем загрузку процессора

		// СЕРВИС: Этот поток работает на любых настройках, по этому сюда ставим работу с сетью
		MC.message.sendMessage();                                            // Отработать отсылку сообщений (внутри скрыта задержка после включения)

		active = MC.message.dnsUpdate();                                       // Обновить адреса через dns если надо Уведомления
#ifdef MQTT
		if (active) active=MC.clMQTT.dnsUpdate();                          // Обновить адреса через dns если надо MQTT
#endif
		if(thisTime > xTaskGetTickCount()) thisTime = 0;                         // переполнение счетчика тиков
		if(xTaskGetTickCount() - thisTime > (uint32_t) 10 * 1000)                // Делим частоту - период 10 сек
		{
			thisTime = xTaskGetTickCount();                                      // Запомнить тики
			// 1. Проверка захваченого семафора сети ожидаем  3 времен W5200_TIME_WAIT если мютекса не получаем то сбрасывае мютекс
			if(SemaphoreTake(xWebThreadSemaphore, ((3 + (fWebUploadingFilesTo != 0) * 30) * W5200_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
				SemaphoreGive(xWebThreadSemaphore);
				journal.jprintf(pP_TIME, "UNLOCK mutex xWebThread\n");
				active = false;
				MC.num_resMutexSPI++;
			} // Захват мютекса SPI или ОЖИДАНИНЕ 2 времен W5200_TIME_WAIT и его освобождение
			else SemaphoreGive(xWebThreadSemaphore);

			// Проверка и сброс митекса шины I2C  если мютекса не получаем то сбрасывае мютекс
			if(SemaphoreTake(xI2CSemaphore, (3 * I2C_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
				SemaphoreGive(xI2CSemaphore);
				journal.jprintf(pP_TIME, "UNLOCK mutex xI2CSemaphore\n");
				MC.num_resMutexI2C++;
			} // Захват мютекса I2C или ОЖИДАНИНЕ 3 времен I2C_TIME_WAIT  и его освобождение
			else SemaphoreGive(xI2CSemaphore);

			// 2. Чистка сокетов
			if(MC.time_socketRes() > 0) {
				STORE_DEBUG_INFO(3);
				checkSockStatus();                   // Почистить старые сокеты  если эта позиция включена
			}

			// 3. Сброс сетевого чипа по времени
			if((MC.time_resW5200() > 0) && (active))                             // Сброс W5200 если включен и время подошло
			{
				STORE_DEBUG_INFO(4);
				resW5200 = xTaskGetTickCount();
				if(timeResetW5200 == 0) timeResetW5200 = resW5200;      // Первая итерация не должна быть сразу
				if(resW5200 - timeResetW5200 > MC.time_resW5200() * 1000UL) {
					journal.printf("Resetting the chip %s by timer . . .\n", nameWiznet);
					MC.fNetworkReset = true;                          // Послать команду сброса и применения сетевых настроек
					timeResetW5200 = resW5200;                         // Запомить время сброса
					active = false;
				}
			}
			// 4. Проверка связи с чипом
			if((MC.get_fInitW5200()) && (thisTime - iniW5200 > 60 * 1000UL) && (active)) // проверка связи с чипом сети раз в минуту
			{
				STORE_DEBUG_INFO(5);
				iniW5200 = thisTime;
				if(!MC.NO_Power) {
					boolean lst = linkStatusWiznet(false);
					if(!lst || !network_last_link) {
						if(!lst) journal.jprintf(pP_TIME, "%s no link[%02X], resetting...\n", nameWiznet, W5100.readPHYCFGR());
						MC.fNetworkReset = true;                          // Послать команду сброса и применения сетевых настроек
						MC.num_resW5200++;              // Добавить счетчик инициализаций
						active = false;
					}
					network_last_link = lst;
				}
			}
			// 5.Обновление времени 1 раз в сутки или по запросу (MC.timeNTP==0)
			if((MC.timeNTP == 0) || ((MC.get_updateNTP()) && (thisTime - MC.timeNTP > 60 * 60 * 24 * 1000UL) && (active))) // Обновление времени раз в день 60*60*24*1000 в тиках MC.timeNTP==0 признак принудительного обновления
			{
				STORE_DEBUG_INFO(6);
				MC.timeNTP = thisTime;
				set_time_NTP();                                                 // Обновить время
				active = false;
			}
			// 6. ping сервера если это необходимо
			if((MC.get_pingTime() > 0) && (thisTime - pingt > MC.get_pingTime() * 1000UL) && (active)) {
				STORE_DEBUG_INFO(7);
				pingt = thisTime;
				pingServer();
				active = false;
			}

#ifdef MQTT                                     // признак использования MQTT
			// 7. Отправка нанародный мониторинг
			if ((MC.clMQTT.get_NarodMonUse())&&(thisTime-narmont>TIME_NARMON*1000UL)&&(active))// если нужно & время отправки пришло
			{
				narmont=thisTime;
				sendNarodMon(false);                       // отладка выключена
				active=false;
			}  // if ((MC.clMQTT.get_NarodMonUse()))

			// 8. Отправка на MQTT сервер
			if ((MC.clMQTT.get_MqttUse())&&(thisTime-mqttt>MC.clMQTT.get_ttime()*1000UL)&&(active))// если нужно & время отправки пришло
			{
				mqttt=thisTime;
				if(MC.clMQTT.get_TSUse()) sendThingSpeak(false);
				else sendMQTT(false);
				active=false;
			}
#endif   // MQTT
			taskYIELD();
		} // if (xTaskGetTickCount()-thisTime>10000)

	} //for
	vTaskDelete( NULL);
}

// Второй поток
void vWeb1(void *)
{ //const char *pcTaskName = "Web server is running\r\n";
	for(;;) {
		web_server(1);
		vTaskDelay(TIME_WEB_SERVER / portTICK_PERIOD_MS); // задержка чтения уменьшаем загрузку процессора
	}
	vTaskDelete( NULL);
}
// Третий поток
void vWeb2(void *)
{ //const char *pcTaskName = "Web server is running\r\n";
	for(;;) {
		web_server(2);
		vTaskDelay(TIME_WEB_SERVER / portTICK_PERIOD_MS); // задержка чтения уменьшаем загрузку процессора
	}
	vTaskDelete( NULL);
}

//////////////////////////////////////////////////////////////////////////
// Задача Пользовательский интерфейс (MC.xHandleKeysLCD) "KeysLCD"
void vKeysLCD( void * )
{

	lcd.begin(LCD_COLS, LCD_ROWS); // Setup: cols, rows
	LCD_print((char*)"WaterTreatment v");
	LCD_print((char*)VERSION);
	lcd.setCursor(0, 1);
	_delay(1);
	LCD_print((char*)"Vadim Kulakov(c)2019");
	lcd.setCursor(0, 2);
	_delay(1);
	LCD_print((char*)"vad7@yahoo.com");
	pinMode(PIN_KEY_UP, INPUT_PULLUP);
	pinMode(PIN_KEY_DOWN, INPUT_PULLUP);
	pinMode(PIN_KEY_OK, INPUT_PULLUP);
	static uint32_t DisplayTick = xTaskGetTickCount();
	static char buffer[LCD_COLS + 1];
	//static uint32_t displayed_area = 0; // b1 - Yd,Days Fe,Soft
	for(;;)
	{
		if(!digitalReadDirect(PIN_KEY_OK)) {
			journal.printf("[OK]\n");
		} else if(!digitalReadDirect(PIN_KEY_UP)) {
			journal.printf("[UP]\n");
		} else if(!digitalReadDirect(PIN_KEY_DOWN)) {
			journal.printf("[DWN]\n");
		}
		if(xTaskGetTickCount() - DisplayTick > DISPLAY_UPDATE) { // Update display
			// Display:
			// 12345678901234567890
			// F: 0.000 m3h > 0.000
			// P: 0.00 Tank: 100 %
			// Days Fe:123 Soft:123
			// Day: 0.000 Yd: 0.000
			char *buf = buffer;
			lcd.setCursor(0, 0); vTaskDelay(1);
			int32_t tmp = MC.sFrequency[FLOW].get_Value();
			strcpy(buf, "F:"); buf += 2;
			if(tmp < 10000) *buf++ = ' ';
			buf = dptoa(buf, tmp, 3);
			strcpy(buf, " m3h \x7E"); buf += 6;
			tmp = MC.WorkStats.UsedSinceLastRegen + MC.RTC_store.UsedToday;
			if(tmp < 10000) *buf++ = ' ';
			buf = dptoa(buf, tmp, 3);
			buffer_space_padding(buf, LCD_COLS - (buf - buffer));
			LCD_print(buffer);

			lcd.setCursor(0, 1); vTaskDelay(1);
			strcpy(buf = buffer, "P: "); buf += 3;
			buf = dptoa(buf, MC.sADC[PWATER].get_Value(), 2);
			strcpy(buf, " Tank: "); buf += 7;
			buf = dptoa(buf, MC.sADC[LTANK].get_Value() / 100, 0);
			buffer_space_padding(buf, LCD_COLS - (buf - buffer));
			lcd.print(buffer);

			lcd.setCursor(0, 2); vTaskDelay(1);
			strcpy(buf = buffer, "Days,Fe:"); buf += 8;
			tmp = MC.WorkStats.DaysFromLastRegen;
			if(tmp < 100) *buf++ = ' ';
			buf = dptoa(buf, tmp, 0);
			strcpy(buf = buffer, " Soft:"); buf += 6;
			tmp = MC.WorkStats.DaysFromLastRegenSoftening;
			if(tmp < 100) *buf++ = ' ';
			buf = dptoa(buf, tmp, 0);
			buffer_space_padding(buf, LCD_COLS - (buf - buffer));
			LCD_print(buffer);

			lcd.setCursor(0, 3); vTaskDelay(1);
			strcpy(buf = buffer, "Day:"); buf += 4;
			tmp = MC.RTC_store.UsedToday;
			if(tmp < 10000) *buf++ = ' ';
			buf = dptoa(buf, tmp, 3);
			strcpy(buf = buffer, " Yd:"); buf += 4;
			tmp = MC.WorkStats.UsedYesterday;
			if(tmp < 10000) *buf++ = ' ';
			buf = dptoa(buf, tmp, 3);
			buffer_space_padding(buf, LCD_COLS - (buf - buffer));
			lcd.print(buffer);

			DisplayTick = xTaskGetTickCount();
		}
		vTaskDelay(10);
	}

	vTaskDelete( NULL );
}

//////////////////////////////////////////////////////////////////////////
// Задача чтения датчиков
void vReadSensor(void *)
{ //const char *pcTaskName = "ReadSensor\r\n";
	static unsigned long readPWM = 0;
	static uint32_t ttime;
	static uint32_t oldTime = millis();
	static uint8_t  prtemp = 0;
	
	for(;;) {
		int8_t i;
		//WDT_Restart(WDT);

		ttime = millis();
#ifdef RADIO_SENSORS		
		radio_timecnt++;
#endif		
		if(OW_scan_flags == 0) {
			prtemp = MC.Prepare_Temp(0);
#ifdef ONEWIRE_DS2482_SECOND
			prtemp |= MC.Prepare_Temp(1);
#endif
#ifdef ONEWIRE_DS2482_THIRD
			prtemp |= MC.Prepare_Temp(2);
#endif
#ifdef ONEWIRE_DS2482_FOURTH
			prtemp |= MC.Prepare_Temp(3);
#endif
		}
		// read in vPumps():
		//for(i = 0; i < ANUMBER; i++) MC.sADC[i].Read();                  // Прочитать данные с датчиков давления
	#ifdef USE_UPS
		if(!MC.NO_Power)
	#endif
			MC.dPWM.get_readState(0); // Основная группа регистров, включая мощность
		// read in vPumps():
		//for(i = 0; i < INUMBER; i++) MC.sInput[i].Read();                // Прочитать данные сухой контакт
		for(i = 0; i < FNUMBER; i++) MC.sFrequency[i].Read();			// Получить значения датчиков потока

		// Flow
		if(!MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) {
			TimeFeedPump +=	(uint32_t)MC.sFrequency[FLOW].get_Value() * TIME_READ_SENSOR / MC.Option.FeedPumpMaxFlow;
		}
		uint32_t passed = MC.sFrequency[FLOW].Passed;
		MC.sFrequency[FLOW].Passed = 0;
		if(MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input() || MC.sInput[REG2_ACTIVE].get_Input()) {
			MC.RTC_store.UsedRegen += passed;
			Stats_WaterRegen_work += passed;
			History_WaterRegen_work += passed;
			NeedSaveRTC |= (1<<bRTC_UsedRegen);
		} else {
			MC.RTC_store.UsedToday += passed;
			History_WaterUsed_work += passed;
			NeedSaveRTC |= (1<<bRTC_UsedToday);
		}
		//
		Weight_NeedRead = true;
		vReadSensor_delay8ms((cDELAY_DS1820 - (millis() - ttime)) / 8); 	// Ожидать время преобразования
		if(Weight_Percent <= MC.Option.Weight_Empty) {
			set_Error(ERR_WEIGHT_LOW, (char*)__FUNCTION__);
		}

		if(OW_scan_flags == 0) {
			uint8_t flags = 0;
			for(i = 0; i < TNUMBER; i++) {                                   // Прочитать данные с температурных датчиков
				if((prtemp & (1<<MC.sTemp[i].get_bus())) == 0) {
					if(MC.sTemp[i].Read() == OK) flags |= MC.sTemp[i].get_setup_flags();
				}
				_delay(1);     												// пауза
			}
			/* do not need averaging...
			int32_t temp;
			if(GETBIT(flags, fTEMP_as_TIN_average)) { // Расчет средних датчиков для TIN
				temp = 0;
				uint8_t cnt = 0;
				for(i = 0; i < TNUMBER; i++) {
					if(MC.sTemp[i].get_setup_flag(fTEMP_as_TIN_average) && MC.sTemp[i].get_Temp() != STARTTEMP) {
						temp += MC.sTemp[i].get_Temp();
						cnt++;
					}
				}
				if(cnt) temp /= cnt; else temp = STARTTEMP;
			} else temp = STARTTEMP;
			int16_t temp2 = temp;
			if(GETBIT(flags, fTEMP_as_TIN_min)) { // Выбор минимальной температуры для TIN
				for(i = 0; i < TNUMBER; i++) {
					if(MC.sTemp[i].get_setup_flag(fTEMP_as_TIN_min) && temp2 > MC.sTemp[i].get_Temp()) temp2 = MC.sTemp[i].get_Temp();
				}
			}
			if(temp2 != STARTTEMP) MC.sTemp[TIN].set_Temp(temp2);
		    */ // do not need averaging...
		}

	#ifdef USE_UPS
		if(!MC.NO_Power)
	#endif
			if(millis() - readPWM > PWM_READ_PERIOD) {
				readPWM=millis();
				MC.dPWM.get_readState(1);     // Последняя группа регистров
			}

		MC.calculatePower();  // Расчет мощностей
		Stats.Update();

		vReadSensor_delay8ms((TIME_READ_SENSOR - (millis() - ttime)) / 2 / 8);     // 1. Ожидать время нужное для цикла чтения

		//  Синхронизация часов с I2C часами если стоит соответствующий флаг
		if(MC.get_updateI2C())  // если надо обновить часы из I2c
		{
			if(millis() - oldTime > (uint32_t)TIME_I2C_UPDATE) // время пришло обновляться надо Период синхронизации внутренних часов с I2C часами (сек)
			{
				oldTime = rtcSAM3X8.unixtime();
				uint32_t t = TimeToUnixTime(getTime_RtcI2C());       // Прочитать время из часов i2c тут проблема
				if(t) {
					rtcSAM3X8.set_clock(t);                		 // Установить внутренние часы по i2c
					int32_t dt = t > oldTime ? t - oldTime : -(oldTime - t);
					MC.updateDateTime(dt);  // Обновить переменные времени с новым значением часов
					journal.printf("Sync from I2C RTC: %s %s (%d)\n", NowDateToStr(), NowTimeToStr(), dt);
				} else {
					journal.printf("Error read I2C RTC\n");
				}
				oldTime = millis();
			}
		}
		// Проверки граничных температур для уведомлений, если разрешено!
		static uint8_t last_life_h = 255;
		if(MC.message.get_fMessageLife()) // Подача сигнала жизни если разрешено!
		{
			uint8_t hour = rtcSAM3X8.get_hours();
			if(hour == HOUR_SIGNAL_LIFE && hour != last_life_h) {
				MC.message.setMessage(pMESSAGE_LIFE, (char*) "Контроллер работает . . .", 0);
			}
			last_life_h = hour;
		}
		//
		vReadSensor_delay8ms((TIME_READ_SENSOR - (millis() - ttime)) / 8);     // Ожидать время нужное для цикла чтения
		ttime = TIME_READ_SENSOR - (millis() - ttime);
		if(ttime && ttime <= 8) vTaskDelay(ttime);

	}  // for
	vTaskDelete( NULL);
}

// Вызывается во время задержек в задаче чтения датчиков
void vReadSensor_delay8ms(int16_t ms8)
{
	do {
		if(ms8) vTaskDelay(8);

		if(Weight_NeedRead && Weight.is_ready()) {
			Weight_NeedRead = false;
			// Read HX711
			int32_t adc_val = Weight.read();
			// Усреднение значений
			Weight_adc_sum = Weight_adc_sum + adc_val - Weight_adc_filter[Weight_adc_idx];
			Weight_adc_filter[Weight_adc_idx] = adc_val;
			if(Weight_adc_idx < sizeof(Weight_adc_filter) / sizeof(Weight_adc_filter[0]) - 1) Weight_adc_idx++;
			else {
				Weight_adc_idx = 0;
				Weight_adc_flagFull = true;
			}
			if(Weight_adc_flagFull) adc_val = Weight_adc_sum / (sizeof(Weight_adc_filter) / sizeof(Weight_adc_filter[0])); else adc_val = Weight_adc_sum / Weight_adc_idx;
			Weight_Percent = (Weight_value = (adc_val - MC.Option.WeightZero) * 10000 / MC.Option.WeightScale - MC.Option.WeightTare) * 10000 / MC.Option.WeightFull;
			if(Weight_Percent < 0) Weight_Percent = 0; else if(Weight_Percent > 10000) Weight_Percent = 10000;
		}

#ifdef USE_UPS
		MC.sInput[SPOWER].Read(true);
		if(MC.sInput[SPOWER].is_alarm()) { // Электричество кончилось
			if(!MC.NO_Power) {
				MC.save_WorkStats();
				Stats.SaveStats(0);
				Stats.SaveHistory(0);
				journal.jprintf(pP_DATE, "Power lost!\n");
			}
			MC.NO_Power_delay = NO_POWER_ON_DELAY_CNT;
		} else if(MC.NO_Power) { // Включаемся
			if(MC.NO_Power_delay) {
				if(--MC.NO_Power_delay == 0) MC.fNetworkReset = true;
			} else {
				journal.jprintf(pP_DATE, "Power restored!\n");
				MC.NO_Power = 0;
			}
		}
#endif
#ifdef RADIO_SENSORS
		check_radio_sensors();
#endif
	} while(--ms8 > 0);
}

//////////////////////////////////////////////////////////////////////////
// Задача Управления насосами (MC.xHandlePumps) "Pumps"
void vPumps( void * )
{
	while(!ADC_has_been_read) vTaskDelay(1000 / ADC_FREQ + TIME_SLICE_PUMPS); // ms
	for(;;)
	{
		WDT_Restart(WDT); // Reset Watchdog here, most important task.
		if(WaterBoosterStatus != 0) {
			Stats_WaterBooster_work += TIME_SLICE_PUMPS;
			History_WaterBooster_work += TIME_SLICE_PUMPS;
		}
		if((WaterBoosterTimeout += TIME_SLICE_PUMPS) < TIME_SLICE_PUMPS) WaterBoosterTimeout = 0xFFFFFFFF;
		for(uint8_t i = 0; i < RNUMBER; i++) MC.dRelay[i].NextTimerOn();

		// Read sensors
		for(uint8_t i = 0; i < INUMBER; i++) MC.sInput[i].Read(true);		// Прочитать данные сухой контакт
		TankEmpty = MC.sInput[TANK_EMPTY].get_Input();
		if(ADC_has_been_read) {		// Не чаще, чем ADC
			ADC_has_been_read = false;
			for(uint8_t i = 0; i < ANUMBER; i++) MC.sADC[i].Read();			// Прочитать данные с датчиков давления
			if(MC.sADC[LTANK].get_Value() < MC.Option.LTANK_Empty) TankEmpty = true;
		}
		if(TankEmpty) vPumpsNewError = ERR_TANK_EMPTY;

		// Check Errors
		int16_t press = MC.sADC[PWATER].get_Value();
		if(press == ERROR_PRESS) {
			if(WaterBoosterStatus) {
				vPumpsNewError = ERR_PRESS;
				if(MC.dRelay[RBOOSTER2].get_Relay()) {
					MC.dRelay[RBOOSTER1].set_ON();
					_delay(10);
					MC.dRelay[RBOOSTER2].set_OFF();
					_delay(20);
				}
				MC.dRelay[RBOOSTER1].set_OFF();
				WaterBoosterTimeout = 0;
				WaterBoosterStatus = 0;
			}
		} else if(MC.sInput[FLOODING].get_Input()) {
			if(FloodingTime == 0) FloodingTime = millis();
			else if(millis() - FloodingTime > (uint32_t) MC.Option.FloodingDebounceTime * 1000) {
				FloodingTime = millis() | 1;
				vPumpsNewError = ERR_FLOODING;
				if(MC.dRelay[RFILL].get_Relay()) MC.dRelay[RFILL].set_OFF();
				if(MC.dRelay[RDRAIN].get_Relay()) MC.dRelay[RDRAIN].set_OFF();
				MC.dRelay[RFEEDPUMP].set_OFF();
				MC.dRelay[RWATEROFF].set_ON();
				FloodingError = true;
				if(WaterBoosterStatus > 0) goto xWaterBooster_OFF;
			}
		} else {
			if(FloodingError) {
				if(FloodingTime - rtcSAM3X8.unixtime() > MC.Option.FloodingTimeout) {
					MC.dRelay[RWATEROFF].set_OFF();
					FloodingError = false;
					FloodingTime = 0;
				}
			} else FloodingTime = 0;
		}
		if((!FloodingError && MC.get_errcode() == ERR_FLOODING) || (!TankEmpty && MC.get_errcode() == ERR_TANK_EMPTY)) MC.eraseError();

		// Water Booster
		if(!WaterBoosterStatus && !WaterBoosterError && !FloodingError && press != ERROR_PRESS && !TankEmpty
				&& WaterBoosterTimeout >= MC.Option.MinWaterBoostOffTime
				&& press <= (MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input() || MC.sInput[REG2_ACTIVE].get_Input() ? MC.Option.PWATER_RegMin : MC.sADC[PWATER].get_minValue())) { // Starting
			MC.dRelay[RBOOSTER1].set_ON();
			WaterBoosterTimeout = 0;
			WaterBoosterStatus = 1;
		} else if(WaterBoosterStatus > 0) {
			if(WaterBoosterTimeout > MC.Option.PWM_StartingTime) {
				if(MC.Option.PWM_DryRun && MC.dPWM.get_Power() < MC.Option.PWM_DryRun) { // Сухой ход
					WaterBoosterError = true;
					vPumpsNewError = ERR_PWM_DRY_RUN;
					goto xWaterBooster_OFF;
				} else if(MC.Option.PWM_Max && MC.dPWM.get_Power() > MC.Option.PWM_Max) { // Перегрузка
					WaterBoosterError = true;
					vPumpsNewError = ERR_PWM_MAX;
					goto xWaterBooster_OFF;
				}
			}
			if(WaterBoosterTimeout >= MC.Option.MinWaterBoostOnTime && (press >= MC.sADC[PWATER].get_maxValue() || TankEmpty)) { // Stopping
xWaterBooster_OFF:
				if(WaterBoosterStatus == 1) {
					MC.dRelay[RBOOSTER1].set_OFF();
					WaterBoosterStatus = 0;
				} else if(WaterBoosterStatus == 2) {
					MC.dRelay[RBOOSTER2].set_OFF();
					WaterBoosterStatus = -1;
				} else { // Off full cycle
					MC.dRelay[RBOOSTER1].set_ON();
					WaterBoosterStatus = -2;
				}
			} else if(WaterBoosterStatus == 1) {
				MC.dRelay[RBOOSTER2].set_ON();
				WaterBoosterStatus = 2;
			} else if(WaterBoosterStatus == 2) {
				MC.dRelay[RBOOSTER1].set_OFF();
				WaterBoosterStatus = 3;
			}
		} else if(WaterBoosterStatus == -2) { // Start all off
			MC.dRelay[RBOOSTER2].set_OFF();
			WaterBoosterStatus = -1;
		} else if(WaterBoosterStatus == -1) {
			MC.dRelay[RBOOSTER1].set_OFF();
			WaterBoosterTimeout = 0;
			WaterBoosterStatus = 0;
		}

		// Feed Pump
		if(MC.dRelay[RFEEDPUMP].get_Relay()) {
			taskENTER_CRITICAL();
			if(TimeFeedPump >= TIME_SLICE_PUMPS) {
				TimeFeedPump -= TIME_SLICE_PUMPS;
				Stats_FeedPump_work += TIME_SLICE_PUMPS;
				History_FeedPump_work += TIME_SLICE_PUMPS;
			}
			taskEXIT_CRITICAL();
			if(TimeFeedPump < TIME_SLICE_PUMPS) MC.dRelay[RFEEDPUMP].set_OFF();
		} else if(TimeFeedPump >= MC.Option.MinPumpOnTime) {
			MC.dRelay[RFEEDPUMP].set_ON();
		}
		// Fill tank
#ifndef TANK_ANALOG_LEVEL
		if(MC.sInput[TANK_LOW].get_Input()) {
			if(MC.dRelay[RFILL].get_Relay()) {
#else
		if(MC.sADC[LTANK].get_Value() <= MC.sADC[LTANK].get_minValue()) {
			if(MC.dRelay[RFILL].get_Relay()) {
				if(MC.Option.FillingTankTimeout) {
					if(WaterBoosterStatus == 0 && TimerDrainingWater == 0 && !MC.sInput[REG_ACTIVE].get_Input() && !MC.sInput[REG_BACKWASH_ACTIVE].get_Input() && !MC.sInput[REG2_ACTIVE].get_Input()) { // No water consuming
						if(millis() - FillingTankTimer >= (uint32_t) MC.Option.FillingTankTimeout * 1000) {
							if(FillingTankLastLevel + FILLING_TANK_STEP > MC.sADC[LTANK].get_Value()) vPumpsNewError = ERR_TANK_NO_FILLING;
							FillingTankLastLevel = MC.sADC[LTANK].get_Value();
							FillingTankTimer = millis();
						}
					} else FillingTankTimer = millis();
				}
#endif
				taskENTER_CRITICAL();
				Charts_FillTank_work += TIME_SLICE_PUMPS * 100 / 1000; // in percent
				taskEXIT_CRITICAL();
			} else if(!FloodingError) {
				MC.dRelay[RFILL].set_ON();	// Start filling tank
				FillingTankLastLevel = MC.sADC[LTANK].get_Value();
				FillingTankTimer = millis();
			}
#ifdef TANK_ANALOG_LEVEL
		} else if(MC.sADC[LTANK].get_Value() >= MC.sADC[LTANK].get_maxValue()) {
#else
		}
		if(MC.sInput[TANK_FULL].get_Input()) {
#endif
			if(MC.dRelay[RFILL].get_Relay()) MC.dRelay[RFILL].set_OFF();	// Stop filling tank
		}

		// Regenerating
		if(MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) { // Regen Iron removing filter
			if(MC.dRelay[RSTARTREG].get_Relay()) MC.dRelay[RSTARTREG].set_OFF();
			if(!(MC.RTC_store.Work & RTC_Work_Regen_F1)) {
				MC.RTC_store.Work |= RTC_Work_Regen_F1;
				MC.dRelay[RWATEROFF].set_ON();
				NewRegenStatus = true;
				MC.RTC_store.UsedRegen = 0;
				NeedSaveRTC = RTC_SaveAll;
			}
		} else if(MC.sInput[REG2_ACTIVE].get_Input()) {	// Regen Softening filter
			if(MC.dRelay[RSTARTREG2].get_Relay()) MC.dRelay[RSTARTREG2].set_OFF();
			if(!(MC.RTC_store.Work & RTC_Work_Regen_F2)) {
				MC.RTC_store.Work |= RTC_Work_Regen_F2;
				NewRegenStatus = true;
				MC.RTC_store.UsedRegen = 0;
				NeedSaveRTC = RTC_SaveAll;
			}
		}

		// Drain OFF
		if(TimerDrainingWater && GetTickCount() - TimerDrainingWater >= (uint32_t) MC.Option.DrainTime * 1000) {
			MC.dRelay[RDRAIN].set_OFF();
			if(MC.RTC_store.UsedToday < MC.Option.MinDrainLiters) {
				if(vPumpsNewError == OK) {
					vPumpsNewError = ERR_FEW_LITERS_DRAIN;
					TimerDrainingWater = 0;
				}
			} else TimerDrainingWater = 0;
		}

		vTaskDelay(TIME_SLICE_PUMPS); // ms
	}
	vTaskDelete( NULL );
}

// Service ///////////////////////////////////////////////
// Графики в ОЗУ, счетчики моточасов, сохранение статистики, дисплей
void vService(void *)
{
	static uint8_t  task_updstat_countm = rtcSAM3X8.get_minutes();
	static uint8_t  task_every_min = task_updstat_countm;
	static uint32_t timer_sec = GetTickCount();

	for(;;) {
		if(vPumpsNewError != 0) {
			set_Error(vPumpsNewError, (char*)"vPumps");
			vPumpsNewError = 0;
		}
		register uint32_t t = GetTickCount();
		if(t - timer_sec >= 1000) { // 1 sec
			timer_sec = t;

			if(++task_updstat_chars >= MC.get_tChart()) { // пришло время
				task_updstat_chars = 0;
				MC.updateChart();                                       // Обновить графики
			}
			uint8_t m = rtcSAM3X8.get_minutes();
			if(m != task_updstat_countm) { 								// Через 1 минуту
				task_updstat_countm = m;
				MC.updateCount();                                       // Обновить счетчики
				if(task_updstat_countm == 59) MC.save_WorkStats();		// сохранить раз в час
				Stats.History();                                        // запись истории в файл

				if((MC.RTC_store.Work & RTC_Work_WeekDay_MASK) != rtcSAM3X8.get_day_of_week()) { // Next day
					uint32_t ut;
					{
						vTaskSuspendAll(); // запрет других задач
						MC.RTC_store.Work = (MC.RTC_store.Work & ~RTC_Work_WeekDay_MASK) | rtcSAM3X8.get_day_of_week();
						ut = MC.RTC_store.UsedToday;
						MC.RTC_store.UsedToday = 0;
						MC.WorkStats.DaysFromLastRegen++;
						MC.WorkStats.DaysFromLastRegenSoftening++;
						MC.WorkStats.UsedYesterday = ut;
						MC.WorkStats.UsedSinceLastRegen += ut;
						MC.WorkStats.UsedSinceLastRegenSoftening += ut;
						MC.WorkStats.UsedTotal += ut;
						if(ut > 10) {
							MC.WorkStats.UsedAverageDay += ut;
							MC.WorkStats.UsedAverageDayNum++;
						}
						NeedSaveWorkStats = 1;
						NeedSaveRTC = RTC_SaveAll;
						xTaskResumeAll(); // Разрешение других задач
						update_RTC_store_memory();
					}
					if(MC.Option.DrainTime && MC.WorkStats.UsedYesterday < 10 && MC.get_errcode() == OK) { // Used less than 10 liters
						TimerDrainingWater = GetTickCount() | 1;
						MC.dRelay[RDRAIN].set_ON();
					}
				}

				if(MC.dRelay[RSTARTREG].get_Relay() && !(MC.RTC_store.Work & RTC_Work_Regen_F1)) { // 1 minute passed but regeneration did not start
					set_Error(ERR_START_REG, (char*)__FUNCTION__);
				}
				if(MC.dRelay[RSTARTREG2].get_Relay() && !(MC.RTC_store.Work & RTC_Work_Regen_F2)) { // 1 minute passed but regeneration did not start
					set_Error(ERR_START_REG2, (char*)__FUNCTION__);
				}
				if(!WaterBoosterError && !FloodingError && !TankEmpty && MC.get_errcode() != ERR_START_REG && MC.get_errcode() != ERR_START_REG2
						&& !(MC.RTC_store.Work & RTC_Work_Regen_MASK) && rtcSAM3X8.get_hours() == MC.Option.RegenHour) {
					uint32_t need_regen = 0;
					if((MC.Option.DaysBeforeRegen && MC.WorkStats.DaysFromLastRegen >= MC.Option.DaysBeforeRegen) || (MC.Option.UsedBeforeRegen && MC.WorkStats.UsedSinceLastRegen + MC.RTC_store.UsedToday >= MC.Option.UsedBeforeRegen))
						need_regen |= RTC_Work_Regen_F1;
					else if(MC.Option.UsedBeforeRegenSoftener && MC.WorkStats.UsedSinceLastRegenSoftening + MC.RTC_store.UsedToday >= MC.Option.UsedBeforeRegenSoftener)
						need_regen |= RTC_Work_Regen_F2;
					if(need_regen) {
#ifdef TANK_ANALOG_LEVEL
						if(MC.sADC[LTANK].get_Value() >= MC.sADC[LTANK].get_maxValue()) {
#else
						if(MC.sInput[TANK_FULL].get_Input()) {
#endif
							if((need_regen & RTC_Work_Regen_F1) && !MC.dRelay[RSTARTREG].get_Relay()) {
								journal.jprintf(pP_DATE, "Regen F1 start\n");
								MC.dRelay[RWATEROFF].set_ON();
								MC.dRelay[RSTARTREG].set_ON();
							} else if((need_regen & RTC_Work_Regen_F2) && !MC.dRelay[RSTARTREG2].get_Relay()) {
								journal.jprintf(pP_DATE, "Regen F2 start\n");
								MC.dRelay[RSTARTREG2].set_ON();
							}
						} else {
							MC.dRelay[RFILL].set_ON();	// Start filing tank
						}
					}
				}
			} else {
				if(NeedSaveWorkStats) {
					if(MC.save_WorkStats() == OK) NeedSaveWorkStats = 0;
				} else if((NeedSaveRTC & (1<<bRTC_Urgently)) || (NeedSaveRTC && m != task_every_min)) {
					task_every_min = m;
					uint8_t err = update_RTC_store_memory();
					if(err) {
						journal.printf("Error %d save RTC!\n", err);
						set_Error(ERR_RTC_WRITE, (char*)__FUNCTION__);
					}
				} else {
					if(MC.RTC_store.Work & RTC_Work_Regen_F1) {	// Regen Iron removing filter
						if(!MC.sInput[REG_ACTIVE].get_Input() && !MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) {
							MC.WorkStats.UsedLastRegen = MC.RTC_store.UsedRegen;
							MC.RTC_store.UsedRegen = 0;
							MC.WorkStats.DaysFromLastRegen = 0;
							MC.WorkStats.RegCnt++;
							MC.WorkStats.UsedSinceLastRegen = 0;
							MC.RTC_store.Work &= ~RTC_Work_Regen_F1;
							taskENTER_CRITICAL();
							NeedSaveRTC |= (1<<bRTC_Work) | (1<<bRTC_UsedRegen) | (1<<bRTC_Urgently);
							taskEXIT_CRITICAL();
							NeedSaveWorkStats = 1;
							MC.dRelay[RWATEROFF].set_OFF();
							journal.jprintf(pP_DATE, "Regen F1 finished.\n");
							if(MC.WorkStats.UsedLastRegen < MC.Option.MinRegenLiters) {
								set_Error(ERR_FEW_LITERS_REG, (char*)__FUNCTION__);
							}
						} else if(NewRegenStatus) {
							journal.jprintf(pP_DATE, "Regen F1 begin\n");
							NewRegenStatus = false;
						}
					} else if(MC.RTC_store.Work & RTC_Work_Regen_F2) { // Regen Softening filter
						if(!MC.sInput[REG2_ACTIVE].get_Input()) {
							MC.WorkStats.UsedLastRegenSoftening = MC.RTC_store.UsedRegen;
							MC.RTC_store.UsedRegen = 0;
							MC.WorkStats.DaysFromLastRegenSoftening = 0;
							MC.WorkStats.RegCntSoftening++;
							MC.WorkStats.UsedSinceLastRegenSoftening = 0;
							MC.RTC_store.Work &= ~RTC_Work_Regen_F2;
							taskENTER_CRITICAL();
							NeedSaveRTC |= (1<<bRTC_Work) | (1<<bRTC_UsedRegen) | (1<<bRTC_Urgently);
							taskEXIT_CRITICAL();
							NeedSaveWorkStats = 1;
							journal.jprintf(pP_DATE, "Regen F2 finished.\n");
						} else if(NewRegenStatus) {
							journal.jprintf(pP_DATE, "Regen F2 begin\n");
							NewRegenStatus = false;
						}
					}
				}
			}
			Stats.CheckCreateNewFile();
			if(ResetDUE_countdown && --ResetDUE_countdown == 0) Software_Reset();      // Сброс
		}
		vTaskDelay(1); // задержка чтения уменьшаем загрузку процессора
	}
	vTaskDelete(NULL);
}
