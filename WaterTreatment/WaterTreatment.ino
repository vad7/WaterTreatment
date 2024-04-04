/*
 * Copyright (c) 2020-2022 by Vadim Kulakov vad7@yahoo.com, vad711
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
//EthernetClient ethClient(W5200_SOCK_SYS);           // для MQTT
// для set_time_NTP() и Send_HTTP_Request()

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
#ifdef USE_RC_CLOCK_SOURCE
RTC_clock rtcSAM3X8(RC);                                               // Внутренние часы, используется внутренний RC генератор
#else
RTC_clock rtcSAM3X8(XTAL);                                               // Внутренние часы, используется часовой кварц
#endif
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

void vWeb0(void *) __attribute__((naked));
void vWeb1(void *) __attribute__((naked));
void vWeb2(void *) __attribute__((naked));
void vKeysLCD( void * ) __attribute__((naked));
void vReadSensor(void *) __attribute__((naked));
void vPumps( void * ) __attribute__((naked));
void vService(void *) __attribute__((naked));

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
	// Отключить питание (VUSB) на Native USB
	Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSHWC);
	PIO_Configure(PIOB, PIO_OUTPUT_0, PIO_PB10A_UOTGVBOF, PIO_DEFAULT);
	PIOB->PIO_CODR = PIO_PB10A_UOTGVBOF; // =0
	//
	// Борьба с зависшими устройствами на шине  I2C (в первую очередь часы) неудачный сброс
	Recover_I2C_bus();
	delay(1);

	// Настройка сервисных выводов
	pinMode(PIN_KEY_SAFE, INPUT_PULLUP);    // Кнопка 1, Нажатие при включении - режим safeNetwork (настрока сети по умолчанию 192.168.0.177  шлюз 192.168.0.1, не спрашивает пароль на вход в веб морду)
	pinMode(PIN_BEEP, OUTPUT);              // Выход на пищалку
	pinMode(PIN_LED_OK, OUTPUT);            // Выход на светодиод мигает 0.5 герца - ОК  с частотой 2 герца ошибка
#ifdef PIN_LED_SRV_INFO
	pinMode(PIN_LED_SRV_INFO, OUTPUT);
	digitalWriteDirect(PIN_LED_SRV_INFO, LOW);
#endif
	digitalWriteDirect(PIN_BEEP,LOW);       // Выключить пищалку
	digitalWriteDirect(PIN_LED_OK,HIGH);    // Включить светодиод

	// 2. Инициализация журнала и в нем последовательный порт
	// 2. Инициализация журнала
	uint8_t b;
	uint8_t ret = eepromI2C.read(I2C_COUNT_EEPROM, &b, 1);
	if(ret == 0 && b != I2C_COUNT_EEPROM_HEADER && b != 0xFF) {
		ret = 0xFF;
	}
	DebugToJournalOn = ret;
#ifndef DEBUG
	if(ret)
#endif
#ifndef DEBUG_NATIVE_USB
		SerialDbg.begin(UART_SPEED);                   // Если надо инициализировать отладочный порт
#endif
	while(ret) {
		SerialDbg.print("Wrong I2C EEPROM or setup, press KEY[D");
		SerialDbg.print(PIN_KEY_SAFE);
		SerialDbg.println("] to continue...");
		WDT_Restart(WDT);
		if(!digitalReadDirect(PIN_KEY_SAFE)) {
			b = I2C_COUNT_EEPROM_HEADER;
			ret = eepromI2C.write(I2C_COUNT_EEPROM, &b, 1);
			if(ret) {
				SerialDbg.print("Error ");
				SerialDbg.print(ret);
				SerialDbg.println(" write to EEPROM!");
			} else SerialDbg.println("Wait...");
#ifdef TEST_BOARD
			break;
#else
			while(1) ;
#endif
		}
		for(uint8_t i = 0; i < 1000 / TIME_LED_ERR; i++) {
			digitalWriteDirect(PIN_BEEP, i & 1);
			digitalWriteDirect(PIN_LED_OK, i & 1);
			delay(TIME_LED_ERR);
		}
	}
	journal.Init();
	if(!DebugToJournalOn) {
		uint16_t flags;
		if(readEEPROM_I2C(I2C_SETTING_EEPROM + 2 + (sizeof(MC.Option) < 128 ? 1 : 2) + ((uint8_t*)&MC.Option.flags - &MC.Option.ver), (uint8_t*)&flags, sizeof(flags))) {
			DebugToJournalOn = true;
			journal.printf("\nError read EEPROM!\n");
		} else DebugToJournalOn = GETBIT(flags, fDebugToJournal);
	}
#ifdef TEST_BOARD
	DebugToJournalOn = true;
	journal.jprintf("\n---> TEST BOARD!!!\n\n");
#endif

	//journal.jprintfopt("Firmware version: %s\n", VERSION);
	//showID();                                                                  // информация о чипе
	//getIDchip((char*)Socket[0].inBuf);
	//journal.jprintfopt("Chip ID SAM3X8E: %s\n", Socket[0].inBuf);// информация об серийном номере чипа
	if(GPBR->SYS_GPBR[0] & 0x80000000) GPBR->SYS_GPBR[0] = 0; else GPBR->SYS_GPBR[0] |= 0x80000000; // очистка старой причины
	lastErrorFreeRtosCode = GPBR->SYS_GPBR[0] & 0x7FFFFFFF;         // Сохранение кода ошибки при старте (причину перегрузки)
	journal.jprintf("\n# RESET: %s, 0x%04x", ResetCause(), lastErrorFreeRtosCode);
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
	journal.jprintfopt("* Init I2C devices:\n");

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
			journal.jprintfopt("$ERROR - I2C mem init failed on %dkHz, status = %d\n",twiClock100kHz/1000,eepStatus);
			eepromI2C.begin(I2C_SPEED);
			goto x_I2C_init_std_message;
		} else journal.jprintfopt("I2C bus low speed, init on %dkHz - OK\n",twiClock100kHz/1000);
	} else {
x_I2C_init_std_message:
		journal.jprintfopt("I2C init on %dkHz - OK\n",I2C_SPEED/1000);
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
				journal.jprintfopt("I2C found at %s",byteToHex(address));
				switch (address)
				{
#ifdef ONEWIRE_DS2482
				case I2C_ADR_DS2482_4:
				case I2C_ADR_DS2482_3:
				case I2C_ADR_DS2482_2:
				case I2C_ADR_DS2482:  		journal.jprintfopt(" - 1-Wire DS2482-100 bus: %d%s\n", address - I2C_ADR_DS2482 + 1, (ONEWIRE_2WAY & (1<<(address - I2C_ADR_DS2482))) ? " (2W)" : ""); break;
#endif
#if I2C_FRAM_MEMORY == 1
				case I2C_ADR_EEPROM:	journal.jprintfopt(" - FRAM FM24V%02d\n", I2C_MEMORY_TOTAL*10/1024); break;
#if I2C_MEMORY_TOTAL != I2C_SIZE_EEPROM
				case I2C_ADR_EEPROM+1:	journal.jprintfopt(" - FRAM second 64k page\n"); break;
#endif
#else
				case I2C_ADR_EEPROM:	journal.jprintfopt(" - EEPROM AT24C%d\n", I2C_SIZE_EEPROM);break; // 0x50 возможны варианты
#if I2C_MEMORY_TOTAL != I2C_SIZE_EEPROM
				case I2C_ADR_EEPROM+1:	journal.jprintfopt(" - EEPROM second 64k page\n"); break;
#endif
#endif
				case I2C_ADR_RTC   :		journal.jprintfopt(" - RTC DS3231\n"); break; // 0x68
				default            :		journal.jprintfopt(" - Unknow\n"); break; // не определенный тип
				}
				_delay(100);
			}
			//    else journal.jprintfopt("I2C device bad endTransmission at address %s code %d",byteToHex(address), error);
		} // for
	} //  (eepStatus==0)

#ifndef ONEWIRE_DS2482         // если нет моста
	if(OneWireBus.Init()) journal.jprintf("Error init 1-Wire: %d\n", OneWireBus.GetLastErr());
	else journal.jprintfopt("1-Wire init Ok\n");
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
	journal.jprintfopt("* Init %s\n",(char*)nameMainClass);
	MC.init();                           // Основной класс

	// 5. Проверка кнопки Safe Network

	MC.safeNetwork=!digitalReadDirect(PIN_KEY_SAFE);
	if(MC.safeNetwork) journal.jprintf("* Mode safe Network ON\n");

	// 6. Чтение ЕЕПРОМ
	journal.jprintfopt("* Load data from I2C\n");
	if(MC.load_WorkStats() == ERR_HEADER2_EEPROM)           // Загрузить счетчики
	{
		journal.jprintfopt("I2C memory is empty!\n");
		MC.save_WorkStats();
	} else {
		MC.load((uint8_t *)Socket[0].outBuf, 0);      // Загрузить настройки
	}
	WaterBoosterTimeout = MC.Option.MinWaterBoostOffTime;

	// обновить хеш для пользователей
	WebSec_user.hash = WebSec_admin.hash = NULL;
	calc_WebSec_hash(&WebSec_user, (char*)NAME_USER, MC.get_passUser(), Socket[0].outBuf);
	calc_WebSec_hash(&WebSec_admin, (char*)NAME_ADMIN, MC.get_passAdmin(), Socket[0].outBuf);

	// 7. Инициализация СД карты и запоминание результата 3 попытки
	journal.jprintfopt("* Init SD card\n");
	WDT_Restart(WDT);
	MC.set_fSD(initSD());
	WDT_Restart(WDT);                          // Сбросить вачдог  иногда карта долго инициализируется
	digitalWriteDirect(PIN_LED_OK,LOW);        // Включить светодиод - признак того что сд карта инициализирована
	//_delay(100);

	// 8. Инициализация spi флеш диска
#ifdef SPI_FLASH
	journal.jprintfopt("* Init SPI flash disk\n");
	MC.set_fSPIFlash(initSpiDisk(true));  // проверка диска с выводом инфо
#endif

	journal.jprintfopt(" Web source: ");
	switch (MC.get_SourceWeb())
	{
	case pMIN_WEB:   journal.jprintfopt("internal\n"); break;
	case pSD_WEB:    journal.jprintfopt("SD card\n"); break;
	case pFLASH_WEB: journal.jprintfopt("SPI Flash\n"); break;
	default:         journal.jprintfopt("unknown\n"); break;
	}

	journal.jprintfopt("* Start ADC\n");
	//journal.jprintf_date("Temp SAM3x: %.2f\n", temp_DUE());
	start_ADC(); // после инициализации
	//journal.jprintfopt(" Mask ADC_IMR: 0x%08x\n",ADC->ADC_IMR);

	// 10. Сетевые настройки
	journal.jprintfopt("* Set Network\n");
	if(initW5200(true)) {   // Инициализация сети с выводом инфы в консоль
		W5100.getMACAddress((uint8_t *)Socket[0].outBuf);
		journal.jprintfopt(" MAC: %s\n", MAC2String((uint8_t *)Socket[0].outBuf));
	}
	digitalWriteDirect(PIN_BEEP,LOW);          // Выключить пищалку
	WDT_Restart(WDT);

	// 11. Разбираемся со всеми часами и синхронизацией
	journal.jprintfopt("* Set time\n");
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
			} else journal.jprintf_date(" RTC MEM: %02X, D: %d, R: %d\n", MC.RTC_store.Work, MC.RTC_store.UsedToday, MC.RTC_store.UsedRegen);
		}
	} else journal.jprintf(" Error read RTC!\n");
	set_time();

	// 12. Инициалазация уведомлений
	//journal.jprintfopt("* Message DNS update.\n");
	//MC.message.dnsUpdate();

	// 13. Инициалазация MQTT
#ifdef MQTT
	journal.jprintfopt("* Init MQTT\n");
	MC.clMQTT.dnsUpdateStart();
#endif

	WDT_Restart(WDT);
	// Инициализация весов
	Weight.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
	Weight_Clear_Averaging();
	journal.jprintfopt("* Scale inited, ADC: %d. ", Weight.read());
	Weight_adc_median1 = Weight_adc_median2 = Weight.read();
	int cnt = 4, t = 100;
	while(cnt) {
		WDT_Restart(WDT);
		if(Weight_Read(1)) cnt--;
		delay(10);
		if(--t == 0) break;
	}
	journal.jprintfopt("Weight: %.1d g\n", Weight_value);

	// 14. Инициалазация Statistics
	journal.jprintfopt("* Statistics ");
	if(MC.get_fSD()) {
		journal.jprintfopt("writing on SD\n");
		Stats.Init();             // Инициализовать статистику
	} else journal.jprintfopt("not available\n");

#ifdef TEST_BOARD
	// Scan oneWire - TEST.
	//MC.scan_OneWire(Socket[0].outBuf);
#endif

	// Создание задач FreeRTOS  ----------------------
	journal.jprintfopt("* Create tasks FreeRTOS ");
	MC.mRTOS=236;  //расчет памяти для задач 236 - размер данных шедуллера, каждая задача требует 64 байта+ стек (он в словах!!)
	MC.mRTOS=MC.mRTOS+64+4*configMINIMAL_STACK_SIZE;  // задача бездействия
	//MC.mRTOS=MC.mRTOS+4*configTIMER_TASK_STACK_DEPTH;  // программные таймера (их теперь нет)

	// ПРИОРИТЕТ 4 Высший приоритет
	if(xTaskCreate(vPumps, "Pumps", 90, NULL, 4, &MC.xHandlePumps) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 90;
	//vTaskSuspend(MC.xHandleFeedPump);      // Остановить задачу

	// ПРИОРИТЕТ 3 Очень высокий приоритет
	if(xTaskCreate(vReadSensor, "ReadSensor", 150, NULL, 3, &MC.xHandleReadSensor) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 150;

	// ПРИОРИТЕТ 2 средний
	if(xTaskCreate(vKeysLCD, "KeysLCD", 90, NULL, 2, &MC.xHandleKeysLCD) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 90;
	if(xTaskCreate(vService, "Service", 180, NULL, 2, &MC.xHandleService) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4* 180;

	// ПРИОРИТЕТ 1 низкий - обслуживание вебморды в несколько потоков
	// ВНИМАНИЕ первый поток должен иметь больший стек для обработки фоновых сетевых задач
	// 1 - поток
	#define STACK_vWebX 200
	if(xTaskCreate(vWeb0,"Web0", STACK_vWebX+10,NULL,1,&MC.xHandleUpdateWeb0)==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) set_Error(ERR_MEM_FREERTOS,(char*)nameFREERTOS);
	MC.mRTOS=MC.mRTOS+64+4*(STACK_vWebX+10);
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
	Modbus.initModbus();
	//
	journal.jprintfopt("OK, size %d bytes\n", MC.mRTOS);

	//journal.jprintfopt("* Send a notification . . .\n");
	if(rstc_get_reset_cause(RSTC) != RSTC_SOFTWARE_RESET) {
		MC.message.setMessage(pMESSAGE_RESET,(char*)"Контроллер был перезагружен: ",0);    // сформировать уведомление о загрузке
	}
	journal.jprintfopt("* Information:\n");
	freeRamShow();
	MC.startRAM=freeRam()-MC.mRTOS;   // оценка свободной памяти до пуска шедулера, поправка на 1054 байта
	journal.jprintfopt("FREE MEMORY %d bytes\n",MC.startRAM);
	journal.jprintfopt("Temp DS2331: %.2d\n", getTemp_RtcI2C());
	if(Is_otg_vbus_high()) journal.jprintf("+USB\n");

	//MC.Stat.generate_TestData(STAT_POINT); // Сгенерировать статистику STAT_POINT точек только тестирование
	//journal.jprintfopt("Start FreeRTOS.\n\n");
	eepromI2C.use_RTOS_delay = 1;
	//
	vTaskStartScheduler();
	journal.jprintf("FreeRTOS FAILURE!\n");
}


void loop() {}

//  З А Д А Ч И -------------------------------------------------
// Это и есть поток с минимальным приоритетом измеряем простой процессора
//extern "C" 
//{
static bool Error_Beep_confirmed = false;

extern "C" void vApplicationIdleHook(void)  // FreeRTOS expects C linkage
{
	static unsigned long countLED = 0;
	static unsigned long countBeep = 0;

	WDT_Restart(WDT);                                                            // Сбросить вачдог
//	static unsigned long cpu_idle_max_count = 0; // 1566594 // максимальное значение счетчика, вычисляется при калибровке и соответствует 100% CPU idle
//	static unsigned long ulIdleCycleCount = 0;
//	ulIdleCycleCount++;                                                          // приращение счетчика
//	if(ticks - countLastTick >= 3000)		// мсек
//	{
//		countLastTick = ticks;                            // расчет нагрузки
//		if(ulIdleCycleCount > cpu_idle_max_count) cpu_idle_max_count = ulIdleCycleCount; // это калибровка запоминаем максимальные значения
//		HP.CPU_IDLE = (100 * ulIdleCycleCount) / cpu_idle_max_count;              // вычисляем текущую загрузку
//		ulIdleCycleCount = 0;
//	}
	register unsigned long ticks = GetTickCount();
	// Светодиод мигание в зависимости от ошибки и подача звукового сигнала при ошибке
	if(MC.get_errcode() != OK) {          // Ошибка
		if(ticks - countLED > TIME_LED_ERR) {
			countLED = ticks;
			bool pin = !digitalReadDirect(PIN_LED_OK);
			digitalWriteDirect(PIN_LED_OK, pin);
#ifdef PIN_LED_SRV_INFO
			digitalWriteDirect(PIN_LED_SRV_INFO, pin);
#endif
		}
		if(ticks - countBeep > TIME_BEEP_ERR) {
			countBeep = ticks;
			digitalWriteDirect(PIN_BEEP, !MC.get_Beep() || Error_Beep_confirmed ? LOW : !digitalReadDirect(PIN_BEEP)); // звуковой сигнал
			if(MC.get_errcode() == ERR_SALT_FINISH && digitalReadDirect(PIN_BEEP)) countBeep -= TIME_BEEP_ERR / 2;
		}
	} else if(ticks - countLED > TIME_LED_OK) {   // Ошибок нет и время пришло
		countLED = ticks;
		Error_Beep_confirmed = false;
		digitalWriteDirect(PIN_BEEP, LOW);
		digitalWriteDirect(PIN_LED_OK, !digitalReadDirect(PIN_LED_OK));
	}
//	static uint32_t tst = 1000;
//	uint32_t tm;
//	if(--tst < 5) tm = micros();
	pmc_enable_sleepmode(0);
//	if(tst < 5) {
//		Serial.print("$"); Serial.print(micros() - tm);	Serial.print("$\n");
//		if(tst == 0) tst = 1000;
//	}
}

// --------------------------- W E B ------------------------
// Задача обслуживания web сервера
// Сюда надо пихать все что связано с сетью иначе конфликты не избежны
// Здесь также обслуживается посылка уведомлений MQTT
// Первый поток веб сервера - дополнительно нагружен различными сервисами
void vWeb0(void *)
{ //const char *pcTaskName = "Web server is running\r\n";
	uint32_t timeResetW5200 = 0;
	uint32_t thisTime = 0;
	uint32_t resW5200 = 0;
	uint32_t iniW5200 = 0;
	uint32_t pingt = 0;
	uint32_t PIN_LED_SRV_INFO_timer = 0;
#ifdef MQTT
	uint32_t narmont=0;
	uint32_t mqttt=0;
#endif
	static boolean active;  // ФЛАГ Одно дополнительное действие за один цикл - распределяем нагрузку
	static boolean network_last_link = true;

	Request_LowConsume = (-(uint32_t)MC.Option.LowConsumeRequestPeriod) | 0x1;
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

		STORE_DEBUG_INFO(39);

		active = MC.message.dnsUpdate();                                       // Обновить адреса через dns если надо Уведомления

		STORE_DEBUG_INFO(18);

#ifdef MQTT
		if (active) active=MC.clMQTT.dnsUpdate();                          // Обновить адреса через dns если надо MQTT
#endif
		if(xTaskGetTickCount() - thisTime > (uint32_t) WEB0_OTHER_FUNC_PERIOD * 1000) { // Другие функции
			thisTime = xTaskGetTickCount();                                      // Запомнить тики
//			// 1. Проверка захваченого семафора сети ожидаем  3 времен W5200_TIME_WAIT если мютекса не получаем то сбрасывае мютекс
//			if(SemaphoreTake(xWebThreadSemaphore, ((3 + (fWebUploadingFilesTo != 0) * 30) * W5200_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
//				SemaphoreGive(xWebThreadSemaphore);
//				journal.jprintf_time("UNLOCK mutex xWebThread\n");
//				active = false;
//				MC.num_resMutexSPI++;
//			} // Захват мютекса SPI или ОЖИДАНИНЕ 2 времен W5200_TIME_WAIT и его освобождение
//			else SemaphoreGive(xWebThreadSemaphore);
//
//			// Проверка и сброс митекса шины I2C  если мютекса не получаем то сбрасывае мютекс
//			if(SemaphoreTake(xI2CSemaphore, (3 * I2C_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
//				SemaphoreGive(xI2CSemaphore);
//				journal.jprintf_time("UNLOCK mutex xI2CSemaphore\n");
//				MC.num_resMutexI2C++;
//			} // Захват мютекса I2C или ОЖИДАНИНЕ 3 времен I2C_TIME_WAIT  и его освобождение
//			else SemaphoreGive(xI2CSemaphore);

			// 2. Чистка сокетов
			if(MC.time_socketRes() > 0) {
				STORE_DEBUG_INFO(3);
				checkSockStatus();                   // Почистить старые сокеты  если эта позиция включена
			}

			// 3. Сброс сетевого чипа по времени
			if(MC.time_resW5200() > 0)                             // Сброс W5200 если включен и время подошло
			{
				STORE_DEBUG_INFO(4);
				resW5200 = xTaskGetTickCount();
				if(timeResetW5200 == 0) timeResetW5200 = resW5200;      // Первая итерация не должна быть сразу
				if(resW5200 - timeResetW5200 > MC.time_resW5200() * 1000UL) {
					journal.jprintfopt("Resetting the chip %s by timer.\n", nameWiznet);
					MC.fNetworkReset = 1;                              // Послать команду сброса и применения сетевых настроек
					timeResetW5200 = resW5200;                         // Запомить время сброса
				}
			}
			// 4. Проверка связи с чипом
			if((MC.get_fInitW5200()) && (thisTime - iniW5200 > W5500_LINK_CHECK * 1000UL)) // проверка связи с чипом сети раз в 30 сек
			{
				STORE_DEBUG_INFO(5);
				iniW5200 = thisTime;
				if(!MC.NO_Power) {
					boolean lst = linkStatusWiznet(false);
					if(!lst || !network_last_link) {
						if(!lst && network_last_link) journal.jprintf_time("%s no link[%02X] - resetting\n", nameWiznet, W5100.readPHYCFGR());
						MC.fNetworkReset = 1;           // Послать команду сброса и применения сетевых настроек
						MC.num_resW5200++;              // Добавить счетчик инициализаций
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

			if(active && Request_LowConsume && thisTime - Request_LowConsume > MC.Option.LowConsumeRequestPeriod * 1000) {
				STORE_DEBUG_INFO(8);
				Request_LowConsume = MC.Option.LowConsumeRequestPeriod ? thisTime | 0x1 : 0;
				int tmp = Send_HTTP_Request(MC.Option.LowConsumeRequest);
				if(tmp <= -2000000000 && tmp >= -2000000001) Request_LowConsume = (Request_LowConsume - (MC.Option.LowConsumeRequestPeriod > 5 ? MC.Option.LowConsumeRequestPeriod * 1000 - 5000 : 0)) | 0x1;
				if(GETBIT(MC.Option.flags, fLowConsumeReq_OnByErr) || tmp >= 0) { // if error and allowed - set low consume mode
					LowConsumeMode = tmp;
					if(!LowConsumeMode) AfterFilledTimer = 0;
				}
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

			STORE_DEBUG_INFO(9);
			// Расчет предполагаемой регенерации
			MC.NextRegenAfterDays = MC.CalcNextRegenAfterDays(MC.Option.DaysBeforeRegen, MC.WorkStats.DaysFromLastRegen, MC.Option.UsedBeforeRegen, MC.WorkStats.UsedSinceLastRegen);	// железо
			MC.NextRegenSoftAfterDays = MC.CalcNextRegenAfterDays(MC.Option.DaysBeforeRegenSoftening, MC.WorkStats.DaysFromLastRegenSoftening, MC.Option.UsedBeforeRegenSoftening, MC.WorkStats.UsedSinceLastRegenSoftening);	// умягчитель
		}
		STORE_DEBUG_INFO(19);
#ifdef PIN_LED_SRV_INFO
		if(MC.get_errcode() == OK) {
			if(!MC.dRelay[RWATERON].get_Relay()) {
				digitalWriteDirect(PIN_LED_SRV_INFO, 1);
				PIN_LED_SRV_INFO_timer = 0;
			} else if((((MC.NextRegenAfterDays == 0 || MC.NextRegenSoftAfterDays == 0) && SAM_RTC_HOUR(RTC->RTC_TIMR) >= PIN_LED_SRV_INFO_NEXT_REGEN_BEGIN_HOUR)
					 || (MC.WorkStats.Flags & (WS_F_StartRegen | WS_F_StartRegenSoft | WS_F_RegenPreparing)))
						&& GETBIT(MC.Option.flags2, fLED_SRV_INFO_PlanReg)) {
				if(PIN_LED_SRV_INFO_timer == 0) {
					digitalWriteDirect(PIN_LED_SRV_INFO, 1);
					PIN_LED_SRV_INFO_timer += PIN_LED_SRV_INFO_NEXT_REGEN_PULSE;
				} else if(xTaskGetTickCount() > PIN_LED_SRV_INFO_timer) {
					PIN_LED_SRV_INFO_timer = xTaskGetTickCount();
					if(digitalReadDirect(PIN_LED_SRV_INFO)) {
						digitalWriteDirect(PIN_LED_SRV_INFO, 0);
						PIN_LED_SRV_INFO_timer += PIN_LED_SRV_INFO_NEXT_REGEN_PAUSE;
					} else {
						digitalWriteDirect(PIN_LED_SRV_INFO, 1);
						PIN_LED_SRV_INFO_timer += PIN_LED_SRV_INFO_NEXT_REGEN_PULSE;
					}
				}
				PIN_LED_SRV_INFO_timer |= 1;
			} else {
				digitalWriteDirect(PIN_LED_SRV_INFO, 0);
				PIN_LED_SRV_INFO_timer = 0;
			}
		}
#endif
		taskYIELD();
	} //for
	vTaskSuspend(NULL);
}

// Второй поток
void vWeb1(void *)
{ //const char *pcTaskName = "Web server is running\r\n";
	for(;;) {
		web_server(1);
		vTaskDelay(TIME_WEB_SERVER / portTICK_PERIOD_MS); // задержка чтения уменьшаем загрузку процессора
	}
	vTaskSuspend(NULL);
}
// Третий поток
void vWeb2(void *)
{ //const char *pcTaskName = "Web server is running\r\n";
	for(;;) {
		web_server(2);
		vTaskDelay(TIME_WEB_SERVER / portTICK_PERIOD_MS); // задержка чтения уменьшаем загрузку процессора
	}
	vTaskSuspend(NULL);
}

//////////////////////////////////////////////////////////////////////////
#define LCD_SetupFlag 			0x80000000
#define LCD_SetupMenuItems		5
#define LCD_SetupMenu_Relays	0x100
#define LCD_SetupMenu_Sensors	0x200
#define LCD_SetupMenu_FlowCheck	0x300
#define LCD_SetupMenu_Options	0x400
#define LCD_SetupMenu_Relays_Max 8 // сколько помещается на экране
const char *LCD_SetupMenu[LCD_SetupMenuItems] = { "1. Exit", "2. Relays", "3. Sensors", "4. Flow check", "5. Options" };
uint32_t LCD_setup = 0; // 0x8000MMII: 8 - Setup active, MМ - Menu item (0..<max LCD_SetupMenuItems-1>) , II - Selecting item (0...)

// Задача Пользовательский интерфейс (MC.xHandleKeysLCD) "KeysLCD"
void vKeysLCD( void * )
{
	pinMode(PIN_KEY_UP, INPUT_PULLUP);
	pinMode(PIN_KEY_DOWN, INPUT_PULLUP);
	pinMode(PIN_KEY_OK, INPUT_PULLUP);
	lcd.begin(LCD_COLS, LCD_ROWS); // Setup: cols, rows
	lcd.print((char*)"WaterTreatment v");
	lcd.print((char*)VERSION);
	lcd.setCursor(0, 2);
	lcd.print((char*)"Vadim Kulakov(c)2022");
	lcd.setCursor(0, 3);
	lcd.print((char*)"vad7@yahoo.com");
	vTaskDelay(3000);
	static uint32_t DisplayTick = xTaskGetTickCount();
	static char buffer[ALIGN(LCD_COLS * 2)];
	static uint32_t setup_timeout = 0;
	//static uint32_t displayed_area = 0; // b1 - Yd,Days Fe,Soft
	for(;;)
	{
		if(LCD_setup) if(--setup_timeout == 0) goto xSetupExit;
		if(!digitalReadDirect(PIN_KEY_OK)) { // OK (middle) key
			vTaskDelay(KEY_DEBOUNCE_TIME);
			if(LCD_setup) {
				{
					uint32_t t = 2000 / KEY_DEBOUNCE_TIME;
					while(!digitalReadDirect(PIN_KEY_OK)) {
						if(--t == 0) {
							lcd.clear();
							while(!digitalReadDirect(PIN_KEY_OK)) vTaskDelay(KEY_DEBOUNCE_TIME);
							break;
						}
						vTaskDelay(KEY_DEBOUNCE_TIME);
					}
					if(t == 0) goto xSetupExit;
				}
				if((LCD_setup & 0xFFFF) == 0) { // menu item 1 selected - Exit
xSetupExit:
					lcd.command(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
					LCD_setup = 0;
					setup_timeout = 0;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Relays) { // inside menu item selected - Relay
					MC.dRelay[LCD_setup & 0xFF].set_Relay(MC.dRelay[LCD_setup & 0xFF].get_Relay() ? fR_StatusAllOff : fR_StatusManual);
					if((LCD_setup & 0xFF) == RFILL) FillingTankLastLevel = 0;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Options) { // inside menu item selected - Options
					MC.fNetworkReset = 1;
					goto xSetupExit;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_FlowCheck) { // inside menu item selected - Flow check
					goto xSetupExit;
				} else if((LCD_setup & 0xFF00) == 0) {	// select menu item
					LCD_setup = (LCD_setup << 8) | LCD_SetupFlag;
					if((LCD_setup & 0xFF00) == LCD_SetupMenu_FlowCheck) { // Flow check
						FlowPulseCounter = 0;
						lcd.command(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
					} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Sensors) {
						lcd.command(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
					}
				} else { // inside menu item selected
					goto xSetupExit;
				}
				DisplayTick = ~DisplayTick;
				setup_timeout = DISPLAY_SETUP_TIMEOUT / KEY_CHECK_PERIOD;
			} else if(MC.get_errcode() && !Error_Beep_confirmed) {
				if(MC.get_errcode() == ERR_SALT_FINISH) MC.clear_all_errors();
				Error_Beep_confirmed = true; // Supress beeping
			} else { // Enter Setup
				LCD_setup = LCD_SetupFlag;
				lcd.command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON);
				DisplayTick = ~DisplayTick;
				setup_timeout = DISPLAY_SETUP_TIMEOUT / KEY_CHECK_PERIOD;
			}
			while(!digitalReadDirect(PIN_KEY_OK)) vTaskDelay(KEY_DEBOUNCE_TIME);
			//journal.jprintfopt("[OK]\n");
		} else if(!digitalReadDirect(PIN_KEY_UP)) { // ->
			vTaskDelay(KEY_DEBOUNCE_TIME);
			while(!digitalReadDirect(PIN_KEY_UP)) vTaskDelay(KEY_DEBOUNCE_TIME);
			if(LCD_setup) {
				if((LCD_setup & 0xFF00) == 0) { // select menu item
					if((LCD_setup & 0xFF) < LCD_SetupMenuItems-1) {
						LCD_setup++;
						DisplayTick = ~DisplayTick;
					}
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Relays) {
					LCD_setup++;
					if((LCD_setup & 0xFF) >= RNUMBER) LCD_setup &= ~0xFF;
					DisplayTick = ~DisplayTick;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Options) { // Options
					goto xSetupExit;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_FlowCheck) { // inside menu item selected - Flow check
					lcd.clear();
					uint32_t tmp = FlowPulseCounter * 100 / MC.sFrequency[FLOW].get_kfValue();
					lcd.print("K = Edges / Cup(L)");
					lcd.setCursor(0, 1);
					lcd.print("0.5L = "); dptoa(buffer, tmp * 2, 2); lcd.print(buffer);
					lcd.setCursor(0, 2);
					lcd.print("1L   = "); dptoa(buffer, tmp, 2); lcd.print(buffer);
					lcd.setCursor(0, 3);
					lcd.print("2L   = "); dptoa(buffer, tmp / 2, 2); lcd.print(buffer);
					while(!digitalReadDirect(PIN_KEY_OK)) vTaskDelay(KEY_DEBOUNCE_TIME);
					setup_timeout = DISPLAY_SETUP_TIMEOUT / KEY_CHECK_PERIOD;
					while(--setup_timeout) {
						vTaskDelay(KEY_CHECK_PERIOD);
						if(!digitalReadDirect(PIN_KEY_UP) || !digitalReadDirect(PIN_KEY_DOWN) || !digitalReadDirect(PIN_KEY_OK)) break;
					}
					do { vTaskDelay(KEY_DEBOUNCE_TIME); } while(!digitalReadDirect(PIN_KEY_UP) || !digitalReadDirect(PIN_KEY_DOWN) || !digitalReadDirect(PIN_KEY_OK));
					DisplayTick = ~DisplayTick;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Sensors) DisplayTick = ~DisplayTick;
				setup_timeout = DISPLAY_SETUP_TIMEOUT / KEY_CHECK_PERIOD;
			} else {
xErrorsProcessing:
				if(MC.get_errcode()) {
					if(!Error_Beep_confirmed) Error_Beep_confirmed = true;
					else {
						lcd.setCursor(0, 2);
						lcd.print(" OK - CLEAR ERRORS? ");
						setup_timeout = 10000 / KEY_CHECK_PERIOD;
						while(--setup_timeout) {
							vTaskDelay(KEY_CHECK_PERIOD);
							if(!digitalReadDirect(PIN_KEY_UP) || !digitalReadDirect(PIN_KEY_DOWN)) break;
							if(!digitalReadDirect(PIN_KEY_OK)) {
								MC.clear_all_errors();
								break;
							}
						}
						do { vTaskDelay(KEY_DEBOUNCE_TIME); } while(!digitalReadDirect(PIN_KEY_UP) || !digitalReadDirect(PIN_KEY_DOWN) || !digitalReadDirect(PIN_KEY_OK));
						DisplayTick = ~DisplayTick;
					}
				}
			}
			//journal.jprintfopt("[UP]\n");
		} else if(!digitalReadDirect(PIN_KEY_DOWN)) { // <-
			vTaskDelay(KEY_DEBOUNCE_TIME);
			while(!digitalReadDirect(PIN_KEY_DOWN)) vTaskDelay(KEY_DEBOUNCE_TIME);
			if(LCD_setup) {
				if((LCD_setup & 0xFF00) == LCD_SetupMenu_FlowCheck) { // Flow check - reset calc
					FlowPulseCounter = 0;
				} else if((LCD_setup & 0xFF00) == 0) {
					if(LCD_setup & 0xFF) {
						LCD_setup--;
						DisplayTick = ~DisplayTick;
					}
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Relays) {
					if((LCD_setup & 0xFF) != 0) LCD_setup--; else LCD_setup |= RNUMBER - 1;
					DisplayTick = ~DisplayTick;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Options) { // Options
					goto xSetupExit;
				}
				setup_timeout = DISPLAY_SETUP_TIMEOUT / KEY_CHECK_PERIOD;
			} else goto xErrorsProcessing;
			//journal.jprintfopt("[DWN]\n");
		}
		if(xTaskGetTickCount() - DisplayTick > DISPLAY_UPDATE) { // Update display
			// Display:
			// 12345678901234567890
			// 0.000 m3h > 0.000
			// P: 0.00 Tank: 100 %
			// Days Fe:123 Soft:123
			// Day: 0.000 Yd: 0.000
			char *buf = buffer;
			if(LCD_setup) {
				if((LCD_setup & 0xFF00) == LCD_SetupMenu_Relays) { // Relays
					lcd.clear();
					uint8_t ipage = (LCD_setup & 0xFF) / LCD_SetupMenu_Relays_Max;
					for(uint8_t i = 0; i < LCD_SetupMenu_Relays_Max && ipage * LCD_SetupMenu_Relays_Max + i < RNUMBER; i++) {
						lcd.setCursor(10 * (i % 2), i / 2);
						lcd.print(MC.dRelay[ipage * LCD_SetupMenu_Relays_Max + i].get_Relay() ? '*' : ' ');
						lcd.print(MC.dRelay[ipage * LCD_SetupMenu_Relays_Max + i].get_name());
					}
					uint8_t item = (LCD_setup & 0xFF) % LCD_SetupMenu_Relays_Max;
					lcd.setCursor(10 * (item % 2), item / 2);
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Sensors) { // Sensors
					lcd.clear();
					for(uint8_t i = 0; i < (INUMBER > 8 ? 8 : INUMBER) ; i++) {
						lcd.setCursor(10 * (i % 2), i / 2);
						lcd.print(MC.sInput[i].get_Input() ? '*' : ' ');
						lcd.print(MC.sInput[i].get_name());
					}
					lcd.setCursor(10 * ((LCD_setup & 0xFF) % 2), (LCD_setup & 0xFF) / 2);
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_FlowCheck) { // Flow check
					// 12345678901234567890
					// Flow: 1.655
					// Edges: 41234
					// Liters: 112.1234
					// m*h: 12.123 14.123
					lcd.setCursor(0, 0);
					strcpy(buf, "Flow: "); buf += 6;
					uint32_t tmp = MC.sFrequency[FLOW].get_Value();
					if(tmp) setup_timeout = DISPLAY_SETUP_TIMEOUT / KEY_CHECK_PERIOD; // не выходить из настроек пока проток есть
					buf = dptoa(buf, tmp, 3);
					buffer_space_padding(buf, LCD_COLS - (buf - buffer));
					lcd.print(buffer);

					lcd.setCursor(0, 1);
					strcpy(buf = buffer, "Edges: "); buf += 7;
					tmp = FlowPulseCounter / 100;
					buf += i10toa(tmp, buf, 0);
					buffer_space_padding(buf, LCD_COLS - (buf - buffer));
					lcd.print(buffer);

					lcd.setCursor(0, 2);
					strcpy(buf = buffer, "Liters: "); buf += 8;
					tmp*= 100;
					buf += i10toa(tmp / MC.sFrequency[FLOW].get_kfValue(), buf, 0);
					*buf++ = '.';
					buf += i10toa((uint32_t)(tmp % MC.sFrequency[FLOW].get_kfValue()) * 10000 / MC.sFrequency[FLOW].get_kfValue(), buf, 4);
					buffer_space_padding(buf, LCD_COLS - (buf - buffer));
					lcd.print(buffer);

					lcd.setCursor(0, 3);
					tmp = MC.CalcFilteringSpeed(MC.FilterTankSquare);
					if(tmp == 0) { // выводим K, если протока нет
								// 12345678901234567890
						lcd.print("Clear: \x7F, Calc: \x7E   ");
					} else {
						strcpy(buf = buffer, "m*h: "); buf += 4;
						buf = dptoa(buf, tmp, 3);
						*buf++ = ' '; //*buf++ = ' ';
						buf = dptoa(buf, MC.CalcFilteringSpeed(MC.FilterTankSoftenerSquare), 3);
						buffer_space_padding(buf, LCD_COLS - (buf - buffer));
						lcd.print(buffer);
					}

					DisplayTick = xTaskGetTickCount() - (DISPLAY_UPDATE - 1000); // every 1000 ms
					vTaskDelay(KEY_CHECK_PERIOD);
					continue;
				} else if((LCD_setup & 0xFF00) == LCD_SetupMenu_Options) { // Options
					lcd.clear();
					lcd.print("Ok - Network reset");
				} else {
					lcd.clear();
					lcd.print(LCD_SetupMenu[LCD_setup & 0xFF]);
					lcd.setCursor(0, 1);
					lcd.print("Long press OK - Exit");
					lcd.setCursor(0, 3);
					NowDateToStr(buffer);
					buffer[10] = ' ';
					NowTimeToStr(buffer + 11);
					lcd.print(buffer);
					lcd.setCursor(0, 0);
				}
			} else {
				lcd.setCursor(0, 0);
				if(MC.sFrequency[FLOW].WebCorrectCnt > 1) *buf++ = '+';
				int32_t tmp = MC.sFrequency[FLOW].get_Value();
				buf = dptoa(buf, tmp, 3);
				strcpy(buf, " m3h "); buf += 5;
				uint8_t _regactive = (MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) | (MC.sInput[REG2_ACTIVE].get_Input() << 1);
				if(_regactive) {
					*buf++ = 'R';
					tmp = MC.RTC_store.UsedRegen;
				} else {
					*buf++ = MC.NO_Power ? 'x' : LowConsumeMode ? 'G' : '\x7E'; // '->'
					tmp = MC.WorkStats.UsedSinceLastRegen + MC.RTC_store.UsedToday;
				}
				if(tmp < 10000) *buf++ = ' ';
				buf = dptoa(buf, tmp, 3);
				buffer_space_padding(buf, LCD_COLS - (buf - buffer));
				lcd.print(buffer);

				lcd.setCursor(0, 1);
				strcpy(buf = buffer, "P: "); buf += 3;
				buf = dptoa(buf, MC.sADC[PWATER].get_Value(), 2);
				strcpy(buf, " Tank: "); buf += 7;
				buf = dptoa(buf, MC.sADC[LTANK].get_Value() / 100, 0);
				*buf++ = '%';
				buffer_space_padding(buf, LCD_COLS - (buf - buffer));
				lcd.print(buffer);

				lcd.setCursor(0, 2);
				strcpy(buf = buffer, "Day:"); buf += 4;
				tmp = MC.RTC_store.UsedToday;
				if(tmp < 10000) *buf++ = ' ';
				buf = dptoa(buf, tmp, 3);
				strcpy(buf, " Yd:"); buf += 4;
				tmp = MC.WorkStats.UsedYesterday;
				if(tmp < 10000) *buf++ = ' ';
				buf = dptoa(buf, tmp, 3);
				buffer_space_padding(buf, LCD_COLS - (buf - buffer));
				lcd.print(buffer);

				lcd.setCursor(0, 3);
				buf = buffer; *buf = '\0';
				// 12345678901234567890
				// FLOOD! EMPTY! ERR-21
				if(CriticalErrors & ERRC_Flooding) {
					if(MC.sInput[FLOODING].get_Input()) {
						strcpy(buf, "FLOOD! "); buf += 7;
					} else {
						strcpy(buf, "LEAK! "); buf += 6;
					}
				}
				if(CriticalErrors & ERRC_TankEmpty) {
					strcpy(buf, "EMPTY! "); buf += 7;
				}
				if(CriticalErrors & ERRC_WaterCounter) {
					strcpy(buf, "CNT! "); buf += 5;
				}
				if(CriticalErrors & ERRC_SepticAlarm) {
					strcpy(buf, "SEPTIC! "); buf += 8;
				}
				if(CriticalErrors & ERRC_WeightEmpty) {
					strcpy(buf, "BRINE! "); buf += 7;
					goto xShowWeight;
				}
				if((CriticalErrors & ERRC_TankFillingLong) && (uint32_t)(buf - buffer) <= sizeof(buffer) - 8) {
					strcpy(buf, "FTANK! "); buf += 7;
				}
				if((CriticalErrors & ERRC_LongWaterConsuming) && (uint32_t)(buf - buffer) <= sizeof(buffer) - 8) {
					strcpy(buf, "LFLOW! "); buf += 7;
				}
				if(MC.get_errcode()) {
					if(MC.get_errcode() == ERR_SALT_FINISH) {
						strcpy(buf, "SALT!"); buf += 5;
					} else {
						strcpy(buf, "ERR"); buf += 3;
						buf = dptoa(buf, MC.get_errcode(), 0);
						if(MC.get_errcode() == ERR_WEIGHT_LOW || MC.get_errcode() == ERR_WEIGHT_EMPTY) {
							strcpy(buf, " W:"); buf += 3;
							goto xShowWeight;
						}
					}
				}
				if(buf == buffer) {
					if(_regactive) {
						// 12345678901234567890
						// 0:30, m*h: 5.123
						uint32_t t = RegenStarted ? rtcSAM3X8.unixtime() - RegenStarted : 0;
						if(t < 86400) {
							buf += i10toa(t / 60, buf, 0);
							*buf++ = ':';
							buf += i10toa(t % 60, buf, 2);
						} else {
							strcat(buf, "?:?");
							buf += 3;
						}
						strcat(buf, ", m*h: "); buf += 7;
						buf = dptoa(buf, MC.CalcFilteringSpeed((_regactive & 2) ? MC.FilterTankSoftenerSquare : MC.FilterTankSquare), 3);
					} else if((tmp = MC.dPWM.get_Power()) != 0) {
						strcpy(buf, "Power,W: "); buf += 9;
						buf = dptoa(buf, tmp, 0);
					} else {
						if((setup_timeout = !setup_timeout)) {
							strcpy(buf, "Weight:"); buf += 7;
							if(Weight_Percent < 10000) *buf++ = ' ';
xShowWeight:
							buf = dptoa(buf, Weight_value / 10, 0);
							*buf++ = '\x7E'; // '->'
							buf = dptoa(buf, Weight_Percent / 10, 1);
							*buf++ = '%';
						} else {
							strcpy(buf, "Days,Fe:"); buf += 8;
							tmp = MC.WorkStats.DaysFromLastRegen;
							if(tmp < 100) *buf++ = ' ';
							buf = dptoa(buf, tmp, 0);
							strcpy(buf, " Soft:"); buf += 6;
							tmp = MC.WorkStats.DaysFromLastRegenSoftening;
							if(tmp < 100) *buf++ = ' ';
							buf = dptoa(buf, tmp, 0);
						}
					}
				}
				tmp = LCD_COLS - (buf - buffer);
				if(tmp > 0)	buffer_space_padding(buf, tmp); else buffer[LCD_COLS] = '\0';
				lcd.print(buffer);
			}

			DisplayTick = xTaskGetTickCount();
		}
		vTaskDelay(KEY_CHECK_PERIOD);
	}
	vTaskSuspend(NULL);
}

//////////////////////////////////////////////////////////////////////////
// Задача чтения датчиков
void vReadSensor(void *)
{ //const char *pcTaskName = "ReadSensor\r\n";
	static unsigned long readPWM = 0;
	static uint32_t ttime;
	static uint8_t  prtemp;
	static uint32_t oldTime, OneWire_time;
	oldTime = GetTickCount();
	OneWire_time = oldTime - ONEWIRE_READ_PERIOD;
	//Weight_adc_median1 = Weight_adc_median2 = Weight.read();
	for(;;) {
		int8_t i;
		//WDT_Restart(WDT);

		prtemp = fDS2482_bus_mask;
		ttime = GetTickCount();
		if(ttime - OneWire_time >= ONEWIRE_READ_PERIOD) {
			OneWire_time = ttime;
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
		}

#ifdef RADIO_SENSORS
		radio_timecnt++;
#endif
		// read in vPumps():
		//for(i = 0; i < ANUMBER; i++) MC.sADC[i].Read();                  // Прочитать данные с датчиков давления
#ifdef USE_UPS
		if(!MC.NO_Power)
#endif
		{
			// Основная группа регистров, включая мощность
			if(MC.dPWM.get_readState(0) == OK) {
				if(WaterBoosterStatus > 0 && WaterBoosterTimeout > MC.Option.PWM_StartingTime) {
					if(MC.Option.PWM_DryRun && MC.dPWM.get_Power() < MC.Option.PWM_DryRun) { // Сухой ход
						CriticalErrors |= ERRC_WaterBooster;
						set_Error(ERR_PWM_DRY_RUN, (char*)__FUNCTION__);
					} else if(MC.Option.PWM_Max && MC.dPWM.get_Power() > MC.Option.PWM_Max) { // Перегрузка
						CriticalErrors |= ERRC_WaterBooster;
						set_Error(ERR_PWM_MAX, (char*)__FUNCTION__);
					}
				}
			}
			if(MC.dPWM.get_lastErr() == ERR_NO_POWER) {
				if(!MC.NO_Power) {
					MC.NO_Power = 1;
					MC.save_WorkStats();
					Stats.SaveStats(0);
					Stats.SaveHistory(0);
					journal.jprintf_date("Power lost!\n");
				}
				MC.NO_Power_delay = NO_POWER_ON_DELAY * 1000 / TIME_READ_SENSOR;
			} else if(MC.NO_Power) { // Включаемся
				if(MC.NO_Power_delay) {
					if(--MC.NO_Power_delay == 0) MC.fNetworkReset = 1;
				} else {
					journal.jprintf_date("Power restored!\n");
					MC.NO_Power = 0;
				}
			}
		}

		// read in vPumps():
		//for(i = 0; i < INUMBER; i++) MC.sInput[i].Read();                // Прочитать данные сухой контакт
		for(i = FLOW + 1; i < FNUMBER; i++) MC.sFrequency[i].Read();		// Получить значения датчиков потока, кроме FLOW
		// Add to FLOW
		int32_t add_to_flow = 0;
		int16_t pw = MC.sADC[PWATER].get_Value();
		if(MC.sFrequency[FLOW].WebCorrectCnt > 1) MC.sFrequency[FLOW].WebCorrectCnt--;	// 1 sec
		if(MC.sFrequency[FLOW].get_ValueReal() <= MC.Option.FlowIncByPress_MinFlow) {
			if(pw < MC.Option.PWATER_Osmos_Min) { // нет протока и давление низкое, счетчик раннего включения насосной станции
				if(pw > MC.Osmos_PWATER_Last) MC.Osmos_PWATER_Cnt -= MC.Osmos_PWATER_Cnt > 0 ? 1 : 0;
				else if(pw < MC.Osmos_PWATER_Last) MC.Osmos_PWATER_Cnt++;
			} else MC.Osmos_PWATER_Cnt = 0;
			if(GETBIT(MC.Option.flags, fFlowIncByPressure) && WaterBoosterTimeout > MC.Option.PWATER_Osmos_FullDelay * 1000UL && MC.Osmos_PWATER_BoosterMax > 100
					&& !(MC.RTC_store.Work & RTC_Work_Regen_MASK) && TimerDrainingWater == 0) { // низкий проток и не идет - регенерация/слив и т.п.
				int32_t d = MC.Osmos_PWATER_LastPress - pw;
				if(d > 0 && (d >= PWATER_OSMOS_MIN_DELTA || MC.Osmos_PWATER_Added == 2)) {
					int16_t dd = MC.sADC[PWATER].get_maxValue() - MC.sADC[PWATER].get_minValue();
					if(d > dd) d = dd;
					int16_t pw2 = pw - MC.sADC[PWATER].get_minValue();
					if(pw2 > dd / 2) {
						add_to_flow = d * MC.Osmos_PWATER_BoosterMax * 100 / dd * (MC.sFrequency[FLOW].get_kfValue()/10) / 1000;
					} else { //if(pw2 > dd / 2 / 2) {
						add_to_flow = d * MC.Osmos_PWATER_BoosterMax * 150 / dd * (MC.sFrequency[FLOW].get_kfValue()/10) / 1000; // *1.5
//						} else {
//							add_to_flow = d * MC.Osmos_PWATER_BoosterMax * 100 * 2 / dd * (MC.sFrequency[FLOW].get_kfValue()/10) / 1000;
					}
					//if(add_to_flow > (int32_t)MC.sFrequency[FLOW].get_count()) add_to_flow -= MC.sFrequency[FLOW].get_count();
					MC.sFrequency[FLOW].WebCorrectCnt = (TIMER_TO_SHOW_STATUS + 1000) / TIME_READ_SENSOR + 1;
					//TimeFeedPump +=	d * MC.Osmos_PWATER_BoosterMax * 1000 / ((MC.sADC[PWATER].get_maxValue() - MC.sADC[PWATER].get_minValue()) * 100) * TIME_READ_SENSOR / MC.Option.FeedPumpMaxFlow;
					MC.Osmos_PWATER_Added = 2;
					MC.Osmos_PWATER_LastPress = pw;
					MC.Osmos_PWATER_LastPress_Timer = 0;
				} else if(MC.Osmos_PWATER_Added == 2) {
					if(++MC.Osmos_PWATER_LastPress_Timer > PWATER_OSMOS_LASTPRESS_RENEW || d <= -1) {
						MC.Osmos_PWATER_Added = 1;
						MC.Osmos_PWATER_LastPress_Timer = 0;
					}
				}
			} else {
				if(MC.Osmos_PWATER_Added == 2) MC.Osmos_PWATER_Added = 1;
				MC.Osmos_PWATER_LastPress = pw;
				MC.Osmos_PWATER_LastPress_Timer = 0;
			}
		} else {
			MC.Osmos_PWATER_Cnt = 0;
			if(MC.Osmos_PWATER_Added == 2) MC.Osmos_PWATER_Added = 1;
			MC.Osmos_PWATER_LastPress = pw;
			MC.Osmos_PWATER_LastPress_Timer = 0;
		}
		MC.Osmos_PWATER_Last = pw;
		MC.sFrequency[FLOW].add_pulses100 += add_to_flow;
		if(MC.sFrequency[FLOW].Read()) {	// Обновить значения датчика потока, был проток
			UsedWaterContinuousCntNot = 0;
			if(++UsedWaterContinuousCntUsed == USED_WATER_CONTINUOUS_MINTIME) {
				UsedWaterContinuousTimer++;
				if(MC.Option.UsedWaterContinuous && UsedWaterContinuousTimer > MC.Option.UsedWaterContinuous * (60000 / TIME_READ_SENSOR / USED_WATER_CONTINUOUS_MINTIME)) {
					CriticalErrors |= ERRC_LongWaterConsuming;
					set_Error(ERR_LONG_WATER_CONSUMING, (char*)__FUNCTION__);
					MC.dRelay[RWATEROFF1].set_ON();
					MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
				}
				UsedWaterContinuousCntUsed = 0;
			}
			// Flow water
			uint32_t passed;
			{
				passed = MC.sFrequency[FLOW].Passed;
				MC.sFrequency[FLOW].Passed = 0;
			}
			WaterBoosterCountP100 += MC.sFrequency[FLOW].count_real_last100;
			FlowPulseCounter += MC.sFrequency[FLOW].count_real_last100;
			MC.sFrequency[FLOW].ChartLiters_rest = MC.sFrequency[FLOW].PassedRest;
			if(passed) {
				MC.sFrequency[FLOW].ChartLiters_accum += passed;
				if(!MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) {
					TimeFeedPump +=	passed * 1000 * TIME_READ_SENSOR / MC.Option.FeedPumpMaxFlow;
				} else if(RegBackwashTimer == 0) { // В начале обратной промывки реагент не подаем, в конце - усиленная подача
					TimeFeedPump +=	passed * 1000 * TIME_READ_SENSOR / MC.Option.BackWashFeedPumpMaxFlow;
				}
				uint32_t utm = rtcSAM3X8.unixtime();
				if(MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input() || MC.sInput[REG2_ACTIVE].get_Input()) {
					// Regen
					if(RegMaxFlow < MC.sFrequency[FLOW].get_Value()) RegMaxFlow = MC.sFrequency[FLOW].get_Value();
					if(RegMinPress > pw) RegMinPress = pw;
					MC.RTC_store.UsedRegen += passed;
					Stats_WaterRegen_work += passed;
					History_WaterRegen_work += passed;
					MC.WorkStats.UsedLastTime = utm;
					UsedWaterContinuousTimer = 0;
					NeedSaveRTC |= (1<<bRTC_UsedRegen);
				} else {
					if(MC.dRelay[RDRAIN].get_Relay() || MC.WorkStats.LastDrain + MC.Option.DrainTime + (TIME_READ_SENSOR/1000) >= utm) {
						MC.WorkStats.UsedDrain += passed * 10;
					} else {
						Stats_WaterUsed_work += passed;
						MC.RTC_store.UsedToday += passed;
						MC.WorkStats.UsedLastTime = utm;
					}
					History_WaterUsed_work += passed;
					Passed100Count += passed;
					if(Passed100Count >= 100 || passed >= 100) {
						Passed100Count = 0;
						if(MC.WorkStats.UsedDrainSiltL100 < 255) MC.WorkStats.UsedDrainSiltL100++;
						if(MC.Option.FilterCounter1_Max) MC.WorkStats.FilterCounter1++;
						if(MC.Option.FilterCounter2_Max) MC.WorkStats.FilterCounter2++;
					}
					NeedSaveRTC |= (1<<bRTC_UsedToday);
				}
			}
		} else {
			if(++UsedWaterContinuousCntNot == USED_WATER_CONTINUOUS_MINTIME) {
				if(UsedWaterContinuousTimerMax < UsedWaterContinuousTimer) UsedWaterContinuousTimerMax = UsedWaterContinuousTimer;
				UsedWaterContinuousTimer = 0;
				UsedWaterContinuousCntNot = 0;
				UsedWaterContinuousCntUsed = 0;
			}
		}
		//

#ifdef USE_UPS
		if(!MC.NO_Power)
#endif
			if(GetTickCount() - readPWM > PWM_READ_PERIOD) {
				readPWM = GetTickCount();
				MC.dPWM.get_readState(1);     // Последняя группа регистров
			}

		vReadSensor_delay1ms(cDELAY_DS1820 - (int32_t)(GetTickCount() - ttime)); 	// Ожидать время преобразования

		// do not need averaging: // uint8_t flags = 0;
		for(i = 0; i < TNUMBER; i++) {                                   // Прочитать данные с температурных датчиков
			if((prtemp & (1<<MC.sTemp[i].get_bus())) == 0) {
				MC.sTemp[i].Read();
				// do not need averaging: // if(MC.sTemp[i].Read() == OK) flags |= MC.sTemp[i].get_setup_flags();
			}
			_delay(1);     												// пауза
		}
		/* do not need averaging:
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
		*/ // do not need averaging.

		MC.calculatePower();	// Расчет мощностей
		Stats.Update();
		if((MC.RTC_store.Work & RTC_Work_WeekDay_MASK) != rtcSAM3X8.get_day_of_week()) {	// новый день
			//vTaskSuspendAll(); // запрет других задач
			MC.RTC_store.Work = (MC.RTC_store.Work & ~RTC_Work_WeekDay_MASK) | rtcSAM3X8.get_day_of_week();
			uint32_t ut = MC.RTC_store.UsedToday;
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
				if(MC.WorkStats.UsedAverageDayNum >= WS_AVERAGE_DAYS) {
					MC.WorkStats.UsedAverageDay = MC.WorkStats.UsedAverageDay / MC.WorkStats.UsedAverageDayNum;
					MC.WorkStats.UsedAverageDayNum = 1;
				}
			}
			NeedSaveWorkStats = 1;
			NeedSaveRTC = RTC_SaveAll;
			//xTaskResumeAll(); // Разрешение других задач
			update_RTC_store_memory();
		}

		vReadSensor_delay1ms((TIME_READ_SENSOR - (int32_t)(GetTickCount() - ttime)) / 2);     // 1. Ожидать время нужное для цикла чтения

		//  Синхронизация часов с I2C часами если стоит соответствующий флаг
		if(MC.get_updateI2C())  // если надо обновить часы из I2c
		{
			if(GetTickCount() - oldTime > (uint32_t)TIME_I2C_UPDATE) // время пришло обновляться надо Период синхронизации внутренних часов с I2C часами (сек)
			{
				oldTime = rtcSAM3X8.unixtime();
				uint32_t t = TimeToUnixTime(getTime_RtcI2C());       // Прочитать время из часов i2c
				if(t) {
					rtcSAM3X8.set_clock(t);                		 // Установить внутренние часы по i2c
					int32_t dt = t > oldTime ? t - oldTime : -(oldTime - t);
					MC.updateDateTime(dt);  // Обновить переменные времени с новым значением часов
					journal.jprintfopt("Sync from I2C RTC: %s %s (%d)\n", NowDateToStr(), NowTimeToStr(), dt);
				} else {
					journal.jprintfopt("Error read I2C RTC\n");
				}
				oldTime = GetTickCount();
			}
		}
		// Проверки граничных температур для уведомлений, если разрешено!
		static uint8_t last_life_h = 255;
		if(MC.message.get_fMessageLife()) // Подача сигнала жизни если разрешено!
		{
			uint8_t hour = rtcSAM3X8.get_hours();
			if(hour == HOUR_SIGNAL_LIFE && hour != last_life_h) {
				MC.message.setMessage(pMESSAGE_LIFE, (char*) "Контроллер работает...", 0);
			}
			last_life_h = hour;
		}
		//
		vReadSensor_delay1ms(TIME_READ_SENSOR - (int32_t)(GetTickCount() - ttime));     // Ожидать время нужное для цикла чтения
	}  // for
	vTaskSuspend(NULL);
}

// Вызывается во время задержек в задаче чтения датчиков, должна быть вызвана перед основным циклом на время заполнения буфера усреднения
void vReadSensor_delay1ms(int32_t ms)
{
	if(ms <= 0 || ms >= (int32_t)TIME_READ_SENSOR) return;
	if(ms < 10) {
		vTaskDelay(ms);
		return;
	}
	ms -= 10;
	uint32_t tm = GetTickCount();
	do {
		Weight_Read();

#ifdef USE_UPS
		MC.sInput[SPOWER].Read(true);
		if(MC.sInput[SPOWER].get_Input()) { // Электричество кончилось
			if(!MC.NO_Power) {
				MC.NO_Power = 1;
				MC.save_WorkStats();
				Stats.SaveStats(0);
				Stats.SaveHistory(0);
				journal.jprintf_date("Power lost!\n");
			}
			MC.NO_Power_delay = NO_POWER_ON_DELAY_CNT;
		} else if(MC.NO_Power) { // Включаемся
			if(MC.NO_Power_delay) {
				if(--MC.NO_Power_delay == 0) MC.fNetworkReset = 1;
			} else {
				journal.jprintf_date("Power restored!\n");
				MC.NO_Power = 0;
			}
		}
#endif
#ifdef RADIO_SENSORS
		check_radio_sensors();
#endif
		int32_t tm2 = GetTickCount() - tm;
		if((tm2 -= ms) >= 0) {
			if(tm2 < 10) vTaskDelay(10 - tm2);
			break;
		}
		vTaskDelay(tm2 > -10 ? 3 : 10);
	} while(true);
}

//////////////////////////////////////////////////////////////////////////
// Задача Управления насосами (MC.xHandlePumps) "Pumps"
void vPumps( void * )
{
	uint32_t CriticalErrors_timeout = 0;
	while(!ADC_has_been_read) vTaskDelay(1000 / ADC_FREQ + TIME_SLICE_PUMPS); // ms
	for(;;)
	{
		//WDT_Restart(WDT); // Reset Watchdog here, most important task.
		if(CriticalErrors) CriticalErrors_timeout = MC.Option.CriticalErrorsTimeout * 1000;
		else if(CriticalErrors_timeout >= TIME_SLICE_PUMPS) CriticalErrors_timeout -= TIME_SLICE_PUMPS; else CriticalErrors_timeout = 0;
		if(WaterBoosterStatus != 0) {
			Stats_WaterBooster_work += TIME_SLICE_PUMPS;
			History_WaterBooster_work += TIME_SLICE_PUMPS;
			Charts_WaterBooster_work += TIME_SLICE_PUMPS;
			TankCheckFlag = 2;
		}
		if((WaterBoosterTimeout += TIME_SLICE_PUMPS) < TIME_SLICE_PUMPS) WaterBoosterTimeout = 0xFFFFFFFF;
		if(AfterFilledTimer) if((AfterFilledTimer -= TIME_SLICE_PUMPS) < 0) AfterFilledTimer = 0;
		for(uint8_t i = 0; i < RNUMBER; i++) MC.dRelay[i].NextTimerOn();

		// Read sensors
		for(uint8_t i = 0; i < INUMBER; i++) MC.sInput[i].Read(true);		// Прочитать данные сухой контакт (FAST mode)
		bool tank_empty = false;
#ifdef TANK_EMPTY
		tank_empty = MC.sInput[TANK_EMPTY].get_Input();
		if(tank_empty) {
			if(!(CriticalErrors & ERRC_TankEmpty)) vPumpsNewError = ERR_TANK_EMPTY;
			CriticalErrors |= ERRC_TankEmpty;
		} else if(!CriticalErrors_timeout) CriticalErrors &= ~ERRC_TankEmpty;
#endif
		if(ADC_has_been_read) {		// Не чаще, чем ADC
			ADC_has_been_read = false;
			for(uint8_t i = 0; i < ANUMBER; i++) MC.sADC[i].Read();			// Прочитать данные с аналоговых давления
			tank_empty = tank_empty || (MC.sADC[LTANK].get_Value() < MC.sADC[LTANK].get_minValue());
			if(tank_empty) {
				if(!(CriticalErrors & ERRC_TankEmpty)) vPumpsNewError = ERR_TANK_EMPTY;
				CriticalErrors |= ERRC_TankEmpty;
			} else if((CriticalErrors & ERRC_TankEmpty) && !CriticalErrors_timeout) CriticalErrors &= ~ERRC_TankEmpty;
		}
		uint8_t reg_active = (MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) | (MC.sInput[REG2_ACTIVE].get_Input() << 1);
#ifdef ONLY_ONE_REGEN_AT_TIME
		if(reg_active == 3 && vPumpsNewError == OK) vPumpsNewError = ERR_REGEN_CONFLICT;
#endif

		// Check Errors
		int16_t press = MC.sADC[PWATER].get_Value();
		if(MC.sInput[FLOODING].get_Input()
#ifdef LEAK
				|| MC.sInput[LEAK].get_Input()
#endif
		){
			if(FloodingTime == 0) FloodingTime = GetTickCount();
			else if(GetTickCount() - FloodingTime > (uint32_t) MC.Option.FloodingDebounceTime * 1000) {
				FloodingTime = GetTickCount() | 1;
#ifdef LEAK
				vPumpsNewError = MC.sInput[FLOODING].get_Input() ? ERR_FLOODING : ERR_LEAK;
#else
				vPumpsNewError = ERR_FLOODING;
#endif
				if(MC.dRelay[RFILL].get_Relay()) MC.dRelay[RFILL].set_Relay(fR_StatusAllOff);
				if(MC.dRelay[RDRAIN].get_Relay()) {
					MC.dRelay[RDRAIN].set_Relay(fR_StatusAllOff);
					TimerDrainingWater = 0;
					TimerDrainingWaterAfterRegen = 0;
				}
				if(MC.dRelay[RDRAIN2].get_Relay()) {
					MC.dRelay[RDRAIN2].set_Relay(fR_StatusAllOff);
					TimerDrainingWaterAfterRegen = 0;
				}
				MC.dRelay[RFEEDPUMP].set_Relay(fR_StatusAllOff);
				MC.dRelay[RWATEROFF1].set_ON();
				MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
#ifdef RSILT
				MC.dRelay[RSILT].set_Relay(fR_StatusAllOff);
#endif
				CriticalErrors |= ERRC_Flooding;
				if(WaterBoosterStatus > 0) goto xWaterBooster_GO_OFF;
			}
		} else {
			if(CriticalErrors & ERRC_Flooding) {
				if(GetTickCount() - FloodingTime > (uint32_t) MC.Option.FloodingTimeout * 1000) {
					MC.dRelay[RWATEROFF1].set_Relay(fR_StatusAllOff);
					MC.dRelay[RWATERON].set_ON();
					CriticalErrors &= ~ERRC_Flooding;
					FloodingTime = 0;
#ifdef TEST_BOARD
					journal.jprintf("Flooding self recovered\n");
#endif
				}
			} else FloodingTime = 0;
		}
		if(MC.sInput[SEPTIC_ALARM].get_Input()) {
			if(++SepticAlarmTime > MC.Option.SepticAlarmDebounce * 1000 / TIME_SLICE_PUMPS) {
				vPumpsNewError = ERR_SEPTIC_ALARM;
				CriticalErrors |= ERRC_SepticAlarm;
				MC.dRelay[RWATEROFF1].set_ON();
				MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
				SepticAlarmTime = 0;
			}
		} else SepticAlarmTime = 0;

		// Water Booster
		if(WaterBoosterStatus == 0) {
			if(!CriticalErrors && press != ERROR_PRESS
					&& WaterBoosterTimeout >= MC.Option.MinWaterBoostOffTime
					&& (press <= (reg_active ? MC.Option.PWATER_RegMin : MC.sADC[PWATER].get_minValue()) || MC.Osmos_PWATER_Cnt > MC.Option.PWATER_Osmos_Step)) {
				// Starting
				if(!LowConsumeMode || (!MC.dRelay[RFEEDPUMP].get_Relay() && AfterFilledTimer == 0)) {
					int32_t l;
					if(WaterBoosterFlag == 0) {
						WaterBoosterFlag = 1;
						goto xWaterBooster_StartFill; // first time
					}
					l = WaterBoosterCountP100 * 100 / MC.sFrequency[FLOW].get_kfValue();
					l += MC.sFrequency[FLOW].get_RawPassed();
					if((LCD_setup & 0xFF00) != LCD_SetupMenu_FlowCheck && l <= MC.Option.MinWaterBoosterCountL) {
						CriticalErrors |= ERRC_WaterCounter;
						if(!vPumpsNewError) vPumpsNewError = ERR_WATER_CNT_FAIL;
						MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
						MC.dRelay[RDRAIN].set_Relay(fR_StatusAllOff);
						MC.dRelay[RDRAIN2].set_Relay(fR_StatusAllOff);
					} else {
						if(History_BoosterCountL == -1) History_BoosterCountL = l; else History_BoosterCountL += l;
						if(!reg_active) {
							if(!MC.Osmos_PWATER_Added && MC.Osmos_PWATER_Cnt <= MC.Option.PWATER_Osmos_Step) {
								if(l > MC.Osmos_PWATER_BoosterMax_Calc) MC.Osmos_PWATER_BoosterMax_Calc = l;
								if(++MC.Osmos_PWATER_BoosterMax_cnt == BOOSTERMAX_HIST_MAX) {
									MC.Osmos_PWATER_BoosterMax = MC.Osmos_PWATER_BoosterMax_Calc * MC.Option.PWATER_Osmos_TankMul / 100;
									MC.Osmos_PWATER_BoosterMax_cnt = 0;
									MC.Osmos_PWATER_BoosterMax_Calc = 0;
								}
								MC.ChartWaterBoosterCount.addPoint(l);
							} else MC.ChartWaterBoosterCount.addPoint(-l);
						}
						MC.sFrequency[FLOW].WebCorrectCnt = 0;
xWaterBooster_StartFill:
						MC.dRelay[RBOOSTER1].set_ON();
						WaterBoosterTimeout = 0;
						WaterBoosterStatus = 1;
						MC.Osmos_PWATER_Added = reg_active != 0;
					}
				}
			}
		} else if(WaterBoosterStatus > 0) {
			if(CriticalErrors || (WaterBoosterTimeout >= MC.Option.MinWaterBoostOnTime && press >= MC.sADC[PWATER].get_maxValue())) { // Stopping
xWaterBooster_GO_OFF:
				if(WaterBoosterStatus == 1) {
					goto xWaterBooster_OFF;
				} else {// Off full cycle
					MC.dRelay[RBOOSTER2].set_OFF();
					WaterBoosterStatus = -1;
				}
			} else if(WaterBoosterStatus == 1) {
				MC.dRelay[RBOOSTER2].set_ON();
				WaterBoosterStatus = 2; // Now - ON
			}
		} else if(WaterBoosterStatus == -1) {
xWaterBooster_OFF:
			MC.dRelay[RBOOSTER1].set_OFF();
			WaterBoosterTimeout = 0;
			WaterBoosterStatus = 0;
			WaterBoosterCountP100 = 0;
		}

		// Feed Pump
		if(MC.dRelay[RFEEDPUMP].get_Relay()) {
			if(TimeFeedPump >= TIME_SLICE_PUMPS) {
				taskENTER_CRITICAL();
				TimeFeedPump -= TIME_SLICE_PUMPS;
				Stats_FeedPump_work += TIME_SLICE_PUMPS;
				History_FeedPump_work += TIME_SLICE_PUMPS;
				Charts_FeedPump_work += TIME_SLICE_PUMPS;
				taskEXIT_CRITICAL();
				if((FeedPumpWork += TIME_SLICE_PUMPS) > FEEDPUMP_MAX_WORK_TIME_ERR) {
					if(!vPumpsNewError) vPumpsNewError = ERR_FEEDPUMP_TIME;
					MC.dRelay[RFEEDPUMP].set_Relay(fR_StatusAllOff);
					MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
				}
			}
			if(TimeFeedPump < TIME_SLICE_PUMPS) MC.dRelay[RFEEDPUMP].set_OFF();
		} else if(TimeFeedPump >= MC.Option.MinPumpOnTime) {
			if(LowConsumeMode && (WaterBoosterStatus || AfterFilledTimer)) {
				if(WaterBoosterStatus == 2 && WaterBoosterTimeout > WB_LOW_CONSUME_MAX_TIME) goto xWaterBooster_GO_OFF;
			} else if(FeedPumpWork <= FEEDPUMP_MAX_WORK_TIME_ERR) {
				FeedPumpWork = 0;
				MC.dRelay[RFEEDPUMP].set_ON();
			}
		}
		// Fill tank
#ifndef TANK_ANALOG_LEVEL
		if(MC.sInput[TANK_LOW].get_Input()) {
			if(MC.dRelay[RFILL].get_Relay()) {
				taskENTER_CRITICAL();
				Charts_FillTank_work += TIME_SLICE_PUMPS * 100 / 1000; // in percent
				taskEXIT_CRITICAL();
			} else if(!(CriticalErrors & ~ERRC_TankEmpty)) {
				FillingTankLastLevel = 0;
				MC.dRelay[RFILL].set_ON();	// Start filling tank
			}
		}
		if(MC.sInput[TANK_FULL].get_Input()) {
			if(MC.dRelay[RFILL].get_Relay()) {
				FillingTankLastLevel = 0;
				MC.dRelay[RFILL].set_OFF();	// Stop filling tank
			}
		}
#else
		if(MC.dRelay[RFILL].get_Relay()) {
			if(LowConsumeMode) {
				if((int32_t)MC.sADC[LTANK].get_Value() >= (int32_t)MC.Option.LTank_LowConsumeMin + FILLING_TANK_LOW_CONSUME_TIME * FILLING_TANK_STEP / MC.Option.FillingTankTimeout)
					MC.dRelay[RFILL].set_OFF();	// Stop filling tank
				AfterFilledTimer = MC.Option.LTank_AfterFilledTimer * 1000;
				FillingTankLastLevel = 0;
			} else {
				if(MC.sADC[LTANK].get_Value() >= MC.sADC[LTANK].get_maxValue()) {
					FillingTankLastLevel = 0;
					MC.dRelay[RFILL].set_OFF();	// Stop filling tank
				}
			}
			//taskENTER_CRITICAL();
			Charts_FillTank_work += TIME_SLICE_PUMPS * 100 / 1000; // in percent
			//taskEXIT_CRITICAL();
		} else {
			if(MC.sADC[LTANK].get_Value() <= (MC.sInput[REG_BACKWASH_ACTIVE].get_Input() ? MC.sADC[LTANK].get_maxValue() - FILL_TANK_REGEN_DELTA : LowConsumeMode ? MC.Option.LTank_LowConsumeMin : rtcSAM3X8.get_hours() == MC.Option.LTank_Hour ? MC.Option.LTank_Hour_Low : MC.Option.LTANK_Low)) {
				if(!(CriticalErrors & ~ERRC_TankEmpty)) {
					if(!LowConsumeMode || (!MC.dRelay[RFEEDPUMP].get_Relay() && WaterBoosterStatus == 0)) {
						if(LowConsumeMode && !(MC.RTC_store.Work & (RTC_Work_Regen_F1 | RTC_Work_Regen_F2))) AfterFilledTimer = MC.Option.LTank_AfterFilledTimer * 1000;
						FillingTankLastLevel = 0;
						MC.dRelay[RFILL].set_ON();	// Start filling tank
					}
				}
			}
		}
#endif  // TANK_ANALOG_LEVEL

		// Regenerating
		if(reg_active & 1) { // Regen Iron removing filter
			if(MC.dRelay[RSTARTREG].get_Relay()) {
				MC.dRelay[RSTARTREG].set_Relay(fR_StatusAllOff);
				if(MC.get_errcode() == ERR_START_REG && Errors[1] == 0) MC.clear_error();
			}
			if(!(MC.RTC_store.Work & RTC_Work_Regen_F1)) { // Started?
				MC.RTC_store.Work |= RTC_Work_Regen_F1;
				MC.dRelay[RWATEROFF1].set_ON();
				MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
				RegBackwashTimer = MC.Option.BackWashFeedPumpDelay;
				RegStart_Weight = Weight_value / 10;
				RegMaxFlow = 0;
				RegMinPress = 0xFFFF;
				NewRegenStatus = true;
				MC.RTC_store.UsedRegen = 0;
				NeedSaveRTC = RTC_SaveAll;
			}
		} else if(reg_active & 2) {	// Regen Softening filter
			if(MC.dRelay[RSTARTREG2].get_Relay()) {
				MC.dRelay[RSTARTREG2].set_Relay(fR_StatusAllOff);
				if(MC.get_errcode() == ERR_START_REG2 && Errors[1] == 0) MC.clear_error();
			}
			if(!(MC.RTC_store.Work & RTC_Work_Regen_F2)) { // Started?
				MC.RTC_store.Work |= RTC_Work_Regen_F2;
				MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
				RegStart_Weight = Weight_value / 10;
				NewRegenStatus = true;
				MC.RTC_store.UsedRegen = 0;
				NeedSaveRTC = RTC_SaveAll;
			}
		}

		vTaskDelay(TIME_SLICE_PUMPS); // ms
	}
	vTaskSuspend(NULL);
}

// Service ///////////////////////////////////////////////
// Графики в ОЗУ, счетчики моточасов, сохранение статистики, дисплей
void vService(void *)
{
	static uint8_t  task_updstat_countm = rtcSAM3X8.get_minutes();
	static uint8_t  task_dailyswitch_countm = task_updstat_countm;
	static uint8_t  task_every_min = task_updstat_countm;
	static TickType_t timer_sec = GetTickCount(), timer_idle = 0, timer_total = 0;
#ifdef CHECK_DRAIN_PUMP
	static uint32_t tmp;
#endif

	for(;;)
	{
		if(vPumpsNewError != 0) {
			set_Error(vPumpsNewError, (char*)"vPumps");
			vPumpsNewError = 0;
		}
		register TickType_t t = xTaskGetTickCount();
		if(t - timer_sec >= 1000) { // 1 sec
			WDT_Restart(WDT);  						// Watchdog reset
			extern TickType_t xTickCountZero;
			TickType_t ttmpt = t - xTickCountZero - timer_total;
			if(ttmpt > 5000) {
				if(t < timer_sec) {
					vTaskResetRunTimeCounters();
					timer_idle = timer_total = 0;
				} else {
					TickType_t ttmp = timer_idle;
					MC.CPU_LOAD = 100 - (((timer_idle = vTaskGetRunTimeCounter(xTaskGetIdleTaskHandle())) - ttmp) / (ttmpt / 100UL));
					timer_total = t - xTickCountZero;
				}
			}
			timer_sec = t;
			// Drain OFF
			if(TimerDrainingWater) {
				TimerDrainingWater--;
				if(TimerDrainingWater <= 2) {
					MC.dRelay[RDRAIN].set_OFF();
					if(TimerDrainingWater == 0) {
						MC.WorkStats.UsedDrain = (int32_t)MC.WorkStats.UsedDrain - ((int32_t)MC.sFrequency[FLOW].PassedRest - UsedDrainRest) * 10 / MC.sFrequency[FLOW].get_kfValue();
						if(MC.WorkStats.UsedDrain < MC.Option.MinDrainLiters) {
							journal.jprintf("Not enough water drained: %.1d!\n", MC.WorkStats.UsedDrain);
							set_Error(ERR_FEW_LITERS_DRAIN, (char*)__FUNCTION__);
						}
					}
				}
			} else if(TimerDrainingWaterAfterRegen && --TimerDrainingWaterAfterRegen == 0) {
				MC.dRelay[RDRAIN].set_OFF();
				MC.dRelay[RDRAIN2].set_OFF();
			}
			// Drain tank's silt
			if(DrainingSiltFlag == 1) {
				if(DrainingSiltNowTimer == 0) {
#ifdef RSILT
					MC.dRelay[RSILT].set_OFF();
#endif
					MC.WorkStats.UsedDrainSiltL100 = 0;
					if(MC.Option.DrainSiltAfterL100 == 0) MC.Option.DrainSiltAfterL100 = 1;
					DrainingSiltFlag = 255 - (60); // (..) sec pause before checking leakage
				} else DrainingSiltNowTimer--;
			}
			if(RWATERON_Switching < 0) {
				if(++RWATERON_Switching == 0) {
					RWATERON_Switching = RWATERON_TIME;
					if(MC.dRelay[RWATEROFF1].get_Relay()) MC.dRelay[RWATEROFF1].set_OFF();
					MC.dRelay[RWATERON].set_ON();
				}
			} else if(RWATERON_Switching > 0) RWATERON_Switching--;
			if(RegBackwashTimer) RegBackwashTimer--;

			uint8_t m = rtcSAM3X8.get_minutes();
			if(m != task_updstat_countm) { 								// Через 1 минуту
				task_updstat_countm = m;
				MC.updateCount();                                       // Обновить счетчики
				if(task_updstat_countm == 59) MC.save_WorkStats();		// сохранить раз в час
				Stats.History();                                        // запись истории в файл

				uint32_t ut = rtcSAM3X8.unixtime();
				if(MC.Option.RFILL_HoursRepeatPulse && ut - MC.RFILL_last_time_ON > (uint32_t) MC.Option.RFILL_HoursRepeatPulse * 60 * 60) {
					MC.dRelay[RFILL].set_ON();
					MC.RFILL_last_time_ON = 0;
				} else if(MC.Option.TankFillingTimeMax && MC.dRelay[RFILL].get_Relay() && ut - MC.RFILL_last_time_ON > MC.Option.TankFillingTimeMax * 60) {
					CriticalErrors |= ERRC_TankFillingLong;
					set_Error(ERR_TANK_FILLING_LONG, (char*)"vService");
					MC.dRelay[RFILL].set_Relay(fR_StatusAllOff);
				}
				// Water did not consumed a long time ago.
				if(MC.Option.DrainAfterNoConsume && MC.Option.DrainTime) {
					if(ut - (MC.WorkStats.UsedLastTime > MC.WorkStats.LastDrain ? MC.WorkStats.UsedLastTime : MC.WorkStats.LastDrain ? MC.WorkStats.LastDrain : ut) >= MC.Option.DrainAfterNoConsume && !CriticalErrors) {
						MC.WorkStats.LastDrain = rtcSAM3X8.unixtime();
						TimerDrainingWater = MC.Option.DrainTime;
						MC.WorkStats.UsedDrain = 0;
						UsedDrainRest = MC.sFrequency[FLOW].PassedRest;
						MC.dRelay[RDRAIN].set_ON();
					}
				}
				if(GETBIT(MC.Option.flags2, fDrainSiltTank) && DrainingSiltFlag == 0 && MC.WorkStats.UsedDrainSiltL100 >= MC.Option.DrainSiltAfterL100
						&& MC.sADC[LTANK].get_Value() > MC.Option.LTank_LowConsumeMin) {
					ut -= MC.WorkStats.UsedLastTime;
					if(ut > MC.Option.DrainSiltAfterNotUsed * 3600 || MC.WorkStats.UsedDrainSiltL100 >= MC.Option.DrainSiltAfterL100 + MC.Option.DrainSiltAfterL100 / 2 + 1) {
						DrainingSiltFlag = 1;
#ifdef RSILT
						MC.dRelay[RSILT].set_ON();
#endif
						DrainingSiltNowTimer = MC.Option.DrainSiltTime * 10;
					}
				}

				if(MC.dRelay[RSTARTREG].get_Relay() && !(MC.RTC_store.Work & RTC_Work_Regen_F1)) { // 1 minute passed but regeneration did not start
					if(rtcSAM3X8.unixtime() - RegenStarted > START_REGEN_WAIT_TIME) set_Error(ERR_START_REG, (char*)__FUNCTION__);
					MC.dRelay[RSTARTREG].set_OFF();
					goto xOtherTask_1min;
				}
				if(MC.dRelay[RSTARTREG2].get_Relay() && !(MC.RTC_store.Work & RTC_Work_Regen_F2)) { // 1 minute passed but regeneration did not start
					if(rtcSAM3X8.unixtime() - RegenStarted > START_REGEN_WAIT_TIME) set_Error(ERR_START_REG2, (char*)__FUNCTION__);
					MC.dRelay[RSTARTREG2].set_OFF();
					goto xOtherTask_1min;
				}
				if(!CriticalErrors) {
					int8_t err = MC.get_errcode();
					if((err == ERR_FLOODING || err == ERR_TANK_EMPTY) && Errors[1] == 0) MC.clear_error();
					uint8_t h = rtcSAM3X8.get_hours();
					if(h == NOT_CITICAL_ALARM_HOUR && MC.WorkStats.RegenSofteningCntAlarm == 0 && MC.Option.RegenSofteningCntAlarm && err != ERR_SALT_FINISH) {
						set_Error(ERR_SALT_FINISH, (char*)__FUNCTION__);
					}
					uint8_t end_hour = (MC.Option.RegenHour & 0x1F) + ((MC.Option.RegenHour/*& 0xE0*/)>>5);
					bool hours_ok = h >= (MC.Option.RegenHour & 0x1F) && (h <= end_hour);
					if(!hours_ok && end_hour >= 24) hours_ok = h <= end_hour - 24;
					if(!(MC.RTC_store.Work & RTC_Work_Regen_MASK) && !LowConsumeMode && hours_ok) {
						uint32_t need_regen = 0;
						if(MC.get_NeedRegen() || (MC.WorkStats.Flags & WS_F_StartRegen)) {
							need_regen |= RTC_Work_Regen_F1;
						} else if(MC.get_NeedRegenSoftening() || (MC.WorkStats.Flags & WS_F_StartRegenSoft)) {
							need_regen |= RTC_Work_Regen_F2;
						} else if(MC.get_RegenExpired()) {
							set_Error(ERR_REGEN_EXPIRED, (char*)__FUNCTION__);
						} else if(MC.get_RegenExpiredSoftening()) {
							set_Error(ERR_REGEN2_EXPIRED, (char*)__FUNCTION__);
						}
						if(need_regen) {
							if(DrainingSiltFlag == 0 && MC.WorkStats.UsedDrainSiltL100 > 0 && GETBIT(MC.Option.flags2, fDrainSiltTankBeforeRegen)) {
								DrainingSiltFlag = 1;
#ifdef RSILT
								MC.dRelay[RSILT].set_ON();
#endif
								DrainingSiltNowTimer = MC.Option.DrainSiltTime * 10;
							}
							if(DrainingSiltFlag == 0) {
#ifdef TANK_ANALOG_LEVEL
								if(MC.sADC[LTANK].get_Value() >= MC.sADC[LTANK].get_maxValue())
#else
								if(MC.sInput[TANK_FULL].get_Input())
#endif
								{
									if((need_regen & RTC_Work_Regen_F1)) {
										if(!MC.dRelay[RSTARTREG].get_Relay()) {
											MC.dRelay[RWATEROFF1].set_ON();
											if(!MC.dRelay[RWATERON].get_Relay() && !RWATERON_Switching) {
												MC.dRelay[RSTARTREG].set_ON();
												journal.jprintf_date("Regen F1 start\n");
												if(RegenStarted == 0) RegenStarted = rtcSAM3X8.unixtime();
												MC.WorkStats.Flags &= ~WS_F_RegenPreparing;
											} else if(MC.dRelay[RWATERON].get_Relay()) {
												RWATERON_Switching = RWATERON_TIME;
												MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
												MC.WorkStats.Flags |= WS_F_RegenPreparing;
											}
										}
									} else if((need_regen & RTC_Work_Regen_F2) && !MC.dRelay[RSTARTREG2].get_Relay()) {
										if(!MC.dRelay[RWATERON].get_Relay() && !RWATERON_Switching) {
											MC.dRelay[RSTARTREG2].set_ON();
											journal.jprintf_date("Regen F2 start\n");
											if(RegenStarted == 0) RegenStarted = rtcSAM3X8.unixtime();
											MC.WorkStats.Flags &= ~WS_F_RegenPreparing;
										} else if(MC.dRelay[RWATERON].get_Relay()) {
											RWATERON_Switching = RWATERON_TIME;
											MC.dRelay[RWATERON].set_Relay(fR_StatusAllOff);
											MC.WorkStats.Flags |= WS_F_RegenPreparing;
										}
									}
								} else {
									FillingTankLastLevel = 0;
									MC.dRelay[RFILL].set_ON();	// Start filling tank
								}
							}
						} else if(MC.WorkStats.Flags & WS_F_RegenPreparing) { // Not need to regen while in preparing
							MC.dRelay[RWATEROFF1].set_OFF();
							MC.dRelay[RWATERON].set_ON();
							MC.WorkStats.Flags &= ~WS_F_RegenPreparing;
#ifdef TEST_BOARD
							journal.jprintf("Not need regen %d\n", MC.WorkStats.Flags);
#endif
						}
					}
				}
			} else { // Every 1 sec except updstat sec
				if(MC.RFILL_last_time_ON == 0) {
					MC.dRelay[RFILL].set_OFF();
					MC.RFILL_last_time_ON = rtcSAM3X8.unixtime();
				}
				if(NeedSaveWorkStats) {
					if(MC.save_WorkStats() == OK) NeedSaveWorkStats = 0;
				} else if((NeedSaveRTC & (1<<bRTC_Urgently)) || (NeedSaveRTC && m != task_every_min)) {
					task_every_min = m;
					uint8_t err = update_RTC_store_memory();
					if(err) {
						journal.jprintfopt("Error %d save RTC!\n", err);
						set_Error(ERR_RTC_WRITE, (char*)__FUNCTION__);
					}
				} else {
					if(MC.RTC_store.Work & RTC_Work_Regen_F1) {	// Regen Iron removing filter
						if(!MC.sInput[REG_ACTIVE].get_Input() && !MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) { // finish?
							MC.WorkStats.UsedLastRegen = MC.RTC_store.UsedRegen;
							MC.RTC_store.UsedRegen = 0;
							MC.WorkStats.DaysFromLastRegen = 0;
							MC.WorkStats.RegCnt++;
							MC.WorkStats.UsedSinceLastRegen = -MC.RTC_store.UsedToday;
							MC.RTC_store.Work &= ~RTC_Work_Regen_F1;
							taskENTER_CRITICAL();
							NeedSaveRTC |= (1<<bRTC_Work) | (1<<bRTC_UsedRegen) | (1<<bRTC_Urgently);
							taskEXIT_CRITICAL();
							NeedSaveWorkStats = 1;
							RegStart_Weight -= Weight_value / 10;
							journal.jprintf_date("Regen F1 finished, Used: %d, reagent: %d g, time: %d s. ", MC.WorkStats.UsedLastRegen, RegStart_Weight, rtcSAM3X8.unixtime() - RegenStarted);
							journal.jprintf("Max flow: %.3d (%.3d mh), Min press: %.2d\n", RegMaxFlow, 10000UL * RegMaxFlow / MC.FilterTankSquare, RegMinPress);
							if(MC.Option.DrainingWaterAfterRegen) {
								TimerDrainingWaterAfterRegen = MC.Option.DrainingWaterAfterRegen;
								MC.dRelay[RDRAIN].set_ON();
							}
							RWATERON_Switching = -(int16_t)TimerDrainingWaterAfterRegen - 1;
							if(MC.WorkStats.UsedLastRegen < MC.Option.MinRegenLiters) {
								set_Error(ERR_FEW_LITERS_REG, (char*)__FUNCTION__);
							} else {
								if(GETBIT(MC.Option.flags2, fCheckDrainPump) && DrainPumpTimeLast < RegenStarted) {
									set_Error(ERR_DRAIN_PUMP_NOT_WORK, (char*)__FUNCTION__);
								}
								if(RegStart_Weight < MC.Option.MinRegenWeightDecrease) {
									set_Error(ERR_REG_FEW_WEIGHT_CONSUME, (char*)__FUNCTION__);
								}
							}
							//Passed100Count = 0; <- выключено, будет +1..100л
							MC.WorkStats.UsedDrainSiltL100 = MC.Option.DrainSiltAfterL100 - DRAIN_SILT_AFTER_REGEN;
							RegenStarted = 0;
						} else if(NewRegenStatus) {
							if(MC.WorkStats.Flags & WS_F_StartRegen) {
								MC.WorkStats.Flags &= ~WS_F_StartRegen;
								NeedSaveWorkStats = 1;
							}
							journal.jprintf_date("Regen F1 begin\n");
							NewRegenStatus = false;
						}
					} else if(MC.RTC_store.Work & RTC_Work_Regen_F2) { // Regen Softening filter
						if(!MC.sInput[REG2_ACTIVE].get_Input()) { // Finish?
							MC.WorkStats.UsedLastRegenSoftening = MC.RTC_store.UsedRegen;
							MC.WorkStats.UsedSinceLastRegen += MC.RTC_store.UsedRegen; // добавим в расход 1-го фильтра
							MC.RTC_store.UsedRegen = 0;
							MC.WorkStats.DaysFromLastRegenSoftening = 0;
							MC.WorkStats.RegCntSoftening++;
							if(MC.WorkStats.RegenSofteningCntAlarm) MC.WorkStats.RegenSofteningCntAlarm--;
							MC.WorkStats.UsedSinceLastRegenSoftening = -MC.RTC_store.UsedToday;
							MC.RTC_store.Work &= ~RTC_Work_Regen_F2;
							taskENTER_CRITICAL();
							NeedSaveRTC |= (1<<bRTC_Work) | (1<<bRTC_UsedRegen) | (1<<bRTC_Urgently);
							taskEXIT_CRITICAL();
							NeedSaveWorkStats = 1;
							RegStart_Weight -= Weight_value / 10;
							journal.jprintf_date("Regen F2 finished, Used: %d, reagent: %d g, time: %d s.\n", MC.WorkStats.UsedLastRegenSoftening, RegStart_Weight, rtcSAM3X8.unixtime() - RegenStarted);
							journal.jprintf("Max flow: %.3d (%.3d mh), Min press: %.2d\n", RegMaxFlow, 10000UL * RegMaxFlow / MC.FilterTankSoftenerSquare, RegMinPress);
							if(MC.Option.DrainingWaterAfterRegenSoftening) {
								TimerDrainingWaterAfterRegen = MC.Option.DrainingWaterAfterRegenSoftening;
								MC.dRelay[RDRAIN2].set_ON();
							}
							RWATERON_Switching = -(int16_t)TimerDrainingWaterAfterRegen - 1;
							if(MC.WorkStats.UsedLastRegenSoftening < MC.Option.MinRegenLitersSoftening) {
								set_Error(ERR_FEW_LITERS_REG, (char*)__FUNCTION__);
							} else {
								if(GETBIT(MC.Option.flags2, fCheckDrainPump) && DrainPumpTimeLast < RegenStarted) {
									set_Error(ERR_DRAIN_PUMP_NOT_WORK, (char*)__FUNCTION__);
								}
								if(RegStart_Weight < MC.Option.MinRegenWeightDecreaseSoftening) {
									set_Error(ERR_REG_FEW_WEIGHT_CONSUME, (char*)__FUNCTION__);
								}
							}
							//Passed100Count = 0; <- выключено, будет +1..100л
							MC.WorkStats.UsedDrainSiltL100 = MC.Option.DrainSiltAfterL100 - DRAIN_SILT_AFTER_REGEN;
							RegenStarted = 0;
						} else if(NewRegenStatus) {
							if(MC.WorkStats.Flags & WS_F_StartRegenSoft) {
								MC.WorkStats.Flags &= ~WS_F_StartRegenSoft;
								NeedSaveWorkStats = 1;
							}
							journal.jprintf_date("Regen F2 begin\n");
							NewRegenStatus = false;
						}
					}
					uint32_t tt = rtcSAM3X8.get_hours() * 100 + m;
					for(uint8_t i = 0; i < DAILY_SWITCH_MAX; i++) {
						if(MC.Option.DailySwitch[i].Device == 0) break;
						uint32_t st = MC.Option.DailySwitch[i].TimeOn * 10;
						uint32_t end = MC.Option.DailySwitch[i].TimeOff * 10;
						uint32_t tmr = MC.dRelay[MC.Option.DailySwitch[i].Device].get_TimerOn();
						if(st == end && tmr && TIMER_TO_SHOW_STATUS - 1000 > tmr && MC.dRelay[MC.Option.DailySwitch[i].Device].get_Relay()) { // after 1 sec -> off
							MC.dRelay[MC.Option.DailySwitch[i].Device].set_Relay(-fR_StatusDaily);
						} else if(m != task_dailyswitch_countm) {
							task_dailyswitch_countm = m;
							MC.dRelay[MC.Option.DailySwitch[i].Device].set_Relay(((end >= st && tt >= st && tt <= end) || (end < st && (tt >= st || tt <= end)))
									&& !MC.NO_Power /* && !LowConsumeMode */ ? fR_StatusDaily : -fR_StatusDaily);
						}
					}
				}
				// Tank fill/leakage check
				if(DrainingSiltFlag > 1) DrainingSiltFlag++;
				if(MC.dRelay[RFILL].get_Relay()) {
					if(MC.Option.FillingTankTimeout && MC.RFILL_last_time_ON) { // Check tank filling speed
						// FillingTankLastLevel == 0 - Start watching
						if(TankCheckFlag == 1 && FillingTankLastLevel && DrainingSiltFlag == 0) { // No water consuming from tank
							if(++FillingTankTimer >= MC.Option.FillingTankTimeout) {
								int16_t d = MC.sADC[LTANK].get_Value() - FillingTankLastLevel;
								if(d < (MC.Option.TankCheckPercent ? MC.Option.TankCheckPercent * 100 : FILLING_TANK_STEP)) {
									set_Error(ERR_TANK_NO_FILLING, (char*)"vService");
									journal.jprintf("FILLING %d sec = +%.2d%%!\n", MC.Option.FillingTankTimeout, d);
								}
								FillingTankLastLevel = MC.sADC[LTANK].get_Value();
								FillingTankTimer = 0;
							}
						} else {
							TankCheckFlag = 1;
							FillingTankLastLevel = MC.sADC[LTANK].get_Value();
							FillingTankTimer = 0;
						}
					}
				} else if(TankCheckFlag || DrainingSiltFlag) {
					TankCheckFlag = 0;
					FillingTankLastLevel = MC.sADC[LTANK].get_Value();
					FillingTankTimer = 0;
				} else if(MC.Option.TankCheckPercent) {
					int16_t d = FillingTankLastLevel - MC.sADC[LTANK].get_Value();
 					if(d > MC.Option.TankCheckPercent * 100) {
						set_Error(ERR_TANK_LEAKAGE, (char*)"vService");
						journal.jprintf("LEAKAGE %.2d%%!\n", d);
						DrainingSiltFlag = 2;
					}
					if(++FillingTankTimer > LEAKAGE_TANK_RESTART_TIME) DrainingSiltFlag = 255;
				}
#ifdef CHECK_DRAIN_PUMP
				bool skip_this_iteration = false;
				if(GETBIT(MC.Option.flags2, fCheckDrainPump)) {
					if(GETBIT(MC.Option.flags2, fDrainPumpRelay) &&
							(DrainPumpRelayStatus == MODBUS_RELAY_CMD_ON || DrainPumpRelayStatus == MODBUS_RELAY_CMD_OFF)) {
#ifdef MODBUS_DRAIN_PUMP_RELAY_ADDR
						skip_this_iteration = true;
						int8_t err = Modbus.MODBUS_PUMP_FUNC(MODBUS_DRAIN_PUMP_RELAY_ADDR, MODBUS_DRAIN_PUMP_RELAY_ID,
									DrainPumpRelayStatus == MODBUS_RELAY_CMD_ON ? MODBUS_DRAIN_PUMP_ON_CMD : MODBUS_DRAIN_PUMP_OFF_CMD);
						if(err) {
							if(++DrainPumpRelayErrCnt == MODBUS_OTHER_MAX_ERRORS) {
								if(MC.get_errcode() != ERR_DRAIN_PUMP_RELAY_LINK) journal.jprintf("%s Link Error %d!\n", "PUMP Relay", err);
								set_Error(ERR_DRAIN_PUMP_RELAY_LINK, (char*)"vService");
							} else if(GETBIT(MC.Option.flags, fPWMLogErrors)) journal.jprintf_time("%s Read Error %d\n", "PUMP Relay", err);
						} else {
							DrainPumpRelayErrCnt = 0;
							if(DrainPumpRelayStatus == MODBUS_RELAY_CMD_OFF) {
								journal.jprintf_time("DRAIN PUMP -> OFF!\n", err);
#ifdef MODBUS_DRAIN_PUMP_ON_PULSE
								DrainPumpRelayStatus = MODBUS_RELAY_CMD_ON;	// Pulse 1 sec
#else
								DrainPumpRelayStatus = MODBUS_RELAY_OFF;
#endif
							} else {
								DrainPumpRelayStatus = MODBUS_RELAY_ON;
								journal.jprintfopt_time("%s Relay: %s\n", "PUMP", DrainPumpRelayStatus == MODBUS_RELAY_CMD_ON ? "ON" : "OFF");
							}
#endif
						}
					} else {
						PumpReadCounter++;
#ifdef MODBUS_SEPTIC_PUMP_ADDR
						if(PumpReadCounter == MODBUS_PUMP_PERIOD / 2) {
							// to do...
						} else
#endif
						if(PumpReadCounter >= MODBUS_PUMP_PERIOD) {
							PumpReadCounter = 0;
							skip_this_iteration = true;
							int8_t err = Modbus.readInputRegisters32(MODBUS_DRAIN_PUMP_ADDR, PWM_POWER, &tmp);
							if(err == OK) {
								tmp /= 10;
								if(tmp > MC.Option.DrainPumpMinPower * 10) { // работает
									if(DrainPumpPower <= MC.Option.DrainPumpMinPower * 10) DrainPumpTimeLast = rtcSAM3X8.unixtime(); // время включения
									else if(MC.Option.DrainPumpMaxTime && MC.get_errcode() != ERR_DRAIN_PUMP_TOOLONG
											&& rtcSAM3X8.unixtime() - DrainPumpTimeLast > MC.Option.DrainPumpMaxTime * 30) {
										set_Error(ERR_DRAIN_PUMP_TOOLONG, (char*)"vService");
										DrainPumpRelayStatus = MODBUS_RELAY_CMD_OFF;
									}
								}
								DrainPumpPower = tmp;
								DrainPumpErrCnt = 0;
							} else {
								PumpReadCounter = MODBUS_PUMP_PERIOD - 1;
								if(++DrainPumpErrCnt == MODBUS_OTHER_MAX_ERRORS) {
									if(MC.get_errcode() != ERR_DRAIN_PUMP_LINK) journal.jprintf("%s Link Error %d!\n", "PUMP", err);
									set_Error(ERR_DRAIN_PUMP_LINK, (char*)"vService");
									//DrainPumpErrCnt = 0;
								} else if(GETBIT(MC.Option.flags, fPWMLogErrors)) journal.jprintf_time("%s Read Error %d\n", "PUMP", err);
							}
						}
					}
				}
				if(!skip_this_iteration)
#endif //CHECK_DRAIN_PUMP
#if defined(MODBUS_SEPTIC_HEAT_RELAY_ADDR) && defined(SEPTIC_LOW_TEMP)
				if(GETBIT(MC.Option.flags2, fSepticHeatRelay)) {
					bool input = MC.sInput[SEPTIC_LOW_TEMP].get_Input();
					if(input != SepticRelayStatus) { // -> ON/OFF
						int8_t err = Modbus.MODBUS_SEPTIC_HEAT_FUNC(MODBUS_SEPTIC_HEAT_RELAY_ADDR, MODBUS_SEPTIC_HEAT_RELAY_ID,
								input ? MODBUS_SEPTIC_HEAT_RELAY_ON : MODBUS_SEPTIC_HEAT_RELAY_OFF);
						if(err) {
							if(++SepticRelayErrCnt == MODBUS_OTHER_MAX_ERRORS) {
								if(MC.get_errcode() != ERR_SEPTIC_RELAY_LINK) journal.jprintf("%s Link Error %d!\n", "SEPTIC Relay", err);
								set_Error(ERR_SEPTIC_RELAY_LINK, (char*)"vService");
							} else if(GETBIT(MC.Option.flags, fPWMLogErrors)) journal.jprintf_time("%s Read Error %d\n", "SEPTIC Relay", err);
						} else {
							SepticRelayStatus = input;
							SepticRelayErrCnt = 0;
							journal.jprintfopt_time("%s Relay: %s\n", "SEPTIC", input ? "ON" : "OFF");
						}
					}
				}
#else
				;
#endif
			}
			// every 1 sec
#ifdef PIN_LED_SRV_INFO

#endif
xOtherTask_1min:
			if(++task_updstat_chars >= MC.get_tChart()) { // пришло время
				task_updstat_chars = 0;
				MC.updateChart();                                       // Обновить графики
			}
			Stats.CheckCreateNewFile();
			if(ResetDUE_countdown && --ResetDUE_countdown == 0) Software_Reset();      // Сброс
			if(MC.fNetworkReset && --MC.fNetworkReset == 0) {
				if(!initW5200(true)) MC.fNetworkReset = 60;                                  // Инициализация сети с выводом инфы в консоль
				else {
					for(uint8_t i = 0; i < W5200_THREAD; i++) SETBIT1(Socket[i].flags,fABORT_SOCK);  // Признак инициализации сокета, надо прерывать передачу в сервере
					MC.num_resW5200++;                                // Добавить счетчик инициализаций
				}
			}
		} // 1 sec
		vTaskDelay(1); // задержка чтения уменьшаем загрузку процессора
	}
	vTaskSuspend(NULL);
}
