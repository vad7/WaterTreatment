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
 */
// --------------------------------------------------------------------------------
// Описание базовых классов для работы
// --------------------------------------------------------------------------------
#ifndef MainClass_h
#define MainClass_h
#include "Constant.h"                       // Вся конфигурация и константы проекта Должен быть первым !!!!
#include "Hardware.h"
#include "Message.h"
#include "Information.h"
extern char *MAC2String(byte* mac);

#define I2C_COUNT_EEPROM_HEADER 0xAB
struct type_WorkStats {
	uint8_t  Header;
	uint32_t UsedSinceLastRegen;	// Liters
	uint32_t UsedTotal;				// Liters
	uint16_t UsedToday;				// Liters
	uint16_t UsedYesterday;			// Liters
	uint16_t UsedAverageDay;		// Liters
	uint16_t UsedLastRegen;			// Liters
	uint16_t UsedLastRegenSoftening;// Liters
	uint16_t UsedDischarge;			// Liters
	uint16_t DaysFromLastRegen;
	uint16_t DaysFromLastRegenSoftening;
	uint16_t RegCnt;
	uint8_t  WeekDay; 				// 1-7 active wday
} __attribute__((packed));


#define RTC_Work_WeekDay_Mask		0x07	// Active weekday (1-7)
#define RTC_Work_NeedRegen_Mask		0x30	// 0 - not, 1 - wait a regen iron hour, 2 - regen iron in process, 3 - regen softener in process
#define RTC_Work_NeedRegen_WaitIron	0x10
#define RTC_Work_NeedRegen_Iron		0x20
#define RTC_Work_NeedRegen_Softener	0x30

struct type_RTC_memory { // DS3231/DS3232 used alarm memory, starts from 0x07, size 7 bytes
	uint16_t UsedToday;
	uint8_t  Work;			// NeedRegen + WeekDay
} __attribute__((packed));


int8_t		WaterBoosterStatus = 0; // 0 - выключено, 1 - вкл твердотельное, 2 - вкл обычное, 3 - выкл твердотельное, -1 - выкл твердотельное, -2 - выкл обычное, -3 вкл твердотельное
uint32_t	TimeFeedPump = 0;
uint8_t		fNeedRegen = 0;			// 0 - not, 1 - wait a regen hour, 2 - regen in process
int8_t		vPumpsNewError = 0;
int8_t		Errors[10];				// Active Errors array

int32_t motohour_IN_work = 0;  // рабочий для счетчиков - энергия потребленная, мВт
uint16_t task_updstat_chars = 0;

//  Работа с отдельными флагами type_option
#define fWebStoreOnSPIFlash 0				// флаг, что веб морда лежит на SPI Flash, иначе на SD карте
#define fBeep               1               // флаг Использование звука
#define fHistory            2               // флаг записи истории на карту памяти
#define f1Wire2TSngl		3				// На 2-ой шине 1-Wire(DS2482) только один датчик
#define f1Wire3TSngl		4				// На 3-ей шине 1-Wire(DS2482) только один датчик
#define f1Wire4TSngl		5				// На 4-ей шине 1-Wire(DS2482) только один датчик
#define fLogWirelessSensors 6				// Логировать обмен между беспроводными датчиками
#define fPWMLogErrors  		7               // флаг писать в лог ошибки электросчетчика
#define fDontRegenOnWeekend	8				// Не делать регенерацию в выходные
#define fDischargeEveryDay	9				// Сливать воду, если не было расхода за день
 
// Структура для хранения опций
struct type_option {
	uint8_t ver;						// номер версии для сохранения
	uint16_t flags;						// Флаги опций до 16 флагов
	uint16_t tChart;					// период графиков в секундах!!
	uint32_t FeedPumpMaxFlow;			// Максимальный проток до которого распределяется время включения дозатора, литры в час
	uint8_t  RegenHour;					// Час регенерации (0..23)
	uint16_t UsedBeforeRegen;			// Количество литров до регенерации
	uint16_t MinPumpOnTime;				// Минимальное время включения дозатора, мсек
	uint16_t MinRegenLiters;			// Тревога, если за регенерацию израсходовано меньше литров
	uint16_t MinDischargeLiters;		// Тревога, если слито (Discharge) при сбросе меньше литров
	uint16_t DischargeTime;				// Время слива воды, сек

} __attribute__((packed));

//  Работа с отдельными флагами type_DateTime
#define fUpdateNTP     0                // флаг Обновление часов по NTP при старте
#define fUpdateI2C     1                // флаг Обновление часов раз в час с I2C  часами
#define fUpdateByHTTP  2                // флаг Обновление по HTTP - спец страница: define HTTP_TIME_REQUEST

// Структура для хранения настроек времени, для удобного сохранения.
struct type_DateTime
{
    uint8_t flags;                        //  Флаги опций до 8 флагов
    int8_t timeZone;                      //  Часовой пояс
    char serverNTP[NTP_SERVER_LEN+1];     //  Адрес NTP сервера
    uint32_t saveTime;                    //  дата и время сохранения настроек в eeprom
};

//  Работа с отдельными флагами type_Network
#define fDHCP         0                  // флаг Использование DHCP
#define fPass         1                  // флаг Использование паролей
#define fInitW5200    2                  // флаг Ежеминутный контроль SPI для сетевого чипа
#define fNoAck        4                  // флаг Не ожидать ответа ACK
#define fNoPing       5                  // флаг Запрет пинга контроллера
#define fWebLogError  6					// Логировать ошибки
#define fWebFullLog   7					// Полный лог

// Структура для хранения сетевых настроек, для удобного сохранения.
struct type_Network
{
    uint16_t flags;                       // !save! Флаги
    IPAddress ip;                         // !save! ip адрес
    IPAddress sdns;                       // !save! сервер dns
    IPAddress gateway;                    // !save! шлюз
    IPAddress subnet;                     // !save!подсеть
    byte mac[6];                          // !save! mac адрес
    uint16_t resSocket;                   // !save! Время очистки сокетов секунды
    uint32_t resW5200;                    // !save! Время сброса чипа     секунды
    char passUser[PASS_LEN+1];            // !save! Пароль пользователя
    char passAdmin[PASS_LEN+1];           // !save! Пароль администратора
    uint16_t sizePacket;                  // !save! Размер пакета для отправки в байтах
    uint16_t port;                        // !save! порт веб сервера
    uint8_t delayAck;                     // !save! задержка мсек перед отправкой пакета
    char pingAdr[40];                     // !save! адрес для пинга, может быть в любом виде
    uint16_t pingTime;                    // !save! время пинга в секундах
};

// Структура для хранения переменных для паролей
struct type_Security
{
  char hashUser[80];                      // Хеш для пользователя
  uint16_t hashUserLen;                   // Длина хеша пользователя
  char hashAdmin[80];                     // Хеш для администратора
  uint16_t hashAdminLen;                  // Длина хеша администратора
};

// ------------------------- ОСНОВНОЙ КЛАСС --------------------------------------
class MainClass
{
public:
	void init();                                     // Конструктор
	// Информационные функции определяющие состояние
	void calculatePower();                           // Вычисление мощностей контуров
	 void eraseError();                              // стереть последнюю ошибку
	__attribute__((always_inline)) inline int8_t get_errcode(){return error;} // Получить код последней ошибки
	char  *get_lastErr(){return note_error;} // Получить описание последней ошибки, которая вызвала останов, при удачном запуске обнуляется
	void   scan_OneWire(char *result_str); // Сканирование шины OneWire на предмет датчиков
    inline TEST_MODE get_testMode(){return testMode;} // Получить текущий режим работы
    void   set_testMode(TEST_MODE t);    // Установить значение текущий режим работы
    void   StateToStr(char * ret);                 // Получить состояние в виде строки
    char  *TestToStr();                  // Получить режим тестирования

	uint32_t get_errorReadDS18B20();    // Получить число ошибок чтения датчиков температуры
	void     Reset_TempErrors();		// Сбросить счетчик ошибок всех датчиков

	int32_t save(void); 		        // Записать настройки в eeprom i2c на входе адрес с какого, на выходе код ошибки (меньше нуля) или количество записанных  байт
	int32_t load(uint8_t *buffer, uint8_t from_RAM); // Считать настройки из i2c или RAM, на выходе код ошибки (меньше нуля)
	int8_t loadFromBuf(int32_t adr,byte *buf);// Считать настройки из буфера на входе адрес с какого, на выходе код ошибки (меньше нуля)
	int8_t save_motoHour();             // запись счетчиков в ЕЕПРОМ
	int8_t load_motoHour();             // чтение счетчиков в ЕЕПРОМ

	//  ===================  К Л А С С Ы  ===========================
	// Датчики
#if	TNUMBER > 0
	sensorTemp sTemp[TNUMBER];           // Датчики температуры
#endif
	sensorADC sADC[ANUMBER];             // Датчик аналоговый
	sensorDiditalInput sInput[INUMBER];  // Контактные датчики
	sensorFrequency sFrequency[FNUMBER]; // Частотные датчики
	// Устройства  исполнительные
	devRelay dRelay[RNUMBER];           // Реле
	devPWM dPWM;						// Счетчик электроэнергии

	// Сервис
	Message message;                     // Класс уведомления

#ifdef MQTT
	clientMQTT clMQTT;                // MQTT клиент
#endif
	// Сетевые настройки
	boolean set_network(char *var, char *c);        // Установить параметр из строки
	char*   get_network(char *var,char *ret);       // Получить параметр из строки
	//  inline uint16_t get_sizePacket() {return Network.sizePacket;} // Получить размер пакета при передаче
	inline uint16_t get_sizePacket() {return 2048;} // Получить размер пакета при передаче

	uint8_t set_hashUser();                               // расчитать хеш для пользователя возвращает длину хеша
	uint8_t set_hashAdmin();                              // расчитать хеш для администратора возвращает длину хеша

	// Дата время
	boolean set_datetime(char *var, char *c);              //  Установить параметр дата и время из строки
	char*   get_datetime(char *var,char *ret);             //  Получить параметр дата и время из строки
	IPAddress get_ip() { return Network.ip;}               //  Получить ip адрес
	IPAddress get_sdns() { return Network.sdns;}           //  Получить sdns адрес
	IPAddress get_subnet() { return Network.subnet;}       //  Получить subnet адрес
	IPAddress get_gateway() { return Network.gateway;}     //  Получить gateway адрес
	void set_ip(IPAddress ip) {Network.ip=ip;}             //  Установить ip адрес
	void set_sdns(IPAddress sdns) {Network.sdns=sdns;}     //  Установит sdns адрес
	void set_subnet(IPAddress subnet) {Network.subnet=subnet;}  //  Установит subnet адрес
	void set_gateway(IPAddress gateway) {Network.gateway=gateway;}//  Установит gateway адрес
	uint16_t get_port() {return Network.port;}             //  получить порт вебсервера
    __attribute__((always_inline)) inline uint16_t get_NetworkFlags() { return Network.flags; }
	boolean get_NoAck() { return GETBIT(Network.flags,fNoAck);}  //  Получить флаг Не ожидать ответа ACK
	uint8_t get_delayAck() {return Network.delayAck;}      //  получить задержку перед отсылкой следующего пакета
	uint16_t get_pingTime() {return Network.pingTime;}     //  получить вермя пингования сервера, 0 если не надо
	char *  get_pingAdr() {return Network.pingAdr;}         //  получить адрес сервера для пингования
	boolean get_NoPing() { return GETBIT(Network.flags,fNoPing);} //  Получить флаг блокировки пинга
	char *  get_netMAC() {return MAC2String(Network.mac);}  //  получить мас адрес контроллера

	boolean get_DHCP() { return GETBIT(Network.flags,fDHCP);}    //  Получить использование DHCP
	byte *get_mac() { return Network.mac;}                 //  Получить mac адрес
	uint32_t socketRes() {return countResSocket;}          //  Получить число сбросов сокетов
	void add_socketRes() {countResSocket++;}               //  Добавить 1 к счетчику число сбросов сокетов
	uint32_t time_socketRes() {return Network.resSocket;}  //  Получить период сбросов сокетов
	uint32_t time_resW5200() {return Network.resW5200;}    //  Получить период сбросов W5200
	boolean get_fPass() { return GETBIT(Network.flags,fPass);}   //  Получить флаг необходимости идентификации
	boolean get_fInitW5200() { return GETBIT(Network.flags,fInitW5200);}  //  Получить флаг Контроля w5200

	// Параметры
	boolean set_option(char *var, float xx);                // Установить опции  из числа (float)
	char*   get_option(char *var, char *ret);              // Получить опции 

	// Опции
	uint8_t  get_Beep() {return GETBIT(Option.flags,fBeep);};           // подача звуковых сигналов
	uint8_t  get_WebStoreOnSPIFlash() {return GETBIT(Option.flags,fWebStoreOnSPIFlash);}// получить флаг хранения веб морды на флеш диске

	uint16_t get_flags() { return Option.flags; }					  // Все флаги
	void	 save_DumpJournal(void);

	// Времена
	void set_countNTP(uint32_t b) {countNTP=b;}             // Установить текущее время обновления по NTP, (секундах)
	uint32_t get_countNTP()  {return countNTP;}             // Получить время последнего обновления по NTP (секундах)
	void set_updateNTP(boolean b);                          // Установить синхронизацию по NTP
	boolean get_updateNTP();                                // Получить флаг возможности синхронизации по NTP
	unsigned long get_saveTime(){return  DateTime.saveTime;}// Получить время сохранения текущих настроек
	char* get_serverNTP() {return DateTime.serverNTP;}      // Получить адрес сервера
	void updateDateTime(int32_t  dTime);                    // После любого изменения часов необходимо пересчитать все времна которые используются
	boolean  get_updateI2C(){return GETBIT(DateTime.flags,fUpdateI2C);}// Получить необходимость обновления часов I2C
	unsigned long timeNTP;                                  // Время обновления по NTP в тиках (0-сразу обновляемся)

	__attribute__((always_inline)) inline uint32_t get_uptime() {return rtcSAM3X8.unixtime()-timeON;} // Получить время с последенй перезагрузки в секундах
	uint32_t get_startDT(){return timeON;}                  // Получить дату и время последеней перезагрузки

	void resetCount(boolean full);                          // Сборос сезонного счетчика моточасов
	void updateCount();                                     // Обновление счетчиков моточасов

	void set_uptime(unsigned long ttime){timeON=ttime;}     // Установить текущее время как начало старта контроллера

    uint8_t  get_fSD() { return fSD;}        				// Получить флаг наличия РАБОТАЮЩЕЙ СД карты
    void     set_fSD(uint8_t f) { fSD=f; }   				// Установить флаг наличия РАБОТАЮЩЕЙ СД карты
    uint8_t  get_fSPIFlash() { return fSPIFlash;}   		// Получить флаг наличия РАБОТАЮЩЕГО флеш диска
    void     set_fSPIFlash(uint8_t f) {fSPIFlash=f;}    	// Установить флаг наличия РАБОТАЮЩЕГО флеш диска
    TYPE_SOURSE_WEB get_SourceWeb();                    	// Получить источник загрузки веб морды

	// Переменные
	uint8_t CPU_IDLE;                                      // загрузка CPU
	uint32_t mRTOS;                                        // Память занимаемая задачами
	uint32_t startRAM;                                     // Свободная память при старте FREE Rtos - пытаемся определить свободную память при работе

	uint16_t num_resW5200;                                 // + текущее число сброса сетевого чипа
	uint16_t num_resMutexSPI;                              // + текущее число сброса митекса SPI
	uint16_t num_resMutexI2C;                              // + текущее число сброса митекса I2C
	uint16_t num_resMQTT;                                  // + число повторных инициализация MQTT клиента
	uint16_t num_resPing;                                  // + число не прошедших пингов

	uint16_t AdcVcc;                                       // напряжение питания

	type_Security Security;                              // хеш паролей
	boolean safeNetwork;                                   // Режим работы safeNetwork (сеть по умолчанию, паролей нет)

	// функции для работой с графикками
	uint16_t get_tChart(){return Option.tChart;}           // Получить время накопления ститистики в секундах
	void updateChart();                                     // обновить статистику
	void startChart();                                      // Запуститьь статистику
	char * get_listChart(char* str);				          // получить список доступных графиков
	void get_Chart(char *var,char* str);   				   // получить данные графика

	// графики не по датчикам (по датчикам она хранится внутри датчика)
	statChart ChartRCOMP;                                   // Статистика по включению компрессора

	TaskHandle_t xHandlePumps;
	TaskHandle_t xHandleBooster;
	TaskHandle_t xHandleReadSensor;                     // Заголовок задачи "Чтение датчиков"
	TaskHandle_t xHandleService;						// Задача обслуживания
	TaskHandle_t xHandleKeysLCD;
	TaskHandle_t xHandleUpdateWeb0;                     // Заголовок задачи "Веб сервер 0"
	TaskHandle_t xHandleUpdateWeb1;                     // Заголовок задачи "Веб сервер 1"
	TaskHandle_t xHandleUpdateWeb2;                     // Заголовок задачи "Веб сервер 2"

	boolean Task_vUpdate_run;							// задача vUpdate работает

	int8_t	Prepare_Temp(uint8_t bus);				// Запуск преобразования температуры
	// Настройки опций
	type_option Option;                  			// Опции

	uint8_t  NO_Power;					  // Нет питания основных узлов
	uint8_t  NO_Power_delay;
	boolean  fNetworkReset;				// Нужно сбросить сеть
    TEST_MODE testMode;                                  // Значение режима тестирования
	type_WorkStats WorkStats;               // Структура для хранения счетчиков периодическая запись
	type_RTC_memory RTC_store;

private:

	void resetSetting();                // Функция сброса настроек
	void relayAllOFF();                   // Все реле выключить
	int8_t check_crc16_eeprom(int32_t addr, uint16_t size);// Проверить контрольную сумму в EEPROM для данных на выходе ошибка, длина определяется из заголовка


	// Ошибки и описания
	int8_t error;                         // Код ошибки
	char   source_error[16];              // источник ошибки
	char   note_error[160+1];             // Строка c описанием ошибки формат "время источник:описание"
    uint8_t fSD;                          // Признак наличия SD карты: 0 - нет, 1 - есть, но пустая, 2 - есть, веб в наличии
    uint8_t fSPIFlash;                    // Признак наличия (физического) SPI флеш: 0 - нет, 1 - есть, но пустая, 2 - есть, веб в наличии

	// Различные времена
	type_DateTime DateTime;             // структура где хранится все что касается времени и даты
	uint32_t timeON;                      // время включения контроллера для вычисления UPTIME
	uint32_t countNTP;                    // число секунд с последнего обновления по NTP

	// Сетевые настройки
	type_Network Network;                 // Структура для хранения сетевых настроек
	uint32_t countResSocket;                // Число сбросов сокетов

	friend int8_t set_Error(int8_t err, char *nam );// Установка критической ошибки для класса 
};

#endif
