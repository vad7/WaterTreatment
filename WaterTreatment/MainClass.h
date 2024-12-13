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
 */
// --------------------------------------------------------------------------------
// Описание базовых классов для работы
// --------------------------------------------------------------------------------
#ifndef MainClass_h
#define MainClass_h
#include "Constant.h"                       // Вся конфигурация и константы проекта
#include "Hardware.h"
#include "Message.h"
#include "Information.h"
extern char *MAC2String(byte* mac);

#ifndef TEST_BOARD
	#define I2C_COUNT_EEPROM_HEADER 0xAB
#else
	#define I2C_COUNT_EEPROM_HEADER 0xAC
#endif
struct type_WorkStats {
	uint8_t  Header;
	uint32_t ResetTime;
	uint32_t LastDrain;				// time
	uint16_t UsedDrain;				// Liters*10
	uint32_t UsedLastTime;			// time
	uint32_t UsedTotal;				// Liters
	uint32_t UsedAverageDay;		// Liters, sum of UsedAverageDayNum
	uint16_t UsedAverageDayNum;
	uint16_t UsedYesterday;			// Liters
	uint16_t DaysFromLastRegen;
	uint16_t DaysFromLastRegenSoftening;
	int32_t  UsedSinceLastRegenSoftening;	// Liters
	int32_t  UsedSinceLastRegen;	// Liters
	uint16_t RegCnt;
	uint16_t UsedLastRegen;			// Liters
	uint16_t RegCntSoftening;
	uint16_t UsedLastRegenSoftening;// Liters
	uint8_t  Flags;					// WS_F_*
	uint8_t  RegenSofteningCntAlarm;// Alarm when zero
	uint8_t  UsedDrainSiltL100;		// How many L * 100 remain before draining silt.
	uint16_t FilterCounter1;		// * 100L
	uint16_t FilterCounter2;		// * 100L
	uint16_t RO_FilterCounter1;		// * 100L
	uint16_t RO_FilterCounter2;		// * 100L
} __attribute__((packed));

#define WS_F_StartRegen				0x01	// Запланирована регенерация обезжелезивателя вручную
#define WS_F_StartRegenSoft			0x02	// Запланирована регенерация умягчителя вручную
#define WS_F_RegenPreparing			0x04	// Идет подготовка к регенерации
#define WS_F_MASK					0x3F
#define WS_F_bNeedRegen				6		// Зарезервировано для вывода в веб
#define WS_F_bNeedRegenSoft			7		// Зарезервировано для вывода в веб
#define WS_AVERAGE_DAYS				100		// После этого начнется новый отсчет, предыдущее среднее значение будет взято как первое значение.

#define RTC_Work_WeekDay_MASK		0x07	// Active weekday (1-7)
#define RTC_Work_Regen_MASK			0x30	// 0 - not, bit - wait a regen hour
#define RTC_Work_Regen_F1			0x10	// Iron remover regen in process
#define RTC_Work_Regen_F2			0x20	// Softener regen in process

struct type_RTC_memory { // DS3231/DS3232 used alarm memory, starts from 0x07, max size 7 bytes
	volatile uint16_t UsedToday;	// 0. used daily until switch to new day, liters
	volatile uint16_t UsedRegen;	// 1.
	volatile uint8_t  Work;			// 2. NeedRegen + WeekDay
} __attribute__((packed));

// Critical errors
#define	ERRC_WaterBooster		0x01
#define	ERRC_Flooding			0x02
#define	ERRC_TankEmpty			0x04
#define	ERRC_WeightEmpty		0x08
#define	ERRC_SepticAlarm		0x10
#define	ERRC_WaterCounter		0x20
#define	ERRC_TankFillingLong	0x40
#define	ERRC_LongWaterConsuming	0x80
volatile uint32_t CriticalErrors = 0;	// Stop any work when these errors have occurred
int32_t  vPumpsNewErrorData = 0;
int8_t   vPumpsNewError = 0;
int8_t   Errors[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };// Active Errors array
uint32_t ErrorsTime[16];

volatile bool ADC_has_been_read = false;
int		 WaterBoosterStatus = 0; // 0 - все выключены, 1 - вкл твердотельное, 2 - вкл оба, -1 - выкл обычное
uint32_t WaterBoosterTimeout = 0;  // ms
uint32_t WaterBoosterCountP100 = 0;	// real pulses*100
uint8_t  WaterBoosterFlag = 0;		// 1 - второе и более включение
uint32_t FeedPumpWork = 0;						// ms

//bool 	 WaterBoosterError = false;
//volatile bool FloodingError = false;
//bool	 TankEmpty = false;
uint32_t FloodingTime = 0;
uint32_t SepticAlarmTime;
#ifdef CHECK_DRAIN_PUMP
#define  MODBUS_RELAY_CMD_OFF	-1	// need switch on
#define  MODBUS_RELAY_OFF		0
#define  MODBUS_RELAY_CMD_ON	1	// need switch off
#define  MODBUS_RELAY_ON		2
#ifdef MODBUS_DRAIN_PUMP_ON_PULSE
int8_t   DrainPumpRelayStatus = MODBUS_RELAY_OFF; // MODBUS_RELAY_*
#else
uint8_t  DrainPumpRelayStatus = MODBUS_RELAY_CMD_ON;
#endif
uint8_t  PumpReadCounter = 0;
uint32_t DrainPumpTimeLast = 0;	// time
uint16_t DrainPumpPower = 0; // W
uint8_t  DrainPumpErrCnt = 0;
uint8_t  DrainPumpRelayErrCnt = 0;
//uint32_t SepticPumpTimeLast = 0;
//uint16_t SepticPumpPower = 0; // W
#endif
#ifdef MODBUS_SEPTIC_HEAT_RELAY_ADDR
bool   SepticRelayStatus = false;		// 0 - off, 1 - on
uint8_t  SepticRelayErrCnt = 0;
#endif
uint16_t FillingTankTimer = 0;
int16_t  FillingTankLastLevel = 0;	// in 0.01%
uint8_t  TankCheckFlag = 0;			// 0 - проверка на герметичность, 1 - проверка на скорость заполнения, 2 - сброс
uint32_t TimeFeedPump = 0;			// ms
uint8_t  NeedSaveWorkStats = 0;
uint32_t TimerDrainingWater = 0;
uint32_t TimerDrainingWaterAfterRegen = 0;
int32_t  UsedDrainRest = 0;
volatile uint8_t NewRegenStatus = 0; // b1 - new?; b2 - backwash = yes(1) / no(0)
volatile uint32_t RegBackwashTimer = 0;	// sec
int16_t  RegStart_Weight;
uint16_t RegMaxFlow = 0;			// l*h
uint16_t RegMinPress = 0xFFFF;		// bar*100
uint32_t ResetDUE_countdown = 0;
bool	 DebugToJournalOn = false;
uint32_t FlowPulseCounter;	// real pulses * 100
bool     LowConsumeMode = false; //
int32_t  AfterFilledTimer = 0; // Время после отключения реле заполнения бака до останова глубинного насоса, сек
uint32_t Request_LowConsume;
uint32_t RegenStarted = 0;
uint8_t  Passed100Count = 0;	// L
uint8_t  DrainingSiltFlag = 0;	// 0 - нет/ожидание, 1 - идет слив, 2..255 - закрываемся и ожидаем (сек)
uint16_t DrainingSiltNowTimer = 0;
uint8_t  UsedWaterContinuousCntUsed = 0;
uint8_t  UsedWaterContinuousCntNot = 0;
uint16_t UsedWaterContinuousTimer = 0;	// /=USED_WATER_CONTINUOUS_MINTIME
uint16_t UsedWaterContinuousTimerMax = 0;
uint8_t  PIN_LED_SRV_INFO_off = 0;		// off value

int16_t  RWATERON_Switching = 0; // >0 - в процессе переключения, <0 - задержка включения, сек

// Weight
//bool Weight_NeedRead = false; // allways
int32_t  Weight_adc_filter[WEIGHT_AVERAGE_BUFFER]; 	// массив накопленных значений
int32_t  Weight_adc_sum;                          	// сумма
uint8_t  Weight_adc_idx;  			                // текущий индекс
int32_t  Weight_adc_median1, Weight_adc_median2;	// медианный фильтр

boolean  Weight_adc_flagFull;          			    // буфер полный
int32_t	 Weight_value = 0;							// десятые грамма
int16_t  Weight_Percent = 0;						// %, сотые
int32_t  Weight_Test = 100000;						// десятые грамма, Тест
uint8_t  Weight_Wrong_ADC_Cnt = 0;
void Weight_Clear_Averaging(void);
bool Weight_Read(bool skip_error = 0);

#define  bRTC_UsedToday		0
#define  bRTC_UsedRegen		1
#define  bRTC_Work			2
#define  bRTC_Urgently		7
#define  RTC_SaveAll		((1<<bRTC_UsedToday) | (1<<bRTC_UsedRegen) | (1<<bRTC_Work) | (1<<bRTC_Urgently))
volatile uint8_t NeedSaveRTC = 0;

uint16_t task_updstat_chars = 0;

struct type_DailySwitch {
	uint8_t Device;					// Реле; 0 - нет и конец массива
	uint8_t TimeOn;					// Время включения hh:m0
	uint8_t TimeOff;				// Время выключения hh:m0
} __attribute__((packed));

type_WebSecurity WebSec_user;				// хеш паролей
type_WebSecurity WebSec_admin;				// хеш паролей

// type_option.flags
#define fWebStoreOnSPIFlash 0				// флаг, что веб морда лежит на SPI Flash, иначе на SD карте
#define fBeep               1               // флаг Использование звука
#define fHistory            2               // флаг записи истории на карту памяти
#define f1Wire1TSngl		3				// На основной (1-ой) шине 1-Wire только один датчик
#define f1Wire2TSngl		4				// На 2-ой шине 1-Wire(DS2482) только один датчик
#define f1Wire3TSngl		5				// На 3-ей шине 1-Wire(DS2482) только один датчик
#define f1Wire4TSngl		6				// На 4-ей шине 1-Wire(DS2482) только один датчик
#define fLogWirelessSensors 7				// Логировать обмен между беспроводными датчиками
#define fPWMLogErrors  		8               // флаг писать в лог ошибки электросчетчика
#define fDontRegenOnWeekend	9				// Не делать регенерацию в выходные
#define fDebugToJournal		10				// Расширенная отладка в журнал
#define fDebugToSerial		11				// Расширенная отладка в Serial
#define fRegenAllowed		12				// Разрешена регенерация обезжелезивателя
#define fRegenAllowedSoftener 13			// Разрешена регенерация умягчителя
#define fFlowIncByPressure	14				// Добавка к потреблению при низком расходе
#define fLowConsumeReq_OnByErr 15			// Если нет ответа на LowConsumeRequest HTTP запрос, то считать, что работаем от резерва
// type_option.flags2
#define fDrainSiltTank 		0				// Сливать осадок с бака периодически
#define fDrainSiltTankBeforeRegen 1			// Сливать осадок с бака перед регенерацией
#define fLED_SRV_INFO_PlanReg 2				// Мигать редко на PIN_LED_SRV_INFO при запланированной регенерации
#define fCheckDrainPump		3				// Проверять работу дренажного насоса
#define fDrainPumpRelay		4				// Использовать реле отключения насоса
#define fSepticHeatRelay	5				// Использовать реле нагрева септика

// Структура для хранения настроек
struct type_option {
	uint8_t  ver;					// номер версии для сохранения
	uint8_t  RegenHour;				// Час регенерации (0..23) по маске 0x1F, + количество часов (1..7) по маске 0xE0.
	uint8_t  FilterTank;			// Диаметр фильтра обезжелезивателя в дюймах
	uint8_t  FilterTankSoftener;	// Диаметр фильтра умягчителя в дюймах
	uint16_t flags;					// Флаги опций до 16 флагов
	uint16_t tChart;				// период графиков в секундах!!
	uint32_t FeedPumpMaxFlow;		// тысячные литров в cек, Расход для постоянного включения дозатора (= FeedRate_ml_sec / Need_ml_sec * 1000)
	int32_t  WeightScale;			// Коэффициент калибровки весов, десятитысячные (~50.0)
	int32_t  WeightZero;			// Вес 0, АЦП
	int32_t  WeightTare;			// Вес тары, десятые грамма
	int32_t  WeightFull;			// Полный вес реагента без тары, граммы
	uint32_t DrainAfterNoConsume;	// Через сколько секунд сливать воду при отсутствии потребления
	uint32_t BackWashFeedPumpMaxFlow; // тысячные литров в cек, Расход для постоянного включения дозатора (= FeedRate_ml_sec / Need_ml_sec * 1000)
	uint16_t BackWashFeedPumpDelay; // задержка включения дозатора, сек
	uint16_t DaysBeforeRegen;		// Дней до регенерации обезжелезивателя, 0 - не проверять
	uint16_t UsedBeforeRegen;		// Количество литров до регенерации обезжелезивателя, 0 - нет
	uint16_t UsedBeforeRegenSoftening;// Количество литров до регенерации умягчителя, 0 - нет
	uint16_t MinWaterBoostOnTime;	// Минимальное время работы насосной станции, ms
	uint16_t MinWaterBoostOffTime;	// Минимальное перерыва работы насосной станции, ms
	uint16_t MinPumpOnTime;			// мсек, Минимальное время работы дозатора
	uint16_t MinRegenLiters;		// Тревога, если за регенерацию израсходовано меньше литров
	uint16_t MinDrainLiters;		// Тревога, если слито (Drain) при сбросе меньше литров*10
	uint16_t PWM_DryRun;			// Вт, Мощность сухого хода, если ниже во время работы - то стоп
	uint16_t PWM_Max;				// Вт, Максимальная мощность, если больше во время работы - то стоп
	uint16_t PWM_StartingTime;		// мсек, Время пуска
	uint16_t FloodingDebounceTime;	// сек, Время исключения помех срабатывания датчика протечки
	uint16_t FloodingTimeout;		// сек, Время ожидания перед началом работы после срабатывания датчика протечки
	int16_t  PWATER_RegMin;			// сотые бара, Нижний предел давления PWATER при регенерации
	int16_t  LTANK_Low;				// сотые %, Низкий уровень воды в баке - нужно включить заполнение бака до максимального
	uint16_t DrainTime;				// Время слива воды, сек
	uint16_t FillingTankTimeout;	// сек, Время заполнения бака на TankCheckPercent при отсутствии потребления
	int16_t  Weight_Low;			// сотые %, Низкий уровень реагента, для тревоги
	uint16_t CriticalErrorsTimeout;	// сек, время восстановления после критических ошибок, кроме протечки
	uint8_t  DrainingWaterAfterRegen;// сек, Слив после промывки обезжелезивателя
	uint8_t  DrainingWaterAfterRegenSoftening;// сек, Слив после промывки умягчителя
	uint16_t DaysBeforeRegenSoftening;// Дней до регенерации умягчителя, 0 - не проверять
	uint16_t LTank_LowConsumeMin;	// Низкий уровень бака при низком потреблении ( от резервного источника), сотые %
	uint16_t LTank_AfterFilledTimer;// Время после отключения реле заполнения бака до останова глубинного насоса, сек
	char     LowConsumeRequest[64];	// HTTP GET запрос о режиме низкого потребления, формат server/request, возврат после '=': 0 - нет, 1 - да
	uint16_t LowConsumeRequestPeriod;// Периодичность запроса о режиме низкого потребления, если 0, то только при старте, сек
	uint16_t SepticAlarmDebounce;	// Время исключения помех датчика аварии септика, сек
	uint16_t MinRegenLitersSoftening;// Тревога, если за регенерацию умягчителя израсходовано меньше литров
	uint16_t MinRegenWeightDecrease;// Тревога, если за регенерацию обезжелезивателя вес уменьшился меньше, чем задано, граммы
	uint16_t LTank_Hour_Low;		// Низкий уровень ночного заполнения бака, сотые %
	int8_t   LTank_Hour;			// Час ночного заполнение бака
	uint8_t  RegenSofteningCntAlarm;// Счетчик регенераций до включения тревоги, 0 - нет
	uint16_t MinRegenWeightDecreaseSoftening;// Тревога, если за регенерацию умягчителя вес уменьшился меньше, чем задано, граммы
	uint16_t MinWaterBoosterCountL; // Тревога, если между включениями насосной станции потреблено меньше литров, сотые
	int16_t  PWATER_Osmos_Min;		// Минимальное давление для осмоса, если низкий расход, то насосная станция включается при этом значении (0 - выкл), сотые бара
	uint8_t  PWATER_Osmos_Step;		// Осмос - Через сколько раз уменьшений давления без протока включать насосную станцию
	uint8_t  RFILL_HoursRepeatPulse;// Периодичность для импульса (1 сек) реле RFILL, часы
	uint32_t FeedPumpRate;			// Производительность дозатора, тысячные милилитров в сек
	type_DailySwitch DailySwitch[DAILY_SWITCH_MAX];	// дневное периодическое включение
	uint16_t FlowIncByPress_MinFlow;// Минимальный проток меньше которого начинается добавка, литры в час
	uint16_t flags2;				// Флаги опций #2 до 16 флагов
	uint8_t  DrainSiltTime;			// Время слива осадка с бака, секунды * 10
	uint8_t  DrainSiltAfterL100;	// Через сколько литров сливать осадок, литров * 100
	uint8_t  DrainSiltAfterNotUsed; // Сливать осадок, после отсутствия потребления в течении, часов. Если не получается, то слив будет после DrainSiltL100 * 50%
	uint8_t  TankCheckPercent;		// Проверка на утечку/заполняемость бака, тревога, если уровень уменьшится без потребителей на %, либо заполнение бака идет медленно (FillingTankTimeout)
	uint8_t  TankFillingTimeMax;	// Максимальное время заполнения бака за один раз, минуты
	uint8_t  UsedWaterContinuous;	// Максимальное время непрерывного потребления воды, минуты
	uint16_t FilterCounter1_Max;	// Предел для счетчика 1, *100л
	uint16_t FilterCounter2_Max;	// Предел для счетчика 2, *100л
	uint8_t  DrainPumpMaxTime;		// Максимальное время работы дренажного насоса, 0 - нет, сек * 30
	uint8_t  DrainPumpMinPower;		// Минимальная мощность дренажного насоса для определения его работы, W * 10
	uint8_t  PWATER_Osmos_TankMul;	// Множитель объема расчетного бака, сотые
	uint8_t  PWATER_Osmos_FullDelay;// Задержка начала контроля малого потребления после наполнения бака НС, сек
	uint8_t  PWATER_Osmos_FullMinus;// Дельта от макс. давления бака НС, контроль малого потребления после снижения давления ниже, сотые бара
	uint8_t  WaterBoosterMinTank;	// Контроль среднего минимального объема бака насосной станции, 0 - нет, литры
	uint16_t DrainPumpMaxPower;		// Максимальная мощность дренажного насоса после времени старта, Вт
	uint8_t  DrainPumpStartTime;	// Время старта дренажного насоса, с
	uint8_t  PWATER_Osmos_Delay;	// Задержка после прекращения потребления (ниже минимума) до начала работы алгоритма добавки, сек
	uint16_t RO_FilterCounter1_Max;	// Предел для фильтров обратного осмоса #1, *100л
	uint16_t RO_FilterCounter2_Max;	// Предел для фильтров обратного осмоса #2, *100л
};

//  Работа с отдельными флагами type_DateTime
#define fUpdateNTP     0                // флаг Обновление часов по NTP при старте
#define fUpdateI2C     1                // флаг Обновление часов раз в час с I2C  часами
#define fUpdateByHTTP  2                // флаг Обновление по HTTP - спец страница: define HTTP_TIME_REQUEST

// Структура для хранения настроек времени, для удобного сохранения.
struct type_DateTime
{
    uint8_t flags;                        //  Флаги опций до 8 флагов
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
	void clear_error();                              // стереть последнюю ошибку
	void clear_all_errors(void);					// стереть все ошибки
	__attribute__((always_inline)) inline int8_t get_errcode(){return error;} // Получить код последней ошибки
	char  *get_lastErr(){return note_error;} // Получить описание последней ошибки, которая вызвала останов, при удачном запуске обнуляется
	void   scan_OneWire(char *result_str); // Сканирование шины OneWire на предмет датчиков
    inline TEST_MODE get_testMode(){return testMode;} // Получить текущий режим работы
    void   set_testMode(TEST_MODE t);    // Установить значение текущий режим работы
    void   StateToStr(char * ret);                 // Получить состояние в виде строки
    char  *TestToStr();                  // Получить режим тестирования
    inline bool get_NeedRegen(void) {
		return GETBIT(Option.flags, fRegenAllowed) && ((Option.DaysBeforeRegen && WorkStats.DaysFromLastRegen >= Option.DaysBeforeRegen)
				|| (Option.UsedBeforeRegen && WorkStats.UsedSinceLastRegen + RTC_store.UsedToday >= Option.UsedBeforeRegen));
    }
    inline bool get_RegenExpired(void) {
		return ((Option.DaysBeforeRegen && WorkStats.DaysFromLastRegen > Option.DaysBeforeRegen)
				|| (Option.UsedBeforeRegen && WorkStats.UsedSinceLastRegen + RTC_store.UsedToday > Option.UsedBeforeRegen));
    }
    inline bool get_NeedRegenSoftening(void) {
    	return GETBIT(Option.flags, fRegenAllowedSoftener) && ((Option.DaysBeforeRegenSoftening && WorkStats.DaysFromLastRegenSoftening >= Option.DaysBeforeRegenSoftening)
    			|| (Option.UsedBeforeRegenSoftening && WorkStats.UsedSinceLastRegenSoftening + RTC_store.UsedToday >= Option.UsedBeforeRegenSoftening));
    }
    inline bool get_RegenExpiredSoftening(void) {
    	return ((Option.DaysBeforeRegenSoftening && WorkStats.DaysFromLastRegenSoftening > Option.DaysBeforeRegenSoftening)
    			|| (Option.UsedBeforeRegenSoftening && WorkStats.UsedSinceLastRegenSoftening + RTC_store.UsedToday > Option.UsedBeforeRegenSoftening));
    }

	uint32_t get_errorReadTemp();       // Получить число ошибок чтения датчиков температуры
	void     Reset_TempErrors();		// Сбросить счетчик ошибок всех датчиков
	uint32_t CalcFilterSquare(uint8_t diameter);	// d - inch, ret = m2 * 10000
	uint32_t CalcFilteringSpeed(uint32_t square);	// square = m2 * 10000, ret = m*h * 1000

	int32_t save(void); 		        // Записать настройки в eeprom i2c на входе адрес с какого, на выходе код ошибки (меньше нуля) или количество записанных  байт
	int32_t load(uint8_t *buffer, uint8_t from_RAM); // Считать настройки из i2c или RAM, на выходе код ошибки (меньше нуля)
	int8_t loadFromBuf(int32_t adr,byte *buf);// Считать настройки из буфера на входе адрес с какого, на выходе код ошибки (меньше нуля)
	int8_t save_WorkStats();             // запись счетчиков в ЕЕПРОМ
	int8_t load_WorkStats();             // чтение счетчиков в ЕЕПРОМ

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

	// Дата время
	boolean set_datetime(char *var, char *c);              //  Установить параметр дата и время из строки
	void    get_datetime(char *var,char *ret);             //  Получить параметр дата и время из строки
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
	inline  char* get_passUser() { return Network.passUser; }
	inline  char* get_passAdmin() { return Network.passAdmin; }

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
	inline uint8_t  get_Beep() {return GETBIT(Option.flags,fBeep);};           // подача звуковых сигналов
	uint8_t  get_WebStoreOnSPIFlash() {return GETBIT(Option.flags,fWebStoreOnSPIFlash);}// получить флаг хранения веб морды на флеш диске
	uint16_t get_flags() { return Option.flags; }					  // Все флаги

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

	void resetCount();                          		// Сборос счетчиков
	void updateCount();                                     // Обновление счетчиков

	void set_uptime(unsigned long ttime){timeON=ttime;}     // Установить текущее время как начало старта контроллера

    uint8_t  get_fSD() { return fSD;}        				// Получить флаг наличия РАБОТАЮЩЕЙ СД карты
    void     set_fSD(uint8_t f) { fSD=f; }   				// Установить флаг наличия РАБОТАЮЩЕЙ СД карты
    uint8_t  get_fSPIFlash() { return fSPIFlash;}   		// Получить флаг наличия РАБОТАЮЩЕГО флеш диска
    void     set_fSPIFlash(uint8_t f) {fSPIFlash=f;}    	// Установить флаг наличия РАБОТАЮЩЕГО флеш диска
    TYPE_SOURSE_WEB get_SourceWeb();                    	// Получить источник загрузки веб морды
    uint8_t CalcNextRegenAfterDays(int _DaysBeforeRegen, int _DaysFromLastRegen, int _UsedBeforeRegen, int _UsedSinceLastRegen);// Рассчитать сколько осталось дней до регенерации

	// Переменные
	uint8_t CPU_LOAD;                                      // загрузка CPU
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
	void updateChart();                                    // обновить графики
	void clearChart();                                     // Запустить графики
	char * get_listChart(char* str);				       // получить список доступных графиков
	void get_Chart(char *var,char* str);   				   // получить данные графика

	// графики не по датчикам (по датчикам она хранится внутри датчика)
	statChart ChartWaterBoost;
	statChart ChartFeedPump;
	statChart ChartFillTank;
	statChart ChartBrineWeight;
	statChart ChartWaterBoosterCount;

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

	uint8_t  NO_Power;					  			// Нет питания основных узлов
	uint16_t NO_Power_delay;
	uint8_t  fNetworkReset;							// Нужно сбросить сеть
    TEST_MODE testMode;                             // Значение режима тестирования
	type_WorkStats WorkStats;               		// Структура для хранения счетчиков периодическая запись
	type_WorkStats WorkStats_saved;
	type_RTC_memory RTC_store;
	uint32_t FilterTankSquare;						// m2 * 10000
	uint32_t FilterTankSoftenerSquare;				// m2 * 10000
	uint8_t  Osmos_PWATER_Cnt;
	int16_t  Osmos_PWATER_Last;
	int16_t  Osmos_PWATER_BoosterMax;				// сотые литра
	int16_t  Osmos_PWATER_BoosterMax_Calc;			// для расчета Osmos_PWATER_BoosterMax
	uint8_t  Osmos_PWATER_BoosterMax_cnt;
	int16_t  Osmos_PWATER_LastPress;				// последнее давление, от которого идет отсчет
	uint8_t  Osmos_PWATER_LastPress_Timer;
	uint8_t  Osmos_PWATER_Added;					// Во время текущего расходования бака насосной станции была добавка для Осмоса, 2 = только что
	uint8_t  Osmos_PWATER_DelayCnt;					// Счетчик задержки после прекращения потребления (ниже минимума) до начала работы алгоритма добавки
	uint32_t RFILL_last_time_ON;					// время последнего включения реле RFILL, если 0, то RFILL -> OFF
	uint8_t  NextRegenAfterDays;
	uint8_t  NextRegenSoftAfterDays;

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
