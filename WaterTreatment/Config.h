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

#ifndef Config_h
#define Config_h
#include "Ethernet.h"

// --------------------------------------------------------------------------------
// КОНФИГУРАЦИИ ----------------------------------------------------------------
// --------------------------------------------------------------------------------
#define CONFIG_1             // vad7

#define ERR_TANK_EMPTY		-58

struct CORRECT_POWER220_STRUCT {
	uint8_t  num;	// номер реле
	int16_t  value; // Вт
};

enum {
	STATS_TYPE_MIN = 0,
	STATS_TYPE_AVG,
	STATS_TYPE_MAX,
	STATS_TYPE_SUM,
	STATS_TYPE_TIME // Time, ms
};

struct Stats_Data {
	int32_t		value;			// Для среднего, макс единица: +-1491308
	uint8_t		object;			// STATS_OBJ_*
	uint8_t		type;			// STATS_TYPE_*
};

enum {
	STATS_OBJ_Temp = 0,		// °C, TAIR
	STATS_OBJ_Press,		// bar
	STATS_OBJ_Flow,			// м³ч
	STATS_OBJ_Voltage,		// V
	STATS_OBJ_Power,		// кВт*ч
	STATS_OBJ_WaterUsed,	// м³
	STATS_OBJ_WaterRegen,	// м³
	STATS_OBJ_BrineWeight,	// кг
	STATS_OBJ_WaterBooster,	// сек
	STATS_OBJ_FeedPump,		// сек
	STATS_OBJ_Level			// %
};
struct History_setup {
	uint8_t		object;			// STATS_OBJ_*
	uint8_t 	number;			// номер датчика
	const char *name;			// Заголовок
};
// Конец определений ==================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------
// =============================================== C O N F I G   1 ===================================================================
// -----------------------------------------------------------------------------------------------------------------------------------
//  Arduino DUE Core
#ifdef CONFIG_1    // Имя и описание конфигурации и ОСОБЕННОСТИ конфигурации -------------------------------
	#define TEST_BOARD 				// Тестовая плата!

    #define CONFIG_NAME   "vad7"
    #define CONFIG_NOTE   "Обезжелезивание Quantum DMI-65, дозирование хлора, водоснабжение"
    // Колонна 1354, голова Autotrol 263 + 742 (засыпка 70л Quantum DMI-65)
	// Дозатор гипохлорита натрия Stenner 45MHP10
    // Насосная станция Wilo HWJ 204-EM-50 (50л)
    #define UART_SPEED    250000	// Скорость отладочного порта
	#define KEY_ON_OFF				// + KEY1 Наличие кнопки включения и переключения в safeNetwork (нажата при сбросе)
    //#define SPI_FLASH				// + Наличие чипа флеш памяти на шине SPI
    //#define ONEWIRE_DS2482		// + Использование мастера i2c Onewire DS2482 (адрес AD1,0 = 0,0)
    //#define ONEWIRE_DS2482_SECOND	// второй мастер i2 Onewire DS2482 (адрес AD1,0 = 0,1)
	//#define ONEWIRE_DS2482_THIRD	// третий мастер i2 Onewire DS2482 (адрес AD1,0 = 1,0)
	//#define ONEWIRE_DS2482_FOURTH	// четвертый мастер i2 Onewire DS2482 (адрес AD1,0 = 1,1)
    //#define ONEWIRE_DS2482_2WAY  	// Используются 2-х проводные шины OneWire (паразитное питание)
	#ifdef ONEWIRE_DS2482_2WAY
      const uint8_t ONEWIRE_2WAY = 0b1010; // На каких шинах (4|3|2|1) двух-проводные датчики, битовая маска
	#else
      const uint8_t ONEWIRE_2WAY = 0b0000;
	#endif
	#define PIN_ONE_WIRE_BUS 41     // ULN5. нога с интерфейсом программный 1-Wire (температурные датчики)
    #define ONEWIRE_DONT_CHG_RES    // Не записывать 9/12-битное разрешение в датчик при привязке, а просто устанавливать
    #define LOAD_VERIFICATION     	// Признак чтения настроек c проверкой версии, длины, CRC16. Закоментируйте эту строку для ПОПЫТКИ загрузить старый формат, Запись всегда идет в новом
    //#define EXTERNAL_AREF     	  	// Использование внешней опоры для АЦП
    #ifdef EXTERNAL_AREF        	// Какая опора для АЦП используется
      #define SAM3X_ADC_REF  3.00   // Используется внешняя опора ADR4530ARZ
    #else
      #define SAM3X_ADC_REF  3.30   // Штатное подключение используется питание DUE
    #endif
    //#define USE_UPS					// Используется ИБП на контроллер, проверка через вход SPOWER
	#define STATS_USE_BUFFER_FOR_SAVING // Сохранять статистику только когда буфер (512 байт) заполнен, иначе каждый день

	//#define RADIO_SENSORS			// Радиодатчики через ZONT МЛ‑489.
    // Подключение через плату UART - K-line или через 4 платы UART-RS485 отдельно на RX и TX, на стороне МЛ-489 : питание 5V, через диод K-line >--TX--|>|--RX
    // Новый датчик ждем в течении 2-х минут после старта НК (нажать кнопку датчика пока не загорится его светодиод)
	//#define RADIO_SENSORS_PORT	2				// Номер Serial
	//#define RADIO_SENSORS_PSPEED	115200			// Скорость порта
	//#define RADIO_SENSORS_PCONFIG	SERIAL_8N1		// Конфигурация порта
	//#define USE_SERIAL4							// Использовать порт Serial4 на D52(RXD2) и A11/D62(TXD2)

	#define FORMAT_DATE_STR_CUSTOM
    const char *FORMAT_DATE_STR	 = { "%02d.%02d.%04d" };
	//#define HTTP_TIME_REQUEST	// Запрос времени по HTTP протоколу с собственного сервера (текстовый файл формата: "<время UTC>;").
    							// У меня берется с электросчетчика Меркурий 231 АТ через подключенный к нему esp8266 (http://vad-7.blogspot.com/2017/03/mercury231.html)
	#ifdef HTTP_TIME_REQUEST
    const char HTTP_TIME_REQ[]	= "/curr_time.csv";
	#endif
	#define INDEX_FILE			"plan.html"       // стартовый файл по умолчанию для веба

	#ifdef TEST_BOARD
		#define DEBUG                   // В последовательный порт шлет сообщения в первую очередь ошибки
    	//#define DEBUG_NATIVE_USB		// Отладка через второй USB порт (Native)
		#define DEBUG_LEVEL		 2		// 0 - silence, 1 - more...
    	//#define I2C_JOURNAL_IN_RAM		// Журнал в ОЗУ
		#define I2C_FRAM_MEMORY  0		// 1 - FRAM память
	#else
		#define DEBUG                   // В последовательный порт шлет сообщения в первую очередь ошибки
    	//#define DEBUG_NATIVE_USB		// Отладка через второй USB порт (Native)
		#define DEBUG_LEVEL		 1		// 0 - silence, 1 - more...
		#define I2C_EEPROM_64KB	        // Размер I2C памяти (одна страница)
		#define I2C_FRAM_MEMORY  0		// 1 - FRAM память
		#define DONT_LOG_SUCCESS_PING	// Не логировать в журнал успешные пинги
	#endif
	#ifdef  I2C_EEPROM_64KB                    // AT24C512C. В зависимости от типа чипа.
		#define I2C_ADR_EEPROM    0x50         // Адрес чипа на шине I2C
		#define I2C_SIZE_EEPROM   512    	   // Объем чипа в килобитах
  		#define I2C_MEMORY_TOTAL  512   	   // Итоговый размер I2C памяти в килобитах
		#define I2C_PAGE_EEPROM   128           // Размер страницы для чтения, байты
	#else // все остальное
		#define I2C_ADR_EEPROM    0x57         // Адрес чипа на шине I2C
		#define I2C_SIZE_EEPROM   32	       // Объем чипа в килобитах
		#define I2C_MEMORY_TOTAL  I2C_SIZE_EEPROM // Итоговый размер I2C памяти в килобитах
		#define I2C_PAGE_EEPROM   32           // Размер страницы для чтения, байты
	#endif
	#define REBOOT_ON_I2C_ERRORS				// Soft RESET при постоянной ошибке I2C

	//#define MQTT                             // признак использования MQTT, при неиспользовании необходимо закоментировать

    // СЕТЕВЫЕ НАСТРОЙКИ --------------------------------------------------------------
	#ifndef  TEST_BOARD
		uint8_t SPI_RATE 			  = 2;	// делитель для SPI шины, 2=42MHz, 3=28MHz, 4=21MHz, 6=14MHz
		#define SD_CLOCK				28	// частота SPI для SD карты в МГц
		const boolean   defaultDHCP	=	false;
		const IPAddress defaultIP		(192, 168, 0,   199);
		const IPAddress defaultSDNS		(  8,   8, 8,   8);
		const IPAddress defaultGateway	(192, 168, 0,   1);
	#else
		uint8_t SPI_RATE 			  = 6;	// делитель для SPI шины, 2=42MHz, 3=28MHz, 4=21MHz, 6=14MHz
		#define SD_CLOCK				20	// частота SPI для SD карты в МГц
		const boolean   defaultDHCP	=	true;
		const IPAddress defaultIP		(192, 168, 0, 199);
		const IPAddress defaultSDNS		(192, 168, 0,   10);
		const IPAddress defaultGateway	(192, 168, 0,   10);
	#endif
	const IPAddress defaultSubnet      (255, 255, 255, 0);
     //  #define SUPERBOILER               // Использование предкондесатора для нагрева ГВС
     //  #define SUPERBOILER_FC (90*100)   // частота супербойлера для частотника
     //  #define SUPERBOILER_DT (10*100)   // разница температур компресссора и бойлера для включения насоса
    // --------------------------------------------------------------------------------
    // ЖЕЛЕЗО  - привязка к ногам контроллера  В зависимости от конкретной схемы и платы
    // Для каждой конфигурации теперь свои определения!!!
    // --------------------------------------------------------------------------------
    // Конфигурирование Modbus электро счетчика
	#define MODBUS_PORT_NUM         Serial2     // Аппаратный порт куда прицеплен Modbus
    #define MODBUS_PORT_SPEED       9600        // Скорость порта куда прицеплен частотник и счетчик
    #define MODBUS_PORT_CONFIG      SERIAL_8N1  // Конфигурация порта куда прицеплен частотник и счетчик
    #define MODBUS_TIME_WAIT        1000        // Время ожидания захвата мютекса для modbus мсек
    #define MODBUS_TIME_TRANSMISION 0           // Пауза (msec) между запросом и ответом по модбаc
    //#define PIN_MODBUS_RSE          22          // Не используется из-за платы UART-RS485! Управление направлением передачи 485 для связи с инвертором по Modbus (1-передача 0-прием)
	#define PWM_MODBUS_ADR			0xF8		// PZEM-004T V.3 Modbus

    // SPI шина управление отдельными устройствами до 3-х устройств (активный уровень низкий)
    #define PIN_SPI_CS_W5XXX	10		// ETH-CS   сигнал CS управление сетевым чипом w5500
    #define PIN_SPI_CS_SD		52		// SD-CS    сигнал CS управление SD картой (D52 пересекается с Serial4!)
    //#define PIN_SPI_CS_FLASH	4		// FLASH-CS сигнал CS управление чипом флеш памяти

    // Сервис
    #define PIN_ETH_RES			55          // ETH-RES Сброс сетевого чипа w5500 активный low нормально high
	#define PIN_LED_OK			42          // Зеленый светодиод Выход на светодиод мигает 0.5 герца - ОК  с частотой 2 герца ошибка
    #define PIN_LED_ERROR		13		    // Для библиотеки FreeRTOS
    #define PIN_KEY_SAFE		26          // Нажатие при включении - режим safeNetwork (настрока сети по умолчанию, не спрашивает пароль на вход в веб морду)
    #define PIN_BEEP			11          // SOUND Выход на пищалку  88- нога не использующиеся
	#define PIN_KEY_UP			32			// KEYS.2
	#define PIN_KEY_DOWN		30			// KEYS.3
	#define PIN_KEY_OK			12			// KEYS.4

	#define TANK_ANALOG_LEVEL				// Использовать аналоговый датчик уровня в баке, MINPRESS = уровень сухого бака, MAXPRESS = уровень перелива.

	// Весы
	#define HX711_DOUT_PIN		39			// ULN6
	#define HX711_SCK_PIN		40			// ULN7
	#define WEIGHT_AVERAGE_BUFFER 10

    // Контактные датчики (sInput[]------------------------------------------------------------------
#ifndef TANK_ANALOG_LEVEL
    #define INUMBER             7   	// Число контактных датчиков цифровые входы
#else
	#define INUMBER             5   	// Число контактных датчиков цифровые входы
#endif
    // Имена индексов
	#define REG_ACTIVE			0        // Активна регенерация (INP2)
	#define REG_BACKWASH_ACTIVE 1        // Активна обратная промывка (INP3)
	#define REG2_ACTIVE			2        // Активна регенерация умягчителя (INP4)
	#define FLOODING			3        // Протечка (INP5)
	#define TANK_EMPTY			4        // Емкость пуста (INP6)
#ifndef TANK_ANALOG_LEVEL
    #define TANK_LOW			5        // Нужен долив бака 500л (DAC0 [D66])
	#define TANK_FULL			6        // Емкость полна (REL8 [D5])
#endif

	// Массив ног
	const uint8_t pinsInput[INUMBER] = { 56, 43, 54, 23, 24
#ifndef TANK_ANALOG_LEVEL
			, 66, 5
#endif
		};
      // Описание датчиков
    const char *noteInput[INUMBER] = {	"Идет регенерация",
    		  	  	  	  	  	  	  	"Идет обратная промывка",
										"Идет регенерация умягчителя",
										"Затопление!",
										"Пустой бак!"
#ifndef TANK_ANALOG_LEVEL

										,"Долив бака",
										"Бак полный"
#endif
                                     };
      // Имена датчиков
    const char *nameInput[INUMBER] = {	"REG",
										"REGBW",
										"REG2",
										"FLOODING",
										"EMPTY"
#ifndef TANK_ANALOG_LEVEL
										,"LOW",
										"FULL"
#endif
                                     };
     
#ifndef TANK_ANALOG_LEVEL
    const bool TESTINPUT[INUMBER]        = { 0, 0, 0, 0, 0, 0, 0 };    // Значения датчиков при тестировании  опция TEST
    const bool LEVELINPUT[INUMBER]       = { 0, 0, 0, 1, 0, 0, 0 };    // Значение датчика, когда сработал
    const bool PULLUPINPUT[INUMBER]      = { 0, 0, 0, 0, 0, 1, 1 };    // если 1 - то на порту выставляется подтяжка к VCC.
    const int8_t SENSOR_ERROR[INUMBER]   = { 0, 0, 0, 0, ERR_TANK_EMPTY, 0, 0 };  // При срабатывании генерить ошибку с заданным кодом, если не 0
#else
      const bool TESTINPUT[INUMBER]        = { 0, 0, 0, 0, 0 };    // Значения датчиков при тестировании  опция TEST
      const bool LEVELINPUT[INUMBER]       = { 0, 0, 0, 1, 0 };    // Значение датчика, когда сработал
      const bool PULLUPINPUT[INUMBER]      = { 0, 0, 0, 0, 0 };    // если 1 - то на порту выставляется подтяжка к VCC.
      const int8_t SENSOR_ERROR[INUMBER]   = { 0, 0, 0, 0, ERR_TANK_EMPTY };  // При срабатывании генерить ошибку с заданным кодом, если не 0
#endif
    // ---------------------------------------------------------------------------------------------------------------------------------------
    // Частотные датчики ------------------------------------------------------------------
    #define FNUMBER             1       // Число частотных датчиков цифровые входы
    // SIKA VVX25: +12V - коричневый (Xn.2), GND - голубой (Xn.3), выход - черный (Xn.1)
    // Имена индексов
    #define FLOW                0        // Датчик протока (INP1)
    // Массив ног соглано индексов
    const uint8_t pinsFrequency[FNUMBER] = { 57 };
    // Описание датчиков
    const char *noteFrequency[FNUMBER] = {	"Датчик протока"
                                       	 };
    // Имена датчиков
    const char *nameFrequency[FNUMBER] = {  "FLOW",
                                         };

    const uint16_t TRANSFLOW[FNUMBER]= { 40000 };	// Коэффициент преобразования импульсов за литр, сотые
    const uint16_t TESTFLOW[FNUMBER] = { 0 };		// Значения датчиков при тестировании  опция TEST

   // Исполнительные устройства (реле и сухие контакты) ------------------------------------------------------------------
    #define RNUMBER                    8        // Число исполнительных устройств (всех)
	#define RELAY_INVERT			   // Реле выходов: включение высоким уровнем (High Level trigger)

    // Имена индексов
    // устройства AC 220V
	#define RBOOSTER1				0	// Реле насосной станции (твердотельное)
	#define RBOOSTER2				1	// Реле насосной станции (контактное реле)
	#define RFEEDPUMP          		2	// Реле включения дозирующего насоса
	#define RWATEROFF				3	// Реле перекрывающего крана после фильтра 1
	#define RDRAIN					4	// Реле слива
	#define RFILL					5	// Реле заполнения бака
    // устройства DC 5..24V
	#define RSTARTREG				6	// Реле старта регенерации обезжелезивателя
	#define RSTARTREG2				7	// Реле старта регенерации умягчителя

	//#define RELAY_WAIT_SWITCH		10	// Заморозить выполнение задач на это время после переключения реле, ms

   	//#define CORRECT_POWER220				// Корректировка потребляемой мощности из электросети, если включены указанные реле, Вт
	#ifdef CORRECT_POWER220
    	CORRECT_POWER220_STRUCT correct_power220[] = { {R, 25} }; //
	#endif

    // Массив ног соглаcно индексов
	const uint8_t pinsRelay[RNUMBER] = {	47, // REL1
								   	   		3,  // RELR1
											46,	// REL2
											45,	// REL3
											44,	// REL4
											92, // REL5 // PC20 (Arduino DUE Core only!)
											9,	// REL6, DC_OUT7(L)
											8	// REL7, DC_OUT8(L)
                                       };
	// Описание реле
	const char *noteRelay[RNUMBER] = {	"Реле насосной станции",
                                     	"Реле насосной станции 2",
										"Реле включения дозирующего насоса",
										"Реле отключения воды",
										"Реле слива",
										"Реле заполнения бака",
										"Реле старта рег. железа",
										"Реле старта рег. умягчителя",
                                   };
	//  Имя реле
	const char *nameRelay[RNUMBER] = {	"RBST1",
                                     	"RBST2",
										"RFEEDP",
										"RWOFF",
										"RDRAIN",
										"RFILL",
										"RSTREG",
										"RSTREG2",
                                     };

	// ДАТЧИКИ ТЕМПЕРАТУРЫ. СОТЫЕ ГРАДУСА ------------------------------------------------------------------------
	// наличие датчиков  датчиков от конфигурации минимальные максимальные и тестовые температуры
	// Температура хранится в сотых градуса
	// --------------------------------------------------------------------------------
	#define TNUMBER     1    // Число температурных датчиков
	//#define RADIO_SENSORS_MAX 0
	#define TNTC        9	// Количество датчиков NTC на DUE
	////#define TNTC_EXT    4	// Количество датчиков NTC на внешнем АЦП

	#define TAIR        0    // Температура воздуха
    // Наличие датчика в конфигурации: 0 - нет, >0 - есть, +2(бит_1) - выводить датчик отдельно внизу на странице "схема" (если бит_0 = 0 - то только когда привязан)
    //                                                     +4(бит_2) - не строить график в ОЗУ
    //...................................0.....1.....2.....3.....4.....5.....6.....7.....8.....9....10....11....12....13....14....15....16....17....18....19....20....21....22....23....
    const uint8_t SENSORTEMP[TNUMBER]={    1 };
    // минимальные значения температур                                                                                                                                                
    const int16_t MINTEMP[TNUMBER] = {   100 };
    // Макимальные значения температур, ВНИМАНИЕ! для TBOILER температура меняется если включен режим сальмонеллы на SALLMONELA_TEMP+300                                              
    const int16_t MAXTEMP[TNUMBER] = {  3000 };
    // Значения датчиков при тестировании, опция TEST                                                                                                                                 
    const int16_t TESTTEMP[TNUMBER]= {  1200 };
    // Ошибки датчиков (систематические) нужны для калибровки ОШИБКИ ДОБАВЛЯЮТСЯ!!! к значениям В СОТЫХ ГРАДУСА                                                                       
    const int16_t ERRTEMP[TNUMBER]=  {     0 };

    // NTC
	#define TEMP_TABLE_START	-5500
	#define TEMP_TABLE_STEP		500
	#ifdef TNTC
    // NTC, 10K, B3435, Таблица сопротивлений через 5°, (-55..125°), резистор 15k
    const uint16_t NTC_table[] = { 4008, 3972, 3924, 3863, 3784, 3688, 3570, 3432, 3272, 3094, 2898, 2690, 2475, 2257, 2042, 1835, 1638, 1456, 1288, 1136, 1000, 879, 773, 679, 597, 525, 463, 409, 361, 320, 284, 253, 226, 202, 181, 162, 146 };
    const int8_t TADC[TNTC]    = { 7, 6, 5, 4, 3, 2, 1, 0, 10 };
	#endif
	#ifdef TNTC_EXT
    const int8_t TADC_EXT[TNTC]= { 0, 1, 2, 3 };
	#endif

    // Имена датчиков
    const char *nameTemp[TNUMBER] = {
    		"AIR"	             // 1. Температура воздуха
    	};
    // Описание датчиков
    const char *noteTemp[TNUMBER] = {
    		"Температура воздуха"
    	};

	#define TIME_SLICE_PUMPS  20
	#define TIME_READ_SENSOR  1000        // Период опроса датчиков (мсек)
	#define T_NUMSAMLES       1           // Число значений для усреднения показаний температуры
	#define GAP_TEMP_VAL      500         // Допустимая разница (в сотых C) показаний между двумя считываниями (борьба с помехами) - при привышении ошибка не возникает, но данные пропускаются.
	#define GAP_TEMP_VAL_CRC  200     	  // Датчики с флагом игнорировать CRC. Допустимая разница (в сотых C) показаний между двумя считываниями (борьба с помехами) - при привышении ошибка не возникает, но данные пропускаются.
	#define GAP_NUMBER        3       	  // Максимальное число идущих подряд показаний превышающих на GAP_TEMP_VAL, после этого эти показания выдаются за действительные
	#define GAP_NUMBER_CRC    7       	  // Датчики с флагом игнорировать CRC. Максимальное число идущих подряд показаний превышающих на GAP_TEMP_VAL, после этого эти показания выдаются за действительные

    // АНАЛОГОВЫЕ ДАТЧИКИ  -------------------------------------------------------------------
    // Давление харится в сотых единиц
	#define ANUMBER			2       // Число аналоговых датчиков
	#define PWATER			0       // Датчик давления воды, в сотых бара
	#define LTANK			1       // Датчик уровня воды в баке, в сотых %
	// Имена датчиков
	const char *namePress[] = { "PWATER",
								"LTANK"
							  };
	// Описание датчиков
	const char *notePress[] = { "Датчик давления воды",
								"Датчик уровня бака"
							  };

	// Номера каналов АЦП, в нумерации SAM3X (AD*):
	const uint8_t pinsAnalog[ANUMBER] = {	10, // A8(D62), INA1 - желтый, красный "+5V", черный "-".
											 1	// A6(D60), INA4-20_1.2 - (+12V)-красный, INA4-20_1.1 - (-12V)-черный.
										};
	// Коэффициент преобразования отсчеты АЦП-давление, тысячные
	const uint16_t TRANsADC[ANUMBER]  = { 181, 2442 };
	// напряжение (отсчеты АЦП) соответсвующее cZero
	const uint16_t ZEROPRESS[ANUMBER] = { 70, 0 };

	const boolean SENSORPRESS[ANUMBER]= { true, true };		// Присутствие датчика в конфигурации
	const int16_t MINPRESS[ANUMBER]   = {  250, 500 };		// минимальные значения давления, в сотых
	const uint16_t MAXPRESS[ANUMBER]  = {  370, 10000 };	// Максимальные значения давления, в сотых
	const uint16_t TESTPRESS[ANUMBER] = {  300, 8000 };		// Значения датчиков при тестировании  опция TEST, в сотых

	//#define ANALOG_MODBUS 								// Данные аналоговых датчиков читаются по Modbus RTU
	#ifdef ANALOG_MODBUS
	  #define ANALOG_MODBUS_NUM_READ				3			// Число попыток чтения
	  #define ANALOG_MODBUS_ERR_DELAY				50			// Задержка при ошибки чтения
	  const uint16_t ANALOG_MODBUS_ADDR[ANUMBER] = { 1, 1 };	// Адрес устройства Модбас, если 0, то датчик обычный.
	  const uint16_t ANALOG_MODBUS_REG[ANUMBER]  = { 60, 59 };	// Регистр Модбас, по которому доступно значение датчика. Для Vacon=(AI2, AI1)
	#endif
	// ------------------- ADC SENSOR ----------------------------------
	#define ADC_PRESCAL					9		// = (42 / ADCClockMhz - 1), - 4.2 MHz
	#define ADC_SKIP_EXTREMUM			300		// Отбрасывать максимумы/минимумы больше заданной дельты
	#define P_NUMSAMLES					1		// Число значений для усреднения показаний давления
	#define ADC_FREQ					10		// период опроса аналоговых датчиков в секунду
	#define FILTER_SIZE					3		// Длина фильтра для датчиков давления
	//#define FILTER_SIZE_OTHER			4		// Длина фильтра для остальных датчиков
	#define FILLING_TANK_STEP			300		// сотые %, На сколько должен заполняться бак за время Option.FillingTankTimeout

	#define DELAY_AFTER_SWITCH_RELAY	250		// Задержка после переключения реле, для сглаживания потребления и уменьшения помех(мс)

	// Статистика по дням
	#define STATS_ID_Temp	TAIR
	#define STATS_ID_Press	PWATER
	#define STATS_ID_Flow	FLOW
	Stats_Data Stats_data[] = {
		{ 0, STATS_OBJ_WaterUsed, STATS_TYPE_MAX },
		{ 0, STATS_OBJ_WaterRegen, STATS_TYPE_SUM },
		{ 0, STATS_OBJ_Flow, STATS_TYPE_MAX },
		{ 0, STATS_OBJ_WaterBooster, STATS_TYPE_SUM },
		{ 0, STATS_OBJ_FeedPump, STATS_TYPE_SUM },
		{ 0, STATS_OBJ_BrineWeight, STATS_TYPE_MIN },
		{ 0, STATS_OBJ_Temp, STATS_TYPE_MIN },
		{ 0, STATS_OBJ_Power, STATS_TYPE_SUM },
		{ 0, STATS_OBJ_Power, STATS_TYPE_MAX },
		{ 0, STATS_OBJ_Voltage, STATS_TYPE_MIN },
		{ 0, STATS_OBJ_Voltage, STATS_TYPE_MAX },
	};

	// История (графики)
	const History_setup HistorySetup[] = {
			{ STATS_OBJ_WaterUsed, 0, "Использовано, м³" },
			{ STATS_OBJ_WaterRegen, 0, "Регенерация, м³" },
			{ STATS_OBJ_WaterBooster, 0, "Насосная станция, сек" },
			{ STATS_OBJ_FeedPump, 0, "Дозирующий насос, сек" },
			{ STATS_OBJ_BrineWeight, 0, "Вес раствора, кг" },
			{ STATS_OBJ_Temp, TAIR, noteTemp[TAIR] },
			{ STATS_OBJ_Flow, FLOW, noteFrequency[FLOW] },
			{ STATS_OBJ_Level, LTANK, "Уровень в баке, %" },
			{ STATS_OBJ_Power, 0, "Потребление, кВт" },
	};



#endif  // CONFIG_1
  
#endif
