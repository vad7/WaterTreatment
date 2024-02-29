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
// Константы проекта
// --------------------------------------------------------------------------------
#ifndef Constant_h
#define Constant_h
#include "Config.h"                         // Цепляем сразу конфигурацию
#include "Util.h"

// ОПЦИИ КОМПИЛЯЦИИ ПРОЕКТА -------------------------------------------------------
#define VERSION			  "1.52"			// Версия прошивки
#define VER_SAVE		  12				// Версия формата сохраняемых данных в I2C память
//#define LOG                               // В последовательный порт шлет лог веб сервера (логируются запросы)
#define FAST_LIB                            // использование допиленной библиотеки езернета
#define TIME_ZONE         3                 // поправка на часовой пояс по ДЕФОЛТУ
#define NTP_SERVER        "time.nist.gov"   // NTP сервер для синхронизации времени по ДЕФОЛТУ
#define NTP_SERVER_LEN    60                // максимальная длиная адреса NTP сервера
#define NTP_PORT		  123
#define NTP_LOCAL_PORT    8888              // локальный порт, который будет прослушиваться на предмет UDP-пакетов NTP сервера
#define NTP_REPEAT        3                 // Число попыток запросов NTP сервера
#define NTP_REPEAT_TIME   1000              // (мсек) Время между повторами ntp пакетов
#define PING_SERVER       "8.8.8.8"         // ping сервер по ДЕФОЛТУ
#define WDT_TIME          6                 // период Watchdog таймера секунды но не более 16 секунд!!! ЕСЛИ установить 0 то Watchdog будет отключен!!!
#ifndef INDEX_FILE
#define INDEX_FILE        "index.html"       // стартовый файл по умолчанию для большой морды
#endif
#define INDEX_MOB_FILE    INDEX_FILE         // стартовый файл по умолчанию для мобильной морды
#define HEADER_BIN        "WT-SAVE-DATA"    // Заголовок (начало) файла при сохранении настроек. Необходим для поиска данных в буфере данных при восстановлении из файла
#define MAX_LEN_PM        250               // максимальная длина строкового параметра в запросе (расписание бойлера 175 байт) кодирование описания профиля 40 букв одна буква 6 байт (двойное кодирование)
#ifndef CHART_POINT                         // Можно определить себе спецальный размер графика в своем конфиге 
#define CHART_POINT       300               // Максимальное число точек графика, одна точка это 2 байта * число графиков
#endif
#ifndef ADC_PRESCAL
#define ADC_PRESCAL       2					// ADCClock = MCK / ( (PRESCAL+1) * 2 )
#endif

// СЕТЕВЫЕ НАСТРОЙКИ --------------------------------------------------------------
// По умолчанию и в демо режиме (действуют там и там)
// В рабочем режиме настройки берутся из ЕЕПРОМ, если прочитать не удалось, то действуют настройки по умолчанию
byte defaultMAC[] = { 0xDE, 0xA1, 0x1E, 0x01, 0x02, 0x04 };// не менять
const uint16_t  defaultPort=80;

// Макросы работы с битами байта, используется для флагов
#define GETBIT(b,f)   ((b&(1<<(f)))?true:false)              // получить состяние бита
#define SETBIT1(b,f)  (b|=(1<<(f)))                          // установка бита в 1
#define SETBIT0(b,f)  (b&=~(1<<(f)))                         // установка бита в 0
#define ALIGN(a) ((a + 3) & ~3)
// ------------------- SPI ----------------------------------
// Карта памяти
#define SD_REPEAT         3                 // Число попыток чтения карты, открытия файлов, при неудаче переход на работу без карты

// чип W5200 (точнее любой используемый чип)
#ifndef W5200_THREAD
#define W5200_THREAD      3                 // Число потоков для сетевого чипа w5200 допустимо 1-4 потока, на 4 скорее всего не хватить места в оперативке
#endif
#define W5200_NUM_PING    4                 // Число попыток пинга до определения потери связи
#define W5200_TIME_PING   1000              // мсек Время между попытками пинга (если не удача)
#define W5200_MAX_LEN     2048 //=W5100.SSIZE // Максимальная длина буфера, определяется W5200 не более 2048 байт
#define W5200_NUM_LINK    2                 // Число попыток сброса чипа w5500 и проверки появления связи (кабель воткнут) используется для инициализаци чипа
#define W5200_TIME_LINK   4000              // Максимальное время ожидания устанoвления связи (поднятие Link) кабель воткнут  используется для инициализаци чипа (мсек)
#define W5200_TIME_WAIT   3000              // Время ожидания захвата мютекса (переключение потоков) мсек
//#define W5200_STACK_SIZE  230             переименована и переехало в конфиг  // Размер стека (слова!!! - 4 байта) до обрезки стеков было 340 - работает
#define W5200_SPI_SPEED   SPI_RATE          // ЭТО ДЕЛИТЕЛЬ (SPI_RATE определен в w5100.h)!!! Частота SPI w5200 = 84/W5200_SPI_SPEED т.е. 2-42МГц 3-28МГц 4-21МГц 6-14МГц Диапазон 2-6
#define W5200_SOCK_SYS    (MAX_SOCK_NUM-1)  // Номер системного сокета который не использутся в вебсервере, это последний сокет, НЕ МЕНЯТЬ
#define W5200_RTR         (2*0x07D0)        // время таймаута в 100 мкс интервалах  (по умолчанию 200ms(100us X 2000(0x07D0))) актуально для комманд CONNECT, DISCON, CLOSE, SEND, SEND_MAC, SEND_KEEP
#define W5200_RCR         (0x04)            // число повторов передачи  (по умолчанию 0x08 раз))
#define MAIN_WEB_TASK      0                // какой поток вебсервера является основным (поток в котором идет отправка MQTT и уведомлений) обычно 0
#define W5500_LINK_CHECK  30				// Периодичност контроля соединения, сек

// ------------------- SERIAL --------------------------------
#ifdef DEBUG_NATIVE_USB
	#define SerialDbg	SerialUSB
#else
	#define SerialDbg	Serial
#endif
// Конфигурирование Modbus
#if RADIO_SENSORS_PORT == 2
	#define RADIO_SENSORS_SERIAL	Serial2	// Аппаратный порт
#elif RADIO_SENSORS_PORT == 3
	#define RADIO_SENSORS_SERIAL	Serial3	// Аппаратный порт
#endif
#define RADIO_LOST_TIMEOUT 30*60*1000		// через сколько считать, что связь потеряна с датчиком, мсек

// ------------------- TIME & DELAY ----------------------------------
// Времена и задержки
#define cDELAY_DS1820         750            // мсек. Задержка для чтения DS1820 (время преобразования)
#ifndef TIME_READ_SENSOR 
#define TIME_READ_SENSOR      4000		     // мсек. Период опроса датчиков
#endif
#define TIME_WEB_SERVER       2              // мсек. Период опроса web servera было 5
#define TIME_I2C_UPDATE       (60*60)*1000   // мсек. Время обновления внутренних часов по I2С часам (если конечно нужно)
#define TIME_MESSAGE_TEMP     300			 // 1/10 секунды, Проверка граничных температур для уведомлений
#define TIME_LED_OK           1500UL         // Период мигания светодиода при ОК (мсек)
#define TIME_LED_ERR          200            // Период мигания светодиода при ошибке (мсек).
#define TIME_BEEP_ERR         2000UL         // Период звукового сигнала при ошибке, мсек
#define cDELAY_START_MESSAGE  60             // Задержка (сек) после старта на отправку сообщений
#define NO_POWER_ON_DELAY     15		     // Задержка включения после появления питания, сек
#define WB_LOW_CONSUME_MAX_TIME 25000		 // Максимальное время бесперерывной работы насосной станции. После него будет переыв на включение дозатора, ms
#define WEB0_OTHER_FUNC_PERIOD 10            // Период выполнения других функция в задаче Web0, сек

#define PWM_READ_PERIOD      (3*1000)        // Время опроса не критичных параметров счетчика, ms
#define PWM_NUM_READ          2              // Число попыток чтения счетчика (подряд) до ошибки
#define PWM_DELAY_REPEAT      50             // мсек Время между ПОВТОРНЫМИ попытками чтения

#define DISPLAY_UPDATE		  2500           // Время обновления информации на дисплее (мсек)
#define KEY_CHECK_PERIOD	  10             // ms
#define KEY_DEBOUNCE_TIME	  50             // ms
#define DISPLAY_SETUP_TIMEOUT 600000         // ms

#define TIMER_TO_SHOW_STATUS  2500			// мсек, Время показа активного состояния (дозирующего насоса, ...)

#define HTTP_REQ_LowConsume   60  			// s, Период запроса о работе от резервного источника, по умолчанию
#define HTTP_REQ_TIMEOUT      2000			// ms

#define HOUR_SIGNAL_LIFE      12            // Час когда генерится сигнал жизни

#define NIGHT_END_HOUR			7			// Час после ночного тарифа

// ------------------- ОБЩИЕ НАСТРОЙКИ ----------------------------------
#define LCD_COLS			20			// Колонок на LCD экране
#define LCD_ROWS			4			// Строк на LCD экране
// ------------------- SENSOR TEMP----------------------------------
#define MAX_TEMP_ERR      700            // Максимальная систематическая ошибка датчика температуры (сотые градуса)
#define NUM_READ_TEMP_ERR 10             // Число ошибок подряд чтения датчика температуры после которого считается что датчик не исправен
#define RES_ONEWIRE_ERR   3              // Число попыток сброса датчиков температуры перед генерацией ошибки шины

#define FREQ_BASE_TIME_READ 1            // Частотные датчики - время (сек) на котором измеряется число импульсов, в конце идет пересчет в частоту

// ------------------- MQTT ----------------------------------
#define MQTT_REPEAT                      // Делать попытку повторного соединения с сервером
#define MQTT_NUM_ERR_OFF   8             // Число ошибок отправки подряд при котором отключается сервис отправки MQTT (флаг сбрасывается)

#define DEFAULT_PORT_MQTT 1883           // Адрес порта сервера MQTT
#define DEFAULT_TIME_MQTT (3*60)         // период отправки на сервер в сек. 10...60000
#define DEFAULT_ADR_MQTT "mqtt.thingspeak.com"   // Адрес MQTT сервера по умолчанию
#define DEFAULT_ADR_NARMON "narodmon.ru" // Адрес сервера народного мониторинга
#define TIME_NARMON       (5*60)         // (сек) Период отправки на народный монитоинг (константа) меньше 5 минут не ставить

// ------------------- HEAP ----------------------------------
#define PASS_LEN          10             // Максимальная длина пароля для входа на контроллер
const char NAME_USER[]  = "user";        // имя пользователя
const char NAME_ADMIN[] = "admin";       // имя администратора
//#define FILE_STATISTIC    "statistic.csv"// имя файла статистики за ТЕКУШИЙ период

#define ATOF_ERROR       -9876543.0f     // Код ошибки преобразования строки во флоат

//----------------------- WEB ----------------------------
const char WEB_HEADER_OK_CT[] 			= "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: ";
const char WEB_HEADER_TEXT_ATTACH[] 	= "text/plain\r\nContent-Disposition: attachment; filename=\"";
const char WEB_HEADER_BIN_ATTACH[] 		= "application/x-binary\r\nContent-Disposition: attachment; filename=\"";
const char WEB_HEADER_TXT_KEEP[] 		= "text/html\r\nConnection: keep-alive";
const char WEB_HEADER_END[]				= "\r\n\r\n";
const char* HEADER_FILE_NOT_FOUND = {"HTTP/1.1 404 Not Found\r\n\r\n<html>\r\n<head><title>404 NOT FOUND</title><meta charset=\"utf-8\" /></head>\r\n<body><h1>404 NOT FOUND</h1></body>\r\n</html>\r\n\r\n"};
//const char* HEADER_FILE_WEB       = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: keep-alive\r\n\r\n"}; // КЕШ НЕ ИСПОЛЬЗУЕМ
const char* HEADER_FILE_WEB       = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: keep-alive\r\nCache-Control: max-age=3600, must-revalidate\r\n\r\n"}; // КЕШ ИСПОЛЬЗУЕМ
const char* HEADER_FILE_CSS       = {"HTTP/1.1 200 OK\r\nContent-Type: text/css\r\nConnection: keep-alive\r\nCache-Control: max-age=3600, must-revalidate\r\n\r\n"}; // КЕШ ИСПОЛЬЗУЕМ
const char* HEADER_ANSWER         = {"HTTP/1.1 200 OK\r\nContent-Type: text/ajax\r\nAccess-Control-Allow-Origin: *\r\n\r\n"};  // начало ответа на запрос
static uint8_t  fWebUploadingFilesTo = 0;                // Куда грузим файлы: 1 - SPI flash, 2 - SD card

// Константы регистров контроллера питания SOPC SAM3x ---------------------------------------
// Регистр SMMR
// Уровень контроля питания ядра
#define   SUPC_SMMR_SMTH_1_9V (0x0u << 0) 
#define   SUPC_SMMR_SMTH_2_0V (0x1u << 0) 
#define   SUPC_SMMR_SMTH_2_1V (0x2u << 0) 
#define   SUPC_SMMR_SMTH_2_2V (0x3u << 0) 
#define   SUPC_SMMR_SMTH_2_3V (0x4u << 0) 
#define   SUPC_SMMR_SMTH_2_4V (0x5u << 0) 
#define   SUPC_SMMR_SMTH_2_5V (0x6u << 0) 
#define   SUPC_SMMR_SMTH_2_6V (0x7u << 0) 
#define   SUPC_SMMR_SMTH_2_7V (0x8u << 0) 
#define   SUPC_SMMR_SMTH_2_8V (0x9u << 0) 
#define   SUPC_SMMR_SMTH_2_9V (0xAu << 0) 
#define   SUPC_SMMR_SMTH_3_0V (0xBu << 0) 
#define   SUPC_SMMR_SMTH_3_1V (0xCu << 0) 
#define   SUPC_SMMR_SMTH_3_2V (0xDu << 0) 
#define   SUPC_SMMR_SMTH_3_3V (0xEu << 0) 
#define   SUPC_SMMR_SMTH_3_4V (0xFu << 0) 
// Время контроля питания
#define   SUPC_SMMR_SMSMPL_SMD (0x0u << 8)       // запрещено
#define   SUPC_SMMR_SMSMPL_CSM (0x1u << 8)       // непрерывно
#define   SUPC_SMMR_SMSMPL_32SLCK (0x2u << 8) 
#define   SUPC_SMMR_SMSMPL_256SLCK (0x3u << 8) 
#define   SUPC_SMMR_SMSMPL_2048SLCK (0x4u << 8) 
#define   SUPC_SMMR_SMRSTEN (0x1u << 12) /**< \brief (SUPC_SMMR) Supply Monitor Reset Enable */
#define   SUPC_SMMR_SMRSTEN_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_SMMR) the core reset signal "vddcore_nreset" is not affected when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMRSTEN_ENABLE (0x1u << 12) /**< \brief (SUPC_SMMR) the core reset signal, vddcore_nreset is asserted when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMIEN (0x1u << 13) /**< \brief (SUPC_SMMR) Supply Monitor Interrupt Enable */
#define   SUPC_SMMR_SMIEN_NOT_ENABLE (0x0u << 13) /**< \brief (SUPC_SMMR) the SUPC interrupt signal is not affected when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMIEN_ENABLE (0x1u << 13) /**< \brief (SUPC_SMMR) the SUPC interrupt signal is asserted when a supply monitor detection occurs. */
// Регистр MR
#define SUPC_MR_KEY_Pos 24
#define SUPC_MR_KEY_Msk (0xffu << SUPC_MR_KEY_Pos) // Ключ для записи!!!
#define SUPC_MR_KEY(value) ((SUPC_MR_KEY_Msk & ((value) << SUPC_MR_KEY_Pos)))
#define SUPC_MR_BODRSTEN (0x1u << 12) /**< \brief (SUPC_MR) Brownout Detector Reset Enable */
#define SUPC_MR_BODRSTEN_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_MR) the core reset signal "vddcore_nreset" is not affected when a brownout detection occurs. */
#define SUPC_MR_BODRSTEN_ENABLE (0x1u << 12) /**< \brief (SUPC_MR) the core reset signal, vddcore_nreset is asserted when a brownout detection occurs. */
#define SUPC_MR_BODDIS (0x1u << 13)     // Вот это /**< \brief (SUPC_MR) Brownout Detector Disable */
#define SUPC_MR_BODDIS_ENABLE (0x0u << 13) /**< \brief (SUPC_MR) the core brownout detector is enabled. */
#define SUPC_MR_BODDIS_DISABLE (0x1u << 13) /**< \brief (SUPC_MR) the core brownout detector is disabled. */

// GPBR  - 8 32x битных регистров не обнуляющиеся при сбросе, Энергозависимы!
// адереса - 0x90-0xDC General Purpose Backup Register GPBR   
// The System Controller embeds Eight general-purpose backup registers.
// Карта использования GPBR в НК
// GPBR->SYS_GPBR[0] текущую задача (сдвиг на 8 влево) +  номера ошибки RTOS 
// GPBR->SYS_GPBR[1] причина сброса контроллера
// GPBR->SYS_GPBR[4] последня точка отладки перед сбросом максимальнаое значение точки отладки сейчас 54 (обновляем!)
#define STORE_DEBUG_INFO(s) GPBR->SYS_GPBR[4] = s  // Сохранение номера точки отладки в энергозависимой памяти sam3
#define STORE_DEBUG_INFO2(s) GPBR->SYS_GPBR[5] = s  // Сохранение номера точки отладки в энергозависимой памяти sam3

#ifndef FORMAT_DATE_STR_CUSTOM
const char *FORMAT_DATE_STR	 = { "%02d/%02d/%04d" };
#endif

// --------------------------------------------------------------------------------------------------------------------------------
// Строковые константы многократно используемые по всем файлам --------------------------------------------------------------------
const char *cYes= {"Да" };
const char *cNo = {"Нет"};
const char *cOne= {"1"  };
const char *cZero={"0"  };
const char *cError={"error"};
const char *cInvalid={"ERR"};
const char *cStrEnd={"\n"};
const char *cErrorRS485={"%s: Read error %s, code=%d repeat . . .\n"};  // имя, функция, код
const char *cErrorMutex={"Function %s: %s, mutex is buzy\n"};           // функция, мютекс
const char http_get_str1[] = "GET ";
const char http_get_str2[] = " HTTP/1.0\r\nHost: ";
const char http_get_str3[] = "\r\nAccept: text/html\r\n\r\n";
const char http_get_str4[] = "/&";
const char http_key_ok1[] = "HTTP/"; // "1.1"
const char http_key_ok2[] = " 200 OK\r\n";
const char http_Length[]   = "Content-Length: "; // где лежит длина файла
const uint8_t save_end_marker[1] = { 0 };
#define WEBDELIM	"\x7f" // ALT+127
const char SendMessageTitle[]	= "Водоподготовка";
const char SendSMSTitle[] 		= "Water";

const char *nameFREERTOS =     {"FreeRTOS"};           // Имя источника ошибки (нужно для передачи в функцию) - операционная система
const char *nameMainClass =    {"WaterTreatment"};     // Имя (для лога ошибок)
const char *MutexI2CBuzy =     {"I2C"}; 
const char *MutexModbusBuzy=   {"Modbus"}; 
const char *MutexWebThreadBuzy={"WebThread"}; 
const char *MutexSPIBuzy=      {"SPI"}; 
const char *MutexCommandBuzy = {"Command"}; 

// Описание имен параметров MQTT для функций get_paramMQTT set_paramMQTT
const char *mqtt_USE_TS           =  {"USE_TS"};         // флаг использования ThingSpeak - формат передачи для клиента
const char *mqtt_USE_MQTT         =  {"USE_MQTT"};       // флаг использования MQTT
const char *mqtt_BIG_MQTT         =  {"BIG_MQTT"};       // флаг отправки ДОПОЛНИТЕЛЬНЫХ данных на MQTT
const char *mqtt_SDM_MQTT         =  {"SDM_MQTT"};       // флаг отправки данных электросчетчика на MQTT
const char *mqtt_FC_MQTT          =  {"FC_MQTT"};        // флаг отправки данных инвертора на MQTT
const char *mqtt_COP_MQTT         =  {"COP_MQTT"};       // флаг отправки данных COP на MQTT
const char *mqtt_TIME_MQTT        =  {"TIME_MQTT"};      // период отправки на сервер в сек. 10...60000
const char *mqtt_ADR_MQTT         =  {"ADR_MQTT"};       // Адрес сервера
const char *mqtt_IP_MQTT          =  {"IP_MQTT"};        // IP Адрес сервера
const char *mqtt_PORT_MQTT        =  {"PORT_MQTT"};      // Адрес порта сервера
const char *mqtt_LOGIN_MQTT       =  {"LOGIN_MQTT"};     // логин сервера
const char *mqtt_PASSWORD_MQTT    =  {"PASSWORD_MQTT"};  // пароль сервера
const char *mqtt_ID_MQTT          =  {"ID_MQTT"};        // Идентификатор клиента на MQTT сервере
   // народный мониторинг
const char *mqtt_USE_NARMON       =  {"USE_NARMON"};     // флаг отправки данных на народный мониторинг
const char *mqtt_BIG_NARMON       =  {"BIG_NARMON"};     // флаг отправки данных на народный мониторинг ,большую версию
const char *mqtt_ADR_NARMON       =  {"ADR_NARMON"};     // Адрес сервера народного мониторинга
const char *mqtt_IP_NARMON        =  {"IP_NARMON"};      // IP Адрес сервера народного мониторинга
const char *mqtt_PORT_NARMON      =  {"PORT_NARMON"};    // Адрес порта сервера  народного мониторинга
const char *mqtt_LOGIN_NARMON     =  {"LOGIN_NARMON"};   // логин сервера народного мониторинга
const char *mqtt_PASSWORD_NARMON  =  {"PASSWORD_NARMON"};// пароль сервера народного мониторинга
const char *mqtt_ID_NARMON        =  {"ID_NARMON"};      // Идентификатор клиента на MQTT сервере

// Описание имен параметров электросчетчика для функций get_paramSDM ("get_PWM"), set_paramSDM ("set_PWM")
const char *pwm_NAME        = {"NAME"};            // Имя счетчика
const char *pwm_NOTE        = {"NOTE"};            // Описание счетчика
const char *pwm_VOLTAGE     = {"V"};               // Напряжение
const char *pwm_CURRENT     = {"I"};               // Ток
const char *pwm_POWER       = {"P"};               // Полная мощность
const char *pwm_PFACTOR     = {"K"};               // Коэффициент мощности
const char *pwm_FREQ        = {"F"};               // Частота
const char *pwm_ACENERGY    = {"E"};               // Суммарная активная энергия
const char *pwm_ERRORS  	= {"ERR"};             // Ошибок чтения Modbus
const char *pwm_RESET		= {"RESET"};
const char *pwm_TestPower	= {"TP"};
const char *pwm_ModbusAddr	= {"M"};

// Описание имен параметров уведомлений для функций set_messageSetting get_messageSetting
const char *mess_MAIL         = {"MAIL"};                // флаг уведомления скидывать на почту
const char *mess_MAIL_AUTH    = {"MAIL_AUTH"};           // флаг необходимости авторизации на почтовом сервере
const char *mess_MAIL_INFO    = {"MAIL_INFO"};           // флаг необходимости добавления в письмо информации о состоянии
const char *mess_SMS          = {"SMS"};                 // флаг уведомления скидывать на СМС
const char *mess_MESS_RESET   = {"MESS_RESET"};          // флаг уведомления Сброс
const char *mess_MESS_ERROR   = {"MESS_ERROR"};          // флаг уведомления Ошибка
const char *mess_MESS_LIFE    = {"MESS_LIFE"};           // флаг уведомления Сигнал жизни
const char *mess_MESS_TEMP    = {"MESS_TEMP"};           // флаг уведомления Достижение граничной температуры
const char *mess_MESS_SD      = {"MESS_SD"};             // флаг уведомления "Проблемы с sd картой"
const char *mess_MESS_WARNING = {"MESS_WARNING"};        // флаг уведомления "Прочие уведомления"
const char *mess_SMTP_SERVER  = {"SMTP_SERVER"};         // Адрес сервера
const char *mess_SMTP_IP      = {"SMTP_IP"};             // IP Адрес сервера
const char *mess_SMTP_PORT    = {"SMTP_PORT"};           // Адрес порта сервера
const char *mess_SMTP_LOGIN   = {"SMTP_LOGIN"};          // логин сервера если включена авторизация
const char *mess_SMTP_PASS    = {"SMTP_PASS"};           // пароль сервера если включена авторизация
const char *mess_SMTP_MAILTO  = {"SMTP_MAILTO"};         // адрес отправителя
const char *mess_SMTP_RCPTTO  = {"SMTP_RCPTTO"};         // адрес получателя
const char *mess_SMS_SERVICE  = {"SMS_list"};         // сервис отправки смс
const char *mess_SMS_IP       = {"SMS_IP"};              // IP Адрес сервера для отправки смс
const char *mess_SMS_PHONE    = {"SMS_PHONE"};           // телефон куда отправляется смс
const char *mess_SMS_P1       = {"SMS_P1"};              // первый параметр для отправки смс
const char *mess_SMS_P2       = {"SMS_P2"};              // второй параметр для отправки смс
const char *mess_SMS_NAMEP1   = {"SMS_NAMEP1"};          // описание первого параметра для отправки смс
const char *mess_SMS_NAMEP2   = {"SMS_NAMEP2"};          // описание второго параметра для отправки смс
const char *mess_MESS_TIN     = {"MESS_TAIR"};           // Критическая температура (если меньше то генерится уведомление)
const char *mess_MAIL_RET     = {"scan_MAIL"};           // Ответ на тестовую почту
const char *mess_SMS_RET      = {"scan_SMS"};            // Ответ на тестовую  sms

// Дата время
const char *time_TIME       = {"TIME"};         // текущее время  12:45 без секунд
const char *time_DATE       = {"DATE"};         // текушая дата типа  12/04/2016
const char *time_NTP        = {"NTP"};          // адрес NTP сервера строка до 60 символов.
const char *time_UPDATE     = {"UPDATE"};       // Время синхронизации с NTP сервером.
const char *time_UPDATE_I2C = {"UPDATE_I2C"};   // Синхронизация времени раз в час с i2c часами

// Сеть
const char *net_IP         = {"IP"};               // Адрес 
const char *net_DNS        = {"DNS"};              // DNS
const char *net_GATEWAY    = {"GATEWAY"};          // Шлюз
const char *net_SUBNET     = {"SUBNET"};           // Маска подсети
const char *net_DHCP       = {"DHCP"};             // Флаг использования DHCP
const char *net_MAC        = {"MAC"};              // МАС адрес чипа
const char *net_RES_SOCKET = {"NSLS"};             // Время сброса зависших сокетов
const char *net_RES_W5200  = {"NSLR"};             // Время регулярного сброса сетевого чипа
const char *net_PASS       = {"PASS"};             // Использование паролей (флаг)
const char net_PASSUSER[]  = "PASSUSER";           // Пароль пользователя
const char net_PASSADMIN[] = "PASSADMIN";          // Пароль администратора
const char *net_SIZE_PACKET= {"SIZE"};             // размер пакета
const char *net_INIT_W5200 = {"INIT"};             // Ежеминутный контроль SPI для сетевого чипа
const char *net_PORT       = {"PORT"};             // Port веб сервера
const char *net_NO_ACK     = {"NO_ACK"};           // Не ожидать ответа ack
const char *net_DELAY_ACK  = {"DELAY_ACK"};        // Задержка перед посылкой следующего пакета
const char *net_PING_ADR   = {"PING"};             // адрес для пинга
const char *net_PING_TIME  = {"NSLP"};             // время пинга в секундах
const char *net_NO_PING    = {"NO_PING"};          // запрет пинга контроллера
const char *net_fWebLogError={"WLOG"};
const char *net_fWebFullLog= {"WFLOG"};

// Описание имен параметров Графиков для функций get_Chart ЕСЛИ нет совпадения с именами объектов
const char *chart_NONE       = {"NONE"};                    // 0 ничего не показываем
const char *chart_VOLTAGE    = {"Voltage"};                 // Статистика по напряжению
const char *chart_CURRENT    = {"Current"};                 // Статистика по току
const char *chart_fullPOWER  = {"Power"};               // Статистика по Полная мощность
const char *chart_WaterBoostCount = {"BoosterTank, L"};
const char *chart_WaterBoost = {"WaterBooster, s"};
const char *chart_FeedPump   = {"FeedPump, s"};
const char *chart_FillTank   = {"FillTank, s"};
const char *chart_BrineWeight= {"Weight"};

// Описание имен параметров опций   для функций get_option ("get_Opt") set_option ("set_Opt")
const char *option_ATTEMPT            	= {"ATTEMPT"};            // число попыток пуска
const char *option_TIME_CHART         	= {"TIME_CHART"};         // период сбора статистики
const char *option_BEEP               	= {"BEEP"};               // включение звука
const char *option_History            	= {"HIST"};               // запись истории на SD карту
const char *option_PWM_LOG_ERR        	= {"PWMLOG"};          // флаг писать в лог нерегулярные ошибки счетчика
const char *option_SAVE_ON            	= {"SAVE_ON"};            // флаг записи в EEPROM включения (восстановление работы после перезагрузки)
const char option_SGL1W[]             	= "SGL1W_";			    // SGLOW_n, На шине n (1-Wire, DS2482) только один датчик
const char *option_WebOnSPIFlash		= {"WSPIF"};              // флаг, что веб морда лежит на SPI Flash, иначе на SD карте
const char *option_LogWirelessSensors	= {"LOGWS"};              // Логировать обмен между беспроводными датчиками
const char *option_fDontRegenOnWeekend	= {"NRW"};
const char *option_FeedPumpMaxFlow		= {"FPMF"};
const char *option_FeedPumpRate			= {"FPR"};
const char *option_fFlowIncByPressure	= {"FBP"};
const char *option_FlowIncByPress_MinFlow= {"FBM"};
const char *option_BackWashFeedPumpMaxFlow= {"BWMF"};
const char *option_BackWashFeedPumpDelay = {"BWFD"};
const char *option_RegenHour			= {"RH"};
const char *option_DaysBeforeRegen		= {"DBR"};
const char *option_DaysBeforeRegenSoftening={"DBRS"};
const char *option_UsedBeforeRegen		= {"UBR"};
const char *option_UsedBeforeRegenSoftening={"UBRS"};
const char *option_MinPumpOnTime		= {"MPOT"};
const char *option_MinWaterBoostOnTime	= {"MWBT"};
const char *option_MinWaterBoostOffTime	= {"MWBTF"};
const char *option_MinRegen				= {"MR"};
const char *option_MinRegenSoftening	= {"MRS"};
const char *option_MinRegenWeightDecrease={"MRW"};
const char *option_MinRegenWeightDecreaseSoftening={"MRWS"};
const char *option_MinDrain				= {"MD"};
const char *option_MinWaterBoosterCountL= {"MWC"};
const char *option_DrainTime			= {"DT"};
const char *option_DrainAfterNoConsume	= {"DA"};
const char *option_PWM_DryRun			= {"DR"};
const char *option_PWM_Max				= {"PM"};
const char *option_PWM_StartingTime		= {"PST"};
const char *option_FloodingDebounceTime	= {"FDT"};
const char *option_FloodingTimeout		= {"FT"};
const char *option_PWATER_RegMin		= {"WRM"};
const char *option_PWATER_Osmos_Min		= {"WOM"};
const char *option_PWATER_Osmos_Step	= {"WOS"};
const char *option_LTANK_Low			= {"TE"};
const char *option_Weight_Low			= {"WE"};
const char *option_fDebugToJournal		= {"DBG"};
const char *option_fDebugToSerial		= {"DBGS"};
const char *option_fRegenAllowed		= {"RA1"};
const char *option_fRegenAllowedSoftener= {"RA2"};
const char *option_FillingTankTimeout	= {"FTT"};
const char *option_TankCheckPercent		= {"TCP"};
const char *option_TankFillingTimeMax	= {"TFM"};
const char *option_CriticalErrorsTimeout= {"CET"};
const char *option_FilterTank           = {"TD"};
const char *option_FilterTankSoftener   = {"TDS"};
const char *option_DrainingWaterAfterRegen={"DAR"};
const char *option_DrainingWaterAfterRegenSoftening={"DAS"};
const char *option_RegenStart			= {"R"};
const char *option_LTank_LowConsumeMin	= {"LLM"};
const char *option_LTank_AfterFilledTimer={"TAF"};
const char *option_LTank_Hour_Low		= {"LHL"};
const char *option_LTank_Hour			= {"LH"};
const char *option_RFILL_HoursRepeatPulse= {"RFIL"};
const char *option_LowConsumeRequestPeriod={"LCP"};
const char *option_fLowConsumeReq_OnByErr={"LEO"};
const char *option_SepticAlarmDebounce  ={"SD"};
const char *option_RegenSofteningCntAlarm={"RSA"};
const char *option_DrainSiltTime		= {"DST"};
const char *option_DrainSiltAfterL100	= {"DSL"};
const char *option_DrainSiltAfterNotUsed= {"DSH"};
const char *option_fDrainSiltTank		= {"DSF"};
const char *option_fDrainSiltTankBeforeRegen= {"DSR"};
const char *option_UsedWaterContinuous	= {"UWC"};
const char *option_FilterCounter1_Max	= {"FC1"};
const char *option_FilterCounter2_Max	= {"FC2"};
const char *option_DrainPumpMaxTime		= {"DPT"};
const char *option_fLED_SRV_INFO_PlanReg= {"LPR"};
const char *option_fCheckDrainPump		= {"CDP"};
const char option_GetCurrentSaltLevel[] = "LvL";	// get_Opt(LvL)
const char prof_DailySwitch[] 	= "DS";
const char prof_DailySwitchDevice = 'D';		// DSD
const char prof_DailySwitchOn  	= 'S';			// DSS
const char prof_DailySwitchOff 	= 'E';			// DSE


// WorkStats, get_WS..., set_WS...(x)
const char *webWS_UsedToday 					= { "UD" };
const char *webWS_UsedYesterday 				= { "UY" };
const char *webWS_LastDrain		 		 		= { "DD" };
const char *webWS_RegCnt  						= { "RC" };
const char *webWS_DaysFromLastRegen 		 	= { "RD" };
const char *webWS_UsedSinceLastRegen  			= { "RS" };
const char *webWS_UsedLastRegen  				= { "RL" };
const char *webWS_RegCntSoftening  				= { "RSC" };
const char *webWS_DaysFromLastRegenSoftening	= { "RSD" };
const char *webWS_UsedSinceLastRegenSoftening	= { "RSS" };
const char *webWS_UsedLastRegenSoftening  		= { "RSL" };
const char *webWS_NextRegenAfterDays			= { "N" };
const char *webWS_NextRegenSoftAfterDays		= { "NS" };
#define		webWS_UsedAverageDay 					'A'
#define		webWS_WaterBoosterCountL				'B'
#define		webWS_UsedDrain 	 					'D'
#define		webWS_UsedTotal  						'T'
#define		webWS_Velocity							'V'
#define		webWS_Flags  						    'F'

// --------------------------------------------------------------------------------
// ОШИБКИ едины для всего - сквозной список
// --------------------------------------------------------------------------------
#define OK                  0          // Ошибок нет
#define ERR_MINTEMP        -1          // Выход за нижнюю границу температурного датчика
#define ERR_MAXTEMP        -2          // Выход за верхнюю границу температурного датчика
#define ERR_MINPRESS       -3          // Выход за нижнюю границу  датчика давления
#define ERR_MAXPRESS       -4          // Выход за верхнюю границу датчика давления
#define ERR_DINPUT         -5          // Срабатывания контактного датчика -  авария
#define ERR_DEVICE         -6         // Устройство запрещено в текущей конфигурации
#define ERR_ONEWIRE        -7         // Ошибка сброса на OneWire шине (обрыв или замыкание)
#define ERR_MEM_FREERTOS   -8         // Free RTOS не может создать задачу - мало пямяти
#define ERR_SAVE_EEPROM    -9         // Ошибка записи настроек в eeprom I2C
#define ERR_LOAD_EEPROM    -10         // Ошибка чтения настроек из eeprom I2C
#define ERR_CRC16_EEPROM   -11         // Ошибка контрольной суммы для настроек
#define ERR_BAD_LEN_EEPROM -12         // Не совпадение размера данных при чтении настроек
#define ERR_HEADER_EEPROM  -13         // Данные настроек не найдены  в eeprom I2C
#define ERR_SAVE1_EEPROM   -14         // Ошибка записи состояния в eeprom I2C
#define ERR_LOAD1_EEPROM   -15         // Ошибка чтения состояния из eeprom I2C
#define ERR_HEADER1_EEPROM -16         // Данные состояния не найдены в eeprom I2C
#define ERR_SAVE2_EEPROM   -17         // Ошибка записи счетчиков в eeprom I2C
#define ERR_LOAD2_EEPROM   -18         // Ошибка чтения счетчиков из eeprom I2C
#define ERR_READ_PRESS     -19         // Ошибка чтения датчика давления (данные не готовы)
#define ERR_CONFIG         -20         // Сбой внутренней конфигурации (обратитесь к разработчику)
#define ERR_SD_INIT        -21         // Ошибка инициализации SD карты
#define ERR_SD_INDEX       -22         // Файл index.xxx не найден на SD карте
#define ERR_SD_READ        -23         // Ошибка чтения файла с SD карты
#define ERR_485_BUZY       -24         // При обращении к 485 порту привышено время ожидания его освобождения
// Ошибки описаные в протоколе Modbus
#define ERR_MODBUS_0x01    -25         // Modbus 0x01 protocol illegal function exception
#define ERR_MODBUS_0x02    -26         // Modbus 0x02 protocol illegal data address exception
#define ERR_MODBUS_0x03    -27         // Modbus 0x03 protocol illegal data value exception
#define ERR_MODBUS_0x04    -28         // Modbus 0х04 protocol slave device failure exception
#define ERR_MODBUS_0xe0    -29         // Modbus 0xe0 Master invalid response slave ID exception
#define ERR_MODBUS_0xe1    -30         // Modbus 0xe1 Master invalid response function exception
#define ERR_MODBUS_0xe2    -31         // Modbus 0xe2 Master response timed out exception
#define ERR_MODBUS_0xe3    -32         // Modbus 0xe3 Master invalid response CRC exception
#define ERR_MODBUS_UNKNOWN -33         // Modbus не известная ошибка (сбой протокола)
#define ERR_MODBUS_STATE   -34         // Запрещенное (не верное) состояние инвертора
#define ERR_OUT_OF_MEMORY  -35         // Не хватает памяти для выделения массивов
#define ERR_DS2482_NOT_FOUND -36       // Мастер DS2482 не найден на шине, возможно ошибка шины I2C
#define ERR_DS2482_ONEWIRE -37         // Мастер DS2482 не может сбросить шину OneWire бит PPD равен 0
#define ERR_I2C_BUZY       -38         // При обращении к I2C шине превышено время ожидания ее освобождения
#define ERR_HEADER2_EEPROM -39         // Ошибка заголовка счетчиков в eeprom I2C
#define ERR_OPEN_I2C_JOURNAL -40       // Ошибка открытия журнала в I2C памяти (инициализация чипа)
#define ERR_READ_I2C_JOURNAL -41       // Ошибка чтения журнала в I2C памяти
#define ERR_WRITE_I2C_JOURNAL -42      // Ошибка записи журнала в I2C памяти
#define ERR_MAX_VOLTAGE     -43        // Слишком большое напряжение сети
#define ERR_MAX_POWER       -44        // Слишком большая портебляемая мощность
#define ERR_NO_MODBUS       -45        // Modbus требуется но отсутвует в конфигурации
#define ERR_READ_TEMP       -46        // Ошибка чтения температурного датчика (лимит чтения исчерпан)
#define ERR_ONEWIRE_CRC     -47		   // ошибка CRC во время чтения OneWire
#define ERR_ONEWIRE_RW      -48		   // ошибка во время чтения/записи OneWire
#define ERR_SD_WRITE		-49		   // ошибка записи на SD карту
#define ERR_ADDRESS			-50        // Адрес датчика температуры не установлен
#define ERR_FEW_LITERS_REG	-51			// Мало израсходовано воды при регенерации
#define ERR_FEW_LITERS_DRAIN -52		// Мало израсходовано воды при сбросе воды
#define ERR_WATER_CNT_FAIL	-53			// Неисправность счетчика воды
#define ERR_RTC_LOW_BATTERY -54			// Села батарея часов
#define ERR_PWM_DRY_RUN		-55			// Сухой ход двигателя насоса
#define ERR_PWM_MAX			-56			// Перегрузка двигателя насоса
#define ERR_PRESS			-57			// Ошибка давления
#define ERR_FLOODING		-58			// Затопление
//#define ERR_TANK_EMPTY	-59				// Пустой бак! (defined in config.h)
#define ERR_TANK_NO_FILLING	-60			// Бак не заполняется
#define ERR_RTC_WRITE		-61			// Ошибка записи в RTC память
#define ERR_START_REG		-62			// Не запускается регенерация обезжелезивателя
#define ERR_START_REG2		-63			// Не запускается регенерация умягчителя
#define ERR_WEIGHT_LOW		-64			// Маленький вес реагента
#define ERR_WEIGHT_EMPTY	-65			// Пустой бак с реагентом
#define ERR_LEAK			-66			// Протечка
#define ERR_SEPTIC_ALARM	-67			// Авария септика
#define ERR_NO_POWER		-68			// Нет электричества
#define ERR_REG_FEW_WEIGHT_CONSUME -69	// Мало израсходовано реагента во время регенерации
#define ERR_SALT_FINISH		-70			// Закончилась соль в баке умягчителя
#define ERR_REGEN_CONFLICT	-71			// Две регенерации в одно и тоже время
#define ERR_REGEN_EXPIRED	-72			// Регенерация обезжелезивателя долго не запускалась!
#define ERR_REGEN2_EXPIRED	-73			// Регенерация умягчителя долго не запускалась!
#define ERR_FEEDPUMP_TIME	-74			// Слишком долго работает дозатор!
#define ERR_TANK_LEAKAGE	-75			// Бак протек
#define ERR_TANK_FILLING_LONG -76		// Слишком долгое заполнение бака
#define ERR_LONG_WATER_CONSUMING -77	// Долгое непрерывное потребление воды!
#define ERR_DRAIN_PUMP_LINK	 -78		// Ошибка связи со счетчиком насоса
#define ERR_DRAIN_PUMP_TOOLONG -79		// Слишком долго работает насос

#define ERR_ERRMAX			-79			// Последняя ошибка

// Предупреждения
#define WARNING_VALUE        1         // Попытка установить значение за границами диапазона запрос типа SET

// Описание ВСЕХ Ошибок длина описания не более 160 байт (ограничение основной класс note_error[160+1])
const char *noteError[] = {
		"Ok",                                                  								//  0
		"Выход за нижнюю границу температурного датчика",      								// -1
		"Выход за верхнюю границу температурного датчика",     								// -2
		"Выход за нижнюю границу  датчика давления",           								// -3
		"Выход за верхнюю границу датчика давления",           								// -4
		"Срабатывания контактного датчика - авария",           								// -5
		"Устройство запрещено в текущей конфигурации",         								// -6
		"Ошибка сброса на OneWire шине (обрыв или замыкание)", 								// -7
		"Free RTOS не может создать задачу, не хватает пямяти",								// -8
		"Ошибка записи настроек", 			        		  								// -9
		"Ошибка чтения настроек",					          								// -10
		"Ошибка контрольной суммы для настроек в I2C",         								// -11
		"Не совпадение размера данных при чтении настроек",    								// -12
		"Данные настроек не найдены в I2C",			          								// -13
		"Ошибка записи состояния в I2C",			              							// -14
		"Ошибка чтения состояния в I2C",			              							// -15
		"Данные состояния не найдены в I2C",			          							// -16
		"Ошибка записи счетчиков в I2C",                	      							// -17
		"Ошибка чтения счетчиков из I2C", 		              								// -18
		"Ошибка чтения датчика давления (данные не готовы)",          						// -19
		"Сбой внутренней конфигурации (обратитесь к разработчику)",           				// -20
		"Ошибка инициализации SD карты",                                      				// -21
		"Файл индекса не найден на SD карте",                              					// -22
		"Ошибка чтения файла с SD карты",                                     				// -23
		"При обращении к 485 порту привышено время ожидания его освобождения",              // -24
		"Modbus error 0x01 protocol illegal function exception",                            // -25
		"Modbus error 0x02 protocol illegal data address exception",                        // -26
		"Modbus error 0x03 protocol illegal data value exception",                          // -27
		"Modbus error 0х04 protocol slave device failure exception",                        // -28
		"Modbus error 0xe0 Master invalid response slave ID exception",                     // -29
		"Modbus error 0xe1 Master invalid response function exception",                     // -30
		"Modbus error 0xe2 Master response timed out exception",                            // -21
		"Modbus error 0xe3 Master invalid response CRC exception",                          // -32
		"Modbus не известная ошибка (сбой протокола)",                                      //-33
		"Запрещенное (не верное) состояние инвертора",                                      //-34
		"Не хватает памяти для выделения под данные (см. конфигурацию).",                   //-35
		"Мастер DS2482 не найден на шине, возможно ошибка шины I2C",                        //-36
		"Мастер DS2482 не может сбросить шину OneWire бит PPD равен 0",                     //-37
		"При обращении к I2C шине превышено время ожидания ее освобождения",                //-38
		"Ошибка заголовка счетчиков в I2C",                          		                //-39
		"Ошибка открытия журнала в I2C (инициализация чипа)", 		                        //-40
		"Ошибка чтения журнала из I2C памяти",                                              //-41
		"Ошибка записи журнала в I2C память",                                               //-42
		"Слишком большое напряжение сети",                                                  //-43
		"Слишком большая портебляемая мощность",                                            //-44
		"Modbus требуется, но отсутвует в конфигурации",                                    //-45
		"Ошибка чтения температурного датчика (лимит попыток чтения исчерпан)",             //-46
		"Ошибка CRC чтения OneWire",														//-47
		"Ошибка чтения/записи OneWire",														//-48
		"Ошибка записи на SD карту",														//-49
		"Адрес датчика температруры не установлен",											//-50
		"Мало израсходовано воды при регенерации",											//-51
		"Мало израсходовано воды при сливе",												//-52
		"Неисправность счетчика воды!",														//-53
		"Села батарея часов",																//-54
		"Суход ход двигателя насоса",														//-55
		"Перегрузка двигателя насоса",														//-56
		"Ошибка давления",																	//-57
		"Затопление!",																		//-58
		"Пустой бак",																		//-59
		"Бак не заполняется",																//-60
		"Ошибка записи в RTC",																//-61
		"Не запускается регенерация обезжелезивателя",										//-62
		"Не запускается регенерация умягчителя",											//-63
		"Низкий уровень реагента",															//-64
		"Реагент закончился!",																//-65
		"Протечка!",																		//-66
		"Авария септика!",																	//-67
		"Нет электричества",																//-68
		"Мало израсходовано реагента во время регенерации",									//-69
		"Закончилась соль в баке умягчителя",												//-70
		"Две регенерации в одно и тоже время",												//-71
		"Регенерация обезжелезивателя долго не запускалась!",								//-72
		"Регенерация умягчителя долго не запускалась!",										//-73
		"Слишком долго работает дозатор!",													//-74
		"Утечка накопительного бака!",														//-75
		"Слишком долгое заполнение бака!",													//-76
		"Долгое непрерывное потребление воды!",												//-77
		"Ошибка связи со счетчиком насоса дренажа",											//-78
		"Слишком долго работает насос дренажа",												//-79

		"NULL"
		};

// ------------------- I2C ----------------------------------
// Устройства I2C Размер и тип памяти, определен в config.h т.к. он часто меняется
#define I2C_SPEED            twiClock400kHz // Частота работы шины I2C
#define I2C_NUM_INIT         3           // Число попыток инициализации шины
#define I2C_TIME_WAIT        2000        // Время ожидания захвата мютекса шины I2C мсек
#define I2C_ADR_RTC          0x68        // Адрес чипа rtc на шине I2C
#define I2C_ADR_DS2482       0x18        // Адрес чипа OneWire на шине I2C 3-х проводная
#define I2C_ADR_DS2482_2     0x19        // Адрес чипа OneWire на 2-ой шине I2C
#define I2C_ADR_DS2482_3     0x1A        // Адрес чипа OneWire на 3-ей шине I2C
#define I2C_ADR_DS2482_4     0x1B        // Адрес чипа OneWire на 4-ой шине I2C

// --------------------------------------------------------------------------------------------------------------------------
#ifdef  I2C_EEPROM_64KB
// Стартовые адреса -----------------------------------------------------
// КАРТА ПАМЯТИ в чипе i2c объемом 64 кбайта
// I2C_COUNT_EEPROM хранение счетчиков, максимальный размер 0x79 (127) байт. Сейчас используется  байта
// I2C_SETTING_EEPROM хранение настроек максимальный размер 0х980 (2432) байт. Сейчас используется 1095 байт
// I2C_JOURNAL_EEPROM хранение журнала размер журнала область должна быть кратна W5200_MAX_LEN
	#define I2C_COUNT_EEPROM		0x0000      // Адрес внутри чипа eeprom от куда пишется счетчики с начала чипа 0
	#define I2C_SETTING_EEPROM		0x0080      // Адрес внутри чипа eeprom от куда пишутся настройки, а перед ним пишется счетчики
	#define I2C_SETTING_EEPROM_NEXT 0x07FE
	#define DAILY_SWITCH_MAX		10			// Максимальное количество записей ежедневного включения устройств (DailySwitch). MAX=10
	#define I2C_JOURNAL_EEPROM 		I2C_SETTING_EEPROM_NEXT       // Адрес с которого начинается журнал в памяти i2c, в начале лежит признак форматирования журнала. Длина журнала JOURNAL_LEN
	#define I2C_JOURNAL_START 		(I2C_JOURNAL_EEPROM + 2)      // Адрес с которого начинается ДАННЫЕ журнал в памяти i2c ВНИМАНИЕ - 2 байт лежит признак форматирования журнала
	#define I2C_JOURNAL_EEPROM_NEXT (I2C_MEMORY_TOTAL * 1024 / 8) // Адрес после журнала = размер EEPROM
	// Журнал
	#define JOURNAL_LEN 			((I2C_JOURNAL_EEPROM_NEXT-I2C_JOURNAL_START)/W5200_MAX_LEN*W5200_MAX_LEN)// Размер журнала - округление на целое число страниц W5200_MAX_LEN
	#define I2C_JOURNAL_HEAD   		(0x01)                                                                  // Признак головы журнала
	#define I2C_JOURNAL_TAIL   		(0x02)                                                                  // Признак хвоста журнала
	#define I2C_JOURNAL_FORMAT 		(0xff)                                                                  // Символ которым заполняется журнал при форматировании
	#define I2C_JOURNAL_READY  		(0x55aa)                                                                // Признак создания журнала - если его нет по адресу I2C_JOURNAL_START-2 то надо форматировать журнал (первичная инициализация)
#else // 4 кбайта
// 0х0000 - I2C_COUNT_EEPROM хранение счетчиков, максимальный размер 0x80 (128) байт. Сейчас используется 52 байта
// 0х0080 - I2C_SETTING_EEPROM хранение настроек максимальный размер 0х77E (1918) байт.
	#define I2C_COUNT_EEPROM		0x0000      // Адрес внутри чипа eeprom от куда пишется счетчики с начала чипа 0
	#define I2C_SETTING_EEPROM		0x0080      // Адрес внутри чипа eeprom от куда пишутся настройки, а перед ним пишутся счетчики
	#define I2C_SETTING_EEPROM_NEXT 0x07FE
	#define DAILY_SWITCH_MAX		5			// Максимальное количество записей ежедневного включения устройств (DailySwitch)
	#define I2C_JOURNAL_EEPROM 		I2C_SETTING_EEPROM_NEXT       // Адрес с которого начинается журнал в памяти i2c, в начале лежит признак форматирования журнала. Длина журнала JOURNAL_LEN
	#define I2C_JOURNAL_START 		(I2C_JOURNAL_EEPROM + 2)      // Адрес с которого начинается ДАННЫЕ журнал в памяти i2c ВНИМАНИЕ - 2 байт лежит признак форматирования журнала
	#define I2C_JOURNAL_EEPROM_NEXT (I2C_MEMORY_TOTAL * 1024 / 8) // Адрес после журнала = размер EEPROM
	// Журнал
	#define JOURNAL_LEN 			((I2C_JOURNAL_EEPROM_NEXT-I2C_JOURNAL_START)/W5200_MAX_LEN*W5200_MAX_LEN)// Размер журнала - округление на целое число страниц W5200_MAX_LEN
	#define I2C_JOURNAL_HEAD   		(0x01)                                                                  // Признак головы журнала
	#define I2C_JOURNAL_TAIL   		(0x02)                                                                  // Признак хвоста журнала
	#define I2C_JOURNAL_FORMAT 		(0xff)                                                                  // Символ которым заполняется журнал при форматировании
	#define I2C_JOURNAL_READY  		(0x55aa)                                                                // Признак создания журнала - если его нет по адресу I2C_JOURNAL_START-2 то надо форматировать журнал (первичная инициализация)
	//#define I2C_JOURNAL_IN_RAM		// Журнал в ОЗУ
#endif
// Тип записи сохранения, 16bit
#define SAVE_TYPE_END			0
#define SAVE_TYPE_sTemp			-1
#define SAVE_TYPE_sADC			-2
#define SAVE_TYPE_sInput		-3
#define SAVE_TYPE_sFrequency	-4
#define SAVE_TYPE_dSDM			-5
#define SAVE_TYPE_clMQTT		-6
#define SAVE_TYPE_PwrCorr		-7
#define SAVE_TYPE_LIMIT			-100

#define RTC_STORE_ADDR RTC_ALM1_SECONDS		// Starting address of RTC store memory

// --------------------------------- ПЕРЕЧИСЛЯЕМЫЕ ТИПЫ ---------------------------------------------

//  Перечисляемый тип - режим тестирования 
enum TEST_MODE
{
   NORMAL = 0,
   STAT_TEST,
   SAFE_TEST,
   HARD_TEST           // Обязательно должен быть последним, добавляем ПЕРЕД!!!
};

// Названия режимов теста
const char *noteTestMode[] =   {"NORMAL","STAT TEST","SAFE TEST","HARD TEST"};
// Описание режима теста
static const char *noteRemarkTest[] = {"Тестирование отключено, основной режим работы.",
                                       "Значения датчиков берутся из полей 'Тест', работа исполнительных устройств эмулируется.",
                                       "Значения датчиков берутся из полей 'Тест', работа исполнительных устройств эмулируется, статистика сохраняется.",
                                       "Значения датчиков берутся из полей 'Тест', все исполнительные устройства."};


//  Перечисляемый тип - источник загрузки для веб морды
enum TYPE_SOURSE_WEB
{
 pMIN_WEB,                 // Надо грузить минимальную морду
 pSD_WEB,                  // Надо грузить морду с карты 
 pFLASH_WEB,               // Надо грузить морду с флеш диска
 pERR_WEB                  // Внутренняя ошибка? 
};


//  Перечисляемый тип - Типы ответов POST запросов
enum TYPE_RET_POST
{
 pSETTINGS_OK,                // Настройки из выбранного файла восстановлены, CRC16 OK 
 pSETTINGS_ERR,               // Ошибка восстановления настроек из файла (см. журнал) 
 pNULL,                       // "" - пустая строка
 pLOAD_OK,                    // Файлы загружены, подробности смотри в журнале 
 pLOAD_ERR,                   // Ошибка загрузки файла, подробности смотри в журнале      
 pPOST_ERR,                   // Внутреняя ошибка парсера post запросов 
 pNO_DISK,                    // Нет флеш диска
 pSETTINGS_MEM                // Настройки не влезают во внутренний буфер
};

//  Перечисляемый тип -ТИПЫ уведомлений
enum MESSAGE          
{
  pMESSAGE_NONE,                 // 0 Нет уведомлений
  pMESSAGE_TESTMAIL,             // 1 Тестовое уведомление по почте +
  pMESSAGE_TESTSMS,              // 2 Тестовое уведомление по SMS +
  pMESSAGE_RESET,                // 3 Уведомление Сброс +
  pMESSAGE_ERROR,                // 4 Уведомление Ошибка +
  pMESSAGE_LIFE,                 // 5 Уведомление Сигнал жизни +
  pMESSAGE_TEMP,                 // 6 Уведомление Достижение граничной температуры +
  pMESSAGE_SD,                   // 7 Уведомление "Проблемы с sd картой" +
  pMESSAGE_WARNING,              // 8 Уведомление "Прочие уведомления"
  pEND10                         // Обязательно должен быть последним, добавляем ПЕРЕД!!!
};

//  Перечисляемый тип - сервис для отправки смс
enum SMS_SERVICE 
{ 
  pSMS_RU,                       // Сервис sms.ru
  pSMSC_RU,                      // Сервис smsc.ru
  pSMSC_UA,                      // Сервис smsc.ua
  pSMSCLUB                    // Сервис smsclub.mobi
};

const char ADR_SMS_RU[]  = "sms.ru";
const char ADR_SMSC_RU[] = "smsc.ru";
const char ADR_SMSC_UA[] = "smsc.ua";
const char ADR_SMSCLUB[] = "gate.smsclub.mobi";
const char SMS_SERVICE_WEB_SELECT[] = "sms.ru:0;smsc.ru:0;smsc.ua:0;smsclub.mobi:0;";

//  Перечисляемый тип - Время сброса сокетов
enum TIME_RES_SOCKET       
{
    pNONE1,           // нет сброса
    p30sec,           // сброс раз в 30 секунд
    p300sec,          // сброс раз в 300 секунд
    pEND4             // Обязательно должен быть последним, добавляем ПЕРЕД!!!
};

#endif
