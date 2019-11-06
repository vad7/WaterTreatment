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
// Описание базовых классов для работы c "железом"
// (датчики и исполнительные устройства) зависит от контроллера
// --------------------------------------------------------------------------------
#ifndef Hardware_h
#define Hardware_h
#include "Constant.h"                       // Вся конфигурация и константы проекта Должен быть первым !!!!
#include <rtc_clock.h>                      // работа со встроенными часами  Это основные часы!!!
#include <ModbusMaster.h>                   // Используется МОДИФИЦИРОВАННАЯ либа ModbusMaster https://github.com/4-20ma/ModbusMaster
#include "Information.h"
#include "devOneWire.h" 

// Флаги датчиков (единые для всех датчиков!!!!!)
#define fPresent      0               // флаг наличие датчика
#define fTest         1               // флаг режим теста
#define fFull         2               // флаг полного буфера для усреднения
#define fAddress      3               // флаг правильного адреса для температурного датчика
#define fcheckRange	  4				  // флаг Проверка граничного значения
#define fsensModbus	  5				  // флаг дистанционного датчика по Modbus
#define fRadio	      6				  // флаг радио-датчика

extern RTC_clock rtcSAM3X8;
extern int8_t set_Error(int8_t err, char *nam);

// ------------------------------------------------------------------------------------------
// Д А Т Ч И К И  ---------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
#include "DigitalTemp.h"

// ------------------------------------------------------------------------------------------
// Класс аналоговый датчик управление токовая петля + нагрузочный резистор + делитель
// Может быть несколько датчиков
// Давление хранится в СОТЫХ БАРА

#define ERROR_PRESS	-32768

class sensorADC
{
  public:
    void initSensorADC(uint8_t sensor, uint8_t pinA, uint16_t filter_size); // Инициализация датчика  порядковый номер датчика и нога он куда прикреплен
    int8_t  Read();                                      // чтение данных c аналогового датчика давления (АЦП) возвращает код ошибки, делает все преобразования
    //int16_t Test();                                      // полный цикл получения данных возвращает значение давления, только тестирование!! никакие переменные класса не трогает!!
    __attribute__((always_inline)) inline int16_t get_minPress(){return cfg.minPress;}     // Минимальная давление датчика - нижняя граница диапазона, при выходе из него ошибка
    __attribute__((always_inline)) inline int16_t get_minPressReg(){return cfg.minPressReg;}     // Минимальная давление датчика - нижняя граница диапазона, при выходе из него ошибка
    __attribute__((always_inline)) inline int16_t get_maxPress(){return cfg.maxPress;}     // Максимальная давление датчика - верхняя граница диапазона, при выходе из него ошибка
    int16_t get_zeroPress(){return cfg.zeroPress;}           // Выход датчика (отсчеты ацп)  соответсвующий 0
    int8_t  set_zeroPress(int16_t p);                    // Установка Выход датчика (отсчеты ацп)  соответсвующий 0
    uint16_t get_lastADC(){ return lastADC; }            // Последнее считанное значение датчика в отсчетах с фильтром
    __attribute__((always_inline)) inline int16_t get_Press() { return Press; };                                 // Получить значение давления датчика - это то что используется
    uint16_t get_transADC(){return cfg.transADC;}        // Получить значение коэффициента преобразования напряжение-температура
    int8_t set_transADC(float p);                        // Установить значение коэффициента преобразования напряжение-температура
    __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fPresent);} // Наличие датчика в текущей конфигурации
    __attribute__((always_inline)) inline boolean get_fmodbus(){return GETBIT(flags,fsensModbus);} // Подключен по Modbus
    int8_t  get_lastErr(){return err;}                   // Получить последнюю ошибку
    inline uint8_t  get_pinA(){return pin;}               // Получить канал АЦП (нумерация SAM3X) куда прицеплен датчик
    int16_t get_testPress(){return cfg.testPress;}           // Получить значение давления датчика в режиме теста
    int8_t  set_testPress(int16_t p);                    // Установить значение давления датчика в режиме теста
    int8_t  set_minPress(int16_t p) { cfg.minPress = p; return OK; }
    int8_t  set_minPressReg(int16_t p) { cfg.minPressReg = p; return OK; }
    int8_t  set_maxPress(int16_t p)  { cfg.maxPress = p; return OK; }
    void    set_testMode(TEST_MODE t){testMode=t;}       // Установить значение текущий режим работы
     
    char*   get_note(){return note;}                     // Получить наименование датчика
    char*   get_name(){return name;}                     // Получить имя датчика
    uint8_t *get_save_addr(void) { return (uint8_t *)&cfg; } // Адрес структуры сохранения
    uint16_t get_save_size(void) { return sizeof(cfg); } // Размер структуры сохранения
    void	after_load(void);
   
    statChart Chart;                                      // График по датчику
    //type_rawADC adc;                                      // структура для хранения сырых данных с АЦП
    uint32_t adc_sum;                          			// сумма
    uint16_t *adc_filter;              						// массив накопленных значений
    uint16_t adc_filter_max;
    uint16_t adc_last;       			                // текущий индекс
    boolean  adc_flagFull;              			    // буфер полный
    uint16_t adc_lastVal;                      			// последнее считанное значение
    //int16_t  Temp;										// Температура в сотых
    
  private:
    int16_t Press;                                       // давление датчика (обработанное) в сотых бара
    struct {     // Save GROUP, firth number
		uint8_t number;										 // Номер
		int16_t zeroPress;                                   // отсчеты АЦП при нуле датчика
		union {
			float transADC_f;
			struct {
				uint16_t transADC;                           // коэффициент пересчета АЦП в давление, тысячные
				uint16_t _reserved_2;
			} __attribute__((packed));
		};
		int16_t testPress;                                   // давление датчика в режиме тестирования
		int16_t minPress;                                    // минимальное давление
		int16_t maxPress;                                    // максимальное давление
		int16_t minPressReg;								 // минимальное давление при регенерации
    } __attribute__((packed)) cfg;// Save Group end
    TEST_MODE testMode;                                  // Значение режима тестирования
    uint16_t lastADC;                                    // Последние значение отсчета ацп
       
    uint8_t pin;                                         // Ножка куда прицеплен датчик
    int8_t  err;                                         // ошибка датчика (работа)
    byte 	flags;                                       // флаги  датчика
    // Кольцевой буфер для усреднения
    void clearBuffer();                                  // очистить буфер
#if P_NUMSAMLES > 1
    int16_t p[P_NUMSAMLES];                              // буфер для усреднения показаний давления
    int32_t sum;                                         // Накопленная сумма
    uint8_t last;                                        // указатель на последнее (самое старое) значение в буфере диапазон от 0 до P_NUMSAMLES-1
#endif
    char *note;                                          // Описание датчика
    char *name;                                          // Имя датчика
};

// ------------------------------------------------------------------------------------------
// Цифровые контактные датчики (есть 2 состяния 0 и 1) --------------------------------------
class sensorDiditalInput
{
public:
  void initInput(int sensor);                            // Инициализация контактного датчика
  int8_t  Read(boolean fast = false);                    // Чтение датчика возвращает ошибку или ОК
  __attribute__((always_inline)) inline boolean get_Input(){return Input;}   // Получить значение датчика при последнем чтении
  __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fPresent);} // Наличие датчика в текущей конфигурации
  int8_t  get_lastErr(){return err;}                     // Получить последнюю ошибку
  char*   get_note(){return note;}                       // Получить наименование датчика
  char*   get_name(){return name;}                       // Получить имя датчика
  boolean get_testInput(){return testInput;}             // Получить Состояние датчика в режиме теста
  int8_t  set_testInput(int16_t i);                      // Установить Состояние датчика в режиме теста
  void    set_testMode(TEST_MODE t){testMode=t;}       // Установить значение текущий режим работы
  boolean get_alarmInput(){return InputLevel;}           // Состояние аварии датчика
  boolean is_alarm() { return Input == InputLevel; }	// Датчик сработал?
  int8_t  set_alarmInput(int16_t i);                     // Установить Состояние аварии датчика
  inline int8_t  get_pinD(){return pin;}                 // Получить ногу куда прицеплен датчик
  TYPE_SENSOR get_typeInput(){return type;}              // Получить тип датчика
  uint8_t *get_save_addr(void) { return (uint8_t *)&number; } // Адрес структуры сохранения
  uint16_t get_save_size(void) { return (byte*)&InputLevel - (byte*)&number + sizeof(InputLevel); } // Размер структуры сохранения
    
private:
   boolean Input;                                        // Состояние датчика
   struct { // Save GROUP, firth number
   uint8_t number;										 // номер
   boolean testInput;                                    // !save! Состояние датчика в режиме теста
   boolean InputLevel;                                   // !save! Состояние сработавшего датчика
   } __attribute__((packed));// Save Group end, last alarmInput
   TYPE_SENSOR type;                                     // Тип датчика
   int8_t err;                                           // ошибка датчика (работа)
   byte flags;                                           // флаги  датчика
   TEST_MODE testMode;                                  // Значение режима тестирования
   uint8_t  pin;                                         // Ножка куда прицеплен датчик
   char *note;                                           // наименование датчика
   char *name;                                           // Имя датчика
};
// ------------------------------------------------------------------------------------------
// Цифровые частотные датчики (значение кодируется в выходной частоте) ----------------------
// основное назначение - датчики потока
// Частота кодируется в тысячных герца
// Число импульсов рассчитывается за базовый период (BASE_TIME_READ), т.к частоты малы период надо савить не менее 5 сек
// Выходная величина кодируется в тысячных от целого

class sensorFrequency
{
public:
  void initFrequency(int sensor);                   // Инициализация частотного датчика
  void reset(void);									// Сброс счетчика
  __attribute__((always_inline)) inline void InterruptHandler(){count++;} // обработчик прерываний
  int8_t  Read();                                         // Чтение датчика (точнее расчет значения) возвращает ошибку или ОК
  __attribute__((always_inline)) inline uint32_t get_Frequency(){return Frequency;}   // Получить ЧАСТОТУ датчика при последнем чтении
  __attribute__((always_inline)) inline uint16_t get_Value(){return Value;}           // Получить Значение датчика при последнем чтении, литры в час
  __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fPresent);} // Наличие датчика в текущей конфигурации
  __attribute__((always_inline)) inline uint16_t get_minValue(){return minValue * 100;}     // Получить минимальное значение датчика, литры в час
  void set_minValue(float f);							// Установить минимальное значение датчика
  __attribute__((always_inline)) inline boolean get_checkFlow(){return GETBIT(flags,fcheckRange);}// Проверка граничного значения
  void set_checkFlow(boolean f) { flags = (flags & ~(1<<fcheckRange)) | (f<<fcheckRange); }
  int8_t  get_lastErr(){return err;}                     // Получить последнюю ошибку
  char*   get_note(){return note;}                       // Получить описание датчика
  char*   get_name(){return name;}                       // Получить имя датчика
  uint16_t get_testValue(){return testValue;}            // Получить Состояние датчика в режиме теста
  int8_t  set_testValue(int16_t i);                      // Установить Состояние датчика в режиме теста
  void    set_testMode(TEST_MODE t){testMode=t;}         // Установить значение текущий режим работы
  inline uint16_t get_kfValue(){return kfValue;}         // Получить коэффициент пересчета
  void    set_kfValue(uint16_t f) { kfValue=f; }         // Установить коэффициент пересчета
  int8_t set_Capacity(uint16_t c);                       // Установить теплоемкость больше 5000 не устанавливается
  inline int8_t  get_pinF(){return pin;}                 // Получить ногу куда прицеплен датчик
  uint8_t *get_save_addr(void) { return (uint8_t *)&number; } // Адрес структуры сохранения
  uint16_t get_save_size(void) { return (byte*)&minValue - (byte*)&number + sizeof(minValue); } // Размер структуры сохранения
  statChart Chart;                                       // Статистика по датчику
  volatile uint32_t Passed;										 // Счетчик литров
  uint32_t PassedRest;									 // остаток счетчика
    
private:
   uint32_t Frequency;                                   // значение частоты в тысячных герца
   uint32_t Value;                                       // значение датчика ЛИТРЫ В ЧАС (ИЛИ ТЫСЯЧНЫЕ КУБА)
   struct { // SAVE GROUP, number the first
	   uint8_t  number;										 // номер
	   uint16_t testValue;                                   // !save! Состояние датчика в режиме теста
	   uint16_t kfValue; 								 	 // коэффициент пересчета частоты в значение, сотые
	   uint8_t  flags;                                       // флаги  датчика
	   uint8_t  minValue;							     	 // десятые m3/h (0..25,5)
   } __attribute__((packed));// END SAVE GROUP, minValue the last
   volatile uint32_t count;                              // число импульсов за базовый период (то что меняется в прерывании)
   TEST_MODE testMode;                                  // Значение режима тестирования
   uint32_t sTime;                                       // время начала базового периода в тиках
   int8_t err;                                           // ошибка датчика (работа)
   uint8_t  pin;                                         // Ножка куда прицеплен датчик
   char *note;                                           // наименование датчика
   char *name;                                           // Имя датчика
};




// ------------------------------------------------------------------------------------------
// И С П О Л Н И Т Е Л Ь Н Ы Е   У С Т Р О Й С Т В А   --------------------------------------
// ------------------------------------------------------------------------------------------
#define fR_StatusMain		1			// b1: Состояние Вкл/Выкл основного алгоритма
#define fR_StatusSun		2			// b2: Состояние Вкл/Выкл Солнечного Коллектора
#define fR_StatusManual		3			// b3: Состояние Вкл/Выкл Солнечного Коллектора
#define fR_StatusMask		((1<<fR_StatusMain)|(1<<fR_StatusSun)|(1<<fR_StatusManual))	// битовая маска
#define fR_StatusAllOff		-127		// выключить по всем алгоритмам

class devRelay
{
public:
  void initRelay(int sensor);                            // Инициализация реле
  __attribute__((always_inline)) inline int8_t  set_ON() {return set_Relay(fR_StatusMain);}    // Включить реле
  __attribute__((always_inline)) inline int8_t  set_OFF(){return set_Relay(-fR_StatusMain);}   // Выключить реле
  int8_t  set_Relay(int8_t r);                           // Установить реле в состояние (0/-1 - выкл основной алгоритм, fR_Status* - включить, -fR_Status* - выключить)
  __attribute__((always_inline)) inline boolean get_Relay(){return Relay;}                    // Прочитать состояние реле
  int8_t  get_pinD(){return pin;}                        // Получить ногу куда прицеплено реле
  char*   get_note(){return note;}                       // Получить наименование реле
  char*   get_name(){return name;}                       // Получить имя реле
  __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fPresent);} // Наличие датчика в текущей конфигурации
  void    set_testMode(TEST_MODE t){testMode=t;}       // Установить значение текущий режим работы
  byte flags;                                           // флаги  0 - наличие реле, 1.. - fR_Status*
private:
   uint8_t number;										// Номер массива реле
   boolean Relay;                                        // Состояние реле
   uint8_t  pin;                                         // Ножка куда прицеплено реле
   TEST_MODE testMode;                                  // Значение режима тестирования
   char *note;                                           // наименование реле
   char *name;                                           // Имя реле
};  

// Класс Электрический счетчик -----------------------------------------------------------------------------------------------
const char *namePWM = {"PWM"};
const char *notePWM = {"Электрический счетчик"};       // Описание счетчика

// Флаги Электросчетчика
#define fPWM           0              // флаг наличие счетчика
#define fPWMLink       1              //  флаг связь установлена

// PZEM-004T Modbus (UART)
// Read Input register, Function code 04:
#define PWM_VOLTAGE          0x0000			// int16, 0.1V
#define PWM_CURRENT          0x0001			// int32, 0.001A
#define PWM_POWER            0x0003			// int32, 0.1W
#define PWM_ENERGY           0x0005			// int32, 1Wh
#define PWM_FREQ             0x0007			// int16, 0.1Hz
#define PWM_PFACTOR          0x0008			// int16, 0.01
#define PWM_ALARM            0x0009			// 0xFFFF - Alarm, 0 - ok
// Holding Registers
#define PWM_ALARM_THRESHOLD  0x0001			// 1W
#define PWM_MODBUS_ADDR      0x0002			// 1..F7
// Special command
#define PWM_RESET_ENERGY	 0x42

class devPWM
{
   public:  
       int8_t initPWM();                               // Инициализация счетчика и проверка и если надо программирование

      int8_t  get_readState(uint8_t group);            // Прочитать инфо с счетчика
      int8_t  get_lastErr(){ return err; }               // Получить последнюю ошибку счетчика
      uint16_t get_numErr(){ return numErr; }            // Получить число ошибок чтения счетчика
      char*   get_note(){ return note; }                 // Получить описание датчика
      char*   get_name(){ return name; }                 // Получить имя датчика
      int32_t get_Power(){ return Power; }
      uint16_t get_Voltage(){ return Voltage; }

      char* get_param(char *var, char *ret);           // Получить параметр PWM в виде строки
      boolean set_param(char *var, float f);
      
       // Графики из счетчика
      statChart ChartVoltage;                          // Статистика по напряжению
      statChart ChartPower;                            // Статистика по Полная мощность
  private:
      int8_t  err;                                     // ошибка стесчика (работа)
      uint16_t numErr;                                 // число ошибок чтение по модбасу
      byte flags;                                      // флаги  0 - наличие счетчика,
      
      uint16_t Voltage;                                // 0.1V
      uint32_t Power;                                  // Моментальная мощность, 0.1W
      uint32_t TestPower;

      char *note;                                      // Описание
      char *name;                                      // Имя
};


// Класс устройство Модбас  -----------------------------------------------------------------------------------------------
#define fModbus    			0               // флаг наличие modbus
class devModbus
  {
  public:  
    int8_t initModbus();                                                                // Инициализация Modbus и проверка связи возвращает ошибку
     __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fModbus);} // Наличие Modbus в текущей конфигурации
    int8_t readInputRegisters16(uint8_t id, uint16_t cmd, uint16_t *ret);
    int8_t readInputRegisters32(uint8_t id, uint16_t cmd, uint32_t *ret);				   // LITTLE-ENDIAN!
    int8_t readInputRegistersFloat(uint8_t id, uint16_t cmd, float *ret);                  // Получить значение 2-x (Modbus function 0x04 Read Input Registers) регистров (4 байта) в виде float возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegisters16(uint8_t id, uint16_t cmd, uint16_t *ret);                // Получить значение регистра (2 байта) в виде целого  числа возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegisters32(uint8_t id, uint16_t cmd, uint32_t *ret);                // BIG-ENDIAN! Получить значение 2-x регистров (4 байта) в виде целого числа возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegistersFloat(uint8_t id, uint16_t cmd, float *ret);                // Получить значение 2-x регистров (4 байта) в виде float возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegistersNN(uint8_t id, uint16_t cmd, uint16_t num,uint16_t *buf);   // Получить значение N регистров (2*N байта) (положить в buf) возвращает код ошибки
    int8_t writeSingleCoil(uint8_t id,uint16_t cmd, uint8_t u8State);                      // установить битовый вход, возвращает код ошибки Modbus function 0x05 Write Single Coil.
    int8_t readCoil(uint8_t id,uint16_t cmd, boolean *ret);                                // прочитать отдельный бит, возвращает ошибку Modbus function 0x01 Read Coils.
    int8_t writeHoldingRegisters16(uint8_t id, uint16_t cmd, uint16_t data);               // Установить значение регистра (2 байта) МХ2 в виде целого  числа возвращает код ошибки данные data
    int8_t writeHoldingRegistersFloat(uint8_t id, uint16_t cmd, float dat);                // Записать float как 2 регистра числа возвращает код ошибки данные data
    int8_t writeHoldingRegisters32(uint8_t id,uint16_t cmd, uint32_t data); 			   // BIG-ENDIAN! Записать 2 регистра подряд возвращает код ошибки
    int8_t get_err() {return err;}                                                         // Получить код ошибки
    ModbusMaster RS485;                     // Класс модбас 485
private:
    // Переменные
    int8_t flags;                           // Флаги
    int8_t err;                             // Ошибки модбас
    int8_t translateErr(uint8_t result);    // Перевод ошибки протокола Модбас
  }; // End class

 #endif
