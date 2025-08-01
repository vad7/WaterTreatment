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
    __attribute__((always_inline)) inline int16_t get_minValue(){return cfg.minValue;}     // Минимальное значение датчика - нижняя граница диапазона
    __attribute__((always_inline)) inline int16_t get_maxValue(){return cfg.maxValue;}     // Максимальноедавление датчика - верхняя граница диапазона
    int16_t get_zeroValue(){return cfg.zeroValue;}           // Выход датчика (отсчеты ацп)  соответсвующий 0
    int8_t  set_zeroValue(int16_t p);                    // Установка Выход датчика (отсчеты ацп)  соответсвующий 0
    uint16_t get_lastADC(){ return lastADC; }            // Последнее считанное значение датчика в отсчетах с фильтром
    __attribute__((always_inline)) inline int16_t get_Value() { return Value; };  // Получить значение датчика, сотые
    uint16_t get_transADC(){return cfg.transADC;}        // Получить значение коэффициента преобразования
    int8_t set_transADC(float p);                        // Установить значение коэффициента преобразования
    __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fPresent);} // Наличие датчика в текущей конфигурации
    __attribute__((always_inline)) inline boolean get_fmodbus(){return GETBIT(flags,fsensModbus);} // Подключен по Modbus
    int8_t  get_lastErr(){return err;}                   // Получить последнюю ошибку
    inline uint8_t  get_pinA(){return pin;}               // Получить канал АЦП (нумерация SAM3X) куда прицеплен датчик
    int16_t get_testValue(){return cfg.testValue;}           // Получить значение датчика в режиме теста
    int8_t  set_testValue(int16_t p);                    // Установить значение датчика в режиме теста
    int8_t  set_minValue(int16_t p) { if(p < cfg.maxValue) { cfg.minValue = p; return OK; } else return -1; }
    int8_t  set_maxValue(int16_t p)  { if(p > cfg.minValue) { cfg.maxValue = p; return OK; } else return -1; }
    void    set_testMode(TEST_MODE t){testMode=t;}       // Установить значение текущий режим работы
    inline uint8_t get_ADC_Gain() { return cfg.ADC_Gain; }
    void set_ADC_Gain(uint8_t gain);
     
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
    int16_t Value;                                       // Значение датчика (обработанное) в сотых (бара или др. единиц)
    struct {     // Save GROUP, firth number
		uint8_t number;									 // Номер
		int16_t zeroValue;                               // отсчеты АЦП при нуле датчика
		uint16_t transADC;                               // коэффициент пересчета значения АЦП в значение, тысячные
		uint8_t ADC_Gain;								 // Усиление(1..3)
		uint8_t Reserved;
		int16_t testValue;                               // значение датчика в режиме тестирования, сотые
		int16_t minValue;                                // минимальное значение, сотые
		int16_t maxValue;                                // максимальное значение, сотые
    } __attribute__((packed)) cfg;// Save Group end
    TEST_MODE testMode;                                  // Значение режима тестирования
    uint16_t lastADC;                                    // Последние значение отсчета ацп
       
    uint8_t pin;                                         // Канал ADC
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

#ifdef TNTC
uint16_t TNTC_Value[TNTC];
#define  TNTC_Value_Max 4090
#endif
#ifdef TNTC_EXT
uint16_t TNTC_EXT_Value[TNTC_EXT];
#endif

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
  boolean get_InputLevel(){return InputLevel;}           // Уровень сработавшего датчика
  int8_t  set_InputLevel(int16_t i);                     // Установить уровень сработавшего датчика
  inline int8_t  get_pinD(){return pin;}                 // Получить ногу куда прицеплен датчик
  int8_t  get_alarm_error(){return alarm_error;}
  uint8_t *get_save_addr(void) { return (uint8_t *)&number; } // Адрес структуры сохранения
  uint16_t get_save_size(void) { return (byte*)&InputLevel - (byte*)&number + sizeof(InputLevel); } // Размер структуры сохранения
    
private:
   boolean Input;                                        // Состояние датчика
   struct { // Save GROUP, firth number
   uint8_t number;										 // номер
   boolean testInput;                                    // !save! Состояние датчика в режиме теста
   boolean InputLevel;                                   // !save! Состояние сработавшего датчика
   } __attribute__((packed));// Save Group end, last alarmInput
   int8_t alarm_error;                                   // ошибка при срабатывании датчика, 0 - нет
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
  bool  Read(void);				               // Чтение датчика (точнее расчет значения), возвращает был проток или нет, включая добавку
  __attribute__((always_inline)) inline uint32_t get_Frequency(){return Frequency;} // Получить ЧАСТОТУ датчика, тысячных герца
  __attribute__((always_inline)) inline uint16_t get_Value(){return Value;} // Получить значение, литры в час
  __attribute__((always_inline)) inline uint16_t get_ValueReal(){return ValueReal;} // Получить реальное значение, литры в час
  __attribute__((always_inline)) inline boolean get_present(){return kfValue > 0;} // Наличие датчика в текущей конфигурации
#ifdef SENSORS_FREQ_I2C
  uint32_t get_RawPassed(void) { return I2C_addr ? 0 : count * 10000 / kfValue; }  // Получить сырые не обработанные данные, сотые литра
  uint8_t get_I2C_addr(void) { return I2C_addr; };
  void set_I2C_addr(uint8_t addr);
  //__attribute__((always_inline)) inline uint32_t get_count(void) { return I2C_addr ? 0 : count; }
#else
  uint32_t get_RawPassed(void) { return count * 10000 / kfValue; }  // Получить сырые не обработанные данные, сотые литра
#endif
//  int8_t  get_lastErr(){return err;}                     // Получить последнюю ошибку
  char*   get_note(){return note;}                       // Получить описание датчика
  char*   get_name(){return name;}                       // Получить имя датчика
  uint16_t get_testValue(){return testValue;}            // Получить Состояние датчика в режиме теста
  int8_t  set_testValue(int16_t i);                      // Установить Состояние датчика в режиме теста
  inline  uint32_t get_kfValue(){return kfValue;}        // Получить коэффициент пересчета, сотые
  void    set_kfValue(uint32_t f) { kfValue=f; }         // Установить коэффициент пересчета, сотые
  inline  int16_t get_kNLValue(){return kNonLinearity;}  // Получить коэффициент пересчета, сотые
  void    set_kNLValue(int16_t f) { kNonLinearity = f; } // Установить коэффициент пересчета, сотые
  uint8_t get_FlowCalcPeriodValue(){return FlowCalcPeriod;}
  void    set_FlowCalcPeriodValue(uint8_t n) { FlowCalcPeriod = n; }
  int8_t  set_Capacity(uint16_t c);                      // Установить теплоемкость больше 5000 не устанавливается
  inline  int8_t  get_pinF(){return pin;}                // Получить ногу куда прицеплен датчик
  uint8_t *get_save_addr(void) { return (uint8_t *)&number; } // Адрес структуры сохранения
#ifdef SENSORS_FREQ_I2C
  uint16_t get_save_size(void) { return (byte*)&I2C_addr - (byte*)&number + sizeof(I2C_addr); } // Размер структуры сохранения
#else
  uint16_t get_save_size(void) { return (byte*)&kNonLinearity - (byte*)&number + sizeof(kNonLinearity); } // Размер структуры сохранения
#endif
  volatile uint32_t Passed;								 // Счетчик литров
  uint32_t PassedRest;									 // остаток счетчика
  int32_t  add_pulses100;								 // добавка при следующем чтении, *100
  uint32_t count_real_last100;							// счетчик без добавок, последний, *100
  uint32_t FlowPulseCounter;							// real pulses * 100
  uint8_t  WebCorrectCnt;								// счетчик корректировки для веба, *TIME_READ_SENSOR, начиная с 1
  statChart ChartFlow;                                   // Статистика по датчику
#ifdef F_CHART_ChartLiters
  statChart ChartLiters;                                 // Статистика по датчику
  uint16_t ChartLiters_accum;
  uint16_t ChartLiters_rest;
#endif

#ifdef SENSORS_FREQ_I2C
  uint8_t err;
  uint16_t errNum;
#endif
    
private:
  volatile uint32_t count;                              // число импульсов за базовый период (то что меняется в прерывании)
  uint32_t sTime;                                       // время начала базового периода в тиках
  uint32_t count_Flow;									// буфер для расчета протока и частоты
  uint32_t count_FlowReal;								// буфер для расчета протока и частоты
  uint32_t Frequency;                                   // значение частоты в тысячных герца
  uint16_t Value;                                       // расчетное значение датчика c учетом корректировок, ЛИТРЫ В ЧАС (ИЛИ ТЫСЯЧНЫЕ КУБА)
  uint16_t ValueReal;                                   // реальное значение датчика, ЛИТРЫ В ЧАС (ИЛИ ТЫСЯЧНЫЕ КУБА)
  char *note;                                           // наименование датчика
  char *name;                                           // Имя датчика
  struct { // SAVE GROUP, number the first
      uint8_t  number;									 // номер
      uint16_t testValue;                               // !save! Состояние датчика в режиме теста
      uint32_t kfValue; 							 	 // коэффициент пересчета частоты в значение, изменений уровня на литр, сотые
      uint8_t  FlowCalcPeriod;                          // через сколько FREQ_BASE_TIME_READ расчитывать проток
      int16_t  kNonLinearity;							// Коэффициент нелинейности (0 - нет), сотые
#ifdef SENSORS_FREQ_I2C
      uint8_t  I2C_addr;						     	// адрес на I2C шине, если используется для получения данных датчика, формат данных (3 байта): импульсы(0..65535),CRC8(1-Wire)
#endif
  } __attribute__((packed));// END SAVE GROUP, the last
  uint8_t FlowCalcCnt;                                  // счетчик расчета протока
  uint8_t pin;                                         // Ножка куда прицеплен датчик
};

// ------------------------------------------------------------------------------------------
// И С П О Л Н И Т Е Л Ь Н Ы Е   У С Т Р О Й С Т В А   --------------------------------------
// ------------------------------------------------------------------------------------------
#define fR_StatusMain		1			// b1: Состояние Вкл/Выкл основного алгоритма
#define fR_StatusSun		2			// b2: Состояние Вкл/Выкл Солнечного Коллектора
#define fR_StatusManual		3			// b3: Состояние Вкл/Выкл Солнечного Коллектора
#define fR_StatusDaily		4			// b3: Состояние Вкл/Выкл Дневное включение
#define fR_StatusMask		((1<<fR_StatusMain)|(1<<fR_StatusSun)|(1<<fR_StatusManual)|(1<<fR_StatusDaily))	// битовая маска
#define fR_StatusAllOff		-127		// выключить по всем алгоритмам

class devRelay
{
public:
  void    initRelay(int sensor);                            // Инициализация реле
  __attribute__((always_inline)) inline int8_t  set_ON() {return set_Relay(fR_StatusMain);}    // Включить реле
  __attribute__((always_inline)) inline int8_t  set_OFF(){return set_Relay(-fR_StatusMain);}   // Выключить реле
  int8_t  set_Relay(int8_t r);                           // Установить реле в состояние (0/-1 - выкл основной алгоритм, fR_Status* - включить, -fR_Status* - выключить)
  __attribute__((always_inline)) inline boolean get_Relay(){return Relay;}                    // Прочитать состояние реле
  __attribute__((always_inline)) inline boolean get_RelayTimerOn(){ return TimerOn ? true : Relay; } // Прочитать состояние реле
  __attribute__((always_inline)) inline uint32_t get_TimerOn(){ return TimerOn; }
  int8_t  get_pinD(){return pin;}                        // Получить ногу куда прицеплено реле
  char*   get_note(){return note;}                       // Получить наименование реле
  char*   get_name(){return name;}                       // Получить имя реле
  __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fPresent);} // Наличие датчика в текущей конфигурации
  void    set_testMode(TEST_MODE t){testMode=t;}       // Установить значение текущий режим работы
  void    NextTimerOn(void);
  byte flags;                                           // флаги  0 - наличие реле, 1.. - fR_Status*
private:
   uint8_t  number;										// Номер массива реле
   boolean  Relay;                                        // Состояние реле
   uint8_t  pin;                                         // Ножка куда прицеплено реле
   uint32_t TimerOn;									// мсек, Время показа активного состояния
   TEST_MODE testMode;                                  // Значение режима тестирования
   char *note;                                           // наименование реле
   char *name;                                           // Имя реле
};  

// Класс Электрический счетчик -----------------------------------------------------------------------------------------------
const char *namePWM = {"PZEM-004T"};
const char *notePWM = {PWM_NAME};       // Описание счетчика

// Флаги Электросчетчика
#define fPWM           0              // флаг наличие счетчика
#define fPWMLink       1              //  флаг связь установлена

// PZEM-004T v3 (PZEM-014, PZEM-016) Modbus (UART)
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
#define PWM_MODBUS_ADDR      0x0002			// 1..F7 (F8 - fixed)
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
      boolean set_param(char *var, int32_t f);
      void get_param_now(uint8_t modbus_addr, char var, char *strReturn); // Запросить регистр счетчика сейчас, для веб
      
       // Графики из счетчика
      //statChart ChartVoltage;                          // Статистика по напряжению
      statChart ChartPower;                            // Статистика по Полная мощность
  private:
      int8_t  err;                                     // ошибка стесчика (работа)
      uint16_t numErr;                                 // число ошибок чтение по модбасу
      byte flags;                                      // флаги  0 - наличие счетчика,
      
      uint16_t Voltage;                                // 0.1V
      uint32_t Power;                                  // Моментальная мощность, 1W
      uint32_t TestPower;

      char *note;                                      // Описание
      char *name;                                      // Имя
};


// Класс устройство Модбас  -----------------------------------------------------------------------------------------------
//flags:
#define fModbus    			0               // флаг наличие modbus
#define fModbus_Serial2		1				// Следующая команда: 0 - Serial1, 1 - Serial2
#define MODBUS_SET_SERIAL_AND_ID RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : id >= MODBUS_SERIAL3_ADDR_GE ? MODBUS_SERIAL3 : MODBUS_SERIAL2)

class devModbus
  {
  public:  
    int8_t initModbus();                             								        // Инициализация Modbus
     __attribute__((always_inline)) inline boolean get_present(){return GETBIT(flags,fModbus);} // Наличие Modbus в текущей конфигурации
    int8_t readInputRegisters16(uint8_t id, uint16_t cmd, uint16_t *ret);
    int8_t readInputRegisters32(uint8_t id, uint16_t cmd, uint32_t *ret);				   // LITTLE-ENDIAN!
    int8_t readInputRegistersFloat(uint8_t id, uint16_t cmd, float *ret);                  // Получить значение 2-x (Modbus function 0x04 Read Input Registers) регистров (4 байта) в виде float возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegisters16(uint8_t id, uint16_t cmd, uint16_t *ret);                // Получить значение регистра (2 байта) в виде целого  числа возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegisters32(uint8_t id, uint16_t cmd, uint32_t *ret);                // BIG-ENDIAN! Получить значение 2-x регистров (4 байта) в виде целого числа возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegistersFloat(uint8_t id, uint16_t cmd, float *ret);                // Получить значение 2-x регистров (4 байта) в виде float возвращает код ошибки данные кладутся в ret
    int8_t readHoldingRegistersNN(uint8_t id, uint16_t cmd, uint16_t num,uint16_t *buf);   // Получить значение N регистров (2*N байта) (положить в buf) возвращает код ошибки
    int8_t writeSingleCoil(uint8_t id,uint16_t cmd, uint8_t u8State);                      // установить битовый вход, возвращает код ошибки Modbus function 0x05 Write Single Coil.
    int8_t readCoils(uint8_t id,uint16_t cmd, uint8_t *num_ret);                           // прочитать Coils, возвращает ошибку Modbus function 0x01 Read Coils.
    int8_t readDiscreteInputs(uint8_t id, uint16_t cmd, uint8_t *num_ret);					// прочитать дискретные входы
    int8_t writeHoldingRegisters16(uint8_t id, uint16_t cmd, uint16_t data);               // Установить значение регистра (2 байта) МХ2 в виде целого  числа возвращает код ошибки данные data
    int8_t writeHoldingRegistersFloat(uint8_t id, uint16_t cmd, float dat);                // Записать float как 2 регистра числа возвращает код ошибки данные data
    int8_t writeHoldingRegisters32(uint8_t id,uint16_t cmd, uint32_t data); 			   // BIG-ENDIAN! Записать 2 регистра подряд возвращает код ошибки
    int8_t CustomRequest(uint8_t id, uint8_t cmd);		// Отправка произвольной комманды (8 бит)
    int8_t CustomRequestData(uint8_t id, char *str);	// Отправка произвольной комманды в виде строки чисел (hex - 0x...)
    int8_t RelaySwitch(uint8_t id, uint16_t cmd, uint8_t r);								// Установить реле в состояние r
    int8_t get_err() {return err;}                                                         // Получить код ошибки
    ModbusMaster RS485;                     // Класс модбас 485
private:
    // Переменные
    int8_t flags;                           // Флаги
    int8_t err;                             // Ошибки модбас
    type_SEMAPHORE xModbusSemaphore; 	// Семафор Modbus
    int8_t translateErr(uint8_t result);    // Перевод ошибки протокола Модбас
  }; // End class

 #endif
