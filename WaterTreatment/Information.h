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
//  Описание вспомогательных классов данных, предназначенных для получения информации
#ifndef Information_h
#define Information_h
#include "Constant.h"                       // Вся конфигурация и константы проекта Должен быть первым !!!!
//#include "utility/w5100.h"
//#include "utility/socket.h"
// --------------------------------------------------------------------------------------------------------------- 
//  Класс системный журнал пишет в консоль и в память ------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------- 
//  Необходим для получения лога контроллера
//  http://arduino.ru/forum/programmirovanie/etyudy-dlya-nachinayushchikh-interfeis-printable
//  Для записи ТОЛЬКО в консоль использовать функции printf
//  Для записи в консоль И в память (журнал) использовать jprintf
//  По умолчанию журнал пишется в RAM размер JOURNAL_LEN
//  Если включена опция I2C_JOURNAL_IN_RAM то журнал пишется в I2C память. Должен быть устанвлен чип размером более 4кБ, адрес начала записи 0x0fff до I2C_STAT_EEPROM

#define PRINTF_BUF 256                           // размер буфера для одной строки - большаяя длина нужна при отправке уведомлений, там длинные строки (видел 178)

extern uint16_t sendPacketRTOS(uint8_t thread, const uint8_t * buf, uint16_t len,uint16_t pause);
const char *errorReadI2C =    {"$ERROR - read I2C memory\n"};
const char *errorWriteI2C =   {"$ERROR - write I2C memory\n"};

#define I2C_JOURNAL_HEAD   		0x01 					// Признак головы журнала
#define I2C_JOURNAL_TAIL   		0x02					// Признак хвоста журнала
#define I2C_JOURNAL_FORMAT 		0xFF					// Символ которым заполняется журнал при форматировании
#define I2C_JOURNAL_READY  		0x55AA					// Признак создания журнала - если его нет по адресу I2C_JOURNAL_START-2 то надо форматировать журнал (первичная инициализация)

class Journal :public Print
{
public:
  void Init();                                            // Инициализация
  void printf(const char *format, ...) ;                  // Печать только в консоль
  void jprintf(const char *format, ...);                  // Печать в консоль и журнал
  void jprintf_time(const char *format, ...);			// +Time, далее печать в консоль и журнал
  void jprintf_date(const char *format, ...);			// +DateTime, далее печать в консоль и журнал
  void jprintfopt(const char *format, ...);               // Опционально. Печать в консоль и журнал
  void jprintfopt_time(const char *format, ...);		// Опционально.
  void jprintf_only(const char *format, ...);            // Печать ТОЛЬКО в журнал для использования в критических секциях кода
  int32_t send_Data(uint8_t thread);                     // отдать журнал в сеть клиенту  Возвращает число записанных байт
  int32_t available(void);                               // Возвращает размер журнала
  int8_t   get_err(void) { return err; };
  virtual size_t write (uint8_t c);                       // чтобы print работал для это класса
  #ifndef I2C_JOURNAL_IN_RAM                                  // Если журнал находится в i2c
  void Format(void);                      		         // форматирование журнала в еепром
  #else
  void Clear(){bufferTail=0;bufferHead=0;full=false;err=OK;} // очистка журнала в памяти
  #endif
  type_SEMAPHORE Semaphore;                    			// Семафор
private:
  int8_t err;                                             // ошибка журнала
  int32_t bufferHead, bufferTail;                        // Начало и конец
  uint8_t full;                                           // признак полного буфера
  void _write(char *dataPtr);                            // Записать строку в журнал
   // Переменные
  char pbuf[PRINTF_BUF+2];                                // Буфер для одной строки + маркеры
  #ifdef I2C_JOURNAL_IN_RAM                                 // Если журнал находится в памяти
    char _data[JOURNAL_LEN+1];                            // Буфер журнала
  #else
    void writeTAIL();                                     // Записать символ "конец" значение bufferTail должно быть установлено
    void writeHEAD();                                     // Записать символ "начало" значение bufferHead должно быть установлено
    void writeREADY();                                    // Записать признак "форматирования" журнала - журналом можно пользоваться
    boolean checkREADY();                                 // Проверить наличие журнала
  #endif
 };

// ---------------------------------------------------------------------------------
//  Класс Графики работы   ------------------------------------------------------------
// ---------------------------------------------------------------------------------
// Бинарные данные могут хранится в упакованном виде до 16 значений в одной точке.
// запихиваем уже упакованное значение
// читаем get_boolPointsStr
class statChart                                         // организован кольцевой буфер
{
public:
	void init();                                         // инициализация класса
	void clear();                                        // очистить статистику
	void addPoint(int16_t y);                            // добавить точку в массиве (для бинарных данных добавляем все точки сразу)
	inline int16_t get_Point(int16_t x);                 // получить точку нумерация 0-самая старая CHART_POINT - самая новая, (работает кольцевой буфер)
	inline int16_t get_PrevPoint(void);             // получить предыдущую точку
	boolean get_boolPoint(int16_t x,uint16_t mask);      // БИНАРНЫЕ данные: получить точку нумерация 0-самая старая CHART_POINT - самая новая, (работает кольцевой буфер)

	void get_PointsStrDiv100(char *&b);             		// получить строку в которой перечислены все точки в строковом виде через ";" при этом значения делятся на 100
	void get_PointsStrAbsDiv100(char *&b);					// получить строку в которой перечислены все точки в строковом виде через ";" при этом значения делятся на 100 и берется абсолютное
	void get_PointsStrPositiveDiv100(char *&b);				// получить строку в которой перечислены все точки в строковом виде через ";" при этом беруться только положительные значения и делятся на 100
	void get_PointsStrUintDiv100(char *&b);           		// unsigned, получить строку в которой перечислены все точки в строковом виде через ";" при этом значения делятся на 100
	void get_PointsStrUintDiv1000(char *&b);           		// unsigned, получить строку в которой перечислены все точки в строковом виде через ";" при этом значения делятся на 1000
	void get_PointsStr(char *&b);							// получить строку в которой перечислены все точки в строковом виде через ";"
	void get_PointsStrSubDiv100(char *&b, statChart *sChart); // получить строку, вычесть точки sChart
	void get_PointsStrPower(char *&b, statChart *inChart, statChart *outChart, uint16_t Capacity); // Расчитать мощность на лету используется для графика потока, передаются указатели на температуры

private:
	int16_t *data;                                        // указатель на массив для накопления данных
	int16_t pos;                                          // текущая позиция для записи
	boolean flagFULL;                                     // false в буфере менее CHART_POINTS точек
};

#endif
