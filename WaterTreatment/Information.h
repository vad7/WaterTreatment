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

enum type_promt //  Перечисляемый тип - что идет в начале строки при выводе в журнал
{
    pP_NONE,        // ничего
    pP_TIME,        // время
    pP_DATE,        // дата и время
    pP_USER         // константа определяемая пользователем
};

#define PRINTF_BUF 256                           // размер буфера для одной строки - большаяя длина нужна при отправке уведомлений, там длинные строки (видел 178)

extern uint16_t sendPacketRTOS(uint8_t thread, const uint8_t * buf, uint16_t len,uint16_t pause);
const char *MessageLongString = { "Jornal: Input string too long, skip string!"};  
const char *errorReadI2C =    {"$ERROR - read I2C memory\n"};
const char *errorWriteI2C =   {"$ERROR - write I2C memory\n"};
const char *promtUser={"> "};   

class Journal :public Print
{
public:
 // SemaphoreHandle_t xJournalSemaphore;                    // Семафор Journal
  void Init();                                            // Инициализация
  void printf(const char *format, ...) ;                  // Печать только в консоль
  void jprintf(const char *format, ...);                  // Печать в консоль и журнал возвращает число записанных байт
  void jprintf(type_promt pr,const char *format, ...);    // Печать в консоль и журнал возвращает число записанных байт с типом промта
  void jprintf_only(const char *format, ...);             // Печать ТОЛЬКО в журнал возвращает число записанных байт для использования в критических секциях кода
  int32_t send_Data(uint8_t thread);                     // отдать журнал в сеть клиенту  Возвращает число записанных байт
  int32_t available(void);                               // Возвращает размер журнала
  int8_t   get_err(void) { return err; };
  virtual size_t write (uint8_t c);                       // чтобы print работал для это класса
  #ifndef I2C_JOURNAL_IN_RAM                                  // Если журнал находится в i2c
  void Format(void);                      		         // форматирование журнала в еепром
  #else
  void Clear(){bufferTail=0;bufferHead=0;full=false;err=OK;} // очистка журнала в памяти
  #endif
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
  void init(boolean pres);                              // инициализация класса
  void clear();                                         // очистить статистику
  void addPoint(int16_t y);                             // добавить точку в массиве (для бинарных данных добавляем все точки сразу)
  inline int16_t get_Point(uint16_t x);                 // получить точку нумерация 0-самая старая CHART_POINT - самая новая, (работает кольцевой буфер)
  boolean get_boolPoint(uint16_t x,uint16_t mask);      // БИНАРНЫЕ данные: получить точку нумерация 0-самая старая CHART_POINT - самая новая, (работает кольцевой буфер)
  
  void get_PointsStr(uint16_t m, char *&b);             // получить строку в которой перечислены все точки в строковом виде через ";" при этом значения делятся на m
  void get_PointsStrSub(uint16_t m, char *&b, statChart *sChart); // получить строку, вычесть точки sChart
  void get_PointsStrPower(uint16_t m, char *&b, statChart *inChart,statChart *outChart, float kfCapacity); // Расчитать мощность на лету используется для графика потока, передаются указатели на температуры
  
  inline boolean get_present() {return present;}        // Наличие датчика в текущей конфигурации
  inline uint16_t get_num()  {return num;}              // Получить число накопленных точек
 private:
  int8_t err;                                            // Ошибка
  int16_t *data;                                        // указатель на массив для накопления данных
  int16_t pos;                                          // текущая позиция для записи
  int16_t num;                                          // число накопленных точек
  boolean present;                                      // наличие статистики
  boolean flagFULL;                                     // false в буфере менее CHART_POINT точек
};

#endif
