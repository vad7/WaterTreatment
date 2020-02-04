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

//  описание вспомогательных Kлассов данных, предназначенных для получения информации
#include "Information.h"

#define bufI2C Socket[0].outBuf
#define bufI2C Socket[0].outBuf
#ifdef DEBUG_NATIVE_USB
	#define SerialDbg	SerialUSB
#else
	#define SerialDbg	Serial
#endif

// --------------------------------------------------------------------------------------------------------------- 
//  Класс системный журнал пишет в консоль и в память ------------------------------------------------------------
//  Место размещения (озу ли флеш) определяется дефайном #define I2C_JOURNAL_IN_RAM
// --------------------------------------------------------------------------------------------------------------- 
// Инициализация
void Journal::Init()
{
	err = OK;
#ifdef DEBUG
#ifndef DEBUG_NATIVE_USB
	SerialDbg.begin(UART_SPEED);                   // Если надо инициализировать отладочный порт
#endif
#endif

#ifdef I2C_JOURNAL_IN_RAM     // журнал в памяти
	bufferTail = 0;
	bufferHead = 0;
	full = 0;                   // Буфер не полный
	memset(_data, 0, JOURNAL_LEN);
	printf("\nSTART ----\n");
	printf("Init RAM journal, size %d . . .\n", JOURNAL_LEN);
	return;
#else                      // журнал во флеше

	uint8_t eepStatus = 0;
	uint16_t i;
	char *ptr;

	if((eepStatus = eepromI2C.begin(I2C_SPEED) != 0))  // Инициализация памяти
	{
#ifdef DEBUG
		SerialDbg.println("$ERROR - open I2C journal, check I2C chip!");   // ошибка открытия чипа
#endif
		err = ERR_OPEN_I2C_JOURNAL;
		return;
	}

	if(checkREADY() == false) // Проверка наличия журнал
	{
#ifdef DEBUG
		SerialDbg.print("I2C journal not found! ");
#endif
		Format();
		return;
	}

	bufferTail = bufferHead = -1;
	full = 0;
	for(i = 0; i < JOURNAL_LEN / W5200_MAX_LEN; i++)   // поиск журнала начала и конца, для ускорения читаем по W5200_MAX_LEN байт
	{
		WDT_Restart(WDT);
#ifdef DEBUG
		SerialDbg.print(".");
#endif
		if(readEEPROM_I2C(I2C_JOURNAL_START + i * W5200_MAX_LEN, (byte*) &bufI2C,W5200_MAX_LEN)) {
			err=ERR_READ_I2C_JOURNAL;
#ifdef DEBUG
			SerialDbg.print(errorReadI2C);
#endif
			break;
		}
		if((ptr = (char*) memchr(bufI2C, I2C_JOURNAL_HEAD, W5200_MAX_LEN)) != NULL) {
			bufferHead=i*W5200_MAX_LEN+(ptr-bufI2C);
		}
		if((ptr = (char*) memchr(bufI2C, I2C_JOURNAL_TAIL ,W5200_MAX_LEN)) != NULL) {
			bufferTail=i*W5200_MAX_LEN+(ptr-bufI2C);
		}
		if((bufferTail >= 0) && (bufferHead >= 0)) break;
	}
	if(bufferTail < bufferHead) full = 1;                   // Буфер полный
	printf("\nSTART ---\n");
	printf("Found I2C journal: size %d bytes, head=%d, tail=%d\n", JOURNAL_LEN, bufferHead, bufferTail);
#endif //  #ifndef I2C_EEPROM_64KB     // журнал в памяти
}

  
#ifndef I2C_JOURNAL_IN_RAM  // функции долько для I2C журнала
// Записать признак "форматирования" журнала - журналом можно пользоваться
void Journal::writeREADY()
{  
    uint16_t  w=I2C_JOURNAL_READY; 
    if (writeEEPROM_I2C(I2C_JOURNAL_START-2, (byte*)&w,sizeof(w))) 
       { err=ERR_WRITE_I2C_JOURNAL; 
         #ifdef DEBUG
         SerialDbg.println(errorWriteI2C);
         #endif
        }
}
// Проверить наличие журнала
boolean Journal::checkREADY()
{  
    uint16_t  w=0x0; 
    if (readEEPROM_I2C(I2C_JOURNAL_START-2, (byte*)&w,sizeof(w))) 
       { err=ERR_READ_I2C_JOURNAL; 
         #ifdef DEBUG
         SerialDbg.print(errorReadI2C);
         #endif
        }
    if (w!=I2C_JOURNAL_READY) return false; else return true;
}

// Форматирование журнала (инициализация I2C памяти уже проведена), sizeof(buf)=W5200_MAX_LEN
void Journal::Format(void)
{
	err = OK;
	#ifdef DEBUG
	SerialDbg.print("Formating I2C journal ");
	#endif
//	memset(buf, I2C_JOURNAL_FORMAT, W5200_MAX_LEN);
//	for(uint32_t i = 0; i < JOURNAL_LEN / W5200_MAX_LEN; i++) {
//		#ifdef DEBUG
//		SerialDbg.print("*");
//		#endif
//		if(i == 0) {
//			buf[0] = I2C_JOURNAL_HEAD;
//			buf[1] = I2C_JOURNAL_TAIL;
//		} else {
//			buf[0] = I2C_JOURNAL_FORMAT;
//			buf[1] = I2C_JOURNAL_FORMAT;
//		}
//		if(writeEEPROM_I2C(I2C_JOURNAL_START + i * W5200_MAX_LEN, (byte*)&buf, W5200_MAX_LEN)) {
//			err = ERR_WRITE_I2C_JOURNAL;
//			#ifdef DEBUG
//			SerialDbg.println(errorWriteI2C);
//			#endif
//			break;
//		};
//		WDT_Restart(WDT);
//	}
	uint8_t buf[] = { I2C_JOURNAL_HEAD, I2C_JOURNAL_TAIL };
	if(writeEEPROM_I2C(I2C_JOURNAL_START, (uint8_t*)buf, sizeof(buf))) {
		err = ERR_WRITE_I2C_JOURNAL;
		#ifdef DEBUG
		SerialDbg.println(errorWriteI2C);
		#endif
	}
	full = 0;                   // Буфер не полный
	bufferHead = 0;
	bufferTail = 1;
	if(err == OK) {
		writeREADY();                 // было форматирование
		jprintf("\nFormat I2C journal (size %d bytes) - Ok\n", JOURNAL_LEN);
	}
}
#endif
    
// Печать только в консоль
void Journal::printf(const char *format, ...)             
{
#ifdef DEBUG
	if(Is_otg_vbus_high()) return;
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf, PRINTF_BUF, format, ap);
	va_end(ap);
	SerialDbg.print(pbuf);
#endif
}

// Печать в консоль и журнал
void Journal::jprintf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf, PRINTF_BUF, format, ap);
	va_end(ap);
#ifdef DEBUG
	if(Is_otg_vbus_high()) SerialDbg.print(pbuf);
#endif
	// добавить строку в журнал
	_write(pbuf);
}

// Печать в консоль и журнал
void Journal::jprintfopt(const char *format, ...)
{
	if(!DebugToJournalOn) return;
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf, PRINTF_BUF, format, ap);
	va_end(ap);
#ifdef DEBUG
	if(Is_otg_vbus_high()) SerialDbg.print(pbuf);
#endif
	// добавить строку в журнал
	_write(pbuf);
}

// +Time, далее печать в консоль и журнал
void Journal::jprintf_time(const char *format, ...)
{
	strcpy(pbuf, NowTimeToStr());
	strcat(pbuf, " ");
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf + 9, PRINTF_BUF - 9, format, ap);
	va_end(ap);
#ifdef DEBUG
	if(Is_otg_vbus_high()) SerialDbg.print(pbuf);
#endif
	_write(pbuf);   // добавить строку в журнал
}   

// +Time, опционально печать в консоль и журнал
void Journal::jprintfopt_time(const char *format, ...)
{
	if(!DebugToJournalOn) return;
	strcpy(pbuf, NowTimeToStr());
	strcat(pbuf, " ");
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf + 9, PRINTF_BUF - 9, format, ap);
	va_end(ap);
#ifdef DEBUG
	if(Is_otg_vbus_high()) SerialDbg.print(pbuf);
#endif
	_write(pbuf);   // добавить строку в журнал
}

// +DateTime, далее печать в консоль и журнал
void Journal::jprintf_date(const char *format, ...)
{
	strcpy(pbuf, NowDateToStr());
	strcat(pbuf, " ");
	strcat(pbuf, NowTimeToStr());
	strcat(pbuf, " ");
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf + 20, PRINTF_BUF - 20, format, ap);
	va_end(ap);
#ifdef DEBUG
	if(Is_otg_vbus_high()) SerialDbg.print(pbuf);
#endif
	_write(pbuf);   // добавить строку в журнал
}

// Печать ТОЛЬКО в журнал возвращает число записанных байт для использования в критических секциях кода
void Journal::jprintf_only(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	m_vsnprintf(pbuf, PRINTF_BUF, format, ap);
	va_end(ap);
	_write(pbuf);
}

// отдать журнал в сеть клиенту  Возвращает число записанных байт
int32_t Journal::send_Data(uint8_t thread)
{
	int32_t num, len, sum = 0;
#ifndef I2C_JOURNAL_IN_RAM // чтение еепром
	num = bufferHead + 1;                     // Начинаем с начала журнала, num позиция в буфере пропуская символ начала
	for(uint16_t i = 0; i < (JOURNAL_LEN / W5200_MAX_LEN + 1); i++) // Передаем пакетами по W5200_MAX_LEN байт, может быть два неполных пакета!!
	{
		__asm__ volatile ("" ::: "memory");
		if((num > bufferTail))                                        // Текущая позиция больше хвоста (начало передачи)
		{
			if(JOURNAL_LEN - num >= W5200_MAX_LEN) len = W5200_MAX_LEN;	else len = JOURNAL_LEN - num;   // Контроль достижения границы буфера
		} else {                                                        // Текущая позиция меньше хвоста (конец передачи)
			if(bufferTail - num >= W5200_MAX_LEN) len = W5200_MAX_LEN; else len = bufferTail - num;     // Контроль достижения хвоста журнала
		}
		if(readEEPROM_I2C(I2C_JOURNAL_START + num, (byte*) Socket[thread].outBuf, len))         // чтение из памяти
		{
			err = ERR_READ_I2C_JOURNAL;
#ifdef DEBUG
			SerialDbg.print(errorReadI2C);
#endif
			return 0;
		}
		if(sendPacketRTOS(thread, (byte*) Socket[thread].outBuf, len, 0) == 0) return 0;        // передать пакет, при ошибке выйти
		_delay(2);
		sum = sum + len;                                                                        // сколько байт передано
		if(sum >= available()) break;                                                           // Все передано уходим
		num = num + len;                                                                        // Указатель на переданные данные
		if(num >= JOURNAL_LEN) num = 0;                                                         // переходим на начало
	}  // for
#else
	num=bufferHead;                                                   // Начинаем с начала журнала, num позиция в буфере
	for(uint16_t i=0;i<(JOURNAL_LEN/W5200_MAX_LEN+1);i++)// Передаем пакетами по W5200_MAX_LEN байт, может быть два неполных пакета!!
	{
		if((num>bufferTail))                              // Текущая позиция больше хвоста (начало передачи)
		{
			if (JOURNAL_LEN-num>=W5200_MAX_LEN) len=W5200_MAX_LEN; else len=JOURNAL_LEN-num; // Контроль достижения границы буфера
		} else {                                                           // Текущая позиция меньше хвоста (конец передачи)
			if (bufferTail-num>=W5200_MAX_LEN) len=W5200_MAX_LEN; else len=bufferTail-num; // Контроль достижения хвоста журнала
		}
		if(sendPacketRTOS(thread,(byte*)_data+num,len,0)==0) return 0;          // передать пакет, при ошибке выйти
		_delay(2);
		sum=sum+len;// сколько байт передано
		if (sum>=available()) break;// Все передано уходим
		num=num+len;// Указатель на переданные данные
		if (num>=JOURNAL_LEN) num=0;// переходим на начало
	}  // for
#endif
	return sum;
}

// Возвращает размер журнала
int32_t Journal::available(void)
{ 
  #ifdef I2C_EEPROM_64KB
    if (full) return JOURNAL_LEN - 2; else return bufferTail-1;
  #else   
     if (full) return JOURNAL_LEN; else return bufferTail;
  #endif
}    
                 
// чтобы print рабоtал для это класса
size_t Journal::write (uint8_t c)
  {
  SerialDbg.print(char(c));
  return 1;   // one byte output
  }  // end of myOutputtingClass::write
         
// Записать строку в журнал
void Journal::_write(char *dataPtr)
{
	int32_t numBytes;
	if(dataPtr == NULL || (numBytes = strlen(dataPtr)) == 0) return;  // Записывать нечего
#ifndef I2C_JOURNAL_IN_RAM // запись в еепром
	if(numBytes > JOURNAL_LEN - 2) numBytes = JOURNAL_LEN - 2; // Ограничиваем размером журнала JOURNAL_LEN не забываем про два служебных символа
	// Запись в I2C память
	if(SemaphoreTake(xI2CSemaphore, I2C_TIME_WAIT / portTICK_PERIOD_MS) == pdFALSE) {  // Если шедулер запущен то захватываем семафор
		journal.jprintfopt((char*) cErrorMutex, __FUNCTION__, MutexI2CBuzy);
		return;
	}
	__asm__ volatile ("" ::: "memory");
	dataPtr[numBytes] = I2C_JOURNAL_TAIL;
	if(full) dataPtr[numBytes + 1] = I2C_JOURNAL_HEAD;
	if(bufferTail + numBytes + 2 > JOURNAL_LEN) { //  Запись в два приема если число записываемых бит больше чем место от конца очереди до конца буфера ( помним про символ начала)
		int32_t n;
		if(eepromI2C.write(I2C_JOURNAL_START + bufferTail, (byte*) dataPtr, n = JOURNAL_LEN - bufferTail)) {
			#ifdef DEBUG
				if(err != ERR_WRITE_I2C_JOURNAL) SerialDbg.print(errorWriteI2C);
			#endif
			err = ERR_WRITE_I2C_JOURNAL;
		} else {
			dataPtr += n;
			numBytes -= n;
			full = 1;
			if(eepromI2C.write(I2C_JOURNAL_START, (byte*) dataPtr, numBytes + 2)) {
				err = ERR_WRITE_I2C_JOURNAL;
				#ifdef DEBUG
					SerialDbg.print(errorWriteI2C);
				#endif
			} else {
				bufferTail = numBytes;
				bufferHead = bufferTail + 1;
				err = OK;
			}
		}
	} else {  // Запись в один прием Буфер не полный
		if(eepromI2C.write(I2C_JOURNAL_START + bufferTail, (byte*) dataPtr, numBytes + 1 + full)) {
			#ifdef DEBUG
				if(err != ERR_WRITE_I2C_JOURNAL) SerialDbg.print(errorWriteI2C);
			#endif
			err = ERR_WRITE_I2C_JOURNAL;
		} else {
			bufferTail += numBytes;
			if(full) bufferHead = bufferTail + 1;
		}
	}
	SemaphoreGive(xI2CSemaphore);
#else   // Запись в память
	// SerialDbg.print(">"); SerialDbg.print(numBytes); SerialDbg.println("<");

	if( numBytes >= JOURNAL_LEN ) numBytes = JOURNAL_LEN;// Ограничиваем размером журнала
	// Запись в журнал
	if(numBytes > JOURNAL_LEN - bufferTail)//  Запись в два приема если число записываемых бит больше чем место от конца очереди до конца буфера
	{
		int len = JOURNAL_LEN - bufferTail;             // сколько можно записать в конец
		memcpy(_data+bufferTail,dataPtr,len);// Пишем с конца очереди но до конца журнала
		memcpy(_data, dataPtr+len, numBytes-len);// Пишем в конец буфера с начала
		bufferTail = numBytes-len;// Хвост начинает рости с начала буфера
		bufferHead=bufferTail +1;// Буфер полный по этому начало стоит сразу за концом (затирание данных)
		full = 1;// буфер полный
	} else   // Запись в один прием Буфер
	{
		memcpy(_data+bufferTail, dataPtr, numBytes);     // Пишем с конца очереди
		bufferTail = bufferTail + numBytes;// Хвост вырос
		if (full) bufferHead=bufferTail+1;// голова изменяется только при полном буфере (затирание данных)
		else bufferHead=0;
	}
#endif
}

    
// ---------------------------------------------------------------------------------
//  Класс ГРАФИКИ    ------------------------------------------------------------
// ---------------------------------------------------------------------------------

 // Инициализация
void statChart::init(boolean pres)
{  
  err=OK; 
  present=pres;                                 // наличие статистики - зависит от конфигурации
  pos=0;                                        // текущая позиция для записи
  num=0;                                        // число накопленных точек
  flagFULL=false;                               // false в буфере менее CHART_POINT точек
  if (pres)                                     // отводим память если используем статистику
  { 
    data=(int16_t*)malloc(sizeof(int16_t)*CHART_POINT);
    if (data==NULL) {err=ERR_OUT_OF_MEMORY; set_Error(err,(char*)__FUNCTION__);return;}  // ОШИБКА если память не выделена
    for(int i=0;i<CHART_POINT;i++) data[i]=0;     // обнуление
  }  
}

// Очистить статистику
void statChart::clear()
{   
  pos=0;                                        // текущая позиция для записи
  num=0;                                        // число накопленных точек
  flagFULL=false;                               // false в буфере менее CHART_POINT точек
  for(int i=0;i<CHART_POINT;i++) data[i]=0;     // обнуление
}

 // добавить точку в массиве
void statChart::addPoint(int16_t y)
{
 if (!present) return; 
 data[pos]=y;
 if (pos<CHART_POINT-1) pos++; else { pos=0; flagFULL=true; }
 if (!flagFULL) num++ ;   // буфер пока не полный
}

// получить точку нумерация 0-самая новая CHART_POINT-1 - самая старая, (работает кольцевой буфер)
inline int16_t statChart::get_Point(uint16_t x)
{
 if (!present) return 0; 
 if (!flagFULL) return data[x];
 else 
 {
    if ((pos+x)<CHART_POINT) return data[pos+x];else return data[pos+x-CHART_POINT];
 }
}

// БИНАРНЫЕ данные по маске: получить точку нумерация 0-самая старая CHART_POINT - самая новая, (работает кольцевой буфер)
boolean statChart::get_boolPoint(uint16_t x,uint16_t mask)  
{ 
 if (!present) return 0; 
 if (!flagFULL) return data[x]&mask?true:false;
 else 
 {
    if ((pos+x)<CHART_POINT) return data[pos+x]&mask?true:false; 
    else                     return data[pos+x-CHART_POINT]&mask?true:false;
 }
}

// получить строку в которой перечислены все точки в строковом виде через; при этом значения делятся на 100
// строка не обнуляется перед записью
void statChart::get_PointsStrDiv100(char *&b)
{
	if((!present) || (num == 0)) {
		//strcat(b, ";");
		return;
	}
	b += m_strlen(b);
	for(uint16_t i = 0; i < num; i++) {
		b = dptoa(b, get_Point(i), 2);
		*b++ = ';';
		*b = '\0';
	}
}

// получить строку в которой перечислены все точки в строковом виде через;
// строка не обнуляется перед записью
void statChart::get_PointsStr(char *&b)
{
	if((!present) || (num == 0)) {
		//strcat(b, ";");
		return;
	}
	b += m_strlen(b);
	for(uint16_t i = 0; i < num; i++) {
		b += m_itoa(get_Point(i), b, 10, 0);
		*b++ = ';';
		*b = '\0';
	}
}

void statChart::get_PointsStrSubDiv100(char *&b, statChart *sChart)
{
	if(!present || num == 0 || !sChart->get_present() || sChart->get_num() == 0) {
		//strcat(b, ";");
		return;
	}
	b += m_strlen(b);
	for(uint16_t i = 0; i < num; i++) {
		b = dptoa(b, get_Point(i) - sChart->get_Point(i), 2);
		*b++ = ';';
		*b = '\0';
	}
}
// Расчитать мощность на лету используется для графика потока, передаются указатели на графики температуры + теплоемкость
void statChart::get_PointsStrPower(char *&b, statChart *inChart, statChart *outChart, uint16_t Capacity)
{
	if(!present || num == 0 || !inChart->get_present() || inChart->get_num() == 0 || !outChart->get_present() || outChart->get_num() == 0) {
		//strcat(b, ";");
		return;
	}
	b += m_strlen(b);
	for(uint16_t i = 0; i < num; i++) {
		b = dptoa(b, (int32_t)(outChart->get_Point(i)-inChart->get_Point(i)) * get_Point(i) * Capacity / 360, 3);
		*(b-1) = ';';
		//*b = '\0';
	}
}
