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
// --------------------------------------- функции общего использования -------------------------------------------
#ifndef Util_h
#define Util_h

#include <Arduino.h>

#define sign(a) (a > 0 ? 1 : a < 0 ? -1 : 0)
#define signm(a,t) ((a > 0 ? 1 : a < 0 ? -1 : 0) * (abs(a) > t ? 2 : 1))

struct type_SEMAPHORE {
	volatile bool xSemaphore;
	uint16_t BusyCnt;
};
void SemaphoreCreate(type_SEMAPHORE &_sem);
bool SemaphoreTake(type_SEMAPHORE &_sem, uint32_t wait_time);// Захватить семафор с проверкой, что шедуллер работает
inline void SemaphoreGive(type_SEMAPHORE &_sem) { _sem.xSemaphore = false; };

// Структура для хранения переменных для паролей
struct type_WebSecurity
{
	uint8_t len;
	char *hash;
};
void	calc_WebSec_hash(type_WebSecurity *ws, char *login, char *pass, char *buf);

uint16_t calc_crc16(unsigned char * pcBlock, unsigned short len, uint16_t crc = 0xFFFF);
void	int_to_dec_str(int32_t value, int32_t div, char **ret, uint8_t maxfract);
uint8_t	calc_bits_in_mask(uint32_t mask);
int32_t	round_div_int32(int32_t value, int16_t div);
void	getIDchip(char *outstr);
uint8_t update_RTC_store_memory(void);
char*	_itoa( int value, char *string);
uint8_t _ftoa(char *outstr, float val, unsigned char precision);
void 	_dtoa(char *outstr, int val, int precision);
//void LCD_print(char *buf);
void 	buffer_space_padding(char * buf, int add);
char* 	NowTimeToStr(char *buf = NULL);
char* 	NowDateToStr(char *buf = NULL);
uint8_t	GetSerialNum(USARTClass &serial);
#ifdef SECOND_I2C_USED
int8_t  Second_I2C_Read(uint8_t addr, uint8_t len, uint8_t *data); // len без CRC8
#endif

#endif
