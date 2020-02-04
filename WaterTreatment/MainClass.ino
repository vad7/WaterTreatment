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
// Описание базового класса для работы
// --------------------------------------------------------------------------------
#include "MainClass.h"

/* structsize
  char checker(int);
  char checkSizeOfInt1[sizeof(MC.Option)]={checker(&checkSizeOfInt1)};
//*/

// Установка критической ошибки для класса  вызывает останов 
// Возвращает ошибку останова 
int8_t set_Error(int8_t _err, char *nam)
{
	if(MC.error != _err) {
		MC.error = _err;
		strcpy(MC.source_error, nam);
		m_snprintf(MC.note_error, sizeof(MC.note_error), "%s %s: %s", NowTimeToStr(), nam, noteError[abs(_err)]);
	}
	uint32_t i = 0;
	for(; i < sizeof(Errors) / sizeof(Errors[0]); i++) {
		if(Errors[i] == OK) break;
		if(Errors[i] == _err) {
			i = sizeof(Errors) / sizeof(Errors[0]);
			break;
		}
	}
	if(i != sizeof(Errors) / sizeof(Errors[0])) {
		Errors[i] = _err;
		ErrorsTime[i] = rtcSAM3X8.unixtime();
		journal.jprintf_date("$ERROR source: %s, code: %d\n", nam, _err); //journal.jprintf(", code: %d\n",_err);
		if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) MC.save_DumpJournal(); // вывод отладочной информации для начала  если запущена freeRTOS
		MC.message.setMessage(pMESSAGE_ERROR, MC.note_error, 0);    // сформировать уведомление об ошибке
	}
	return _err;
}

void Weight_Clear_Averaging(void)
{
	memset(&Weight_adc_filter, 0, sizeof(Weight_adc_filter));
	Weight_adc_sum = 0;
	Weight_adc_flagFull = false;
	Weight_adc_idx = 0;
}

void MainClass::init()
{
	uint8_t i;
	eraseError();

	for(i = 0; i < TNUMBER; i++) sTemp[i].initTemp(i);            // Инициализация датчиков температуры

	for(i = 0; i < ANUMBER; i++) sADC[i].initSensorADC(i, pinsAnalog[i], ADC_FILTER[i]);	// Инициализация аналогового датчика

	for(i = 0; i < INUMBER; i++) sInput[i].initInput(i);           // Инициализация контактных датчиков
	for(i = 0; i < FNUMBER; i++) sFrequency[i].initFrequency(i);  // Инициализация частотных датчиков
	for(i = 0; i < RNUMBER; i++) dRelay[i].initRelay(i);           // Инициализация реле

	Modbus.initModbus();

	dPWM.initPWM();                                           // инициализация счетчика
	message.initMessage(MAIN_WEB_TASK);                       // Инициализация Уведомлений, параметр - номер потока сервера в котором идет отправка
#ifdef MQTT
	clMQTT.initMQTT(MAIN_WEB_TASK);                           // Инициализация MQTT, параметр - номер потока сервера в котором идет отправка
#endif

	ChartWaterBoost.init(true);
	ChartFeedPump.init(true);
	ChartFillTank.init(true);
	ChartBrineWeight.init(true);

	resetSetting();                                           // все переменные
}
// Стереть последнюю ошибку
void MainClass::eraseError()
{
	strcpy(note_error, "OK");          // Строка c описанием ошибки
	strcpy(source_error, "");          // Источник ошибки
	error = OK;                         // Код ошибки
}

// Получить число ошибок чтения ВСЕХ датчиков темпеартуры
uint32_t MainClass::get_errorReadDS18B20()
{
	uint32_t sum = 0;
	for(uint8_t i = 0; i < TNUMBER; i++) sum += sTemp[i].get_sumErrorRead();     // Суммирование ошибок по всем датчикам
	return sum;
}

void MainClass::Reset_TempErrors()
{
	for(uint8_t i=0; i<TNUMBER; i++) sTemp[i].Reset_Errors();
}

// возвращает строку с найденными датчиками
void MainClass::scan_OneWire(char *result_str)
{
	if(!OW_scan_flags && OW_prepare_buffers()) {
		OW_scan_flags = 1; // Идет сканирование
		journal.jprintf("Scan 1-Wire:\n");
//		char *_result_str = result_str + m_strlen(result_str);
		OneWireBus.Scan(result_str);
#ifdef ONEWIRE_DS2482_SECOND
		OneWireBus2.Scan(result_str);
#endif
#ifdef ONEWIRE_DS2482_THIRD
		OneWireBus3.Scan(result_str);
#endif
#ifdef ONEWIRE_DS2482_FOURTH
		OneWireBus4.Scan(result_str);
#endif
		journal.jprintf("Found: %d\n", OW_scanTableIdx);
//		while(strlen(_result_str)) {
//			journal.jprintf(_result_str);
//			uint16_t l = strlen(_result_str);
//			_result_str += l > PRINTF_BUF-1 ? PRINTF_BUF-1 : l;
//		}
#ifdef RADIO_SENSORS
		journal.jprintf("Radio found(%d): ", radio_received_num);
		for(uint8_t i = 0; i < radio_received_num; i++) {
			OW_scanTable[OW_scanTableIdx].num = OW_scanTableIdx + 1;
			OW_scanTable[OW_scanTableIdx].bus = 7;
			memset(&OW_scanTable[OW_scanTableIdx].address, 0, sizeof(OW_scanTable[0].address));
			OW_scanTable[OW_scanTableIdx].address[0] = tRadio;
			memcpy(&OW_scanTable[OW_scanTableIdx].address[1], &radio_received[i].serial_num, sizeof(radio_received[0].serial_num));
			char *p = result_str + strlen(result_str);
			m_snprintf(p, 64, "%d:RADIO %.1fV/%c:%.2f:%u:7;", OW_scanTable[OW_scanTableIdx].num, (float)radio_received[i].battery/10, Radio_RSSI_to_Level(radio_received[i].RSSI), (float)radio_received[i].Temp/100.0, radio_received[i].serial_num);
			journal.jprintf("%s", p);
			if(++OW_scanTableIdx >= OW_scanTable_max) break;
		}
		journal.jprintf("\n");
#endif
		OW_scan_flags = 0;
	}
}

// Установить синхронизацию по NTP
void MainClass::set_updateNTP(boolean b)
{
	if (b) SETBIT1(DateTime.flags,fUpdateNTP); else SETBIT0(DateTime.flags,fUpdateNTP);
}

// Получить флаг возможности синхронизации по NTP
boolean MainClass::get_updateNTP()
{
  return GETBIT(DateTime.flags,fUpdateNTP);
}

// Получить источник загрузки веб морды
TYPE_SOURSE_WEB MainClass::get_SourceWeb()
{
	if(get_WebStoreOnSPIFlash()) {
		switch (get_fSPIFlash())
		{
		case 0:
			if(get_fSD() == 2) return pSD_WEB;
			break;
		case 2:
			return pFLASH_WEB;
		}
		return pMIN_WEB;
	} else {
		switch (get_fSD())
		{
		case 0:
			if(get_fSPIFlash() == 2) return pFLASH_WEB;
			break;
		case 2:
			return pSD_WEB;
		}
		return pMIN_WEB;
	}
}

// Установить значение текущий режим работы
void MainClass::set_testMode(TEST_MODE b)
{
	int i;
	for(i = 0; i < TNUMBER; i++) sTemp[i].set_testMode(b);         // датчики температуры
	for(i = 0; i < ANUMBER; i++) sADC[i].set_testMode(b);          // Датчик давления
	for(i = 0; i < INUMBER; i++) sInput[i].set_testMode(b);        // Датчики сухой контакт
	for(i = 0; i < FNUMBER; i++) sFrequency[i].set_testMode(b);    // Частотные датчики
	for(i = 0; i < RNUMBER; i++) dRelay[i].set_testMode(b);        // Реле
	testMode = b;
	// новый режим начинаем без ошибок
	eraseError();
}

// -------------------------------------------------------------------------
// СОХРАНЕНИЕ ВОССТАНОВЛЕНИЕ  ----------------------------------------------
// -------------------------------------------------------------------------
// РАБОТА с НАСТРОЙКАМИ  -----------------------------------------------------
// Записать настройки в память i2c: <size all><Option><DateTime><Network><message><dFC> <TYPE_sTemp><sTemp[TNUMBER]><TYPE_sADC><sADC[ANUMBER]><TYPE_sInput><sInput[INUMBER]>...<0x0000><CRC16>
// старотвый адрес I2C_SETTING_EEPROM
// Возвращает ошибку или число записанных байт
int32_t MainClass::save(void)
{
	uint16_t crc = 0xFFFF;
	uint32_t addr = I2C_SETTING_EEPROM + 2; // +size all
	uint8_t tasks_suspended = TaskSuspendAll(); // Запрет других задач
	if(error == ERR_SAVE_EEPROM || error == ERR_LOAD_EEPROM || error == ERR_CRC16_EEPROM) error = OK;
	DateTime.saveTime = rtcSAM3X8.unixtime();   // запомнить время сохранения настроек
	journal.jprintfopt(" Save settings ");
	while(1) {
		// Сохранить параметры и опции отопления и бойлер, уведомления
		Option.ver = VER_SAVE;
		if(save_struct(addr, (uint8_t *) &Option, sizeof(Option), crc)) break;
		if(save_struct(addr, (uint8_t *) &DateTime, sizeof(DateTime), crc)) break;
		if(save_struct(addr, (uint8_t *) &Network, sizeof(Network), crc)) break;
		if(save_struct(addr, message.get_save_addr(), message.get_save_size(), crc)) break;
		// Сохранение отдельных объектов 
		if(save_2bytes(addr, SAVE_TYPE_sTemp, crc)) break;
		for(uint8_t i = 0; i < TNUMBER; i++) if(save_struct(addr, sTemp[i].get_save_addr(), sTemp[i].get_save_size(), crc)) break; // Сохранение датчиков температуры
		if(error == ERR_SAVE_EEPROM) break;
		if(save_2bytes(addr, SAVE_TYPE_sADC, crc)) break;
		for(uint8_t i = 0; i < ANUMBER; i++) if(save_struct(addr, sADC[i].get_save_addr(), sADC[i].get_save_size(), crc)) break; // Сохранение датчика давления
		if(error == ERR_SAVE_EEPROM) break;
		if(save_2bytes(addr, SAVE_TYPE_sInput, crc)) break;
		for(uint8_t i = 0; i < INUMBER; i++) if(save_struct(addr, sInput[i].get_save_addr(), sInput[i].get_save_size(), crc)) break; // Сохранение контактных датчиков
		if(error == ERR_SAVE_EEPROM) break;
		if(save_2bytes(addr, SAVE_TYPE_sFrequency, crc)) break;
		for(uint8_t i = 0; i < FNUMBER; i++) if(save_struct(addr, sFrequency[i].get_save_addr(), sFrequency[i].get_save_size(), crc)) break; // Сохранение контактных датчиков
		if(error == ERR_SAVE_EEPROM) break;
		#ifdef MQTT
		if(save_2bytes(addr, SAVE_TYPE_clMQTT, crc)) break;
		if(save_struct(addr, clMQTT.get_save_addr(), clMQTT.get_save_size(), crc)) break; // Сохранение MQTT
		#endif
		#ifdef CORRECT_POWER220
		if(save_2bytes(addr, SAVE_TYPE_PwrCorr, crc)) break;
		if(save_struct(addr, (uint8_t*)&correct_power220, sizeof(correct_power220), crc)) break; // Сохранение correct_power220
		#endif
		if(save_2bytes(addr, SAVE_TYPE_END, crc)) break;
		if(writeEEPROM_I2C(addr, (uint8_t *) &crc, sizeof(crc))) { error = ERR_SAVE_EEPROM; break; } // CRC
		addr = addr + sizeof(crc) - (I2C_SETTING_EEPROM + 2);
		if(writeEEPROM_I2C(I2C_SETTING_EEPROM, (uint8_t *) &addr, 2)) { error = ERR_SAVE_EEPROM; break; } // size all
		if((error = check_crc16_eeprom(I2C_SETTING_EEPROM + 2, addr - 2)) != OK) {
			journal.jprintfopt("- Verify Error!\n");
			break;
		}
		addr += 2;
		journal.jprintfopt("OK, wrote: %d bytes, crc: %04x\n", addr, crc);
		break;
	}
	if(tasks_suspended) xTaskResumeAll(); // Разрешение других задач

	if(error == ERR_SAVE_EEPROM || error == ERR_LOAD_EEPROM || error == ERR_CRC16_EEPROM) {
		set_Error(error, (char*)__FUNCTION__);
		return error;
	}
	// суммарное число байт
	return addr;
}

// Считать настройки из памяти i2c или из RAM, если не NULL, на выходе длина или код ошибки (меньше нуля)
int32_t MainClass::load(uint8_t *buffer, uint8_t from_RAM)
{
	uint16_t size;
	journal.jprintfopt(" Load settings ");
	if(from_RAM == 0) {
		if(readEEPROM_I2C(I2C_SETTING_EEPROM, (byte*) &size, sizeof(size))) {
x_ReadError:
			error = ERR_CRC16_EEPROM;
x_Error:
			journal.jprintf(" I2C - read error %d!\n", error);
			return error;
		}
		if(size > I2C_SETTING_EEPROM_NEXT - I2C_SETTING_EEPROM) { error = ERR_BAD_LEN_EEPROM; goto x_Error; }
		if(readEEPROM_I2C(I2C_SETTING_EEPROM + sizeof(size), buffer, size)) goto x_ReadError;
	} else {
		journal.jprintfopt("FILE");
		size = *((uint16_t *) buffer);
		buffer += sizeof(size);
	}
	journal.jprintfopt(", size %d, crc: ", size + 2); // sizeof(crc)
	size -= 2;
	#ifdef LOAD_VERIFICATION
	
	uint16_t crc = 0xFFFF;
	for(uint16_t i = 0; i < size; i++)  crc = _crc16(crc, buffer[i]);
	if(crc != *((uint16_t *)(buffer + size))) {
		journal.jprintf("I2C Error: %04x != %04x!\n", crc, *((uint16_t *)(buffer + size)));
		return error = ERR_CRC16_EEPROM;
	}
	journal.jprintfopt("%04x", crc);
	#else
	journal.jprintfopt("*No verification");
	#endif
	uint8_t *buffer_max = buffer + size;
	size += 2;
	load_struct(&Option, &buffer, sizeof(Option));
	journal.jprintfopt(", v.%d ", Option.ver);
	if(Option.ver > 100) {
		journal.jprintf("\nEEPROM with garbage - reseting!\n");
		resetSetting();
		return error = ERR_CRC16_EEPROM;
	}
	load_struct(&DateTime, &buffer, sizeof(DateTime));
	load_struct(&Network, &buffer, sizeof(Network));
	load_struct(message.get_save_addr(), &buffer, message.get_save_size());
	int16_t type = SAVE_TYPE_LIMIT;
	while(buffer_max > buffer) { // динамические структуры
		if(*((int16_t *) buffer) <= SAVE_TYPE_END && *((int16_t *) buffer) > SAVE_TYPE_LIMIT) {
			type = *((int16_t *) buffer);
			buffer += 2;
		}
		// массивы, длина структуры должна быть меньше 128 байт, <size[1]><<number>struct>
		uint8_t n = buffer[1]; // номер элемента
		if(type == SAVE_TYPE_sTemp) { // первый в структуре идет номер датчика
			if(n < TNUMBER) { load_struct(sTemp[n].get_save_addr(), &buffer, sTemp[n].get_save_size());	sTemp[n].after_load(); } else goto xSkip;
		} else if(type == SAVE_TYPE_sADC) {
			if(n < ANUMBER) { load_struct(sADC[n].get_save_addr(), &buffer, sADC[n].get_save_size()); sADC[n].after_load(); } else goto xSkip;
		} else if(type == SAVE_TYPE_sInput) {
			if(n < INUMBER) load_struct(sInput[n].get_save_addr(), &buffer, sInput[n].get_save_size()); else goto xSkip;
		} else if(type == SAVE_TYPE_sFrequency) {
			if(n < FNUMBER) load_struct(sFrequency[n].get_save_addr(), &buffer, sFrequency[n].get_save_size()); else goto xSkip;
		// не массивы <size[1|2]><struct>
#ifdef MQTT
		} else if(type == SAVE_TYPE_clMQTT) {
			load_struct(clMQTT.get_save_addr(), &buffer, clMQTT.get_save_size());
#endif
#ifdef CORRECT_POWER220
		} else if(type == SAVE_TYPE_PwrCorr) {
			load_struct((uint8_t*)&correct_power220, &buffer, sizeof(correct_power220));
#endif
		} else if(type == SAVE_TYPE_END) {
			break;
		} else {
xSkip:		load_struct(NULL, &buffer, 0); // skip unknown type
		}
	}
	journal.jprintfopt("OK\n");
	return size + sizeof(crc);
}

// Проверить контрольную сумму в EEPROM для данных на выходе ошибка, длина определяется из заголовка
int8_t MainClass::check_crc16_eeprom(int32_t addr, uint16_t size)
{
	uint8_t x;
	uint16_t crc = 0xFFFF;
	while(size--) {
		if(readEEPROM_I2C(addr++, &x, sizeof(x))) return ERR_LOAD_EEPROM;
		crc = _crc16(crc, x);
	}
	uint16_t crc2;
	if(readEEPROM_I2C(addr, (uint8_t *)&crc2, sizeof(crc2))) return ERR_LOAD_EEPROM;   // чтение -2 за вычетом CRC16 из длины
	if(crc == crc2) return OK; else return ERR_CRC16_EEPROM;
}

// СЧЕТЧИКИ -----------------------------------
// Write only changed bytes, between changed blocks more than 4 bytes.
int8_t MainClass::save_WorkStats()
{
	WorkStats.Header = I2C_COUNT_EEPROM_HEADER;
	uint32_t ptr = 0;
	do {
		while(ptr < sizeof(WorkStats)) if(((uint8_t*)&WorkStats)[ptr] == ((uint8_t*)&WorkStats_saved)[ptr]) ptr++; else break;
		if(ptr == sizeof(WorkStats)) break;
		uint32_t ptre = ptr + 1;
xNotEqual:
		while(ptre < sizeof(WorkStats)) if(((uint8_t*)&WorkStats)[ptre] != ((uint8_t*)&WorkStats_saved)[ptre]) ptre++; else break;
		if(ptre < sizeof(WorkStats)) {
			uint32_t ptrz = ptre + 1;
			while(ptrz < sizeof(WorkStats)) if(((uint8_t*)&WorkStats)[ptrz] == ((uint8_t*)&WorkStats_saved)[ptrz]) ptrz++; else break;
			if(ptrz < sizeof(WorkStats)) {
				if(ptrz - ptre <= 4) {
					ptre = ptrz + 1;
					goto xNotEqual;
				}
			}
		}
		uint8_t errcode;
		if((errcode = writeEEPROM_I2C(I2C_COUNT_EEPROM + ptr, (byte*)&WorkStats + ptr, ptre - ptr))) {
			journal.jprintf(" ERROR %d save counters!\n", errcode);
			set_Error(ERR_SAVE2_EEPROM, (char*) __FUNCTION__);
			return ERR_SAVE2_EEPROM;
		}
		ptr = ptre;
	} while(ptr < sizeof(WorkStats));
	memcpy(&WorkStats_saved, &WorkStats, sizeof(WorkStats_saved));
	return OK;
}

// чтение счетчиков в ЕЕПРОМ
int8_t MainClass::load_WorkStats()
{
	if(readEEPROM_I2C(I2C_COUNT_EEPROM, &WorkStats.Header, sizeof(WorkStats.Header))) { // прочитать заголовок
		set_Error(ERR_LOAD2_EEPROM, (char*) __FUNCTION__);
		return ERR_LOAD2_EEPROM;
	}
	if(WorkStats.Header != I2C_COUNT_EEPROM_HEADER) { // заголовок плохой
		journal.jprintf("Bad header counters, skip load\n");
		return ERR_HEADER2_EEPROM;
	}
	if(readEEPROM_I2C(I2C_COUNT_EEPROM + sizeof(WorkStats.Header), (byte*) &WorkStats + sizeof(WorkStats.Header), sizeof(WorkStats) - sizeof(WorkStats.Header))) { // прочитать счетчики
		set_Error(ERR_LOAD2_EEPROM, (char*) __FUNCTION__);
		return ERR_LOAD2_EEPROM;
	}
	memcpy(&WorkStats_saved, &WorkStats, sizeof(WorkStats_saved));
	journal.jprintfopt(" Load counters OK, read: %d bytes\n", sizeof(WorkStats));
	return OK;

}
// Сборос сезонного счетчика моточасов
// параметр true - сброс всех счетчиков
void MainClass::resetCount()
{
	memset(&WorkStats, 0, sizeof(WorkStats));
	WorkStats.ResetTime = rtcSAM3X8.unixtime();           // Дата сброса счетчиков
	save_WorkStats();  // записать счетчики
	Stats_Power_work = 0;
	Stats_FeedPump_work = 0;
	Stats_WaterBooster_work = 0;
	Stats_WaterRegen_work = 0;
}

// Обновление счетчиков моточасов, вызывается раз в минуту
void MainClass::updateCount()
{
	int32_t p;
	//taskENTER_CRITICAL();
	p = Stats_Power_work;
	Stats_Power_work = 0;
	//taskEXIT_CRITICAL();
	p /= 1000;
	//WorkStats.E1 += p;
	//WorkStats.E2 += p;
	//taskENTER_CRITICAL();
	//p = motohour_OUT_work;
	//motohour_OUT_work = 0;
	//taskEXIT_CRITICAL();
	p /= 1000;
	//WorkStats.P1 += p;
	//WorkStats.P2 += p;
}

// После любого изменения часов необходимо пересчитать все времна которые используются
// параметр изменение времени - корректировка
void MainClass::updateDateTime(int32_t dTime)
{
	if(dTime != 0)                                   // было изменено время, надо скорректировать переменные времени
	{
		if(timeON > 0) timeON = timeON + dTime;                               // время включения контроллера для вычисления UPTIME
		if(countNTP > 0) countNTP = countNTP + dTime;                           // число секунд с последнего обновления по NTP
	}
}
      

// -------------------------------------------------------------------------
// НАСТРОЙКИ  ------------------------------------------------------------
// -------------------------------------------------------------------------
// Сброс настроек
void MainClass::resetSetting()
{  
	NO_Power = 0;

	fSD = false;                                    // СД карта не рабоатет

	num_resW5200 = 0;                               // текущее число сбросов сетевого чипа
	num_resMutexSPI = 0;                            // текущее число сброса митекса SPI
	num_resMutexI2C = 0;                            // текущее число сброса митекса I2C
	num_resMQTT = 0;                                // число повторных инициализация MQTT клиента
	num_resPing = 0;                                // число не прошедших пингов

	// Инициализациия различных времен
	DateTime.saveTime = 0;                          // дата и время сохранения настроек в eeprom
	timeON = 0;                                     // время включения контроллера для вычисления UPTIME
	countNTP = 0;                                   // число секунд с последнего обновления по NTP
	timeNTP = 0;                                    // Время обновления по NTP в тиках (0-сразу обновляемся)

	safeNetwork = false;                            // режим safeNetwork

	// Установка сетевых параметров по умолчанию
	if(defaultDHCP) SETBIT1(Network.flags, fDHCP);
	else SETBIT0(Network.flags, fDHCP); // использование DHCP
	Network.ip = IPAddress(defaultIP);              // ip адрес
	Network.sdns = IPAddress(defaultSDNS);          // сервер dns
	Network.gateway = IPAddress(defaultGateway);    // шлюз
	Network.subnet = IPAddress(defaultSubnet);      // подсеть
	Network.port = defaultPort;                     // порт веб сервера по умолчанию
	memcpy(Network.mac, defaultMAC, 6);             // mac адрес
	Network.resSocket = 30;                         // Время очистки сокетов
	Network.resW5200 = 0;                           // Время сброса чипа
	countResSocket = 0;                             // Число сбросов сокетов
	SETBIT1(Network.flags, fInitW5200);           // Ежеминутный контроль SPI для сетевого чипа
	SETBIT0(Network.flags, fPass);                 // !save! Использование паролей
	strcpy(Network.passUser, "user");              // !save! Пароль пользователя
	strcpy(Network.passAdmin, "admin");            // !save! Пароль администратора
	Network.sizePacket = 1465;                      // !save! размер пакета для отправки
	SETBIT0(Network.flags, fNoAck);                // !save! флаг Не ожидать ответа ACK
	Network.delayAck = 10;                          // !save! задержка мсек перед отправкой пакета
	strcpy(Network.pingAdr, PING_SERVER);         // !save! адрес для пинга
	Network.pingTime = 60 * 60;                       // !save! время пинга в секундах
	SETBIT0(Network.flags, fNoPing);               // !save! Запрет пинга контроллера

	// Время
	SETBIT0(DateTime.flags, fUpdateNTP);           // Обновление часов по NTP
	SETBIT1(DateTime.flags, fUpdateI2C);           // Обновление часов I2C

	strcpy(DateTime.serverNTP, (char*) NTP_SERVER);  // NTP сервер по умолчанию
	DateTime.timeZone = TIME_ZONE;                   // Часовой пояс

	// Временные задержки
	Option.ver = VER_SAVE;
	Option.tChart = 60;                  //  период накопления статистики по умолчанию 60 секунд
	Option.flags = (1<<fDebugToJournal);
	SETBIT1(Option.flags, fBeep);         //  Звук
	SETBIT1(Option.flags, fHistory);      //  Сброс статистика на карту

	Option.DrainTime = 20;
	Option.FeedPumpMaxFlow = 4000;
	Option.BackWashFeedPumpMaxFlow = 8000;
	Option.FillingTankTimeout = 60;
	Option.FloodingDebounceTime = 10;
	Option.FloodingTimeout = 600;
	Option.MinDrainLiters = 10;
	Option.MinPumpOnTime = 1000;
	Option.MinRegenLiters = 500;
	Option.MinWaterBoostOnTime = 2000;
	Option.MinWaterBoostOffTime = 10000;
	Option.PWATER_RegMin = 300;
	Option.PWM_StartingTime = 500;
	Option.PWM_DryRun = 800;
	Option.PWM_Max = 1500;
	Option.RegenHour = 3;
	Option.DaysBeforeRegen = 20;
	Option.UsedBeforeRegen = 6000;
	Option.UsedBeforeRegenSoftener = 3500;
	Option.LTANK_Empty = 500;
	Option.Weight_Empty = 500;
	Option.DrainAfterNoConsume = 12;
	Option.DrainTime = 11;
	Option.FillingTankTimeout = 30;
	Option.CriticalErrorsTimeout = 300;
	Option.BackWashFeedPumpDelay = 7*60;
}

// --------------------------------------------------------------------
// ФУНКЦИИ РАБОТЫ С НАСТРОЙКАМИ  ------------------------------------
// --------------------------------------------------------------------
// Сетевые настройки --------------------------------------------------
//Установить параметр из строки
boolean MainClass::set_network(char *var, char *c)
{ 
	 int32_t x = atoi(c);
	 if(strcmp(var,net_IP)==0){          return parseIPAddress(c, '.', Network.ip);                 }else
	 if(strcmp(var,net_DNS)==0){        return parseIPAddress(c, '.', Network.sdns);               }else
	 if(strcmp(var,net_GATEWAY)==0){     return parseIPAddress(c, '.', Network.gateway);            }else
	 if(strcmp(var,net_SUBNET)==0){      return parseIPAddress(c, '.', Network.subnet);             }else
	 if(strcmp(var,net_DHCP)==0){        if (strcmp(c,cZero)==0) { SETBIT0(Network.flags,fDHCP); return true;}
	                                    else if (strcmp(c,cOne)==0) { SETBIT1(Network.flags,fDHCP);  return true;}
	                                    else return false;
	                                    }else
	 if(strcmp(var,net_MAC)==0){         return parseBytes(c, ':', Network.mac, 6, 16);             }else
	 if(strcmp(var,net_RES_SOCKET)==0){ switch (x)
				                       {
				                        case 0: Network.resSocket=0;     return true;  break;
				                        case 1: Network.resSocket=10;    return true;  break;
				                        case 2: Network.resSocket=30;    return true;  break;
				                        case 3: Network.resSocket=90;    return true;  break;
				                        default:                    return false; break;
				                       }                                          }else
	 if(strcmp(var,net_RES_W5200)==0){ switch (x)
				                       {
				                        case 0: Network.resW5200=0;        return true;  break;
				                        case 1: Network.resW5200=60*60*6;  return true;  break;   // 6 часов хранение в секундах
				                        case 2: Network.resW5200=60*60*24; return true;  break;   // 24 часа
				                        default:                      return false; break;
				                       }                                          }else
	 if(strcmp(var,net_PASS)==0){        if (x == 0) { SETBIT0(Network.flags,fPass); return true;}
	                                    else if (x == 1) {SETBIT1(Network.flags,fPass);  return true;}
	                                    else return false;
	                                    }else
	 if(strcmp(var,net_PASSUSER)==0){    strcpy(Network.passUser,c);set_hashUser(); return true;   }else
	 if(strcmp(var,net_PASSADMIN)==0){   strcpy(Network.passAdmin,c);set_hashAdmin(); return true; }else
	 if(strcmp(var, net_fWebLogError) == 0) { Network.flags = (Network.flags & ~(1<<fWebLogError)) | ((x == 1)<<fWebLogError); return true; } else
	 if(strcmp(var, net_fWebFullLog) == 0) { Network.flags = (Network.flags & ~(1<<fWebFullLog)) | ((x == 1)<<fWebFullLog); return true; } else
	 if(strcmp(var,net_SIZE_PACKET)==0){
	                                    if((x<64)||(x>2048)) return   false;
	                                    else Network.sizePacket=x; return true;
	                                    }else
	 if(strcmp(var,net_INIT_W5200)==0){    // флаг Ежеминутный контроль SPI для сетевого чипа
	                       if (x == 0) { SETBIT0(Network.flags,fInitW5200); return true;}
	                       else if (x == 1) { SETBIT1(Network.flags,fInitW5200);  return true;}
	                       else return false;
	                       }else
	 if(strcmp(var,net_PORT)==0){
	                       if((x<1)||(x>65535)) return false;
	                       else Network.port=x; return  true;
	                       }else
	 if(strcmp(var,net_NO_ACK)==0){      if (x == 0) { SETBIT0(Network.flags,fNoAck); return true;}
	                       else if (x == 1) { SETBIT1(Network.flags,fNoAck);  return true;}
	                       else return false;
	                       }else
	 if(strcmp(var,net_DELAY_ACK)==0){
	                       if((x<1)||(x>50)) return        false;
	                       else Network.delayAck=x; return  true;
	                       }else
	 if(strcmp(var,net_PING_ADR)==0){     if (strlen(c)<sizeof(Network.pingAdr)) { strcpy(Network.pingAdr,c); return true;} else return false; }else
	 if(strcmp(var,net_PING_TIME)==0){switch (x)
				                       {
				                        case 0: Network.pingTime=0;        return true;  break;
				                        case 1: Network.pingTime=1*60;     return true;  break;
				                        case 2: Network.pingTime=5*60;     return true;  break;
				                        case 3: Network.pingTime=20*60;    return true;  break;
				                        case 4: Network.pingTime=60*60;    return true;  break;
				                        case 5: Network.pingTime=120*60;    return true;  break;
				                        default:                           return false; break;
				                       }                                          }else
	 if(strcmp(var,net_NO_PING)==0){     if (x == 0) { SETBIT0(Network.flags,fNoPing);      pingW5200(get_NoPing()); return true;}
	                       else if (x == 1) { SETBIT1(Network.flags,fNoPing); pingW5200(get_NoPing()); return true;}
	                       else return false;
	                       }else
	   return false;
}
// Сетевые настройки --------------------------------------------------
//Получить параметр из строки
char* MainClass::get_network(char *var,char *ret)
{
 if(strcmp(var,net_IP)==0){   return strcat(ret,IPAddress2String(Network.ip));          }else  
 if(strcmp(var,net_DNS)==0){      return strcat(ret,IPAddress2String(Network.sdns));   }else
 if(strcmp(var,net_GATEWAY)==0){   return strcat(ret,IPAddress2String(Network.gateway));}else                
 if(strcmp(var,net_SUBNET)==0){    return strcat(ret,IPAddress2String(Network.subnet)); }else  
 if(strcmp(var,net_DHCP)==0){      if (GETBIT(Network.flags,fDHCP)) return  strcat(ret,(char*)cOne);
                                   else      return  strcat(ret,(char*)cZero);          }else
 if(strcmp(var,net_MAC)==0){       return strcat(ret,MAC2String(Network.mac));          }else  
 if(strcmp(var,net_RES_SOCKET)==0){
 	return web_fill_tag_select(ret, "never:0;10 sec:0;30 sec:0;90 sec:0;",
				Network.resSocket == 0 ? 0 :
				Network.resSocket == 10 ? 1 :
				Network.resSocket == 30 ? 2 :
				Network.resSocket == 90 ? 3 : 4);
 }else if(strcmp(var,net_RES_W5200)==0){
	 	return web_fill_tag_select(ret, "never:0;6 hours:0;24 hours:0;",
					Network.resW5200 == 0 ? 0 :
					Network.resW5200 == 60*60*6 ? 1 :
					Network.resW5200 == 60*60*24 ? 2 : 3);
  }else if(strcmp(var,net_PASS)==0){      if (GETBIT(Network.flags,fPass)) return  strcat(ret,(char*)cOne);
                                    else      return  strcat(ret,(char*)cZero);               }else
  if(strcmp(var, net_fWebLogError) == 0) { return strcat(ret, (char*)(Network.flags & (1<<fWebLogError) ? cOne : cZero)); } else
  if(strcmp(var, net_fWebFullLog) == 0) { return strcat(ret, (char*)(Network.flags & (1<<fWebFullLog) ? cOne : cZero)); } else
  if(strcmp(var,net_PASSUSER)==0){  return strcat(ret,Network.passUser);                      }else                 
  if(strcmp(var,net_PASSADMIN)==0){ return strcat(ret,Network.passAdmin);                     }else   
  if(strcmp(var,net_SIZE_PACKET)==0){return _itoa(Network.sizePacket,ret);          }else   
    if(strcmp(var,net_INIT_W5200)==0){if (GETBIT(Network.flags,fInitW5200)) return  strcat(ret,(char*)cOne);       // флаг Ежеминутный контроль SPI для сетевого чипа
                                      else      return  strcat(ret,(char*)cZero);               }else      
    if(strcmp(var,net_PORT)==0){return _itoa(Network.port,ret);                       }else    // Порт веб сервера
    if(strcmp(var,net_NO_ACK)==0){    if (GETBIT(Network.flags,fNoAck)) return  strcat(ret,(char*)cOne);
                                      else      return  strcat(ret,(char*)cZero);          }else     
    if(strcmp(var,net_DELAY_ACK)==0){return _itoa(Network.delayAck,ret);         }else    
    if(strcmp(var,net_PING_ADR)==0){  return strcat(ret,Network.pingAdr);                  }else
    if(strcmp(var,net_PING_TIME)==0){
    	return web_fill_tag_select(ret, "never:0;1 min:0;5 min:0;20 min:0;60 min:0;120 min:0;",
					Network.pingTime == 0 ? 0 :
					Network.pingTime == 1*60 ? 1 :
					Network.pingTime == 5*60 ? 2 :
					Network.pingTime == 20*60 ? 3 :
					Network.pingTime == 60*60 ? 4 :
					Network.pingTime == 120*60 ? 5 : 6);
    } else if(strcmp(var,net_NO_PING)==0){if (GETBIT(Network.flags,fNoPing)) return  strcat(ret,(char*)cOne);
                                   else      return  strcat(ret,(char*)cZero);                        }else                                                                                          

 return strcat(ret,(char*)cInvalid);
}

// Установить параметр дата и время из строки
boolean MainClass::set_datetime(char *var, char *c)
{
	float tz;
	int16_t m, h, d, mo, y;
	int16_t buf[4];
	uint32_t oldTime = rtcSAM3X8.unixtime(); // запомнить время
	int32_t dTime = 0;
	if(strcmp(var, time_TIME) == 0) {
		if(!parseInt16_t(c, ':', buf, 2, 10)) return false;
		h = buf[0];
		m = buf[1];
		rtcSAM3X8.set_time(h, m, 0);  // внутренние
		setTime_RtcI2C(rtcSAM3X8.get_hours(), rtcSAM3X8.get_minutes(), rtcSAM3X8.get_seconds()); // внешние
		dTime = rtcSAM3X8.unixtime() - oldTime; // получить изменение времени
	} else if(strcmp(var, time_DATE) == 0) {
		char ch, f = 0;
		m = 0;
		do { // ищем разделитель чисел
			ch = c[m++];
			if(ch >= '0' && ch <= '9') f = 1;
			else if(f == 1) {
				f = 2;
				break;
			}
		} while(ch != '\0');
		if(f != 2 || !parseInt16_t(c, ch, buf, 3, 10)) return false;
		d = buf[0];
		mo = buf[1];
		y = buf[2];
		rtcSAM3X8.set_date(d, mo, y); // внутренние
		setDate_RtcI2C(rtcSAM3X8.get_days(), rtcSAM3X8.get_months(), rtcSAM3X8.get_years()); // внешние
		dTime = rtcSAM3X8.unixtime() - oldTime; // получить изменение времени
	} else if(strcmp(var, time_NTP) == 0) {
		if(strlen(c) == 0) return false;                                                 // пустая строка
		if(strlen(c) > NTP_SERVER_LEN) return false;                                     // слишком длиная строка
		else {
			strcpy(DateTime.serverNTP, c);
			return true;
		}                           // ок сохраняем
	} else if(strcmp(var, time_UPDATE) == 0) {
		if(strcmp(c, cZero) == 0) {
			SETBIT0(DateTime.flags, fUpdateNTP);
			return true;
		} else if(strcmp(c, cOne) == 0) {
			SETBIT1(DateTime.flags, fUpdateNTP);
			countNTP = 0;
			return true;
		} else return false;
	} else if(strcmp(var, time_TIMEZONE) == 0) {
		tz = my_atof(c);
		if(tz == -9876543.00f) return false;
		else if((tz < -12) || (tz > 12)) return false;
		else DateTime.timeZone = (int) tz;
		return true;
	} else if(strcmp(var, time_UPDATE_I2C) == 0) {
		if(strcmp(c, cZero) == 0) {
			SETBIT0(DateTime.flags, fUpdateI2C);
			return true;
		} else if(strcmp(c, cOne) == 0) {
			SETBIT1(DateTime.flags, fUpdateI2C);
			countNTP = 0;
			return true;
		} else return false;
	} else return false;

	if(dTime != 0) updateDateTime(dTime);    // было изменено время, надо скорректировать переменные времени
	return true;
}
// Получить параметр дата и время из строки
void MainClass::get_datetime(char *var, char *ret)
{
	if(strcmp(var, time_TIME) == 0) {
		ret += strlen(ret);
		NowTimeToStr(ret);
		ret[5] = '\0';
	} else if(strcmp(var, time_DATE) == 0) {
		strcat(ret, NowDateToStr());
	} else if(strcmp(var, time_NTP) == 0) {
		strcat(ret, DateTime.serverNTP);
	} else if(strcmp(var, time_UPDATE) == 0) {
		if(GETBIT(DateTime.flags, fUpdateNTP)) strcat(ret, (char*) cOne); else strcat(ret, (char*) cZero);
	} else if(strcmp(var, time_TIMEZONE) == 0) {
		_itoa(DateTime.timeZone, ret);
	} else if(strcmp(var, time_UPDATE_I2C) == 0) {
		if(GETBIT(DateTime.flags, fUpdateI2C)) strcat(ret, (char*) cOne);
		else strcat(ret, (char*) cZero);
	} else strcat(ret, (char*) cInvalid);
}

// Установить опции  из числа (float), "set_Opt"
boolean MainClass::set_option(char *var, float xx)
{
   int32_t x = (int32_t) xx;
   if(strcmp(var,option_TIME_CHART)==0)      { if(x>0) { startChart(); Option.tChart = x; return true; } else return false; } else // Сбросить статистистику, начать отсчет заново
   if(strcmp(var,option_BEEP)==0)            {if (x==0) {SETBIT0(Option.flags,fBeep); return true;} else if (x==1) {SETBIT1(Option.flags,fBeep); return true;} else return false;  }else            // Подача звукового сигнала
   if(strcmp(var,option_History)==0)         {if (x==0) {SETBIT0(Option.flags,fHistory); return true;} else if (x==1) {SETBIT1(Option.flags,fHistory); return true;} else return false;       }else       // Сбрасывать статистику на карту
   if(strcmp(var,option_WebOnSPIFlash)==0)   { Option.flags = (Option.flags & ~(1<<fWebStoreOnSPIFlash)) | ((x!=0)<<fWebStoreOnSPIFlash); return true; } else
   if(strcmp(var,option_LogWirelessSensors)==0){ Option.flags = (Option.flags & ~(1<<fLogWirelessSensors)) | ((x!=0)<<fLogWirelessSensors); return true; } else
   if(strcmp(var,option_fDontRegenOnWeekend)==0){ Option.flags = (Option.flags & ~(1<<fDontRegenOnWeekend)) | ((x!=0)<<fDontRegenOnWeekend); return true; } else
   if(strcmp(var,option_fDebugToSerialOn)==0) { Option.flags = (Option.flags & ~(1<<fDebugToJournal)) | (((DebugToJournalOn = x)!=0)<<fDebugToJournal); return true; } else
   if(strcmp(var,option_FeedPumpMaxFlow)==0) { Option.FeedPumpMaxFlow = x; return true; } else
   if(strcmp(var,option_BackWashFeedPumpMaxFlow)==0){ Option.BackWashFeedPumpMaxFlow = x; return true; } else
   if(strcmp(var,option_BackWashFeedPumpDelay)==0){ Option.BackWashFeedPumpDelay = x; return true; } else
   if(strcmp(var,option_RegenHour)==0)       { Option.RegenHour = x; return true; } else
   if(strcmp(var,option_DaysBeforeRegen)==0) { Option.DaysBeforeRegen = x; return true; } else
   if(strcmp(var,option_UsedBeforeRegen)==0) { Option.UsedBeforeRegen = x; return true; } else
   if(strcmp(var,option_UsedBeforeRegenSoftener)==0) { Option.UsedBeforeRegenSoftener = x; return true; } else
   if(strcmp(var,option_MinPumpOnTime)==0)   { Option.MinPumpOnTime = x; return true; } else
   if(strcmp(var,option_MinWaterBoostOnTime)==0){ Option.MinWaterBoostOnTime = x; return true; } else
   if(strcmp(var,option_MinWaterBoostOffTime)==0){ Option.MinWaterBoostOffTime = x; return true; } else
   if(strcmp(var,option_MinRegen)==0)        { Option.MinRegenLiters = x; return true; } else
   if(strcmp(var,option_MinDrain)==0)        { Option.MinDrainLiters = x; return true; } else
   if(strcmp(var,option_DrainAfterNoConsume)==0){ Option.DrainAfterNoConsume = x * 60 * 60; return true; } else
   if(strcmp(var,option_DrainTime)==0)       { Option.DrainTime = x; return true; } else
   if(strcmp(var,option_PWM_LOG_ERR)==0)     { Option.flags = (Option.flags & ~(1<<fPWMLogErrors)) | ((x!=0)<<fPWMLogErrors); return true; } else
   if(strcmp(var,option_PWM_DryRun)==0)      { Option.PWM_DryRun = x; return true; } else
   if(strcmp(var,option_PWM_Max)==0)         { Option.PWM_Max = x; return true; } else
   if(strcmp(var,option_PWM_StartingTime)==0){ Option.PWM_StartingTime = x; return true; } else
   if(strcmp(var,option_PWATER_RegMin)==0)   { Option.PWATER_RegMin = rd(xx, 100); return true; } else
   if(strcmp(var,option_LTANK_Empty)==0)     { Option.LTANK_Empty = rd(xx, 100); return true; } else
   if(strcmp(var,option_Weight_Empty)==0)    { Option.Weight_Empty = rd(xx, 100); return true; } else
   if(strcmp(var,option_FloodingDebounceTime)==0){ Option.FloodingDebounceTime = x; return true; } else
   if(strcmp(var,option_FloodingTimeout)==0) { Option.FloodingTimeout = x; return true; } else
   if(strcmp(var,option_FillingTankTimeout)==0){ Option.FillingTankTimeout = x; return true; } else
   if(strcmp(var,option_CriticalErrorsTimeout)==0){ Option.CriticalErrorsTimeout = x; return true; } else
   if(strncmp(var,option_SGL1W, sizeof(option_SGL1W)-1)==0) {
	   uint8_t bit = var[sizeof(option_SGL1W)-1] - '0' - 1;
	   if(bit <= 3) {
		   Option.flags = (Option.flags & ~(1<<(f1Wire1TSngl + bit))) | (x == 0 ? 0 : (1<<(f1Wire1TSngl + bit)));
		   return true;
	   }
   }
   return false; 
}

// Получить опции , результат добавляется в ret, "get_Opt"
char* MainClass::get_option(char *var, char *ret)
{
   if(strcmp(var,option_TIME_CHART)==0)       {return _itoa(Option.tChart,ret);} else
   if(strcmp(var,option_BEEP)==0)             {if(GETBIT(Option.flags,fBeep)) return strcat(ret,(char*)cOne); else return strcat(ret,(char*)cZero); }else            // Подача звукового сигнала
   if(strcmp(var,option_History)==0)          {if(GETBIT(Option.flags,fHistory)) return strcat(ret,(char*)cOne); else return strcat(ret,(char*)cZero);   }else            // Сбрасывать статистику на карту
   if(strcmp(var,option_WebOnSPIFlash)==0)    { return strcat(ret, (char*)(GETBIT(Option.flags, fWebStoreOnSPIFlash) ? cOne : cZero)); } else
   if(strcmp(var,option_LogWirelessSensors)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fLogWirelessSensors) ? cOne : cZero)); } else
   if(strcmp(var,option_fDontRegenOnWeekend)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fDontRegenOnWeekend) ? cOne : cZero)); } else
   if(strcmp(var,option_fDebugToSerialOn)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fDebugToJournal) ? cOne : cZero)); } else
   if(strcmp(var,option_FeedPumpMaxFlow)==0){ return _itoa(Option.FeedPumpMaxFlow, ret); } else
   if(strcmp(var,option_BackWashFeedPumpMaxFlow)==0){ return _itoa(Option.BackWashFeedPumpMaxFlow, ret); } else
   if(strcmp(var,option_BackWashFeedPumpDelay)==0){ return _itoa(Option.BackWashFeedPumpDelay, ret); } else
   if(strcmp(var,option_RegenHour)==0){ return _itoa(Option.RegenHour, ret); } else
   if(strcmp(var,option_DaysBeforeRegen)==0){ return _itoa(Option.DaysBeforeRegen, ret); } else
   if(strcmp(var,option_UsedBeforeRegen)==0){ return _itoa(Option.UsedBeforeRegen, ret); } else
   if(strcmp(var,option_UsedBeforeRegenSoftener)==0){ return _itoa(Option.UsedBeforeRegenSoftener, ret); } else
   if(strcmp(var,option_MinPumpOnTime)==0){ return _itoa((uint32_t)Option.MinPumpOnTime, ret); } else
   if(strcmp(var,option_MinWaterBoostOnTime)==0){ return _itoa((uint32_t)Option.MinWaterBoostOnTime, ret); } else
   if(strcmp(var,option_MinWaterBoostOffTime)==0){ return _itoa((uint32_t)Option.MinWaterBoostOffTime, ret); } else
   if(strcmp(var,option_MinRegen)==0){ return _itoa(Option.MinRegenLiters, ret); } else
   if(strcmp(var,option_MinDrain)==0){ return _itoa(Option.MinDrainLiters, ret); } else
   if(strcmp(var,option_DrainAfterNoConsume)==0){ return _itoa(Option.DrainAfterNoConsume / (60 * 60), ret); } else
   if(strcmp(var,option_DrainTime)==0){ return _itoa(Option.DrainTime, ret); } else
   if(strcmp(var,option_PWM_LOG_ERR)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fPWMLogErrors) ? cOne : cZero)); } else
   if(strcmp(var,option_PWM_DryRun)==0){ return _itoa(Option.PWM_DryRun, ret); } else
   if(strcmp(var,option_PWM_Max)==0){ return _itoa(Option.PWM_Max, ret); } else
   if(strcmp(var,option_PWM_StartingTime)==0){ return _itoa(Option.PWM_StartingTime, ret); } else
   if(strcmp(var,option_PWATER_RegMin)==0){ _dtoa(ret, Option.PWATER_RegMin, 2); return ret; } else
   if(strcmp(var,option_LTANK_Empty)==0){ _dtoa(ret, Option.LTANK_Empty, 2); return ret; } else
   if(strcmp(var,option_Weight_Empty)==0){ _dtoa(ret, Option.Weight_Empty, 2); return ret; } else
   if(strcmp(var,option_FloodingDebounceTime)==0){ return _itoa(Option.FloodingDebounceTime, ret); } else
   if(strcmp(var,option_FloodingTimeout)==0){ return _itoa(Option.FloodingTimeout, ret); } else
   if(strcmp(var,option_FillingTankTimeout)==0){ return _itoa(Option.FillingTankTimeout, ret); } else
   if(strcmp(var,option_CriticalErrorsTimeout)==0){ return _itoa(Option.CriticalErrorsTimeout, ret); } else
   if(strncmp(var,option_SGL1W, sizeof(option_SGL1W)-1)==0) {
	   uint8_t bit = var[sizeof(option_SGL1W)-1] - '0' - 1;
	   if(bit <= 3) {
		   return strcat(ret,(char*)(GETBIT(Option.flags, f1Wire1TSngl + bit) ? cOne : cZero));
	   }
   }
   return  strcat(ret,(char*)cInvalid);                
}

// Получить строку состояния  в виде строки
void MainClass::StateToStr(char * ret)
{
	if(error != OK) strcat(ret, "Сбой!");
	else if(get_testMode() != NORMAL) strcat(ret, "Тест!");
	else strcat(ret, "В работе");
}

// получить режим тестирования
char * MainClass::TestToStr()
{
 switch ((int)get_testMode())
             {
              case NORMAL:    return (char*)"NORMAL";    break;
              case SAFE_TEST: return (char*)"SAFE_TEST"; break;
              case TEST:      return (char*)"TEST";      break;
              case HARD_TEST: return (char*)"HARD_TEST"; break;
              default:        return (char*)cError;     break;
             }
}

// --------------------------------------------------------------------
// ФУНКЦИИ РАБОТЫ С ГРАФИКАМИ  -----------------------------------
// --------------------------------------------------------------------
// получить список доступных графиков в виде строки
// cat true - список добавляется в конец, false - строка обнуляется и список добавляется
char * MainClass::get_listChart(char* str)
{
	uint8_t i;
	strcat(str,"none:1;");
	for(i=0;i<TNUMBER;i++) if(sTemp[i].Chart.get_present()) {strcat(str,sTemp[i].get_name()); strcat(str,":0;");}
	for(i=0;i<ANUMBER;i++) if(sADC[i].Chart.get_present()) { strcat(str,sADC[i].get_name()); strcat(str,":0;");}
	for(i=0;i<FNUMBER;i++) if(sFrequency[i].Chart.get_present()) { strcat(str,sFrequency[i].get_name()); strcat(str,":0;");}
	strcat(str, chart_BrineWeight); strcat(str,":0;");
	strcat(str, chart_WaterBoost); strcat(str,":0;");
	strcat(str, chart_FeedPump); strcat(str,":0;");
	strcat(str, chart_FillTank); strcat(str,":0;");
	if(dPWM.ChartVoltage.get_present()) {   strcat(str,chart_VOLTAGE); strcat(str,":0;"); }
	if(dPWM.ChartPower.get_present())   {   strcat(str,chart_fullPOWER); strcat(str,":0;"); }
	return str;
}

// обновить статистику, добавить одну точку и если надо записать ее на карту.
// Все значения в графиках целочислены (сотые), выводятся в формате 0.01
void  MainClass::updateChart()
{
	for(uint8_t i=0;i<TNUMBER;i++) if(sTemp[i].Chart.get_present())  sTemp[i].Chart.addPoint(sTemp[i].get_Temp());
	for(uint8_t i=0;i<ANUMBER;i++) if(sADC[i].Chart.get_present()) sADC[i].Chart.addPoint(sADC[i].get_Value());
	for(uint8_t i=0;i<FNUMBER;i++) if(sFrequency[i].Chart.get_present()) sFrequency[i].Chart.addPoint(sFrequency[i].get_Value() / 10); // Частотные датчики

	int32_t tmp1, tmp2, tmp3;
	taskENTER_CRITICAL();
	tmp1 = Charts_WaterBooster_work;
	Charts_WaterBooster_work = 0;
	tmp2 = Charts_FeedPump_work;
	Charts_FeedPump_work = 0;
	tmp3 = Charts_FillTank_work;
	Charts_FillTank_work = 0;
	taskEXIT_CRITICAL();
	ChartWaterBoost.addPoint(tmp1 / 10);
	ChartFeedPump.addPoint(tmp2 / 10);
	ChartFillTank.addPoint(tmp3 / 10);
	ChartBrineWeight.addPoint(Weight_Percent);
	if(dPWM.ChartVoltage.get_present())   dPWM.ChartVoltage.addPoint(dPWM.get_Voltage() / 10);
	if(dPWM.ChartPower.get_present())     dPWM.ChartPower.addPoint(dPWM.get_Power() / 10);
}

// сбросить графики в ОЗУ
void MainClass::startChart()
{
 uint8_t i; 
 for(i=0;i<TNUMBER;i++) sTemp[i].Chart.clear();
 for(i=0;i<ANUMBER;i++) sADC[i].Chart.clear();
 for(i=0;i<FNUMBER;i++) sFrequency[i].Chart.clear();
 ChartWaterBoost.clear();
 ChartFeedPump.clear();
 ChartFillTank.clear();
 ChartFillTank.clear();
 dPWM.ChartVoltage.clear();                              // Статистика по напряжению
 dPWM.ChartPower.clear();                                // Статистика по Полная мощность
}

// получить данные графика  в виде строки, данные ДОБАВЛЯЮТСЯ к str
void MainClass::get_Chart(char *var, char* str)
{
	uint8_t i;
	if(strcmp(var, chart_NONE) == 0) {
		strcat(str, "");
		return;
	}
	// В начале имена совпадающие с именами объектов
	for(i = 0; i < TNUMBER; i++) {
		if((strcmp(var, sTemp[i].get_name()) == 0) && (sTemp[i].Chart.get_present())) {
			sTemp[i].Chart.get_PointsStrDiv100(str);
			return;
		}
	}
	for(i = 0; i < ANUMBER; i++) {
		if((strcmp(var, sADC[i].get_name()) == 0) && (sADC[i].Chart.get_present())) {
			sADC[i].Chart.get_PointsStrDiv100(str);
			return;
		}
	}
	for(i = 0; i < FNUMBER; i++) {
		if((strcmp(var, sFrequency[i].get_name()) == 0) && (sFrequency[i].Chart.get_present())) {
			sFrequency[i].Chart.get_PointsStrDiv100(str);
			return;
		}
	}
	if(strcmp(var, chart_VOLTAGE) == 0) {
		dPWM.ChartVoltage.get_PointsStr(str);
	} else if(strcmp(var, chart_fullPOWER) == 0) {
		dPWM.ChartPower.get_PointsStrDiv100(str);
	} else if(strcmp(var, chart_WaterBoost) == 0) {
		ChartWaterBoost.get_PointsStrDiv100(str);
	} else if(strcmp(var, chart_FeedPump) == 0) {
		ChartFeedPump.get_PointsStrDiv100(str);
	} else if(strcmp(var, chart_FillTank) == 0) {
		ChartFillTank.get_PointsStrDiv100(str);
	} else if(strcmp(var, chart_BrineWeight) == 0) {
		ChartBrineWeight.get_PointsStrDiv100(str);
	}
}

// расчитать хеш для пользователя возвращает длину хеша
uint8_t MainClass::set_hashUser()
{
	char buf[20];
	strcpy(buf, NAME_USER);
	strcat(buf, ":");
	strcat(buf, Network.passUser);
	base64_encode(Security.hashUser, buf, strlen(buf));
	Security.hashUserLen = strlen(Security.hashUser);
	journal.jprintfopt(" Hash user: %s\n", Security.hashUser);
	return Security.hashUserLen;
}
// расчитать хеш для администратора возвращает длину хеша
uint8_t MainClass::set_hashAdmin()
{
	char buf[20];
	strcpy(buf, NAME_ADMIN);
	strcat(buf, ":");
	strcat(buf, Network.passAdmin);
	base64_encode(Security.hashAdmin, buf, strlen(buf));
	Security.hashAdminLen = strlen(Security.hashAdmin);
	journal.jprintfopt(" Hash admin: %s\n", Security.hashAdmin);
	return Security.hashAdminLen;
}

// --------------------------------------------------------------------------------------------------------
// ---------------------------------- ОСНОВНЫЕ ФУНКЦИИ ------------------------------------------
// --------------------------------------------------------------------------------------------------------

// Все реле выключить
void MainClass::relayAllOFF()
{
	uint8_t i;
	journal.jprintf(" All relay off\n");
	for(i = 0; i < RNUMBER; i++) dRelay[i].set_OFF();         // Выключить все реле;
}                               


// Обновление расчетных величин мощностей и СОР
void MainClass::calculatePower()
{
#ifdef CORRECT_POWER220
	for(uint8_t i = 0; i < sizeof(correct_power220)/sizeof(correct_power220[0]); i++) if(dRelay[correct_power220[i].num].get_Relay()) power220 += correct_power220[i].value;
#endif
}

// Записать состояние в журнал
// Параметр на входе true - вывод в журнал и консоль false - консоль
void MainClass::save_DumpJournal(void)
{
	uint8_t i;
	journal.jprintf(" State:");
	for(i = 0; i < RNUMBER; i++) journal.jprintf(" %s:%d", MC.dRelay[i].get_name(), MC.dRelay[i].get_Relay());
	for(i = 0; i < TNUMBER; i++) if(sTemp[i].get_present() && sTemp[i].Chart.get_present()) journal.jprintfopt(" %s:%.2d", sTemp[i].get_name(), sTemp[i].get_Temp());
	journal.jprintf(" Power:%.3d\n", dPWM.get_Power());
	// Доп инфо
	for(i = 0; i < ANUMBER; i++) if(sADC[i].get_present()) journal.jprintf(" %s:%.2d", sADC[i].get_name(), sADC[i].get_Value());
	for(i = 0; i < INUMBER; i++) if(sInput[i].get_present()) journal.jprintf(" %s:%d", sInput[i].get_name(), sInput[i].get_Input());
	journal.jprintf(cStrEnd);
}

// Возвращает 0 - Нет ошибок или ни одного активного датчика, 1 - ошибка, 2 - превышен предел ошибок
int8_t MainClass::Prepare_Temp(uint8_t bus)
{
	int8_t i, ret = 0;
  #ifdef ONEWIRE_DS2482_SECOND
	if(bus == 1) i = OneWireBus2.PrepareTemp();
	else
  #endif
  #ifdef ONEWIRE_DS2482_THIRD
	if(bus == 2) i = OneWireBus3.PrepareTemp();
	else
  #endif
  #ifdef ONEWIRE_DS2482_FOURTH
	if(bus == 3) i = OneWireBus4.PrepareTemp();
	else
  #endif
	i = OneWireBus.PrepareTemp();
	if(i) {
		for(uint8_t j = 0; j < TNUMBER; j++) {
			if(sTemp[j].get_fAddress() && sTemp[j].get_bus() == bus) {
				if(sTemp[j].inc_error()) {
					ret = 2;
					break;
				}
				ret = 1;
			}
		}
		if(ret) {
			journal.jprintfopt_time("Error %d PrepareTemp bus %d\n", i, bus+1);
			if(ret == 2) set_Error(i, (char*) __FUNCTION__);
		}
	}
	return ret ? (1<<bus) : 0;
}
