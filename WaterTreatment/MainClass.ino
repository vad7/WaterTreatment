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
// Описание базового класса для работы
// --------------------------------------------------------------------------------
#include "MainClass.h"

/* structsize
  char checker(int);
  char checkSizeOfInt1[sizeof(MC.Option.DailySwitch)]={checker(&checkSizeOfInt1)};
  char checkSizeOfInt1[sizeof(MC.WorkStats)]={checker(&checkSizeOfInt1)};
//*/

// Установка критической ошибки, Возвращает ошибку останова, если nam == NULL короткая запись в лог.
int8_t set_Error(int8_t _err, char *nam)
{
	if(MC.error != _err) {
		MC.error = _err;
		if(nam) strncpy(MC.source_error, nam, sizeof(MC.source_error)-1);
		m_snprintf(MC.note_error, sizeof(MC.note_error), "%s %s: %s", NowTimeToStr(), nam != NULL ? nam : "", noteError[abs(_err)]);
	}
	uint32_t i = Get_Errors_IndexEnd(_err);
	if(i != ERRORS_ARR_SIZE) {
		Errors[i] = _err;
		ErrorsTime[i] = rtcSAM3X8.unixtime();
		if(nam) {
			journal.jprintf_date("$ERROR source: %s, code: %d (0x%X)\n", nam, _err, CriticalErrors);
			if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
				journal.jprintf(" State:");
				for(i = 0; i < RNUMBER; i++) journal.jprintf(" %s:%d", MC.dRelay[i].get_name(), MC.dRelay[i].get_Relay());
				for(i = 0; i < INUMBER; i++) if(MC.sInput[i].get_present()) journal.jprintf(" %s:%d", MC.sInput[i].get_name(), MC.sInput[i].get_Input());
				journal.jprintf("\n Power:%d", MC.dPWM.get_Power());
#ifdef CHECK_DRAIN_PUMP
				journal.jprintf(",%d", DrainPumpPower);
#endif
#ifdef CHECK_SEPTIC
				journal.jprintf(",%d", SepticPower);
#endif
				for(i = 0; i < ANUMBER; i++) if(MC.sADC[i].get_present()) journal.jprintf(" %s:%.2d", MC.sADC[i].get_name(), MC.sADC[i].get_Value());
				for(i = 0; i < TNUMBER; i++) if(MC.sTemp[i].get_present()) journal.jprintf(" %s:%.2d", MC.sTemp[i].get_name(), MC.sTemp[i].get_Temp());
				journal.jprintf(" Weight:%.2d%%", Weight_Percent);
				if(_err == ERR_WEIGHT_LOW) {
					journal.jprintf("(%.1d=%d) [", Weight_value, Weight_adc_sum / (Weight_adc_flagFull ? WEIGHT_AVERAGE_BUFFER : Weight_adc_idx));
					for(i = 0; i < WEIGHT_AVERAGE_BUFFER; i++) journal.jprintf("%d,", Weight_adc_filter[i]);
					journal.jprintf("] %d", Weight_adc_idx);
				} else if(_err == ERR_PWM_DRY_RUN || _err == ERR_PWM_MAX) {
					journal.jprintf(" WBTime:%d", WaterBoosterTimeout);

				}
				journal.jprintf("\n");
			}
		} else journal.jprintf_date("$ERROR code: %d (0x%X)\n", _err, CriticalErrors);
		if(i > 0 && ErrorsTime[i] - LastErrorsClearManual < 60 * 2)	Error_Beep_confirmed = true;// Убрать звук и не посылать ошибку, если прошло меньше 2х минут после очистки ошибок.
		else MC.message.setMessage(pMESSAGE_ERROR, MC.note_error, 0);    // сформировать уведомление об ошибке
	}
	return _err;
}

// Возврат номера свободного места в массиве ошибок или размер массива, если он переполнен или ошибка уже в нем есть
uint32_t Get_Errors_IndexEnd(int8_t _err)
{
	uint32_t i = 0;
	for(; i < ERRORS_ARR_SIZE; i++) {
		if(Errors[i] == OK) break;
		if(Errors[i] == _err) {
			i = ERRORS_ARR_SIZE;
			break;
		}
	}
	return i;
}

// Стереть последнюю ошибку
void MainClass::clear_error()
{
	if(!CriticalErrors) {
		strcpy(note_error, "OK");			// Строка c описанием ошибки
		source_error[0] = '\0';				// Источник ошибки
		error = OK;                         // Код ошибки
	}
}

// стереть все ошибки
void MainClass::clear_all_errors()
{
	for(int i = 0; i < ERRORS_ARR_SIZE; i++) {
		if(error == ERR_SALT_FINISH) {
			MC.WorkStats.RegenSofteningCntAlarm = MC.Option.RegenSofteningCntAlarm;
			NeedSaveWorkStats = 1;
#if defined(CHECK_DRAIN_PUMP) && !defined(MODBUS_DRAIN_PUMP_ON_PULSE)
		} else if(error == ERR_DRAIN_PUMP_TOOLONG || error == ERR_DRAIN_PUMP_OVERLOAD || error == ERR_DRAIN_PUMP_DRAIN_RUN) {
			DrainPumpRelayStatus = MODBUS_RELAY_CMD_ON;
#endif
#if defined(CHECK_SEPTIC) && !defined(MODBUS_SEPTIC_PUMP_ON_PULSE)
		} else if(error == ERR_SEPTIC_PUMP_TOOLONG || error == ERR_SEPTIC_PUMP_OVERLOAD || error == ERR_SEPTIC_PUMP_DRAIN_RUN) {
			SepticPumpRelayStatus = MODBUS_RELAY_CMD_ON;
#endif
		} else if(error == ERR_SEPTIC_PUMP_NOT_WORK) {
			UsedWaterToSeptic = 0;
#if !defined(MODBUS_SEPTIC_PUMP_ON_PULSE)
			SepticPumpRelayStatus = MODBUS_RELAY_CMD_ON;
#endif
		}
	}
	memset(Errors, 0, sizeof(Errors));
	memset(ErrorsTime, 0, sizeof(ErrorsTime));
	CriticalErrors = 0;
	MC.clear_error();
	// Water ON
	MC.dRelay[RWATEROFF1].set_OFF();
	MC.dRelay[RWATERON].set_ON();
}

void Weight_Clear_Averaging(void)
{
	memset(Weight_adc_filter, 0, sizeof(Weight_adc_filter));
	Weight_adc_sum = 0;
	Weight_adc_flagFull = false;
	Weight_adc_idx = 0;
}

// Read successfully - true
bool Weight_Read(bool skip_error)
{
//		if(Weight_NeedRead) { // allways
			if(MC.get_testMode() != NORMAL) {
//				Weight_NeedRead = false;
				if(Weight_value != Weight_Test) {
					Weight_value = Weight_Test;
					Weight_Percent = Weight_value * 1000 / MC.Option.WeightFull;
				}
			} else if(Weight.is_ready()) { // 10Hz or 80Hz
//				Weight_NeedRead = false;
				// Read HX711
				int32_t adc_val, median3 = Weight.read();
				if(median3 < MC.Option.WeightZero) {
					if(Weight_Wrong_ADC_Cnt < WEIGHT_SKIP_WRONG_ADC_NUM) {
						Weight_Wrong_ADC_Cnt++;
						return false;
					}
				} else Weight_Wrong_ADC_Cnt = 0;
				// Медианный фильтр
				if(Weight_adc_median1 <= Weight_adc_median2 && Weight_adc_median1 <= median3) {
					adc_val = Weight_adc_median2 <= median3 ? Weight_adc_median2 : median3;
				} else if(Weight_adc_median2 <= Weight_adc_median1 && Weight_adc_median2 <= median3) {
					adc_val = Weight_adc_median1 <= median3 ? Weight_adc_median1 : median3;
				} else {
					adc_val = Weight_adc_median1 <= Weight_adc_median2 ? Weight_adc_median1 : Weight_adc_median2;
				}
				Weight_adc_median1 = Weight_adc_median2;
				Weight_adc_median2 = median3;
//				if(GETBIT(MC.Option.flags, fDebugToSerial)) journal.printf("HX711[%u]: %d=%d", GetTickCount(), median3, adc_val);
				// Усреднение значений
				Weight_adc_sum = Weight_adc_sum + adc_val - Weight_adc_filter[Weight_adc_idx];
				Weight_adc_filter[Weight_adc_idx] = adc_val;
				if(Weight_adc_idx < WEIGHT_AVERAGE_BUFFER - 1) Weight_adc_idx++;
				else {
					Weight_adc_idx = 0;
					Weight_adc_flagFull = true;
				}
				if(Weight_adc_flagFull) adc_val = Weight_adc_sum / WEIGHT_AVERAGE_BUFFER; else adc_val = Weight_adc_sum / Weight_adc_idx;
//				if(GETBIT(MC.Option.flags, fDebugToSerial)) journal.printf("=%d\n", adc_val);
				Weight_value = (int64_t)(adc_val - MC.Option.WeightZero) * 10000 / MC.Option.WeightScale - MC.Option.WeightTare;
				Weight_Percent = Weight_value * 1000 / MC.Option.WeightFull;
				if(Weight_Percent < 0) Weight_Percent = 0; else if(Weight_Percent > 10000) Weight_Percent = 10000;
				if(!skip_error) {
					if(Weight_Percent < MC.Option.Weight_Low) {
						if(!(CriticalErrors & ERRC_WeightEmpty)) {
							if(Weight_Percent <= 10) {
								CriticalErrors |= ERRC_WeightEmpty;
								set_Error(ERR_WEIGHT_EMPTY, (char*)__FUNCTION__);
							} else set_Error(ERR_WEIGHT_LOW, (char*)__FUNCTION__);
						}
					} else if(CriticalErrors & ERRC_WeightEmpty) CriticalErrors &= ~ERRC_WeightEmpty;
				}
				return true;
			}
//		}
		return false;
}

void MainClass::init()
{
	int i;
	clear_error();
	memset(&DailySwitchTimerCnt, 0, sizeof(DailySwitchTimerCnt));

	for(i = 0; i < TNUMBER; i++) sTemp[i].initTemp(i);            // Инициализация датчиков температуры

	for(i = 0; i < ANUMBER; i++) sADC[i].initSensorADC(i, pinsAnalog[i], ADC_FILTER[i]);	// Инициализация аналогового датчика

	for(i = 0; i < INUMBER; i++) sInput[i].initInput(i);           // Инициализация контактных датчиков
	for(i = 0; i < FNUMBER; i++) sFrequency[i].initFrequency(i);  // Инициализация частотных датчиков
	for(i = 0; i < RNUMBER; i++) dRelay[i].initRelay(i);           // Инициализация реле

	dRelay[RWATERON].set_ON();

	dPWM.initPWM();                                           // инициализация счетчика
	message.initMessage(MAIN_WEB_TASK);                       // Инициализация Уведомлений, параметр - номер потока сервера в котором идет отправка
#ifdef MQTT
	clMQTT.initMQTT(MAIN_WEB_TASK);                           // Инициализация MQTT, параметр - номер потока сервера в котором идет отправка
#endif

	ChartWaterBoost.init();
	ChartFeedPump.init();
	ChartFillTank.init();
	ChartBrineWeight.init();
	ChartWaterBoosterCount.init();
#ifdef CHECK_DRAIN_PUMP
	ChartDrainPump.init();
#endif
#ifdef CHECK_SEPTIC
	ChartSepticPump.init();
#endif

	resetSetting();                                           // все переменные
}

// Получить число ошибок чтения ВСЕХ датчиков темпеартуры
uint32_t MainClass::get_errorReadTemp()
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
//#ifdef ONEWIRE_DS2482_FOURTH
//		OneWireBus4.Scan(result_str);
//#endif
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
	for(i = 0; i < RNUMBER; i++) dRelay[i].set_testMode(b);        // Реле
#ifndef TEST_BOARD
	if(testMode > STAT_TEST && b <= STAT_TEST) {
		MC.load_WorkStats();
		if(rtcI2C.readRTC(RTC_STORE_ADDR, (uint8_t*)&MC.RTC_store, sizeof(MC.RTC_store))) {
			memset(&MC.RTC_store, 0, sizeof(MC.RTC_store));
			journal.jprintf(" Error read RTC store!\n");
		}
		Stats_Power_work = 0;
		Stats_WaterUsed_work = 0;
		Stats_WaterRegen_work = 0;
		Stats_FeedPump_work = 0;
		Stats_WaterBooster_work = 0;
		History_WaterUsed_work = 0;
		History_WaterRegen_work = 0;
		History_FeedPump_work = 0;
		History_WaterBooster_work = 0;
		History_BoosterCountL = -1;
		Charts_WaterBooster_work = 0;
		Charts_FeedPump_work = 0;
		Charts_FillTank_work = 0;
	}
#endif
	testMode = b;
	// новый режим начинаем без ошибок
	clear_error();
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
	journal.jprintfopt("Save settings ");
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
		int8_t _err;
		if((_err = check_crc16_eeprom(I2C_SETTING_EEPROM + 2, addr - 2)) != OK) {
			error = _err;
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
//	} else if(Option.ver == 16) { // Convert
//		memmove((uint8_t*)&Option.DailySwitch + 40, (uint8_t*)&Option.DailySwitch + 30, sizeof(Option) - ((uint8_t*)&Option.DailySwitch - (uint8_t*)&Option.ver) - 40);
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
	// Recalc variables
	FilterTankSquare = CalcFilterSquare(Option.FilterTank);
	FilterTankSoftenerSquare = CalcFilterSquare(Option.FilterTankSoftener);
	//
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
#ifndef TEST_BOARD
	if(MC.get_testMode() > STAT_TEST) return 0;
#endif
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

// чтение счетчиков из ЕЕПРОМ
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
	Stats_WaterUsed_work = 0;
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
	num_resMutexWEB = 0;                            // текущее число сброса митекса SPI
	num_resMutexI2C = 0;                            // текущее число сброса митекса I2C
	num_resMQTT = 0;                                // число повторных инициализация MQTT клиента
	num_resPing = 0;                                // число не прошедших пингов

	// Инициализациия различных времен
	DateTime.saveTime = 0;                          // дата и время сохранения настроек в eeprom
	timeON = 0;                                     // время включения контроллера для вычисления UPTIME
	countNTP = 0;                                   // число секунд с последнего обновления по NTP
	timeNTP = 0;                                    // Время обновления по NTP в тиках (0-сразу обновляемся)
	Osmos_PWATER_Cnt = 0;
	Osmos_PWATER_Last = 0;
	Osmos_PWATER_BoosterMax = PWATER_OSMOS_BoosterMax_START;
	Osmos_PWATER_Added = 0;
	NextRegenAfterDays = 255;
	NextRegenSoftAfterDays = 255;

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

	// Временные задержки
	Option.ver = VER_SAVE;
	Option.tChart = 5;
	Option.flags = (1<<fDebugToJournal);
	SETBIT1(Option.flags, fBeep);         //  Звук
	SETBIT1(Option.flags, fHistory);      //  Сброс статистика на карту

	Option.FeedPumpMaxFlow = 2000;
	Option.BackWashFeedPumpMaxFlow = 1700;
	Option.FloodingDebounceTime = 10;
	Option.FloodingTimeout = 600;
	Option.MinDrainLiters = 20;
	Option.MinPumpOnTime = 1500;
	Option.MinRegenLiters = 450;
	Option.MinWaterBoostOnTime = 4000;
	Option.MinWaterBoostOffTime = 5000;
	Option.PWATER_RegMin = 300;
	Option.PWM_StartingTime = 2000;
	Option.PWM_DryRun = 900;
	Option.PWM_Max = 2000;
	Option.RegenHour = 3;
	Option.DaysBeforeRegen = 16;
	Option.UsedBeforeRegen = 3000;
	Option.UsedBeforeRegenSoftening = 3500;
	Option.LTANK_Low = 7000;
	Option.LTank_Hour_Low = 9500;
	Option.LTank_Hour = 6;
	Option.Weight_Low = 500;
	Option.DrainAfterNoConsume = 12;
	Option.DrainTime = 15;
	Option.FillingTankTimeout = 30;
	Option.CriticalErrorsTimeout = 300;
	Option.BackWashFeedPumpDelay = 8*60;
	Option.FilterTank = 13;
	Option.FilterTankSoftener = 10;
	FilterTankSquare = CalcFilterSquare(Option.FilterTank);
	FilterTankSoftenerSquare = CalcFilterSquare(Option.FilterTankSoftener);
	Option.LowConsumeRequestPeriod = HTTP_REQ_LowConsume;
	Option.SepticAlarmDebounce = 30;
	Option.TankCheckPercent = FILLING_TANK_STEP / 10;
	Option.TankFillingTimeMax = 120;
	Option.PWATER_Osmos_TankMul = 100;
	Option.PWATER_Osmos_FullDelay = 20;
	RFILL_last_time_ON = rtcSAM3X8.unixtime();
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
   	 if(strcmp(var,net_PASSUSER)==0){    strncpy(Network.passUser,c, PASS_LEN); calc_WebSec_hash(&WebSec_user, (char*)NAME_USER, Network.passUser, var + sizeof(net_PASSUSER)); return true; }else
   	 if(strcmp(var,net_PASSADMIN)==0){   strncpy(Network.passAdmin,c, PASS_LEN); calc_WebSec_hash(&WebSec_admin, (char*)NAME_ADMIN, Network.passAdmin, var + sizeof(net_PASSADMIN)); return true; }else
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
	int16_t buf[3];
	uint32_t oldTime = rtcSAM3X8.unixtime(); // запомнить время
	int32_t dTime = 0;
	if(strcmp(var, time_TIME) == 0) {
		if(!parseInt16_t(c, ':', buf, 2, 10)) return false;
		rtcSAM3X8.set_time(buf[0], buf[1], 0);  // внутренние
		setTime_RtcI2C(rtcSAM3X8.get_hours(), rtcSAM3X8.get_minutes(), rtcSAM3X8.get_seconds()); // внешние
		dTime = rtcSAM3X8.unixtime() - oldTime; // получить изменение времени
	} else if(strcmp(var, time_DATE) == 0) {
		uint8_t i = 0, f = 0;
		char ch;
		do { // ищем разделитель чисел
			ch = c[i++];
			if(ch >= '0' && ch <= '9') f = 1;
			else if(f == 1) {
				f = 2;
				break;
			}
		} while(ch != '\0');
		if(f != 2 || !parseInt16_t(c, ch, buf, 3, 10)) return false;
		rtcSAM3X8.set_date(buf[0], buf[1], buf[2]); // внутренние
		setDate_RtcI2C(rtcSAM3X8.get_days(), rtcSAM3X8.get_months(), rtcSAM3X8.get_years()); // внешние
		dTime = rtcSAM3X8.unixtime() - oldTime; // получить изменение времени
	} else if(strcmp(var, time_NTP) == 0) {
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
	} else if(strcmp(var, time_UPDATE_I2C) == 0) {
		if(GETBIT(DateTime.flags, fUpdateI2C)) strcat(ret, (char*) cOne);
		else strcat(ret, (char*) cZero);
	} else strcat(ret, (char*) cInvalid);
}

// Установить опции  из числа (float), "set_Opt"
boolean MainClass::set_option(char *var, float xx)
{
	int32_t x = (int32_t) xx;
	if(strcmp(var,option_TIME_CHART)==0)      { if(x>0) { clearChart(); Option.tChart = x; return true; } else return false; } else // Сбросить статистистику, начать отсчет заново
	if(strcmp(var,option_BEEP)==0)            {if (x==0) {SETBIT0(Option.flags,fBeep); return true;} else if (x==1) {SETBIT1(Option.flags,fBeep); return true;} else return false;  }else            // Подача звукового сигнала
	if(strcmp(var,option_History)==0)         {if (x==0) {SETBIT0(Option.flags,fHistory); return true;} else if (x==1) {SETBIT1(Option.flags,fHistory); return true;} else return false;       }else       // Сбрасывать статистику на карту
	if(strcmp(var,option_WebOnSPIFlash)==0)   { Option.flags = (Option.flags & ~(1<<fWebStoreOnSPIFlash)) | ((x!=0)<<fWebStoreOnSPIFlash); return true; } else
	if(strcmp(var,option_LogWirelessSensors)==0){ Option.flags = (Option.flags & ~(1<<fLogWirelessSensors)) | ((x!=0)<<fLogWirelessSensors); return true; } else
	if(strcmp(var,option_fDontRegenOnWeekend)==0){ Option.flags = (Option.flags & ~(1<<fDontRegenOnWeekend)) | ((x!=0)<<fDontRegenOnWeekend); return true; } else
	if(strcmp(var,option_fDebugToJournal)==0) { Option.flags = (Option.flags & ~(1<<fDebugToJournal)) | (((DebugToJournalOn = x)!=0)<<fDebugToJournal); return true; } else
	if(strcmp(var,option_fDebugToSerial)==0) { Option.flags = (Option.flags & ~(1<<fDebugToSerial)) | ((x!=0)<<fDebugToSerial); return true; } else
	if(strcmp(var,option_fRegenAllowed)==0) { Option.flags = (Option.flags & ~(1<<fRegenAllowed)) | ((x!=0)<<fRegenAllowed); return true; } else
	if(strcmp(var,option_fRegenAllowedSoftener)==0) { Option.flags = (Option.flags & ~(1<<fRegenAllowedSoftener)) | ((x!=0)<<fRegenAllowedSoftener); return true; } else
	if(strcmp(var,option_fChartOnlyNonZeroW)==0) { Option.flags = (Option.flags & ~(1<<fChartOnlyNonZeroW)) | ((x!=0)<<fChartOnlyNonZeroW); return true; } else
	if(strcmp(var,option_FeedPumpRate)==0) { Option.FeedPumpRate = rd(xx * 1000, 1000) / 3600; return true; } else
	if(strcmp(var,option_FeedPumpMaxFlow)==0) { Option.FeedPumpMaxFlow = Option.FeedPumpRate * 1000 / rd(xx, 1000); return true; } else
	if(strcmp(var,option_fFlowIncByPressure)==0) { Option.flags = (Option.flags & ~(1<<fFlowIncByPressure)) | ((x!=0)<<fFlowIncByPressure); return true; } else
	if(strcmp(var,option_FlowIncByPress_MinFlow)==0) { Option.FlowIncByPress_MinFlow = rd(xx, 1000); return true; } else // если импульсов = xx * sFrequency[FLOW].get_kfValue() / 100
	if(strcmp(var,option_BackWashFeedPumpMaxFlow)==0){ Option.BackWashFeedPumpMaxFlow = Option.FeedPumpRate * 1000 / rd(xx, 1000); return true; } else
	if(strcmp(var,option_BackWashFeedPumpDelay)==0){ Option.BackWashFeedPumpDelay = x; return true; } else
	if(strcmp(var,option_DaysBeforeRegen)==0) { Option.DaysBeforeRegen = x; return true; } else
	if(strcmp(var,option_DaysBeforeRegenSoftening)==0) { Option.DaysBeforeRegenSoftening = x; return true; } else
	if(strcmp(var,option_UsedBeforeRegen)==0) { Option.UsedBeforeRegen = x; return true; } else
	if(strcmp(var,option_UsedBeforeRegenSoftening)==0) { Option.UsedBeforeRegenSoftening = x; return true; } else
	if(strcmp(var,option_MinPumpOnTime)==0)   { Option.MinPumpOnTime = x; return true; } else
	if(strcmp(var,option_MinWaterBoostOnTime)==0){ Option.MinWaterBoostOnTime = x; return true; } else
	if(strcmp(var,option_MinWaterBoostOffTime)==0){ Option.MinWaterBoostOffTime = x; return true; } else
	if(strcmp(var,option_MinRegen)==0)        { Option.MinRegenLiters = x; return true; } else
	if(strcmp(var,option_MinRegenSoftening)==0){ Option.MinRegenLitersSoftening = x; return true; } else
	if(strcmp(var,option_MinRegenWeightDecrease)==0){ Option.MinRegenWeightDecrease = x; return true; } else
	if(strcmp(var,option_MinRegenWeightDecreaseSoftening)==0){ Option.MinRegenWeightDecreaseSoftening = x; return true; } else
	if(strcmp(var,option_DrainAfterNoConsume)==0){ Option.DrainAfterNoConsume = x * 60 * 60; return true; } else
	if(strcmp(var,option_DrainTime)==0)       { Option.DrainTime = x; if(Option.DrainAfterNoConsume && Option.DrainTime == 0) Option.DrainTime = 1; return true; } else
	if(strcmp(var,option_MinDrain)==0)        { Option.MinDrainLiters = rd(xx, 10); return true; } else
	if(strcmp(var,option_MinWaterBoosterCountL)==0){ Option.MinWaterBoosterCountL = rd(xx, 100); return true; } else
	if(strcmp(var,option_WaterBoosterMinTank)==0){ Option.WaterBoosterMinTank = rd(xx, 100); return true; } else
	if(strcmp(var,option_PWM_LOG_ERR)==0)     { Option.flags = (Option.flags & ~(1<<fPWMLogErrors)) | ((x!=0)<<fPWMLogErrors); return true; } else
	if(strcmp(var,option_PWM_DryRun)==0)      { Option.PWM_DryRun = x; return true; } else
	if(strcmp(var,option_PWM_Max)==0)         { Option.PWM_Max = x; return true; } else
	if(strcmp(var,option_PWM_StartingTime)==0){ Option.PWM_StartingTime = x; return true; } else
	if(strcmp(var,option_PWATER_RegMin)==0)   { Option.PWATER_RegMin = rd(xx, 100); return true; } else
	if(strcmp(var,option_PWATER_Osmos_Min)==0){ Option.PWATER_Osmos_Min = rd(xx, 100); return true; } else
	if(strcmp(var,option_PWATER_Osmos_Step)==0){ Option.PWATER_Osmos_Step = x; return true; } else
	if(strcmp(var,option_PWATER_Osmos_TankMul)==0){ Option.PWATER_Osmos_TankMul = rd(xx, 100); return true; } else
	if(strcmp(var,option_PWATER_Osmos_FullDelay)==0){ Option.PWATER_Osmos_FullDelay = x; return true; } else
	if(strcmp(var,option_PWATER_Osmos_FullMinus)==0){ Option.PWATER_Osmos_FullMinus = rd(xx, 100); return true; } else
	if(strcmp(var,option_PWATER_Osmos_Delay)==0){ Option.PWATER_Osmos_Delay = x; return true; } else
	if(strcmp(var,option_LTANK_Low)==0)        { Option.LTANK_Low = rd(xx, 100); return true; } else
	if(strcmp(var,option_LTank_CriticalMax)==0){ Option.LTank_CriticalMax = x; return true; } else
	if(strcmp(var,option_LTank_LowConsumeMin)==0){ Option.LTank_LowConsumeMin = rd(xx, 100); return true; } else
	if(strcmp(var,option_LTank_AfterFilledTimer)==0){ Option.LTank_AfterFilledTimer = x; return true; } else
	if(strcmp(var,option_LTank_Hour_Low)==0)   { Option.LTank_Hour_Low = rd(xx, 100); return true; } else
	if(strcmp(var,option_LTank_Hour)==0){ Option.LTank_Hour = x; return true; } else
	if(strcmp(var,option_Weight_Low)==0)       { Option.Weight_Low = rd(xx, 100); return true; } else
	if(strcmp(var,option_FloodingDebounceTime)==0){ Option.FloodingDebounceTime = x; return true; } else
	if(strcmp(var,option_FloodingTimeout)==0)  { Option.FloodingTimeout = x; return true; } else
	if(strcmp(var,option_FillingTankTimeout)==0){ Option.FillingTankTimeout = x; return true; } else
	if(strcmp(var,option_TankCheckPercent)==0) { Option.TankCheckPercent = rd(xx, 10); return true; } else
	if(strcmp(var,option_TankFillingTimeMax)==0){ Option.TankFillingTimeMax = x; return true; } else
	if(strcmp(var,option_CriticalErrorsTimeout)==0){ Option.CriticalErrorsTimeout = x; return true; } else
	if(strcmp(var,option_FilterTank)==0){ FilterTankSquare = CalcFilterSquare(Option.FilterTank = x); return true; } else
	if(strcmp(var,option_FilterTankSoftener)==0){ FilterTankSoftenerSquare = CalcFilterSquare(Option.FilterTankSoftener = x); return true; } else
	if(strcmp(var,option_DrainingWaterAfterRegen)==0){ Option.DrainingWaterAfterRegen = x; return true; } else
	if(strcmp(var,option_DrainingWaterAfterRegenSoftening)==0){ Option.DrainingWaterAfterRegenSoftening = x; return true; } else
	if(strcmp(var,option_RegenStart)==0){ Option.DrainingWaterAfterRegen = x; return true; } else // Start regenerate
	if(strcmp(var,option_RFILL_HoursRepeatPulse)==0){ Option.RFILL_HoursRepeatPulse = x; return true; } else // Start regenerate
	if(strcmp(var,option_LowConsumeRequestPeriod)==0){ Option.LowConsumeRequestPeriod = x; Request_LowConsume = xTaskGetTickCount(); return true; } else
	if(strcmp(var,option_fLowConsumeReq_OnByErr)==0) { Option.flags = (Option.flags & ~(1<<fLowConsumeReq_OnByErr)) | ((x!=0)<<fLowConsumeReq_OnByErr); return true; } else
	if(strcmp(var,option_SepticAlarmDebounce)==0){ Option.SepticAlarmDebounce = x; return true; } else
	if(strcmp(var,option_GetCurrentSaltLevel)==0){ if(x == 1) MC.WorkStats.RegenSofteningCntAlarm = Option.RegenSofteningCntAlarm; return true; } else
	if(strcmp(var,option_DrainSiltTime)==0){ Option.DrainSiltTime = x / 10; return true; } else
	if(strcmp(var,option_DrainSiltAfterL100)==0){ Option.DrainSiltAfterL100 = x > 100 ? x / 100 : 1; return true; } else
	if(strcmp(var,option_DrainSiltAfterNotUsed)==0){ Option.DrainSiltAfterNotUsed = x; return true; } else
	if(strcmp(var,option_fDrainSiltTank)==0){ Option.flags2 = (Option.flags2 & ~(1<<fDrainSiltTank)) | ((x!=0)<<fDrainSiltTank); return true; } else
	if(strcmp(var,option_fDrainSiltTankBeforeRegen)==0){ Option.flags2 = (Option.flags2 & ~(1<<fDrainSiltTankBeforeRegen)) | ((x!=0)<<fDrainSiltTankBeforeRegen); return true; } else
	if(strcmp(var,option_UsedWaterContinuous)==0){ Option.UsedWaterContinuous = x; UsedWaterContinuousTimerMax = 0; return true; } else
	if(strcmp(var,option_FilterCounter1_Max)==0){ Option.FilterCounter1_Max = x / 100; return true; } else
	if(strcmp(var,option_FilterCounter2_Max)==0){ Option.FilterCounter2_Max = x / 100; return true; } else
#ifdef REVERSE_OSMOS_FC
	if(strcmp(var,option_RO_FilterCounter1_Max)==0){ Option.RO_FilterCounter1_Max = x / 10; return true; } else
	if(strcmp(var,option_RO_FilterCounter2_Max)==0){ Option.RO_FilterCounter2_Max = x / 10; return true; } else
#endif
	if(strcmp(var,option_PumpStartTime)==0){ Option.PumpStartTime = x; return true; } else
	if(strcmp(var,option_PumpReadPeriod)==0){ Option.PumpReadPeriod = x == 0 ? 1 : x; return true; } else
	if(strcmp(var,option_DrainPumpMaxTime)==0){ Option.DrainPumpMaxTime = x / 20; return true; } else
	if(strcmp(var,option_DrainPumpMinPower)==0){ Option.DrainPumpMinPower = x / 10; return true; } else
	if(strcmp(var,option_DrainPumpMaxPower)==0){ Option.DrainPumpMaxPower = x; return true; } else
	if(strcmp(var,option_DrainPumpDryPower)==0){ Option.DrainPumpDryPower = x; return true; } else
	if(strcmp(var,option_fDrainPumpRelay)==0){ Option.flags2 = (Option.flags2 & ~(1<<fDrainPumpRelay)) | ((x!=0)<<fDrainPumpRelay); return true; } else
	if(strcmp(var,option_SepticPumpMaxTime)==0){ Option.SepticPumpMaxTime = x / 20; return true; } else
	if(strcmp(var,option_SepticPumpMinPower)==0){ Option.SepticPumpMinPower = x / 10; return true; } else
	if(strcmp(var,option_SepticPumpMaxPower)==0){ Option.SepticPumpMaxPower = x; return true; } else
	if(strcmp(var,option_SepticPumpDryPower)==0){ Option.SepticPumpDryPower = x; return true; } else
	if(strcmp(var,option_SepticPumpConsumedMax)==0){ Option.SepticPumpConsumedMax = x; return true; } else
	if(strcmp(var,option_SepticMinPower)==0){ Option.SepticMinPower = x / 10; return true; } else
	if(strcmp(var,option_fLED_SRV_INFO_PlanReg)==0){ Option.flags2 = (Option.flags2 & ~(1<<fLED_SRV_INFO_PlanReg)) | ((x!=0)<<fLED_SRV_INFO_PlanReg); return true; } else
	if(strcmp(var,option_fCheckDrainPump)==0){
		Option.flags2 = (Option.flags2 & ~(1<<fCheckDrainPump)) | ((x!=0)<<fCheckDrainPump);
#if !defined(MODBUS_DRAIN_PUMP_ON_PULSE)
		if(!x) DrainPumpRelayStatus = MODBUS_RELAY_CMD_ON;
#endif
		return true;
	} else
	if(strcmp(var,option_fCheckSepticPump)==0){
		Option.flags2 = (Option.flags2 & ~(1<<fCheckSeptic)) | ((x!=0)<<fCheckSeptic);
#if !defined(SEPTIC_DRAIN_PUMP_ON_PULSE)
		if(!x) SepticPumpRelayStatus = MODBUS_RELAY_CMD_ON;
#endif
		return true;
	} else
	if(strcmp(var,option_fSepticPumpRelay)==0){ Option.flags2 = (Option.flags2 & ~(1<<fSepticPumpRelay)) | ((x!=0)<<fSepticPumpRelay); return true; } else
	if(strcmp(var,option_fSepticPumpRelayNoErr)==0){ Option.flags2 = (Option.flags2 & ~(1<<fSepticPumpRelayNoErr)) | ((x!=0)<<fSepticPumpRelayNoErr); return true; } else
	if(strcmp(var,option_fSepticHeatRelay)==0){
		Option.flags2 = (Option.flags2 & ~(1<<fSepticHeatRelay)) | ((x!=0)<<fSepticHeatRelay);
		if(!x) Modbus.RelaySwitch(MODBUS_SEPTIC_HEAT_RELAY_ADDR, MODBUS_SEPTIC_HEAT_RELAY_ID, MODBUS_SEPTIC_HEAT_RELAY_OFF);
		return true;
	} else
	if(strcmp(var,option_RegenSofteningCntAlarm)==0){
	   Option.RegenSofteningCntAlarm = x;
	   if(x == 0) MC.WorkStats.RegenSofteningCntAlarm = 0; else if(MC.WorkStats.RegenSofteningCntAlarm == 0) MC.WorkStats.RegenSofteningCntAlarm = x;
	   return true;
	} else
	if(strncmp(var,option_SGL1W, sizeof(option_SGL1W)-1)==0) {
	   uint8_t bit = var[sizeof(option_SGL1W)-1] - '0' - 1;
	   if(bit <= 2) {
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
	if(strcmp(var,option_fDebugToJournal)==0){ return strcat(ret, (char*)(DebugToJournalOn ? cOne : cZero)); } else
	if(strcmp(var,option_fDebugToSerial)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fDebugToSerial) ? cOne : cZero)); } else
	if(strcmp(var,option_fRegenAllowed)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fRegenAllowed) ? cOne : cZero)); } else
	if(strcmp(var,option_fRegenAllowedSoftener)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fRegenAllowedSoftener) ? cOne : cZero)); } else
	if(strcmp(var,option_fChartOnlyNonZeroW)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fChartOnlyNonZeroW) ? cOne : cZero)); } else
	if(strcmp(var,option_FeedPumpRate)==0){ _dtoa(ret, Option.FeedPumpRate * 3600 / 1000, 3); return ret; } else // лч
	if(strcmp(var,option_FeedPumpMaxFlow)==0){ _dtoa(ret, Option.FeedPumpRate * 1000 / Option.FeedPumpMaxFlow, 3); return ret; } else // мл на 1л расхода
	if(strcmp(var,option_fFlowIncByPressure)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fFlowIncByPressure) ? cOne : cZero)); } else
	if(strcmp(var,option_FlowIncByPress_MinFlow)==0){ _dtoa(ret, Option.FlowIncByPress_MinFlow, 3); return ret; } else
	if(strcmp(var,option_BackWashFeedPumpMaxFlow)==0){ _dtoa(ret, Option.FeedPumpRate * 1000 / Option.BackWashFeedPumpMaxFlow, 3); return ret; } else // мл на 1л расхода
	if(strcmp(var,option_BackWashFeedPumpDelay)==0){ return _itoa(Option.BackWashFeedPumpDelay, ret); } else
	if(strcmp(var,option_DaysBeforeRegen)==0){ return _itoa(Option.DaysBeforeRegen, ret); } else
	if(strcmp(var,option_DaysBeforeRegenSoftening)==0){ return _itoa(Option.DaysBeforeRegenSoftening, ret); } else
	if(strcmp(var,option_UsedBeforeRegen)==0){ return _itoa(Option.UsedBeforeRegen, ret); } else
	if(strcmp(var,option_UsedBeforeRegenSoftening)==0){ return _itoa(Option.UsedBeforeRegenSoftening, ret); } else
	if(strcmp(var,option_MinPumpOnTime)==0){ return _itoa((uint32_t)Option.MinPumpOnTime, ret); } else
	if(strcmp(var,option_MinWaterBoostOnTime)==0){ return _itoa((uint32_t)Option.MinWaterBoostOnTime, ret); } else
	if(strcmp(var,option_MinWaterBoostOffTime)==0){ return _itoa((uint32_t)Option.MinWaterBoostOffTime, ret); } else
	if(strcmp(var,option_MinRegen)==0){ return _itoa(Option.MinRegenLiters, ret); } else
	if(strcmp(var,option_MinRegenSoftening)==0){ return _itoa(Option.MinRegenLitersSoftening, ret); } else
	if(strcmp(var,option_MinRegenWeightDecrease)==0){ return _itoa(Option.MinRegenWeightDecrease, ret); } else
	if(strcmp(var,option_MinRegenWeightDecreaseSoftening)==0){ return _itoa(Option.MinRegenWeightDecreaseSoftening, ret); } else
	if(strcmp(var,option_MinDrain)==0){ _dtoa(ret, Option.MinDrainLiters, 1); return ret; } else
	if(strcmp(var,option_MinWaterBoosterCountL)==0){ _dtoa(ret, Option.MinWaterBoosterCountL, 2); return ret; } else
	if(strcmp(var,option_WaterBoosterMinTank)==0){ _dtoa(ret, Option.WaterBoosterMinTank, 2); return ret; } else
	if(strcmp(var,option_DrainAfterNoConsume)==0){ return _itoa(Option.DrainAfterNoConsume / (60 * 60), ret); } else
	if(strcmp(var,option_DrainTime)==0){ return _itoa(Option.DrainTime, ret); } else
	if(strcmp(var,option_PWM_LOG_ERR)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fPWMLogErrors) ? cOne : cZero)); } else
	if(strcmp(var,option_PWM_DryRun)==0){ return _itoa(Option.PWM_DryRun, ret); } else
	if(strcmp(var,option_PWM_Max)==0){ return _itoa(Option.PWM_Max, ret); } else
	if(strcmp(var,option_PWM_StartingTime)==0){ return _itoa(Option.PWM_StartingTime, ret); } else
	if(strcmp(var,option_PWATER_RegMin)==0){ _dtoa(ret, Option.PWATER_RegMin, 2); return ret; } else
	if(strcmp(var,option_PWATER_Osmos_Min)==0){ _dtoa(ret, Option.PWATER_Osmos_Min, 2); return ret; } else
	if(strcmp(var,option_PWATER_Osmos_Step)==0){ return _itoa(Option.PWATER_Osmos_Step, ret); } else
	if(strcmp(var,option_PWATER_Osmos_TankMul)==0){ _dtoa(ret, Option.PWATER_Osmos_TankMul, 2); return ret; } else
	if(strcmp(var,option_PWATER_Osmos_FullDelay)==0){ return _itoa(Option.PWATER_Osmos_FullDelay, ret); } else
	if(strcmp(var,option_PWATER_Osmos_FullMinus)==0){ _dtoa(ret, Option.PWATER_Osmos_FullMinus, 2); return ret; } else
	if(strcmp(var,option_PWATER_Osmos_Delay)==0){ return _itoa(Option.PWATER_Osmos_Delay, ret); } else
	if(strcmp(var,option_LTANK_Low)==0){ _dtoa(ret, Option.LTANK_Low, 2); return ret; } else
	if(strcmp(var,option_LTank_LowConsumeMin)==0){ _dtoa(ret, Option.LTank_LowConsumeMin, 2); return ret; } else
	if(strcmp(var,option_LTank_Hour_Low)==0){ _dtoa(ret, Option.LTank_Hour_Low, 2); return ret; } else
	if(strcmp(var,option_LTank_Hour)==0){ return _itoa(Option.LTank_Hour, ret); } else
	if(strcmp(var,option_Weight_Low)==0){ _dtoa(ret, Option.Weight_Low, 2); return ret; } else
	if(strcmp(var,option_FloodingDebounceTime)==0){ return _itoa(Option.FloodingDebounceTime, ret); } else
	if(strcmp(var,option_FloodingTimeout)==0){ return _itoa(Option.FloodingTimeout, ret); } else
	if(strcmp(var,option_FillingTankTimeout)==0){ return _itoa(Option.FillingTankTimeout, ret); } else
	if(strcmp(var,option_TankCheckPercent)==0){ _dtoa(ret, Option.TankCheckPercent, 1); return ret; } else
	if(strcmp(var,option_TankFillingTimeMax)==0){ return _itoa(Option.TankFillingTimeMax, ret); } else
	if(strcmp(var,option_LTank_CriticalMax)==0){ return _itoa(Option.LTank_CriticalMax, ret); } else
	if(strcmp(var,option_CriticalErrorsTimeout)==0){ return _itoa(Option.CriticalErrorsTimeout, ret); } else
	if(strcmp(var,option_FilterTank)==0){ return _itoa(Option.FilterTank, ret); } else
	if(strcmp(var,option_FilterTankSoftener)==0){ return _itoa(Option.FilterTankSoftener, ret); } else
	if(strcmp(var,option_DrainingWaterAfterRegen)==0){ return _itoa(Option.DrainingWaterAfterRegen, ret); } else
	if(strcmp(var,option_DrainingWaterAfterRegenSoftening)==0){ return _itoa(Option.DrainingWaterAfterRegenSoftening, ret); } else
	if(strcmp(var,option_LTank_AfterFilledTimer)==0){ return _itoa(Option.LTank_AfterFilledTimer, ret); } else
	if(strcmp(var,option_LowConsumeRequestPeriod)==0){ return _itoa(Option.LowConsumeRequestPeriod, ret); } else
	if(strcmp(var,option_fLowConsumeReq_OnByErr)==0){ return strcat(ret, (char*)(GETBIT(Option.flags, fLowConsumeReq_OnByErr) ? cOne : cZero)); } else
	if(strcmp(var,option_SepticAlarmDebounce)==0){ return _itoa(Option.SepticAlarmDebounce, ret); } else
	if(strcmp(var,option_RegenSofteningCntAlarm)==0){ return _itoa(Option.RegenSofteningCntAlarm, ret); } else
	if(strcmp(var,option_RFILL_HoursRepeatPulse)==0){ return _itoa(Option.RFILL_HoursRepeatPulse, ret); } else
	if(strcmp(var,option_GetCurrentSaltLevel)==0){ return _itoa(MC.WorkStats.RegenSofteningCntAlarm * 100L / Option.RegenSofteningCntAlarm, ret); } else
	if(strcmp(var,option_DrainSiltTime)==0){ return _itoa(Option.DrainSiltTime * 10, ret); } else
	if(strcmp(var,option_DrainSiltAfterL100)==0){ return _itoa(Option.DrainSiltAfterL100 * 100, ret); } else
	if(strcmp(var,option_DrainSiltAfterNotUsed)==0){ return _itoa(Option.DrainSiltAfterNotUsed, ret); } else
	if(strcmp(var,option_fDrainSiltTank)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fDrainSiltTank) ? cOne : cZero)); } else
	if(strcmp(var,option_fDrainSiltTankBeforeRegen)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fDrainSiltTankBeforeRegen) ? cOne : cZero)); } else
	if(strcmp(var,option_UsedWaterContinuous)==0){ return _itoa(Option.UsedWaterContinuous, ret); } else
	if(strcmp(var,option_FilterCounter1_Max)==0){ return _itoa(Option.FilterCounter1_Max * 100, ret); } else
	if(strcmp(var,option_FilterCounter2_Max)==0){ return _itoa(Option.FilterCounter2_Max * 100, ret); } else
#ifdef REVERSE_OSMOS_FC
	if(strcmp(var,option_RO_FilterCounter1_Max)==0){ return _itoa(Option.RO_FilterCounter1_Max * 10, ret); } else
	if(strcmp(var,option_RO_FilterCounter2_Max)==0){ return _itoa(Option.RO_FilterCounter2_Max * 10, ret); } else
	if(strcmp(var,option_REVERSE_OSMOS_F1_END_STR)==0){ return strcat(ret, REVERSE_OSMOS_F1_END_STR); } else
	if(strcmp(var,option_REVERSE_OSMOS_F2_END_STR)==0){ return strcat(ret, REVERSE_OSMOS_F2_END_STR); } else
#else
	if(strcmp(var,option_RO_FilterCounter1_Max)==0){ return strcat(ret, "-"); } else
	if(strcmp(var,option_RO_FilterCounter2_Max)==0){ return strcat(ret, "-"); } else
#endif
	if(strcmp(var,option_PumpStartTime)==0){ return _itoa(Option.PumpStartTime, ret); } else
	if(strcmp(var,option_PumpReadPeriod)==0){ return _itoa(Option.PumpReadPeriod, ret); } else
	if(strcmp(var,option_DrainPumpMaxTime)==0){ return _itoa(Option.DrainPumpMaxTime * 20, ret); } else
	if(strcmp(var,option_DrainPumpMinPower)==0){ return _itoa(Option.DrainPumpMinPower * 10, ret); } else
	if(strcmp(var,option_DrainPumpMaxPower)==0){ return _itoa(Option.DrainPumpMaxPower, ret); } else
	if(strcmp(var,option_DrainPumpDryPower)==0){ return _itoa(Option.DrainPumpDryPower, ret); } else
	if(strcmp(var,option_fCheckDrainPump)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fCheckDrainPump) ? cOne : cZero)); } else
	if(strcmp(var,option_fDrainPumpRelay)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fDrainPumpRelay) ? cOne : cZero)); } else
	if(strcmp(var,option_SepticPumpMaxTime)==0){ return _itoa(Option.SepticPumpMaxTime * 20, ret); } else
	if(strcmp(var,option_SepticPumpMinPower)==0){ return _itoa(Option.SepticPumpMinPower * 10, ret); } else
	if(strcmp(var,option_SepticPumpMaxPower)==0){ return _itoa(Option.SepticPumpMaxPower, ret); } else
	if(strcmp(var,option_SepticPumpDryPower)==0){ return _itoa(Option.SepticPumpDryPower, ret); } else
	if(strcmp(var,option_SepticPumpConsumedMax)==0){ return _itoa(Option.SepticPumpConsumedMax, ret); } else
	if(strcmp(var,option_SepticMinPower)==0){ return _itoa(Option.SepticMinPower * 10, ret); } else
	if(strcmp(var,option_fCheckSepticPump)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fCheckSeptic) ? cOne : cZero)); } else
	if(strcmp(var,option_fSepticPumpRelay)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fSepticPumpRelay) ? cOne : cZero)); } else
	if(strcmp(var,option_fSepticPumpRelayNoErr)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fSepticPumpRelayNoErr) ? cOne : cZero)); } else
	if(strcmp(var,option_fSepticHeatRelay)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fSepticHeatRelay) ? cOne : cZero)); } else
	if(strcmp(var,option_fLED_SRV_INFO_PlanReg)==0){ return strcat(ret, (char*)(GETBIT(Option.flags2, fLED_SRV_INFO_PlanReg) ? cOne : cZero)); } else
	if(strncmp(var, prof_DailySwitch, sizeof(prof_DailySwitch)-1) == 0) {
		var += sizeof(prof_DailySwitch)-1;
		uint8_t i = *(var + 1) - '0';
		if(i >= DAILY_SWITCH_MAX) return false;
		if(*var == prof_DailySwitchDevice) {
			_itoa(Option.DailySwitch[i].Device == 0 ? 0 : (Option.DailySwitch[i].Device & ~(1<<DS_TimerBit)) - DAILY_RELAY_START_FROM + 1, ret);
		} else if(*var == prof_DailySwitchState) {
			if(GETBIT(MC.DailySwitchTimerCnt[i], DS_StatusBit)) strcat(ret, "Вкл ");
			if(GETBIT(MC.Option.DailySwitch[i].Device, DS_TimerBit)) { strcat(ret, "-"); _itoa(MC.DailySwitchTimerCnt[i] & ~(1<<DS_StatusBit), ret); }
		} else if(*var == prof_DailySwitchType) {
			if(GETBIT(MC.Option.DailySwitch[i].Device, DS_TimerBit)) strcat(ret, "Таймер");
		} else if(*var == prof_DailySwitchOn) {
			if(GETBIT(MC.Option.DailySwitch[i].Device, DS_TimerBit)) _itoa(Option.DailySwitch[i].TimeOn ? Option.DailySwitch[i].TimeOn * DS_TimerMin : 1, ret);
			else m_snprintf(ret + strlen(ret), 32, "%02d:%d0", Option.DailySwitch[i].TimeOn / 10, Option.DailySwitch[i].TimeOn % 10);
		} else if(*var == prof_DailySwitchOff) {
			if(GETBIT(MC.Option.DailySwitch[i].Device, DS_TimerBit)) _itoa(Option.DailySwitch[i].TimeOff ? Option.DailySwitch[i].TimeOff * DS_TimerMin : 1, ret);
			else m_snprintf(ret + strlen(ret), 32, "%02d:%d0", Option.DailySwitch[i].TimeOff / 10, Option.DailySwitch[i].TimeOff % 10);
		}
		return ret;
	} else
	if(strncmp(var,option_SGL1W, sizeof(option_SGL1W)-1)==0) {
	   uint8_t bit = var[sizeof(option_SGL1W)-1] - '0' - 1;
	   if(bit <= 2) {
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
	case STAT_TEST:      return (char*)"TEST";      break;
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
	strcat(str,"---:1;");
//	for(i=0;i<TNUMBER;i++) { strcat(str,sTemp[i].get_name()); strcat(str,":0;"); }
	for(i=0;i<ANUMBER;i++) { strcat(str,sADC[i].get_name()); strcat(str,":0;"); }
	for(i=0;i<FNUMBER;i++) {
		strcat(str,sFrequency[i].get_name()); strcat(str,":0;");
#ifdef F_CHART_ChartLiters
		strcat(str,sFrequency[i].get_name()); strcat(str,"_L:0;");
#endif
	}
	strcat(str, chart_BrineWeight); strcat(str,":0;");
	strcat(str, chart_WaterBoost); strcat(str,":0;");
	strcat(str, chart_WaterBoostCountAll); strcat(str,":0;");
	strcat(str, chart_WaterBoostCount); strcat(str,":0;");
	strcat(str, chart_FeedPump); strcat(str,":0;");
	strcat(str, chart_FillTank); strcat(str,":0;");
//	strcat(str,chart_VOLTAGE); strcat(str,":0;");
	strcat(str,chart_fullPOWER); strcat(str,":0;");
#ifdef CHECK_DRAIN_PUMP
	strcat(str, chart_DrainPump); strcat(str,":0;");
#endif
#ifdef CHECK_SEPTIC
	strcat(str, chart_SepticPump); strcat(str,":0;");
#endif
	return str;
}

// обновить статистику, добавить одну точку и если надо записать ее на карту.
// Все значения в графиках целочислены (сотые), выводятся в формате 0.01
void  MainClass::updateChart()
{
	//for(uint8_t i=0;i<TNUMBER;i++) sTemp[i].Chart.addPoint(sTemp[i].get_Temp());
	for(uint8_t i=0;i<ANUMBER;i++) sADC[i].Chart.addPoint(sADC[i].get_Value());
	for(uint8_t i=0;i<FNUMBER;i++) {
		sFrequency[i].ChartFlow.addPoint(sFrequency[i].get_Value()); // Частотные датчики
#ifdef F_CHART_ChartLiters
		sFrequency[i].ChartLiters.addPoint(sFrequency[i].ChartLiters_accum * 100 + sFrequency[i].ChartLiters_rest); // Частотные датчики
		sFrequency[i].ChartLiters_accum = sFrequency[i].ChartLiters_rest = 0;
#endif
	}

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
	//dPWM.ChartVoltage.addPoint(dPWM.get_Voltage() / 10);
	tmp1 = dPWM.get_Power();
	if(GETBIT(Option.flags, fChartOnlyNonZeroW)) {
		if(tmp1 >= MIN_POWER_FOR_CHARTS || dPWM.ChartPower.get_PrevPoint() >= MIN_POWER_FOR_CHARTS) dPWM.ChartPower.addPoint(tmp1);
	} else dPWM.ChartPower.addPoint(tmp1);
#ifdef CHECK_DRAIN_PUMP
	tmp1 = DrainPumpPower;
	if(GETBIT(Option.flags, fChartOnlyNonZeroW)) {
		if(tmp1 >= MIN_POWER_FOR_CHARTS || ChartDrainPump.get_PrevPoint() >= MIN_POWER_FOR_CHARTS) ChartDrainPump.addPoint(tmp1);
	} else ChartDrainPump.addPoint(tmp1);
#endif
#ifdef CHECK_SEPTIC
	tmp1 = SepticPower;
	tmp2 = Option.SepticPumpMinPower * 10;
	if(GETBIT(Option.flags, fChartOnlyNonZeroW)) {
		if(tmp1 >= tmp2 || ChartSepticPump.get_PrevPoint() >= tmp2) ChartSepticPump.addPoint(tmp1);
	} else ChartSepticPump.addPoint(tmp1);
#endif
}

// сбросить графики в ОЗУ
void MainClass::clearChart()
{
	uint8_t i;
	//for(i=0;i<TNUMBER;i++) sTemp[i].Chart.clear();
	for(i = 0; i < ANUMBER; i++) sADC[i].Chart.clear();
	for(i = 0; i < FNUMBER; i++) {
		sFrequency[i].ChartFlow.clear();
#ifdef F_CHART_ChartLiters
	 sFrequency[i].ChartLiters.clear();
	 sFrequency[i].ChartLiters_accum = sFrequency[i].ChartLiters_rest = 0;
	#endif
	}
	ChartWaterBoost.clear();
	//ChartWaterBoosterCount.clear();
	ChartFeedPump.clear();
	ChartFillTank.clear();
	ChartBrineWeight.clear();
	// dPWM.ChartVoltage.clear();                              // Статистика по напряжению
	dPWM.ChartPower.clear();                                // Статистика по Полная мощность
#ifdef CHECK_DRAIN_PUMP
	ChartDrainPump.clear();
#endif
#ifdef CHECK_SEPTIC
	ChartSepticPump.clear();
#endif
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
//	for(i = 0; i < TNUMBER; i++) {
//		if((strcmp(var, sTemp[i].get_name()) == 0)) {
//			sTemp[i].Chart.get_PointsStrDiv100(str);
//			return;
//		}
//	}
	for(i = 0; i < ANUMBER; i++) {
		if((strcmp(var, sADC[i].get_name()) == 0)) {
			sADC[i].Chart.get_PointsStrDiv100(str);
			return;
		}
	}
	for(i = 0; i < FNUMBER; i++) {
		if((strncmp(var, sFrequency[i].get_name(), strlen(sFrequency[i].get_name())) == 0)) {
#ifdef F_CHART_ChartLiters
			if(var[strlen(sFrequency[i].get_name()) - 1] == 'L') sFrequency[i].ChartLiters.get_PointsStrUintDiv1000(str); else
#endif
			sFrequency[i].ChartFlow.get_PointsStrUintDiv1000(str);
			return;
		}
	}
	if(strcmp(var, chart_fullPOWER) == 0) {
		dPWM.ChartPower.get_PointsStrUintDiv1000(str);
//	} else if(strcmp(var, chart_VOLTAGE) == 0) {
//		dPWM.ChartVoltage.get_PointsStr(str);
#ifdef CHECK_DRAIN_PUMP
	} else if(strcmp(var, chart_DrainPump) == 0) {
		ChartDrainPump.get_PointsStr(str);
#endif
#ifdef CHECK_SEPTIC
	} else if(strcmp(var, chart_SepticPump) == 0) {
		ChartSepticPump.get_PointsStr(str);
#endif
	} else if(strcmp(var, chart_WaterBoostCountAll) == 0) {
		ChartWaterBoosterCount.get_PointsStrAbsDiv100(str);
	} else if(strcmp(var, chart_WaterBoostCount) == 0) {
		ChartWaterBoosterCount.get_PointsStrPositiveDiv100(str);
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
//  #ifdef ONEWIRE_DS2482_FOURTH
//	if(bus == 3) i = OneWireBus4.PrepareTemp();
//	else
//  #endif
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

// d - inch, ret = m2 * 10000
uint32_t MainClass::CalcFilterSquare(uint8_t diameter)
{
	uint32_t n = 31416UL / 4UL * diameter * diameter / 100 * 254 / 100 * 254 / 1000;
	return (n + 5) / 10;
}

// square = m2 * 10000, ret = m*h * 1000
uint32_t MainClass::CalcFilteringSpeed(uint32_t square)
{
	return 10000UL * sFrequency[FLOW].get_Value() / square;
}

// Рассчитать сколько осталось дней до регенерации, 0 - Iron, 1 - Soft
uint8_t MainClass::CalcNextRegenAfterDays(int _DaysBeforeRegen, int _DaysFromLastRegen, int _UsedBeforeRegen, int _UsedSinceLastRegen)
{
	if(_DaysBeforeRegen) {
		_DaysBeforeRegen -= _DaysFromLastRegen;
		if(_DaysBeforeRegen == 0) {
			if(SAM_RTC_HOUR(RTC->RTC_TIMR) <= (MC.Option.RegenHour & 0x1F)) _DaysBeforeRegen = 1;	// current_hour <= x
		} else if(_DaysBeforeRegen < 0) return 0;
	} else _DaysBeforeRegen = 255;
	if(_UsedBeforeRegen) {
		_UsedBeforeRegen -= _UsedSinceLastRegen;
		if(_UsedBeforeRegen <= MC.RTC_store.UsedToday) return 0;
		else {
			_UsedBeforeRegen = _UsedBeforeRegen / (MC.WorkStats.UsedAverageDay / MC.WorkStats.UsedAverageDayNum);
			if(_UsedBeforeRegen <= 0) return 1;
		}
	} else _UsedBeforeRegen = 255;
	return _DaysBeforeRegen < _UsedBeforeRegen ? _DaysBeforeRegen : _UsedBeforeRegen;
}
