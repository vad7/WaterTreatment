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
#include "Statistics.h"
#include "MainClass.h"
#include "SdFat.h"

#define temp_initbuf Socket[0].outBuf
char filename[sizeof(stats_file_start)-1 + 4 + sizeof(stats_file_ext)];

// what: 0 - Stats, 1 - History, Return: OK or Error
int8_t Statistics::CreateOpenFile(uint8_t what)
{
	if(!MC.get_fSD()) return ERR_SD_INIT;
	//journal.jprintfopt("CreateOpenFile(%d), %d\n", what, HistoryBlockCreating);
	uint8_t newfile = 0;
	if(what == ID_HISTORY) {
		if(HistoryBlockCreating) goto xContinue;
		HistoryBlockStart = 0;
		HistoryCurrentPos = 0;
		strcpy(filename, history_file_start);
	} else {
		BlockStart = 0;
		CurrentPos = 0;
		strcpy(filename, stats_file_start);
	}
	_itoa(year, filename);
	strcat(filename, stats_file_ext);
	journal.jprintf(" File: %s ", filename);
	SPI_switchSD();
	if(!StatsFile.opens(filename, O_READ, &open_fname)) {
		uint16_t days = month == 1 ? 366 : (12 - month + 1) * 31;
		if(!StatsFile.createContiguous(filename, what ? HISTORY_MAX_FILE_SIZE(days) : STATS_MAX_FILE_SIZE(days))) {
			Error("create", what);
			return ERR_SD_WRITE;
		} else {
			StatsFile.timestamp(T_CREATE | T_ACCESS | T_WRITE, rtcSAM3X8.get_years(), rtcSAM3X8.get_months(), rtcSAM3X8.get_days(), rtcSAM3X8.get_hours(), rtcSAM3X8.get_minutes(), rtcSAM3X8.get_seconds());
		}
		newfile = 1;
	}
	if(!StatsFile.contiguousRange(what ? &HistoryBlockStart : &BlockStart, what ? &HistoryBlockEnd : &BlockEnd)) {
		Error("get blocks", what);
	} else {
		journal.jprintf("[%u..%u] ", what ? HistoryBlockStart : BlockStart, what ? HistoryBlockEnd : BlockEnd);
		if(newfile) {
			journal.jprintf_time("Create ");
			uint32_t b;
			if(what) {
xContinue:		if(HistoryBlockCreating) b = HistoryBlockCreating; else b = HistoryCurrentBlock = HistoryBlockStart;
			} else {
				b = CurrentBlock = BlockStart;
			}
			uint8_t *temp_buf = (uint8_t*)malloc(SD_BLOCK);
			if(temp_buf == NULL) {
				Error("memory low", what);
				StatsFile.remove();
				return ERR_OUT_OF_MEMORY;
			}
			memset(temp_buf, 0, SD_BLOCK);
			for(; b <= (what ? HistoryBlockEnd : BlockEnd);) {
				WDT_Restart(WDT);
				if(!card.card()->writeBlock(b, temp_buf)) {
					Error("empty", what);
					free(temp_buf);
					goto xError;
				}
				if(((++b) & 0x1FF) == 0) {
					if((b & 0x7FF) == 0) journal.jprintf("."); // каждый 1 Мб
					if(what) { // время другим задачам (~200 bps)
						HistoryBlockCreating = b;
						free(temp_buf);
						_delay(CREATE_STATFILE_PAUSE);
						return OK;
					}
				}
			}
			if(what) HistoryBlockCreating = 0;
			journal.jprintf_time("\n Ok\n");
			free(temp_buf);
			return OK;
		} else if(!FindEndPosition(what)) {
			journal.jprintf("Endpos not found!\n");
		} else {
			return OK;
		}
	}
xError:
	StatsFile.close();
	return ERR_SD_WRITE;
}

boolean Statistics::FindEndPosition(uint8_t what)
{
	uint8_t *buffer, *pos = NULL;
	uint32_t bst, bend, cur;
	if(what) {
		bst = HistoryBlockStart;
		bend = HistoryBlockEnd;
		buffer = history_buffer;
	} else {
		bst = BlockStart;
		bend = BlockEnd;
		buffer = stats_buffer;
	}
	while(bst <= bend) {
		WDT_Restart(WDT);
		cur = bst + (bend - bst) / 2;
		//journal.jprintfopt("BS: %d, %d, %d\n", cur, bst, bend);
		if(!card.card()->readBlock(cur, buffer)) {
			Error("FindPos", what);
			break;
		}
		if(*buffer) {
			if((pos = (uint8_t*)memchr(buffer, 0, SD_BLOCK))) break;
			bst = cur + 1;
			if(bst > bend) {
				if(bend < (what ? HistoryBlockEnd : BlockEnd)) bend++; else break; // file overflow
			}
		} else if(cur == bst) { // empty
			//journal.jprintfopt("Empty: %d, %d, %d\n", cur, bst, bend);
			pos = buffer;
			break;
		} else bend = cur - 1;
	}
	if(pos == NULL) return false;
	if(pos == buffer || *(pos-1) != '\n') { // Обрезанные данные - пропускаем
		journal.jprintf("*CUT* ");
		if(pos != buffer) {
xCutSearch:	while(--pos >= buffer) if(*pos == '\n') break;
			pos++;
		}
		if(pos == buffer && cur > (what ? HistoryBlockStart : BlockStart)) {
			if(!card.card()->readBlock(--cur, buffer)) {
				Error("FindPos", what);
				return false;
			}
			pos = buffer + SD_BLOCK;
			goto xCutSearch;
		}
	}
	if(what) {
		HistoryCurrentBlock = cur;
		HistoryCurrentPos = pos - buffer;
	} else {
		CurrentBlock = cur;
		CurrentPos = pos - buffer;
	}
//#ifdef DEBUG_MODWORK
	journal.jprintf("End pos: %u/%u\n", cur, pos - buffer);
//#endif
	return true;
}

void Statistics::Error(const char *text, uint8_t what)
{
	if(card.cardErrorCode() == SD_CARD_ERROR_DMA) {
		journal.jprintf(" %s DMA Error %s: ", what ? "History" : "Stats", text);
		if(card.cardErrorData() & 0x2) journal.jprintf("TIMEOUT ");
		if(card.cardErrorData() & 0x1) journal.jprintf("OVERRUN");
		journal.jprintf("\n");
	} else if(card.cardErrorCode() == SD_CARD_ERROR_READ_CRC) {
		journal.jprintf(" %s CRC Error %s!\n", what ? "History" : "Stats", text);
	} else {
		journal.jprintf(" %s Error %s (%d,%d)!\n", what ? "History" : "Stats", text, card.cardErrorCode(), card.cardErrorData());
	}
}

void Statistics::Init(uint8_t newyear)
{
	if(!newyear) Reset(false);
	HistoryCurrentBlock = 0;
	HistoryBlockCreating = 0;
	year = rtcSAM3X8.get_years();
#ifdef STATS_DO_NOT_SAVE
	return;
#endif
	if(!MC.get_fSD()) {
		journal.jprintf(" No SD card - statistics will not be saved!\n");
		return;
	}
	if(CreateOpenFile(ID_STATS) == OK) {
		if(!newyear) { // read last stats record
			int32_t pos = (CurrentBlock - BlockStart) * SD_BLOCK + CurrentPos - 1;
			uint8_t b;
			for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
				if(Stats_data[i].object == STATS_OBJ_WaterUsed)	Stats_data[i].value = MC.RTC_store.UsedToday;
				//else if(Stats_data[i].object == STATS_OBJ_WaterRegen) Stats_data[i].value = MC.RTC_store.UsedRegen;
			}
			while(--pos >= 0) {
				if(!StatsFile.seekSet(pos)) {
					Error("seek", ID_STATS);
					break;
				}
				if(!StatsFile.read(&b, 1)) {
					Error("readb", ID_STATS);
					break;
				}
				if(b == '\n' || pos == 0) {
					if(pos) pos++;
					if(!StatsFile.read(temp_initbuf, STATS_MAX_RECORD_LEN)) {
						Error("readl", ID_STATS);
						break;
					}
					m_snprintf(temp_initbuf + STATS_MAX_RECORD_LEN, 16, format_date, year, month, day);
					if(memcmp(temp_initbuf, temp_initbuf + STATS_MAX_RECORD_LEN, format_date_size) == 0) { // date the same
						CurrentBlock = BlockStart + pos / SD_BLOCK;
						CurrentPos = pos % SD_BLOCK;
						temp_initbuf[format_date_size] = '\0';
						//journal.jprintfopt(" %s restored at %d\n", temp_initbuf, pos);
						if(!card.card()->readBlock(CurrentBlock, (uint8_t*)stats_buffer)) {
							Error("readp", ID_STATS);
						} else {
							memcpy(temp_initbuf, stats_buffer + CurrentPos, SD_BLOCK - CurrentPos);
							temp_initbuf[STATS_MAX_RECORD_LEN] = '\0';
							char *p = temp_initbuf + format_date_size;
							if((p = strchr(p, '\n'))) *p = '\0';
							p = temp_initbuf + format_date_size;
							while((p = strchr(p, ';'))) *p++ = '\0';
							p = temp_initbuf + format_date_size;
							for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
								float val = my_atof(++p);
								if(val != ATOF_ERROR) {
									switch(Stats_data[i].object) {
									case STATS_OBJ_WaterUsed:
									//case STATS_OBJ_WaterRegen:
										break;
									case STATS_OBJ_RO_WaterUsed:
										Stats_data[i].value = RO_UsedToday = val;
										break;
									case STATS_OBJ_Temp:
									case STATS_OBJ_Press:
										Stats_data[i].value = val * 100.0f + 0.005f;
										break;
									case STATS_OBJ_Voltage:
										Stats_data[i].value = val;
										break;
									case STATS_OBJ_BrineWeight:
										Stats_data[i].value = val * 1000.0f + 0.0005f;
										if(Stats_data[i].type == STATS_TYPE_DELTA) Stats_data[i].value = Weight_value / 10 - Stats_data[i].value;
										break;
									case STATS_OBJ_Flow:
										Stats_data[i].value = val * 1000.0f + 0.0005f;
										break;
									case STATS_OBJ_Power:
										switch(Stats_data[i].type) {
										case STATS_TYPE_SUM:
										case STATS_TYPE_AVG:
											Stats_data[i].value = val * 1000000.0f;
											break;
										default:
											Stats_data[i].value = val * 1000.0f + 0.0005f;
										}
										break;
									default:
										if(Stats_data[i].type == STATS_TYPE_TIME) Stats_data[i].value = val * 60000.0f; else Stats_data[i].value = val;
										break;
									}
									if(*p == '\0') {
										switch(Stats_data[i].type) {
										case STATS_TYPE_MIN:
											Stats_data[i].value = MAX_INT32_VALUE;
											break;
										case STATS_TYPE_MAX:
											Stats_data[i].value = MIN_INT32_VALUE;
											break;
										}
									} else {
										if(Stats_data[i].type == STATS_TYPE_AVG) counts = 1;
									}
									if((p = (char*)memchr(p, '\0', STATS_MAX_RECORD_LEN)) == NULL) break;
								}
							}
							StatsFileString(temp_initbuf);
							journal.jprintf(" Loaded: %s", temp_initbuf);
						}
					}
					break;
				}
			}
		}
		StatsFile.close();
	}
}

void Statistics::CheckCreateNewFile()
{
	uint8_t sem = 0;
	if(!MC.get_fSD()) return;
	if(NewYearFlag) {
		if(!(sem = SemaphoreTake(xWebThreadSemaphore, 0))) return;
		SaveStats(1);
		SaveHistory(1);
		// Truncate stats
		strcpy(filename, stats_file_start);
		_itoa(year, filename);
		strcat(filename, stats_file_ext);
		SPI_switchSD();
		if(!StatsFile.opens(filename, O_RDWR, &open_fname)) Error("open", ID_STATS);
		else {
			journal.jprintf("Truncate %s to %d\n", filename, (CurrentBlock - BlockStart) * SD_BLOCK + CurrentPos);
			if(!StatsFile.truncate((CurrentBlock - BlockStart) * SD_BLOCK + CurrentPos)) Error("truncate", ID_STATS);
			StatsFile.close();
		}
		// Truncate history
		strcpy(filename, history_file_start);
		_itoa(year, filename);
		strcat(filename, stats_file_ext);
		if(!StatsFile.opens(filename, O_RDWR, &open_fname)) Error("open", ID_HISTORY);
		else {
			journal.jprintf("Truncate %s to %d\n", filename, (HistoryCurrentBlock - HistoryBlockStart) * SD_BLOCK + HistoryCurrentPos);
			if(!StatsFile.truncate((HistoryCurrentBlock - HistoryBlockStart) * SD_BLOCK + HistoryCurrentPos)) Error("truncate", ID_HISTORY);
			StatsFile.close();
		}
		Init(1);
		NewYearFlag = 0;
	}
	if(GETBIT(MC.Option.flags, fHistory) && (HistoryCurrentBlock == 0 || HistoryBlockCreating != 0)) { // Init History
		if(!sem && !(sem = SemaphoreTake(xWebThreadSemaphore, 0))) return;
		if(CreateOpenFile(ID_HISTORY) == OK) {
			StatsFile.close();
		} else SETBIT0(MC.Option.flags, fHistory); // При ошибке выключаем опцию сохранения истории!
	}
	if(sem) SemaphoreGive(xWebThreadSemaphore);
}

// Сбросить накопленные промежуточные значения
void Statistics::Reset(bool newday)
{
#ifndef TEST_BOARD
	if(MC.get_testMode() > STAT_TEST) return;
#endif
	for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
		switch(Stats_data[i].type){
		case STATS_TYPE_MIN:
			Stats_data[i].value = MAX_INT32_VALUE;
			break;
		case STATS_TYPE_MAX:
			Stats_data[i].value = MIN_INT32_VALUE;
			break;
		case STATS_TYPE_DELTA:
			if(newday) {
				if(Stats_data[i].object == STATS_OBJ_BrineWeight) Stats_data[i].value = Weight_value / 10;
			}
			break;
		default:
			Stats_data[i].value = 0;
		}
	}
	counts = 0;
	day = rtcSAM3X8.get_days();
	month = rtcSAM3X8.get_months();
	previous = GetTickCount();
}

// Обновить статистику, вызывается часто, раз в TIME_READ_SENSOR
// Возврат: True - новый день
void Statistics::Update()
{
#ifndef TEST_BOARD
	if(MC.get_testMode() > STAT_TEST) return;
#endif
	if(NewYearFlag) return; // waiting to switch a next year
	uint32_t tm = GetTickCount() - previous;
	previous = GetTickCount();
	if(rtcSAM3X8.get_days() != day) {
		if(SaveStats(2) == OK) {
			Reset(true);
			if(year != rtcSAM3X8.get_years()) NewYearFlag = 1; // waiting to switch a next year
			journal.jprintfopt("=== %s\n", NowDateToStr()); // Новый день.
		}
	}
	int32_t newval = 0;
	for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
		switch(Stats_data[i].object) {
		case STATS_OBJ_WaterUsed:
			newval = Stats_WaterUsed_work;
			Stats_WaterUsed_work = 0;
			break;
		case STATS_OBJ_RO_WaterUsed:
			newval = Stats_RO_WaterUsed_work;
			Stats_RO_WaterUsed_work = 0;
			break;
		case STATS_OBJ_WaterRegen:
			if(!MC.sInput[REG_ACTIVE].get_Input() && !MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) continue;
			newval = Stats_WaterRegen_work;
			Stats_WaterRegen_work = 0;
			break;
		case STATS_OBJ_WaterRegenSoftening:
			if(!MC.sInput[REG2_ACTIVE].get_Input()) continue;
			newval = Stats_WaterRegen_work;
			Stats_WaterRegen_work = 0;
			break;
		case STATS_OBJ_Temp:
			newval = MC.sTemp[STATS_ID_Temp].get_Temp();
			break;
		case STATS_OBJ_Press:
			newval = MC.sADC[STATS_ID_Press].get_Value();
			break;
		case STATS_OBJ_Flow:
			newval = MC.sFrequency[STATS_ID_Flow].get_Value();
			break;
		case STATS_OBJ_Voltage:
			newval = MC.dPWM.get_Voltage();
			break;
		case STATS_OBJ_Power: {
				newval = MC.dPWM.get_Power(); // Вт
				switch(Stats_data[i].type) {
				case STATS_TYPE_SUM:
				//case STATS_TYPE_AVG:
					newval = newval * tm / 3600; // в мВт
					Stats_Power_work += newval;
				}
				break;
			}
		case STATS_OBJ_WaterBooster:
			newval = Stats_WaterBooster_work;
			Stats_WaterBooster_work = 0;
			break;
		case STATS_OBJ_FeedPump:
			newval = Stats_FeedPump_work;
			Stats_FeedPump_work = 0;
			break;
		case STATS_OBJ_BrineWeight:
			newval = Weight_value / 10;
			if(newval <= 0) {
				newval = 0;
				if(Stats_data[i].type == STATS_TYPE_MIN && Stats_data[i].value != MAX_INT32_VALUE) continue; // не фиксировать провалы до нуля (например, при снятии бака для заполнения)
			}
			break;
		}
		switch(Stats_data[i].type){
		case STATS_TYPE_MIN:
			if(newval < Stats_data[i].value) Stats_data[i].value = newval;
			break;
		case STATS_TYPE_MAX:
			if(newval > Stats_data[i].value) Stats_data[i].value = newval;
			break;
		case STATS_TYPE_AVG:
		case STATS_TYPE_SUM:
			Stats_data[i].value += newval;
			break;
		case STATS_TYPE_TIME:
			Stats_data[i].value += tm;
			break;
		case STATS_TYPE_DELTA:
			break;
		}
	}
	counts++;
//	for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) journal.jprintf("%d=%d, ", i, Stats_data[i].value); journal.jprintf("\n");
}

// Логирование параметров работы , раз в 1 минуту
void Statistics::History()
{
	if(!GETBIT(MC.Option.flags, fHistory)
#ifndef TEST_BOARD
			|| MC.get_testMode() > STAT_TEST
#endif
		) return;
	uint16_t y = rtcSAM3X8.get_years();
	if(y != year) return;
	char *mbuf = (char*) malloc(HISTORY_MAX_RECORD_LEN);
	if(mbuf == NULL) {
		Error("memory low", ID_HISTORY);
		return;
	}
	char *buf = mbuf;
	buf += m_snprintf(buf, 20, format_datetime, y, rtcSAM3X8.get_months(), rtcSAM3X8.get_days(), rtcSAM3X8.get_hours(), rtcSAM3X8.get_minutes());
	//journal.jprintfopt("History:(%s)\n", mbuf);
	for(uint8_t i = 0; i < sizeof(HistorySetup) / sizeof(HistorySetup[0]); i++) {
		*buf++ = ';';
		switch(HistorySetup[i].object) { // web will divide by 1/10/1000, except 'L'
		case STATS_OBJ_Temp:		// C
			int_to_dec_str(MC.sTemp[HistorySetup[i].number].get_Temp(), 10, &buf, 0); // T (/10), дробная часть выделяется в html
			break;
		case STATS_OBJ_Press:		// bar
			int_to_dec_str(MC.sADC[HistorySetup[i].number].get_Value(), 1, &buf, 0); // P (/1)
			break;
		case STATS_OBJ_Flow:		// m3h
			int_to_dec_str(MC.sFrequency[HistorySetup[i].number].get_Value(), 1, &buf, 0); // F (/1)
			break;
		case STATS_OBJ_Power:
#ifdef CHECK_DRAIN_PUMP
			int_to_dec_str(HistorySetup[i].number == 0 ? MC.dPWM.get_Power() : DrainPumpPower, 1, &buf, 0);  // W (/1000)
#else
			int_to_dec_str(MC.dPWM.get_Power(), 1, &buf, 0);  // W (/1000)
#endif
			break;
		case STATS_OBJ_WaterUsed: {
				int32_t tmp = History_WaterUsed_work;
				History_WaterUsed_work = 0;
				int_to_dec_str(tmp, 1, &buf, 0);  // L
				break;
			}
		case STATS_OBJ_WaterRegen: {
				int32_t tmp = History_WaterRegen_work;
				History_WaterRegen_work = 0;
				int_to_dec_str(tmp, 1, &buf, 0);  // L
				break;
			}
		case STATS_OBJ_WaterBoosterLiters: {
				int32_t tmp = History_BoosterCountL;
				if(tmp >= 0) {
					History_BoosterCountL = -1;
					int_to_dec_str(tmp, 100, &buf, 2);  // L
				} //else *buf++ = '-';
				break;
			}
		case STATS_OBJ_WaterBooster: {
				int32_t tmp = History_WaterBooster_work;
				History_WaterBooster_work = 0;
				int_to_dec_str(tmp, 1000, &buf, 0);  // sec, S
				break;
			}
		case STATS_OBJ_FeedPump: {
				int32_t tmp = History_FeedPump_work;
				History_FeedPump_work = 0;
				int_to_dec_str(tmp, 1000, &buf, 0);  // sec, S
				break;
			}
		case STATS_OBJ_BrineWeight: {
				int_to_dec_str(Weight_value < 0 ? 0 : Weight_value, 10, &buf, 0); // kg, M (/1000)
				break;
			}
		case STATS_OBJ_Level: { // дробная часть выделяется в html
				int_to_dec_str(MC.sADC[HistorySetup[i].number].get_Value(), 1, &buf, 0); // % (/100), R
				break;
			}
		}
		if(buf > mbuf + HISTORY_MAX_RECORD_LEN - HISTORY_MAX_FIELD_LEN) {
			journal.jprintf("%s memory overflow(%d): %d, max: %d\n", "History", i, buf - mbuf, HISTORY_MAX_RECORD_LEN);
			break;
		}
	}
	*buf++ = '\n'; *buf = '\0';
	uint16_t lensav, len = buf - mbuf + 1;
	memcpy(history_buffer + HistoryCurrentPos, mbuf, lensav = SD_BLOCK - HistoryCurrentPos < len ? SD_BLOCK - HistoryCurrentPos : len);
	if(lensav != len) { // save when there is no space in buffer
		if(SaveHistory(0) == OK) {
			if(HistoryCurrentBlock >= HistoryBlockEnd) {
				Error("File Overflow", ID_HISTORY);
			} else HistoryCurrentBlock++;
			memset(history_buffer, 0, SD_BLOCK);
			memcpy(history_buffer, mbuf + lensav, HistoryCurrentPos = len - lensav - 1);
		}
	} else HistoryCurrentPos += lensav - 1;
	free(mbuf);
}

// Возвращает файл с заголовками полей, flag: +Axis char
void Statistics::HistoryFileHeader(char *ret, uint8_t flag)
{
	strcat(ret, "Время;");
	for(uint8_t i = 0; i < sizeof(HistorySetup) / sizeof(HistorySetup[0]); i++) {
		if(i > 0) strcat(ret, ";");
		if(flag) {
			switch(HistorySetup[i].object) {
			case STATS_OBJ_Temp:
				strcat(ret, "T"); 	// ось температур
				break;
			case STATS_OBJ_Press:
				strcat(ret, "P"); 	// ось давлений
				break;
			case STATS_OBJ_Voltage:
				strcat(ret, "V");	// ось напряжение
				break;
			case STATS_OBJ_Power:
				strcat(ret, "W");	// ось мощность
				break;
			case STATS_OBJ_Flow:
				strcat(ret, "F");	// ось м3ч
				break;
			case STATS_OBJ_WaterUsed:
			case STATS_OBJ_RO_WaterUsed:
			case STATS_OBJ_WaterRegen:
			case STATS_OBJ_WaterBoosterLiters:
				strcat(ret, "L");	// ось л
				break;
			case STATS_OBJ_WaterBooster:
			case STATS_OBJ_FeedPump:
				strcat(ret, "S");	// ось секунды
				break;
			case STATS_OBJ_BrineWeight:
				strcat(ret, "M");	// ось кг
				break;
			case STATS_OBJ_Level:
				strcat(ret, "R");	// ось %
				break;
			default: strcat(ret, "?");
			}
		}
		strcat(ret, HistorySetup[i].name);
	}
	strcat(ret, "\n");
}

// Возвращает заголовок поля, flag: +Axis char
void Statistics::StatsFieldHeader(char *ret, uint8_t i, uint8_t flag)
{
	if(flag && Stats_data[i].type == STATS_TYPE_TIME) strcat(ret, "M"); // ось часы
	switch(Stats_data[i].object) {
	case STATS_OBJ_Temp:
		if(flag) strcat(ret, "T"); // ось температур
		strcat(ret, MC.sTemp[STATS_ID_Temp].get_note());
		break;
	case STATS_OBJ_Press:
		if(flag) strcat(ret, "P"); // ось давление
		strcat(ret, MC.sADC[STATS_ID_Press].get_note());
		break;
	case STATS_OBJ_Flow:
		if(flag) strcat(ret, "F"); // ось проток
		strcat(ret, MC.sFrequency[STATS_ID_Flow].get_note());
		break;
	case STATS_OBJ_WaterBooster:
		if(flag) strcat(ret, "S"); // ось время
		strcat(ret, "Насосная станция, сек");
		break;
	case STATS_OBJ_FeedPump:
		if(flag) strcat(ret, "S"); // ось время
		strcat(ret, "Дозирующий насос, сек");
		break;
	case STATS_OBJ_Voltage:
		if(flag) strcat(ret, "V"); // ось напряжение
		strcat(ret, "Напряжение, V");
		break;
	case STATS_OBJ_Power:
		if(flag) strcat(ret, "W"); // ось мощность
		strcat(ret, "Потребление, кВт"); // хранится в Вт
		if(Stats_data[i].type == STATS_TYPE_SUM) strcat(ret, "ч");
		break;
	case STATS_OBJ_WaterUsed:
		if(flag) strcat(ret, "L");	// ось литры
		strcat(ret, "Потреблено, л");
		return;
	case STATS_OBJ_RO_WaterUsed:
		if(flag) strcat(ret, "L");	// ось литры
		strcat(ret, "Питьевой фильтр, л");
		return;
	case STATS_OBJ_WaterRegen:
		if(flag) strcat(ret, "L");	// ось литры
		strcat(ret, "Регенерация обезжелезивателя, л");
		break;
	case STATS_OBJ_WaterRegenSoftening:
		if(flag) strcat(ret, "L");	// ось литры
		strcat(ret, "Регенерация умягчителя, л");
		break;
	case STATS_OBJ_BrineWeight:
		if(flag) strcat(ret, "M");	// ось вес
		strcat(ret, "Раствор, кг");
		break;
	default: strcat(ret, "?");
	}
	switch(Stats_data[i].type) {
	case STATS_TYPE_MIN:
		strcat(ret, " (Мин)");
		break;
	case STATS_TYPE_MAX:
		strcat(ret, " (Макс)");
		break;
	case STATS_TYPE_AVG:
		strcat(ret, " (Сред)");
		break;
	case STATS_TYPE_DELTA:
		strcat(ret, " (Δ)");
		break;
	}
}

// Возвращает файл с заголовками полей, flag: +Axis char
void Statistics::StatsFileHeader(char *ret, uint8_t flag)
{
	strcat(ret, "Дата;");
	for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
		if(i > 0) strcat(ret, ";");
		StatsFieldHeader(ret, i, flag);
	}
	strcat(ret, "\n");
}

void Statistics::StatsFieldString(char **ret, uint8_t i)
{
	int32_t val;
	if(Stats_data[i].type == STATS_TYPE_AVG) {
		val = counts;
		if(val == 0) {
xSkipEmpty:
			**ret = '\0';
			return;
		}
		val = Stats_data[i].value / val;
	} else val = Stats_data[i].value;
	if((val == MIN_INT32_VALUE && Stats_data[i].type == STATS_TYPE_MAX) || (val == MAX_INT32_VALUE && Stats_data[i].type == STATS_TYPE_MIN)) goto xSkipEmpty;
	switch(Stats_data[i].object) {
	case STATS_OBJ_Temp:					// C
		int_to_dec_str(val / 10, 10, ret, 1);
		break;
	case STATS_OBJ_Press: 					// bar
		int_to_dec_str(val, 100, ret, 1);
		break;
	case STATS_OBJ_Voltage:					// V
		int_to_dec_str(val, 10, ret, 0);
		break;
	case STATS_OBJ_WaterUsed:				// L
	case STATS_OBJ_RO_WaterUsed:			// L
		int_to_dec_str(val, 1, ret, 0);
		break;
	case STATS_OBJ_WaterRegen:				// L
	case STATS_OBJ_WaterRegenSoftening:		// L
		if(val == 0) goto xSkipEmpty;
		int_to_dec_str(val, 1, ret, 0);
		break;
	case STATS_OBJ_BrineWeight:				// kg
		if(Stats_data[i].type == STATS_TYPE_DELTA) {
			val = Weight_value / 10 - val;
		}
	case STATS_OBJ_Flow:					// m3h
		int_to_dec_str(val, 1000, ret, 3);
		break;
	case STATS_OBJ_WaterBooster:			// s
	case STATS_OBJ_FeedPump:				// s
		int_to_dec_str(val, 1000, ret, 0);
		break;
	case STATS_OBJ_Power:					// кВт*ч
		switch(Stats_data[i].type) {
		case STATS_TYPE_SUM:
		case STATS_TYPE_AVG:
			int_to_dec_str(val / 1000, 1000, ret, 3);
			break;
		default:
			int_to_dec_str(val, 1000, ret, 3);
		}
		break;
	default:
		if(Stats_data[i].type == STATS_TYPE_TIME) int_to_dec_str(val / 10000, 6, ret, 1);  // минуты;
		else goto xSkipEmpty;
		break;
	}
}

// Строка со значениями за день (разделитель ";"), при запуске не из Update() возможны неверные данные!
inline void Statistics::StatsFileString(char *ret)
{
	ret += m_snprintf(ret, 20, format_date, year, month, day);
	for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
		*ret++ = ';';
		StatsFieldString(&ret, i);
	}
	*ret = '\n'; *(ret+1) = '\0';
}

void Statistics::StatsWebTable(char *ret)
{
	for(uint8_t i = 0; i < sizeof(Stats_data) / sizeof(Stats_data[0]); i++) {
		StatsFieldHeader(ret, i, 0);
		ret += m_strlen(ret);
		*ret++ = '|';
		StatsFieldString(&ret, i);
		strcat(ret, ";");
	}
}

#define _buffer_ ((uint8_t*)Socket[thread].outBuf)

// Return: OK, 1 - not found, >2 - error. Network is active
void Statistics::SendFileData(uint8_t thread, SdFile *File, char *filename)
{
	SPI_switchSD();
	if(!File->opens(filename, O_READ, &open_fname)) {
		journal.jprintf("Error open %s\n", filename);
		sendConstRTOS(thread, HEADER_FILE_NOT_FOUND);
		return;
	}
	uint32_t bst, bend;
	if(!File->contiguousRange(&bst, &bend)) {
		journal.jprintf("Error get blocks %s\n", filename);
		File->close();
		return;
	}
	if(MC.get_NetworkFlags() & (1<<fWebFullLog)) {
		journal.jprintf("Read %s: %u\n", filename, File->fileSize());
	}
	File->close();
	uint32_t readed = strlen((char*)_buffer_);
	if(sendPacketRTOS(thread, _buffer_, readed, 0) != readed) {
		journal.jprintf("Error sendh %s\n", filename);
		return;
	}
	readed = 0;
	for(uint32_t i = bst; i <= bend; i++) {
		SPI_switchSD();
		if(i == CurrentBlock) {
			memcpy(_buffer_ + readed, stats_buffer, SD_BLOCK);
		} else if(i == HistoryCurrentBlock) {
			memcpy(_buffer_ + readed, history_buffer, SD_BLOCK);
		} else if(!card.card()->readBlock(i, _buffer_ + readed)) {
			Error("read data", ID_STATS);
			break;
		}
		if(_buffer_[readed + SD_BLOCK - 1] == 0) {  // end of data
			if(_buffer_[readed] == 0) break;
			readed = (uint8_t*)memchr(_buffer_ + readed, 0, SD_BLOCK) - _buffer_;
			bend = 0;
		} else {
			readed += SD_BLOCK;
			if(readed <= W5200_MAX_LEN - SD_BLOCK) continue;
		}
		if(sendPacketRTOS(thread, _buffer_, readed, 0) != readed) {
			journal.jprintf("Error send %s\n", filename);
			break;
		}
		readed = 0;
	}
}

// Return: OK, 1 - not found, >2 - error. Network is active. Date format: "yyyymmdd...\0"
void Statistics::SendFileDataByPeriod(uint8_t thread, SdFile *File, char *Prefix, char *TimeStart, char *TimeEnd)
{
	uint32_t bendfile = m_strlen((char*)_buffer_);
	strcpy((char*)_buffer_ + bendfile + 1, Prefix);
	strncat((char*)_buffer_+ bendfile + 1, TimeStart, 4); // year
	strcat((char*)_buffer_ + bendfile + 1, stats_file_ext);
	SPI_switchSD();
	if(!File->opens((char*)_buffer_ + bendfile + 1, O_READ, &open_fname)) {
		journal.jprintf("Error open %s\n", _buffer_ + bendfile + 1);
		sendConstRTOS(thread, HEADER_FILE_NOT_FOUND);
		return;
	}
	if(sendPacketRTOS(thread, _buffer_, bendfile, 0) != bendfile) {
		journal.jprintf("Error sendh %s\n", Prefix);
		return;
	}
	uint32_t bstfile;
	if(!File->contiguousRange(&bstfile, &bendfile)) {
		journal.jprintf("Error get blocks %s\n", filename);
		File->close();
		return;
	}
	File->close();
	uint32_t bst = bstfile, bend = bendfile;
	uint8_t findst = 0;
	char *pos = NULL;
	uint8_t len_Time = strlen(TimeStart);
	while(bst <= bend) {
		uint32_t cur = bst + (bend - bst) / 2;
xReadBlock:
		//journal.printf("BS: %d, %d, %d\n", cur, bst, bend);
		if(cur == CurrentBlock) {
			memcpy(_buffer_, stats_buffer, SD_BLOCK);
		} else if(cur == HistoryCurrentBlock) {
			memcpy(_buffer_, history_buffer, SD_BLOCK);
		} else if(!card.card()->readBlock(cur, _buffer_)) {
			Error("read f", ID_HISTORY);
			return;
		}
		if(*_buffer_) {
			pos = (char*)memchr(_buffer_, '\n', SD_BLOCK);
			if(pos == NULL) return;  // garbage
			if(*++pos == '\0') goto xGoDown;
			{
				int8_t cmp = strncmp(pos, TimeStart, len_Time);
				if(cmp >= 0) {
					//journal.printf("found%d %c%c%c%c%c%c%c%c%c%c (%s)\n", cmp, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7], pos[8], pos[9], TimeStart );
					findst = 1;
					if(cmp > 0) {
						if(cur == bst) {
							if(strncmp(pos, TimeEnd, len_Time) > 0) return; // greater - not found
							goto xNext;
						}
						goto xGoDown;
					}
					if(cur > bst) { // slow down at equal
						cur--;
						goto xReadBlock;
					}
				}
			}
			bst = cur;
xNext:		if(findst) { // found
				//journal.printf("Found at %d - %d\n", cur, (uint8_t*)pos - _buffer_);
				memmove(_buffer_, pos, bend = SD_BLOCK - ((uint8_t*)pos - _buffer_));
				break;
			}
			bst++;
			if(bst > bend) {
				if(bend < bendfile) bend++;
				else { // file overflow
					return;
				}
			}
		} else {
			//journal.printf("Zero\n");
xGoDown:	if(cur == bst) { // low limit
				//journal.printf("Low\n");
				if(bst == bstfile) {
					pos = (char*)_buffer_;
					if(*pos == '\0') return; // empty
				} else {
					pos = (char*)memchr(_buffer_, '\n', SD_BLOCK-1);
					if(pos == NULL) return;
					pos++;
				}
				goto xNext;
			} else {
				bend = cur - 1;
				findst = 0;
			}
		}
	}
	//journal.printf("ST: %d (%d), END: %d\n", bst, bend, bendfile);
	uint32_t readed = 0;
	uint16_t packcnt = 0;
	if(pos) {
		pos = (char*)memchr(_buffer_, 0, bend);
		readed = pos ? pos - (char*)_buffer_ : bend;
		pos = (char*)_buffer_;
		goto xFoundStart;
	}
	for(; bst <= bendfile; bst++) {
		SPI_switchSD();
		if(bst == CurrentBlock) {
			memcpy(_buffer_ + readed, stats_buffer, SD_BLOCK);
		} else if(bst == HistoryCurrentBlock) {
			memcpy(_buffer_ + readed, history_buffer, SD_BLOCK);
		} else if(!card.card()->readBlock(bst, _buffer_ + readed)) {
			Error("read data", ID_HISTORY);
			break;
		}
		if(_buffer_[readed + SD_BLOCK - 1] == '\0') {  // end of data
			if(_buffer_[readed] == '\0') break;
			pos = (char*)memchr(_buffer_ + readed, '\0', SD_BLOCK);
			readed = (uint8_t*)pos - _buffer_;
			bendfile = 0;
		} else {
			pos = (char*)memchr(_buffer_ + readed, '\n', SD_BLOCK);
			readed += SD_BLOCK;
		}
		if(pos++) {
xFoundStart:
			if(strncmp(pos, TimeEnd, len_Time) > 0) {
				//journal.printf("end %c%c%c%c%c%c%c%c%c%c (%s)\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7], pos[8], pos[9], TimeEnd );
				readed = (uint8_t*)pos - _buffer_;
				bendfile = 0; // stop
			} else if(readed <= W5200_MAX_LEN - SD_BLOCK) continue;
		} else bendfile = 0;
		if(sendPacketRTOS(thread, _buffer_, readed, 0) != readed) {
			journal.jprintf("Error send %s\n", filename);
			return;
		}
		if(++packcnt == 50) { // 100kb send
			packcnt = 0;
			SemaphoreGive(xWebThreadSemaphore);
			_delay(WEB_SEND_FILE_PAUSE);
			if(SemaphoreTake(xWebThreadSemaphore, W5200_TIME_WAIT) !=pdPASS) {
				journal.jprintf("Cant take SEM while sending file!\n");
				return;
			}
		}
		readed = 0;
	}
	if(readed) {
		if(sendPacketRTOS(thread, _buffer_, readed, 0) != readed) {
			journal.jprintf("Error send %s\n", filename);
		}
	}
}

// Записать статистику на SD, 0 - только записать, 1 - только записать c веба, 2 - новый день
// Return: OK или Ошибка
int8_t Statistics::SaveStats(uint8_t newday)
{
#ifdef STATS_DO_NOT_SAVE
	return OK;
#endif
#ifndef TEST_BOARD
	if(MC.get_testMode() > STAT_TEST) return OK;
#endif
	if(!MC.get_fSD() || CurrentBlock == 0) return OK;
	char *rbuf = (char*) malloc(STATS_MAX_RECORD_LEN);
	if(rbuf == NULL) {
		Error("memory low", ID_STATS);
		return ERR_OUT_OF_MEMORY;
	}
	int8_t retval = OK;
	StatsFileString(rbuf);
	//journal.jprintfopt("SaveStats(%d):%s\n", newday, rbuf);
	uint16_t lensav, len = m_strlen(rbuf) + 1;
	memcpy(stats_buffer + CurrentPos, rbuf, lensav = SD_BLOCK - CurrentPos < len ? SD_BLOCK - CurrentPos : len);
#ifdef STATS_USE_BUFFER_FOR_SAVING
	if(newday < 2 || lensav != len) { // save when there is no space in buffer
#endif
		if(newday != 1 && SemaphoreTake(xWebThreadSemaphore, newday == 0 ? W5200_TIME_WAIT : 0) == pdFALSE) {
			retval = ERR_CONFIG;
			free(rbuf);
			return retval;
		}
		SPI_switchSD();
		if(!card.card()->writeBlock(CurrentBlock, (uint8_t*)stats_buffer)) {
			Error("save", ID_STATS);
			// to do - reinit card but in other task
			//if(card.cardErrorCode() > SD_CARD_ERROR_NONE && card.cardErrorCode() < SD_CARD_ERROR_READ && card.cardErrorData() == 255) { // reinit card
			//	if(card.begin(PIN_SPI_CS_SD, SD_SCK_MHZ(SD_CLOCK))) goto xContinue;
			//	else journal.jprintf("Reinit SD card failed!\n");
			//}
			retval = ERR_SD_WRITE;
		} else if(lensav != len){ // next block
			if(CurrentBlock >= BlockEnd) {
				journal.jprintf("Stats file size exceeded!\n"); // to do: increase file
				retval = ERR_SD_WRITE;
			} else {
				memset(stats_buffer, 0, SD_BLOCK);
				memcpy(stats_buffer, rbuf + lensav, len - lensav);
				if(!card.card()->writeBlock(CurrentBlock + 1, (uint8_t*)stats_buffer)) {
					Error("save 2", ID_STATS);
					retval = ERR_SD_WRITE;
				} else if(newday == 2) { // new day
					if(CurrentBlock >= BlockEnd) {
						Error("File Overflow", ID_STATS);
						retval = ERR_CONFIG;
					} else CurrentBlock++;
					CurrentPos = len - lensav - 1;
				} else { // reread current block
					if(!card.card()->readBlock(CurrentBlock, (uint8_t*)stats_buffer)) {
						Error("read", ID_STATS);
						retval = ERR_SD_READ;
					}
				}
			}
		} else if(newday == 2) CurrentPos += lensav - 1; // new day
	    if(newday != 1) SemaphoreGive(xWebThreadSemaphore);
#ifdef STATS_USE_BUFFER_FOR_SAVING
	} else CurrentPos += lensav - 1; // new day
#endif
	free(rbuf);
	return retval;
}

// Return: OK или Ошибка
int8_t Statistics::SaveHistory(uint8_t from_web)
{
#ifdef STATS_DO_NOT_SAVE
	return OK;
#endif
#ifndef TEST_BOARD
	if(MC.get_testMode() > STAT_TEST) return OK;
#endif
	if(!GETBIT(MC.Option.flags, fHistory) || !MC.get_fSD() || HistoryCurrentBlock == 0) return OK;
	//journal.jprintfopt("SaveHistory(%d)\n", from_web);
	if(!from_web && SemaphoreTake(xWebThreadSemaphore, W5200_TIME_WAIT) == pdFALSE) return ERR_CONFIG;
	int8_t retval = OK;
	SPI_switchSD();
	if(!card.card()->writeBlock(HistoryCurrentBlock, (uint8_t*)history_buffer)) {
		Error("save", ID_STATS);
		retval = ERR_SD_WRITE;
	}
	if(!from_web) SemaphoreGive(xWebThreadSemaphore);
	return retval;
}

