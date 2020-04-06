/*
 * Copyright (c) 2020 by Vadim Kulakov vad7@yahoo.com, vad711
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
//
#ifndef Statistics_h
#define Statistics_h
#include "Constant.h"
#include "SdFat.h"

//#define STATS_DO_NOT_SAVE
const char format_date[] = "%04d%02d%02d";  			// yyyymmdd
#define format_date_size 8
const char format_datetime[] = "%04d%02d%02d%02d%02d";	// yyyymmddHHMM
#define format_datetime_size 12

#define SD_BLOCK					512
#define STATS_MAX_RECORD_LEN		(format_date_size + 1 + sizeof(Stats_data) / sizeof(Stats_data[0]) * 8)
#define STATS_MAX_FILE_SIZE(days)	((STATS_MAX_RECORD_LEN * days / SD_BLOCK + 1) * SD_BLOCK)

#define HISTORY_MAX_FIELD_LEN		6
#define HISTORY_MAX_RECORD_LEN		(format_datetime_size + 1 + sizeof(HistorySetup) / sizeof(HistorySetup[0]) * HISTORY_MAX_FIELD_LEN)
#define HISTORY_MAX_FILE_SIZE(days)	((HISTORY_MAX_RECORD_LEN * 1440 * days / SD_BLOCK + 1) * SD_BLOCK)

#define MAX_INT32_VALUE 			2147483647
#define MIN_INT32_VALUE 			-2147483647

#define WEB_SEND_FILE_PAUSE			10	// ms
#define CREATE_STATFILE_PAUSE		10	// ms

// what:
#define ID_STATS 	0
#define ID_HISTORY	1

//static char *stats_format = { "%.1f", "" }; // printf format string

volatile int32_t Stats_Power_work = 0;  // рабочий для счетчиков - энергия потребленная, мВт
volatile int32_t Stats_WaterUsed_work = 0; 		// L
volatile int32_t Stats_WaterRegen_work = 0; 	// L
volatile int32_t Stats_FeedPump_work = 0;		// ms
volatile int32_t Stats_WaterBooster_work = 0;	// ms
volatile int32_t History_WaterUsed_work = 0;	// L
volatile int32_t History_WaterRegen_work = 0;	// L
volatile int32_t History_FeedPump_work = 0;		// ms
volatile int32_t History_WaterBooster_work = 0;	// ms
int32_t History_BoosterCountL = -1;				// L*100
volatile int32_t Charts_WaterBooster_work = 0;
volatile int32_t Charts_FeedPump_work = 0;
volatile int32_t Charts_FillTank_work = 0; 		// %

const char stats_file_start[] = "stats_";
const char stats_file_header[] = "head";
const char stats_file_ext[] = ".dat";
const char stats_csv_file_ext[] = ".csv";
const char history_file_start[] = "hist_";
uint8_t stats_buffer[SD_BLOCK];
uint8_t history_buffer[SD_BLOCK];
static fname_t open_fname;

class Statistics
{
public:
	Statistics() { NewYearFlag = 0; }
	void	Init(uint8_t newyear = 0);
	void	Update();										// Обновить статистику, раз в период
	void	UpdateEnergy();									// Обновить энергию и COP, вызывается часто
	void	Reset();										// Сбросить накопленные промежуточные значения
	int8_t	SaveStats(uint8_t newday);						// Записать статистику на SD
	int8_t 	SaveHistory(uint8_t from_web);					// Записать буфер истории на SD
	void	StatsFileHeader(char *buffer, uint8_t flag);	// Возвращает файл с заголовками полей
	void	StatsFieldHeader(char *ret, uint8_t i, uint8_t flag);
	inline void	StatsFileString(char *ret);					// Строка со значениями за день
	void	StatsFieldString(char **ret, uint8_t i);
	void	StatsWebTable(char *ret);
	void 	HistoryFileHeader(char *ret, uint8_t flag);		// Возвращает файл с заголовками полей
	void	SendFileData(uint8_t thread, SdFile *File, char *filename); // Отправить в сокет данные файла
	void	SendFileDataByPeriod(uint8_t thread, SdFile *File, char *Prefix, char *TimeStart, char *TimeEnd); // Отправить в сокет данные файла, обрезанные по датам
	boolean	FindEndPosition(uint8_t what);
	void	CheckCreateNewFile();
	int8_t	CreateOpenFile(uint8_t what);
	void	History();										// Логирование параметров работы , раз в 1 минуту
private:
	void	Error(const char *text, uint8_t what);
	uint16_t counts;										// Кол-во уже совершенных обновлений
	uint32_t previous;
	uint8_t	 day;
	uint8_t	 month;
	uint16_t year;
	uint8_t  NewYearFlag;
	uint32_t BlockStart;
	uint32_t BlockEnd;
	uint32_t CurrentBlock;
	uint16_t CurrentPos;
	uint32_t HistoryBlockStart;
	uint32_t HistoryBlockEnd;
	uint32_t HistoryCurrentBlock;
	uint16_t HistoryCurrentPos;
	uint32_t HistoryBlockCreating;
	SdFile   StatsFile;
};

Statistics Stats;

#endif

