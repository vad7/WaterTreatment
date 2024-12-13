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
// ------------------------------------------------------------------------------------------------------
// Простой веб серевер с обработкой запросов
// ------------------------------------------------------------------------------------------------------
#include "MainClass.h"
#include "utility/socket.h" 
#include "FreeRTOS_ARM.h"     
// HTTP типы запросов
#define HTTP_invalid   0
#define HTTP_GET       1      // Обычный запрос GET
#define HTTP_REQEST    2      // Наш запрос "GET /&"
#define HTTP_POST      3      // POST передача файла
#define HTTP_POST_     4      // POST - подготовка
#define UNAUTHORIZED   5      // Авторизация не пройдена
#define BAD_LOGIN_PASS 6      // Не верный логин или пароль


extern "C" void TaskGetRunTimeStats(void);
extern void  get_txtSettings(uint8_t thread);
extern void  get_txtJournal(uint8_t thread);
extern void  get_datTest(uint8_t thread);
extern void  get_indexNoSD(uint8_t thread);

const char header_Authorization_1[] = "Authorization: Basic ";
const char header_Authorization_2[] = "&&!Z";
const char* pageUnauthorized      = {"HTTP/1.0 401 Unauthorized\r\nWWW-Authenticate: Basic real_m=Admin Zone\r\nContent-Type: text/html\r\nAccess-Control-Allow-Origin: *\r\n\r\n"};
const char* NO_SUPPORT            = {"not supported"};
const char* NO_STAT               = {"Statistics are not supported in the firmware"};

const char *postRet[]            = {"Настройки из выбранного файла восстановлены, CRC16 OK\r\n\r\n",                           //  Ответы на пост запросы
									"Ошибка восстановления настроек из файла (см. журнал)\r\n\r\n",
									"", // пустая строка - ответ не выводится
									"Файлы загружены, подробности в журнале\r\n\r\n",
									"Ошибка загрузки файла, подробности в журнале\r\n\r\n",
									"Внутренняя ошибка парсера post запросов\r\n\r\n",
									"Флеш диск не найден, загружать файлы некуда\r\n\r\n",
									"Файл настроек для разбора не влезает во внутренний буфер (max 6144 bytes)\r\n\r\n"
									};

#define emptyStr			WEB_HEADER_END  	   // пустая строка после которой начинаются данные
#define MAX_FILE_LEN		64  	              // максимальная длина имени файла
const char Title[]          = "Title: ";          // где лежит имя файла
const char SETTINGS[]       = "*SETTINGS*";       // Идентификатор передачи настроек (лежит в Title:)
const char LOAD_FLASH_START[]= "*SPI_FLASH*";     // Идентификатор начала загрузки веб морды в SPI Flash (лежит в Title:)
const char LOAD_FLASH_END[]  = "*SPI_FLASH_END*"; // Идентификатор колнца загрузки веб морды в SPI Flash (лежит в Title:)
const char LOAD_SD_START[]   = "*SD_CARD*";       // Идентификатор начала загрузки веб морды в SD (лежит в Title:)
const char LOAD_SD_END[]     = "*SD_CARD_END*";   // Идентификатор колнца загрузки веб морды в SD (лежит в Title:)

static SdFile wFile;
static fname_t wfname;

#define ADD_WEBDELIM(str) strcat(str, WEBDELIM)

void web_server(uint8_t thread)
{
	int32_t len;
	int8_t sock;

	if(SemaphoreTake(xWebThreadSemaphore, (W5200_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
		return;
	}          // Захват семафора потока или ОЖИДАНИЕ W5200_TIME_WAIT, если семафор не получен то выходим

	Socket[thread].sock = -1;                      // Сокет свободный

	SPI_switchW5200();                    // Это лишнее но для надежности пусть будет
	for(sock = 0; sock < W5200_SOCK_SYS; sock++)  // Цикл по сокетам веб сервера!!!! служебный не трогаем!!
	{

#if    W5200_THREAD < 2
		if (Socket[0].sock==sock) continue;   // исключение повторного захвата сокетов
#elif  W5200_THREAD < 3
		if((Socket[0].sock==sock)||(Socket[1].sock==sock)) continue; // исключение повторного захвата сокетов
#elif  W5200_THREAD < 4
		if((Socket[0].sock == sock) || (Socket[1].sock == sock) || (Socket[2].sock == sock)) continue; // исключение повторного захвата сокетов
#else
		if((Socket[0].sock==sock)||(Socket[1].sock==sock)||(Socket[2].sock==sock)||(Socket[3].sock==sock)) continue; // исключение повторного захвата сокетов
#endif

		// Настройка  переменных потока для работы
		Socket[thread].http_req_type = HTTP_invalid;        // нет полезной инфы
		SETBIT0(Socket[thread].flags, fABORT_SOCK);          // Сокет сброса нет
		SETBIT0(Socket[thread].flags, fUser); // Признак идентификации как обычный пользователь (нужно подменить файл меню)
		Socket[thread].client = server1.available_(sock);  // надо обработать
		Socket[thread].sock = sock;                           // запомнить сокет с которым рабоатет поток

		if(Socket[thread].client) // запрос http заканчивается пустой строкой
		{
			while(Socket[thread].client.connected()) {
				// Ставить вот сюда
				STORE_DEBUG_INFO(10);
				if(Socket[thread].client.available()) {
					len = Socket[thread].client.get_ReceivedSizeRX();                  // получить длину входного пакета
					if(len > W5200_MAX_LEN) {
						journal.jprintfopt("WEB:Big packet: %d - truncated!\n", len);
#ifdef DEBUG
						journal.jprintfopt("%s\n\n", Socket[thread].inBuf);
#endif
						len = W5200_MAX_LEN; // Ограничить размером в максимальный размер пакета w5200
					}
					if(Socket[thread].client.read(Socket[thread].inBuf, len) != len) {
						journal.jprintfopt("WEB:Read error\n");
						break;
					}
					// Ищем в запросе полезную информацию (имя файла или запрос ajax)
					// пройти авторизацию и разобрать заголовок -  получить имя файла, тип, тип запроса, и признак меню пользователя
					Socket[thread].http_req_type = GetRequestedHttpResource(thread);
					if(Socket[thread].http_req_type != HTTP_POST || len < W5200_MAX_LEN) {
						Socket[thread].inBuf[len + 1] = 0;              // обрезать строку
					}
#ifdef LOG
					journal.jprintfopt("$QUERY: %s\n",Socket[thread].inPtr);
					journal.jprintfopt("$INPUT: %s\n",(char*)Socket[thread].inBuf);
#endif
					switch(Socket[thread].http_req_type)  // По типу запроса
					{
					case HTTP_invalid: {
#ifdef DEBUG
						if(MC.get_NetworkFlags() & ((1<<fWebLogError) | (1<<fWebFullLog))) {
							uint8_t ip[4];
							W5100.readSnDIPR(sock, ip);
							journal.jprintf("WEB:Wrong request %d.%d.%d.%d (%d): ", ip[0], ip[1], ip[2], ip[3], len);
							if(MC.get_NetworkFlags() & (1<<fWebFullLog)) {
								for(int8_t i = 0; i < len; i++) journal.jprintf("%c(%d) ", (char)Socket[thread].inBuf[i], Socket[thread].inBuf[i]);
								journal.jprintf("\n");
							} else {
								for(len = 0; len < 4; len++) journal.jprintf("%d ", Socket[thread].inBuf[len]);
								journal.jprintf("...\n");
							}
						}
#endif
						//sendConstRTOS(thread, "HTTP/1.1 Error GET request\r\n\r\n");
						break;
					}
					case HTTP_GET:     // чтение файла
					{
						// Для обычного пользователя подменить файл меню, для сокращения функционала
						if(GETBIT(Socket[thread].flags, fUser)) {
							if(strstr(Socket[thread].inPtr, ".html")) {
								if(!(strcmp(Socket[thread].inPtr, "index.html") == 0
									|| strcmp(Socket[thread].inPtr, "plan.html") == 0
									|| strcmp(Socket[thread].inPtr, "stats.html") == 0
									|| strcmp(Socket[thread].inPtr, "history.html") == 0
									|| strcmp(Socket[thread].inPtr, "about.html") == 0)) goto xUNAUTHORIZED;
							}
						}
						urldecode(Socket[thread].inPtr, Socket[thread].inPtr, len + 1);
						readFileSD(Socket[thread].inPtr, thread);
						break;
					}
					case HTTP_REQEST:  // Запрос AJAX
					{
						strcpy(Socket[thread].outBuf, HEADER_ANSWER);   // Начало ответа
						parserGET(thread);    // выполнение запроса
#ifdef LOG
						journal.jprintfopt("$RETURN: %s\n",Socket[thread].outBuf);
#endif
						if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), strlen(Socket[thread].outBuf)) == 0) {
							if(MC.get_NetworkFlags() & (1<<fWebLogError)) {
								uint8_t ip[4];
								W5100.readSnDIPR(sock, ip);
								journal.jprintfopt("$Error send AJAX(%d.%d.%d.%d): %s\n", ip[0], ip[1], ip[2], ip[3], (char*) Socket[thread].inBuf);
							}
						}
						break;
					}
					case HTTP_POST:    // загрузка настроек
					{
                        uint8_t ret= parserPOST(thread, len);         // разобрать и получить тип ответа
                        strcpy(Socket[thread].outBuf, HEADER_ANSWER);
						strcat(Socket[thread].outBuf, postRet[ret]);   // вернуть текстовый ответ, который надо вывести
               			if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), strlen(Socket[thread].outBuf)) == 0) {
    						if(MC.get_NetworkFlags() & (1<<fWebLogError)) {
    							uint8_t ip[4];
    							W5100.readSnDIPR(sock, ip);
    							journal.jprintfopt("$Error send POST(%d.%d.%d.%d): %s\n", ip[0], ip[1], ip[2], ip[3], (char*) Socket[thread].inBuf);
    						}
               			}
						break;
					}
					case HTTP_POST_: // предварительный запрос post
					{
						sendConstRTOS(thread,
								"HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: HEAD, OPTIONS, GET, POST\r\nAccess-Control-Allow-Headers: Overwrite, Content-Type, Cache-Control, Title\r\n\r\n");
						break;
					}
					case UNAUTHORIZED: {
xUNAUTHORIZED:
						if(MC.get_NetworkFlags() & (1<<fWebLogError)) {
							uint8_t ip[4];
							W5100.readSnDIPR(sock, ip);
							journal.jprintf("$UNAUTHORIZED (%d.%d.%d.%d)\n", ip[0], ip[1], ip[2], ip[3]);
						}
						sendConstRTOS(thread, pageUnauthorized);
						break;
					}
					case BAD_LOGIN_PASS: {
						if(MC.get_NetworkFlags() & (1<<fWebLogError)) {
							uint8_t ip[4];
							W5100.readSnDIPR(sock, ip);
						}
						sendConstRTOS(thread, pageUnauthorized);
						break;
					}

					default:
						journal.jprintfopt("$Unknow  %s\n", (char*) Socket[thread].inBuf);
					}

					SPI_switchW5200();
					Socket[thread].inBuf[0] = 0;
					break;   // Подготовить к следующей итерации
				} // end if (client.available())
			} // end while (client.connected())
	//		taskYIELD();
			Socket[thread].client.stop();   // close the connection
			Socket[thread].sock = -1;
		//	vTaskDelay(TIME_WEB_SERVER / portTICK_PERIOD_MS); // задержка чтения уменьшаем загрузку процессора
			taskYIELD();
		} // end if (client)
	}  // for (int sock = 0; sock < W5200_SOCK_SYS; sock++)
	SemaphoreGive (xWebThreadSemaphore);              // Семафор отдать
}

//  Чтение файла с SD или его генерация
void readFileSD(char *filename, uint8_t thread)
{
	int n;
	SdFile webFile;
	//  journal.jprintfopt("$Thread: %d socket: %d read file: %s\n",thread,Socket[thread].sock,filename);
	char *str;
	STORE_DEBUG_INFO(11);

	// В начале обрабатываем генерируемые файлы (для выгрузки из контроллера)
	if(strcmp(filename, "state.txt") == 0) { get_txtState(thread, true); return; }
	if(strncmp(filename, "settings", 8) == 0) {
		filename += 8;
		if(strcmp(filename, ".txt") == 0) {
		    get_Header(thread,(char*)"settings.txt");
			get_txtSettings(thread);
			return;
		} else if(strcmp(filename, ".bin") == 0) {
			get_binSettings(thread);
			return;
		} else if(strcmp(filename, "_eeprom.bin") == 0) {
			get_binEeprom(thread);
			return;
		}

		filename -= 8;
	}
	if(strcmp(filename, "journal.txt") == 0) { get_txtJournal(thread); return; }
	if(strncmp(filename, stats_file_start, sizeof(stats_file_start)-1) == 0) { // Файл статистики, stats_yyyy.dat, stats_yyyy.csv
		STORE_DEBUG_INFO(12);
	    strcpy(Socket[thread].outBuf, WEB_HEADER_OK_CT);
	    strcat(Socket[thread].outBuf, WEB_HEADER_BIN_ATTACH);
	    strcat(Socket[thread].outBuf, filename);
	    strcat(Socket[thread].outBuf, "\"");
	    strcat(Socket[thread].outBuf, WEB_HEADER_END);
		n = strncmp(filename + sizeof(stats_file_start)-1, stats_file_header, sizeof(stats_file_header)-1) == 0;
		if((str = strstr(filename, stats_csv_file_ext)) != NULL) { // формат csv - нужен заголовок
			Stats.StatsFileHeader(Socket[thread].outBuf, n);
			strcpy(str, stats_file_ext);
		}
		if(!n) {
			Stats.SendFileData(thread, &webFile, filename);
		} else {
			sendPacketRTOS(thread, (byte*)Socket[thread].outBuf, strlen(Socket[thread].outBuf), 0);
		}
		return;
	}
	if(strncmp(filename, history_file_start, sizeof(history_file_start)-1) == 0) { // Файл Истории полностью: только для бакапа - hist_yyyy.dat, hist_yyyy.csv, обрезанный по периоду - hist__yyyymmdd-yyyymmdd
		STORE_DEBUG_INFO(13);
	    strcpy(Socket[thread].outBuf, WEB_HEADER_OK_CT);
	    strcat(Socket[thread].outBuf, WEB_HEADER_BIN_ATTACH);
	    strcat(Socket[thread].outBuf, filename);
	    strcat(Socket[thread].outBuf, "\"");
	    strcat(Socket[thread].outBuf, WEB_HEADER_END);
	    str = strchr(filename, '-');
	    if(str) { // Period format: "hist__yyyymmdd-yyyymmdd"
	    	filename[sizeof(history_file_start)-1] = '\0';
	    	*str = '\0';
	    	Stats.SendFileDataByPeriod(thread, &webFile, filename, filename + sizeof(history_file_start), str + 1);
	    } else {
			n = strncmp(filename + sizeof(history_file_start)-1, stats_file_header, sizeof(stats_file_header)-1) == 0;
			if((str = strstr(filename, stats_csv_file_ext)) != NULL) { // формат csv - нужен заголовок
				Stats.HistoryFileHeader(Socket[thread].outBuf, n);
				strcpy(str, stats_file_ext);
			}
			if(!n) {
				Stats.SendFileData(thread, &webFile, filename);
			} else {
				sendPacketRTOS(thread, (byte*)Socket[thread].outBuf, strlen(Socket[thread].outBuf), 0);
			}
	    }
		return;
	}
	if(strncmp(filename, "TEST.", 5) == 0) {
		STORE_DEBUG_INFO(14);
		filename += 5;
		if(strcmp(filename, "DAT") == 0) { // TEST.DAT
			get_datTest(thread);
		} else if(strcmp(filename, "NOSD") == 0) { // TEST.NOSD
			get_indexNoSD(thread); // минимальная морда
		} else if(strncmp(filename, "SD:", 3) == 0) {  // TEST.SD:<filename> - Тестирует скорость чтения файла с SD карты
			filename += 3;
			journal.jprintfopt("SD card test: %s - ", filename);
			strcpy(Socket[thread].outBuf, HEADER_FILE_WEB_1);
			strcat(Socket[thread].outBuf, WEB_HEADER_MIME_HTML);
			strcat(Socket[thread].outBuf, HEADER_FILE_WEB_2);
			n = strlen(Socket[thread].outBuf);
			if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == n) {
				SPI_switchSD();
				if(webFile.open(filename, O_READ)) {
					uint32_t startTick = GetTickCount();
					uint32_t size = 0;
					for(;;) {
						int n = webFile.read(Socket[thread].outBuf, sizeof(Socket[thread].outBuf));
						if(n < 0) journal.jprintfopt("Read SD error (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
						if(n <= 0) break;
						size += n;
						if(GetTickCount() - startTick > (3*W5200_TIME_WAIT/portTICK_PERIOD_MS) - 1000) break; // на секунду меньше, чем блок семафора
						WDT_Restart(WDT);
					}
					startTick = GetTickCount() - startTick;
					webFile.close();
					journal.jprintf("read %u bytes, %u b/sec\n", size, (uint64_t)size * 1000 / startTick);
					/*/ check write!
					if(!webFile.open(filename, O_RDWR)) journal.jprintfopt("Error open for writing!\n");
					else {
						n = webFile.write("Test write!");
						journal.jprintfopt("Wrote %d byte\n", n);
						if(!webFile.sync()) journal.jprintfopt("Sync failed (%d,%d)\n", card.cardErrorCode(), card.cardErrorData());
						webFile.close();
					}
					//*/
				}
			} else {
				journal.jprintfopt("not found (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
			}
		}
		return;
	}

// загрузка файла -----------
	// Разбираемся откуда грузить надо (три варианта)
	switch(MC.get_SourceWeb()) {
	case pMIN_WEB: // минимальная морда
		get_indexNoSD(thread);
		break;
	case pSD_WEB:
		{ // Чтение с карты  файлов
			STORE_DEBUG_INFO(15);
			SPI_switchSD();
			if(!webFile.open(filename, O_READ))
			{
				if(card.cardErrorCode() > SD_CARD_ERROR_NONE && card.cardErrorCode() < SD_CARD_ERROR_READ
						&& card.cardErrorData() == 255) { // reinit card
					if(card.begin(PIN_SPI_CS_SD, SD_SCK_MHZ(SD_CLOCK))) {
						if(webFile.open(filename, O_READ)) goto xFileFound;
					} else {
						journal.jprintfopt("Reinit SD card failed!\n");
						//MC.set_fSD(false);
					}
				}
				sendConstRTOS(thread, HEADER_FILE_NOT_FOUND);
				uint8_t ip[4];
				W5100.readSnDIPR(Socket[thread].sock, ip);
				if(MC.get_NetworkFlags() & (1<<fWebLogError)) journal.jprintfopt((char*) "WEB: %d.%d.%d.%d - File not found: %s\n", ip[0], ip[1], ip[2], ip[3], filename);
				return;
			} // файл не найден
xFileFound:
			// Файл открыт читаем данные и кидаем в сеть
	#ifdef LOG
			journal.jprintfopt("$Thread: %d socket: %d read file: %s\n",thread,Socket[thread].sock,filename);
	#endif
			strcpy(Socket[thread].outBuf, HEADER_FILE_WEB_1);
			if(strstr(filename, ".css") != NULL) {
				strcat(Socket[thread].outBuf, WEB_HEADER_MIME_CSS);
			} else if(strstr(filename, ".pdf") != NULL) {
				strcat(Socket[thread].outBuf, WEB_HEADER_MIME_PDF);
			} else strcat(Socket[thread].outBuf, WEB_HEADER_MIME_HTML);
			strcat(Socket[thread].outBuf, HEADER_FILE_WEB_2);
			n = strlen(Socket[thread].outBuf);
			if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == n) {
				SPI_switchSD();
				while((n = webFile.read(Socket[thread].outBuf, sizeof(Socket[thread].outBuf))) > 0) {
					if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == 0) break;
					SPI_switchSD();
				} // while
				if(n < 0) journal.jprintfopt("Read SD error (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
				SPI_switchSD();
				webFile.close();
			}
		}
		break;
	case pFLASH_WEB:
		{
			STORE_DEBUG_INFO(16);
			SerialFlashFile ff = SerialFlash.open(filename);
			if(ff) {
	#ifdef LOG
				journal.jprintfopt("$Thread: %d socket: %d read file: %s\n",thread,Socket[thread].sock,filename);
	#endif
				strcpy(Socket[thread].outBuf, HEADER_FILE_WEB_1);
				if(strstr(filename, ".css") != NULL) {
					strcat(Socket[thread].outBuf, WEB_HEADER_MIME_CSS);
				} else if(strstr(filename, ".pdf") != NULL) {
					strcat(Socket[thread].outBuf, WEB_HEADER_MIME_PDF);
				} else strcat(Socket[thread].outBuf, WEB_HEADER_MIME_HTML);
				strcat(Socket[thread].outBuf, HEADER_FILE_WEB_2);
				n = strlen(Socket[thread].outBuf);
				if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == n) {
					while((n = ff.read(Socket[thread].outBuf, sizeof(Socket[thread].outBuf))) > 0) {
						if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == 0) break;
					} // while
					//if(n < 0) journal.jprintfopt("Read SD error (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
				}
			} else {
				sendConstRTOS(thread, HEADER_FILE_NOT_FOUND);
				uint8_t ip[4];
				W5100.readSnDIPR(Socket[thread].sock, ip);
				if(MC.get_NetworkFlags() & (1<<fWebLogError)) journal.jprintfopt((char*) "WEB GET(%d.%d.%d.%d) - Not found: %s\n", ip[0], ip[1], ip[2], ip[3], filename);
				return;
			}
		}
		break;
	default:
		get_indexNoSD(thread);
		break;
	}
}

// ========================== P A R S E R  G E T =================================
// Разбор и обработка строк запросов buf (начало &) входная строка, strReturn = Socket[0].outBuf выходная
// ТОЛЬКО запросы!
// возвращает число обработанных одиночных запросов
void parserGET(uint8_t thread)
{
	char *str,*x,*y,*z;
	float pm=0;
	int16_t i;
	int32_t l_i32;

	char *buf = Socket[thread].inPtr;
	char *strReturn = Socket[thread].outBuf;
	ADD_WEBDELIM(strReturn);   // начало запроса
	str = strstr(buf,"&&");
	if(str) str[1] = 0;   // Обрезать строку после комбинации &&
	urldecode(buf, buf, W5200_MAX_LEN);
	while ((str = strtok_r(buf, "&", &buf)) != NULL) // разбор отдельных комманд
	{
		STORE_DEBUG_INFO(20);
		strReturn += strlen(strReturn);
		if((strpbrk(str,"="))==0) // Повторить тело запроса и добавить "=" ДЛЯ НЕ set_ запросов
		{
			strcat(strReturn,str);
			*(strReturn += strlen(strReturn)) = '=';
			*(++strReturn) = 0;
		}
		if(strReturn > Socket[thread].outBuf + sizeof(Socket[thread].outBuf) - 250)  // Контроль длины выходной строки - если слишком длинная обрезаем и выдаем ошибку
		{
			journal.jprintfopt("$ERROR - Response buffer overflowed!\n");
			journal.jprintfopt("%s\n",strReturn);
			strcat(strReturn,"E07");
			ADD_WEBDELIM(strReturn);
			break;   // выход из обработки запроса
		}
		// 1. Функции без параметра
		if (strcmp(str,"get_status")==0) // Команда get_status Получить состояние  - основные параметры 
		{
			MC.get_datetime((char*)time_TIME,strReturn); strcat(strReturn,"|SD>");
			MC.get_datetime((char*)time_DATE,strReturn);
#ifdef WEB_STATUS_SHOW_VERSION
			strcat(strReturn,"|SV>");
			strcat(strReturn,VERSION);
#endif
			strcat(strReturn,"|ST>");
			TimeIntervalToStr(MC.get_uptime(),strReturn);
			strcat(strReturn,"|SC>");
			uint32_t t = MC.WorkStats.UsedLastTime;
			if(t) TimeIntervalToStr(rtcSAM3X8.unixtime() - t, strReturn, 1); else strcat(strReturn,"-");
			strcat(strReturn,"|SF>");
			if(UsedWaterContinuousTimer) TimeIntervalToStr(UsedWaterContinuousTimer * USED_WATER_CONTINUOUS_MINTIME, strReturn, 1); else strcat(strReturn,"-");
			strcat(strReturn,"|SI>");
			_itoa(MC.CPU_LOAD,strReturn);
			strcat(strReturn,"%");
			//|SM>");
			//_itoa(freeRam()+MC.startRAM,strReturn); strcat(strReturn,"b|");
			ADD_WEBDELIM(strReturn);
			continue;
		}

		if(strcmp(str, "USR") == 0) {// Команда USR
			strcat(strReturn, GETBIT(Socket[thread].flags, fUser) ? "1" : "0");
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_version")==0) // Команда get_version
		{
			strcat(strReturn,VERSION);
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_uptime")==0) // Команда get_uptime
		{
			TimeIntervalToStr(MC.get_uptime(),strReturn);
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_startDT")==0) // Команда get_startDT
		{
			DecodeTimeDate(MC.get_startDT(),strReturn,3);
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_resetCause")==0) // Команда  get_resetCause
		{
			strcat(strReturn,ResetCause());
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_config")==0)  // Функция get_config
		{
			strcat(strReturn,CONFIG_NAME);
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_configNote")==0)  // Функция get_configNote
		{
			strcat(strReturn,CONFIG_NOTE);
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_freeRam")==0)  // Функция freeRam
		{
			_itoa(freeRam()+MC.startRAM,strReturn);
			strcat(strReturn," b" WEBDELIM) ;
			continue;
		}
		if (strcmp(str,"get_loadingCPU")==0)  // Функция freeRam
		{
			_itoa(MC.CPU_LOAD,strReturn);
			strcat(strReturn,"%" WEBDELIM) ;
			continue;
		}
		if (strcmp(str,"get_socketInfo")==0)  // Функция  get_socketInfo
		{
			socketInfo(strReturn);    // Информация  о сокетах
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if (strcmp(str,"get_socketRes")==0)  // Функция  get_socketRes
		{
			_itoa(MC.socketRes(),strReturn);
			ADD_WEBDELIM(strReturn) ;
			continue;
		}
		if(strncmp(str, "get_list", 8) == 0) // get_list*
		{
			str += 8;
			if(strcmp(str,"Chart")==0)  // Функция get_listChart - получить список доступных графиков
			{
				MC.get_listChart(strReturn);  // строка добавляется
				ADD_WEBDELIM(strReturn) ;
				continue;
			}
			if(strcmp(str,"ADC")==0)     // Функция get_listADC
			{
				for(i=0;i<ANUMBER;i++) if (MC.sADC[i].get_present()){strcat(strReturn,MC.sADC[i].get_name());strcat(strReturn,";");}
				ADD_WEBDELIM(strReturn);
				continue;
			}
			if(strcmp(str,"DSR")==0) {    // Функция get_listDSR
				strcat(strReturn, "---;"); // #0
				for(i = DAILY_RELAY_START_FROM; i < RNUMBER; i++) { // #1..
					strcat(strReturn, MC.dRelay[i].get_name()); strcat(strReturn,";");
				}
				ADD_WEBDELIM(strReturn);
				continue;
			}

			i = 0;
			if(strcmp(str,"Stats")==0) i = 1;   // Функция get_listStats
			else if(strcmp(str,"Hist")==0) i = 2;   // Функция get_listHist
			if(i) { // получить список доступных файлов статистики/истории в формате csv
				str = (char*)(i == 1 ? stats_file_start : history_file_start);
				i = 1;
				x = strReturn;
				for(l_i32 = rtcSAM3X8.get_years(); l_i32 > 2000; l_i32--) {
					x += m_strlen(x);
					m_snprintf(x, 8 + 4 + sizeof(stats_csv_file_ext), "%s%04d%s", str, l_i32, stats_file_ext);
					if(!wFile.opens(x, O_READ, &wfname)) {
						*x = '\0';
						break;
					} else wFile.close();
					if((y = strchr(x, '.'))) strcpy(y, stats_csv_file_ext);
					if(i) {
						strcat(x, ":1;");
						i = 0;
					} else strcat(x, ":0;");
				}
				ADD_WEBDELIM(strReturn) ;
				continue;
			} else goto x_FunctionNotFound;
		}
		if (strcmp(str,"update_NTP")==0)  // Функция update_NTP обновление времени по NTP
		{
			// set_time_NTP();                                                 // Обновить время
			MC.timeNTP=0;                                    // Время обновления по NTP в тиках (0-сразу обновляемся)
			strcat(strReturn,"Update time from NTP");
			ADD_WEBDELIM(strReturn);
			continue;
		}
		if(strcmp(str, "get_NTP") == 0)  // тип NTP
		{
#ifdef HTTP_TIME_REQUEST
			strcat(strReturn, "TCP" WEBDELIM);
#else
			strcat(strReturn, "NTP" WEBDELIM);
#endif
			continue;
		}
		if(strcmp(str, "get_NTPr") == 0)  // Запрос
		{
#ifdef HTTP_TIME_REQUEST
			strcat(strReturn, (char *)&HTTP_TIME_REQ);
#endif
			ADD_WEBDELIM(strReturn);
		}
		if ((strcmp(str,"set_updateNet")==0)||(strcmp(str,"RESET_NET")==0))  // Функция Сброс w5200 и применение сетевых настроек, подождите 5 сек . . .
		{
			MC.fNetworkReset = 3;        // Послать команду сброса и применения сетевых настроек
			strcat(strReturn, "OK");
			ADD_WEBDELIM(strReturn);
			continue;
		}
		if(strncmp(str, "get_MODE", 8)==0) { // Функция get_MODE в каком состояниии
			str += 8;
			if(*str == 'D') {	// get_MODED
				if(CriticalErrors & ERRC_Flooding) {
					strcat(strReturn, "Затопление!");
				} else if(CriticalErrors & ERRC_WaterBooster) {
					strcat(strReturn, "Насосная ст.!");
				} else if(CriticalErrors & ERRC_SepticAlarm) {
					strcat(strReturn, "Септик!");
				} else if(CriticalErrors & ERRC_WeightEmpty) {
					strcat(strReturn, "Реагент!");
				} else if(CriticalErrors & ERRC_TankFillingLong) {
					strcat(strReturn, "Бак!");
				} else if(CriticalErrors & ERRC_LongWaterConsuming) {
					strcat(strReturn, "Долгое потребление!");
				} else if(MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input() || MC.sInput[REG2_ACTIVE].get_Input()) {
					strcat(strReturn, "Регенерация");
				} else if(MC.get_errcode()) {
					strcat(strReturn, "Ошибка: ");
					_itoa(MC.get_errcode(), strReturn);
				} else if(MC.NO_Power) {
					strcat(strReturn, "Нет электричества");
				} else if(LowConsumeMode) {
					strcat(strReturn, "От резерва");
				} else {
					strcat(strReturn, "Ok");
				}
			} else {
				MC.StateToStr(strReturn);
			}
			ADD_WEBDELIM(strReturn); continue;
		}
		if (strcmp(str,"get_testMode")==0)  // Функция get_testMode
		{
			for(i=0;i<=HARD_TEST;i++) // Формирование списка
			{ strcat(strReturn,noteTestMode[i]); strcat(strReturn,":"); if(i==MC.get_testMode()) strcat(strReturn,cOne); else strcat(strReturn,cZero); strcat(strReturn,";");  }
			ADD_WEBDELIM(strReturn) ;    continue;
		}
		if (strcmp(str,"get_remarkTest")==0)  // Функция remarkTest
		{
			switch (MC.get_testMode())
			{
			case NORMAL:    strcat(strReturn,noteRemarkTest[0]);     break; //  Режим работа не тст, все включаем
			case SAFE_TEST: strcat(strReturn,noteRemarkTest[1]);     break; //  Ничего не включаем
			case STAT_TEST:      strcat(strReturn,noteRemarkTest[2]);     break; //  Включаем все кроме компрессора
			case HARD_TEST: strcat(strReturn,noteRemarkTest[3]);     break; //  Все включаем и компрессор тоже
			}
			ADD_WEBDELIM(strReturn) ;    continue;
		}
		STORE_DEBUG_INFO(21);

		if (strncmp(str, "set_SAVE", 8) == 0)  // Функция set_SAVE -
		{
			str += 8;
			if(strcmp(str, "_STATS") == 0) { // Сохранить счетчики и статистику
xSaveStats:		if((i = MC.save_WorkStats()) == OK)
					if((i = Stats.SaveStats(1)) == OK)
						i = Stats.SaveHistory(1);
				_itoa(i, strReturn);
			} else if(strcmp(str, "_UPD") == 0) { // Подготовка к обновлению
				//if(MC.is_compressor_on()) _itoa(-1, strReturn);
				//else
					goto xSaveStats;
				//}
			} else {
				l_i32 = MC.save();   // записать настройки в еепром и получить размер записываемых данных
				_itoa(l_i32, strReturn); // сохранение настроек ВСЕХ!
				MC.save_WorkStats();
			}
			ADD_WEBDELIM(strReturn);
			continue;
		}
		
		STORE_DEBUG_INFO(22);

		if(strncmp(str, "get_err", 7) == 0)
		{
			str += 7;
			if(strcmp(str, "c")==0) { // Функция get_errc
				_itoa(MC.get_errcode(),strReturn);
				ADD_WEBDELIM(strReturn); continue;
			} else {   // Функция get_err
				strcat(strReturn, MC.get_lastErr());
				ADD_WEBDELIM(strReturn); continue;
			}
		}
		if(strcmp(str,"get_tDS")==0)  // Функция get_tDS  - получение температуры DS3231
		{
			_dtoa(strReturn, getTemp_RtcI2C(), 2); ADD_WEBDELIM(strReturn); continue;
		}
		if(strncmp(str,"get_PWR", 7) == 0) {
			str += 7;
#ifdef CHECK_DRAIN_PUMP
			if(*str == 'D') _dtoa(strReturn, DrainPumpPower, 3); // get_PWRD (Drain pump)
			else
#endif
				_dtoa(strReturn, MC.dPWM.get_Power(), 3);
			ADD_WEBDELIM(strReturn); continue;
		}
		if(strcmp(str, "get_WDIS") == 0) { // Выход воды отключен
			strcat(strReturn, MC.dRelay[RWATEROFF1].get_Relay() || !MC.dRelay[RWATERON].get_Relay() ? "1" : "0"); // MC.sInput[REG2_ACTIVE].get_Input()
			ADD_WEBDELIM(strReturn); continue;
		}
		if(strncmp(str + 1, "et_WS", 5) == 0) { // get WorkStats
			i = 0;
			if(*str == 's') { // set
				if((x = strchr(str, '='))) {
					*x++ = '\0';
					l_i32 = atoi(x);
					pm = my_atof(x);
					i = 1;
					strcat(strReturn, str);
					strcat(strReturn, "=");
				}
			}
			str += 6;
			if(strcmp(str, webWS_UsedToday) == 0) _itoa(MC.RTC_store.UsedToday, strReturn); // get_WSUD
			else if(strcmp(str, webWS_UsedYesterday) == 0) _itoa(MC.WorkStats.UsedYesterday, strReturn); // get_WSUY
			else if(strcmp(str, webWS_LastDrain) == 0) {
				if(MC.WorkStats.LastDrain) TimeIntervalToStr(rtcSAM3X8.unixtime() - MC.WorkStats.LastDrain, strReturn, 0); else strcat(strReturn, "-"); // get_WSDD
			} else if(strcmp(str, webWS_RegCnt) == 0) {  // get_WSRC
				if(i) MC.WorkStats.RegCnt = l_i32;  // set_WSRC=x
				_itoa(MC.WorkStats.RegCnt, strReturn);
			} else if(strcmp(str, webWS_RegCntSoftening) == 0) {  // get_WSRSC
				if(i) MC.WorkStats.RegCntSoftening = l_i32; // set_WSRSC=x
				_itoa(MC.WorkStats.RegCntSoftening, strReturn);
			} else if(strcmp(str, webWS_DaysFromLastRegen) == 0) {
				if(i) MC.WorkStats.DaysFromLastRegen = l_i32; // set_WSRD=x
				_itoa(MC.WorkStats.DaysFromLastRegen, strReturn); // get_WSRD
			} else if(strcmp(str, webWS_UsedSinceLastRegen) == 0) _itoa(MC.WorkStats.UsedSinceLastRegen + MC.RTC_store.UsedToday, strReturn); // get_WSRS
			else if(strcmp(str, webWS_UsedLastRegen) == 0) { // get_WSRL
				if(MC.sInput[REG_ACTIVE].get_Input() || MC.sInput[REG_BACKWASH_ACTIVE].get_Input()) {
					strReturn += m_snprintf(strReturn += m_strlen(strReturn), 64, "%d (%d)", MC.RTC_store.UsedRegen, MC.WorkStats.UsedLastRegen);
				} else _itoa(MC.WorkStats.UsedLastRegen, strReturn);
			} else if(strcmp(str, webWS_UsedLastRegenSoftening) == 0) { // get_WSRSL
				if(MC.sInput[REG2_ACTIVE].get_Input()) {
					strReturn += m_snprintf(strReturn += m_strlen(strReturn), 64, "%d (%d)", MC.RTC_store.UsedRegen, MC.WorkStats.UsedLastRegenSoftening);
				} else _itoa(MC.WorkStats.UsedLastRegenSoftening, strReturn);
			} else if(strcmp(str, webWS_DaysFromLastRegenSoftening) == 0) {
				if(i) MC.WorkStats.DaysFromLastRegenSoftening = l_i32; // set_WSRSD=x
				_itoa(MC.WorkStats.DaysFromLastRegenSoftening, strReturn); // get_WSRSD
			} else if(strcmp(str, webWS_UsedSinceLastRegenSoftening) == 0) _itoa(MC.WorkStats.UsedSinceLastRegenSoftening + MC.RTC_store.UsedToday, strReturn); // get_WSRSS
			else if(strcmp(str, webWS_NextRegenSoftAfterDays) == 0) _itoa(MC.NextRegenSoftAfterDays, strReturn); // get_WSNS
			else if(strcmp(str, webWS_NextRegenAfterDays) == 0) _itoa(MC.NextRegenAfterDays, strReturn); // get_WSN
			else if(*str == webWS_UsedDrain) _dtoa(strReturn, MC.WorkStats.UsedDrain, 1); // get_WSD
			else if(*str == webWS_UsedTotal) {  // get_WST
				if(i) MC.WorkStats.UsedTotal = pm * 1000 + 0.0005f; // set_WST=x
				_dtoa(strReturn, MC.WorkStats.UsedTotal + MC.RTC_store.UsedToday, 3);
			} else if(*str == webWS_UsedAverageDay) _itoa(MC.WorkStats.UsedAverageDay / MC.WorkStats.UsedAverageDayNum, strReturn); // get_WSA
			else if(*str == webWS_WaterBoosterCountL) {
				strcat(strReturn, "-");
				if(WaterBoosterFlag && WaterBoosterCountP100) {
					_dtoa(strReturn, WaterBoosterCountP100 * 100 / MC.sFrequency[FLOW].get_kfValue(), 2); // get_WSB
					if(MC.Osmos_PWATER_Added) strcat(strReturn, "*");
				}
			} else if(*str == webWS_Velocity) { // get_WSV, get_WSV2
				l_i32 =  MC.CalcFilteringSpeed(*(str+1) == '2' ? MC.FilterTankSoftenerSquare : MC.FilterTankSquare);
				if(l_i32) {
					_dtoa(strReturn, l_i32, 3);
					strcat(strReturn, " мч");
				}
			} else if(*str == webWS_Flags) {	// get_WSF
				if(i) {
					MC.WorkStats.Flags ^= l_i32 & WS_F_MASK; 		// set_WSF=x (XOR x)
					NeedSaveWorkStats = 1;
				}
				l_i32 = MC.WorkStats.Flags | (MC.get_NeedRegen() << WS_F_bNeedRegen) | (MC.get_NeedRegenSoftening() << WS_F_bNeedRegenSoft);
				_itoa(l_i32, strReturn);
			}
			ADD_WEBDELIM(strReturn); continue;
		}

		if (strcmp(str,"get_OneWirePin")==0)  // Функция get_OneWirePin
		{
#ifdef ONEWIRE_DS2482
			strcat(strReturn, "I2C, DS2482(1");
#ifdef ONEWIRE_DS2482_SECOND
			strcat(strReturn, ",2");
#endif
#ifdef ONEWIRE_DS2482_THIRD
			strcat(strReturn, ",3");
#endif
#ifdef ONEWIRE_DS2482_FOURTH
			strcat(strReturn, ",4");
#endif
			strcat(strReturn, ")" WEBDELIM);
#else
			strcat(strReturn,"D"); _itoa((int)(PIN_ONE_WIRE_BUS),strReturn); ADD_WEBDELIM(strReturn);
#endif
			continue;
		}
		if (strcmp(str,"scan_OneWire")==0)  // Функция scan_OneWire  - сканирование датчикиков
		{
			MC.scan_OneWire(strReturn); ADD_WEBDELIM(strReturn); continue;
		}

		STORE_DEBUG_INFO(23);

		if(strcmp(str,"TEST")==0)   // Команда TEST
		{
			_itoa(random(-50,50),strReturn);
			ADD_WEBDELIM(strReturn);
			continue;
		}

		if(strncmp(str,"TASK_LIST", 9)==0)  // Функция получение списка задач и статистики
		{
			str += 9;
			if (strcmp(str,"_RST")==0)  // Функция сброс статистики по задачам
			{
#ifdef STAT_FREE_RTOS   // определена в utility/FreeRTOSConfig.h
				vTaskResetRunTimeCounters();
#else
				strcat(strReturn,NO_SUPPORT);
#endif
			} else {
#ifdef STAT_FREE_RTOS   // определена в utility/FreeRTOSConfig.h
				//strcat(strReturn,cStrEnd);
				vTaskList(strReturn + m_strlen(strReturn));
#else
				strcat(strReturn,NO_SUPPORT);
#endif
			}
			ADD_WEBDELIM(strReturn);  continue;
		}

		if(strncmp(str, "RESET_", 6)==0) {
			STORE_DEBUG_INFO(24);
			str += 6;
			if(strcmp(str,"TERR")==0) {     // Функция RESET_TERR
				MC.Reset_TempErrors();
			} else if (strcmp(str,"JOURNAL")==0)   // RESET_JOURNAL,  Команда очистки журнала (в зависимости от типа)
			{
#ifdef I2C_JOURNAL_IN_RAM     // журнал в памяти
				strcat(strReturn,"Сброс журнала в RAM ");
				journal.Clear();       // Послать команду на очистку журнала в памяти
				journal.jprintf("Reset RAM journal.\n");
#else                      // Журнал в ЕЕПРОМ
				journal.Format();
#endif
				strcat(strReturn, "OK");
			} else if (strcmp(str,"DUE")==0)   // RESET_DUE, Команда сброса контроллера
			{
				journal.jprintf("CPU RESETING...\n\n");
				MC.save_WorkStats();
				Stats.SaveStats(1);
				Stats.SaveHistory(1);
				NeedSaveRTC = RTC_SaveAll;
				update_RTC_store_memory();
				ResetDUE_countdown = 3;
				strcat(strReturn, "OK");
			} else if(strncmp(str, "CNT_", 4) == 0) { // Команда RESET_CNT_*
				str += 4;
				if(strncmp(str, "VAR_", 4) == 0) {	// RESET_CNT_VAR_xx=n
					str += 5;
					*(str + 2) = '\0';
					if((l_i32 = strtol(str + 3, NULL, 0)) != LONG_MAX) { // NO REENTRANT FUNCTION!
						if(strcmp(str, "D1") == 0) {
							MC.WorkStats.ResetTime = l_i32;
						} else if(strcmp(str, "D2") == 0) {
							MC.WorkStats.UsedLastTime = l_i32;
						} else if(strcmp(str, "A1") == 0) {
							MC.WorkStats.UsedAverageDay = l_i32;
						} else if(strcmp(str, "A2") == 0) {
							MC.WorkStats.UsedAverageDayNum = l_i32;
						}
					}
				} else if(strcmp(str, "FC1") == 0) {	// RESET_CNT_FC1
					MC.WorkStats.FilterCounter1 = 0;
					NeedSaveWorkStats = 1;
				} else if(strcmp(str, "FC2") == 0) {	// RESET_CNT_FC2
					MC.WorkStats.FilterCounter2 = 0;
					NeedSaveWorkStats = 1;
				} else if(strcmp(str, "ALL") == 0) {	// RESET_CNT_ALL
					journal.jprintf("Clear All Counters!\n");
					//strcat(strReturn,"Сброс счетчиков ");
					memset(&MC.RTC_store, 0, sizeof(MC.RTC_store));
					MC.RTC_store.Work = (MC.RTC_store.Work & ~RTC_Work_WeekDay_MASK) | rtcSAM3X8.get_day_of_week();
					NeedSaveRTC = RTC_SaveAll;
					update_RTC_store_memory();
					MC.resetCount();  // Полный сброс
					strcat(strReturn, "OK");
				}
			} else if (strcmp(str,"SETTINGS")==0) // RESET_SETTINGS, Команда сброса настроек
			{
				journal.jprintf("Clear ALL settings!\n");
				uint16_t d = 0;
				writeEEPROM_I2C(I2C_SETTING_EEPROM, (byte*)&d, sizeof(d));
				strcat(strReturn, "Настройки сброшены - перезагрузите контроллер.");
			} else if (strcmp(str,"ERR")==0) // RESET_ERR
			{
				MC.clear_all_errors();
			}
			ADD_WEBDELIM(strReturn); continue;
		}

		if(strncmp(str, "hide_", 5) == 0) { // Удаление элементов внутри tag name="hide_*"
			str += 5;
			ADD_WEBDELIM(strReturn); continue;
		}

		if (strcmp(str,"CONST")==0)   // Команда CONST  Информация очень большая по этому разбито на 2 запроса CONST CONST1
		{
			STORE_DEBUG_INFO(25);
			strcat(strReturn,"VERSION|Версия прошивки|");strcat(strReturn,VERSION);strcat(strReturn,";");
			strcat(strReturn,"__DATE__ __TIME__|Дата и время сборки прошивки|");strcat(strReturn,__DATE__);strcat(strReturn," ");strcat(strReturn,__TIME__) ;strcat(strReturn,";");
			strcat(strReturn,"CONFIG_NAME|Имя конфигурации|");strcat(strReturn,CONFIG_NAME);strcat(strReturn,";");
			strcat(strReturn,"CONFIG_NOTE|");strcat(strReturn,CONFIG_NOTE);strcat(strReturn,"|;");
			strcat(strReturn,"configCPU_CLOCK_HZ|Частота CPU (МГц)|");_itoa(configCPU_CLOCK_HZ/1000000,strReturn);strcat(strReturn,";");
			strcat(strReturn,"SD_SPI_SPEED|Частота SPI SD карты (МГц)|");_itoa(SD_CLOCK, strReturn);strcat(strReturn,";");
			strcat(strReturn,"W5200_SPI_SPEED|Частота SPI сети "); strcat(strReturn,nameWiznet);strcat(strReturn," (МГц)|");_itoa(84/W5200_SPI_SPEED, strReturn);strcat(strReturn,";");
			strcat(strReturn,"I2C_SPEED|Частота работы шины I2C (кГц)|"); _itoa(I2C_SPEED/1000,strReturn); strcat(strReturn,";");
			strcat(strReturn,"UART_SPEED|Скорость отладочного порта (бод)|");_itoa(UART_SPEED,strReturn);strcat(strReturn,";");
			strcat(strReturn,"WDT_TIME|Период сторожевого таймера, 0 - нет (сек)|");_itoa(WDT_TIME,strReturn);strcat(strReturn,";");
#if PWM_MODBUS_ADR >= MODBUS_SERIAL1_ADDR_LESS
			strcat(strReturn,"MODBUS_SERIAL1|Modbus RTU порт счетчика насосной станции|Serial");
			m_snprintf(strReturn + strlen(strReturn), 128, "%d (%d);", GetSerialNum(MODBUS_SERIAL1), MODBUS_SERIAL1_SPEED);
#else
			strcat(strReturn,"MODBUS_SERIAL2|Modbus RTU порт счетчика насосной станции|Serial");
			m_snprintf(strReturn + strlen(strReturn), 128, "%d (%d);", GetSerialNum(MODBUS_SERIAL2), MODBUS_SERIAL2_SPEED);
#endif
#if PWM_MODBUS_ADR < MODBUS_SERIAL1_ADDR_LESS
			strcat(strReturn,"MODBUS_SERIAL1|Modbus RTU порт для насосов|Serial");
			m_snprintf(strReturn + strlen(strReturn), 128, "%d (%d);", GetSerialNum(MODBUS_SERIAL1), MODBUS_SERIAL1_SPEED);
#else
			strcat(strReturn,"MODBUS_SERIAL2|Modbus RTU порт для насосов|Serial");
			m_snprintf(strReturn + strlen(strReturn), 128, "%d (%d);", GetSerialNum(MODBUS_SERIAL2), MODBUS_SERIAL2_SPEED);
#endif
			strcat(strReturn,"MODBUS_PORT_CONFIG|Конфигурация порта|8N1;");
			strcat(strReturn,"MODBUS_TIME_WAIT|Максимальное время ожидания освобождения порта (мсек)|");_itoa(MODBUS_TIME_WAIT,strReturn);strcat(strReturn,";");

			// Карта
			m_snprintf(strReturn + strlen(strReturn), 128, "SD_FAT_VERSION|Версия библиотеки SdFat|%s;", SD_FAT_VERSION);
			m_snprintf(strReturn + strlen(strReturn), 128, "USE_SD_CRC|SD - Использовать проверку CRC|%c;", USE_SD_CRC ? '0'+USE_SD_CRC : USE_SD_CRC_FOR_WRITE ? 'W' : '-');
			strcat(strReturn,"SD_REPEAT|SD - Число попыток чтения, при неудаче переход на работу без карты|");_itoa(SD_REPEAT,strReturn);strcat(strReturn,";");

			// W5200
			strcat(strReturn,"W5200_THREAD|Число потоков для сетевого чипа (web сервера) "); strcat(strReturn,nameWiznet);strcat(strReturn,"|");_itoa(W5200_THREAD,strReturn);strcat(strReturn,";");
			strcat(strReturn,"W5200_TIME_WAIT|Время ожидания захвата мютекса, для управления потоками (мсек)|");_itoa( W5200_TIME_WAIT,strReturn);strcat(strReturn,";");
			strcat(strReturn,"STACK_vWebX|Размер стека для задачи одного web потока "); strcat(strReturn,nameWiznet);strcat(strReturn," (х4 байта)|");_itoa(STACK_vWebX,strReturn);strcat(strReturn,";");
			strcat(strReturn,"W5200_NUM_PING|Число попыток пинга до определения потери связи |");_itoa(W5200_NUM_PING,strReturn);strcat(strReturn,";");
			strcat(strReturn,"W5200_MAX_LEN|Размер аппаратного буфера  сетевого чипа "); strcat(strReturn,nameWiznet);strcat(strReturn," (байт)|");_itoa(W5200_MAX_LEN,strReturn);strcat(strReturn,";");
			strcat(strReturn,"INDEX_FILE|Файл загружаемый по умолчанию|");strcat(strReturn,INDEX_FILE);strcat(strReturn,";");
			strcat(strReturn,"TIME_ZONE|Часовой пояс|");_itoa(TIME_ZONE,strReturn);strcat(strReturn,";");
			// FreeRTOS
			strcat(strReturn,"FREE_RTOS_ARM_VERSION|Версия библиотеки FreeRTOS|");_itoa(FREE_RTOS_ARM_VERSION,strReturn);strcat(strReturn,";");
			strcat(strReturn,"configTICK_RATE_HZ|Квант времени системы FreeRTOS (мкс)|");_itoa(configTICK_RATE_HZ,strReturn);strcat(strReturn,";");

			strcat(strReturn,"TIME_READ_SENSOR|Период опроса датчиков (мсек)|");_itoa(TIME_READ_SENSOR,strReturn);//strcat(strReturn,";");
			ADD_WEBDELIM(strReturn);  continue;
		} // end CONST

		if (strcmp(str,"CONST1")==0)   // Команда CONST1 Информация очень большая по этому разбито на 2 запроса CONST CONST1
		{
#ifdef MODBUS_PUMP_PERIOD
			strcat(strReturn,"MODBUS_PUMP_PERIOD|Период опроса насосов (сек)|"); _itoa(MODBUS_PUMP_PERIOD,strReturn); strcat(strReturn,";");
#endif
			strcat(strReturn,"TIME_WEB_SERVER|Период опроса web сервера "); strcat(strReturn,nameWiznet);strcat(strReturn," (мсек)|");_itoa(TIME_WEB_SERVER,strReturn);strcat(strReturn,";");
			strcat(strReturn,"TIME_I2C_UPDATE |Период синхронизации внутренних часов с I2C часами (мсек)|");_itoa(TIME_I2C_UPDATE,strReturn);strcat(strReturn,";");
			// i2c
			strcat(strReturn,"I2C_COUNT_EEPROM|Адрес внутри чипа I2C с которого пишется счетчики |"); strcat(strReturn,uint16ToHex(I2C_COUNT_EEPROM)); strcat(strReturn,";");
			strcat(strReturn,"I2C_SETTING_EEPROM|Адрес внутри чипа I2C с которого пишутся настройки |"); strcat(strReturn,uint16ToHex(I2C_SETTING_EEPROM)); strcat(strReturn,";");
			// Датчики
			strcat(strReturn,"P_NUMSAMLES|Число значений для усреднения показаний давления|");_itoa(P_NUMSAMLES,strReturn);strcat(strReturn,";");
			strcat(strReturn,"T_NUMSAMLES|Число значений для усреднения показаний температуры|");_itoa(T_NUMSAMLES,strReturn);strcat(strReturn,";");
			strcat(strReturn,"GAP_TEMP_VAL|Допустимая разница показаний между двумя считываниями (°C)|");_dtoa(strReturn,GAP_TEMP_VAL,2);strcat(strReturn,";");
			strcat(strReturn,"MAX_TEMP_ERR|Максимальная систематическая ошибка датчика температуры (°C)|");_dtoa(strReturn,MAX_TEMP_ERR,2);strcat(strReturn,";");
			strcat(strReturn,"PWATER_OSMOS_MIN_DELTA|Минимальная разница между показаниями давления для добавки|");_dtoa(strReturn,PWATER_OSMOS_MIN_DELTA,2);strcat(strReturn,";");
			strcat(strReturn,"PWATER_OSMOS_LASTPRESS_RENEW|Через сколько времени обновлять LastPress, с|");_dtoa(strReturn,PWATER_OSMOS_MIN_DELTA,2);strcat(strReturn,";");

			strcat(strReturn,"VER_SAVE|Версия формата сохраненных данных в I2C памяти|");
			_itoa(VER_SAVE,strReturn);
			//if(VER_SAVE != MC.Option.ver) { strcat(strReturn," ("); _itoa(MC.Option.ver, strReturn); strcat(strReturn,")"); }
			strcat(strReturn,";");
			strcat(strReturn,"DEBUG|Вывод в порт отладочных сообщений|");
#ifdef DEBUG
			strcat(strReturn,"ON;");
#else
			strcat(strReturn,"OFF;");
#endif
			strcat(strReturn,"STAT_FREE_RTOS|Накопление статистики FreeRTOS (отладка)|");
#ifdef STAT_FREE_RTOS
			strcat(strReturn,"ON;");
#else
			strcat(strReturn,"OFF;");
#endif
			strcat(strReturn,"LOG|Вывод в порт лога web сервера|");
#ifdef LOG
			strcat(strReturn,"ON;");
#else
			strcat(strReturn,"OFF;");
#endif
			strcat(strReturn,"CHART_POINT|Максимальное число точек графиков|");_itoa(CHART_POINT,strReturn);strcat(strReturn,";");
			strcat(strReturn,"I2C_JOURNAL_IN_RAM|Место хранения системного журнала|");
#ifndef I2C_JOURNAL_IN_RAM
			strcat(strReturn,"I2C flash memory;");
#else
			strcat(strReturn,"RAM memory;");
#endif
			strcat(strReturn,"JOURNAL_LEN|Размер буфера журнала (байт)|");_itoa(JOURNAL_LEN,strReturn);//strcat(strReturn,";");
			ADD_WEBDELIM(strReturn) ;
			continue;
		} // end CONST1

		if(strncmp(str, "get_sys", 7) == 0)
		{
			str += 7;
			STORE_DEBUG_INFO(26);
			if(strcmp(str, "Info") == 0) { // "get_sysInfo" - Функция вывода системной информации для разработчика
				strcat(strReturn, "Текущая ошибка|");
				if(MC.get_errcode() == OK) strcat(strReturn, "-"); else _itoa(MC.get_errcode(), strReturn);
				strcat(strReturn,";");
				strcat(strReturn, "Источник загрузки web интерфейса |");
				switch (MC.get_SourceWeb())
				{
				case pMIN_WEB:   strcat(strReturn,"internal;"); break;
				case pSD_WEB:    strcat(strReturn,"SD card;"); break;
				case pFLASH_WEB: strcat(strReturn,"SPI Flash;"); break;
				default:         strcat(strReturn,"unknown;"); break;
				}

				strcat(strReturn, "Напряжение питания контроллера, V:|");
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "если ниже %.1d - сброс;", ((SUPC->SUPC_SMMR & SUPC_SMMR_SMTH_Msk) >> SUPC_SMMR_SMTH_Pos) + 19);
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Режим safeNetwork (%sадрес:%d.%d.%d.%d ", defaultDHCP ?"DHCP, ":"",defaultIP[0],defaultIP[1],defaultIP[2],defaultIP[3]);
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "шлюз:%d.%d.%d.%d)|%s;", defaultGateway[0],defaultGateway[1],defaultGateway[2],defaultGateway[3],MC.safeNetwork ?cYes:cNo);
				//strcat(strReturn,"Уникальный ID чипа SAM3X8E|");
				//getIDchip(strReturn); strcat(strReturn,";");
				//strcat(strReturn,"Значение регистра VERSIONR сетевого чипа WizNet (51-w5100, 3-w5200, 4-w5500)|");_itoa(W5200VERSIONR(),strReturn);strcat(strReturn,";");

				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Состояние системы (день недели: %d)|Err:%d(%X) B:%d ", MC.RTC_store.Work & RTC_Work_WeekDay_MASK, MC.get_errcode(), CriticalErrors, WaterBoosterStatus);
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "%s%s Day:%d Reg:%d;", MC.RTC_store.Work & RTC_Work_Regen_F1 ? "R1 " : "", MC.RTC_store.Work & RTC_Work_Regen_F2 ? "R2 " : "", MC.RTC_store.UsedToday, MC.RTC_store.UsedRegen);
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Состояние FreeRTOS при старте (task+err_code) <sup>2</sup>|0x%04X 0x%04X;", lastErrorFreeRtosCode, GPBR->SYS_GPBR[5]);

				startSupcStatusReg |= SUPC->SUPC_SR;                                  // Копим изменения
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Регистры контроллера питания (SUPC) SAM3X8E [SMMR MR SR]|0x%08X %08X %08X", SUPC->SUPC_SMMR, SUPC->SUPC_MR, startSupcStatusReg);  // Регистры состояния контроллера питания
				if((startSupcStatusReg|SUPC_SR_SMS)==SUPC_SR_SMS_PRESENT) strcat(strReturn," bad VDDIN!");
				strcat(strReturn,";");

				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Расчет бака насосной станции|%.2d (%.2d/%d)%c;", MC.Osmos_PWATER_BoosterMax, MC.Osmos_PWATER_BoosterMax_Calc, MC.Osmos_PWATER_BoosterMax_cnt, MC.Osmos_PWATER_Added ? '*' : ' ');

				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Площадь фильтрации обезжелезивателя, м2|%.4d;", MC.FilterTankSquare);
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Площадь фильтрации умягчителя, м2|%.4d;", MC.FilterTankSoftenerSquare);

				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Потреблено с последнего слива осадка, л|%d;", MC.WorkStats.UsedDrainSiltL100 * 100);
				if(RegMaxFlow) strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Макс проток за регенерацию, м3ч|%.3d;", RegMaxFlow);
				if(RegMinPress != 0xFFFF) strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Мин давление при регенерации, атм|%.2d;", RegMinPress);
#ifdef MODBUS_DRAIN_PUMP_RELAY_ADDR
				strcat(strReturn, "Состояние реле дренажного насоса|");
				if(GETBIT(MC.Option.flags2, fDrainPumpRelay)) {
					strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "%s %s;", abs(DrainPumpRelayStatus) == 1 ? "go ->" : "",
																							DrainPumpRelayStatus > 0 ? "ON" : "OFF");
				} else strcat(strReturn, "DISABLED;");
#endif
				STORE_DEBUG_INFO(47);

				strcat(strReturn,"<b> Времена</b>|;");
				strReturn += m_snprintf(strReturn += strlen(strReturn), 256, "Текущее время [%u]|", GetTickCount());
				DecodeTimeDate(rtcSAM3X8.unixtime(),strReturn,3); strcat(strReturn,";");
				strcat(strReturn,"Время сохранения текущих настроек |");DecodeTimeDate(MC.get_saveTime(),strReturn,3);strcat(strReturn,";");
				strcat(strReturn,"Время заполнения бака|");if(MC.RFILL_last_time_ON) DecodeTimeDate(MC.RFILL_last_time_ON,strReturn,3); strcat(strReturn,";");
				strcat(strReturn,"Время последнего потребления|");DecodeTimeDate(MC.WorkStats.UsedLastTime,strReturn,3);strcat(strReturn,";");
				strcat(strReturn,"Максимальное потребление воды за раз|"); TimeIntervalToStr(UsedWaterContinuousTimerMax * USED_WATER_CONTINUOUS_MINTIME, strReturn, 1); strcat(strReturn,";");
#ifdef CHECK_DRAIN_PUMP
				strcat(strReturn,"Последнее включение дренажного насоса|"); DecodeTimeDate(DrainPumpTimeLast, strReturn, 3); strcat(strReturn,";");
#endif

			} else if(strcmp(str, "Info2") == 0) { // "get_sysInfo2" - Функция вывода системной информации для разработчика
				strcat(strReturn,"<b> Счетчики ошибок</b>|;");
				strcat(strReturn,"Счетчик \"Потеря связи с "); strcat(strReturn,nameWiznet);strcat(strReturn,"\", повторная инициализация  <sup>3</sup>|");_itoa(MC.num_resW5200,strReturn);strcat(strReturn,";");
				strcat(strReturn,"Счетчик числа сбросов мютекса захвата шины SPI|");_itoa(MC.num_resMutexSPI,strReturn);strcat(strReturn,";");
				strcat(strReturn,"Счетчик числа сбросов мютекса захвата шины I2C|");_itoa(MC.num_resMutexI2C,strReturn);strcat(strReturn,";");
	#ifdef MQTT
				strcat(strReturn,"Счетчик числа повторных соединений MQTT клиента|");_itoa(MC.num_resMQTT,strReturn);strcat(strReturn,";");
	#endif
				strcat(strReturn,"Счетчик неудачных ping|");_itoa(MC.num_resPing,strReturn);strcat(strReturn,";");
				strcat(strReturn,"Счетчик числа ошибок чтения датчиков температуры|");_itoa(MC.get_errorReadTemp(),strReturn);strcat(strReturn,";");

				strcat(strReturn,"<b> Глобальные счетчики</b>|;");
				strcat(strReturn,"Время сброса|"); DecodeTimeDate(MC.WorkStats.ResetTime, strReturn, 3); strcat(strReturn,";");
				strcat(strReturn,"Счетчик фильтра 1, л|"); _itoa(MC.WorkStats.FilterCounter1 * 100, strReturn);
				if(MC.WorkStats.FilterCounter1 > MC.Option.FilterCounter1_Max) strcat(strReturn," - превышен!");
				strcat(strReturn,";");
				strcat(strReturn,"Счетчик фильтра 2, л|"); _itoa(MC.WorkStats.FilterCounter2 * 100, strReturn);
				if(MC.WorkStats.FilterCounter2 > MC.Option.FilterCounter2_Max) strcat(strReturn," - превышен!");
				strcat(strReturn,";");
				STORE_DEBUG_INFO(48);
				strcat(strReturn,"<b> Статистика за день</b>|;");
				strReturn += strlen(strReturn);
				Stats.StatsWebTable(strReturn);

			} else if(strcmp(str, "Err") == 0) { // "get_sysErr" - Описание всех ошибок
				strReturn += strlen(strReturn);
				for(i = 0; i >= ERR_ERRMAX; i--) {
					strReturn += m_snprintf(strReturn, 256, "%d|%s;", i, noteError[abs(i)]);
					if(strReturn >= Socket[thread].outBuf + sizeof(Socket[thread].outBuf) - 256) {
						if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), strlen(Socket[thread].outBuf)) == 0) {
							journal.jprintf("$Error send get_sysErr!\n");
							break;
						}
						strReturn = Socket[thread].outBuf;
						strReturn[0] = '\0';
					}
				}
			}
			ADD_WEBDELIM(strReturn); continue;
		}
		STORE_DEBUG_INFO(27);

		if (strcmp(str,"test_Mail")==0)  // Функция test_mail
		{
			if (MC.message.setTestMail()) { strcat(strReturn,"Send test mail to "); MC.message.get_messageSetting((char*)mess_SMTP_RCPTTO,strReturn); }
			else { strcat(strReturn,"Error send test mail.");}
			ADD_WEBDELIM(strReturn) ;
			continue;
		}   // test_Mail
		if (strcmp(str,"test_SMS")==0)  // Функция test_mail
		{
			if (MC.message.setTestSMS()) { strcat(strReturn,"Send SMS to "); MC.message.get_messageSetting((char*)mess_SMS_PHONE,strReturn);}  //strcat(strReturn,MC.message.get_messageSetting(pSMS_PHONE));}
			else { strcat(strReturn,"Error send test sms.");}
			ADD_WEBDELIM(strReturn) ;
			continue;
		}   // test_Mail

		// -------------- СПИСКИ ДАТЧИКОВ и ИСПОЛНИТЕЛЬНЫХ УСТРОЙСТВ  -----------------------------------------------------
		// Список аналоговых датчиков выводятся только присутсвующие датчики список вида "name;"
		// Таблица колонка_1|колонка_2;
		if(strncmp(str, "get_tbl", 7) == 0) {
			str += 7;
			if(strcmp(str, "TempF") == 0) // get_tblTempF, Возвращает список датчиков через ";"
			{
				for(i = 0; i < TNUMBER; i++) if(MC.sTemp[i].get_present()) { strcat(strReturn, MC.sTemp[i].get_name()); strcat(strReturn, ";"); }
			} else if(strncmp(str, "Temp", 4) == 0) // get_tblTempN - Возвращает список датчиков через ";", N число в конце - возвращаются датчики имеющие этот бит в SENSORTEMP[]
			{
				uint8_t m = atoi(str + 4);
				for(i = 0; i < TNUMBER; i++)
					if((MC.sTemp[i].get_cfg_flags() & (1<<m)) && ((MC.sTemp[i].get_cfg_flags()&(1<<0)) || MC.sTemp[i].get_fAddress())) {
						strcat(strReturn, MC.sTemp[i].get_name()); strcat(strReturn, ";");
					}
			} else if(strcmp(str,"Input")==0)     // Функция get_tblInput
			{
				for(i=0;i<INUMBER;i++) if(MC.sInput[i].get_present()){strcat(strReturn,MC.sInput[i].get_name());strcat(strReturn,";");}
			} else if(strcmp(str,"Relay")==0)     // Функция get_tblRelay
			{
				for(i=0;i<RNUMBER;i++) if(MC.dRelay[i].get_present()){strcat(strReturn,MC.dRelay[i].get_name());strcat(strReturn,";");}
			} else if(strcmp(str,"Flow")==0)     // Функция get_tblFlow
			{
				for(i=0;i<FNUMBER;i++) if(MC.sFrequency[i].get_present()){strcat(strReturn,MC.sFrequency[i].get_name());strcat(strReturn,";");}
			} else if(strcmp(str,"PDS")==0) {    // Функция get_tblPDS
				for(i = 0; i < DAILY_SWITCH_MAX; i++) {
					if(MC.Option.DailySwitch[i].Device == 0) {
						strcat(strReturn, "---;;00:00;00:00|");
						break;
					}
					strReturn += m_snprintf(strReturn += m_strlen(strReturn), 256, "%s;%s;%02d:%d0;%02d:%d0|", MC.dRelay[MC.Option.DailySwitch[i].Device].get_name(), MC.dRelay[MC.Option.DailySwitch[i].Device].get_note(),
							MC.Option.DailySwitch[i].TimeOn / 10, MC.Option.DailySwitch[i].TimeOn % 10, MC.Option.DailySwitch[i].TimeOff / 10, MC.Option.DailySwitch[i].TimeOff % 10);
				}
#ifdef CORRECT_POWER220
			} else if(strcmp(str,"PwrC")==0) {    // Функция get_tblPwrC
				for(i = 0; i < (int8_t)(sizeof(correct_power220)/sizeof(correct_power220[0])); i++) {
					m_snprintf(strReturn + m_strlen(strReturn), 64, "%s;%d;", MC.dRelay[correct_power220[i].num].get_name(), correct_power220[i].value);
				}
#endif
			} else if(strcmp(str,"Err")==0)     // Функция get_tblErr
			{
				for(i = 0; i < (int16_t)(sizeof(Errors) / sizeof(Errors[0])); i++) {
					if(Errors[i] == OK) break;
					DecodeTimeDate(ErrorsTime[i], strReturn, 3);
					strReturn += m_snprintf(strReturn += m_strlen(strReturn), 128, "|%d|%s\n", Errors[i], noteError[abs(Errors[i])]);
				}
#ifdef REVERSE_OSMOS_FC
				if(MC.WorkStats.RO_FilterCounter1 > MC.Option.RO_FilterCounter1_Max) {
					strReturn += m_snprintf(strReturn += m_strlen(strReturn), 128, "-|%dл|%s ", REVERSE_OSMOS_STR, MC.WorkStats.RO_FilterCounter1 * 100);
					strcat(strReturn, REVERSE_OSMOS_F1_END_STR);
					strcat(strReturn, REVERSE_OSMOS_STR_END);
				}
				if(MC.WorkStats.RO_FilterCounter2 > MC.Option.RO_FilterCounter2_Max) {
					strReturn += m_snprintf(strReturn += m_strlen(strReturn), 128, "-|%dл|%s ", REVERSE_OSMOS_STR, MC.WorkStats.RO_FilterCounter2 * 100);
					strcat(strReturn, REVERSE_OSMOS_F2_END_STR);
					strcat(strReturn, REVERSE_OSMOS_STR_END);
				}
#endif
				if(MC.WorkStats.FilterCounter1 > MC.Option.FilterCounter1_Max) strReturn += m_snprintf(strReturn += m_strlen(strReturn), 128, "-|%dл|Выработан ресурс фильтра %c\n", MC.WorkStats.FilterCounter1 * 100, '1');
				if(MC.WorkStats.FilterCounter2 > MC.Option.FilterCounter2_Max) strReturn += m_snprintf(strReturn += m_strlen(strReturn), 128, "-|%dл|Выработан ресурс фильтра %c\n", MC.WorkStats.FilterCounter1 * 100, '2');
			} else goto x_FunctionNotFound;
			ADD_WEBDELIM(strReturn);
			continue;
		}
#ifdef RADIO_SENSORS
		if(strncmp(str, "set_radio_cmd", 13) == 0) {
			if((x = strchr(str, '='))) {
				radio_sensor_send(x + 1);
			}
			ADD_WEBDELIM(strReturn); continue;
		}
#endif
		if(strncmp(str, "LowConsume", 10) == 0) {
			str += 10;
			if(strncmp(str, "_set=", 5) == 0) {  // Функция "LowConsume_set=x", x = 0, 1
				str += 5;
				LowConsumeMode = atoi(str);
				if(!LowConsumeMode) AfterFilledTimer = 0;
			} else if(strncmp(str, "_Req", 4) == 0) { // LowConsume_Req=... Заменить: & на $
				str += 4;
				if(*str++ == '=') {
					str_replace(str, '$', '&');
					strncpy(MC.Option.LowConsumeRequest, str, sizeof(MC.Option.LowConsumeRequest) - 1);
					*str = '\0';
					strcat(strReturn, str - 1 - 4 - 10);
				}
			    strcat(strReturn, MC.Option.LowConsumeRequest); // LowConsume_Req
				ADD_WEBDELIM(strReturn);
				continue;
			} else _itoa(LowConsumeMode, strReturn); // Функция LowConsume...
			ADD_WEBDELIM(strReturn);
			continue;
		}

		// -----------------------------------------------------------------------------------------------------
		// 2. Функции с параметром ------------------------------------------------------------------------------
		// Ищем скобки ------------------------------------------------------------------------------------------
		STORE_DEBUG_INFO(28);
		if (((x=strpbrk(str,"("))!=0)&&((y=strpbrk(str,")"))!=0))  // Функция с одним параметром - найдена открывающиеся и закрывающиеся скобка
		{
			// Выделяем параметр функции на выходе число - номер параметра
			// применяется кодирование 0-19 - температуры 20-29 - сухой контакт 30-39 -аналоговые датчики
			y[0]=0;             // Стираем скобку ")"  строка х содержит параметр
			*x++ = 0;			// '(' -> '\0'

			// ----------------------------------------------------------------------------------------------------------
			// 2.2 Функции с одним параметром
			// ----------------------------------------------------------------------------------------------------------
			STORE_DEBUG_INFO(29);

			// ------------------------------------------------------------------------
			if (strcmp(str,"set_testMode")==0)  // Функция set_testMode(x)
			{
				if ((pm=my_atof(x))==ATOF_ERROR)  strcat(strReturn,"E09");      // Ошибка преобразования   - завершить запрос с ошибкой
				else
				{
					MC.set_testMode((TEST_MODE)pm);             // Установить режим работы тестирования
					for(i=0;i<=HARD_TEST;i++)                    // Формирование списка
					{ strcat(strReturn,noteTestMode[i]); strcat(strReturn,":"); if(i==MC.get_testMode()) strcat(strReturn,cOne); else strcat(strReturn,cZero); strcat(strReturn,";");  }
				} // else
				ADD_WEBDELIM(strReturn) ;    continue;
			}
			if(strncmp(str, "get_Err", 7) == 0)  // Функция get_Err(X)
			{
				if(*x == 'B') { // get_Err(B) - WaterBoosterError
					strcat(strReturn, CriticalErrors & ERRC_WaterBooster ? "1" : "0");
				} else if(*x == 'F') {  // get_ErrF - FloodingError
					strcat(strReturn, CriticalErrors & ERRC_Flooding ? "1" : "0");
				}
				ADD_WEBDELIM(strReturn);
				continue;
			}

			// -----------------------------------------------------------------------------
			// ПРОФИЛИ функции с одним параметром
			// -----------------------------------------------------------------------------
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//  2.3 Функции с параметрами
			// проверяем наличие функции set_  конструкция типа (TIN=23)
			STORE_DEBUG_INFO(30);
			if((z=strpbrk(x,"=")) != NULL)  // нашли знак "=" запрос на установку параметра
			{
				*z++ = 0; // '=' -> '\0'
				pm = my_atof(z);
				// формируем начало ответа - повторение запроса без параметра установки  ДЛЯ set_ запросов
				strcat(strReturn,str);
				strcat(strReturn,"(");
				strcat(strReturn,x);
				strcat(strReturn,")=");
				//       { strcat(strReturn,"E04");ADD_WEBDELIM(strReturn);  continue;  }
			} // "=" - не обнаружено, значит значение пустая строка

			// Вот сюда будет вставлятся код нового парсера (который не будет кодировать параметры в целые числа)
			// ВХОД str - полное имя запроса до (), x - содержит строку (имя параметра), z - после = (значение), pm - флоат z
			// ВЫХОД strReturn  надо Добавлять + в конце &

			// 2. Проверка для запросов содержащих MQTT ---------------------------------------------
#ifdef MQTT
			if (strcmp(str,"get_MQTT")==0){           // Функция получить настройки MQTT
				MC.clMQTT.get_paramMQTT(x,strReturn);
				ADD_WEBDELIM(strReturn);
				continue;
			} else if (strcmp(str,"set_MQTT")==0) {         // Функция записать настройки MQTT
				if (MC.clMQTT.set_paramMQTT(x,z))  MC.clMQTT.get_paramMQTT(x,strReturn);   // преобразование удачно
				else strcat(strReturn,"E32") ; // ошибка преобразования строки
				ADD_WEBDELIM(strReturn);
				continue;
			}
#endif
			STORE_DEBUG_INFO(31);

			if(strcmp(str, "get_PWM") == 0) {          // Функция получить настройки счетчика
				MC.dPWM.get_param(x, strReturn);
				ADD_WEBDELIM(strReturn); continue;
			} else if(strcmp(str, "set_PWM") == 0) {          // Функция записать настройки счетчика
				if(MC.dPWM.set_param(x, (int32_t)pm)) MC.dPWM.get_param(x, strReturn); // преобразование удачно
				else strcat(strReturn, "E31");            // ошибка преобразования строки
				ADD_WEBDELIM(strReturn); continue;
			}

			//6.  Настройки Уведомлений --------------------------------------------------------
			if (strcmp(str,"get_Message")==0)           // Функция get_Message - получить значение настройки уведомлений
			{
				MC.message.get_messageSetting(x,strReturn);
				ADD_WEBDELIM(strReturn) ; continue;
			} else if (strcmp(str,"set_Message")==0)           // Функция set_Message - установить значениена стройки уведомлений
			{
				if (MC.message.set_messageSetting(x,z)) MC.message.get_messageSetting(x,strReturn); // преобразование удачно
				else strcat(strReturn,"E20") ; // ошибка преобразования строки
				ADD_WEBDELIM(strReturn) ; continue;
			}

			//8.  Настройки дата время --------------------------------------------------------
			if (strcmp(str,"get_datetime")==0)           // Функция get_datetim - получить значение даты времени
			{
				STORE_DEBUG_INFO(33);
				MC.get_datetime(x,strReturn);
				ADD_WEBDELIM(strReturn) ; continue;
			} else if (strcmp(str,"set_datetime")==0)           // Функция set_datetime - установить значение даты и времени
			{
				STORE_DEBUG_INFO(34);
				if (MC.set_datetime(x,z))  MC.get_datetime(x,strReturn);    // преобразование удачно
				else  strcat(strReturn,"E18") ; // ошибка преобразования строки
				ADD_WEBDELIM(strReturn) ; continue;
			}

			//9.  Настройки сети -----------------------------------------------------------
			if (strcmp(str,"get_Net")==0)           // Функция get_Network - получить значение параметра Network
			{
				MC.get_network(x,strReturn);
				ADD_WEBDELIM(strReturn) ; continue;
			} else if (strcmp(str,"set_Net")==0)           // Функция set_Network - установить значение паремтра Network
			{
				if (MC.set_network(x,z))  MC.get_network(x,strReturn);     // преобразование удачно
				else strcat(strReturn,"E15") ; // ошибка преобразования строки
				ADD_WEBDELIM(strReturn);
				break;
			}

			//11.  Графики смещение  используется в одной функции get_Chart -------------------------------------------
			if (strcmp(str,"get_Chart")==0)           // Функция get_Chart - получить график
			{
				MC.get_Chart(x,strReturn);
				ADD_WEBDELIM(strReturn); continue;
			}

			// 13 Опции
			if (strcmp(str,"get_Opt")==0)           // Функция get_option - получить значение параметра
			{
				if(strcmp(x, option_RegenHour)==0) {
x_get_RH:			_itoa(MC.Option.RegenHour & 0x1F, strReturn);
					strcat(strReturn, "-");
					_itoa(((MC.Option.RegenHour /*& 0xE0*/)>>5) + 1, strReturn);
				} else MC.get_option(x,strReturn);
				ADD_WEBDELIM(strReturn); continue;
			} else if (strcmp(str,"set_Opt")==0)           // Функция set_option - установить значение параметра
			{
				if(strcmp(x, option_RegenHour)==0) {
					y = strchr(z, '-');
					if(y) {
						*y++ = '\0';
						l_i32 = ((atoi(y) - 1)<<5);
						if(l_i32 < 0) l_i32 = 0;
						MC.Option.RegenHour = (atoi(z) & 0x1F) + l_i32;
						goto x_get_RH;
					} else strcat(strReturn,"E17");
				} else if(pm != ATOF_ERROR) {   // нет ошибки преобразования
					if (MC.set_option(x,pm)) MC.get_option(x,strReturn);  // преобразование удачно,
					else strcat(strReturn,"E17") ; // выход за диапазон значений
				} else strcat(strReturn,"E11");   // ошибка преобразования во флоат
				ADD_WEBDELIM(strReturn); continue;
			}
			STORE_DEBUG_INFO(37);

#ifdef SENSORS_FREQ_I2C
			if(strcmp(str, "CMD_I2C_2") == 0) {
				Second_I2C_Custom_cmd(x, strReturn);
				ADD_WEBDELIM(strReturn); continue;
			}
#endif

			// str - полное имя запроса до (), x - содержит строку что между (), z - после =
			// код обработки установки значений модбас
			// get_modbus_val(N:D:X), set_modbus_val(N:D:X=YYY)
			// N - номер устройства, D - тип данных, X - адрес, Y - новое значение
			if(strncmp(str+1, "et_modbus", 9) == 0) {
				STORE_DEBUG_INFO(38);
				if(str[11] == 'p') { // set_modbus_p(n=x) - установить параметры протокола Modbus
					l_i32 = pm;
					if(strcmp(x, "timeout") == 0) { // Таймаут
						if(str[0] == 's') Modbus.RS485.ModbusResponseTimeout = l_i32; else l_i32 = Modbus.RS485.ModbusResponseTimeout;
					} else if(strcmp(x, "pause") == 0) { // Пауза между транзакциями
						if(str[0] == 's') Modbus.RS485.ModbusMinTimeBetweenTransaction = l_i32; else l_i32 = Modbus.RS485.ModbusMinTimeBetweenTransaction;
					} else if(strcmp(x, "id") == 0) { // id устройств
						strcat(strReturn, "Serial");
#if PWM_MODBUS_ADR >= MODBUS_SERIAL1_ADDR_LESS
						_itoa(GetSerialNum(MODBUS_SERIAL1), strReturn);
#else
						_itoa(GetSerialNum(MODBUS_SERIAL2), strReturn);
#endif
						strcat(strReturn, ": Счетчик - ");
						_itoa(PWM_MODBUS_ADR, strReturn);
						strcat(strReturn, ". Serial");
#if PWM_MODBUS_ADR < MODBUS_SERIAL1_ADDR_LESS
						_itoa(GetSerialNum(MODBUS_SERIAL1), strReturn);
#else
						_itoa(GetSerialNum(MODBUS_SERIAL2), strReturn);
#endif
						strcat(strReturn, ": Насос - ");
						_itoa(MODBUS_DRAIN_PUMP_ADDR, strReturn);
#ifdef MODBUS_DRAIN_PUMP_RELAY_ADDR
						strcat(strReturn, ", Реле - ");
						_itoa(MODBUS_DRAIN_PUMP_RELAY_ADDR, strReturn);
#endif
						ADD_WEBDELIM(strReturn);
						continue;
					} else goto x_FunctionNotFound;
					_itoa(l_i32, strReturn);
					ADD_WEBDELIM(strReturn);
					continue;
				} else if((y = strchr(x, ':'))) {
					*y++ = '\0';
					uint8_t id = atoi(x);
					uint16_t par = atoi(y + 2);
					i = OK;
					if(strncmp(str, "set", 3) == 0) {
						// strtol - NO REENTRANT FUNCTION!
						if(*y == 'h') i = Modbus.writeHoldingRegisters16(id, par, strtol(z, NULL, 0)); // 1 register (int16).
						//else if(*y == 'u') i = Modbus.writeHoldingRegisters32(id, par, strtol(z, NULL, 0)); // 2 registers (int32).
						else if(*y == 'f') i = Modbus.writeHoldingRegistersFloat(id, par, strtol(z, NULL, 0)); // 2 registers (float).
						else if(*y == 'c') i = Modbus.writeSingleCoil(id, par, atoi(z));	// coil
						else if(*y == 'z') i = Modbus.CustomRequestData(id, y + 2); // Custom = [ id, par ]
						else goto x_FunctionNotFound;
#ifdef MODBUS_TIME_TRANSMISION
						_delay(MODBUS_TIME_TRANSMISION * 10); // Задержка перед чтением
#endif
					} else if(strncmp(str, "get", 3) == 0) {
					} else goto x_FunctionNotFound;
					if(i == OK) {
						if(*y == 'w') {
							if((i = Modbus.readInputRegisters16(id, par, &par)) == OK) _itoa(par, strReturn);
						} else if(*y == 'l') {
							if((i = Modbus.readInputRegisters32(id, par, (uint32_t *)&l_i32)) == OK) _itoa(l_i32, strReturn);
						} else if(*y == 'i') {
							if((i = Modbus.readInputRegistersFloat(id, par, &pm)) == OK) _ftoa(strReturn, pm, 2);
						} else if(*y == 'h') {
							if((i = Modbus.readHoldingRegisters16(id, par, &par)) == OK) _itoa(par, strReturn);
						} else if(*y == 'f') {
							if((i = Modbus.readHoldingRegistersFloat(id, par, &pm)) == OK) _ftoa(strReturn, pm, 2);
						} else if(*y == 'c') {
							l_i32 = 8;
							if((i = Modbus.readCoils(id, par / 8, (uint8_t *)&l_i32)) == OK) _itoa((l_i32 & (1<<par)) != 0, strReturn);
						} else if(*y == 'd') {
							l_i32 = 8;
							if((i = Modbus.readDiscreteInputs(id, par / 8, (uint8_t *)&l_i32)) == OK) _itoa((l_i32 & (1<<par)) != 0, strReturn);
						} else if(*y == 'z') {
							i = Modbus.CustomRequestData(id, y + 2); // Custom = [ id, par ]
						} else goto x_FunctionNotFound;
					}
					if(i != OK) {
						strcat(strReturn, "E"); _itoa(i, strReturn);
					}
					ADD_WEBDELIM(strReturn);
					continue;
				}
			}

#ifdef CORRECT_POWER220
			if(strcmp(str,"set_PwrC")==0) { // Функция установки correct_power220
				for(i = 0; i < (int8_t)(sizeof(correct_power220)/sizeof(correct_power220[0])); i++) {
					if(strcmp(x, MC.dRelay[correct_power220[i].num].get_name()) == 0) {
						correct_power220[i].value = pm;
						strcat(strReturn, z);
						break;
					}
				}
				ADD_WEBDELIM(strReturn);
				continue;
			}
#endif

			//////////////////////////////////////////// массивы датчиков ////////////////////////////////////////////////
			STORE_DEBUG_INFO(40);

			{ // Массивы датчиков
				int p = -1;
				// Датчики температуры смещение
				if(strstr(str,"Temp"))          // Проверка для запросов содержащих Temp
				{
					for(i = 0; i < TNUMBER; i++) {
						if(strcmp(x, MC.sTemp[i].get_name()) == 0) {
							p = i;
							break;
						}
					}
					if(p < 0 || p >= TNUMBER) { // Не соответсвие имени функции и параметра
						strcat(strReturn, "E03");
						ADD_WEBDELIM(strReturn);
						continue;
					} else {  // параметр верный
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strcmp(str,"Temp")==0)              // Функция get_Temp
							{
								if(MC.sTemp[p].get_present() && MC.sTemp[p].get_Temp() != STARTTEMP)  // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sTemp[p].get_Temp(), 2);
								else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if (strncmp(str,"raw",3)==0)           // Функция get_RawTemp
							{ 	if(MC.sTemp[p].get_present() && MC.sTemp[p].get_Temp() != STARTTEMP)  // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sTemp[p].get_rawTemp(), 2);
								else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "full", 4) == 0)         // Функция get_fullTemp
							{
								if(MC.sTemp[p].get_present() && MC.sTemp[p].get_Temp() != STARTTEMP) // Если датчик есть в конфигурации то выводим значение
								{
									if(MC.sTemp[p].get_lastTemp() == STARTTEMP) strcat(strReturn, "-.-");
									else _dtoa(strReturn, MC.sTemp[p].get_Temp(), 2); //MC.sTemp[p].get_fRadio() ? 1 : 2);
								} else strcat(strReturn, "-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn);
								continue;
							}

							if(strncmp(str, "min", 3)==0)           // Функция get_minTemp
							{
								if (MC.sTemp[p].get_present()) // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sTemp[p].get_minTemp(), 2);
								else strcat(strReturn,"-");              // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "max", 3)==0)           // Функция get_maxTemp
							{
								if (MC.sTemp[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sTemp[p].get_maxTemp(), 2);
								else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "er", 2)==0)           // Функция get_erTemp
							{
xget_erTemp:					_dtoa(strReturn, MC.sTemp[p].get_errTemp(), 2); ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "aT", 2) == 0)           // Функция get_aTemp (address)
							{
x_get_aTemp:
								if(!MC.sTemp[p].get_fAddress()) strcat(strReturn, "не привязан");
								else if(MC.sTemp[p].get_fRadio()) _itoa(*(uint32_t*)(MC.sTemp[p].get_address() + 1), strReturn);
								else strcat(strReturn, addressToHex(MC.sTemp[p].get_address()));
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "test", 4)==0)           // Функция get_testTemp
							{
xget_testTemp:					_dtoa(strReturn, MC.sTemp[p].get_testTemp(), 2); ADD_WEBDELIM(strReturn); continue;
							}

							if (strncmp(str, "eT", 2)==0)           // Функция get_eTemp (errcode)
							{ _itoa(MC.sTemp[p].get_lastErr(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if (strncmp(str, "esT", 3) == 0)           // Функция get_esTemp (errors)
							{ _itoa(MC.sTemp[p].get_sumErrorRead(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if (strncmp(str, "isT", 3)==0)           // Функция get_isTemp (present)
							{
								if (MC.sTemp[p].get_present()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "nTemp", 5) == 0)           // Функция get_nTemp, если радиодатчик: добавляется уровень сигнала, если get_nTemp2 - +напряжение батарейки
							{
								strcat(strReturn, MC.sTemp[p].get_note());
	#ifdef RADIO_SENSORS
								if(MC.sTemp[p].get_fRadio()) {
									i = MC.sTemp[p].get_radio_received_idx();
									if(i >= 0) {
										m_snprintf(strReturn + strlen(strReturn), 20, " \xF0\x9F\x93\xB6%c", Radio_RSSI_to_Level(radio_received[i].RSSI));
										if(str[5] == '2') m_snprintf(strReturn + strlen(strReturn), 20, ", %.1dV", radio_received[i].battery);
									} else strcat(strReturn, " \xF0\x9F\x93\xB6");
								}
	#endif
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "bT", 2) == 0)           // Функция get_bTemp (note)
							{
								if(MC.sTemp[p].get_fAddress()) _itoa(MC.sTemp[p].get_bus() + 1, strReturn); else strcat(strReturn, "-");
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "fTemp", 5)==0){  // get_fTempX(N): X - номер флага fTEMP_* (1..), N - имя датчика (flag)
								_itoa(MC.sTemp[p].get_setup_flag(str[5] - '0' - 1 + fTEMP_ignory_errors), strReturn);
								ADD_WEBDELIM(strReturn);  continue;
							}

						// ---- SET ----------------- Для температурных датчиков - запросы на УСТАНОВКУ парметров
						} else if(strncmp(str,"set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "test", 4)==0)           // Функция set_testTemp
							{ 	if (MC.sTemp[p].set_testTemp(rd(pm, 100))==OK)    // Установить значение в сотых градуса
									goto xget_testTemp;
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}       // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
							if(strncmp(str, "er", 2)==0)           // Функция set_erTemp
							{ 	if (MC.sTemp[p].set_errTemp(rd(pm, 100))==OK)    // Установить значение в сотых градуса
									goto xget_erTemp;
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}      // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}

							if(strncmp(str, "fTemp", 5) == 0) {   // set_fTempX(N=V): X - номер флага fTEMP_* (1..), N - имя датчика (flag)
								i = str[5] - '0' - 1 + fTEMP_ignory_errors;
								MC.sTemp[p].set_setup_flag(i, int(pm));
								_itoa(MC.sTemp[p].get_setup_flag(i), strReturn);
								ADD_WEBDELIM(strReturn); continue;
							}

							if (strncmp(str, "aT", 2)==0)        // Функция set_aTemp (address)
							{
								uint8_t n = pm;
								if(n <= TNUMBER)                  // Если индекс находится в диапазоне допустимых значений Здесь индекс начинается с 1, ЗНАЧЕНИЕ 0 - обнуление адреса!!
								{
									if(n == 0) MC.sTemp[p].set_address(NULL, 0);   // Сброс адреса
									else if(OW_scanTable) MC.sTemp[p].set_address(OW_scanTable[n-1].address, OW_scanTable[n-1].bus);
								}
								goto x_get_aTemp;
							}  // вернуть адрес
						}
						strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
						ADD_WEBDELIM(strReturn);
						continue;

					}  // end else
				} //if ((strstr(str,"Temp")>0)

				// РЕЛЕ
				if(strstr(str,"Relay"))          // Проверка для запросов содержащих Relay
				{
					STORE_DEBUG_INFO(44);
					for(i=0;i<RNUMBER;i++) if(strcmp(x,MC.dRelay[i].get_name())==0) {p=i; break;} // Реле
					if ((p<0)||(p>=RNUMBER))  {strcat(strReturn,"E03");ADD_WEBDELIM(strReturn);  continue; }  // Не соответсвие имени функции и параметра
					else  // параметр верный
					{
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strncmp(str, "Relay", 5)==0)           // Функция get_Relay
							{
								if(MC.dRelay[p].get_RelayTimerOn()) strcat(strReturn,cOne); else strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "is", 2)==0)           // Функция get_isRelay
							{
								if (MC.dRelay[p].get_present()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "nR", 2)==0)           // Функция get_nRelay (note)
							{ strcat(strReturn,MC.dRelay[p].get_note()); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "pin", 3)==0)           // Функция get_pinRelay
							{ strcat(strReturn,"D"); _itoa(MC.dRelay[p].get_pinD(),strReturn); ADD_WEBDELIM(strReturn); continue; }

						// ---- SET ----------------- Для реле - запросы на УСТАНОВКУ парметров
						} else if(strncmp(str,"set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "Relay", 5) == 0)           // Функция set_Relay
							{
								if(MC.dRelay[p].set_Relay(pm == 0 ? fR_StatusAllOff : fR_StatusManual) == OK)
								{
									if(MC.dRelay[p].get_Relay()) strcat(strReturn, cOne); else strcat(strReturn, cZero);
									ADD_WEBDELIM(strReturn);
									continue;
								} else { // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
									strcat(strReturn, "E05" WEBDELIM);
									continue;
								}
							}
						}
						strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
						ADD_WEBDELIM(strReturn);
						continue;
					}  // else end
				} //if ((strstr(str,"Relay")>0)  5

				// Датчики аналоговые, давления, ТОЧНОСТЬ СОТЫЕ
				if(strstr(str,"ADC"))          // Проверка для запросов содержащих ADC
				{
					STORE_DEBUG_INFO(41);
					for(i=0;i<ANUMBER;i++) if(strcmp(x,MC.sADC[i].get_name())==0) {p=i; break;} // Поиск среди имен
					if ((p<0)||(p>=ANUMBER))  {strcat(strReturn,"E03");ADD_WEBDELIM(strReturn);  continue; }  // Не соответсвие имени функции и параметра
					else  // параметр верный
					{
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strcmp(str,"ADC")==0)           // Функция get_ADC
							{
								if(MC.sADC[p].get_present())         // Если датчик есть в конфигурации то выводим значение
								{
									_dtoa(strReturn, MC.sADC[p].get_Value(), 2);
								} else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "adc", 3)==0)           // Функция get_adcADC
							{ _itoa(MC.sADC[p].get_lastADC(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "min", 3)==0)           // Функция get_minADC
							{
x_get_minValue: 				_dtoa(strReturn, MC.sADC[p].get_minValue(), 2);
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "max", 3)==0)           // Функция get_maxADC
							{
x_get_maxValue:					_dtoa(strReturn, MC.sADC[p].get_maxValue(), 2);
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "pin", 3)==0)           // Функция get_pinADC
							{
	#ifdef ANALOG_MODBUS
								if(MC.sADC[p].get_fmodbus()) {
									strcat(strReturn,"MB #"); _itoa(ANALOG_MODBUS_REG[p], strReturn);
								} else
	#endif
								{
									strcat(strReturn,"AD"); _itoa(MC.sADC[p].get_pinA(),strReturn);
								}
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "test", 4)==0)           // Функция get_testADC
							{ _dtoa(strReturn, MC.sADC[p].get_testValue(), 2); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "eA", 2)==0)           // Функция get_eADC (errorcode)
							{ _itoa(MC.sADC[p].get_lastErr(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "isA", 3)==0)           // Функция get_isADC
							{
								if (MC.sADC[p].get_present()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "nA", 2)==0)           // Функция get_nADC (note)
							{ strcat(strReturn,MC.sADC[p].get_note()); ADD_WEBDELIM(strReturn); continue; }

							if(*str == 'z')                       // Функция get_zADC
							{ _itoa(MC.sADC[p].get_zeroValue(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "tA", 2)==0)           // Функция get_tADC
							{ _dtoa(strReturn, MC.sADC[p].get_transADC(), 3); ADD_WEBDELIM(strReturn); continue; }

							if(*str == 'G') { 					// get_GADC
x_get_GADC:						i = MC.sADC[p].get_ADC_Gain();
								if(i >= 3) i = 4;
								_itoa(i, strReturn);
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strcmp(str, "ADCLvL")==0) {   // Функция get_ADCLvL()
								l_i32 = MC.sADC[p].get_Value();
								_itoa(l_i32 / 100 + (l_i32 % 100 >= 50 ? 1 : 0), strReturn);
								ADD_WEBDELIM(strReturn); continue;
							}

						// ---- SET ----------------- Для аналоговых  датчиков - запросы на УСТАНОВКУ параметров
						} else if(strncmp(str,"set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "test", 4) == 0)           // Функция set_testADC
							{
								if(MC.sADC[p].set_testValue(rd(pm, 100)) == OK)    // Установить значение
								{
									_dtoa(strReturn, MC.sADC[p].get_testValue(), 2);
									ADD_WEBDELIM(strReturn); continue;
								} else {// выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
									strcat(strReturn, "E05" WEBDELIM);	continue;
								}
							}
							if(strncmp(str, "min", 3) == 0) {  // set_minADC
								if(MC.sADC[p].set_minValue(rd(pm, 100)) == OK) goto x_get_minValue;
								else { strcat(strReturn, "E05" WEBDELIM);  continue; }         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
							if(strncmp(str, "max", 3) == 0) { // set_maxADC
								if(MC.sADC[p].set_maxValue(rd(pm, 100)) == OK) goto x_get_maxValue;
								else {  strcat(strReturn, "E05" WEBDELIM); continue;  }         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}

							if(strncmp(str, "tA", 2)==0)              // Функция set_tADC
							{ if (MC.sADC[p].set_transADC(pm)==OK)    // Установить значение
								{_dtoa(strReturn, MC.sADC[p].get_transADC(), 3); ADD_WEBDELIM(strReturn); continue;}
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}

							if(*str == 'G') { 					// set_GADC
								MC.sADC[p].set_ADC_Gain(pm);
								goto x_get_GADC;
							}

							if(*str == 'z')                      // Функция set_zADC
							{
								if(MC.sADC[p].set_zeroValue((int16_t) pm) == OK)    // Установить значение
								{
									_itoa(MC.sADC[p].get_zeroValue(), strReturn);
									ADD_WEBDELIM(strReturn);
									continue;
								} else {   // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
									strcat(strReturn, "E05" WEBDELIM);	continue;
								}
							}
						}
						strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
						ADD_WEBDELIM(strReturn);
						continue;

					}  // end else
				} //if ((strstr(str,"ADC")>0)

				// Частотные датчики ДАТЧИКИ ПОТОКА
				if(strstr(str,"Flow"))          // Проверка для запросов содержащих Frequency
				{
					STORE_DEBUG_INFO(43);
					for(i=0;i<FNUMBER;i++) if(strcmp(x,MC.sFrequency[i].get_name())==0) {p=i; break;} // Частотные датчики
					if ((p<0)||(p>=FNUMBER))  {strcat(strReturn,"E03");ADD_WEBDELIM(strReturn);  continue; }  // Не соответсвие имени функции и параметра
					else  // параметр верный
					{
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strncmp(str, "Flow", 4)==0)           // Функция get_Flow
							{
								if (MC.sFrequency[p].get_present()) {         // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sFrequency[p].get_Value(), 3);
									if(MC.sFrequency[p].WebCorrectCnt > 1) strcat(strReturn, "+");	// Информация о корректировки
								} else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "rFlow", 4)==0)           // Функция get_rFlow
							{
								if (MC.sFrequency[p].get_present()) {         // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sFrequency[p].get_ValueReal(), 3);
								} else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "PFlow", 4)==0)           // Функция get_PFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sFrequency[p].Passed, 3);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "frF", 3)==0)           // Функция get_frFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sFrequency[p].get_Frequency(), 3);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "kfF", 3)==0)           // Функция get_kfFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sFrequency[p].get_kfValue(), 2);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "cF", 2)==0)           // Функция get_cFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_itoa(MC.sFrequency[p].get_FlowCalcPeriodValue() * FREQ_BASE_TIME_READ / 1000, strReturn);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "eF", 2)==0)           // Функция get_eFlow
							{
#ifdef SENSORS_FREQ_I2C
								_itoa(MC.sFrequency[p].errNum, strReturn);
								l_i32 = MC.sFrequency[p].err;
								if(l_i32) strReturn += m_snprintf(strReturn + strlen(strReturn), 256, "(%d)", l_i32);
#else
								strcat(strReturn, "-");
#endif
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "test", 4)==0)           // Функция get_testFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_dtoa(strReturn, MC.sFrequency[p].get_testValue(), 3);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "pin", 3) == 0)              // Функция get_pinFlow
							{
								if(MC.sFrequency[p].get_I2C_addr()) {
									strcat(strReturn, "I2C_2");
								} else {
									strcat(strReturn, "D");
									_itoa(MC.sFrequency[p].get_pinF(), strReturn);
								}
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "I2C", 3) == 0)              // Функция get_I2CFlow
							{
								_itoa(MC.sFrequency[p].get_I2C_addr(), strReturn);
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "nF", 2) == 0)               // Функция get_nFlow (note)
							{
								strcat(strReturn, MC.sFrequency[p].get_note());
								ADD_WEBDELIM(strReturn); continue;
							}

						// ---- SET ----------------- Для частотных  датчиков - запросы на УСТАНОВКУ парметров
						} else if(strncmp(str, "set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "test", 4) == 0) {           // Функция set_testFlow
								if(MC.sFrequency[p].set_testValue(rd(pm, 1000)) == OK) {   // Установить значение
									_dtoa(strReturn, MC.sFrequency[p].get_testValue(), 3);
									ADD_WEBDELIM(strReturn); continue;
								} else {
									strcat(strReturn, "E35" WEBDELIM); continue;
								}
							}
							if(strncmp(str, "kfF", 3) == 0) {          // Функция set_kfFlow float
								MC.sFrequency[p].set_kfValue(rd(pm, 100));    // Установить значение
								_dtoa(strReturn, MC.sFrequency[p].get_kfValue(), 2);
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "I2C", 2) == 0) {          // Функция set_I2CFlow
								MC.sFrequency[p].set_I2C_addr(pm);
								_itoa(MC.sFrequency[p].get_I2C_addr(), strReturn);
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "cF", 2) == 0) {          // Функция set_cFlow
								l_i32 = int(pm) * 1000 / FREQ_BASE_TIME_READ;
								if(l_i32 < 1) l_i32 = 1;
								MC.sFrequency[p].set_FlowCalcPeriodValue(l_i32);    // Установить значение
								_itoa(MC.sFrequency[p].get_FlowCalcPeriodValue() * FREQ_BASE_TIME_READ / 1000, strReturn);
								ADD_WEBDELIM(strReturn); continue;
							}
						}
						strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
						ADD_WEBDELIM(strReturn);
						continue;
					}  // else end
				} //if ((strstr(str,"Flow")>0)

				//  Датчики сухой контакт
				if(strstr(str,"Input"))          // Проверка для запросов содержащих Input
				{
					STORE_DEBUG_INFO(42);
					for(i=0;i<INUMBER;i++) if(strcmp(x,MC.sInput[i].get_name())==0) {p=i; break;} // Поиск среди имен
					if ((p<0)||(p>=INUMBER))  {strcat(strReturn,"E03");ADD_WEBDELIM(strReturn);  continue; }  // Не соответсвие имени функции и параметра
					else  // параметр верный
					{
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strcmp(str,"Input")==0)           // Функция get_Input
							{
								if (MC.sInput[p].get_present())          // Если датчик есть в конфигурации то выводим значение
								{ if (MC.sInput[p].get_Input()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);}
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "isI", 3)==0)           // Функция get_isInput
							{
								if (MC.sInput[p].get_present()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "nI", 2)==0)           // Функция get_nInput (note)
							{ strcat(strReturn,MC.sInput[p].get_note()); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "test", 4)==0)           // Функция get_testInput
							{
								if (MC.sInput[p].get_testInput()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}

							if(strncmp(str, "alarm", 5)==0)           // Функция get_alarmInput
							{
								if (MC.sInput[p].get_InputLevel()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "eI", 2)==0)           // Функция get_eInput (errorcode)
							{ _itoa(MC.sInput[p].get_lastErr(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "pin", 3)==0)           // Функция get_pinInput
							{ strcat(strReturn,"D"); _itoa(MC.sInput[p].get_pinD(),strReturn);
								ADD_WEBDELIM(strReturn); continue; }
							if(strncmp(str, "type", 4)==0)           // Функция get_typeInput
							{
								if(MC.sInput[p].get_present()) {  // датчик есть в конфигурации
									strcat(strReturn, MC.sInput[i].get_alarm_error() == 0 ? "" : "Тревога");
								} else strcat(strReturn,"none");                                 // датчик отсутвует
								ADD_WEBDELIM(strReturn); continue;
							}

					// ---- SET ----------------- Для датчиков сухой контакт - запросы на УСТАНОВКУ парметров
						} else if(strncmp(str,"set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "test", 4) == 0)           // Функция set_testInput
							{
								if(MC.sInput[p].set_testInput((int16_t) pm) == OK)    // Установить значение
								{
									if(MC.sInput[p].get_testInput()) strcat(strReturn, cOne); else strcat(strReturn, cZero);
									ADD_WEBDELIM(strReturn);
									continue;
								} else { // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
									strcat(strReturn, "E05" WEBDELIM);
									continue;
								}
							}
							if(strncmp(str, "alarm", 5) == 0)           // Функция set_alarmInput
							{
								if(MC.sInput[p].set_InputLevel((int16_t) pm) == OK)    // Установить значение
								{
									if(MC.sInput[p].get_InputLevel()) strcat(strReturn, cOne); else strcat(strReturn, cZero);
									ADD_WEBDELIM(strReturn);
									continue;
								} else { // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
									strcat(strReturn, "E05" WEBDELIM);
									continue;
								}
							}
						}
						strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
						ADD_WEBDELIM(strReturn);
						continue;
					}  // else end
				} //if ((strstr(str,"Input")>0)

				//  Весы
				if(strcmp(str + 4, "Wgt") == 0) {
					if(strncmp(str, "get_", 4) == 0) {
xWgt_get:
						if(*x == 'W') {      	// get_Wgt(W) - Weight
							_dtoa(strReturn, Weight_value, 1);
						} else if(*x == 'T') {      	// get_Wgt(T) - Tare
							_dtoa(strReturn, MC.Option.WeightTare, 1);
						} else if(*x == 'N') {      	// get_Wgt(N) - full brine weight
							_dtoa(strReturn, MC.Option.WeightFull, 0);
						} else if(*x == 'A') {      	// get_Wgt(A) - ADC value
							//_itoa(Weight_adc_filter[Weight_adc_idx ? Weight_adc_idx - 1 : sizeof(Weight_adc_filter) / sizeof(Weight_adc_filter[0]) - 1], strReturn); // one reading
							_itoa(Weight_adc_sum / (sizeof(Weight_adc_filter) / sizeof(Weight_adc_filter[0])), strReturn); // averaged
						} else if(*x == '0') {      	// get_Wgt(0) - Zero (ADC)
							_itoa(MC.Option.WeightZero, strReturn);
						} else if(*x == 'K') {      	// get_Wgt(K) - Coefficient
							_dtoa(strReturn, MC.Option.WeightScale, 4);
						} else if(*x == 'P') {      	// get_Wgt(P) - Pins
							m_snprintf(strReturn + m_strlen(strReturn), 32, "D%d D%d", HX711_DOUT_PIN, HX711_SCK_PIN);
						} else if(*x == 'X') {       		// get_Wgt(X) - Test value
							_dtoa(strReturn, Weight_Test, 1);
						} else if(strcmp(x, "LvL")==0) {       		// get_Wgt(LvL) - Level
							_dtoa(strReturn, Weight_Percent, 2);
						}
						ADD_WEBDELIM(strReturn);
						continue;
					} else if(strncmp(str, "set_", 4) == 0) {
						if(*x == 'T') {      	// set_Wgt(T=) - Tare
							MC.Option.WeightTare = pm * 10 + 0.05f;
						} else if(*x == 'N') {      	// set_Wgt(N=) - full brine weight
							MC.Option.WeightFull = pm;
						} else if(*x == 'K') {      	// set_Wgt(K=) - Coefficient
							MC.Option.WeightScale = (int32_t)pm * 10000;
							if((z = strchr(z, '.'))) { // add 4 digit after decimal point (float has low resolution)
								i = m_strlen(++z);
								if(i > 4) {
									*(z + 4) = '\0';
									i = 0;
								} else i = 4 - i;
								l_i32 = atoi(z);
								while(i--) l_i32 *= 10;
								MC.Option.WeightScale += l_i32;
							}
						} else if(*x == '0') {      	// set_Wgt(0=) - Zero
							MC.Option.WeightZero = pm;
						} else if(*x == 'X') {      	// set_Wgt(X=) - Test value
							Weight_Test = pm * 10;
						}
						goto xWgt_get;
					}
					strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
					ADD_WEBDELIM(strReturn);
					continue;
				}

			} // Массивы датчиков

			// ------------------------ конец разбора -------------------------------------------------
x_FunctionNotFound:
			strcat(strReturn,"E01");                             // функция не найдена ошибка
			ADD_WEBDELIM(strReturn) ;
			continue;
		} // 2. Функции с параметром

		if (str[0]=='&') {break; } // второй символ & подряд признак конца запроса и мы выходим
		strcat(strReturn,"E01");   // Ошибка нет такой команды
		ADD_WEBDELIM(strReturn) ;
	}
	STORE_DEBUG_INFO(45);
	ADD_WEBDELIM(strReturn) ; // двойной знак закрытие посылки
}

// ===============================================================================================================
// Выделение имени файла (или содержания запроса) и типа файла и типа запроса клиента
// thread - номер потока, возсращает тип запроса
uint16_t GetRequestedHttpResource(uint8_t thread)
{
	STORE_DEBUG_INFO(50);
	if((MC.get_fPass()) && (!MC.safeNetwork))  // идентификация если установлен флаг и перемычка не в нуле
	{
		//Serial.print("\n"); Serial.print((char*)Socket[thread].inBuf); Serial.print("\n");
		char *str = strstr((char*)Socket[thread].inBuf, header_Authorization_1);
		if(str) str += sizeof(header_Authorization_1) - 1;
		else if((str = strstr((char*)Socket[thread].inBuf, header_Authorization_2))) str += sizeof(header_Authorization_2) - 1;
		if(str) {
			if(WebSec_admin.hash && strncmp(str, WebSec_admin.hash, WebSec_admin.len) == 0) goto x_ok;
			else if(!*MC.get_passUser() || !WebSec_user.hash || strncmp(str, WebSec_user.hash, WebSec_user.len) == 0) SETBIT1(Socket[thread].flags, fUser);
			else return BAD_LOGIN_PASS;
		} else if(!*MC.get_passUser()) SETBIT1(Socket[thread].flags, fUser); else return UNAUTHORIZED;
	}
x_ok:
	// Идентификация пройдена
	//if(strstr((char*)Socket[thread].inBuf,"Access-Control-Request-Method: POST")) {request_type = HTTP_POST_; return request_type; }  //обработка предваритаельного запроса перед получением файла
	char *str_token, *tmpptr;
	str_token = strtok_r((char*) Socket[thread].inBuf, " ", &tmpptr);    // Обрезаем по пробелам
	if(strcmp(str_token, "GET") == 0)   // Ищем GET
	{
		str_token = strtok_r(NULL, " ", &tmpptr);                       // get the file name
		if(strcmp(str_token, "/") == 0)                   // Имени файла нет, берем файл по умолчанию
		{
			Socket[thread].inPtr = (char*) INDEX_FILE;      // Указатель на имя файла по умолчанию
			return HTTP_GET;
		} else if(strlen(str_token) <= W5200_MAX_LEN - 100)   // Проверка на длину запроса или имени файла
		{
			Socket[thread].inPtr = (char*) (str_token + 1);       // Указатель на имя файла
			if(Socket[thread].inPtr[0] == '&') return HTTP_REQEST;       // Проверка на аякс запрос
			return HTTP_GET;
		} // if ((len=strlen(str_token)) <= W5200_MAX_LEN-100)
		else return HTTP_invalid;  // слишком длинная строка HTTP_invalid
	}   //if (strcmp(str_token, "GET") == 0)
	else if(strcmp(str_token, "POST") == 0) {Socket[thread].inPtr = (char*) (str_token +strlen("POST") + 1);  return HTTP_POST;}    // Запрос POST Socket[thread].inPtr - указывает на начало запроса (начало полезных данных)
	else if(strcmp(str_token, "OPTIONS") == 0) return HTTP_POST_;
	return HTTP_invalid;
}

// ========================== P A R S E R  P O S T =================================
uint16_t numFilesWeb = 0;                   // Число загруженных файлов

// Разбор и обработка POST запроса inPtr входная строка использует outBuf для хранения файла настроек!
// Сейчас реализована загрузка настроек и загрузка веб морды в спи диск
// Возврат тип ответа (потом берется из массива строк)
TYPE_RET_POST parserPOST(uint8_t thread, uint16_t size)
{
	byte *ptr, *pStart;
	char *nameFile;      // указатель имя файла
	int32_t buf_len, lenFile;

	//journal.printf(" POST =>"); journal.printf("%s\n", Socket[thread].inPtr); if(strlen(Socket[thread].inPtr) >= PRINTF_BUF) journal.printf("%s\n", Socket[thread].inPtr + PRINTF_BUF - 1);
	STORE_DEBUG_INFO(51);

	// Поиски во входном буфере: данных, имени файла и длины файла
	ptr = (byte*) strstr(Socket[thread].inPtr, emptyStr);     // поиск начала данных
	if(!ptr) return pLOAD_ERR;
	ptr += sizeof(emptyStr) - 1;
	nameFile = strstr(Socket[thread].inPtr, Title);
	pStart = (byte*) strstr(Socket[thread].inPtr, http_Length);
	if(pStart) pStart += sizeof(http_Length) - 1;
	if(nameFile) {
		char *p = strchr(nameFile += sizeof(Title) - 1, '\r');
		if(p) *p = '\0'; else nameFile = NULL;
	}
	if(!nameFile) { // Имя файла не найдено, запрос не верен, выходим
		journal.jprintf("Upload: Name not found!\n");
		return pLOAD_ERR;
	}
	urldecode(nameFile, nameFile, MAX_FILE_LEN + 10);
	if(strlen(nameFile) > MAX_FILE_LEN) {
		journal.jprintf("Upload: %s name length > %d bytes!\n", nameFile, MAX_FILE_LEN - 1);
		return pLOAD_ERR;
	}
	if(!pStart) { // Размер файла не найден, запрос не верен, выходим
xLenErr:
		journal.jprintf("Upload: %s - length not found!\n", nameFile);
		return pLOAD_ERR;
	}
	lenFile = strtol((char*) pStart, NULL, 10);	// получить длину
	// все нашлось, можно обрабатывать
	buf_len = size - (ptr - (byte *) Socket[thread].inBuf);                  // длина (остаток) данных (файла) в буфере
	// В зависимости от имени файла (Title)
	if(strcmp(nameFile, SETTINGS) == 0) {  // Чтение настроек
		if(lenFile <= 0) goto xLenErr;
		STORE_DEBUG_INFO(52);
		int32_t len;
		// Определение начала данных (поиск HEADER_BIN)
xContinueSearchHeader:
		pStart = (byte*)strstr((char*) ptr, HEADER_BIN);    // Поиск заголовка
		if(pStart == NULL) {              // Заголовок не найден
			if((len = Socket[thread].client.available())) {
				if(len > W5200_MAX_LEN) len = W5200_MAX_LEN;
				Socket[thread].client.read(ptr = (uint8_t*)Socket[thread].inBuf, len);            // прочитать буфер
				goto xContinueSearchHeader;
			}
			journal.jprintf("Upload: Wrong save format: %s!\n", nameFile);
			//if(__.get_NetworkFlags() & (1<<fWebFullLog)) journal.jprintf("[Avail:%d] %s\n\n", Socket[thread].client.available(), ptr);
			return pSETTINGS_ERR;
		}
		len=pStart+sizeof(HEADER_BIN)-1 - (byte*) Socket[thread].inBuf;         // размер текстового заголовка в буфере до окончания HEADER_BIN, дальше идут бинарные данные
		buf_len = size - len;                                                   // определяем размер бинарных данных в первом пакете
		memcpy(Socket[thread].outBuf, pStart+sizeof(HEADER_BIN)-1, buf_len);    // копируем бинарные данные в буфер, без заголовка!
		while(1)  // Чтение остальных бинарных данных по сети
		{
			for(uint8_t i = 0; i < 255; i++) {
				if(!Socket[thread].client.available()) _delay(1); else break; // ждем получение пакета
			}
			if(!Socket[thread].client.available()) break;                                          // пакета нет - выходим
			len = Socket[thread].client.get_ReceivedSizeRX();                                      // получить длину входного пакета
			if(len > W5200_MAX_LEN) len = W5200_MAX_LEN;                                   		// Ограничить размером в максимальный размер пакета w5200
			if(buf_len + len > (int32_t)sizeof(Socket[thread].outBuf)) return pSETTINGS_MEM;     // проверить длину если не влезает то выходим
			Socket[thread].client.read((uint8_t*)Socket[thread].outBuf + buf_len, len);            // прочитать буфер
			buf_len = buf_len + len;                                                               // определить размер данных
		}
	    ptr = (byte*) Socket[thread].outBuf;
		journal.jprintf("Loading %s, length data %d bytes:\n", SETTINGS, buf_len);
		// Чтение настроек из ptr
		len = MC.load(ptr, 1);
		if(len <= 0) return pSETTINGS_ERR; // ошибка загрузки настроек
		return pSETTINGS_OK;
	} //if (strcmp(nameFile,"*SETTINGS*")==0)

	// загрузка вебморды
	 else if(MC.get_fSPIFlash() || MC.get_fSD())  { // если есть куда писать
		STORE_DEBUG_INFO(53);
		if(strcmp(nameFile, LOAD_FLASH_START) == 0) {  // начало загрузки вебморды в SPI Flash
			if(!MC.get_fSPIFlash()) {
				journal.jprintf("Upload: No SPI Flash installed!\n");
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_ERR;
			}
			fWebUploadingFilesTo = 1;
			if(SemaphoreTake(xLoadingWebSemaphore, 10) == pdFALSE) {
				journal.jprintf("%s: Upload already started\n", (char*) __FUNCTION__);
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_ERR;
			}
			numFilesWeb = 0;
			journal.jprintf_time("Start upload, erase SPI disk ");
			SerialFlash.eraseAll();
			while(SerialFlash.ready() == false) {
				xSemaphoreGive(xWebThreadSemaphore); // отдать семафор вебморды, что бы обработались другие потоки веб морды
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				if(SemaphoreTake(xWebThreadSemaphore, (3 * W5200_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) { // получить семафор веб морды
					journal.jprintf("%s: Socket %d %s\n", (char*) __FUNCTION__, Socket[thread].sock, MutexWebThreadBuzy);
					return pLOAD_ERR;
				} // если не удается захватить мютекс, то ошибка и выход
				journal.jprintf(".");
			}
			journal.jprintf(" Ok, free %d bytes\n", SerialFlash.free_size());
			return pNULL;
		} else if(strcmp(nameFile, LOAD_SD_START) == 0) {  // начало загрузки вебморды в SD
			if(!MC.get_fSD()) {
				journal.jprintf("Upload: No SD card available!\n");
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_ERR;
			}
			fWebUploadingFilesTo = 2;
			if(SemaphoreTake(xLoadingWebSemaphore, 10) == pdFALSE) {
				journal.jprintf("%s: Upload already started\n", (char*) __FUNCTION__);
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_ERR;
			}
			numFilesWeb = 0;
			journal.jprintf_time("Start upload to SD.\n");
			return pNULL;
		} else if(strcmp(nameFile, LOAD_FLASH_END) == 0 || strcmp(nameFile, LOAD_SD_END) == 0) {  // Окончание загрузки вебморды
			if(SemaphoreTake(xLoadingWebSemaphore, 0) == pdFALSE) { // Семафор не захвачен (был захвачен ранее) все ок
				journal.jprintf_time("Ok, %d files uploaded, free %.1f KB\n", numFilesWeb, fWebUploadingFilesTo == 1 ? (float)SerialFlash.free_size() / 1024 : (float)card.vol()->freeClusterCount() * card.vol()->blocksPerCluster() * 512 / 1024);
				fWebUploadingFilesTo = 0;
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_OK;
			} else { 	// семафор БЫЛ захвачен, ошибка, отдать обратно
				journal.jprintf("%s: Unable to finish upload!\n", (char*) __FUNCTION__);
				fWebUploadingFilesTo = 0;
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_ERR;
			}
		} else { // загрузка отдельных файлов веб морды
			if(SemaphoreTake(xLoadingWebSemaphore, 0) == pdFALSE) { // Cемафор занят - загрузка файла
				if(lenFile == 0) {
					journal.jprintf("Upload: %s length = %s!\n", nameFile, pStart);
					return pLOAD_ERR;
				}
				// Файл может лежать во множестве пакетов. Если в SPI Flash, то считается что spi диск отформатирован и ожидает запись файлов с "нуля"
				// Входные параметры:
				// nameFile - имя файла
				// lenFile - общая длина файла
				// thread - поток веб сервера,котрый обрабатывает post запрос
				// ptr - указатель на начало данных (файла) в буфере Socket[thread].inPtr.
				// buf_len - размер данных в буфере ptr (по сети осталось принять lenFile-buf_len)
				if(fWebUploadingFilesTo == 1) { // SPI flash
					uint16_t numPoint = 0;
					int32_t loadLen; // Обработанная (загруженная) длина
					STORE_DEBUG_INFO(54);
					if(MC.get_NetworkFlags() & (1<<fWebFullLog)) journal.jprintfopt("%s (%d) ", nameFile, lenFile);
					loadLen = SerialFlash.free_size();
					if(lenFile > loadLen) {
						journal.jprintf("Not enough space, free: %d\n", loadLen);
						loadLen = 0;
					} else {
						loadLen = SerialFlash.create(nameFile, lenFile);
						if(loadLen == 0) {
							SerialFlashFile ff = SerialFlash.open(nameFile);
							if(ff) {
								if(buf_len > 0) loadLen = ff.write(ptr, buf_len); // первый пакет упаковали если он не нулевой
								while(loadLen < lenFile)  // Чтение остальных пакетов из сети
								{
									_delay(2);                                                 // время на приход данных
									buf_len = Socket[thread].client.get_ReceivedSizeRX(); // получить длину входного пакета
									if(buf_len == 0) {
										if(Socket[thread].client.connected()) continue;	else break;
									}
									//      if(len>W5200_MAX_LEN-1) len=W5200_MAX_LEN-1;                             // Ограничить размером в максимальный размер пакета w5200
									Socket[thread].client.read(Socket[thread].inBuf, buf_len);        // прочитать буфер
									loadLen = loadLen + ff.write(Socket[thread].inBuf, buf_len);             // записать
									numPoint++;
									if(numPoint >= 20) {                   // точка на 30 кб приема (20 пакетов по 1540)
										numPoint = 0;
										if(MC.get_NetworkFlags() & (1<<fWebFullLog)) journal.jprintfopt(".");
									}
								}
								ff.close();
								if(loadLen == lenFile) journal.jprintfopt("Ok\n");
								else { // Длины не совпали
									journal.jprintf("%db, Error length!\n", loadLen);
									loadLen = 0;
								}
							} else journal.jprintf("Error Open!\n");
						} else journal.jprintf("Error Create %d!\n", loadLen);
					}
					if(loadLen == lenFile) {
						numFilesWeb++;
						return pNULL;
					} else {
						SemaphoreGive (xLoadingWebSemaphore);
						return pLOAD_ERR;
					}
				} else if(fWebUploadingFilesTo == 2) { // Запись на SD,
					STORE_DEBUG_INFO(54);
					if(MC.get_NetworkFlags() & (1<<fWebFullLog)) journal.jprintfopt("%s (%d) ", nameFile, lenFile);
					for(uint16_t _timeout = 0; _timeout < 2000 && card.card()->isBusy(); _timeout++) _delay(1);
					if(wFile.opens(nameFile, O_CREAT | O_TRUNC | O_RDWR, &wfname)) {
						wFile.timestamp(T_CREATE | T_ACCESS | T_WRITE, rtcSAM3X8.get_years(), rtcSAM3X8.get_months(), rtcSAM3X8.get_days(), rtcSAM3X8.get_hours(), rtcSAM3X8.get_minutes(), rtcSAM3X8.get_seconds());
						if((int32_t)wFile.write(ptr, buf_len) != buf_len) {
							journal.jprintf("Error write file %s (%d,%d)!\n", nameFile, card.cardErrorCode(), card.cardErrorData());
						} else {
							uint16_t numPoint = 0;
							while((lenFile -= buf_len) > 0)  // Чтение остальных пакетов из сети
							{
								_delay(2);                                                                 // время на приход данных
								buf_len = Socket[thread].client.get_ReceivedSizeRX();                  // получить длину входного пакета
								if(buf_len == 0) {
									if(Socket[thread].client.connected()) continue; else break;
								}
								Socket[thread].client.read(Socket[thread].inBuf, buf_len);                      // прочитать буфер
								STORE_DEBUG_INFO(56);
								for(uint16_t _timeout = 0; _timeout < 2000 && card.card()->isBusy(); _timeout++) _delay(1);
								if((int32_t)wFile.write(Socket[thread].inBuf, buf_len) != buf_len) {
									journal.jprintf("Error write file %s (%d,%d)!\n", nameFile, card.cardErrorCode(), card.cardErrorData());
									break;
								}
								STORE_DEBUG_INFO(57);
								if(++numPoint >= 20) {// точка на 30 кб приема (20 пакетов по 1540)
									numPoint = 0;
									if(MC.get_NetworkFlags() & (1<<fWebFullLog)) journal.jprintfopt(".");
								}
							}
						}
						STORE_DEBUG_INFO(58);
						if(!wFile.close()) {
							journal.jprintf("Error close file (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
						}
						if(lenFile == 0) {
							if(MC.get_NetworkFlags() & (1<<fWebFullLog)) journal.jprintfopt("Ok\n");
						} else journal.jprintf("Error - rest %d!\n", lenFile);
					} else journal.jprintf("Error create (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
					if(lenFile == 0) {
						numFilesWeb++;
						return pNULL;
					} else {
						SemaphoreGive (xLoadingWebSemaphore);
						return pLOAD_ERR;
					}
				}
			} else { // семафор БЫЛ захвачен, ошибка, отдать обратно
				uint8_t ip[4];
				W5100.readSnDIPR(Socket[thread].sock, ip);
				journal.jprintf("Unable to upload file %s (%d.%d.%d.%d)!\n", nameFile, ip[0], ip[1], ip[2], ip[3]);
				SemaphoreGive (xLoadingWebSemaphore);
				return pLOAD_ERR;
			}
		}
	} else {
		journal.jprintf("%s: Upload: No web store!\n", (char*) __FUNCTION__);
		SemaphoreGive (xLoadingWebSemaphore);
		return pNO_DISK;
	}
	return pPOST_ERR; // До сюда добегать не должны
}
