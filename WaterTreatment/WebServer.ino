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
extern void  get_fileState(uint8_t thread);
extern void  get_fileSettings(uint8_t thread);
extern void  get_txtJournal(uint8_t thread);
extern uint16_t get_csvStatistic(uint8_t thread);
extern void  get_datTest(uint8_t thread);
extern int16_t  get_indexNoSD(uint8_t thread);
extern void  noCsvStatistic(uint8_t thread);


// Названия режимов теста
const char *noteTestMode[] =   {"NORMAL","SAFE_TEST","TEST","HARD_TEST"};
// Описание режима теста
static const char *noteRemarkTest[] = {"Тестирование отключено, основной режим работы.",
                                       "Значения датчиков берутся из полей 'Тест', работа исполнительных устройств эмулируется - Безопасно.",
                                       "Значения датчиков берутся из полей 'Тест', исполнительные устройства работают за исключением компрессора (FC и RCOMP) - Почти безопасно.",
                                       "Значения датчиков берутся из полей 'Тест', все исполнительные устройства работают. Внимание! Может быть поврежден компрессор!"};
                               
                               
const char* file_types[] = {"text/html", "image/x-icon", "text/css", "application/javascript", "image/jpeg", "image/png", "image/gif", "text/plain", "text/ajax"};

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
						journal.jprintf("WEB:Big packet: %d - truncated!\n", len);
#ifdef DEBUG
						journal.jprintf("%s\n\n", Socket[thread].inBuf);
#endif
						len = W5200_MAX_LEN; // Ограничить размером в максимальный размер пакета w5200
					}
					if(Socket[thread].client.read(Socket[thread].inBuf, len) != len) {
						journal.jprintf("WEB:Read error\n");
						break;
					}
					// Ищем в запросе полезную информацию (имя файла или запрос ajax)
					// пройти авторизацию и разобрать заголовок -  получить имя файла, тип, тип запроса, и признак меню пользователя
					Socket[thread].http_req_type = GetRequestedHttpResource(thread);
					if(Socket[thread].http_req_type != HTTP_POST || len < W5200_MAX_LEN) {
						Socket[thread].inBuf[len + 1] = 0;              // обрезать строку
					}
#ifdef LOG
					journal.jprintf("$QUERY: %s\n",Socket[thread].inPtr);
					journal.jprintf("$INPUT: %s\n",(char*)Socket[thread].inBuf);
#endif
					switch(Socket[thread].http_req_type)  // По типу запроса
					{
					case HTTP_invalid: {
#ifdef DEBUG
						uint8_t ip[4];
						W5100.readSnDIPR(sock, ip);
						journal.jprintf("WEB:Wrong request %d.%d.%d.%d (%d): ", ip[0], ip[1], ip[2], ip[3], len);
						//for(int16_t i = 0; i < len; i++) journal.jprintf("%c(%d) ", (char)Socket[thread].inBuf[i], Socket[thread].inBuf[i]);
						for(len = 0; len < 4; len++) journal.jprintf("%d ", Socket[thread].inBuf[len]);
						journal.jprintf("...\n");
#endif
						//sendConstRTOS(thread, "HTTP/1.1 Error GET request\r\n\r\n");
						break;
					}
					case HTTP_GET:     // чтение файла
					{
						// Для обычного пользователя подменить файл меню, для сокращения функционала
						if(GETBIT(Socket[thread].flags, fUser)) {
							if(strcmp(Socket[thread].inPtr, "menu.js") == 0) strcpy(Socket[thread].inPtr, "menu-user.js");
							else if(strstr(Socket[thread].inPtr, ".html")) {
								if(!(strcmp(Socket[thread].inPtr, "index.html") == 0
									|| strcmp(Socket[thread].inPtr, "plan.html") == 0
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
						parserGET(thread, sock);    // выполнение запроса
#ifdef LOG
						journal.jprintf("$RETURN: %s\n",Socket[thread].outBuf);
#endif
						if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), strlen(Socket[thread].outBuf)) == 0) {
							uint8_t ip[4];
							W5100.readSnDIPR(sock, ip);
							journal.jprintf("$Error send AJAX(%d.%d.%d.%d): %s\n", ip[0], ip[1], ip[2], ip[3], (char*) Socket[thread].inBuf);
						}
						break;
					}
					case HTTP_POST:    // загрузка настроек
					{
                        uint8_t ret= parserPOST(thread, len);         // разобрать и получить тип ответа
                        strcpy(Socket[thread].outBuf, HEADER_ANSWER);
						strcat(Socket[thread].outBuf, postRet[ret]);   // вернуть текстовый ответ, который надо вывести
               			if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), strlen(Socket[thread].outBuf)) == 0) {
							uint8_t ip[4];
							W5100.readSnDIPR(sock, ip);
							journal.jprintf("$Error send POST(%d.%d.%d.%d): %s\n", ip[0], ip[1], ip[2], ip[3], (char*) Socket[thread].inBuf);
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
xUNAUTHORIZED:			journal.jprintf("$UNAUTHORIZED\n");
						sendConstRTOS(thread, pageUnauthorized);
						break;
					}
					case BAD_LOGIN_PASS: {
						journal.jprintf("$Wrong login or password\n");
						sendConstRTOS(thread, pageUnauthorized);
						break;
					}

					default:
						journal.jprintf("$Unknow  %s\n", (char*) Socket[thread].inBuf);
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
#ifdef FAST_LIB  // Переделка
	}  // for (int sock = 0; sock < W5200_SOCK_SYS; sock++)
#endif
	SemaphoreGive (xWebThreadSemaphore);              // Семафор отдать
}

//  Чтение файла с SD или его генерация
void readFileSD(char *filename, uint8_t thread)
{
	int n;
	SdFile webFile;
	//  journal.jprintf("$Thread: %d socket: %d read file: %s\n",thread,Socket[thread].sock,filename);
	char *str;
	STORE_DEBUG_INFO(11);

	// В начале обрабатываем генерируемые файлы (для выгрузки из контроллера)
	if(strcmp(filename, "state.txt") == 0) { get_txtState(thread, true); return; }
	if(strncmp(filename, "settings", 8) == 0) {
		filename += 8;
		if(strcmp(filename, ".txt") == 0) {	get_txtSettings(thread); return; }
		else if(strcmp(filename, ".bin") == 0) { get_binSettings(thread); return; }
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
			journal.jprintf("SD card test: %s - ", filename);
			sendConstRTOS(thread, HEADER_FILE_WEB);
			SPI_switchSD();
			if(webFile.open(filename, O_READ)) {
				uint32_t startTick = millis();
				uint32_t size = 0;
				for(;;) {
					int n = webFile.read(Socket[thread].outBuf, sizeof(Socket[thread].outBuf));
					if(n < 0) journal.jprintf("Read SD error (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
					if(n <= 0) break;
					size += n;
					if(millis() - startTick > (3*W5200_TIME_WAIT/portTICK_PERIOD_MS) - 1000) break; // на секунду меньше, чем блок семафора
					WDT_Restart(WDT);
				}
				startTick = millis() - startTick;
				journal.jprintf("read %d bytes, %d b/sec\n", size, size * 1000 / startTick);
				webFile.close();
				/*/ check write!
				if(!webFile.open(filename, O_RDWR)) journal.jprintf("Error open for writing!\n");
				else {
					n = webFile.write("Test write!");
					journal.jprintf("Wrote %d byte\n", n);
					if(!webFile.sync()) journal.jprintf("Sync failed (%d,%d)\n", card.cardErrorCode(), card.cardErrorData());
					webFile.close();
				}
				//*/
			} else {
				journal.jprintf("not found (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
			}
		}
		return;
	}
#ifdef I2C_EEPROM_64KB
//	if (strcmp(filename,"statistic.csv")==0) {get_csvStatistic(thread); return;}
#endif

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
						journal.jprintf("Reinit SD card failed!\n");
						//MC.set_fSD(false);
					}
				}
				sendConstRTOS(thread, HEADER_FILE_NOT_FOUND);
				uint8_t ip[4];
				W5100.readSnDIPR(Socket[thread].sock, ip);
				journal.jprintf((char*) "WEB: %d.%d.%d.%d - File not found: %s\n", ip[0], ip[1], ip[2], ip[3], filename);
				return;
			} // файл не найден
xFileFound:
			// Файл открыт читаем данные и кидаем в сеть
	#ifdef LOG
			journal.jprintf("$Thread: %d socket: %d read file: %s\n",thread,Socket[thread].sock,filename);
	#endif
			if(strstr(filename, ".css") != NULL) sendConstRTOS(thread, HEADER_FILE_CSS); // разные заголовки
			else sendConstRTOS(thread, HEADER_FILE_WEB);
			SPI_switchSD();
			while((n = webFile.read(Socket[thread].outBuf, sizeof(Socket[thread].outBuf))) > 0) {
				if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == 0) break;
				SPI_switchSD();
			} // while
			if(n < 0) journal.jprintf("Read SD error (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
			SPI_switchSD();
			webFile.close();
		}
		break;
	case pFLASH_WEB:
		{
			STORE_DEBUG_INFO(16);
			SerialFlashFile ff = SerialFlash.open(filename);
			if(ff) {
	#ifdef LOG
				journal.jprintf("$Thread: %d socket: %d read file: %s\n",thread,Socket[thread].sock,filename);
	#endif
				if(strstr(filename, ".css") != NULL) sendConstRTOS(thread, HEADER_FILE_CSS); // разные заголовки
				else sendConstRTOS(thread, HEADER_FILE_WEB);
				//	SPI_switchSD();
				while((n = ff.read(Socket[thread].outBuf, sizeof(Socket[thread].outBuf))) > 0) {
					//SPI_switchW5200();
					if(sendBufferRTOS(thread, (byte*) (Socket[thread].outBuf), n) == 0) break;
					//	SPI_switchSD();
				} // while
				//if(n < 0) journal.jprintf("Read SD error (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
				//SPI_switchSD();
				//webFile.close();
			} else {
				sendConstRTOS(thread, HEADER_FILE_NOT_FOUND);
				uint8_t ip[4];
				W5100.readSnDIPR(Socket[thread].sock, ip);
				journal.jprintf((char*) "WEB GET(%d.%d.%d.%d) - Not found: %s\n", ip[0], ip[1], ip[2], ip[3], filename);
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
#ifdef SENSOR_IP
void parserGET(uint8_t thread, int8_t sock)
{
#else
void parserGET(uint8_t thread, int8_t )
{
#endif
	char *str,*x,*y,*z;
	float pm=0;
	int16_t i;
	// переменные для удаленных датчиков
#ifdef SENSOR_IP                           // Получение данных удаленного датчика
	char *ptr;
	int16_t a,b,c,d;
#endif
	int32_t e;

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
			journal.jprintf("$ERROR - Response buffer overflowed!\n");
			journal.jprintf("%s\n",strReturn);
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
//			strcat(strReturn,"|SC>");
//			uint32_t t = MC.get_command_completed();
//			if(t) TimeIntervalToStr(rtcSAM3X8.unixtime() - t, strReturn, 1);
//			else strcat(strReturn,"-");
			strcat(strReturn,"|SI>");
			_itoa(100-MC.CPU_IDLE,strReturn);
			strcat(strReturn,"%|SF>");
			//_itoa(freeRam()+MC.startRAM,strReturn);                          strcat(strReturn,"b|");
			ADD_WEBDELIM(strReturn);
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
			DecodeTimeDate(MC.get_startDT(),strReturn);
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
			_itoa(100-MC.CPU_IDLE,strReturn);
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
			if(strcmp(str,"Press")==0)     // Функция get_listPress
			{
				for(i=0;i<ANUMBER;i++) if (MC.sADC[i].get_present()){strcat(strReturn,MC.sADC[i].get_name());strcat(strReturn,";");}
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
				for(e = rtcSAM3X8.get_years(); e > 2000; e--) {
					x += m_strlen(x);
					m_snprintf(x, 8 + 4 + sizeof(stats_csv_file_ext), "%s%04d%s", str, e, stats_file_ext);
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
			//strcat(strReturn,"Сброс Wiznet w5XXX и применение сетевых настроек, подождите 5 сек . . .");
			//ADD_WEBDELIM(strReturn);
			initW5200(true);                                  // Инициализация сети с выводом инфы в консоль
			for(uint8_t i = 0; i < W5200_THREAD; i++) SETBIT1(Socket[i].flags,fABORT_SOCK);                                 // Признак инициализации сокета, надо прерывать передачу в сервере
			MC.num_resW5200++;                                                       // Добавить счетчик инициализаций
			continue;
		}
		if (strcmp(str,"get_MODE")==0)  // Функция get_MODE в каком состояниии находится сейчас насос
		{
			MC.StateToStr(strReturn);
			ADD_WEBDELIM(strReturn) ;    continue;
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
			case TEST:      strcat(strReturn,noteRemarkTest[2]);     break; //  Включаем все кроме компрессора
			case HARD_TEST: strcat(strReturn,noteRemarkTest[3]);     break; //  Все включаем и компрессор тоже
			}
			ADD_WEBDELIM(strReturn) ;    continue;
		}
		STORE_DEBUG_INFO(21);

		if (strncmp(str, "set_SAVE", 8) == 0)  // Функция set_SAVE -
		{
			str += 8;
			if(strcmp(str, "_STATS") == 0) { // Сохранить счетчики и статистику
				xSaveStats:		if((i = MC.save_motoHour()) == OK)
					if((i = Stats.SaveStats(1)) == OK)
						i = Stats.SaveHistory(1);
				_itoa(i, strReturn);
			} else if(strcmp(str, "_UPD") == 0) { // Подготовка к обновлению
				//if(MC.is_compressor_on()) _itoa(-1, strReturn);
				//else
					goto xSaveStats;
				//}
			} else {
				e = MC.save();   // записать настройки в еепром, а потом будем их писать и получить размер записываемых данных
				_itoa(e, strReturn); // сохранение настроек ВСЕХ!
				MC.save_motoHour();
			}
			ADD_WEBDELIM(strReturn);
			continue;
		}
		
		STORE_DEBUG_INFO(22);

		if (strcmp(str,"get_errcode")==0)  // Функция get_errcode
		{
			_itoa(MC.get_errcode(),strReturn); ADD_WEBDELIM(strReturn) ;    continue;
		}
		if (strcmp(str,"get_error")==0)  // Функция get_error
		{
			strcat(strReturn,MC.get_lastErr()); ADD_WEBDELIM(strReturn) ;    continue;
		}
		if (strcmp(str,"get_tempSAM3x")==0)  // Функция get_tempSAM3x  - получение температуры чипа sam3x
		{
			_ftoa(strReturn,temp_DUE(),2); ADD_WEBDELIM(strReturn); continue;
		}
		if (strcmp(str,"get_tempDS3231")==0)  // Функция get_tempDS3231  - получение температуры DS3231
		{
			_ftoa(strReturn,getTemp_RtcI2C(),2); ADD_WEBDELIM(strReturn); continue;
		}
		if (strcmp(str,"get_Power220") == 0)
		{
			_ftoa(strReturn, (float)MC.dPWM.get_Power()/1000.0,3); ADD_WEBDELIM(strReturn); continue;
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
		if (strcmp(str,"get_numberIP") == 0)  // Удаленные датчики - получить число датчиков
		{
#ifdef SENSOR_IP
			_itoa(IPNUMBER,strReturn);ADD_WEBDELIM(strReturn); continue;
#else
			strcat(strReturn,"0" WEBDELIM);continue;
#endif
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
			} else if (strcmp(str,"STAT")==0)   // RESET_STAT, Команда очистки статистики (в зависимости от типа)
			{
#ifndef I2C_EEPROM_64KB     // Статистика в памяти
				strcat(strReturn,"Статистика не поддерживается в конфигурации . . .");
				journal.jprintf("No support statistics (low I2C) . . .\n");
#else                      // Статистика в ЕЕПРОМ
				//strcat(strReturn,"Форматирование I2C статистики, ожидайте 10 сек . . .");
				//MC.sendCommand(pSFORMAT);
#endif
			} else if (strcmp(str,"JOURNAL")==0)   // RESET_JOURNAL,  Команда очистки журнала (в зависимости от типа)
			{
#ifndef I2C_EEPROM_64KB     // журнал в памяти
				strcat(strReturn,"Сброс системного журнала в RAM . . .");
				journal.Clear();       // Послать команду на очистку журнала в памяти
				journal.jprintf("Reset system RAM journal . . .\n");
#else                      // Журнал в ЕЕПРОМ
				journal.Format(strReturn);
				//MC.sendCommand(pJFORMAT);        // Послать команду форматирование журнала
				strcpy(strReturn, HEADER_ANSWER);   // Начало ответа
				strcat(strReturn,"OK");
#endif
			} else if (strcmp(str,"DUE")==0)   // RESET_DUE, Команда сброса контроллера
			{
				//strcat(strReturn,"Сброс контроллера, подождите 10 секунд . . .");
				journal.jprintf("$SOFTWARE RESET control . . .\n\n");
				MC.save_motoHour();
				Stats.SaveStats(0);
				Stats.SaveHistory(0);
				_delay(500);            // задержка что бы вывести сообщение в консоль
				Software_Reset() ;      // Сброс
			} else if (strcmp(str,"COUNT")==0) // Команда RESET_COUNT
			{
				journal.jprintf("$RESET counter moto hour . . .\n");
				strcat(strReturn,"Сброс счетчика моточасов за сезон");
				MC.resetCount(false);
			} else if (strcmp(str,"ALL_COUNT")==0) // Команда RESET_ALL_COUNT
			{
				journal.jprintf("$RESET All counter moto hour . . .\n");
				strcat(strReturn,"Сброс ВСЕХ счетчика моточасов");
				MC.resetCount(true);  // Полный сброс
			} else if (strcmp(str,"SETTINGS")==0) // RESET_SETTINGS, Команда сброса настроек
			{
					journal.jprintf("$RESET All settings . . .\n");
					uint16_t d = 0;
					writeEEPROM_I2C(I2C_SETTING_EEPROM, (byte*)&d, sizeof(d));
					//MC.sendCommand(pRESET);        // Послать команду на сброс
					strcat(strReturn, "OK");
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
			strcat(strReturn,"MODBUS_PORT_NUM|Используемый порт для обмена по Modbus RTU|Serial");
			if(&MODBUS_PORT_NUM==&Serial1) strcat(strReturn,cOne);
			else if(&MODBUS_PORT_NUM==&Serial2) strcat(strReturn,"2");
			else if(&MODBUS_PORT_NUM==&Serial3) strcat(strReturn,"3");
			else strcat(strReturn,"?");
			strcat(strReturn,";");
			strcat(strReturn,"MODBUS_PORT_SPEED|Скорость обмена (бод)|");_itoa(MODBUS_PORT_SPEED,strReturn);strcat(strReturn,";");
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
			strcat(strReturn,"TIME_WEB_SERVER|Период опроса web сервера "); strcat(strReturn,nameWiznet);strcat(strReturn," (мсек)|");_itoa(TIME_WEB_SERVER,strReturn);strcat(strReturn,";");
			strcat(strReturn,"TIME_I2C_UPDATE |Период синхронизации внутренних часов с I2C часами (мсек)|");_itoa(TIME_I2C_UPDATE,strReturn);strcat(strReturn,";");
			// i2c
			strcat(strReturn,"I2C_COUNT_EEPROM|Адрес внутри чипа I2C с которого пишется счетчики |"); strcat(strReturn,uint16ToHex(I2C_COUNT_EEPROM)); strcat(strReturn,";");
			strcat(strReturn,"I2C_SETTING_EEPROM|Адрес внутри чипа I2C с которого пишутся настройки |"); strcat(strReturn,uint16ToHex(I2C_SETTING_EEPROM)); strcat(strReturn,";");
			// Датчики
			strcat(strReturn,"P_NUMSAMLES|Число значений для усреднения показаний давления|");_itoa(P_NUMSAMLES,strReturn);strcat(strReturn,";");
			strcat(strReturn,"T_NUMSAMLES|Число значений для усреднения показаний температуры|");_itoa(T_NUMSAMLES,strReturn);strcat(strReturn,";");
			strcat(strReturn,"GAP_TEMP_VAL|Допустимая разница показаний между двумя считываниями (°C)|");_ftoa(strReturn,(float)GAP_TEMP_VAL/100.0,2);strcat(strReturn,";");
			strcat(strReturn,"MAX_TEMP_ERR|Максимальная систематическая ошибка датчика температуры (°C)|");_ftoa(strReturn,(float)MAX_TEMP_ERR/100.0,2);strcat(strReturn,";");
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
			strcat(strReturn,"I2C_EEPROM_64KB|Место хранения системного журнала|");
#ifdef I2C_EEPROM_64KB
			strcat(strReturn,"I2C flash memory;");
#else
			strcat(strReturn,"RAM memory;");
#endif
			strcat(strReturn,"JOURNAL_LEN|Размер кольцевого буфера системного журнала (байт)|");_itoa(JOURNAL_LEN,strReturn);//strcat(strReturn,";");
			ADD_WEBDELIM(strReturn) ;
			continue;
		} // end CONST1

		if(strncmp(str, "get_sys", 7) == 0)
		{
			str += 7;
			STORE_DEBUG_INFO(26);
			if(strcmp(str, "Info") == 0) { // "get_sysInfo" - Функция вывода системной информации для разработчика
				strcat(strReturn,"Источник загрузкки web интерфейса |");
				switch (MC.get_SourceWeb())
				{
				case pMIN_WEB:   strcat(strReturn,"internal;"); break;
				case pSD_WEB:    strcat(strReturn,"SD card;"); break;
				case pFLASH_WEB: strcat(strReturn,"SPI Flash;"); break;
				default:         strcat(strReturn,"unknown;"); break;
				}

				strcat(strReturn,"Входное напряжение питания контроллера (В): |");
				m_snprintf(strReturn+strlen(strReturn), 256, "если ниже %.1fV - сброс;", (float)((SUPC->SUPC_SMMR & SUPC_SMMR_SMTH_Msk) >> SUPC_SMMR_SMTH_Pos) / 10 + 1.9);
				m_snprintf(strReturn += strlen(strReturn), 256, "Режим safeNetwork (%sадрес:%d.%d.%d.%d шлюз:%d.%d.%d.%d, не спрашивать пароль)|%s;", defaultDHCP ?"DHCP, ":"",defaultIP[0],defaultIP[1],defaultIP[2],defaultIP[3],defaultGateway[0],defaultGateway[1],defaultGateway[2],defaultGateway[3],MC.safeNetwork ?cYes:cNo);
				//strcat(strReturn,"Уникальный ID чипа SAM3X8E|");
				//getIDchip(strReturn); strcat(strReturn,";");
				//strcat(strReturn,"Значение регистра VERSIONR сетевого чипа WizNet (51-w5100, 3-w5200, 4-w5500)|");_itoa(W5200VERSIONR(),strReturn);strcat(strReturn,";");

				m_snprintf(strReturn+strlen(strReturn), 256, "Состояние FreeRTOS при старте (task+err_code) <sup>2</sup>|0x%04X;", lastErrorFreeRtosCode);

				startSupcStatusReg |= SUPC->SUPC_SR;                                  // Копим изменения
				m_snprintf(strReturn += strlen(strReturn), 256, "Регистры контроллера питания (SUPC) SAM3X8E [SUPC_SMMR SUPC_MR SUPC_SR]|0x%08X %08X %08X", SUPC->SUPC_SMMR, SUPC->SUPC_MR, startSupcStatusReg);  // Регистры состояния контроллера питания
				if((startSupcStatusReg|SUPC_SR_SMS)==SUPC_SR_SMS_PRESENT) strcat(strReturn," bad VDDIN!");
				strcat(strReturn,";");

				STORE_DEBUG_INFO(47);
				strcat(strReturn,"<b> Времена</b>|;");
				strcat(strReturn,"Текущее время|"); DecodeTimeDate(rtcSAM3X8.unixtime(),strReturn); strcat(strReturn,";");
				strcat(strReturn,"Время сохранения текущих настроек |");DecodeTimeDate(MC.get_saveTime(),strReturn);strcat(strReturn,";");

				strcat(strReturn,"<b> Счетчики ошибок</b>|;");
				strcat(strReturn,"Счетчик \"Потеря связи с "); strcat(strReturn,nameWiznet);strcat(strReturn,"\", повторная инициализация  <sup>3</sup>|");_itoa(MC.num_resW5200,strReturn);strcat(strReturn,";");
				strcat(strReturn,"Счетчик числа сбросов мютекса захвата шины SPI|");_itoa(MC.num_resMutexSPI,strReturn);strcat(strReturn,";");
				strcat(strReturn,"Счетчик числа сбросов мютекса захвата шины I2C|");_itoa(MC.num_resMutexI2C,strReturn);strcat(strReturn,";");
	#ifdef MQTT
				strcat(strReturn,"Счетчик числа повторных соединений MQTT клиента|");_itoa(MC.num_resMQTT,strReturn);strcat(strReturn,";");
	#endif
				strcat(strReturn,"Счетчик неудачных ping|");_itoa(MC.num_resPing,strReturn);strcat(strReturn,";");
				strcat(strReturn,"Счетчик числа ошибок чтения датчиков температуры (DS18x20)|");_itoa(MC.get_errorReadDS18B20(),strReturn);strcat(strReturn,";");

				strcat(strReturn,"<b> Глобальные счетчики (Всего за весь период)</b>|;");
				//strcat(strReturn,"Потребленная энергия (кВт*ч)|");_ftoa(strReturn, (float)MC.get_motoHourE1() / 1000.0, 2);strcat(strReturn,";");

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
		// Список аналоговых датчиков выводятся только присутсвующие датчики список вида name:0;
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
#ifdef CORRECT_POWER220
			} else if(strcmp(str,"PwrC")==0) {    // Функция get_tblPwrC
				for(i = 0; i < (int8_t)(sizeof(correct_power220)/sizeof(correct_power220[0])); i++) {
					m_snprintf(strReturn + m_strlen(strReturn), 64, "%s;%d;", MC.dRelay[correct_power220[i].num].get_name(), correct_power220[i].value);
				}
#endif
			} else goto x_FunctionNotFound;
			ADD_WEBDELIM(strReturn);
			continue;
		}
#ifdef RADIO_SENSORS
		if(strcmp(str, "set_radio_cmd") == 0) {
			if((x = strchr(str, '='))) {
				x++;
				radio_sensor_send(x);
			}
			ADD_WEBDELIM(strReturn);	continue;
		}
#endif
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
			if (strcmp(str,"set_testMode")==0)  // Функция set_testMode  - Установить режим работы бойлера
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
				ADD_WEBDELIM(strReturn) ; continue;
			}

			//11.  Графики смещение  используется в одной функции get_Chart -------------------------------------------
			if (strcmp(str,"get_Chart")==0)           // Функция get_Chart - получить график
			{
				MC.get_Chart(x,strReturn);
				ADD_WEBDELIM(strReturn); continue;
			}

			// 13 Опции
			if (strcmp(str,"get_Opt")==0)           // Функция get_option - получить значение параметра отопления 
			{
				MC.get_option(x,strReturn); ADD_WEBDELIM(strReturn) ; continue;
			} else if (strcmp(str,"set_Opt")==0)           // Функция set_option - установить значение паремтра  опций
			{
				if (pm!=ATOF_ERROR) {   // нет ошибки преобразования
					if (MC.set_option(x,pm)) MC.get_option(x,strReturn);  // преобразование удачно,
					else strcat(strReturn,"E17") ; // выход за диапазон значений
				} else strcat(strReturn,"E11");   // ошибка преобразования во флоат
				ADD_WEBDELIM(strReturn); continue;
			}
			STORE_DEBUG_INFO(37);


			// str - полное имя запроса до (), x - содержит строку что между (), z - после =
			// код обработки установки значений модбас
			// get_modbus_val(N:D:X), set_modbus_val(N:D:X=YYY)
			// N - номер устройства, D - тип данных, X - адрес, Y - новое значение
			if(strncmp(str+1, "et_modbus_", 10) == 0) {
				STORE_DEBUG_INFO(38);
				if((y = strchr(x, ':'))) {
					*y++ = '\0';
					uint8_t id = atoi(x);
					uint16_t par = atoi(y + 2); // Передается нумерация регистров с 1, а в modbus с 0
					if(par--) {
						i = OK;
						if(strncmp(str, "set", 3) == 0) {
							if(*y == 'w') i = Modbus.writeHoldingRegisters16(id, par, strtol(z, NULL, 0));
							else if(*y == 'l') i = Modbus.writeHoldingRegisters32(id, par, strtol(z, NULL, 0));
							else if(*y == 'f') i = Modbus.writeHoldingRegistersFloat(id, par, strtol(z, NULL, 0));
							else if(*y == 'c') i = Modbus.writeSingleCoil(id, par, atoi(z));
							else goto x_FunctionNotFound;
							_delay(MODBUS_TIME_TRANSMISION * 10); // Задержка перед чтением
						} else if(strncmp(str, "get", 3) == 0) {
						} else goto x_FunctionNotFound;
						if(i == OK) {
							if(*y == 'w') {
								if((i = Modbus.readHoldingRegisters16(id, par, &par)) == OK) _itoa(par, strReturn);
							} else if(*y == 'l') {
								if((i = Modbus.readHoldingRegisters32(id, par, (uint32_t *)&e)) == OK) _itoa(e, strReturn);
							} else if(*y == 'i') {
								if((i = Modbus.readInputRegistersFloat(id, par, &pm)) == OK) _ftoa(strReturn, pm, 2);
							} else if(*y == 'f') {
								if((i = Modbus.readHoldingRegistersFloat(id, par, &pm)) == OK) _ftoa(strReturn, pm, 2);
							} else if(*y == 'c') {
								if((i = Modbus.readCoil(id, par, (boolean *)&par)) == OK) _itoa(par, strReturn);
							} else goto x_FunctionNotFound;
						}
						if(i != OK) {
							strcat(strReturn, "E"); _itoa(i, strReturn);
						}
						ADD_WEBDELIM(strReturn);
						continue;
					}
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
					for(i=0; i<TNUMBER; i++) if(strcmp(x,MC.sTemp[i].get_name())==0) {p=i; break;} // Поиск среди имен  смещение 0
					if(p >= TNUMBER)  {strcat(strReturn,"E03");ADD_WEBDELIM(strReturn);  continue; }  // Не соответсвие имени функции и параметра
					else  // параметр верный
					{
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strcmp(str,"Temp")==0)              // Функция get_Temp
							{
								if(MC.sTemp[p].get_present() && MC.sTemp[p].get_Temp() != STARTTEMP)  // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sTemp[p].get_Temp()/100,2);
								else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if (strncmp(str,"raw",3)==0)           // Функция get_RawTemp
							{ 	if(MC.sTemp[p].get_present() && MC.sTemp[p].get_Temp() != STARTTEMP)  // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sTemp[p].get_rawTemp()/100,2);
								else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "full", 4) == 0)         // Функция get_fullTemp
							{
								if(MC.sTemp[p].get_present() && MC.sTemp[p].get_Temp() != STARTTEMP) // Если датчик есть в конфигурации то выводим значение
								{
									if(MC.sTemp[p].get_lastTemp() == STARTTEMP) strcat(strReturn, "-.-");
									else _ftoa(strReturn, (float) MC.sTemp[p].get_Temp() / 100, MC.sTemp[p].get_fRadio() ? 1 : 2);
								} else strcat(strReturn, "-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn);
								continue;
							}

							if(strncmp(str, "min", 3)==0)           // Функция get_minTemp
							{
								if (MC.sTemp[p].get_present()) // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sTemp[p].get_minTemp()/100,1);
								else strcat(strReturn,"-");              // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "max", 3)==0)           // Функция get_maxTemp
							{
								if (MC.sTemp[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sTemp[p].get_maxTemp()/100,1);
								else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "err", 3)==0)           // Функция get_errTemp
							{ _ftoa(strReturn,(float)MC.sTemp[p].get_errTemp()/100,2); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "aT", 2) == 0)           // Функция get_aTemp (address)
							{
x_get_aTemp:
								if(!MC.sTemp[p].get_fAddress()) strcat(strReturn, "не привязан");
								else if(MC.sTemp[p].get_fRadio()) _itoa(*(uint32_t*)(MC.sTemp[p].get_address() + 1), strReturn);
								else strcat(strReturn, addressToHex(MC.sTemp[p].get_address()));
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "test", 4)==0)           // Функция get_testTemp
							{ _ftoa(strReturn,(float)MC.sTemp[p].get_testTemp()/100,1); ADD_WEBDELIM(strReturn); continue; }

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
										if(str[9] == '2') m_snprintf(strReturn + strlen(strReturn), 20, ", %.1fV", (float)radio_received[i].battery / 10.0);
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
								_itoa(MC.sTemp[p].get_setup_flag(str[9] - '0' - 1 + fTEMP_ignory_errors), strReturn);
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
									{ _ftoa(strReturn,(float)MC.sTemp[p].get_testTemp()/100,1); ADD_WEBDELIM(strReturn);  continue;  }
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}       // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
							if(strncmp(str, "err", 3)==0)           // Функция set_errTemp
							{ 	if (MC.sTemp[p].set_errTemp(rd(pm, 100))==OK)    // Установить значение в сотых градуса
									{ _ftoa(strReturn,(float)MC.sTemp[p].get_errTemp()/100,2); ADD_WEBDELIM(strReturn); continue; }
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}      // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}

							if(strncmp(str, "fTemp", 5) == 0) {   // set_fTempX(N=V): X - номер флага fTEMP_* (1..), N - имя датчика (flag)
								i = str[9] - '0' - 1 + fTEMP_ignory_errors;
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
								if (MC.dRelay[p].get_Relay()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
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
								if(MC.dRelay[p].set_Relay(pm == 0 ? fR_StatusAllOff : fR_StatusManual) == OK) // Установить значение
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
				if(strstr(str,"Press"))          // Проверка для запросов содержащих Press
				{
					STORE_DEBUG_INFO(41);
					for(i=0;i<ANUMBER;i++) if(strcmp(x,MC.sADC[i].get_name())==0) {p=i; break;} // Поиск среди имен
					if ((p<0)||(p>=ANUMBER))  {strcat(strReturn,"E03");ADD_WEBDELIM(strReturn);  continue; }  // Не соответсвие имени функции и параметра
					else  // параметр верный
					{
						if(strncmp(str,"get_", 4)==0) {              // Функция get_
							str += 4;
							if(strcmp(str,"Press")==0)           // Функция get_Press
							{
								if(MC.sADC[p].get_present())         // Если датчик есть в конфигурации то выводим значение
								{
									_ftoa(strReturn,(float)MC.sADC[p].get_Press() / 100, 2);
	#ifdef EEV_DEF
									if(p < 2) {
										m_snprintf(strReturn + m_strlen(strReturn), 20, " [%.2f°]", (float)PressToTemp(p) / 100);
									}
	#endif
								} else strcat(strReturn,"-");             // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn); continue;
							}

							if(strncmp(str, "adc", 3)==0)           // Функция get_adcPress
							{ _itoa(MC.sADC[p].get_lastADC(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "min", 3)==0)           // Функция get_minPress
							{ if (MC.sADC[p].get_present())          // Если датчик есть в конфигурации то выводим значение
								x_get_minPress: _ftoa(strReturn,(float)MC.sADC[p].get_minPress()/100.0,2);
							else strcat(strReturn,"-");              // Датчика нет ставим прочерк
							ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "max", 3)==0)           // Функция get_maxPress
							{ if (MC.sADC[p].get_present())           // Если датчик есть в конфигурации то выводим значение
								x_get_maxPress: _ftoa(strReturn,(float)MC.sADC[p].get_maxPress()/100.0,2);
							else strcat(strReturn,"-");               // Датчика нет ставим прочерк
							ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "zero", 4)==0)           // Функция get_zeroPress
							{ _itoa(MC.sADC[p].get_zeroPress(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "trans", 5)==0)           // Функция get_transPress
							{ _ftoa(strReturn,(float)MC.sADC[p].get_transADC() / 1000, 3); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "pin", 3)==0)           // Функция get_pinPress
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

							if(strncmp(str, "test", 4)==0)           // Функция get_testPress
							{ _ftoa(strReturn,(float)MC.sADC[p].get_testPress()/100.0,2); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "eP", 2)==0)           // Функция get_ePress (errorcode)
							{ _itoa(MC.sADC[p].get_lastErr(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "isP", 3)==0)           // Функция get_isPress
							{
								if (MC.sADC[p].get_present()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "nP", 2)==0)           // Функция get_nPress (note)
							{ strcat(strReturn,MC.sADC[p].get_note()); ADD_WEBDELIM(strReturn); continue; }

						// ---- SET ----------------- Для аналоговых  датчиков - запросы на УСТАНОВКУ парметров
						} else if(strncmp(str,"set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "test", 4) == 0)           // Функция set_testPress
							{
								if(MC.sADC[p].set_testPress(rd(pm, 100)) == OK)    // Установить значение
								{
									_ftoa(strReturn, (float) MC.sADC[p].get_testPress() / 100.0, 2);
									ADD_WEBDELIM(strReturn); continue;
								} else {// выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
									strcat(strReturn, "E05" WEBDELIM);	continue;
								}
							}
							if(strncmp(str, "min", 3) == 0) {  // set_minPress
								if(MC.sADC[p].set_minPress(rd(pm, 100)) == OK) goto x_get_minPress;
								else { strcat(strReturn, "E05" WEBDELIM);  continue; }         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
							if(strncmp(str, "max", 3) == 0) { // set_maxPress
								if(MC.sADC[p].set_maxPress(rd(pm, 100)) == OK) goto x_get_maxPress;
								else {  strcat(strReturn, "E05" WEBDELIM); continue;  }         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}

							if(strncmp(str, "trans", 5)==0)           // Функция set_transPress float
							{ if (MC.sADC[p].set_transADC(pm)==OK)    // Установить значение
								{_ftoa(strReturn,(float)MC.sADC[p].get_transADC() / 1000, 3); ADD_WEBDELIM(strReturn); continue;}
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}

							if(strncmp(str, "zero", 4) == 0)           // Функция set_zeroPress
							{
								if(MC.sADC[p].set_zeroPress((int16_t) pm) == OK)    // Установить значение
								{
									_itoa(MC.sADC[p].get_zeroPress(), strReturn);
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
				} //if ((strstr(str,"Press")>0)

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
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sFrequency[p].get_Value()/1000.0,3);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "frF", 3)==0)           // Функция get_frFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sFrequency[p].get_Frequency()/1000.0,3);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "min", 3)==0)           // Функция get_minFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sFrequency[p].get_minValue()/1000.0,1);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "kfF", 3)==0)           // Функция get_kfFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sFrequency[p].get_kfValue()/100,2);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "test", 4)==0)           // Функция get_testFlow
							{
								if (MC.sFrequency[p].get_present())          // Если датчик есть в конфигурации то выводим значение
									_ftoa(strReturn,(float)MC.sFrequency[p].get_testValue()/1000.0,3);
								else strcat(strReturn,"-");               // Датчика нет ставим прочерк
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "pin", 3)==0)              // Функция get_pinFlow
							{ strcat(strReturn,"D"); _itoa(MC.sFrequency[p].get_pinF(),strReturn);
								ADD_WEBDELIM(strReturn); continue; }
							if(strncmp(str, "eF", 2)==0)           // Функция get_eFlow (errorcode)
							{ _itoa(MC.sFrequency[p].get_lastErr(),strReturn);
								ADD_WEBDELIM(strReturn); continue; }
							if(strncmp(str, "nF", 2)==0)               // Функция get_nFlow (note)
							{ strcat(strReturn,MC.sFrequency[p].get_note()); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "check", 5)==0) // get_checkFlow
							{ _itoa(MC.sFrequency[p].get_checkFlow(), strReturn); ADD_WEBDELIM(strReturn); continue; }

						// ---- SET ----------------- Для частотных  датчиков - запросы на УСТАНОВКУ парметров
						} else if(strncmp(str, "set_", 4)==0) {              // Функция set_
							str += 4;
							if(pm == ATOF_ERROR) {   // Ошибка преобразования для чисел - завершить запрос с ошибкой
								strcat(strReturn, "E04");
								ADD_WEBDELIM(strReturn);
								continue;
							}
							if(strncmp(str, "min", 3)==0) {           // Функция set_minFlow
								MC.sFrequency[p].set_minValue(pm);
								_ftoa(strReturn,(float)MC.sFrequency[p].get_minValue()/1000.0,1); ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "check", 5)==0) {           // Функция set_checkFlow
								MC.sFrequency[p].set_checkFlow(pm != 0);
								_itoa(MC.sFrequency[p].get_checkFlow(), strReturn); ADD_WEBDELIM(strReturn); continue;
							}
							if(strncmp(str, "test", 4)==0)           // Функция set_testFlow
							{ if (MC.sFrequency[p].set_testValue(rd(pm, 1000))==OK)    // Установить значение
								{_ftoa(strReturn,(float)MC.sFrequency[p].get_testValue()/1000.0,3); ADD_WEBDELIM(strReturn); continue; }
								else { strcat(strReturn,"E35" WEBDELIM); continue;}         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
							if(strncmp(str, "kfF", 3)==0)           // Функция set_kfFlow float
							{ MC.sFrequency[p].set_kfValue(rd(pm, 100));    // Установить значение
								_ftoa(strReturn,(float)MC.sFrequency[p].get_kfValue()/100,2); ADD_WEBDELIM(strReturn); continue;
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
								if (MC.sInput[p].get_alarmInput()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero);
								ADD_WEBDELIM(strReturn) ;    continue;
							}
							if(strncmp(str, "eI", 2)==0)           // Функция get_eInput (errorcode)
							{ _itoa(MC.sInput[p].get_lastErr(),strReturn); ADD_WEBDELIM(strReturn); continue; }

							if(strncmp(str, "pin", 3)==0)           // Функция get_pinInput
							{ strcat(strReturn,"D"); _itoa(MC.sInput[p].get_pinD(),strReturn);
								ADD_WEBDELIM(strReturn); continue; }
							if(strncmp(str, "type", 4)==0)           // Функция get_typeInput
							{
								if (MC.sInput[p].get_present()==true) {  // датчик есть в кнфигурации
									switch((int)MC.sInput[p].get_typeInput())
									{
										case pALARM: strcat(strReturn,"Alarm"); break;                // 0 Аварийный датчик, его срабатываение приводит к аварии и останове Тн
										case pSENSOR:strcat(strReturn,"Work");  break;                // 1 Обычный датчик, его значение используется в алгоритмах 
										case pPULSE: strcat(strReturn,"Pulse"); break;                // 2 Импульсный висит на прерывании и считает частоты - выходная величина ЧАСТОТА
										default:strcat(strReturn,"err_type"); break;                  // Ошибка??
									}
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
							if(strncmp(str, "test", 4)==0)           // Функция set_testInput
							{ if (MC.sInput[p].set_testInput((int16_t)pm)==OK)    // Установить значение
								{ if (MC.sInput[p].get_testInput()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero); ADD_WEBDELIM(strReturn); continue; }
								else { strcat(strReturn,"E05" WEBDELIM);  continue;}         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
							if(strncmp(str, "alarm", 5)==0)           // Функция set_alarmInput
							{ if (MC.sInput[p].set_alarmInput((int16_t)pm)==OK)    // Установить значение
								{ if (MC.sInput[p].get_alarmInput()==true)  strcat(strReturn,cOne); else  strcat(strReturn,cZero); ADD_WEBDELIM(strReturn); continue; }
								else { strcat(strReturn,"E05" WEBDELIM); continue;}         // выход за диапазон ПРЕДУПРЕЖДЕНИЕ значение не установлено
							}
						}
						strcat(strReturn,"E08"); // выход за диапазон, значение не установлено
						ADD_WEBDELIM(strReturn);
						continue;
					}  // else end
				} //if ((strstr(str,"Input")>0)

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
const char *header_Authorization_="Authorization: Basic ";
const char *header_POST_="Access-Control-Request-Method: POST";
// Выделение имени файла (или содержания запроса) и типа файла и типа запроса клиента
// thread - номер потока, возсращает тип запроса
uint16_t GetRequestedHttpResource(uint8_t thread)
{
	char *str_token, *pass;
	boolean user, admin;
	uint8_t i;
	uint16_t len;
	STORE_DEBUG_INFO(50);
	if((MC.get_fPass()) && (!MC.safeNetwork))  // идентификация если установлен флаг и перемычка не в нуле
	{
		if(!(pass = strstr((char*) Socket[thread].inBuf, header_Authorization_))) return UNAUTHORIZED; // строка авторизации не найдена
		else  // Строка авторизации найдена смотрим логин пароль
		{
			pass = pass + strlen(header_Authorization_);
			user = true;
			for(i = 0; i < MC.Security.hashUserLen; i++)
				if(pass[i] != MC.Security.hashUser[i]) {
					user = false;
					break;
				}
			if(user != true) // это не пользователь
			{
				admin = true;
				for(i = 0; i < MC.Security.hashAdminLen; i++)
					if(pass[i] != MC.Security.hashAdmin[i]) {
						admin = false;
						break;
					}
				if(admin != true) return BAD_LOGIN_PASS; // Не верный логин или пароль
			} //  if (user!=true)
			else SETBIT1(Socket[thread].flags, fUser); // зашел простой пользователь
		} // else
	}

	// Идентификация пройдена
	//if(strstr((char*)Socket[thread].inBuf,"Access-Control-Request-Method: POST")) {request_type = HTTP_POST_; return request_type; }  //обработка предваритаельного запроса перед получением файла
	char *tmpptr;
	str_token = strtok_r((char*) Socket[thread].inBuf, " ", &tmpptr);    // Обрезаем по пробелам
	if(strcmp(str_token, "GET") == 0)   // Ищем GET
	{
		str_token = strtok_r(NULL, " ", &tmpptr);                       // get the file name
		if(strcmp(str_token, "/") == 0)                   // Имени файла нет, берем файл по умолчанию
		{
			Socket[thread].inPtr = (char*) INDEX_FILE;      // Указатель на имя файла по умолчанию
			return HTTP_GET;
		} else if((len = strlen(str_token)) <= W5200_MAX_LEN - 100)   // Проверка на длину запроса или имени файла
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
#define emptyStr			WEB_HEADER_END  	   // пустая строка после которой начинаются данные
#define MAX_FILE_LEN		64  	              // максимальная длина имени файла
const char Title[]          = "Title: ";          // где лежит имя файла
const char Length[]         = "Content-Length: "; // где лежит длина файла
const char SETTINGS[]       = "*SETTINGS*";       // Идентификатор передачи настроек (лежит в Title:)
const char LOAD_FLASH_START[]= "*SPI_FLASH*";     // Идентификатор начала загрузки веб морды в SPI Flash (лежит в Title:)
const char LOAD_FLASH_END[]  = "*SPI_FLASH_END*"; // Идентификатор колнца загрузки веб морды в SPI Flash (лежит в Title:)
const char LOAD_SD_START[]   = "*SD_CARD*";       // Идентификатор начала загрузки веб морды в SD (лежит в Title:)
const char LOAD_SD_END[]     = "*SD_CARD_END*";   // Идентификатор колнца загрузки веб морды в SD (лежит в Title:)

uint16_t numFilesWeb = 0;                   // Число загруженных файлов

// Разбор и обработка POST запроса inPtr входная строка использует outBuf для хранения файла настроек!
// Сейчас реализована загрузка настроек и загрузка веб морды в спи диск
// Возврат тип ответа (потом берется из массива строк)
TYPE_RET_POST parserPOST(uint8_t thread, uint16_t size)
{
	static byte *ptr, *pStart;
	char *nameFile;      // указатель имя файла
	int32_t len, buf_len = 0, lenFile;

	//journal.printf(" POST =>"); journal.printf("%s\n", Socket[thread].inPtr); if(strlen(Socket[thread].inPtr) >= PRINTF_BUF) journal.printf("%s\n", Socket[thread].inPtr + PRINTF_BUF - 1);
	STORE_DEBUG_INFO(51);

	// Поиски во входном буфере: данных, имени файла и длинны файла
	ptr = (byte*) strstr(Socket[thread].inPtr, emptyStr) + sizeof(emptyStr) - 1;    // поиск начала даных

	if((nameFile = strstr(Socket[thread].inPtr, Title)) == NULL) { // Имя файла не найдено, запрос не верен, выходим
		journal.jprintf("Upload: Name not found!\n");
		return pLOAD_ERR;
	}
	nameFile += sizeof(Title) - 1;
	char *tmp = strchr(nameFile, '\r');
	if(tmp == NULL || tmp - nameFile >= MAX_FILE_LEN) {
		nameFile[MAX_FILE_LEN] = '\0';
		journal.jprintf("Upload: %s name length > %d bytes!\n", nameFile, MAX_FILE_LEN - 1);
		return pLOAD_ERR;
	}
	pStart = (byte*) strstr(Socket[thread].inPtr, Length);
	if(pStart == NULL) { // Размер файла не найден, запрос не верен, выходим
		journal.jprintf("Upload: %s - length not found!\n", nameFile);
		return pLOAD_ERR;
	}
	pStart += sizeof(Length) - 1;
	*tmp = '\0';
	urldecode(Socket[thread].outBuf, nameFile, 128);
	nameFile = Socket[thread].outBuf;
	tmp = strchr((char*) pStart, '\r');
	if(tmp) {
		*tmp = '\0';
		lenFile = atoi((char*) pStart);	// получить длину
	} else lenFile = 0;
	// все нашлось, можно обрабатывать
	buf_len = size - (ptr - (byte *) Socket[thread].inBuf);                  // длина (остаток) данных (файла) в буфере
	// В зависимости от имени файла (Title)
	if(strcmp(nameFile, SETTINGS) == 0) {  // Чтение настроек
		STORE_DEBUG_INFO(52);
		// Определение начала данных (поиск HEADER_BIN)
		if((strstr((char*) ptr, HEADER_BIN)) == NULL || lenFile == 0) {  // Заголовок не найден
			journal.jprintf("Upload: Wrong save format: %s!\n", nameFile);
			return pSETTINGS_ERR;
		}
		buf_len = size - (ptr - (byte *) Socket[thread].inBuf); // определяем размер данных в пакете
		memcpy(Socket[thread].outBuf, ptr, buf_len);         // копируем начало данных в буфер

		while(buf_len < lenFile)  // Чтение остальных данных по сети
		{
			_delay(10);
			len = Socket[thread].client.get_ReceivedSizeRX();                          // получить длину входного пакета
			if(len > W5200_MAX_LEN - 1) len = W5200_MAX_LEN - 1; // Ограничить размером в максимальный размер пакета w5200
			Socket[thread].client.read(Socket[thread].inBuf, len);                      // прочитать буфер
			if(buf_len + len >= (int32_t) sizeof(Socket[thread].outBuf)) return pSETTINGS_MEM; // проверить длину если не влезает то выходим
			memcpy(Socket[thread].outBuf + buf_len, Socket[thread].inBuf, len);           // Добавить пакет в буфер
			buf_len = buf_len + len;                                                     // определить размер данных
		}
		ptr = (byte*) Socket[thread].outBuf + sizeof(HEADER_BIN) - 1;                     // отрезать заголовок в данных
		journal.jprintf("Loading %s, length %d bytes:\n", SETTINGS, buf_len);
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
			journal.jprintf(pP_TIME, "Start upload, erase SPI disk ");
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
			journal.jprintf(pP_TIME, "Start upload to SD.\n");
			return pNULL;
		} else if(strcmp(nameFile, LOAD_FLASH_END) == 0 || strcmp(nameFile, LOAD_SD_END) == 0) {  // Окончание загрузки вебморды
			if(SemaphoreTake(xLoadingWebSemaphore, 0) == pdFALSE) { // Семафор не захвачен (был захвачен ранее) все ок
				journal.jprintf(pP_TIME, "Ok, %d files uploaded, free %.1f KB\n", numFilesWeb, fWebUploadingFilesTo == 1 ? (float)SerialFlash.free_size() / 1024 : (float)card.vol()->freeClusterCount() * card.vol()->blocksPerCluster() * 512 / 1024);
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
				if(fWebUploadingFilesTo == 1) {
					if(loadFileToSpi(nameFile, lenFile, thread, ptr, buf_len)) {
						numFilesWeb++;
						return pNULL;
					} else {
						SemaphoreGive (xLoadingWebSemaphore);
						return pLOAD_ERR;
					}
				} else if(fWebUploadingFilesTo == 2) { // Запись на SD,
					STORE_DEBUG_INFO(54);
					journal.jprintf("%s (%d) ", nameFile, lenFile);
					SPI_switchSD();
					if(wFile.opens(nameFile, O_CREAT | O_TRUNC | O_RDWR, &wfname)) {
						wFile.timestamp(T_CREATE | T_ACCESS | T_WRITE, rtcSAM3X8.get_years(), rtcSAM3X8.get_months(), rtcSAM3X8.get_days(), rtcSAM3X8.get_hours(), rtcSAM3X8.get_minutes(), rtcSAM3X8.get_seconds());
						int32_t wrote = wFile.write(ptr, buf_len);
						if(wrote != buf_len) {
							journal.jprintf("Error write file %s (%d,%d)!\n", nameFile, card.cardErrorCode(), card.cardErrorData());
						} else {
							uint16_t numPoint = 0;
							while((lenFile -= wrote) > 0)  // Чтение остальных пакетов из сети
							{
								_delay(2);                                                                 // время на приход данных
								SPI_switchW5200();
								int32_t len = Socket[thread].client.get_ReceivedSizeRX();                            // получить длину входного пакета
								if(len == 0) {
									if(Socket[thread].client.connected()) continue; else break;
								}
								Socket[thread].client.read(Socket[thread].inBuf, len);                      // прочитать буфер
								STORE_DEBUG_INFO(56);
								SPI_switchSD();
								wrote = wFile.write(Socket[thread].inBuf, len);                        // записать
								if(wrote != len) {
									journal.jprintf("Error write file %s (%d,%d)!\n", nameFile, card.cardErrorCode(), card.cardErrorData());
									break;
								}
								STORE_DEBUG_INFO(57);
								if(++numPoint >= 20) {// точка на 30 кб приема (20 пакетов по 1540)
									numPoint = 0;
									journal.jprintf(".");
								}
							}
						}
						STORE_DEBUG_INFO(58);
						SPI_switchSD();
						if(!wFile.close()) {
							journal.jprintf("Error close file (%d,%d)!\n", card.cardErrorCode(), card.cardErrorData());
						}
						if(lenFile == 0) journal.jprintf("Ok\n"); else journal.jprintf("Error length %d!\n", lenFile);
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
// Загрузка файла в память возвращает число записанных байт на диск 0 если ошибка
// Файл может лежать во множестве пакетов. Если в SPI Flash, то считается что spi диск отформатирован и ожидает запись файлов с "нуля"
// Входные параметры:
// nameFile - имя файла
// lenFile - общая длина файла
// thread - поток веб сервера,котрый обрабатывает post запрос
// ptr - указатель на начало данных (файла) в буфере Socket[thread].inPtr.
// sizeBuf - размер данных в буфере ptr (по сети осталось принять lenFile-sizeBuf)
uint32_t loadFileToSpi(char * nameFile, uint32_t lenFile, uint8_t thread, byte* ptr, uint16_t sizeBuf)
{
	uint16_t numPoint = 0;
	uint32_t loadLen; // Обработанная (загруженная) длина

	STORE_DEBUG_INFO(54);
	journal.jprintf("%s ", nameFile);
	loadLen = SerialFlash.free_size();
	if(lenFile > loadLen) {
		journal.jprintf("Not enough space, free: %d\n", loadLen);
		loadLen = 0;
	} else if(SerialFlash.create(nameFile, lenFile)) {
		loadLen = 0;
		SerialFlashFile ff = SerialFlash.open(nameFile);
		if(ff) {
			if(sizeBuf > 0) loadLen = ff.write(ptr, sizeBuf);  // первый пакет упаковали если он не нулевой
			while(loadLen < lenFile)  // Чтение остальных пакетов из сети
			{
				_delay(2);                                                                 // время на приход данных
				sizeBuf = Socket[thread].client.get_ReceivedSizeRX();                            // получить длину входного пакета
				if(sizeBuf == 0) {
					if(Socket[thread].client.connected()) continue; else break;
				}
				//      if(len>W5200_MAX_LEN-1) len=W5200_MAX_LEN-1;                             // Ограничить размером в максимальный размер пакета w5200
				Socket[thread].client.read(Socket[thread].inBuf, sizeBuf);                      // прочитать буфер
				loadLen = loadLen + ff.write(Socket[thread].inBuf, sizeBuf);                        // записать
				numPoint++;
				if(numPoint >= 20) {// точка на 30 кб приема (20 пакетов по 1540)
					numPoint = 0;
					journal.jprintf(".");
				}
			}
			ff.close();
			if(loadLen == lenFile) journal.jprintf("%d\n", loadLen);
			else { // Длины не совпали
				journal.jprintf("%db, Error length!\n", loadLen);
				loadLen = 0;
			}
		} else journal.jprintf("Error open!\n");
	} else journal.jprintf("Error create!\n");
	return loadLen;
}
