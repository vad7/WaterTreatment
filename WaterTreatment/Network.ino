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
#include "Constant.h"                       // Вся конфигурация и константы проекта Должен быть первым !!!!
#include "utility/w5100.h"
#include "utility/socket.h"
#include <ICMPPing.h>
// -------------------------------------------------------------------------------------------
// Работа с W5200 c поддержкой многопоточности и Free RTOS
// по возможности работаем через сокеты
// -------------------------------------------------------------------------------------------
static unsigned long connectTime[MAX_SOCK_NUM];    // время соединения сокета, здесь по всем сокетам (и служебному)
#define W5200_LINK        0x20                     // МАСКА регистра PHYSTATUS(W5200 PHY status Register) при котором считается что связь есть
#define W5500_LINK        0x01                     // МАСКА регистра PHYCFGR  (W5500 PHY Configuration Register) [R/W] [0x002E] при котором считается что связь есть
#define W5500_SPEED       0x02                     // МАСКА регистра PHYCFGR  (W5500 PHY Configuration Register) [R/W] [0x002E] определяется скорость Speed Status
#define W5500_DUPLEX      0x04                     // МАСКА регистра PHYCFGR  (W5500 PHY Configuration Register) [R/W] [0x002E] определяется дуплекс Duplex Status

// SPI переключение между устройствами -------------------------------------------------------------------------------------------------
// Функции переключения SPI между тремя! устройствами (активный уровень низкий)
 __attribute__((always_inline)) inline void SPI_switchW5200()   // Переключение на сеть
{ //_delay(1);
  
  digitalWriteDirect(PIN_SPI_CS_SD,HIGH);  
  #ifdef SPI_FLASH
  digitalWriteDirect(PIN_SPI_CS_FLASH,HIGH);  
  #endif
  digitalWriteDirect(PIN_SPI_CS_W5XXX,LOW);
   }

 __attribute__((always_inline)) inline void SPI_switchSD()     // Переключение на карту памяти
{
 // _delay(1);

  #ifdef SPI_FLASH
  digitalWriteDirect(PIN_SPI_CS_FLASH,HIGH);  
  #endif
  digitalWriteDirect(PIN_SPI_CS_W5XXX,HIGH);  
  digitalWriteDirect(PIN_SPI_CS_SD,LOW);
 
  }
#ifdef SPI_FLASH
 __attribute__((always_inline)) inline void SPI_switchFlash()  // переключение на флеш память
{
 // _delay(1);
  digitalWriteDirect(PIN_SPI_CS_SD,HIGH);  
  digitalWriteDirect(PIN_SPI_CS_W5XXX,HIGH);
  digitalWriteDirect(PIN_SPI_CS_FLASH,LOW);
 
   }
#endif
 __attribute__((always_inline)) inline void SPI_switchAllOFF()  // Все выключить
{
 // _delay(1);
  digitalWriteDirect(PIN_SPI_CS_SD,HIGH);  
  digitalWriteDirect(PIN_SPI_CS_W5XXX,HIGH);
  #ifdef SPI_FLASH
  digitalWriteDirect(PIN_SPI_CS_FLASH,HIGH);
  #endif
}

// Функции для первоначальной настройки сетевого чипа  ----------------------------------------------------------------
// Получить номер версии сетевого чипа
uint8_t W5200VERSIONR()
{
  #if defined(W5100_ETHERNET_SHIELD)
    return 51;                    // В чипе w5100 нет номера версии по этому делаем условно 51
  #else
    return W5100.readVERSIONR();  // Для w5200 и 5500
  #endif
}
// Проверить состояние сетевого чипа без сброса! (бит LINK)
// show - нужен вывод в консоль или нет, возврат true- связь есть
boolean linkStatusWiznet(boolean show)
{
#if defined(W5500_ETHERNET_SHIELD) // Задание имени чипа для вывода сообщений
	uint8_t st = W5100.readPHYCFGR();
	if(show) {
#ifdef W5500_LOG_FULL_INFO
		if(st & W5500_SPEED) journal.jprintfopt(" Speed Status: 100Mpbs\n"); else journal.jprintfopt(" Speed Status: 10Mpbs\n");
		if(st & W5500_DUPLEX) journal.jprintfopt(" Duplex Status: full duplex\n"); else journal.jprintfopt(" Duplex Status: half duplex\n");
		journal.jprintfopt(" Register PHYCFGR: 0x%02x\n", st);
#else
		journal.jprintfopt(" %s%c[%02X] ", st & W5500_SPEED ? "100" : "10", st & W5500_DUPLEX ? 'F' : 'H', st);
#endif
	}
	if(st & W5500_LINK) return true;
	else return false;
#elif defined(W5200_ETHERNET_SHIELD)
	if (W5100.readPHYSTATUS()&W5200_LINK) return true; else return false;
#else // w5100
	return true;
#endif
}

// Сброс чипа и проверка на соедиенние, делается несколько попыток (бит LINK)
// show - нужен вывод в консль или нет, возврат true- связь есть
boolean resetWiznet(boolean show)
{
    uint8_t i;
    uint16_t t;
    for (i = 0; i <  W5200_NUM_LINK; i++)  // делается несколько попыток связи до появления LINK с задержкой
    { 
     WDT_Restart(WDT);
     digitalWriteDirect(PIN_ETH_RES, LOW); _delay(5);digitalWriteDirect(PIN_ETH_RES, HIGH);                        // Аппаратный сброс чипа (если он завис вдруг это помогает)
     W5100.init();                                                                                                 // Программная инициализация чипа (программный сброс и программирование)
     for (t = 0; t <  W5200_TIME_LINK; t=t+50)                                                                     // Ожидание установления связи но не более W5200_TIME_LINK мсек
       {
       _delay(50);                                                                                                 
       if (linkStatusWiznet(false)) { if(show)journal.jprintfopt(" %s: link OK (time %d mc)\n",(char*)__FUNCTION__, t);return true;}  // link есть, едим дальше
       }
     if (show) journal.jprintfopt(" %s: no link\n",(char*)__FUNCTION__);
    }
  return false;                                                                                                     // линка нет
}
// Инициализация сети
// flag true - полный вывод на консоль false - скоращенный вывод на консоль
// Проверят сетевой кабель, возврат true - OK false - проблемы, сеть не работает
const char* NetworkChipOK={" Network library setting: %s, ID chip: 0x%x\n"};
const char* NetworkChipBad={" WRONG setting library, library: %s, ID: chip 0x%x\n"};
const char* NetworkError={" $ERROR: Problem reset and setting %s\n"};
boolean initW5200(boolean flag)
{
	uint8_t i;
	boolean EthernetOK = true;   // флаг успешности инициализации
	pinMode(PIN_ETH_RES, OUTPUT);
	if(flag) journal.jprintfopt("Network setup:");
	if(!resetWiznet(false))  // 1. Сброс и проверка провода (молча)
	{
#ifdef W5500_LOG_FULL_INFO
		journal.jprintfopt(" WARNING: %s no link, check ethernet cable\n", nameWiznet);
		journal.jprintfopt((char*) NetworkError, nameWiznet);
		return false; // дальше ехать бесполезно
	} else if(flag) journal.jprintfopt(" SUCCESS: %s link OK\n", nameWiznet);
#else
		journal.jprintfopt(" WARNING: %s no link\n", nameWiznet);
		return false;
	}
#endif
	linkStatusWiznet(flag);  // вывести полученные настройки чипа

	if(flag)  // 2. Печать настроек соответствия либы и чипа (правильная настройка либы)
	{
#if defined(W5500_ETHERNET_SHIELD) // Определение соответстивия библиотеки и чипа
		if(W5200VERSIONR() == 0x04) {
#ifdef W5500_LOG_FULL_INFO
			journal.jprintfopt((char*) NetworkChipOK, nameWiznet, W5200VERSIONR());
#endif
		} else {
			journal.jprintfopt((char*) NetworkChipBad, nameWiznet, W5200VERSIONR());
			journal.jprintfopt((char*) NetworkError, nameWiznet);
			return false;
		} // дальше ехать бесполезно
#elif defined(W5200_ETHERNET_SHIELD)
		if (W5200VERSIONR()==0x03) journal.jprintfopt((char*)NetworkChipOK,nameWiznet,W5200VERSIONR());
		else {journal.jprintfopt((char*)NetworkChipBad,nameWiznet,W5200VERSIONR());journal.jprintfopt((char*)NetworkError,nameWiznet); return false;} // дальше ехать бесполезно
#else
		if (W5200VERSIONR()==0x51) journal.jprintfopt((char*)NetworkChipOK,nameWiznet,W5200VERSIONR());
		else {journal.jprintfopt((char*)NetworkChipBad,nameWiznet,W5200VERSIONR());journal.jprintfopt((char*)NetworkError,nameWiznet); return false;} // дальше ехать бесполезно
#endif
	}

	//  3. Подготовить структура для потоков
	for(i = 0; i < W5200_THREAD; i++) {
		Socket[i].flags = 0x00;
		Socket[i].sock = -1;
		memset((char*) Socket[i].inBuf, 0x00, sizeof(Socket[i].inBuf));
		memset((char*) Socket[i].outBuf, 0x00, sizeof(Socket[i].outBuf));
	}

	// 4. Иницилизация сетевого адаптера, установка сетевых настроек
	WDT_Restart(WDT);                          // Сбросить вачдог  DHCP при отключенном кабеле - большой таймаут
	if(MC.safeNetwork) {
		Ethernet.begin((uint8_t*) defaultMAC, (IPAddress) defaultIP, (IPAddress) defaultSDNS,
				(IPAddress) defaultGateway, (IPAddress) defaultSubnet); // Инициализация сетевого адаптера  в режиме safeNetwork = КОНСТАНТЫ
		if(defaultIP != Ethernet.localIP()) EthernetOK = false;
		else {
			beginWeb(defaultPort);
			journal.jprintfopt(" Set mode safeNetwork!\n");
		}
	} else {
		if(MC.get_DHCP()) // Работаем по DHCP
		{
			journal.jprintfopt(" Try DHCP: ");
			WDT_Restart(WDT);
			if(Ethernet.begin((uint8_t*) MC.get_mac()) == 0) {
				journal.jprintfopt("Failed!\n");
				goto x_TryStaticIP;
			} else {
				journal.jprintfopt("OK\n");
				MC.set_ip(Ethernet.localIP());       // Получили удачно DHCP адрес - сохраняем в сетевые настройки
				MC.set_subnet(Ethernet.subnetMask());
				MC.set_sdns(Ethernet.dnsServerIP());
				MC.set_gateway(Ethernet.gatewayIP());
			}
		} else {
x_TryStaticIP:
			WDT_Restart(WDT);
			Ethernet.begin((uint8_t*) MC.get_mac(), (IPAddress) MC.get_ip(), (IPAddress) MC.get_sdns(),
					(IPAddress) MC.get_gateway(), (IPAddress) MC.get_subnet()); // Статика
			if(MC.get_ip() != Ethernet.localIP()) EthernetOK = false;
			else beginWeb(MC.get_port());
		}
	}

	pingW5200(MC.get_NoPing());  // Установка пинга флага разрешенеия пинга
//	W5100.writeMR(W5100.readMR() | 2);	// FARP flag
	W5100.writeRTR(W5200_RTR);   // установка таймаута
	W5100.writeRCR(W5200_RCR);   // установка числа повторов

#ifdef W5500_LOG_FULL_INFO
	if(flag)  // 5. Печать сетевых настроек
	{
		if(EthernetOK) {
			journal.jprintfopt(" DHCP use: ");
			if(MC.get_DHCP()) journal.jprintfopt("YES\n");
			else journal.jprintfopt("NO\n");
			IPAddress dip;
			dip = Ethernet.localIP();
			journal.jprintfopt(" IP: %s\n", IPAddress2String(dip));
			dip = Ethernet.subnetMask();
			journal.jprintfopt(" Subnet: %s\n", IPAddress2String(dip));
			dip = Ethernet.dnsServerIP();
			journal.jprintfopt(" DNS: %s\n", IPAddress2String(dip));
			dip = Ethernet.gatewayIP();
			journal.jprintfopt(" Gateway: %s\n", IPAddress2String(dip));
		} else journal.jprintfopt((char*) NetworkError, nameWiznet);
	} else   // Кратко выводим сообщение в журнал
	{
		if(EthernetOK) journal.jprintfopt_time("Reset %s . . . \n", nameWiznet);
		else journal.jprintfopt((char*) NetworkError, nameWiznet);
	}
#else
	if(flag)  // 5. Печать сетевых настроек
	{
		if(EthernetOK) {
			IPAddress dip;
			dip = Ethernet.localIP();
			journal.jprintfopt("%s%s/%d ", MC.get_DHCP() ? "DHCP " : "", IPAddress2String(dip), calc_bits_in_mask(Ethernet.subnetMask()));
			dip = Ethernet.gatewayIP();
			journal.jprintfopt("G:%s ", IPAddress2String(dip));
			dip = Ethernet.dnsServerIP();
			journal.jprintfopt("DNS:%s\n", IPAddress2String(dip));
		} else journal.jprintfopt(" ERROR: setting %s", nameWiznet);
	} else   // Кратко выводим сообщение в журнал
	{
		if(EthernetOK) journal.jprintfopt("Reset %s Ok.\n", nameWiznet);
		else journal.jprintfopt(" ERROR: setting %s", nameWiznet);
	}
#endif
	return EthernetOK;
}
// DNS -------------------------------------------------------------------------------------------
// Используется системный сокет!! W5200_SOCK_SYS
// Проверить и преобразовать тип адреса (буквы или цифры) и если это буквы то резольвить через dns
// Задача определить  IP адрес. Если на входе был также IP то он и возвращается,
// При не удаче возвращается 0, при удаче 1 - нашли сразу, 2 - был запрос к DNS
uint8_t check_address(char *adr, IPAddress &ip)
{
	IPAddress tempIP(0, 0, 0, 0);
	DNSClient dns;
	int8_t ret = 0;
	// 1. Попытка преобразовать строку в IP (цифры, нам повезло DNS не нужен)
	if(parseIPAddress(adr, '.', tempIP)) {
		//        journal.jprintfopt("Input string is address %s\n",adr);  // Сообщение что ДНС не требуется, входная строка и так является адресом
		ip = tempIP;
		return 1;
	} // Удачно выходим
	// 2. Это буквы, нужен dns
	dns.begin(Ethernet.dnsServerIP());    // только запоминаем dnsServerIP ничего больше не делаем с сокетом
	ret = dns.getHostByName(adr, tempIP, W5200_SOCK_SYS); // вот тут с сокетами начинаем работать
	if(ret == 1)  // Адрес получен
	{
		journal.jprintfopt(" %s", adr);
		journal.jprintfopt(" resolved by %s to %d.%d.%d.%d\n", dns.get_protocol() ? "TCP" : "UDP", tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
		ip = tempIP;
		return 2;
	} else {
		journal.jprintfopt(" %s DNS lookup by %s failed! Code: %d\n", adr, dns.get_protocol() ? "TCP" : "UDP", ret);
		ip = tempIP;
		return 0;
	}
}
// Вывести состояние регистров сокета -------------------------------------------------------------
void ShowSockRegisters(uint8_t s)
{
 journal.jprintf("#%d", s); // номер сокета
 journal.jprintf(" MR:%02x",W5100.readSnMR(s)); 
 journal.jprintf(" CR:%02x",W5100.readSnCR(s)); 
 journal.jprintf(" IR:%02x",W5100.readSnIR(s)); 
 journal.jprintf(" SR:%02x",W5100.readSnSR(s)); 
 journal.jprintf(" PORT:%04x",W5100.readSnPORT(s)); 
 journal.jprintf(" DPORT:%04x",W5100.readSnDPORT(s));  
 #if defined(W5500_ETHERNET_SHIELD)  
 journal.jprintf(" TOS:%02x",W5100.readSnTOS(s));
 journal.jprintf(" TTL:%02x",W5100.readSnTTL(s));
// journal.jprintf(" IMR:%02x",W5100.readSnIMR(s)); 
// journal.jprintf(" FRAG:%04x",W5100.readSnFRAG(s)); 
// journal.jprintf(" KPALVTR:%02x",W5100.readSnKPALVTR(s)); 
 #endif
 journal.jprintf(" "); 
 
}


// Вывести статистику по сокетам -------------------------------------------------------------
void ShowSockStatus()
{
  journal.jprintf("- Socket info -\n");
  for (int i = 0; i < MAX_SOCK_NUM; i++) { // По всем сокетам!!
    journal.jprintf("Socket#");
    journal.jprintf("%d",i);
    uint8_t s = W5100.readSnSR(i);  // Статус
    journal.jprintf(":0x");
    journal.jprintf("%x",s);
    journal.jprintf(" %d <",W5100.readSnPORT(i));
    uint8_t dmac[6];
    W5100.readSnDHAR(i,dmac);
    journal.jprintf("%s",MAC2String(dmac));
    journal.jprintf("> ");
    journal.jprintf(" D:");
    uint8_t dip[4];
    W5100.readSnDIPR(i, dip);
    for (int j=0; j<4; j++) {
      journal.jprintf("%d",dip[j],10);
      if (j<3) journal.jprintf(".");
      }
    journal.jprintf("(%d)\n",W5100.readSnDPORT(i));

  }
}

// Получить строку со статусом всех сокетов -------------------------------------------------
char* socketInfo(char *buf)
{
  for (int i = 0; i < MAX_SOCK_NUM; i++)                      // По всем сокетам!!
  {
   _itoa(i,buf);                                    // Номер по списку
   strcat(buf,"|");
   uint8_t s = W5100.readSnSR(i);                             // статус сокета
   switch (s)
         {
          case 0x00: strcat(buf,"0x00 CLOSED"); break;
          case 0x01: strcat(buf,"0x01 ARP <sup>1</sup>"); break;
          case 0x13: strcat(buf,"0x13 INIT"); break;
          case 0x14: strcat(buf,"0x14 LISTEN"); break;
          case 0x15: strcat(buf,"0x15 SYNSENT <sup>1</sup>"); break;
          case 0x16: strcat(buf,"0x16 SYNRECV <sup>1</sup>"); break;
          case 0x17: strcat(buf,"0x17 ESTABLISHED <sup>2</sup>"); break;
          case 0x18: strcat(buf,"0x18 FIN_WAIT <sup>1</sup>"); break;
          case 0x1a: strcat(buf,"0x1a CLOSING <sup>1</sup>"); break;
          case 0x1b: strcat(buf,"0x1b TIME_WAIT <sup>1</sup>"); break;
          case 0x1c: strcat(buf,"0x1c CLOSE_WAIT <sup>2</sup>"); break;
          case 0x1d: strcat(buf,"0x1d LAST_ACK <sup>1</sup>"); break;
          case 0x22: strcat(buf,"0x22 UDP"); break;
          case 0x32: strcat(buf,"0x32 IPRAW"); break;
          case 0x42: strcat(buf,"0x42 MACRAW"); break;
          case 0x5f: strcat(buf,"0x5f PPPOE"); break;
          default:   strcat(buf,byteToHex(s));strcat(buf," unknow state");  break;
         }   
  strcat(buf,"|");
  uint8_t dmac[6];
  W5100.readSnDHAR(i,dmac);                                   // mac адрес
  strcat(buf,MAC2String(dmac));  
  strcat(buf,"|");
  uint8_t dip[4];
  W5100.readSnDIPR(i, dip);                                  // IP адрес
    for (int j=0; j<4; j++) { _itoa(dip[j],buf);if (j<3) strcat(buf,"."); }
  strcat(buf,"|");
  _itoa(W5100.readSnDPORT(i),buf);                 // Порт
  strcat(buf,";"); 
  }
 return buf; 
}
// Сброс зависших сокетов ------------------------------------------------------------
// Учитывается многопоточность, не сбрасываются сокеты которые сейчас в работе Socket[xxxx].sock
void checkSockStatus()
{
  unsigned long thisTime = GetTickCount();
  if(SemaphoreTake(xWebThreadSemaphore,(W5200_TIME_WAIT/portTICK_PERIOD_MS))==pdFALSE) {journal.jprintfopt((char*)cErrorMutex,__FUNCTION__,MutexWebThreadBuzy);return;} // Захват мютекса потока или ОЖИДАНИНЕ W5200_TIME_WAIT
  for (uint8_t i = 0; i < MAX_SOCK_NUM; i++) {        // По всем сокетам!!
        // Не сбрасывать сокеты которые используется в потоке ОБЯЗАТЕЛЬНО!!
        #if    W5200_THREAD < 2
         if (Socket[0].sock==i)  continue;   
        #elif  W5200_THREAD < 3
          if((Socket[0].sock==i)||(Socket[1].sock==i))  continue;    
        #elif  W5200_THREAD < 4
          if((Socket[0].sock==i)||(Socket[1].sock==i)||(Socket[2].sock==i))  continue;   
        #else
          if((Socket[0].sock==i)||(Socket[1].sock==i)||(Socket[2].sock==i)||(Socket[3].sock==i))  continue;   
        #endif
    uint8_t s = W5100.readSnSR(i);                                          // Прочитать статус сокета
    if((s == SnSR::ESTABLISHED) || (s == SnSR::CLOSE_WAIT) /*|| (s == 0x22)*/ ) { // если он "кандидат"
        if(thisTime - connectTime[i] > MC.time_socketRes()*1000UL) {        // Время пришло
          journal.jprintfopt("%s : Socket frozen: %d\n",NowTimeToStr(),i);
    //      close(i);
          W5100.execCmdSn(i, Sock_CLOSE);
          W5100.writeSnIR(i, 0xFF);
          MC.add_socketRes();                                               // добавить счетчик
        }
    } // if((s == 0x17) || (s == 0x1C))
    else connectTime[i] = thisTime;                                         // Обновить время если статус не кандидат
  } // for
  SemaphoreGive(xWebThreadSemaphore);                                      // Отдать мютекс
}
// Послать один пакет!!! ----------------------------------------------------------------------------------
// Послать данные TCP (максимальный размер данных W5200_MAX_LEN. больше обрезается), при ожидании освобождения буфера отдает управление Free RTOS
// Для разделения доступа к spi используется мютекс  (SemaphoreHandle_t xSPI)
// возвращает число посланых байт (0-ничего не послано)
// НЕ РАБОТАЕТ с КОНСТАНТАМИ!! их предварительно надо скопировать в ОЗУ т.к. используется DMA
// Если pause=0 то ждем подтверждения ACK пакета если pause>0 то просто ждем нужное значение (pause) мсек - работа без подтвержения
uint16_t sendPacketRTOS(uint8_t thread, const uint8_t * buf, uint16_t len, uint16_t pause)
{
	uint8_t status = 0;
	uint16_t ret = 0;
	uint16_t freesize = 0;

	SPI_switchW5200();
	if(len > W5200_MAX_LEN) ret = W5200_MAX_LEN;
	else ret = len;

	if(pause == 0) // Честно ждем ack
	{
		//   Serial.println("ask");
		do// Ожидание освобождения буфера
		{
			if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
				xSemaphoreGive (xWebThreadSemaphore);  //                                      // Мютекс потока отдать
				taskYIELD();
			} else delay(1);
			if(SemaphoreTake(xWebThreadSemaphore, (W5200_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
				journal.jprintfopt("Socket: %d %s\n", Socket[thread].sock, MutexWebThreadBuzy);
				return 0;
			} // Захват мютекса потока или ОЖИДАНИНЕ W5200_TIME_WAIT
			//taskENTER_CRITICAL();
			freesize = W5100.getTXFreeSize(Socket[thread].sock);
			if(freesize >= ret) {
				//taskEXIT_CRITICAL();
				break;
			}
			status = W5100.readSnSR(Socket[thread].sock);
			//taskEXIT_CRITICAL();
			if((status != SnSR::ESTABLISHED) && (status != SnSR::CLOSE_WAIT)) {
				ret = 0;
				break;
			}
		} while(freesize < ret);
	} else  // Не ждем ACK а просто делаем задержку
	{
		//  Serial.println("pause no ask");
		SemaphoreGive (xWebThreadSemaphore);                                                             // Мютекс потока отдать
		_delay(pause);                                                            // Ждем pause мсек
		if(SemaphoreTake(xWebThreadSemaphore, (W5200_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) {
			journal.jprintfopt("Socket: %d %s\n", Socket[thread].sock, MutexWebThreadBuzy);
			return 0;
		} // Захват мютекса потока или ОЖИДАНИНЕ W5200_TIME_WAIT
	}

	if(GETBIT(Socket[thread].flags, fABORT_SOCK)) {
		SETBIT0(Socket[thread].flags, fABORT_SOCK);
		return 0;
	}              // Произошел сброс прерываем передачу

	// послать данные
	//taskENTER_CRITICAL();
	W5100.send_data_processing_offset(Socket[thread].sock, 0, (uint8_t *) buf, ret);
	//taskEXIT_CRITICAL();
	W5100.execCmdSn(Socket[thread].sock, Sock_SEND);
	////taskEXIT_CRITICAL();

	// +2008.01 bj : reduce code
	while((W5100.readSnIR(Socket[thread].sock) & SnIR::SEND_OK) != SnIR::SEND_OK) {
		if(W5100.readSnSR(Socket[thread].sock) == SnSR::CLOSED) {
			close(Socket[thread].sock);
			return 0;
		}
	}
	W5100.writeSnIR(Socket[thread].sock, SnIR::SEND_OK);
	return ret;
}

// Послать буфер, может быть множество пакетов!!
// Послать данные TCP (максимальный размер данных не ограничен, может отправлять несколько пакетов),
// Использует функцию sendPaketRTOS
// возвращает число посланых байт (0-ничего не послано, или ошибка передачи)
// НЕ РАБОТАЕТ с КОНСТАНТАМИ!! их предварительно надо скопировать в ОЗУ т.к. используется DMA
uint16_t sendBufferRTOS(uint8_t thread, const uint8_t * buf, uint16_t len)
{
	uint16_t i = 0;
	while(len > 0) {
		if(len > W5200_MAX_LEN) {
			if(sendPacketRTOS(thread, (byte*) buf + i, W5200_MAX_LEN, MC.get_NoAck() ? MC.get_delayAck() : 0) == 0) { // ошибка передачи
				return 0;
			}
			i += W5200_MAX_LEN;
			len -= W5200_MAX_LEN;
		} else {  // последний пакет или ошибка передачи
			if(sendPacketRTOS(thread, (byte*) buf + i, len, MC.get_NoAck() ? MC.get_delayAck() : 0) == 0) return 0;
			else {
				i += len;
				break;
			}
		}
	}
	return i;
}


// Послать один пакет КОНСТАНТ!!!
// Послать данные TCP (максимальный размер данных W5200_MAX_LEN. больше обрезается), при ожидании освобождения буфера отдает управление Free RTOS
// Для разделения доступа к spi используется мютекс  (SemaphoreHandle_t xSPI)
// возвращает число посланых байт (0-ничего не послано)
// РАБОТАЕТ С КОНСТАНТАМИ, идет предварительное копирование в буфер потока
 __attribute__((always_inline)) inline uint16_t sendConstRTOS(uint8_t thread, const char * buf)
{
if (strlen(buf)>W5200_MAX_LEN) return 0;   // В один пакет не влезает выходим
        strcpy(Socket[thread].outBuf,buf); 
return  sendPacketRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf),0);   
}

 // Принт в сокет
uint16_t sendPrintfRTOS(uint8_t thread, const char *format, ...)             
{
  va_list ap;
  va_start(ap, format);
  m_vsnprintf((Socket[thread].outBuf), sizeof((Socket[thread].outBuf)), format, ap);
  va_end(ap);
  return sendBufferRTOS(thread, (byte*)Socket[thread].outBuf, strlen((Socket[thread].outBuf)));
}
     
// Куски либы переделанные под многопоточность, сделаны как отдельные функции
// определение - необходимости отработки сокета
// Старт прослушивания порта веб морды (подъем веб сервера), на прослушивание ставятся не все сокеты, системный остается свободным
void beginWeb(uint16_t port)
{
 for (int sock = 0; sock < W5200_SOCK_SYS; sock++)
 {
    EthernetClient client(sock);
    if (client.status() == SnSR::CLOSED) 
    {
      socket(sock, SnMR::TCP, port, 0);
      listen(sock);
      EthernetClass::_server_port[sock] = MC.get_port();
      break;
    }
  }  
  
}

// Пингование сервера
ICMPPing ping(W5200_SOCK_SYS, (uint16_t)random(0, 255));
boolean pingServer()
{
	IPAddress ip;
	if(SemaphoreTake(xWebThreadSemaphore,(W5200_TIME_WAIT/portTICK_PERIOD_MS))==pdFALSE)  {return false;}  // Захват семафора потока или ОЖИДАНИЕ W5200_TIME_WAIT, если семафор не получен то выходим
	if(!check_address(MC.get_pingAdr(), ip)) {journal.jprintfopt("Wrong ping server\n"); SemaphoreGive(xWebThreadSemaphore); return false;}  // адрес не верен, или DNS не работает - ничего не делаем
 	// Адрес правильный
	ping.setTimeout(W5200_TIME_PING);                   // время между попытками пинга мсек
	WDT_Restart(WDT);                                   // Сбросить вачдог
	ICMPEchoReply echoReply = ping(ip,W5200_NUM_PING);  // адрес и число попыток
	SemaphoreGive(xWebThreadSemaphore);                 // отдать семафор
#ifndef DONT_LOG_SUCCESS_PING
	journal.jprintfopt("Ping[%d] ", echoReply.data.seq);
#endif
	if(echoReply.status == SUCCESS) {
#ifndef DONT_LOG_SUCCESS_PING
		//journal.jprintfopt("%dms TTL=%u\n", GetTickCount() - echoReply.data.time, echoReply.ttl);
		if(ping.attempts()) {
			journal.jprintfopt("%dms, lost: %d.\n", GetTickCount() - echoReply.data.time, ping.attempts());
		} else {
			journal.jprintfopt("%dms\n", GetTickCount() - echoReply.data.time);
		}
#endif
		return true;
	} else {
#ifdef DONT_LOG_SUCCESS_PING
		journal.jprintfopt("Ping[%d] %d.%d.%d.%d: FAILED - ", echoReply.data.seq, ip[0], ip[1], ip[2], ip[3]);
#else
		journal.jprintfopt("%d.%d.%d.%d: FAILED - ", ip[0], ip[1], ip[2], ip[3]);
#endif
		switch (echoReply.status)
		{
		case SEND_TIMEOUT: journal.jprintfopt( "send timed out");  break;
		case NO_RESPONSE:  journal.jprintfopt( "no response");    break;
		case BAD_RESPONSE: journal.jprintfopt( "bad reponse");        break;
		default:           journal.jprintfopt( "error: %d", echoReply.status); break;
		}
		if(!MC.NO_Power) {
			journal.jprintfopt(", Resetting %s...\n", nameWiznet);
			initW5200(true);                                  // Инициализация сети с выводом инфы в консоль
			for(uint8_t i = 0; i < W5200_THREAD; i++) SETBIT1(Socket[i].flags,fABORT_SOCK);                                 // Признак инициализации сокета, надо прерывать передачу в сервере
			MC.num_resPing++;
		}
		return false;
	}
	return false;
}

// Установка флага пингования контроллера w5200 или w5500 на w5100 не проверялся
// true - пинг запрещен
// false - пинг разрешен
#define MR_BIT_PB 0x04  //MR (Mode Register) Ping Block Mode bit (0x04) If the bit is ‘1’, it blocks the response to a ping request.
void pingW5200(boolean f)
{
	uint8_t x = W5100.readMR();
	if(f) {
		SETBIT1(x, MR_BIT_PB);
#ifdef W5500_LOG_FULL_INFO
		journal.jprintfopt(" Enable Ping block\n");
#endif
	} else {
		SETBIT0(x, MR_BIT_PB);
#ifdef W5500_LOG_FULL_INFO
		journal.jprintfopt(" Disable Ping block\n");
#endif
	}
	W5100.writeMR(x);
}

// =============================== M Q T T ==================================================
#ifdef MQTT    // признак использования MQTT
static char root[60],topic[140], temp[16];
const char* MQTTpublish={">> %s "};
const char* MQTTPublishOK={"OK\n"};
const char* MQTTDebugStr={" %s %s,"};  // вывод информации при отладке

// Посылка данных на народный мониторинг
// debug  true - выводить в консоль информацию о посылаемых данных false - нет вывода
boolean sendNarodMon(boolean debug)
{
 uint8_t i;
     
     if (memcmp(defaultMAC,MC.get_mac(),sizeof(defaultMAC))==0) {journal.jprintf("sendNarodMon ignore: Wrong MAC address, change MAC from default.\n"); return false;}
     journal.jprintf((char*)MQTTpublish,MC.clMQTT.get_narodMon_server());

     strcpy(root,"");  // Формирование строки корня, куда потом пишутся топики
     MC.clMQTT.get_paramMQTT((char*)mqtt_LOGIN_NARMON,root);
     strcat(root,"/");
     MC.clMQTT.get_paramMQTT((char*)mqtt_ID_NARMON,root);
     strcat(root,"/");
     
     // посылка отдельных топиков
     strcpy(topic,root);
     strcat(topic,MC.sTemp[TOUT].get_name());
     ftoa(temp,(float)MC.sTemp[TOUT].get_Temp()/100.0f,1);
     if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
    
     strcpy(topic,root);
     strcat(topic,MC.sTemp[TIN].get_name());
     ftoa(temp,(float)MC.sTemp[TIN].get_Temp()/100.0f,1);
     if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
     
     strcpy(topic,root);
     strcat(topic,MC.sTemp[TBOILER].get_name());
     ftoa(temp,(float)MC.sTemp[TBOILER].get_Temp()/100.0f,1);
     if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

     strcpy(topic,root);
     if(MC.dFC.get_present())
         {
          strcat(topic,"FC");
          ftoa(temp,(float)MC.dFC.get_frequency()/100.0f,2);
          if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
         }
     else
         {
         strcat(topic,MC.dRelay[RCOMP].get_name());
         if (MC.dRelay[RCOMP].get_Relay()){ if (MC.clMQTT.sendTopic(topic,(char*)cOne,true,debug,false)) {if (debug) journal.jprintf(" %s 1\n", topic);}  else return false;  }
         else                             { if (MC.clMQTT.sendTopic(topic,(char*)cZero,true,debug,false)) {if (debug) journal.jprintf(" %s 0\n", topic);}  else return false;  }
         } 
         
         strcpy(topic,root);
         strcat(topic,"ERROR");
         itoa(MC.get_errcode(),temp,10);
         if (MC.clMQTT.sendTopic(topic,temp,true,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
         
        if (debug) journal.jprintf(cStrEnd);   
         
      // Послать расширенный набор данных TCOMP OWERHEAT мощность выходная коп полный, положение ЭРВ, два давления,
      if (MC.clMQTT.get_NarodMonBig())
         {
         _delay(100);// пауза перед отправкой следующего пакета - разгружаем сервер и балансируем загрузку у себя
         if (debug) journal.jprintf("Additional data:");  
         strcpy(topic,root);
         strcat(topic,MC.sTemp[TCOMP].get_name());
         ftoa(temp,(float)MC.sTemp[TCOMP].get_Temp()/100.0,1);
         if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

         strcpy(topic,root);
         #ifdef EEV_DEF
         strcat(topic,"OWERHEAT");
         ftoa(temp,(float)MC.dEEV.get_Overheat()/100.0,1);
         #else
          strcat(topic,MC.sTemp[TEVAOUTG].get_name());
          if(MC.sTemp[TEVAOUTG].get_present()) ftoa(temp,(float)MC.sTemp[TEVAOUTG].get_Temp()/100.0,1);
          else   strcat(topic,"skip1 0");
         #endif
         if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

         for(i=0;i<ANUMBER;i++)
             if(MC.sADC[i].get_present())
             {
                 strcpy(topic,root);
                 strcat(topic,MC.sADC[i].get_name());
                 ftoa(temp,(float)MC.sADC[i].get_Press()/100.0,2);
                 if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
             }
         
             strcpy(topic,root);
           #ifdef EEV_DEF   
             strcat(topic,MC.dEEV.get_name());
             itoa(MC.dEEV.get_EEV(),temp,10);
           #else
             strcat(topic,MC.sTemp[TEVAING].get_name());
             if(MC.sTemp[TEVAING].get_present()) ftoa(temp,(float)MC.sTemp[TEVAING].get_Temp()/100.0,1);
             else   strcat(topic,"skip2 0");
           #endif  
             if (MC.clMQTT.sendTopic(topic,temp,true,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
            
           strcpy(topic,root);
           strcat(topic,"powerCO");
           ftoa(temp,MC.powerCO,1);
           if (MC.clMQTT.sendTopic(topic,temp,true,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
           if (debug) journal.jprintf(cStrEnd);   
         }//  if (MC.clMQTT.get_NarodMonBig())
         if (!debug) journal.jprintf((char*)MQTTPublishOK);
           
  return true;
}

// Посылка данных на брокер MQTT
// debug  true - выводить в консоль информацию о посылаемых данных false - нет вывода
boolean sendMQTT(boolean debug)
{
 uint8_t i; 
     strcpy(root,"");  // Формирование строки корня, куда потом пишутся топики
     journal.jprintf((char*)MQTTpublish,MC.clMQTT.get_mqtt_server()); //if (!debug) journal.jprintf(" OK\n");
     MC.clMQTT.get_paramMQTT((char*)mqtt_ID_MQTT,root);
     strcat(root,"/");
     
     strcpy(topic,root);
     strcat(topic,MC.sTemp[TOUT].get_name());
     ftoa(temp,(float)MC.sTemp[TOUT].get_Temp()/100.0,1);
     if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
     
     strcpy(topic,root);
     strcat(topic,MC.sTemp[TIN].get_name());
     ftoa(temp,(float)MC.sTemp[TIN].get_Temp()/100.0,1);
     if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
     
     strcpy(topic,root);
     strcat(topic,MC.sTemp[TBOILER].get_name());
     ftoa(temp,(float)MC.sTemp[TBOILER].get_Temp()/100.0,1);
     if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

     strcpy(topic,root);
     if(MC.dFC.get_present())
         {
          strcat(topic,"FC");
          ftoa(temp,(float)MC.dFC.get_frequency()/100.0,1);
          if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
         }
     else
         {
         strcat(topic,MC.dRelay[RCOMP].get_name());
         if (MC.dRelay[RCOMP].get_Relay()){if (MC.clMQTT.sendTopic(topic,(char*)cOne,true,debug,false)) {if (debug) journal.jprintf(" %s 1\n", topic);}  else return false;  }
         else                             {if (MC.clMQTT.sendTopic(topic,(char*)cZero,true,debug,false)) {if (debug) journal.jprintf(" %s 0\n", topic);}  else return false;  }
         } 
         
       strcpy(topic,root);
       strcat(topic,"ERROR");
       itoa(MC.get_errcode(),temp,10);
       if (MC.clMQTT.sendTopic(topic,temp,false,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
       if (debug) journal.jprintf(cStrEnd);
         
     
      if (MC.clMQTT.get_MqttBig())   // Послать расширенный набор данных TCOMP OWERHEAT мощность выходная коп полный, положение ЭРВ, два давления,
         {
         _delay(100);// пауза перед отправкой следующего пакета - разгружаем сервер и балансируем загрузку у себя
         if (debug) journal.jprintf("Additional data:");  

        if(MC.sTemp[TCOMP].get_present())
          {
	         strcpy(topic,root);
	         strcat(topic,MC.sTemp[TCOMP].get_name());
	         ftoa(temp,(float)MC.sTemp[TCOMP].get_Temp()/100.0,1);
	         if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
          }        

         if(MC.sTemp[TCONING].get_present())
          {
             strcpy(topic,root);
             strcat(topic,MC.sTemp[TCONING].get_name());
             ftoa(temp,(float)MC.sTemp[TCONING].get_Temp()/100.0,1);
             if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
          }

          if(MC.sTemp[TCONOUTG].get_present())
          {
             strcpy(topic,root);
             strcat(topic,MC.sTemp[TCONOUTG].get_name());
             ftoa(temp,(float)MC.sTemp[TCONOUTG].get_Temp()/100.0,1);
             if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
          }
          
          if(MC.sTemp[TEVAOUTG].get_present())
          {
             strcpy(topic,root);
             strcat(topic,MC.sTemp[TEVAOUTG].get_name());
             ftoa(temp,(float)MC.sTemp[TEVAOUTG].get_Temp()/100.0,1);
             if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
          }
          if(MC.sTemp[TEVAING].get_present())
          {
             strcpy(topic,root);
             strcat(topic,MC.sTemp[TEVAING].get_name());
             ftoa(temp,(float)MC.sTemp[TEVAING].get_Temp()/100.0,1);
             if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
          }      
         strcpy(topic,root);
         strcat(topic,"OVERHEAT");
         #ifdef EEV_DEF
         ftoa(temp,(float)MC.dEEV.get_Overheat()/100.0,1);
         #else
         strcpy(temp,cZero);
         #endif
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

         for(i=0;i<ANUMBER;i++)
             if(MC.sADC[i].get_present())
             {
                 strcpy(topic,root);
                 strcat(topic,MC.sADC[i].get_name());
                 ftoa(temp,(float)MC.sADC[i].get_Press()/100.0,2);
                 if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
             }
             
             #ifdef EEV_DEF 
             strcpy(topic,root);
             strcat(topic,MC.dEEV.get_name());
             itoa(MC.dEEV.get_EEV(),temp,10);
             if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
             #endif
             
             strcpy(topic,root);
             strcat(topic,"powerCO");
             ftoa(temp,MC.powerCO,1);
             if (MC.clMQTT.sendTopic(topic,temp,false,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
             if (debug) journal.jprintf(cStrEnd);        
         }//  if (MC.clMQTT.get_MqttBig())
      
        #ifdef USE_ELECTROMETER_SDM    // Послать данные электросчетчика на сервер MQTT
        if (MC.clMQTT.get_MqttSDM120())
        {
         _delay(100);// пауза перед отправкой следующего пакета - разгружаем сервер и балансируем загрузку у себя
         if (debug) journal.jprintf("SDM120 data:");   
         strcpy(topic,root);
         strcat(topic,"ACPOWER");
         strcpy(temp,"");
         MC.dSDM.get_paramSDM((char*)sdm_ACPOWER,temp);
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

         strcpy(topic,root);
         strcat(topic,"CURRENT");
         strcpy(temp,"");
         MC.dSDM.get_paramSDM((char*)sdm_CURRENT,temp);
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

         strcpy(topic,root);
         strcat(topic,"VOLTAGE");
         strcpy(temp,"");
         MC.dSDM.get_paramSDM((char*)sdm_VOLTAGE,temp);
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
         if (debug) journal.jprintf(cStrEnd);  
        }
        #endif
        
       if ((MC.clMQTT.get_MqttFC())&&(MC.dFC.get_present()))   // Отправка данных об инверторе,если он есть
        {
         _delay(100);// пауза перед отправкой следующего пакета - разгружаем сервер и балансируем загрузку у себя
         if (debug) journal.jprintf("FC data:");  
         strcpy(topic,root);
         strcat(topic,"powerFC");
         strcpy(temp,"");
         MC.dFC.get_paramFC((char*)fc_cPOWER,temp);
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
         
         strcpy(topic,root);
         strcat(topic,"freqFC");
         strcpy(temp,"");
         MC.dFC.get_paramFC((char*)fc_cFC,temp);
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;

         strcpy(topic,root);
         strcat(topic,"currentFC");
         ftoa(temp,(float)MC.dFC.get_current()/100.0,1);
         if (MC.clMQTT.sendTopic(topic,temp,false,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
         if (debug) journal.jprintf(cStrEnd); 
        }
          
        if (MC.clMQTT.get_MqttCOP())   // Отправка данных об СОР
          {
          _delay(100);// пауза перед отправкой следующего пакета - разгружаем сервер и балансируем загрузку у себя
          if (debug) journal.jprintf("COP data:");     
          #ifdef USE_ELECTROMETER_SDM
           strcpy(topic,root);
           strcat(topic,"fullCOP");
           ftoa(temp,(float)MC.fullCOP/100.0,2);
           if (MC.clMQTT.sendTopic(topic,temp,false,debug,false)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
          #endif 
           if (MC.dFC.get_present())
           {
           strcpy(topic,root);
           strcat(topic,"COP");
           ftoa(temp,(float)MC.COP/100.0,2);
           if (MC.clMQTT.sendTopic(topic,temp,false,debug,true)) {if (debug) journal.jprintf((char*)MQTTDebugStr, topic,temp);} else return false;
           }        
           if (debug) journal.jprintf(cStrEnd);   
          }  
        if (!debug) journal.jprintf((char*)MQTTPublishOK);   
  return true;
}
 
// Отослать данные на ThingSpeak отсылка идет одним пакетом
// debug  true - выводить в консоль информацию о посылаемых данных false - нет вывода
// возвращает true если удачно
// При удачной отправке Socket# 7 Sn_MR:1 Sn_CR:0 Sn_IR:7 Sn_SR:0 Sn_PORT:401 Sn_DPORT:75b Sn_TOS:0 Sn_TTL:80 Sn_IMR:ff Sn_FRAG:4000 Sn_KPALVTR:0
// При неудачной
boolean  sendThingSpeak(boolean debug)
{
 //uint8_t i;
      // Готовим данные
     strcpy(root,"channels/");  // Название топика едино посылаем одним запросом
     MC.clMQTT.get_paramMQTT((char*)mqtt_LOGIN_MQTT,root);
     strcat(root,"/publish/");
     MC.clMQTT.get_paramMQTT((char*)mqtt_PASSWORD_MQTT,root);
    
     // Формируем данные и посылаем данные  сразу на много полей
     strcpy(topic,"field1=");
     strcat(topic,ftoa(temp,(float)MC.sTemp[TOUT].get_Temp()/100.0,1));
 
     strcat(topic,"&field2=");
     strcat(topic,ftoa(temp,(float)MC.sTemp[TIN].get_Temp()/100.0,1));
 
     strcat(topic,"&field3=");
     strcat(topic,ftoa(temp,(float)MC.sTemp[TBOILER].get_Temp()/100.0,1));

     strcat(topic,"&field4=");
     if(MC.dFC.get_present())
         {
          strcat(topic,ftoa(temp,(float)MC.dFC.get_frequency()/100.0,1));
         }
     else
         {
         if (MC.dRelay[RCOMP].get_Relay()) strcat(topic,cOne);
         else                              strcat(topic,cZero);  
         }
          
     strcat(topic,"&field5=");
     strcat(topic,ftoa(temp,(float)MC.sTemp[TCOMP].get_Temp()/100.0,1));
         
     strcat(topic,"&field6=");
     #ifdef EEV_DEF 
     strcat(topic,ftoa(temp,(float)MC.dEEV.get_Overheat()/100.0,1));
     #else
     if(MC.sTemp[TEVAOUTG].get_present()) strcat(topic,ftoa(temp,(float)MC.sTemp[TEVAOUTG].get_Temp()/100.0,1));
     else   strcat(topic,cZero);
     #endif
      
     strcat(topic,"&field7=");
     #ifdef EEV_DEF
     _itoa(MC.dEEV.get_EEV(),topic);
     #else  // Вместо положения ЭРВ посылаем тепературу гликоля
     if(MC.sTemp[TEVAING].get_present()) strcat(topic,ftoa(temp,(float)MC.sTemp[TEVAING].get_Temp()/100.0,1));
     else   strcat(topic,cZero);
     #endif
     
     strcat(topic,"&field8=");
     _itoa(MC.get_errcode(),topic);
     strcat(topic,(char*)"&status=MQTTPUBLISH");
     // Проверка на длины
     if((strlen(root)>=sizeof(root)-2)||(strlen(topic)>sizeof(topic)-2)) { journal.jprintf("$WARNING: Long topic or data string, is problem.\n"); return false;}
     // Отправка
     journal.jprintf((char*)MQTTpublish,MC.clMQTT.get_mqtt_server());
     
     if (MC.clMQTT.sendTopic(root,topic,false,debug,true)) { if (debug) journal.jprintf(" %s %s\n", root,topic);else journal.jprintf((char*)MQTTPublishOK);return true;} else return false;
     return true;
}
  
// получить  состояние MQTT клиента в виде строки
char *get_stateMQTT()
{
    switch (w5200_MQTT.state())
        {
        case MQTT_CONNECTION_TIMEOUT      : return (char*)"MQTT_CONNECTION_TIMEOUT";       break;
        case MQTT_CONNECTION_LOST         : return (char*)"MQTT_CONNECTION_LOST";          break;
        case MQTT_CONNECT_FAILED          : return (char*)"MQTT_CONNECT_FAILED";           break;
        case MQTT_DISCONNECTED            : return (char*)"MQTT_DISCONNECTED";             break;
        case MQTT_CONNECTED               : return (char*)"MQTT_CONNECTED";                break;
        case MQTT_CONNECT_BAD_PROTOCOL    : return (char*)"MQTT_CONNECT_BAD_PROTOCOL";     break;
        case MQTT_CONNECT_BAD_CLIENT_ID   : return (char*)"MQTT_CONNECT_BAD_CLIENT_ID";    break;
        case MQTT_CONNECT_UNAVAILABLE     : return (char*)"MQTT_CONNECT_UNAVAILABLE";      break;
        case MQTT_CONNECT_BAD_NAME_OR_PASS: return (char*)"MQTT_CONNECT_BAD_NAME_OR_PASS"; break;
        case MQTT_CONNECT_UNAUTHORIZED    : return (char*)"MQTT_CONNECT_UNAUTHORIZED";     break;
        default                           : return (char*)"MQTT_UNKNOW_ERROR";             break;
        }
return (char*)cError;  
}
#endif
