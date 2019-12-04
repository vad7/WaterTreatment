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
// Герерация различных файлов для выгрузки
#include "MainClass.h"
#include "NoSDpage.h"
#define STR_END   strcat(Socket[thread].outBuf,"\r\n")      // Макрос на конец строки
extern uint16_t sendPacketRTOS(uint8_t thread, const uint8_t * buf, uint16_t len,uint16_t pause);

// Генерация заголовка
void get_Header(uint8_t thread,char *name_file)
{
  // journal.jprintf("$Generate file %s\n",name_file);
    strcpy(Socket[thread].outBuf, WEB_HEADER_OK_CT);
    strcat(Socket[thread].outBuf, WEB_HEADER_TEXT_ATTACH);
    strcat(Socket[thread].outBuf, name_file);
    strcat(Socket[thread].outBuf, "\"");
    strcat(Socket[thread].outBuf, WEB_HEADER_END);
	sendPacketRTOS(thread, (byte*)Socket[thread].outBuf, strlen(Socket[thread].outBuf), 0);
	sendPrintfRTOS(thread, " ------ Контроллер водоподоготовки ver. %s  сборка %s %s ------\r\nКонфигурация: %s: %s\r\nСоздание файла: %s %s \r\n\r\n", VERSION,__DATE__,__TIME__,CONFIG_NAME,CONFIG_NOTE,NowTimeToStr(),NowDateToStr());
}

// Получить состояние
// header - заголовок файла ставить или нет
void get_txtState(uint8_t thread, boolean header)
{   uint8_t i,j; 
    int16_t x; 
     if (header) get_Header(thread,(char*)"state.txt");
     sendPrintfRTOS(thread, "\n  1. Водоподготовка\r\nПоследняя ошибка: %d - %s\r\n", MC.get_errcode(), MC.get_lastErr());
     
     strcat(Socket[thread].outBuf,"\r\n\r\n  2. Датчики температуры\r\n");
#if	TNUMBER > 0
     for(i=0;i<TNUMBER;i++)   // Информация по  датчикам температуры
         {
              strcat(Socket[thread].outBuf,MC.sTemp[i].get_name()); if((x=8-strlen(MC.sTemp[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
              strcat(Socket[thread].outBuf,"["); strcat(Socket[thread].outBuf,addressToHex(MC.sTemp[i].get_address())); strcat(Socket[thread].outBuf,"] ");
              strcat(Socket[thread].outBuf,MC.sTemp[i].get_note());  strcat(Socket[thread].outBuf,": ");
              if (MC.sTemp[i].get_present())
                {
                  _ftoa(Socket[thread].outBuf,(float)MC.sTemp[i].get_Temp()/100.0f,2);
                  if (MC.sTemp[i].get_lastErr()!=OK) { strcat(Socket[thread].outBuf," error:"); _itoa(MC.sTemp[i].get_lastErr(),Socket[thread].outBuf); }
                  STR_END; 
                }
                else strcat(Socket[thread].outBuf," absent\r\n"); 
         }
      sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));
#endif
      
      strcpy(Socket[thread].outBuf,"\n  3. Аналоговые датчики\r\n"); // Новый пакет
      for(i=0;i<ANUMBER;i++)   // Информация по  датчикам температуры
         {   
            strcat(Socket[thread].outBuf,MC.sADC[i].get_name()); if((x=8-strlen(MC.sADC[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.sADC[i].get_note());  strcat(Socket[thread].outBuf,": ");
            if (MC.sADC[i].get_present())
                { 
                  _ftoa(Socket[thread].outBuf,(float)MC.sADC[i].get_Press()/100.0f,2); if (MC.sADC[i].get_lastErr()!=OK ) { strcat(Socket[thread].outBuf," error:"); _itoa(MC.sADC[i].get_lastErr(),Socket[thread].outBuf); }
                  STR_END; 
      
                } 
            else strcat(Socket[thread].outBuf," absent\r\n"); 
         }
   
       strcat(Socket[thread].outBuf,"\n  4. Датчики 'сухой контакт'\r\n");
       for(i=0;i<INUMBER;i++)  
           {
            strcat(Socket[thread].outBuf,MC.sInput[i].get_name()); if((x=8-strlen(MC.sInput[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.sInput[i].get_note());  strcat(Socket[thread].outBuf,": ");
      
            if (MC.sInput[i].get_present())
                { 
                  _itoa(MC.sInput[i].get_Input(),Socket[thread].outBuf);
                  strcat(Socket[thread].outBuf," alarm:"); _itoa(MC.sInput[i].get_alarmInput(),Socket[thread].outBuf); STR_END;
                }
                else strcat(Socket[thread].outBuf," absent\r\n");      
           }
       
              
       strcat(Socket[thread].outBuf,"\n  5. Реле\r\n");
       for(i=0;i<RNUMBER;i++)  
           {
            strcat(Socket[thread].outBuf,MC.dRelay[i].get_name()); if((x=8-strlen(MC.dRelay[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.dRelay[i].get_note());  strcat(Socket[thread].outBuf,": ");
      
            if (MC.dRelay[i].get_present()) _itoa(MC.dRelay[i].get_Relay(),Socket[thread].outBuf);
            else strcat(Socket[thread].outBuf," absent"); 
            STR_END;         
           }
         
        strcat(Socket[thread].outBuf,"\n  6. Электросчетчик\r\n");
           strcat(Socket[thread].outBuf,"Текущее входное напряжение [В]: ");                          MC.dPWM.get_param((char*)pwm_VOLTAGE,Socket[thread].outBuf); STR_END;
           strcat(Socket[thread].outBuf,"Текущий потребляемый ток [А]: ");                         MC.dPWM.get_param((char*)pwm_CURRENT,Socket[thread].outBuf); STR_END;
           strcat(Socket[thread].outBuf,"Текущая потребляемая мощность [Вт]: ");        MC.dPWM.get_param((char*)pwm_POWER,Socket[thread].outBuf); STR_END;
           strcat(Socket[thread].outBuf,"Коэффициент мощности: ");                                    MC.dPWM.get_param((char*)pwm_PFACTOR,Socket[thread].outBuf); STR_END;
           strcat(Socket[thread].outBuf,"Суммарная активная энергия [кВт/ч]: ");                     MC.dPWM.get_param((char*)pwm_ACENERGY,Socket[thread].outBuf); STR_END;
   
   strcat(Socket[thread].outBuf,"\n  7. Частотные датчики потока\r\n");
     for(i=0;i<FNUMBER;i++)  
           {
            strcat(Socket[thread].outBuf,MC.sFrequency[i].get_name()); if((x=8-strlen(MC.sFrequency[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.sFrequency[i].get_note());  strcat(Socket[thread].outBuf,": ");
            if (MC.sFrequency[i].get_present()) _ftoa(Socket[thread].outBuf,(float)MC.sFrequency[i].get_Value()/1000.0f,3);
            else strcat(Socket[thread].outBuf," absent"); 
            STR_END;         
           }
   
        
  sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));   
}


// Получить настройки в текстовом виде
void get_txtSettings(uint8_t thread)
{
     uint8_t i,j;
     int16_t x; 
     
     get_Header(thread,(char*)"settings.txt");
     sendPrintfRTOS(thread, "Последняя ошибка: %d - %s\r\n", MC.get_errcode(),MC.get_lastErr());
     strcpy(Socket[thread].outBuf,"\n  1. Общие настройки\r\n");
     
     strcpy(Socket[thread].outBuf,"\n  1.1 Опции\r\n");
    
     strcat(Socket[thread].outBuf," - Настройка встроенных графиков -\r\n");
     strcat(Socket[thread].outBuf,"Период накопления графиков (сек): "); MC.get_option((char*)option_TIME_CHART,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Запись графиков на карту памяти: "); MC.get_option((char*)option_History,Socket[thread].outBuf);STR_END;

     strcat(Socket[thread].outBuf," - Дополнительное оборудование -\r\n");
     strcat(Socket[thread].outBuf,"Логировать обмен между беспроводными датчиками: "); MC.get_option((char*)option_LogWirelessSensors,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Генерация звукового сигнала: "); MC.get_option((char*)option_BEEP,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"На шинах 1-Wire(DS2482) только один датчик: "); if((MC.get_flags() & (1<<f1Wire2TSngl))) strcat(Socket[thread].outBuf, "2 "); if((MC.get_flags() & (1<<f1Wire3TSngl))) strcat(Socket[thread].outBuf, "3 "); if((MC.get_flags() & (1<<f1Wire4TSngl))) strcat(Socket[thread].outBuf, "4");
     sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));  
    
     strcpy(Socket[thread].outBuf,"\n\n  1.2 Сетевые настройки\r\n");
     strcat(Socket[thread].outBuf,"Использование DHCP: "); MC.get_network((char*)net_DHCP,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"IP адрес контроллера: "); MC.get_network((char*)net_IP,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"DNS сервер: "); MC.get_network((char*)net_DNS,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Шлюз: "); MC.get_network((char*)net_GATEWAY,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Маска подсети: "); MC.get_network((char*)net_SUBNET,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"MAC адрес контроллера: "); MC.get_network((char*)net_MAC,Socket[thread].outBuf);STR_END;

     strcat(Socket[thread].outBuf,"Запрет пинга контроллера: "); MC.get_network((char*)net_NO_PING,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Использование паролей: ");   MC.get_network((char*)net_PASS,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Пароль пользователя (user): "); MC.get_network((char*)net_PASSUSER,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Пароль администратора (admin): "); MC.get_network((char*)net_PASSADMIN,Socket[thread].outBuf); STR_END;

     strcat(Socket[thread].outBuf,"Проверка ping до сервера: "); MC.get_network((char*)net_PING_TIME,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Адрес пингуемого сервера: "); MC.get_network((char*)net_PING_ADR,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Ежеминутный контроль связи с Wiznet W5xxx: "); MC.get_network((char*)net_INIT_W5200,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Время очистки сокетов: "); MC.get_network((char*)net_RES_SOCKET,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Время сброса чипа: "); strcat(Socket[thread].outBuf,nameWiznet);strcat(Socket[thread].outBuf,": "); MC.get_network((char*)net_RES_W5200,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Размер пакета для отправки в байтах: "); MC.get_network((char*)net_SIZE_PACKET,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Не ожидать получения ACK при посылке следующего пакета: "); MC.get_network((char*)net_NO_ACK,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Пауза перед отправкой следующего пакета, если нет ожидания ACK. (мсек): "); MC.get_network((char*)net_DELAY_ACK,Socket[thread].outBuf);STR_END;

     
     strcat(Socket[thread].outBuf,"\n  1.3 Настройки даты и времени\r\n");
     strcat(Socket[thread].outBuf,"Установленная дата: "); MC.get_datetime((char*)time_DATE,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Установленное время: "); MC.get_datetime((char*)time_TIME,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Адрес NTP сервера: "); MC.get_datetime((char*)time_NTP,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Часовой пояс (часы): "); MC.get_datetime((char*)time_TIMEZONE,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Синхронизация времени по NTP раз в сутки: "); MC.get_datetime((char*)time_UPDATE,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Синхронизация раз в час с I2C часами: "); MC.get_datetime((char*)time_UPDATE_I2C,Socket[thread].outBuf);STR_END;
  
     sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));  
      
     strcpy(Socket[thread].outBuf,"\n  1.4 Уведомления\r\n");
     strcat(Socket[thread].outBuf,"Сброс контроллера: "); MC.message.get_messageSetting((char*)mess_MESS_RESET,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Возникновение ошибок: "); MC.message.get_messageSetting((char*)mess_MESS_ERROR,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Сигнал «жизни» (ежедневно в 12-00): "); MC.message.get_messageSetting((char*)mess_MESS_LIFE,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Достижение граничной температуры: "); MC.message.get_messageSetting((char*)mess_MESS_TEMP,Socket[thread].outBuf);  STR_END;
     strcat(Socket[thread].outBuf," Граничная температура в доме (если меньше то посылается уведомление): "); MC.message.get_messageSetting((char*)mess_MESS_TIN,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf," Граничная температура бойлера (если меньше то посылается уведомление): "); MC.message.get_messageSetting((char*)mess_MESS_TBOILER,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf," Граничная температура компрессора (если больше то посылается уведомление): "); MC.message.get_messageSetting((char*)mess_MESS_TCOMP,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Проблемы с SD картой и SPI flash: "); MC.message.get_messageSetting((char*)mess_MESS_SD,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Прочие уведомления: "); MC.message.get_messageSetting((char*)mess_MESS_WARNING,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf," - Настройка отправки почты -\r\n");
     strcat(Socket[thread].outBuf,"Посылать уведомления по почте: "); MC.message.get_messageSetting((char*)mess_MAIL,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Адрес smtp сервера: "); MC.message.get_messageSetting((char*)mess_SMTP_SERVER,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Порт smtp сервера: "); MC.message.get_messageSetting((char*)mess_SMTP_PORT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Необходимость авторизации на сервере: ");MC.message.get_messageSetting((char*)mess_MAIL_AUTH,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Логин для входа: "); MC.message.get_messageSetting((char*)mess_SMTP_LOGIN,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Пароль для входа: "); MC.message.get_messageSetting((char*)mess_SMTP_PASS,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Адрес отправителя: "); MC.message.get_messageSetting((char*)mess_SMTP_MAILTO,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Адрес получателя: "); MC.message.get_messageSetting((char*)mess_SMTP_RCPTTO,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Добавлять в уведомление информацию о состоянии: "); MC.message.get_messageSetting((char*)mess_MAIL_INFO,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf," - Настройка отправки SMS -\r\n");
     strcat(Socket[thread].outBuf,"Посылать уведомления по SMS: "); MC.message.get_messageSetting((char*)mess_SMS,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Телефон получателя: "); MC.message.get_messageSetting((char*)mess_SMS_PHONE,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Сервис отправки SMS: ");MC.message.get_messageSetting((char*)mess_SMS_SERVICE,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Адрес сервиса отправки SMS: ");MC.message.get_messageSetting((char*)mess_SMS_IP,Socket[thread].outBuf);STR_END;
     MC.message.get_messageSetting((char*)mess_SMS_NAMEP1,Socket[thread].outBuf);strcat(Socket[thread].outBuf,": ");MC.message.get_messageSetting((char*)mess_SMS_P1,Socket[thread].outBuf);STR_END;
     MC.message.get_messageSetting((char*)mess_SMS_NAMEP2,Socket[thread].outBuf);strcat(Socket[thread].outBuf,": ");MC.message.get_messageSetting((char*)mess_SMS_P2,Socket[thread].outBuf);STR_END;
     sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf)); 
      
      // MQTT
     strcpy(Socket[thread].outBuf,"\n  1.5 Настройка MQTT\r\n");
      #ifdef MQTT
     strcat(Socket[thread].outBuf,"Включить отправку на сервер MQTT: ");MC.clMQTT.get_paramMQTT((char*)mqtt_USE_MQTT,Socket[thread].outBuf);  STR_END;
     strcat(Socket[thread].outBuf,"Отправка на сервер ThingSpeak: "); MC.clMQTT.get_paramMQTT((char*)mqtt_USE_TS,Socket[thread].outBuf);  STR_END;
     strcat(Socket[thread].outBuf,"Включить отсылку дополнительных данных: "); MC.clMQTT.get_paramMQTT((char*)mqtt_BIG_MQTT,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Интервал передачи данных [1...1000] (минут): "); MC.clMQTT.get_paramMQTT((char*)mqtt_TIME_MQTT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Адрес MQTT сервера: ");MC.clMQTT.get_paramMQTT((char*)mqtt_ADR_MQTT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Порт MQTT сервера: "); MC.clMQTT.get_paramMQTT((char*)mqtt_PORT_MQTT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Логин для входа: "); MC.clMQTT.get_paramMQTT((char*)mqtt_LOGIN_MQTT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Пароль для входа: "); MC.clMQTT.get_paramMQTT((char*)mqtt_PASSWORD_MQTT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Идентификатор клиента: "); MC.clMQTT.get_paramMQTT((char*)mqtt_ID_MQTT,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf," - Сервис <Народный мониторинг> -\r\n");
     strcat(Socket[thread].outBuf,"Включить передачу данных: "); MC.clMQTT.get_paramMQTT((char*)mqtt_USE_NARMON,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Посылать расширенный набор данных: "); MC.clMQTT.get_paramMQTT((char*)mqtt_BIG_NARMON,Socket[thread].outBuf); STR_END;
     strcat(Socket[thread].outBuf,"Адрес сервиса: ");MC.clMQTT.get_paramMQTT((char*)mqtt_ADR_NARMON,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Порт сервиса: "); MC.clMQTT.get_paramMQTT((char*)mqtt_PORT_NARMON,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Логин для входа (получается при регистрации): "); MC.clMQTT.get_paramMQTT((char*)mqtt_LOGIN_NARMON,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Личный код для передачи (смотреть в разделе API MQTT сервиса): "); MC.clMQTT.get_paramMQTT((char*)mqtt_PASSWORD_NARMON,Socket[thread].outBuf);STR_END;
     strcat(Socket[thread].outBuf,"Имя устройства (корень всех топиков): "); MC.clMQTT.get_paramMQTT((char*)mqtt_ID_NARMON,Socket[thread].outBuf);STR_END;
     #else
      strcat(Socket[thread].outBuf,"Не поддерживается, нет в прошивке");
     #endif
     
     strcat(Socket[thread].outBuf,"\n  2.1 Датчики температуры\r\n");
#if	TNUMBER > 0
     for(i=0;i<TNUMBER;i++)   // Информация по  датчикам температуры
         {
              strcat(Socket[thread].outBuf,MC.sTemp[i].get_name()); if((x=8-strlen(MC.sTemp[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
              strcat(Socket[thread].outBuf,"["); strcat(Socket[thread].outBuf,addressToHex(MC.sTemp[i].get_address())); strcat(Socket[thread].outBuf,"] ");
              strcat(Socket[thread].outBuf,MC.sTemp[i].get_note());  strcat(Socket[thread].outBuf,": ");
        
              if (MC.sTemp[i].get_present())
                {
                  strcat(Socket[thread].outBuf," T=");      _ftoa(Socket[thread].outBuf,(float)MC.sTemp[i].get_Temp()/100.0f,2);
                  strcat(Socket[thread].outBuf,", Tmin=");  _ftoa(Socket[thread].outBuf,(float)MC.sTemp[i].get_minTemp()/100.0f,2);
                  strcat(Socket[thread].outBuf,", Tmax=");  _ftoa(Socket[thread].outBuf,(float)MC.sTemp[i].get_maxTemp()/100.0f,2);
                  strcat(Socket[thread].outBuf,", Terr=");  _ftoa(Socket[thread].outBuf,(float)MC.sTemp[i].get_errTemp()/100.0f,2);
                  strcat(Socket[thread].outBuf,", Ttest="); _ftoa(Socket[thread].outBuf,(float)MC.sTemp[i].get_testTemp()/100.0f,2);
                  if (MC.sTemp[i].get_lastErr()!=OK) { strcat(Socket[thread].outBuf," error:"); _itoa(MC.sTemp[i].get_lastErr(),Socket[thread].outBuf); }  STR_END;
                }
                else strcat(Socket[thread].outBuf," absent \r\n"); 
         }   

#endif
     sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));   
          
      strcpy(Socket[thread].outBuf,"\n  2.2 Аналоговые датчики\r\n");
      for(i=0;i<ANUMBER;i++)   // Информация по  аналоговым датчикм - например давление
         {   
            strcat(Socket[thread].outBuf,MC.sADC[i].get_name()); if((x=8-strlen(MC.sADC[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.sADC[i].get_note());  strcat(Socket[thread].outBuf,": ");
            if (MC.sADC[i].get_present())
                { 
                  strcat(Socket[thread].outBuf," P=");    _ftoa(Socket[thread].outBuf,(float)MC.sADC[i].get_Press()/100.0f,2);
                  strcat(Socket[thread].outBuf," Pmin="); _ftoa(Socket[thread].outBuf,(float)MC.sADC[i].get_minPress()/100.0f,2);
                  strcat(Socket[thread].outBuf," Pmax="); _ftoa(Socket[thread].outBuf,(float)MC.sADC[i].get_maxPress()/100.0f,2);
                  strcat(Socket[thread].outBuf," Ptest=");_ftoa(Socket[thread].outBuf,(float)MC.sADC[i].get_testPress()/100.0f,2);
                  strcat(Socket[thread].outBuf," Zero="); _itoa(MC.sADC[i].get_zeroPress(),Socket[thread].outBuf);
                  strcat(Socket[thread].outBuf," Kof=");  _ftoa(Socket[thread].outBuf,(float)MC.sADC[i].get_transADC(),3);
                  strcat(Socket[thread].outBuf," Pin=AD");_itoa(MC.sADC[i].get_pinA(),Socket[thread].outBuf);
                  if (MC.sADC[i].get_lastErr()!=OK ) { strcat(Socket[thread].outBuf," error:"); _itoa(MC.sADC[i].get_lastErr(),Socket[thread].outBuf); }  STR_END;
                } 
            else strcat(Socket[thread].outBuf," absent \r\n"); 
         }  
         
  
       strcat(Socket[thread].outBuf,"\n  2.3 Датчики сухой контакт\r\n");
       for(i=0;i<INUMBER;i++)  
           {
            strcat(Socket[thread].outBuf,MC.sInput[i].get_name()); if((x=8-strlen(MC.sInput[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.sInput[i].get_note());  strcat(Socket[thread].outBuf,": ");
      
            if (MC.sInput[i].get_present())
                { 
                 strcat(Socket[thread].outBuf," In=");       _itoa(MC.sInput[i].get_Input(),Socket[thread].outBuf);
                 strcat(Socket[thread].outBuf," Alarm=");    _itoa(MC.sInput[i].get_alarmInput(),Socket[thread].outBuf);
                 strcat(Socket[thread].outBuf," Test=");     _itoa(MC.sInput[i].get_testInput(),Socket[thread].outBuf);
                 strcat(Socket[thread].outBuf," Pin=");      _itoa(MC.sInput[i].get_pinD(),Socket[thread].outBuf);
                 strcat(Socket[thread].outBuf," Type=");
                 strcat(Socket[thread].outBuf, MC.sInput[i].get_alarm_error() == 0 ? "" : "Alarm");
                 if(MC.sInput[i].get_lastErr()!=OK) { strcat(Socket[thread].outBuf," error:"); _itoa(MC.sInput[i].get_lastErr(),Socket[thread].outBuf);}   STR_END;
                }
                else strcat(Socket[thread].outBuf," absent \r\n");      
           } 

     sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));   
          
        strcpy(Socket[thread].outBuf,"\n  3.1 Реле\r\n");
        for(i=0;i<RNUMBER;i++)  
           {
            strcat(Socket[thread].outBuf,MC.dRelay[i].get_name()); if((x=8-strlen(MC.dRelay[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.dRelay[i].get_note());
            strcat(Socket[thread].outBuf," Pin:"); _itoa(MC.dRelay[i].get_pinD(),Socket[thread].outBuf); strcat(Socket[thread].outBuf," Status: ");
      
            if (MC.dRelay[i].get_present()) { _itoa(MC.dRelay[i].get_Relay(),Socket[thread].outBuf); STR_END;}
            else strcat(Socket[thread].outBuf," absent \r\n");      
           }       

       sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));   
        

        strcat(Socket[thread].outBuf,"\n  3.5. Частотные датчики потока\r\n");
        for(i=0;i<FNUMBER;i++)  
           {
            strcat(Socket[thread].outBuf,MC.sFrequency[i].get_name()); if((x=8-strlen(MC.sFrequency[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(Socket[thread].outBuf," "); }
            strcat(Socket[thread].outBuf,MC.sFrequency[i].get_note());  strcat(Socket[thread].outBuf,": ");
            if (MC.sFrequency[i].get_present())
            {
             strcat(Socket[thread].outBuf," Контроль мин. потока: ");  if (MC.sFrequency[i].get_checkFlow())  strcat(Socket[thread].outBuf,(char*)cOne);else strcat(Socket[thread].outBuf,(char*)cZero);
             strcat(Socket[thread].outBuf,", Минимальный поток (куб/ч)="); _ftoa(Socket[thread].outBuf,(float)MC.sFrequency[i].get_minValue()/1000.0f,3);
             strcat(Socket[thread].outBuf,", Коэффициент (имп*л)=");       _ftoa(Socket[thread].outBuf,(float)MC.sFrequency[i].get_kfValue()/100.0f,3);
             strcat(Socket[thread].outBuf,", Тест (куб/ч)="); _ftoa(Socket[thread].outBuf,(float)MC.sFrequency[i].get_testValue()/1000.0f,3);
             
            }
            else strcat(Socket[thread].outBuf," absent"); 
            STR_END;         
           }

     sendBufferRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf));   

}

// Записать в клиента бинарный файл настроек
uint16_t get_binSettings(uint8_t thread)
{
	uint16_t i, len;
	byte b;
	// 1. Заголовок
    strcpy(Socket[thread].outBuf, WEB_HEADER_OK_CT);
    strcat(Socket[thread].outBuf, WEB_HEADER_BIN_ATTACH);
    strcat(Socket[thread].outBuf, "settings.bin\"\r\n\r\n");
	sendPacketRTOS(thread, (byte*)Socket[thread].outBuf, strlen(Socket[thread].outBuf), 0);
	sendConstRTOS(thread, HEADER_BIN);
	
	// 2. Запись настроек
	if((len = MC.save())<= 0) return 0; // записать настройки в еепром, а потом будем их писать и получить размер записываемых данных
	for(i = 0; i < len; i++) {  // Копирование в буффер
		readEEPROM_I2C(I2C_SETTING_EEPROM + i, &b, 1);
		Socket[thread].outBuf[i] = b;
	}
	if(sendPacketRTOS(thread, (byte*) Socket[thread].outBuf, len, 0) == 0) return 0; // передать пакет, при ошибке выйти

	return len;
}

/*
// файл статистики на карте отсутсвует 
void noCsvStatistic(uint8_t thread)
{
   get_Header(thread,(char*)FILE_STATISTIC);
   sendPrintfRTOS(thread, "Файл статистики за выбранный год не найден на карте памяти.\r\n");
}
*/ 
   
// Получить индексный файл при отсутвии SD карты
// выдача массива index_noSD в index_noSD
int16_t get_indexNoSD(uint8_t thread)
{
    uint16_t n,i=0;
     #ifdef LOG 
       journal.jprintf("$Read: indexNoSD from memory\n");  
     #endif
   	strcpy(Socket[thread].outBuf, WEB_HEADER_OK_CT);
   	strcat(Socket[thread].outBuf, WEB_HEADER_TXT_KEEP);
   	strcat(Socket[thread].outBuf, WEB_HEADER_END);
   	sendPacketRTOS(thread, (byte*)Socket[thread].outBuf, strlen(Socket[thread].outBuf), 0);
    n=sizeof(index_noSD);              // сколько надо передать байт
    while (n>0)                        // Пока есть не отправленные данные
      {
       if(n>=W5200_MAX_LEN)
           {
            memcpy(Socket[thread].outBuf,index_noSD+i,W5200_MAX_LEN);
            if(sendPacketRTOS(thread,(byte*)Socket[thread].outBuf,W5200_MAX_LEN,0)==0) return 0 ;                      // не последний пакет
            i=i+W5200_MAX_LEN;n=n-W5200_MAX_LEN;
           }
       else 
           {
            memcpy(Socket[thread].outBuf,(index_noSD+i),n);
            if(sendPacketRTOS(thread,(byte*)Socket[thread].outBuf,n,0)==0) return 0; else return sizeof(index_noSD);  // последний пакет
           }
     } // while (n>0)
  return n;
}   

// Выдать файл журнала в виде файла
void get_txtJournal(uint8_t thread)
{
   get_Header(thread,(char*)"journal.txt");
   sendPrintfRTOS(thread, "  Системный журнал (размер %d из %d байт)\r\n", journal.available(),JOURNAL_LEN);
   STORE_DEBUG_INFO(17);
   journal.send_Data(thread);
}

// Сгенерировать тестовый файл
#define SIZE_TEST 2*1024*1024  // размер теста в байтах, реально передается округление до целого числа sizeof(Socket[thread].outBuf)
void get_datTest(uint8_t thread)
{
   uint16_t j;
   unsigned long startTick;
   // Заголовок
   strcpy(Socket[thread].outBuf, WEB_HEADER_OK_CT);
   strcat(Socket[thread].outBuf, "application/x-binary\r\nContent-Length:");
   _itoa((SIZE_TEST/sizeof(Socket[thread].outBuf))*sizeof(Socket[thread].outBuf),Socket[thread].outBuf);  //реальный размер передачи
   strcat(Socket[thread].outBuf,"\r\nContent-Disposition: attachment; filename=\"test.dat\"\r\n\r\n");
   sendPacketRTOS(thread,(byte*)Socket[thread].outBuf,strlen(Socket[thread].outBuf),0);
   
   // Генерация файла используется выходной буфер
   memset(Socket[thread].outBuf,0x55,sizeof(Socket[thread].outBuf));   // Заполнение буфера
   startTick=millis();                                      // Запомнить время старта
   for(j=0;j<SIZE_TEST/sizeof(Socket[thread].outBuf);j++)  
   {
    if (sendBufferRTOS(thread,(byte*)(Socket[thread].outBuf),sizeof(Socket[thread].outBuf))==0) break;
   }
   startTick=millis()-startTick;
   journal.jprintf("Download test.dat speed:%d bytes/sec\n",SIZE_TEST*1000/startTick);

}

// Получить состояние для отсылки по почте
// посылается в клиента используя tempBuf
void get_mailState(EthernetClient client,char *tempBuf)
{
uint8_t i,j;
int16_t x; 

       strcpy(tempBuf,"\r\n  Водоподготовка");
       strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
    
       strcpy(tempBuf,"Состояние: "); MC.StateToStr(tempBuf);
       strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));  
      
       strcpy(tempBuf,"Последняя ошибка: ");  _itoa(MC.get_errcode(),tempBuf); strcat(tempBuf," - "); strcat(tempBuf,MC.get_lastErr());
       strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
   
       strcpy(tempBuf,"Режим работы: ");strcat(tempBuf,MC.TestToStr());
       strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf)); 
           
      strcpy(tempBuf,"Последняя перезагрузка: "); DecodeTimeDate(MC.get_startDT(),tempBuf,3);
      strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));  
     
      strcpy(tempBuf,"Время с последней перезагрузки: "); TimeIntervalToStr(MC.get_uptime(),tempBuf);
      strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));  
     
      strcpy(tempBuf,"Причина последней перезагрузки: "); strcat(tempBuf,ResetCause());
      strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));  

      strcpy(tempBuf,"\n  2. Датчики температуры");
      strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
      for(i=0;i<TNUMBER;i++)   // Информация по  датчикам температуры
         {
            if (MC.sTemp[i].get_present())  // только присутсвующие датчики
            {
              strcpy(tempBuf,MC.sTemp[i].get_name()); if((x=8-strlen(MC.sTemp[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(tempBuf," "); }
              strcat(tempBuf,"["); strcat(tempBuf,addressToHex(MC.sTemp[i].get_address())); strcat(tempBuf,"] ");
              strcat(tempBuf,MC.sTemp[i].get_note());  strcat(tempBuf,": ");
              _ftoa(tempBuf,(float)MC.sTemp[i].get_Temp()/100.0f,2);
              if (MC.sTemp[i].get_lastErr()!=OK) { strcat(tempBuf," error:"); _itoa(MC.sTemp[i].get_lastErr(),tempBuf); }
              strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
            }  //   if (MC.sTemp[i].get_present())
         }
      strcpy(tempBuf,"\n  3. Аналоговые датчики");
      strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
      for(i=0;i<ANUMBER;i++)   // Информация по  аналоговым датчикам
         {   
           if (MC.sADC[i].get_present()) // только присутсвующие датчики
            {          
            strcpy(tempBuf,MC.sADC[i].get_name()); if((x=8-strlen(MC.sADC[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(tempBuf," "); }
            strcat(tempBuf,MC.sADC[i].get_note());  strcat(tempBuf,": ");
            _ftoa(tempBuf,(float)MC.sADC[i].get_Press()/100.0f,2);
            if (MC.sADC[i].get_lastErr()!=OK ) { strcat(tempBuf," error:"); _itoa(MC.sADC[i].get_lastErr(),tempBuf); }
            strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
            }
         }
   
       strcpy(tempBuf,"\n  4. Датчики 'сухой контакт'");
       strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
       for(i=0;i<INUMBER;i++)  
           {
           if (MC.sInput[i].get_present()) // только присутсвующие датчики
              { 
              strcpy(tempBuf,MC.sInput[i].get_name()); if((x=8-strlen(MC.sInput[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(tempBuf," "); }
              strcat(tempBuf,MC.sInput[i].get_note());  strcat(tempBuf,": ");
              _itoa(MC.sInput[i].get_Input(),tempBuf);
              strcat(tempBuf," alarm:"); _itoa(MC.sInput[i].get_alarmInput(),tempBuf);
              strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
              }     
           }

       strcpy(tempBuf,"\n  5. Реле");
       strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf)); 
       for(i=0;i<RNUMBER;i++)  
           {
            if (MC.dRelay[i].get_present()) // только присутсвующие датчики
              {
              strcpy(tempBuf,MC.dRelay[i].get_name()); if((x=8-strlen(MC.dRelay[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(tempBuf," "); }
              strcat(tempBuf,MC.dRelay[i].get_note());  strcat(tempBuf,": ");
              _itoa(MC.dRelay[i].get_Relay(),tempBuf);
              strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf)); 
              }         
           }
       
       strcpy(tempBuf,"\n  6. Электросчетчик"); strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
           strcpy(tempBuf,"Текущее входное напряжение [В]: ");            MC.dPWM.get_param((char*)pwm_VOLTAGE,tempBuf);strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));
           strcpy(tempBuf,"Текущая потребляемая мощность [Вт]: ");        MC.dPWM.get_param((char*)pwm_POWER,tempBuf); strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));

        strcpy(tempBuf,"\n  7. Частотные датчики потока");
        strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf)); 
        for(i=0;i<FNUMBER;i++)  
           {
              if (MC.sFrequency[i].get_present())
                {
                strcpy(tempBuf,MC.sFrequency[i].get_name()); if((x=8-strlen(MC.sFrequency[i].get_name()))>0) { for(j=0;j<x;j++)  strcat(tempBuf," "); }
                strcat(tempBuf,MC.sFrequency[i].get_note());  strcat(tempBuf,": ");
                _ftoa(tempBuf,(float)MC.sFrequency[i].get_Value()/1000.0f,3);
                strcat(tempBuf,cStrEnd);  client.write(tempBuf,strlen(tempBuf));    
                }      
           }        
       
}

