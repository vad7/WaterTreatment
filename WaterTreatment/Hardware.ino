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
// Описание базовых классов для работы c "железом"
// (датчики и исполнительные устройства) зависит от контроллера
// --------------------------------------------------------------------------------
#include "Hardware.h"

// --------------------------------------------------------------------------------
// Настройка таймера и ацп для чтения датчика давления
// Зависит от чипа
// --------------------------------------------------------------------------------
// Старт считывания АЦП
void start_ADC()
{
	adc_setup();                                       // setup ADC
	pmc_enable_periph_clk(TC_INTERFACE_ID + 0 * 3 + 0);    // clock the TC0 channel 0

	TcChannel * t = &(TC0->TC_CHANNEL)[0];              // pointer to TC0 registers for its channel 0
	t->TC_CCR = TC_CCR_CLKDIS;                          // disable internal clocking while setup regs
	t->TC_IDR = 0xFFFFFFFF;                             // disable interrupts
	t->TC_SR;                                           // read int status reg to clear pending
	t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |             // use TCLK1 (prescale by 2, = 42MHz)
			TC_CMR_WAVE |                            // waveform mode
			TC_CMR_WAVSEL_UP_RC |                    // count-up PWM using RC as threshold
			TC_CMR_EEVT_XC0 |                        // Set external events from XC0 (this setup TIOB as output)
			TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET |    // set clear and set from RA and RC compares
			TC_CMR_BCPB_NONE | TC_CMR_BCPC_NONE;

	t->TC_RC = SystemCoreClock / 2 / ADC_FREQ;        // counter resets on RC, so sets period in terms of 42MHz clock
	t->TC_RA = SystemCoreClock / 2 / ADC_FREQ / 2;     // roughly square wave
	t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;  // re-enable local clocking and switch to hardware trigger source.

}

// Установка АЦП
void adc_setup()
{
	uint32_t adcMask = 0;
	uint32_t adcGain = 0x55555555; // All gains set to x1 Channel Gain Register
	uint8_t max = 0;
#ifdef VCC_CONTROL                             // если разрешено чтение напряжение питания
	adcMask |= (1<<PIN_ADC_VCC);               // Добавить маску контроля питания
	max = PIN_ADC_VCC;
#endif
	//   adcMask=adcMask|(0x1u<<ADC_TEMPERATURE_SENSOR);         // добавить маску для внутреннего датчика температуры
	//   adc_enable_ts(ADC);                                     // разрешить чтение температурного датчика в регистре ADC Analog Control Register

	// Расчет маски каналов
	for(uint8_t i = 0; i < ANUMBER; i++) {   // по всем датчикам
		if(MC.sADC[i].get_present() && !MC.sADC[i].get_fmodbus()) {
			uint32_t ach = MC.sADC[i].get_pinA();
			if(max < ach) max = ach;
			adcMask |= 1 << ach;
			adcGain = (adcGain & ~(3 << (2 * ach))) | (MC.sADC[i].get_ADC_Gain() << (2 * ach));
		}
	}
#ifdef TNTC
	for(uint8_t i = 0; i < TNTC; i++) {
		if(max < TADC[i]) max = TADC[i];
		adcMask |= 1 << TADC[i];
	}
#endif

	journal.jprintfopt("adcMask = %X, adcGain = %X\n", adcMask, adcGain);

	NVIC_EnableIRQ(ADC_IRQn);        // enable ADC interrupt vector
	ADC->ADC_IDR = 0xFFFFFFFF;       // disable interrupts IDR Interrupt Disable Register
	ADC->ADC_IER = 1 << max;         // Самый старший канал
	ADC->ADC_CHDR = 0xFFFF;          // Channel Disable Register CHDR disable all channels
	ADC->ADC_CHER = adcMask;         // Channel Enable Register CHER enable just A11  каналы здесь SAMX3!!
	ADC->ADC_CGR = adcGain;          // Gain: 0,1 = x1, 2 = x2, 3 = x4
	ADC->ADC_COR = 0x00000000;       // All offsets off Channel Offset Register
	// 12bit, 14MHz, trig source TIO from TC0
	ADC->ADC_MR = ADC_MR_PRESCAL(ADC_PRESCAL) | ADC_MR_LOWRES_BITS_12 | ADC_MR_USEQ_NUM_ORDER | ADC_MR_STARTUP_SUT16 | ADC_MR_TRACKTIM(16) | ADC_MR_SETTLING_AST17 | ADC_MR_TRANSFER(2) | ADC_MR_TRGSEL_ADC_TRIG1 | ADC_MR_TRGEN | ADC_MR_ANACH;
	adc_set_bias_current(ADC, 0);    // for sampling frequency: 0 - below 500 kHz, 1 - between 500 kHz and 1 MHz.
}


// Обработчик прерывания, как можно короче
#ifdef __cplusplus
extern "C"
{
#endif
void ADC_Handler(void)
{
#ifdef VCC_CONTROL  // если разрешено чтение напряжение питания
	MC.AdcVcc = (uint32_t)(*(ADC->ADC_CDR + PIN_ADC_VCC));
#endif
	for(uint8_t i = 0; i < ANUMBER; i++)    // по всем датчикам
	{
		sensorADC *adc = &MC.sADC[i];
#ifdef ADC_SKIP_EXTREMUM
		int32_t a = ADC->ADC_CDR[adc->get_pinA()]; // get conversion result
		if(adc->adc_lastVal != 0xFFFF && abs(a - adc->adc_lastVal) > ADC_SKIP_EXTREMUM) {
			adc->adc_lastVal = 0xFFFF;
			continue;
		}
		adc->adc_lastVal = a;
#else
		adc->adc_lastVal = (uint32_t)ADC->ADC_CDR[adc->get_pinA()];  // get conversion result
#endif
		// Усреднение значений
		adc->adc_sum = adc->adc_sum + adc->adc_lastVal - adc->adc_filter[adc->adc_last];   // Добавить новое значение, Убрать самое старое значение
		adc->adc_filter[adc->adc_last] = adc->adc_lastVal;			                       // Запомнить новое значение
		if(adc->adc_last < adc->adc_filter_max) adc->adc_last++;
		else {
			adc->adc_last = 0;
			adc->adc_flagFull = true;
		}
	}
#ifdef TNTC
	for(uint8_t i = 0; i < TNTC; i++) TNTC_Value[i] = ADC->ADC_CDR[TADC[i]]; // get conversion result
#endif
	// if (ADC->ADC_ISR & (1<<ADC_TEMPERATURE_SENSOR))   // ensure there was an End-of-Conversion and we read the ISR reg
	//            MC.AdcTempSAM3x =(unsigned int)(*(ADC->ADC_CDR+ADC_TEMPERATURE_SENSOR));   // если готов прочитать результат
	ADC_has_been_read = true;
}

#ifdef __cplusplus
}
#endif
    
// ------------------------------------------------------------------------------------------
// Аналоговые датчики давления --------------------------------------------------------------
// Давление хранится в СОТЫХ БАР
void sensorADC::initSensorADC(uint8_t sensor, uint8_t pinA, uint16_t filter_size)
{
	// Инициализация структуры для хранения "сырых"данных с аналогового датчика.
	if(SENSORPRESS[sensor]) adc_filter_max = filter_size;   // отводим память если используем датчик под сырые данные
	else adc_filter_max = 1;
	adc_filter = (uint16_t*) malloc(sizeof(uint16_t) * adc_filter_max);
	if(adc_filter == NULL) {   // ОШИБКА если память не выделена
		set_Error(ERR_OUT_OF_MEMORY, (char*) "sensorADC");
		return;
	}
	memset(adc_filter, 0, sizeof(uint16_t) * adc_filter_max);
	adc_filter_max--;
	adc_sum = 0;                                                                   // сумма
	adc_last = 0;                                                                  // текущий индекс
	adc_flagFull = false;                                                          // буфер полный
	adc_lastVal = 0;                                                               // последнее считанное значение
	clearBuffer();

	testMode = NORMAL;                           // Значение режима тестирования
	cfg.minValue = MINPRESS[sensor];                 // минимально разрешенное давление
	cfg.maxValue = MAXPRESS[sensor];                 // максимально разрешенное давление
	cfg.testValue = TESTPRESS[sensor];               // Значение при тестировании
	cfg.zeroValue = ZEROPRESS[sensor];               // отсчеты АЦП при нуле датчика
	cfg.transADC = TRANsADC[sensor];                 // коэффициент пересчета АЦП в давление
	cfg.ADC_Gain = ADC_GAIN[sensor];
	cfg.number = sensor;
	pin = pinA;
	flags = SENSORPRESS[sensor] << fPresent;	 // наличие датчика
#ifdef ANALOG_MODBUS
	flags |= (ANALOG_MODBUS_ADDR[sensor] != 0)<<fsensModbus;  // Дистанционный датчик по модбас
#endif
	Chart.init();  			// инициалазация статистики
	err = OK;                                     // ошибка датчика (работа)
	Value = ERROR_PRESS;                      // давление датчика (обработанное)
	//Temp = ERROR_TEMPERATURE;
	note = (char*) notePress[sensor];              // присвоить наименование датчика
	name = (char*) namePress[sensor];              // присвоить имя датчика
}
    
 // очистить буфер АЦП
 void sensorADC::clearBuffer()
 {
#if P_NUMSAMLES > 1
      for(uint16_t i=0;i<P_NUMSAMLES;i++) p[i]=0;         // обнуление буффера значений
      sum=0;
      last=0;
#endif
      SETBIT0(flags,fFull);                      // Буфер не полный
 }
  
// чтение данных c аналогового датчика (АЦП) возвращает код ошибки, делает все преобразования
 int8_t sensorADC::Read()
 {
	 if(!(GETBIT(flags, fPresent))) return err;        // датчик запрещен в конфигурации ничего не делаем
	 int16_t lastPress;
	 if(testMode != NORMAL) {
		 lastPress = cfg.testValue;                // В режиме теста
	 } else {                                      // Чтение датчика
#ifdef ANALOG_MODBUS
		 if(get_fmodbus()) {
			 for(uint8_t i = 0; i < ANALOG_MODBUS_NUM_READ; i++) {
				 err = Modbus.readHoldingRegisters16(ANALOG_MODBUS_ADDR[cfg.number], ANALOG_MODBUS_REG[cfg.number] - 1, &adc_lastVal);
				 if(err == OK) {
					 lastADC = adc_lastVal;
					 break;
				 }
				 _delay(ANALOG_MODBUS_ERR_DELAY);
			 }
			 if(err) {
				 journal.jprintf_time("Error read %s by Modbus: %d\n", name, err);
				 set_Error(ERR_READ_PRESS, name);
				 return ERR_READ_PRESS;
			 }
		 } else
#endif
		 {
			 if(adc_flagFull) lastADC = adc_sum / (adc_filter_max + 1);	else lastADC = adc_sum / adc_last;
			 //if(adc.error!=OK)  {err=ERR_READ_PRESS;set_Error(err,name);return err;}   // Проверка на ошибку чтения ацп
		 }
		 lastPress = (int32_t) lastADC * cfg.transADC / 1000 - cfg.zeroValue;
	 }
#if P_NUMSAMLES > 1
	 // Усреднение значений
	 sum=sum+lastPress;// Добавить новое значение
	 sum=sum-p[last];// Убрать самое старое значение
	 p[last]=lastPress;// Запомить новое значение
	 if (last<P_NUMSAMLES-1) last++; else {last=0; SETBIT1(flags,fFull);}
	 if (GETBIT(flags,fFull)) lastPress=sum/P_NUMSAMLES; else lastPress=sum/last;
#endif
	 if(Value != lastPress) {
		 Value = lastPress;
		 //Temp = ERROR_TEMPERATURE;
	 }
	 err = OK;                                         // Новый цикл новые ошибки
	 return err;
 }

// Установка 0 датчика темпеартуры
int8_t sensorADC::set_zeroValue(int16_t p)
{
		clearBuffer();
		cfg.zeroValue = p;
		return OK;
}

// Установить значение коэффициента преобразования напряжение (отсчеты ацп)-температура
int8_t sensorADC::set_transADC(float p)
{
	clearBuffer();   // Суммы обнулить надо
	cfg.transADC = p * 1000 + 0.0005f;
	return OK;
}

// Установить значение давления датчика в режиме теста
int8_t sensorADC::set_testValue(int16_t p)            
{
	cfg.testValue=p;
	return OK;
}

// gain = x1, x2, x4
void sensorADC::set_ADC_Gain(uint8_t gain)
{
	if(gain < 1) gain = 1; else if(gain > 3) gain = 3;
	cfg.ADC_Gain = gain;
	ADC->ADC_CGR = (ADC->ADC_CGR & ~(3 << (2 * pin))) | (gain << (2 * pin));
}

void sensorADC::after_load(void)
{
}

// ------------------------------------------------------------------------------------------
// Цифровые контактные датчики (есть 2 состяния 0 и 1) --------------------------------------

// Инициализация контактного датчика
void  sensorDiditalInput::initInput(int sensor)
{
   Input=false;                    // Состояние датчика
   number = sensor;
   testInput=TESTINPUT[sensor];    // Состояние датчика в режиме теста
   testMode=NORMAL;                // Значение режима тестирования
   InputLevel=LEVELINPUT[sensor];  // Состояние датчика в режиме аварии
   err=OK;                         // ошибка датчика (работа)
   flags=0x00;                     // сброс флагов
   // флаги  0 - наличие датчика,  1- режим теста
   SETBIT1(flags,fPresent);        // наличие датчика в текушей конфигурации
   alarm_error = SENSOR_ERROR[sensor];
   pin=pinsInput[sensor];           // пин датчика
   pinMode(pin, PULLUPINPUT[sensor] ? INPUT_PULLUP : INPUT); // Настроить ножку на вход
   note=(char*)noteInput[sensor];   // присвоить наименование датчика
   name=(char*)nameInput[sensor];   // присвоить имя датчика
};

// Чтение датчика возвращает ошибку или ОК
int8_t sensorDiditalInput::Read(boolean fast)
{
	err = OK;                                            // Ошибки сбросить
	if(testMode != NORMAL) Input = testInput;            // В режиме теста
	else {
		boolean in = digitalReadDirect(pin) == InputLevel;
		if(!fast) {
			if(in != Input) {
				uint8_t i;
				for(i = 0; i < 2; i++) {
					_delay(1);
					if(in != (digitalReadDirect(pin) == InputLevel)) break;
				}
				if(i == 2) Input = in;
			}
		} else Input = in;
	}
	if(alarm_error && Input)     // Срабатывание аварийного датчика
	{
		err = alarm_error;
		set_Error(err, name);
	}
	return err;
}
    
// Установить Состояние датчика в режиме теста
int8_t sensorDiditalInput::set_testInput(int16_t i)
{
	if(i == 1) {
		testInput = true;
		return OK;
	} else if(i == 0) {
		testInput = false;
		return OK;
	} else return WARNING_VALUE;
}

// Установить уровень сработавшего датчика
int8_t sensorDiditalInput::set_InputLevel(int16_t i)
{
	if(i == 1) {
		InputLevel = true;
		return OK;
	} else if(i == 0) {
		InputLevel = false;
		return OK;
	} else return WARNING_VALUE;
}

// ------------------------------------------------------------------------------------------
// Цифровые частотные датчики (значение кодируется в выходной частоте) ----------------------
// основное назначение - датчики потока
// Частота кодируется в тысячных герца
// Число импульсов рассчитывается за базовый период (BASE_TIME), т.к частоты малы период надо савить не менее 5 сек

// Обработчики прерываний для подсчета частоты
#if FNUMBER > 0
void InterruptFreq0() { MC.sFrequency[0].InterruptHandler(); }
#if FNUMBER > 1
void InterruptFreq1() { MC.sFrequency[1].InterruptHandler(); }
#if FNUMBER > 2
void InterruptFreq2() { MC.sFrequency[2].InterruptHandler(); }
#if FNUMBER > 3
void InterruptFreq3() { MC.sFrequency[3].InterruptHandler(); }
#if FNUMBER > 4
void InterruptFreq4() { MC.sFrequency[4].InterruptHandler(); }
#if FNUMBER > 5
	#error FNUMBER is too big! Maximum 5 allowed!
#endif
#endif
#endif
#endif
#endif
#endif

// Инициализация частотного датчика, на входе номер сенсора по порядку
void sensorFrequency::initFrequency(int sensor)                     
{
	number = sensor;
	reset();
	testValue=TESTFLOW[sensor];                    // Состояние датчика в режиме теста
	kfValue=TRANSFLOW[sensor];                     // коэффициент пересчета частоты в значение
//	err=OK;                                        // ошибка датчика (работа)
	pin=pinsFrequency[sensor];                     // Ножка куда прицеплен датчик
	note=(char*)noteFrequency[sensor];             // наименование датчика
	name=(char*)nameFrequency[sensor];             // Имя датчика
	Chart.init();                              // инициалазация статистики
	// Привязывание обработчика преваний к методу конкретного класса
	//   LOW вызывает прерывание, когда на порту LOW
	//   CHANGE прерывание вызывается при смене значения на порту, с LOW на HIGH и наоборот
	//   RISING прерывание вызывается только при смене значения на порту с LOW на HIGH
	//   FALLING прерывание вызывается только при смене значения на порту с HIGH на LOW
	PIO_SetInput(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, PIO_DEGLITCH);
	//PIO_SetDebounceFilter(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, 1); // PIO_DEBOUNCE - Cutoff freq in Hz
#if FNUMBER > 0
	if(sensor == 0) attachInterrupt(pin, InterruptFreq0, CHANGE);
#if FNUMBER > 1
	else if(sensor == 1) attachInterrupt(pin, InterruptFreq1, CHANGE);
#if FNUMBER > 2
	else if(sensor == 2) attachInterrupt(pin, InterruptFreq2, CHANGE);
#if FNUMBER > 3
	else if(sensor == 3) attachInterrupt(pin, InterruptFreq3, CHANGE);
#if FNUMBER > 4
	else if(sensor == 4) attachInterrupt(pin, InterruptFreq4, CHANGE);
#endif
#endif
#endif
#endif
#endif
}

// Получить (точнее обновить) значение датчика, возвращает 1, если был проток
int8_t sensorFrequency::Read(int32_t add_pulses100)
{
	uint32_t tickCount;
	int8_t flow = 0;
	if((tickCount = GetTickCount()) - sTime >= (uint32_t) FREQ_BASE_TIME_READ) {  // если только пришло время измерения
		uint32_t cnt;
		noInterrupts();
		cnt = count;
		count = 0;
		interrupts();
		//__asm__ volatile ("" ::: "memory");0
		uint32_t ticks = tickCount - sTime;
		sTime = tickCount;
		if(MC.testMode != NORMAL) {    // В режиме теста
			Value = testValue;
			cnt = Value * kfValue / 360;
			if(cnt) flow = 1;
			Frequency = cnt / 2;
			PassedRest += cnt / 10 * ticks / FREQ_BASE_TIME_READ;
			Passed = PassedRest / kfValue;
			PassedRest %= kfValue;
		} else {
			cnt *= 100;
			if(add_pulses100 > 0) cnt += add_pulses100;
			if(cnt) flow = 1;
			PassedRest += cnt;
			Passed += PassedRest / kfValue;
			PassedRest %= kfValue;
			count_Flow += cnt;
			if(++FlowCalcCnt >= FlowCalcPeriod) {
				if(ticks == FREQ_BASE_TIME_READ) {
					count_Flow *= 10;
				} else if(cnt > 0xFFFFFFFF / FREQ_BASE_TIME_READ / 10) { // will overflow u32
					count_Flow = (count_Flow * (10 * FREQ_BASE_TIME_READ / 100)) / ticks * 100;
				} else {
					count_Flow = (count_Flow * 10 * FREQ_BASE_TIME_READ) / ticks; // ТЫСЯЧНЫЕ ГЦ время в миллисекундах частота в тысячных герца
				}
				Frequency = count_Flow / 2;
				//   Value=60.0*Frequency/kfValue/1000.0;               // Frequency/kfValue  литры в минуту а нужны кубы
				//       Value=((float)Frequency/1000.0)/((float)kfValue/360000.0);          // ЛИТРЫ В ЧАС (ИЛИ ТЫСЯЧНЫЕ КУБА) частота в тысячных, и коэффициент правим
				Value = count_Flow * 360 / kfValue; // ЛИТРЫ В ЧАС (ИЛИ ТЫСЯЧНЫЕ КУБА) частота в тысячных, и коэффициент правим
				count_Flow = 0;
			}
		}
		//journal.jprintfopt("Flow(%d): %d = %d (%d, %d) f: %d\n", ticks, cnt / 100, Value, Passed, PassedRest / 100, Frequency);
	}
	return flow;
}

void sensorFrequency::reset(void)
{
	sTime = GetTickCount();
	count = 0;
	count_Flow = 0;
	Passed = 0;
	PassedRest = 0;
	FlowCalcCnt = 0;
	WebCorrectCnt = 0;
	Frequency=0;                                   // значение частоты
	Value=0;                                       // значение датчика в ТЫСЯЧНЫХ (умножать на 1000)
}

// Установить Состояние датчика в режиме теста
int8_t  sensorFrequency::set_testValue(int16_t i) 
{
   testValue=i;
   return OK;
}

// ------------------------------------------------------------------------------------------
// Исполнительное устройство РЕЛЕ (есть 2 состяния 0 и 1) --------------------------------------
// Relay = true - это означает включение исполнительного механизама. 
// При этом реальный выход и состояние (физическое реле) определяется дефайнами RELAY_INVERT и R4WAY_INVERT
// ВНИМАНИЕ: По умолчанию (не определен RELAY_INVERT) выход инвертируется - Влючение реле (Relay=true) соответствует НИЗКИЙ уровень на выходе МК
void devRelay::initRelay(int sensor)
{
   flags=0x00;
   number = sensor;
   testMode=NORMAL;                // Значение режима тестирования
   flags=0x01;                     // наличие датчика в текушей конфигурации (отстатки прошлого, реле сейчас есть всегда)  флаги  0 - наличие датчика,  1- режим теста
   pin=pinsRelay[sensor];  
   pinMode(pin, OUTPUT);           // Настроить ножку на выход
   Relay=false;                    // Состояние реле - выключено
   TimerOn = 0;
#ifndef RELAY_INVERT            // Нет инвертирования реле -  Влючение реле (Relay=true) соответсвует НИЗКИЙ уровень на выходе МК
	#ifdef R4WAY_INVERT              // Признак инвертирования 4х ходового
   	   digitalWriteDirect(pin, number != R4WAY);  // Установить значение
	#else
   	   digitalWriteDirect(pin, true);  // Установить значение
	#endif
#else
	#ifdef R4WAY_INVERT              // Признак инвертирования 4х ходового
   	   digitalWriteDirect(pin, number == R4WAY);  // Установить значение
	#else
   	   digitalWriteDirect(pin, false);  // Установить значение
	#endif
#endif
   note=(char*)noteRelay[sensor];  // присвоить описание реле
   name=(char*)nameRelay[sensor];  // присвоить имя реле
}

// Уменьшить таймер отображения активного состояния
void devRelay::NextTimerOn(void) {
	if(TimerOn) {
		if(TimerOn >= TIME_SLICE_PUMPS) TimerOn -= TIME_SLICE_PUMPS; else TimerOn = 0;
	}
}

// Установить реле в состояние r, базовая функция все остальные функции используют ее
// Если состояния совпадают то ничего не делаем, 0/-1 - выкл основной алгоритм, fR_Status* - включить, -fR_Status* - выключить)
int8_t devRelay::set_Relay(int8_t r)
{
	if(!(flags & (1 << fPresent))) return ERR_DEVICE;  // Реле не установлено  и пытаемся его включить
	if(r == 0) r = -fR_StatusMain;
	else if(r == fR_StatusAllOff) {
		flags &= ~fR_StatusMask;
		r = -fR_StatusMain;
	}
	flags = (flags & ~(1 << abs(r))) | ((r > 0) << abs(r));
	r = (flags & fR_StatusMask) != 0;
	if(Relay == r) return OK;   // Ничего менять не надо - выходим
    if(TimerOn || Relay) TimerOn = TIMER_TO_SHOW_STATUS;
	if(number == RFILL && r) MC.RFILL_last_time_ON = rtcSAM3X8.unixtime();
	if(testMode == NORMAL || testMode == HARD_TEST) {
#ifndef RELAY_INVERT            // Нет инвертирования реле -  Влючение реле (Relay=true) соответсвует НИЗКИЙ уровень на выходе МК
		r = !r;
#endif
#ifdef R4WAY_INVERT              // Признак инвертирования 4х ходового
		if(number == R4WAY) r = !r;
#endif
		digitalWriteDirect(pin, r);
	}
    Relay = r;                  // Все удачно, сохранить
#ifdef RELAY_WAIT_SWITCH
	uint8_t tasks_suspended = TaskSuspendAll();
	delay(RELAY_WAIT_SWITCH);
	if(tasks_suspended) xTaskResumeAll();
#endif
	if(number > RFEEDPUMP) journal.jprintfopt_time("%s: %s\n", name, Relay ? "ON" : "OFF");
	return OK;
}

// ------------------------ Счетчик PWM ---------------------------------------
// Инициализация счетчика и проверка и если надо программирование
int8_t devPWM::initPWM()
{
	err = OK;                                        // Ошибок нет
	numErr = 0;                                      // счетчик 0
	Voltage = 0;                                     // Напряжение
	Power = 0;
	flags = 0x00;
	// Настройки
	name = (char*) namePWM;
	note = (char*) notePWM;

	SETBIT1(flags, fPWM);                           // счетчик представлен
	// инициализация статистики
	ChartVoltage.init();               // Статистика по напряжению
	ChartPower.init();                 // Статистика по Полная мощность
	return err;
}

// Прочитать инфо с счетчика, group: 0 - основная (при каждом цикле); 2 - через PWM_READ_PERIOD
int8_t devPWM::get_readState(uint8_t group)
{
#ifdef USE_UPS
	if(MC.NO_Power) {
		Power = 0;
		Voltage = 0;
		return ERR_MODBUS_UNKNOWN;
	}
#endif
	err=OK;
	if(MC.get_testMode() != NORMAL) {
		Power = TestPower;
		Voltage = 2200;
		return err;
	}

	for(int8_t i=0; i < PWM_NUM_READ; i++)   // делаем PWM_NUM_READ попыток чтения
	{
		if(group == 0) {
			uint32_t tmp;
			err = Modbus.readInputRegisters32(PWM_MODBUS_ADR, PWM_POWER, &tmp);
			if(err == OK) {
				tmp = tmp / 10 + (tmp % 10 >= 5 ? 1 : 0); // round to W
				Power = tmp;
				/* group = 1 */
			} else goto xErr;
		}
		if(group == 1) {
			err = Modbus.readInputRegisters16(PWM_MODBUS_ADR, PWM_VOLTAGE, &Voltage);
			if(err != OK) goto xErr;
		}
/*		else if(group == 2) {
			err=Modbus.readInputRegistersFloat(PWM_MODBUS_ADR, PWM_CURRENT,&tmp);
			if(err == OK) { Current=tmp; group = 3; } else goto xErr;
		}
		if(group == 3) {
			err=Modbus.readInputRegistersFloat(PWM_MODBUS_ADR, PWM_ENERGY,&tmp);
			if(err==OK) AcEnergy=tmp;
		}
*/
		if(err == OK) break;
xErr:
#ifdef SPOWER
		MC.sInput[SPOWER].Read(true);
        if(MC.sInput[SPOWER].get_Input()) return err;
#endif
#ifdef  USE_UPS_220
        if(MC.NO_Power) return err = ERR_NO_POWER;
#endif
        numErr++;                  // число ошибок чтение по модбасу
		if(GETBIT(MC.Option.flags, fPWMLogErrors)) {
			journal.jprintf_time("%s: Read #%d error %d\n", name, group, err);      // Выводим сообщение о повторном чтении
		}
		_delay(PWM_DELAY_REPEAT);  // Чтение не удачно, делаем паузу
	}
#ifdef  USE_UPS_220
	if(err) {
		numErr -= PWM_NUM_READ;
		err = ERR_NO_POWER;
	}
#else
	if(err && GETBIT(MC.Option.flags, fPWMLogErrors)) {
		journal.jprintf_time("%s: Read #%d error %d!\n", name, group, err);
	}
#endif
	return err;
}

// Получить параметр счетчика в виде строки
char* devPWM::get_param(char *var, char *ret)
{
	static uint32_t tmp;

	if(strcmp(var, pwm_VOLTAGE) == 0) {      // Напряжение
		_dtoa(ret, Voltage, 1);
		return ret;
	} else if(strcmp(var, pwm_POWER) == 0) {      // мощность
		_dtoa(ret, Power, 0);
		return ret;
	} else if(strcmp(var, pwm_TestPower) == 0) {
		_dtoa(ret, TestPower, 0);
		return ret;
	} else if(strcmp(var, pwm_CURRENT) == 0) {       // Ток
		if(Modbus.readInputRegisters32(PWM_MODBUS_ADR, PWM_CURRENT, &tmp) == OK) {
			_dtoa(ret, tmp, 3);
			return ret;
		}
	} else if(strcmp(var, pwm_PFACTOR) == 0) {       // Коэффициент мощности
		if(Modbus.readInputRegisters32(PWM_MODBUS_ADR, PWM_PFACTOR, &tmp) == OK) {
			_dtoa(ret, tmp, 2);
			return ret;
		}
	} else if(strcmp(var, pwm_FREQ) == 0) {         // Частота
		if(Modbus.readInputRegisters32(PWM_MODBUS_ADR, PWM_FREQ, &tmp) == OK) {
			_dtoa(ret, tmp, 1);
			return ret;
		}
	} else if(strcmp(var, pwm_ACENERGY) == 0) {
		if(Modbus.readInputRegisters32(PWM_MODBUS_ADR, PWM_ENERGY, &tmp) == OK) {
			_dtoa(ret, tmp, 3);
			return ret;
		}
	} else if(strcmp(var, pwm_NAME) == 0) {
		return strcat(ret, (char*) name);
	} else if(strcmp(var, pwm_NOTE) == 0) {
		return strcat(ret, (char*) note);
	} else if(strcmp(var, pwm_ERRORS) == 0) {
		return _itoa(numErr, ret);
	} else if(strcmp(var, pwm_ModbusAddr) == 0) {
		return _itoa(PWM_MODBUS_ADR, ret);
	}
	return strcat(ret,(char*)cInvalid);
}

// Установить параметр счетчика в виде строки
boolean devPWM::set_param(char *var, int32_t f)
{
   if(strcmp(var, pwm_RESET) == 0) {
	   uint8_t st = Modbus.CustomRequest(PWM_MODBUS_ADR, PWM_RESET_ENERGY);
	   if(st) journal.jprintf("PWM energy reset error %d!\n", st); else journal.jprintf("PWM energy reseted!\n");
	   return st;
   } else if(strcmp(var, pwm_TestPower) == 0) {
	   TestPower = f;
	   return true;
   }
   return false;
}

// МОДБАС Устройство ----------------------------------------------------------
// функции обратного вызова
static uint8_t Modbus_Entered_Critical = 0;
static inline void idle() // задержка между чтениями отдельных байт по Modbus
    {
//		delay(1);		// Не отдает время другим задачам
		_delay(1);		// Отдает время другим задачам
    }
static inline void preTransmission() // Функция вызываемая ПЕРЕД началом передачи
    {
      #ifdef PIN_MODBUS_RSE
      digitalWriteDirect(PIN_MODBUS_RSE, HIGH);
      #endif
      Modbus_Entered_Critical = TaskSuspendAll(); // Запрет других задач во время передачи по Modbus
    }
static inline void postTransmission() // Функция вызываемая ПОСЛЕ окончания передачи
    {
	if(Modbus_Entered_Critical) {
		xTaskResumeAll();
		Modbus_Entered_Critical = 0;
	}
#if MODBUS_TIME_TRANSMISION > 0
	while(!(MODBUS_PORT_NUM.availableForWrite() >= SERIAL_BUFFER_SIZE-1 && (MODBUS_PORT_NUM._pUart->UART_SR & UART_SR_TXEMPTY))) _delay(1);
#endif
#ifdef PIN_MODBUS_RSE
	#if MODBUS_TIME_TRANSMISION != 0
    _delay(MODBUS_TIME_TRANSMISION);// Минимальная пауза между командой и ответом 3.5 символа
	#endif
    digitalWriteDirect(PIN_MODBUS_RSE, LOW);
#endif
}

// Инициализация Modbus без проверки связи
int8_t devModbus::initModbus()
     {
#if defined(MODBUS_SERIAL1) || defined(MODBUS_SERIAL2)
        flags=0x00;
	#ifdef PIN_MODBUS_RSE
        pinMode(PIN_MODBUS_RSE , OUTPUT);                                            // Подготовка управлением полудуплексом
        digitalWriteDirect(PIN_MODBUS_RSE , LOW);
	#endif
		vSemaphoreCreateBinary(xModbusSemaphore);                       // Создание мютекса
		if(xModbusSemaphore==NULL) set_Error(ERR_MEM_FREERTOS,(char*)__FUNCTION__);
		else {
#ifdef MODBUS_SERIAL1
			MODBUS_SERIAL1.begin(MODBUS_SERIAL1_SPEED,MODBUS_PORT_CONFIG);
#endif
#ifdef MODBUS_SERIAL2
			MODBUS_SERIAL2.begin(MODBUS_SERIAL2_SPEED,MODBUS_PORT_CONFIG);
#endif
	        //MODBUS_SERIAL1.setInterruptPriority(1);
	        //RS485.begin(0, MODBUS_SERIAL1);	// установка сериала и адреса устройства
			RS485.ModbusMinTimeBetweenTransaction = MODBUS_MIN_TIME_BETWEEN_TRNS;
			RS485.ModbusResponseTimeout = MODBUS_TIMEOUT;
	#ifdef MODBUS_TIME_TRANSMISION
			// Назначение функций обратного вызова
			RS485.preTransmission(preTransmission);
			RS485.postTransmission(postTransmission);
	#endif
			RS485.idle(idle);
	        SETBIT1(flags,fModbus);                                                      // модбас присутствует
			err=OK;                                                                      // Связь есть
		}
#else
        flags=0x00;
        SETBIT0(flags,fModbus);                                                     // модбас отсутвует
        err=ERR_NO_MODBUS;
#endif
        return err;                                                                 
     }

// ФУНКЦИИ ЧТЕНИЯ ----------------------------------------------------------------------------------------------
// Получить значение 2-x (Modbus function 0x04 Read Input Registers) регистров (4 байта) в виде float возвращает код ошибки данные кладутся в ret
int8_t devModbus::readInputRegistersFloat(uint8_t id, uint16_t cmd, float *ret)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	//RS485.set_slave(id);
	uint8_t result = RS485.readInputRegisters(cmd, 2);                                               // послать запрос,
	if(result == RS485.ku8MBSuccess) {
		err = OK;
		*ret = fromInt16ToFloat(RS485.getResponseBuffer(0), RS485.getResponseBuffer(1));
		SemaphoreGive(xModbusSemaphore);
		return OK;
	} else {
		*ret = 0;
		SemaphoreGive(xModbusSemaphore);
		return err = translateErr(result);
	}
}

int8_t devModbus::readInputRegisters16(uint8_t id, uint16_t cmd, uint16_t *ret)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.readInputRegisters(cmd, 1);
	if(result == RS485.ku8MBSuccess) {
		*ret = RS485.getResponseBuffer(0);
		err = OK;
	} else {
		err = translateErr(result);
	}
	SemaphoreGive(xModbusSemaphore);
	return err;
}

// LITTLE-ENDIAN! 0x04
int8_t devModbus::readInputRegisters32(uint8_t id, uint16_t cmd, uint32_t *ret)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.readInputRegisters(cmd, 2);
	if(result == RS485.ku8MBSuccess) {
		*ret = (RS485.getResponseBuffer(1) << 16) | RS485.getResponseBuffer(0);
		err = OK;
	} else {
		err = translateErr(result);
	}
	SemaphoreGive(xModbusSemaphore);
	return err;
}

// Получить значение регистра (2 байта) в виде целого  числа возвращает код ошибки данные кладутся в ret
int8_t devModbus::readHoldingRegisters16(uint8_t id, uint16_t cmd, uint16_t *ret)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.readHoldingRegisters(cmd, 1);                                                   // послать запрос,
	if(result == RS485.ku8MBSuccess) {
		*ret = RS485.getResponseBuffer(0);
		err = OK;
	} else {
		*ret = 0;
		err = translateErr(result);
	}
	SemaphoreGive(xModbusSemaphore);
	return err;
}
    
// Получить значение 2-x регистров (4 байта) в виде целого  числа возвращает код ошибки данные кладутся в ret, BIG-ENDIAN!
int8_t devModbus::readHoldingRegisters32(uint8_t id, uint16_t cmd, uint32_t *ret)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.readHoldingRegisters(cmd, 2);                                             // послать запрос,
	if(result == RS485.ku8MBSuccess) {
		*ret = (RS485.getResponseBuffer(0) << 16) | RS485.getResponseBuffer(1);
		err = OK;
	} else {
		*ret = 0;
		err = translateErr(result);
	}
	SemaphoreGive(xModbusSemaphore);
	return err;
}
      
// Получить значение 2-x регистров (4 байта) в виде float возвращает код ошибки данные кладутся в ret
int8_t devModbus::readHoldingRegistersFloat(uint8_t id, uint16_t cmd, float *ret)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE)      // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.readHoldingRegisters(cmd, 2);                                             // послать запрос,
	if(result == RS485.ku8MBSuccess) {
		err = OK;
		*ret = fromInt16ToFloat(RS485.getResponseBuffer(0), RS485.getResponseBuffer(1));
	} else {
		err = translateErr(result);
		*ret = 0;
	}
	SemaphoreGive (xModbusSemaphore);
	return err;
}


// Получить значение N регистров c cmd (2*N байта) МХ2 в виде целого  числа (uint16_t *buf) при ошибке возвращает err
int8_t devModbus::readHoldingRegistersNN(uint8_t id, uint16_t cmd, uint16_t num, uint16_t *buf)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.readHoldingRegisters(cmd, num);                                           // послать запрос,
	if(result == RS485.ku8MBSuccess) {
		for(int16_t i = 0; i < num; i++)
			buf[i] = RS485.getResponseBuffer(i);
		err = OK;
		SemaphoreGive (xModbusSemaphore);
		return err;
	} else {
		err = translateErr(result);
		SemaphoreGive (xModbusSemaphore);
		return err;
	}
}

// прочитать Coils, возвращает ошибку, количество в num_ret, макс 8, Modbus function 0x01 Read Coils.
int8_t devModbus::readCoils(uint8_t id, uint16_t cmd, uint8_t *num_ret)
{
	uint8_t result;
// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	result = RS485.readCoils(cmd, *num_ret);                                  // послать запрос, Нумерация регистров с НУЛЯ!!!!
	if(result == RS485.ku8MBSuccess) {
		err = OK;
		SemaphoreGive (xModbusSemaphore);
		*num_ret = RS485.getResponseBuffer(0);
		return err;
	} else {
		err = translateErr(result);
		SemaphoreGive (xModbusSemaphore);
		return err;
	}
}

// прочитать дискретные входы, возвращает ошибку, количество в num_ret, макс 8, Modbus function 0x01 Read Coils.
int8_t devModbus::readDiscreteInputs(uint8_t id, uint16_t cmd, uint8_t *num_ret)
{
	uint8_t result;
// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	result = RS485.readDiscreteInputs(cmd, *num_ret);                                  // послать запрос, Нумерация регистров с НУЛЯ!!!!
	if(result == RS485.ku8MBSuccess) {
		err = OK;
		SemaphoreGive (xModbusSemaphore);
		*num_ret = RS485.getResponseBuffer(0);
		return err;
	} else {
		err = translateErr(result);
		SemaphoreGive (xModbusSemaphore);
		return err;
	}
}

// ФУНКЦИИ ЗАПИСИ ----------------------------------------------------------------------------------------------
// установить битовый вход функция Modbus function 0x05 Write Single Coil.
int8_t devModbus::writeSingleCoil(uint8_t id, uint16_t cmd, uint8_t u8State)
{
	uint8_t result;
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	result = RS485.writeSingleCoil(cmd, u8State);                                         // послать запрос,
	if(result == RS485.ku8MBSuccess) {
		err = OK;
	} else {
		err = translateErr(result);
	}
	SemaphoreGive(xModbusSemaphore);
	return err;
}
// Установить значение регистра (2 байта) МХ2 в виде целого  числа возвращает код ошибки данные data
int8_t devModbus::writeHoldingRegisters16(uint8_t id, uint16_t cmd, uint16_t data)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	uint8_t result = RS485.writeSingleRegister(cmd, data);                                            // послать запрос,
	SemaphoreGive(xModbusSemaphore);
	return err = translateErr(result);

}

// Записать 2 регистра подряд возвращает код ошибки, BIG-ENDIAN!
int8_t devModbus::writeHoldingRegisters32(uint8_t id, uint16_t cmd, uint32_t data)
{
	uint8_t result;
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	RS485.setTransmitBuffer(0, data >> 16);
	RS485.setTransmitBuffer(1, data & 0xFFFF);
	result = RS485.writeMultipleRegisters(cmd, 2);                                                 // послать запрос,
	SemaphoreGive(xModbusSemaphore);
	return err = translateErr(result);
}

// Записать float как 2 регистра числа возвращает код ошибки данные dat
int8_t devModbus::writeHoldingRegistersFloat(uint8_t id, uint16_t cmd, float dat)
{
	union {
		float f;
		uint16_t i[2];
	} float_map = { .f = dat };
	uint8_t result;
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	RS485.setTransmitBuffer(0, float_map.i[1]);
	RS485.setTransmitBuffer(1, float_map.i[0]);
	result = RS485.writeMultipleRegisters(cmd, 2);
	//   result = RS485.writeSingleRegister(cmd,dat);                                               // послать запрос,
	SemaphoreGive(xModbusSemaphore);
	return err = translateErr(result);

}

// Специальная функция
int8_t devModbus::CustomRequest(uint8_t id, uint8_t cmd)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	RS485.send(cmd);
	uint8_t result = RS485.ModbusMasterTransaction(ku8MBCustomRequest);
	SemaphoreGive(xModbusSemaphore);
	return err = translateErr(result);
}

// Отправка данных в виде строки чисел (hex - 0x...)
int8_t devModbus::CustomRequestData(uint8_t id, char *str)
{
	// Если шедулер запущен то захватываем семафор
	if(SemaphoreTake(xModbusSemaphore, (MODBUS_TIME_WAIT / portTICK_PERIOD_MS)) == pdFALSE) // Захват мютекса потока или ОЖИДАНИНЕ MODBUS_TIME_WAIT
	{
		journal.jprintf((char*) cErrorMutex, __FUNCTION__, MutexModbusBuzy);
		return err = ERR_485_BUZY;
	}
	RS485.begin(id, id >= MODBUS_SERIAL1_ADDR_GE ? MODBUS_SERIAL1 : MODBUS_SERIAL2);	// установка сериала и адреса устройства
	char *tail;
	do {
		uint8_t b = strtol(str, &tail, 0);
		if(tail == str) break;
		RS485.send(b);
		str = tail;
	} while(str && *str);
	uint8_t result = RS485.ModbusMasterTransaction(ku8MBCustomRequest);
	SemaphoreGive(xModbusSemaphore);
	return err = translateErr(result);
}

// Перевод ошибки протокола Модбас (что дает либа)
int8_t devModbus::translateErr(uint8_t result)
{
 switch (result)
    {
    // Сдандартные ошибки протокола modbus  едины для всех устройств на модбасе
    case 0x00:      return OK;                  break;
    case 0x01:      return ERR_MODBUS_0x01;     break;
    case 0x02:      return ERR_MODBUS_0x02;     break;
    case 0x03:      return ERR_MODBUS_0x03;     break;
    case 0x04:      return ERR_MODBUS_0x04;     break;
    case 0xe0:      return ERR_MODBUS_0xe0;     break;
    case 0xe1:      return ERR_MODBUS_0xe1;     break;
    case 0xe2:      return ERR_MODBUS_0xe2;     break;
    case 0xe3:      return ERR_MODBUS_0xe3;     break;
    default  :      return ERR_MODBUS_UNKNOWN;  break;
    }

}
