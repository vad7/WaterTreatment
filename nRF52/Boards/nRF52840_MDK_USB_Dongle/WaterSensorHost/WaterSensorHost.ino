#include "Arduino.h"

//#include "nrf.h"
//#include "nrf_radio.h"
//#ifdef ARDUINO_ARCH_NRF52
//#define ARDUINO_ARCH_NRF52
//#endif
//
#include "defines.h"
#include "NRF5\MyConfig.h"
#include "NRF5\MyHwNRF5.h"
#include "NRF5\MyHwHAL.h"
//#include "NRF5\NVRAM.h"
//#include "NRF5\MyHwHAL.cpp"
//#include "NRF5\Radio.h"
//#include "NRF5\Radio.cpp"
//#include "NRF5\MyHwNRF5.cpp"
//#include "NRF5\Radio_ESB.h"
//#include "NRF5\Radio_ESB.cpp"
//#include "NRF5\MyTransportNRF5_ESB.cpp"

/*
int main(void)
{
	init();
#if defined(USBCON)
	USBDevice.attach();
#endif
	_begin(); // Startup MySensors library
	for(;;) {
		_process();  // Process incoming data
		if (loop) {
			loop(); // Call sketch loop
		}
		if (serialEventRun) {
			serialEventRun();
		}
	}
	return 0;
}
*/


/*
#include <MySensors.h>
//#define NRF52
#define MY_LED 0
#define MY_KEY 4
//#define MY_BUTTON_ADC_PIN X // номер пина АЦП смотреть в MyBoardNRF5.cpp
//#define MY_BUTTON_PIN (MY_BUTTON_ADC_PIN X-1)  //this is the same pin number MY_BUTTON_ADC_PIN, see the pins table above (only nrf51, I think so)
#define MY_BUTTON_ADC_PIN 6 // номер пина АЦП смотреть в MyBoardNRF5.cpp
//#define MY_BUTTON_PIN 5  //this is the same pin number MY_BUTTON_ADC_PIN, see the pins table above (only nrf51, I think so)


void preHwInit()
{
	pinMode(MY_LED, OUTPUT);
	//  pinMode(MY_BUTTON_PIN, INPUT_PULLUP);
	pinMode(MY_KEY, INPUT_PULLUP);
	digitalWrite(MY_LED, LOW);
}

void before()
{
	NRF_POWER->DCDCEN = 1; // закоментировать если E73
	NRF_UART0->ENABLE = 0;
}

void setup()
{
	analogReference (AR_VBG);
	send(msgVolt.set(((float) hwCPUVoltage() / 1000), 2)); //отправляю напряжение батареи
	lpComp();  // if y use lpcomp interrupt
	led(4, 1);
	detection = 0;
	wait(1000);
}

void loop()
{
	sleep (sleep_batt);
	if(detection == 1) {
		s_lpComp();
		if(digitalRead(MY_KEY) == 1) {
			send(msgDoor.setDestination(0).set("true!"));
			led(2, 1);
			wait(100);
		}
		if(digitalRead(MY_KEY) == 0) {
			send(msgDoor.setDestination(0).set("false!"));
			led(1, 1);
			wait(100);
		}
		//    led(3, 3);
		detection = 0;
		NRF_LPCOMP->EVENTS_CROSS = 0;
		r_lpComp();
	} else {
		send(msgVolt.set(((float) hwCPUVoltage() / 1000), 2)); //отправляю напряжение батареи
	}
}

void led(uint8_t flash, uint8_t iteration)
{
	//    for (int x = 0; x < iteration; x++) {  //повтор
	for(int i = 0; i < flash; i++) {
		digitalWrite(MY_LED, HIGH);
		wait(20);
		digitalWrite(MY_LED, LOW);
		wait(20);
		//    }
		//    wait(500);
	}
}

void lpComp()
{
	NRF_LPCOMP->PSEL = MY_BUTTON_ADC_PIN;
	NRF_LPCOMP->ANADETECT = 0; // детектирование EVENTS_DOWN.
	//ANADETECT=0; // детектирование EVENTS_CROSS.
	//ANADETECT=1; // детектирование EVENTS_UP.
	//ANADETECT=2; // детектирование EVENTS_DOWN.
	NRF_LPCOMP->INTENSET = B1000; // активация прерывания для EVENTS_DOWN
	//B1000; // прерываниt для EVENTS_CROSS
	//B0100; // прерываниt для EVENTS_UP
	//B1000; // прерываниt для EVENTS_DOWN
	NRF_LPCOMP->ENABLE = 1;
	NRF_LPCOMP->TASKS_START = 1;
	NVIC_SetPriority(LPCOMP_IRQn, 15);
	NVIC_ClearPendingIRQ(LPCOMP_IRQn);
	NVIC_EnableIRQ(LPCOMP_IRQn);
}

void s_lpComp()
{
	if((NRF_LPCOMP->ENABLE) && (NRF_LPCOMP->EVENTS_READY)) {
		NRF_LPCOMP->INTENCLR = B1000; //деактивация прерывания для LPCOMP
	}
}

void r_lpComp()
{
	NRF_LPCOMP->INTENSET = B1000;
}
#if __CORTEX_M == 0x04
#define NRF5_RESET_EVENT(event)                                                 \
		event = 0;                                                                   \
		(void)event
#else
#define NRF5_RESET_EVENT(event) event = 0
#endif
extern "C" {
	void LPCOMP_IRQHandler(void)
	{
		detection = 1;
		NRF5_RESET_EVENT(NRF_LPCOMP->EVENTS_CROSS);
		NRF_LPCOMP->EVENTS_CROSS = 0;
		MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2);
	}
}
*/
