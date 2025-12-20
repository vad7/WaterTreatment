/*
 * Timer.c
 *
 * Created: 23.05.2024 19:00:00
 *  Author: Vadim Kulakov, vad7@yahoo.com
 * 
 * ATTiny13A
 *
 * Clock 4.8Mhz or 9.6Mhz (setup by fuses)
 * Connections:
 * pin 6 (PB1) -> OUT
 * pin 1 (PB2) -> LED - (cathode)
 * pin 2 (PB3) <- input Opto Interrupter SGM20001 (pullup to +)
 * pin 3 (PB4) <- setup KEY
 *
 */ 
#define F_CPU 4800000UL
// Fuses(0=set): BODLEVEL = 2.7V (BODLEVEL[1:0]=01), RSTDISBL=0, CKSEL[1:0] = 01(4.8Mhz) / = 10(9.6Mhz), CKDIV8=0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define THRESHOLD_LOW_DEFAULT	113		// +20% to minimum ADC
#define THRESHOLD_HI_DEFAULT	188		// -20% from maximum ADC

#define OUTPUT			(1<<PORTB1)
#define OUTPUT_LOW		PORTB &= ~OUTPUT
#define OUTPUT_HI		PORTB |= OUTPUT
#define OUTPUT_CHANGE	PORTB ^= OUTPUT
#define OUTPUT_STATUS	(PORTB & OUTPUT)
#define SETUP_OUTPUT	DDRB |= OUTPUT

#define KEY				(1<<PORTB4)
#define KEY_PRESSED		!(PINB & KEY)
#define SETUP_KEY		PORTB |= KEY

#define LED1_PORT		PORTB
#define LED1			(1<<PORTB2)
#define LED1_ON			LED1_PORT &= ~LED1	// drain (-)
#define LED1_OFF		LED1_PORT |= LED1
#define LED1_IS_ON		!(PORTB & LED1)
#define SETUP_LED		LED1_OFF; DDRB |= LED1

#define SETUP_UNUSED_PINS PORTB |= (1<<PORTB5) | (1<<PORTB0)

#define SETUP_WATCHDOG { WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0); } // Watchdog 0.25 s

struct _EEPROM {
	uint8_t		Divider;		// pulse divider
	uint8_t		Thr_Low;		// threshold -> 0
	uint8_t		Thr_Hi;			// threshold -> 1
	uint8_t		Setup_percent;	// %
	uint16_t	Setup_pulses_passed;
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

register uint8_t status asm("16");		// 0/1
register uint8_t status_div asm("17");
register uint8_t Divider asm("15");
register uint8_t Thr_Low asm("14");			// threshold -> 0
register uint8_t Thr_Hi asm("13");			// threshold -> 1
register uint8_t ADC_Last asm("12");
uint8_t  ADC_Prev = 0;
register uint8_t LED1_Cnt asm("11");
register uint8_t fuse_clock asm("10");		// 0 - 4.8Mhz, 1 - 9.6Mhz
register uint8_t last_output asm("9");
uint8_t  Timer;				// sec
uint8_t  timer_cnt	= 0;
uint8_t  setup		= 0;	// 0 - off, 1 - working, 2 - finish
uint8_t	 setup_low;
uint8_t	 setup_hi;
uint16_t Cnt = 0;

void Delay100ms(uint8_t ms) { // 0..127
	if(fuse_clock) ms *= 2;
	while(ms-- > 0) {
		_delay_ms(100);
		wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

void DelayFlashNumber(void)
{
	Delay100ms(15);
}

void FlashNumber(uint8_t Number) // HEX
{
	FlashLED(2, 1, 1);
	DelayFlashNumber();
	FlashLED(Number / 16, 7, 7);
	DelayFlashNumber();
	FlashLED(2, 1, 1);
	DelayFlashNumber();
	FlashLED(Number % 16, 7, 7);
	DelayFlashNumber();
}

ISR(ADC_vect)
{
	ADC_Last = ADCH;
	if(status) {
		if(ADC_Last <= Thr_Low) {
			if(++status_div == Divider) {
				OUTPUT_CHANGE;
				status_div = 0;
			}
			status = 0;
		}
	} else if(ADC_Last >= Thr_Hi) {
		if(++status_div == Divider) {
			OUTPUT_CHANGE;
			status_div = 0;
		}
		status = 1;
	}
}

ISR(TIM0_OVF_vect, ISR_NOBLOCK)	// 0.05s
{
	if(++timer_cnt == 20) { // 1 sec
		timer_cnt = 0;
		if(Timer) if(--Timer == 0) {
			if(setup) setup = 2;
		}
	}
	if(LED1_IS_ON) {
		if(++LED1_Cnt == 5) {
			LED1_OFF;
			LED1_Cnt = 0;
		}
	} else LED1_Cnt = 0;
}

int main(void)
{
	// get fuses low byte
	asm("ldi R30,0; ldi R31,0");
	SPMCSR = (1<<RFLB) | (1<<SPMEN);
	asm("lpm R10,Z");
	// get fuses high byte
	//asm("ldi R30,3");
	//SPMCSR = (1<<RFLB) | (1<<SPMEN);
	//asm("lpm R10,Z");
	fuse_clock = (fuse_clock >> 1) & 1; // R10
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler: 1
	SETUP_OUTPUT;
	SETUP_KEY;
	SETUP_LED;
	SETUP_UNUSED_PINS;
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Sleep idle enable
	//GIMSK |= (1<<INT0); // INT0: External Interrupt Request 0 Enable
	//GIFR = (1<<INTF0);
	// Timer 8 bit
	TCCR0A = (0<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00); // Timer0 prescaller: 1024
	TIMSK0 |= (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
	OCR0A = fuse_clock ? 234 : 117; // OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	//OCR0B = 0; // 0..OCR0A, Half Duty cycle = ((TOP+1)/2-1)
	//TCCR0A |= (1<<COM0B1); // Start PWM out
	ADMUX = (0<<REFS0) | (1<<ADLAR) | (1<<MUX1)|(1<<MUX0); // Voltage Reference VCC, ADC Left Adjust, in: ADC3 (PB3)
	ADCSRA = (1<<ADEN) | (1<<ADSC ) | (1<<ADIE) | (1<<ADATE) | // ADC Interrupt Enable, ADC Start, ADC Auto Trigger Enable,
			(fuse_clock ? (1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0) : (1<<ADPS2)|(0<<ADPS1)|(1<<ADPS0));  // ADC Prescaler: 32/64
	status = 1;
	status_div = 0;
	if(eeprom_read_byte(&EEPROM.Divider) == 0xFF) {
 		eeprom_update_byte(&EEPROM.Divider, 1);
 		eeprom_update_byte(&EEPROM.Thr_Low, THRESHOLD_LOW_DEFAULT);
 		eeprom_update_byte(&EEPROM.Thr_Hi, THRESHOLD_HI_DEFAULT);
 		eeprom_update_byte(&EEPROM.Setup_percent, 20);
	}
	Divider = eeprom_read_byte(&EEPROM.Divider);
	Thr_Low = eeprom_read_byte(&EEPROM.Thr_Low);
	Thr_Hi = eeprom_read_byte(&EEPROM.Thr_Hi);
	if(KEY_PRESSED) {	// Setup on power on: press KEY before power on, wait 3..7 sec, release KEY, calibrating within 60 sec
		Delay100ms(30);	// 3 sec
		if(KEY_PRESSED) {
			for(uint8_t i = 0; i < 50; i++) {
				Delay100ms(1);
				if(!KEY_PRESSED) {
					FlashLED(5, 2, 2);
					Timer = 60;
					setup_low = 0xFF;
					setup_hi = 0;
					setup = 1;
					break;
				}
			}
		}
	}
	sei();
	FlashLED(Divider, 1, 1);
	last_output = OUTPUT_STATUS;
    while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();	// power down
		wdt_reset();
		if(setup) {
			if(setup == 2) { // finish
				if(setup_hi - setup_low > 7) {
					uint8_t n = (setup_hi - setup_low) * eeprom_read_byte(&EEPROM.Setup_percent) / 100;
					eeprom_update_byte(&EEPROM.Thr_Low, Thr_Low = setup_low + n);
					eeprom_update_byte(&EEPROM.Thr_Hi, Thr_Hi = setup_hi - n);
					eeprom_update_byte((uint8_t *)&EEPROM.Setup_pulses_passed, Cnt & 0xFF);
					eeprom_update_byte((uint8_t *)&EEPROM.Setup_pulses_passed+1, Cnt / 256);
					FlashLED(10, 2, 2);
				}
				setup = 0;
			} else if(ADC_Prev != ADC_Last) {
				if(ADC_Last < setup_low) setup_low = ADC_Last;
				else if(ADC_Last > setup_hi) setup_hi = ADC_Last;
				ADC_Prev = ADC_Last;
			}
		}
		if(last_output != OUTPUT_STATUS) {
			LED1_ON;
			Cnt++;
			last_output = OUTPUT_STATUS;
		}
		if(KEY_PRESSED) {
			Delay100ms(3);
			if(KEY_PRESSED) {
				FlashNumber(Cnt / 256);
				FlashNumber(Cnt & 0xFF);
				Cnt = 0;
			}
		}
	}
}
