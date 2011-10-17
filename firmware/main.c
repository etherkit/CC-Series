/*
 * main.c
 *
 *  Created on: Oct 20, 2010
 *      Author: Jason Milldrum, NT7S
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
//#include <avr/wdt.h>

#include "morsechar.h"
#include "sinewave.h"

// Input and output pin definitions
#define	SIDETONE				PD6			// Sidetone output
#define SIDETONE_PORT			PORTD		// Sidetone output port
#define SIDETONE_DDR			DDRD		// Sidetone output DDR

#define PADDLE_DIT				PD3			// Dit paddle
#define PADDLE_DIT_PORT			PORTD		// Dit paddle port
#define	PADDLE_DIT_DDR			DDRD		// Dit paddle DDR
#define PADDLE_DIT_PIN			PIND		// Dit paddle pin
#define PADDLE_DIT_PC			PCINT19

#define PADDLE_DAH				PD4			// Dah paddle
#define PADDLE_DAH_PORT			PORTD		// Dah paddle port
#define	PADDLE_DAH_DDR			DDRD		// Dah paddle DDR
#define PADDLE_DAH_PIN			PIND		// Dah paddle pin
#define PADDLE_DAH_PC			PCINT20

#define CMD_BUTTON				PD2			// Command button
#define CMD_BUTTON_PORT			PORTD
#define CMD_BUTTON_DDR			DDRD
#define CMD_BUTTON_PIN			PIND
#define CMD_BUTTON_PC			PCINT18

#define MSG_BUTTON				PD1			// Message button
#define MSG_BUTTON_PORT			PORTD
#define MSG_BUTTON_DDR			DDRD
#define MSG_BUTTON_PIN			PIND
#define MSG_BUTTON_PC			PCINT17

#define ENC_A					PB0			// Rotary encoder A output
#define ENC_A_PORT				PORTB
#define ENC_A_DDR				DDRB
#define ENC_A_PIN				PINB
#define ENC_A_PC				PCINT0

#define ENC_B					PD7			// Rotary encoder B output
#define ENC_B_PORT				PORTD
#define ENC_B_DDR				DDRD
#define ENC_B_PIN				PIND
#define ENC_B_PC				PCINT23

#define ENC_BUTTON				PD0			// Rotary encoder button
#define ENC_BUTTON_PORT			PORTD
#define ENC_BUTTON_DDR			DDRD
#define ENC_BUTTON_PIN			PIND
#define ENC_BUTTON_PC			PCINT16

#define MUTE					PC5			// Mute output
#define MUTE_PORT				PORTC
#define MUTE_DDR				DDRC

#define TX						PC4			// Transmit output
#define TX_PORT					PORTC
#define TX_DDR					DDRC

#define SPI_DDR					DDRB		// SPI
#define SPI_PORT				PORTB
#define SPI_MOSI				PB3
#define SPI_SCK					PB5
#define SPI_FSYNC				PB1
#define SPI_SS					PB2

#define RIT_LED					PC1			// RIT enable LED
#define RIT_LED_PORT			PORTC
#define RIT_LED_DDR				DDRC

// Constant defines
#define TIMER2_COUNT			249			// 4 us clk period * 250 ticks (0-249)= 1 ms Timer2 CTC A overflow
#define DEBOUNCE_PRESS_TIME		5			// Amount of captures for press keybounce (in 1 ms increments)
#define DEBOUNCE_HOLD_TIME		500			// Amount of captures for hold keybounce (in 1 ms increments)
#define DEFAULT_WPM				13			// Default keyer WPM
#define	MIN_WPM					5			// Minimum WPM setting
#define MAX_WPM					40			// Maximum WPM setting
#define TX_ON_DELAY				1			// TX sequence delay time (in 1 ms increments)
#define MUTE_OFF_DELAY			50			// Mute off delay time (in 1 ms increments)
#define ANNOUNCE_BUFFER_SIZE	41			// Buffer size for announce string
//#define IF_FREQ					4915000UL	// IF frequency
//#define FC_GATE_TIME			100 		// Frequency counter gate time (in 1 ms increments)
#define MENU_EXPIRATION			4000		// Menu expiration time (in 1 ms increments)
#define REC_EXPIRATION			1000		// Keyer memory character record expiration
#define MSG_BUFFER_SIZE			41			// Keyer message size in characters
#define SLEEP_DELAY				300		// Time (in ms) to delay before going to sleep because of inactivity
#define ST_REFCLK				268435		// Sidetone DDS ref clock - 16 MHz clock / 16 kHz sample rate
#define ST_DEFAULT				500			// Default sidetone frequency
#define ST_HIGH					900			// High sidetone frequency
#define ST_LOW					400			// Low sidetone frequency
#define XIT_BLINK				500		// LED blick time in ms

// DDS tuning steps (25 MHz master clock)
#define DDS_10HZ				0x1A		// DDS tuning word 10 Hz increment (/4, since each click of enc is 4 activations)
#define DDS_20HZ				0x34
#define DDS_50HZ				0x86
#define DDS_100HZ				0x10C
#define DDS_200HZ				0x218

// Band constants
#define DDS_INIT				0x05111F0C	// 14.060 MHz
#define FREQ_INIT				14060000
#define LOWER_FREQ_LIMIT		14000000
#define UPPER_FREQ_LIMIT		14350000

// Macro for any button press
#define ANYBUTTON				(dit_active == TRUE) || (dah_active == TRUE) || (cmd_btn == PRESS) || (msg_btn == PRESS)

// State machine and other enumerations
enum BOOL {FALSE, TRUE};
enum STATE {INIT, IDLE, DIT, DAH, DITDELAY, DAHDELAY, WORDDELAY, KEYDOWN, ENDKEYDOWN, MENUANNOUNCE, MENUINPUT, VALIDATECHAR, EXIT};
enum MODE {KEYER, SK, ANNOUNCE, TUNE, MENU, SETWPM, PLAYBACK, RECORD};
enum BTN {OFF, PRESS, HOLD};
enum TUNERATE {SLOW, FAST};
enum FREQREG {REG_0, REG_1}; // During incremental tuning, REG_0 is the RX freq, REG_1 is the TX freq
enum INCTUNE {NONE, RIT, XIT};

// Global variable defs
uint16_t dit_length;
uint8_t wpm, prev_wpm, pwm_delay;
uint32_t cur_state_end, prev_state_end, sleep_timer;
enum STATE prev_state, cur_state, next_state;
enum MODE prev_mode, cur_mode, default_mode;
char * announce_buffer;
char msg_buffer[MSG_BUFFER_SIZE];
char menu[] = {'S', 'W', 'R', 'V', 'K', '\0'};
enum TUNERATE tune_rate = FAST;
uint16_t tune_step = DDS_100HZ;
uint8_t tune_freq_step = 25;
uint16_t st_freq, prev_st_freq;

// Global variables used in ISRs
volatile uint32_t timer, cur_timer;
//volatile uint16_t fc_ovf, fc_count, fc_period;
volatile uint8_t ind;
volatile uint8_t port_b_latch, port_d_latch;
volatile unsigned long freq;
volatile enum BOOL sidetone_on = FALSE;
volatile enum BOOL mute_on = FALSE;
volatile enum BOOL key_down = FALSE;
volatile enum BOOL fc_done = FALSE;
volatile enum BOOL dit_active, dah_active;
volatile enum BOOL allow_sleep = TRUE;
volatile enum BTN cmd_btn, msg_btn, both_btn, enc_btn;
volatile enum BOOL enc_a, enc_b;
volatile enum INCTUNE inc_tune_state;
volatile enum FREQREG tune_reg;
volatile uint32_t tx_start, tx_end, mute_start, mute_end, led_toggle;
volatile uint32_t st_phase_acc, st_tune_word;
volatile uint8_t st_sine_lookup;
volatile uint32_t dds_freq_word, dds_it_freq_word;
volatile uint32_t tune_freq;
volatile int16_t it_delta;

// EEPROM variables
uint8_t EEMEM ee_wpm = DEFAULT_WPM;
enum BOOL EEMEM ee_keyer = TRUE;
char EEMEM ee_msg_mem_1[MSG_BUFFER_SIZE - 1] = "CQ CQ CQ DE NT7S NT7S K";

// Function prototypes
void set_wpm(uint8_t);
void init(void);
void debounce(enum BOOL);
void announce(char * msg, uint16_t freq, uint8_t speed);
void read_voltage(void);
void count_frequency(void);
void poll_buttons(void);
void tune_dds(uint32_t dds_word, enum FREQREG reg, enum BOOL init);
//void init_dds(uint32_t, enum FREQREG);
void send_dds_word(uint16_t);
void set_dds_freq_reg(enum FREQREG reg);
void set_st_freq(uint32_t);


// Timer1 ISR
//
// Timer1 is the sinewave generator.
ISR(TIMER1_COMPA_vect)
{
	if(sidetone_on == TRUE)
	{
		//SIDETONE_DDR |= _BV(SIDETONE);

		st_phase_acc = st_phase_acc + st_tune_word;
		st_sine_lookup = (uint8_t)(st_phase_acc >> 24);
		OCR0A = pgm_read_byte_near(&sinewave[st_sine_lookup]); // Just use the upper 8 bits for sine lookup
	}
	/*
	else
	{
		// Hi-Z the port when not using
		SIDETONE_DDR &= ~(_BV(SIDETONE));
		OCR0A = 0;
	} */
}

// Timer1 ISR
//
// Timer1 is used as the frequency counter. The only thing we need to do during this ISR is
// capture the number of timer overflows as a "17th bit" for the counter.
/*
ISR(TIMER1_OVF_vect)
{
	//fc_ovf++;
}
*/

// Timer2 ISR
//
// Fires every 1 ms. Used as a main system clock, for frequency counting, and handles the
// mute and transmit ports.
ISR(TIMER2_COMPA_vect)
{
	/*
	uint8_t sreg;

	fc_period++;

	if(fc_period >= FC_GATE_TIME)
	{
		fc_done = TRUE;
		sreg = SREG;
		fc_count = TCNT1;
		SREG = sreg;
		TCNT1 = 0;
		freq = (((fc_ovf * 0x10000) + fc_count) * (1000 / FC_GATE_TIME)) + IF_FREQ;

		fc_period = 0;
		fc_ovf = 0;
	}
	*/

	// Handle mute
	if(/*((timer > mute_start) && (timer < mute_end)) || */(mute_on == TRUE))
		MUTE_PORT |= _BV(MUTE);
	else
		MUTE_PORT &= ~(_BV(MUTE));

	// Handle transmit
	if((key_down == TRUE) && (timer < tx_end) && (timer > tx_start))
	{
		if(inc_tune_state == RIT || inc_tune_state == XIT)
			set_dds_freq_reg(REG_1);
		else
			set_dds_freq_reg(REG_0);
		TX_PORT |= _BV(TX);
	}
	else
	{
		set_dds_freq_reg(tune_reg);
		TX_PORT &= ~(_BV(TX));
	}

	// Handle the RIT/XIT LED
	if(inc_tune_state != NONE)
	{
		if(inc_tune_state == RIT)
		{
			//RIT_LED_DDR |= _BV(RIT_LED);
			RIT_LED_PORT |= _BV(RIT_LED);
		}
		else if(inc_tune_state == XIT)
		{
			if(cur_timer > led_toggle)
			{
				RIT_LED_PORT ^= _BV(RIT_LED);
				led_toggle = cur_timer + XIT_BLINK;
			}
		}

	}

	debounce(FALSE);

	// Need to consider timer overflow?
	timer++;
}

// Just needed to wake up on pin change
ISR(PCINT2_vect)
{
	// Stop any more pin change interrupts
	PCICR = 0;

	cur_mode = default_mode;
	cur_state = IDLE;

	// Needs some idle time to get up to speed
	cur_state_end = cur_timer + 100;
	sleep_timer = cur_timer + SLEEP_DELAY;
}

void init(void)
{
	// Disable interrupts
	cli();

	// Osc is 16 MHz, configure system clock for /1
	CLKPR = _BV(CLKPCE);
	CLKPR = 0;

	// Setup Timer0 as phase correct PWM
	TCCR0A = _BV(COM0A1) | _BV(WGM00); // Set for Phase Correct PWM mode, output on OC0A
	TCCR0B = _BV(CS00); // Prescaler /1
	//TCCR0B = _BV(CS01);
	//TIMSK0 |= _BV(TOIE0); // Enable Timer0 CTC overflow interrupt

	// Setup Timer1 as sample rate generator for sidetone
	TCCR1B = _BV(WGM12) | _BV(CS10); // Set for CTC mode, Prescaler /1
	TCCR1A = 0;
	OCR1A = 1000; // 16 MHz clock / 16 kHz sample rate = 1000
	TIMSK1 = _BV(OCIE1A);

	/*
	TCCR1A = 0; // Normal mode
	//TCCR1B = 0; // No counting for now
	TCCR1B = _BV(CS12) | _BV(CS11) | _BV(CS10); // Ext. clock source on T1, rising edge
	TIMSK1 = _BV(TOIE1); // Enable overflow interrupt
	*/

	// Setup Timer2 as main event timer, 4 us tick
	TCCR2A = _BV(WGM21); // Set for CTC mode
	//TCCR2B = _BV(CS21) | _BV(CS20); // Prescaler /32 for 8 MHz clock
	TCCR2B = _BV(CS22); // Prescaler /64 for 16 MHz clock
	//TCCR2B = _BV(CS22) | _BV(CS20);
	TIMSK2 |= _BV(OCIE2A); // Enable Timer2 CTC interrupt
	OCR2A = TIMER2_COUNT; // Timer2 CTC A value

	// Setup ADC
	ADCSRA |= _BV(ADPS2) | _BV(ADEN); // Prescaler /16, enable ADC
	ADMUX = _BV(REFS0) | _BV(ADLAR);  // AREF ref voltage, left adjust result, read channel 0

	// Setup pin change interrupts on paddle inputs and buttons
	//PCMSK2 = _BV(PCINT18);
	PCMSK0 = _BV(ENC_A_PC);
	PCMSK2 = _BV(PADDLE_DIT_PC) | _BV(PADDLE_DAH_PC) | _BV(CMD_BUTTON_PC) | _BV(MSG_BUTTON_PC) |_BV(ENC_B_PC) | _BV(ENC_BUTTON_PC);
	//PCICR = _BV(PCIE2);

	// Configure output ports
	SIDETONE_DDR |= _BV(SIDETONE);
	MUTE_DDR |= _BV(MUTE);
	TX_DDR |= _BV(TX);
	RIT_LED_DDR &= ~(_BV(RIT_LED));
	RIT_LED_PORT &= ~(_BV(RIT_LED));

	// Configure input ports
	PADDLE_DIT_DDR &= ~(_BV(PADDLE_DIT));
	PADDLE_DIT_PORT |= _BV(PADDLE_DIT); // Enable pull-up

	PADDLE_DAH_DDR &= ~(_BV(PADDLE_DAH));
	PADDLE_DAH_PORT |= _BV(PADDLE_DAH); // Enable pull-up

	CMD_BUTTON_DDR &= ~(_BV(CMD_BUTTON));
	CMD_BUTTON_PORT |= _BV(CMD_BUTTON); // Enable pull-up

	MSG_BUTTON_DDR &= ~(_BV(MSG_BUTTON));
	MSG_BUTTON_PORT |= _BV(MSG_BUTTON); // Enable pull-up

	ENC_A_DDR &= ~(_BV(ENC_A));
	ENC_A_PORT |= _BV(ENC_A); // Enable pull-up

	ENC_B_DDR &= ~(_BV(ENC_B));
	ENC_B_PORT |= _BV(ENC_B); // Enable pull-up

	ENC_BUTTON_DDR &= ~(_BV(ENC_BUTTON));
	ENC_BUTTON_PORT |= _BV(ENC_BUTTON); // Enable pull-up

	// Configure SPI
	uint8_t spi_data;
	SPI_DDR |= _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_SS) | _BV(SPI_FSYNC);
	SPCR = _BV(SPE) | _BV(MSTR) |_BV(CPOL) | _BV(SPR0);
	//SPI_PORT |= _BV(SPI_SS);

	spi_data = SPSR; // Dummy read to clear interrupt flag
	spi_data = SPDR;

	// Power saving
	power_twi_disable();
	power_usart0_disable();

	set_sleep_mode(SLEEP_MODE_STANDBY);

	// Initialize global variables
	prev_state = IDLE;
	cur_state = IDLE;
	next_state = IDLE;

	timer = 0;

	//dds_freq_word = 0x05DA5119;
	dds_freq_word = DDS_INIT;
	tune_freq = FREQ_INIT;
	tune_dds(dds_freq_word, REG_0, TRUE);
	tune_dds(dds_freq_word, REG_1, FALSE);

	st_freq = ST_DEFAULT;
	set_st_freq(st_freq);

	inc_tune_state = OFF;
	tune_reg = REG_0;

	// Check to see if we should startup in straight key mode
	for (uint8_t i = 0; i < DEBOUNCE_PRESS_TIME + 10; i++)
		debounce(FALSE);

	eeprom_busy_wait();
	wpm = eeprom_read_byte(&ee_wpm);
	set_wpm(wpm);

	eeprom_busy_wait();
	if(eeprom_read_byte(&ee_keyer) == FALSE)
		cur_mode = SK;
	else
		cur_mode = KEYER;

	if((dah_active == TRUE) && (dit_active == FALSE))
		cur_mode = SK;

	// Enable interrupts
	sei();
}

void set_wpm(uint8_t new_wpm)
{
	// Dit length in milliseconds is 1200 ms / WPM
	// then divide that by the 1 ms per timer tick
	dit_length = (1200 / new_wpm);
}

void debounce(enum BOOL flush)
{

	static uint16_t dit_on_count, dah_on_count, dit_off_count, dah_off_count, cmd_on_count, msg_on_count, both_on_count;
	static uint16_t enca_on_count, enca_off_count, encb_on_count, encb_off_count, enc_on_count;

	if(flush == TRUE)
	{
		dit_on_count = 0;
		dah_on_count = 0;
		dit_off_count = 0;
		dah_off_count = 0;
		cmd_on_count = 0;
		msg_on_count = 0;
		both_on_count = 0;
		enc_on_count = 0;
		enca_on_count = 0;
		encb_on_count = 0;
	}

	// Debounce DIT
	if(bit_is_clear(PADDLE_DIT_PIN, PADDLE_DIT))
	{
		if(dit_on_count < DEBOUNCE_PRESS_TIME)
			dit_on_count++;
		dit_off_count = 0;
	}
	else
	{
		if(dit_off_count < DEBOUNCE_PRESS_TIME)
			dit_off_count++;
		dit_on_count = 0;
	}

	// Debounce DAH
	if(bit_is_clear(PADDLE_DAH_PIN, PADDLE_DAH))
	{
		if(dah_on_count < DEBOUNCE_PRESS_TIME)
			dah_on_count++;
		dah_off_count = 0;
	}
	else
	{
		if(dah_off_count < DEBOUNCE_PRESS_TIME)
			dah_off_count++;
		dah_on_count = 0;
	}

	// Set button flags according to final debounce count
	if(dit_on_count >= DEBOUNCE_PRESS_TIME)
		dit_active = TRUE;
	if(dit_off_count >= DEBOUNCE_PRESS_TIME)
		dit_active = FALSE;

	if(dah_on_count >= DEBOUNCE_PRESS_TIME)
		dah_active = TRUE;
	if(dah_off_count >= DEBOUNCE_PRESS_TIME)
		dah_active = FALSE;


	// Debounce both control buttons
	if((bit_is_clear(CMD_BUTTON_PIN, CMD_BUTTON)) && bit_is_clear(MSG_BUTTON_PIN, MSG_BUTTON))
		both_on_count++;
	else
	{
		if((both_on_count >= DEBOUNCE_PRESS_TIME) && (both_on_count < DEBOUNCE_HOLD_TIME))
			both_btn = PRESS;
		else if(both_on_count >= DEBOUNCE_HOLD_TIME)
			both_btn = HOLD;
		else
			both_btn = OFF;

		both_on_count = 0;
	}


	// Debounce CMD/FREQ button
	if(bit_is_clear(CMD_BUTTON_PIN, CMD_BUTTON))
	{
		cmd_on_count++;
		//mute_on = TRUE;
	}
	else
	{
		if((cmd_on_count >= DEBOUNCE_PRESS_TIME) && (cmd_on_count < DEBOUNCE_HOLD_TIME))
			cmd_btn = PRESS;
		else if(cmd_on_count >= DEBOUNCE_HOLD_TIME)
			cmd_btn = HOLD;
		else
			cmd_btn = OFF;

		cmd_on_count = 0;
	}

	// Debounce MSG/OK button
	if(bit_is_clear(MSG_BUTTON_PIN, MSG_BUTTON))
		msg_on_count++;
	else
	{
		if((msg_on_count >= DEBOUNCE_PRESS_TIME) && (msg_on_count < DEBOUNCE_HOLD_TIME))
			msg_btn = PRESS;
		else if(msg_on_count >= DEBOUNCE_HOLD_TIME)
			msg_btn = HOLD;
		else
			msg_btn = OFF;

		msg_on_count = 0;
	}

	// Debounce encoder button
	if(bit_is_clear(ENC_BUTTON_PIN, ENC_BUTTON))
		enc_on_count++;
	else
	{
		if((enc_on_count >= DEBOUNCE_PRESS_TIME) && (enc_on_count < DEBOUNCE_HOLD_TIME))
			enc_btn = PRESS;
		else if(enc_on_count >= DEBOUNCE_HOLD_TIME)
			enc_btn = HOLD;
		else
			enc_btn = OFF;

		enc_on_count = 0;
	}


	// Debounce Encoder A
	if(bit_is_clear(ENC_A_PIN, ENC_A))
	{
		if(enca_on_count < DEBOUNCE_PRESS_TIME)
			enca_on_count++;
		enca_off_count = 0;
	}
	else
	{
		if(enca_off_count < DEBOUNCE_PRESS_TIME)
			enca_off_count++;
		enca_on_count = 0;
	}


	// Debounce Encoder B
	if(bit_is_clear(ENC_B_PIN, ENC_B))
	{
		if(encb_on_count < DEBOUNCE_PRESS_TIME)
			encb_on_count++;
		encb_off_count = 0;
	}
	else
	{
		if(encb_off_count < DEBOUNCE_PRESS_TIME)
			encb_off_count++;
		encb_on_count = 0;
	}

	// Set encoder flags
	if(enca_on_count >= DEBOUNCE_PRESS_TIME)
		enc_a = TRUE;
	if(enca_off_count >= DEBOUNCE_PRESS_TIME)
		enc_a = FALSE;

	if(encb_on_count >= DEBOUNCE_PRESS_TIME)
		enc_b = TRUE;
	if(encb_off_count >= DEBOUNCE_PRESS_TIME)
		enc_b = FALSE;

	/*
	// Don't go to sleep if there are any paddle or button presses
	if((dit_on_count > 0) || (dah_on_count > 0) || (cmd_on_count > 0) || (msg_on_count > 0) || (both_on_count > 0))
		allow_sleep = FALSE;
	else
		allow_sleep = TRUE;
		*/
}

void announce(char * msg, uint16_t freq, uint8_t speed)
{
	// Convert to uppercase
	strupr(msg);

	// Need buffer overflow checking here
	strcpy(announce_buffer, msg);

	// Retain the current state and mode
	prev_state = cur_state;
	prev_state_end = cur_state_end;
	prev_mode = cur_mode;
	prev_st_freq = st_freq;
	st_freq = freq;
	prev_wpm = wpm;
	wpm = speed;

	set_st_freq(st_freq);
	set_wpm(wpm);

	// Set into announce mode
	cur_state = IDLE;
	cur_mode = ANNOUNCE;
}

void read_voltage(void)
{
	uint16_t vcc, vcc_mon;
	char *vcc_out;

	vcc_out = malloc(10);

	// Start ADC conversion
	ADCSRA |= _BV(ADSC);

	// Wait for ADC conversion to finish
	loop_until_bit_is_clear(ADCSRA, ADSC);

	// Get ADC value
	vcc_mon = ADCH;

	// Full scale reading at uC is 16.1 V
	// We'll use fixed point numbers, so full scale is 161 * 0.1 V
	vcc = (vcc_mon * 161) / 256;

	// Format for output
	sprintf(vcc_out, "%dR%d", vcc / 10, vcc % 10);

	announce(vcc_out, st_freq, wpm);

	free(vcc_out);
}

void count_frequency(void)
{
	char *freq_out;

	freq_out = malloc(15);

	// Format and output frequency
	if(inc_tune_state == XIT || inc_tune_state == RIT)
	{
		if(it_delta < 0)
			sprintf(freq_out, "-%1iR%2.2i", (int)abs(((it_delta / 1000) % 100)), (int)abs(((it_delta % 1000) / 10)));
		else
			sprintf(freq_out, "%1iR%2.2i", (int)((it_delta / 1000) % 100), (int)abs(((it_delta % 1000) / 10)));
	}
	else
		sprintf(freq_out, "%02uR%02u", (unsigned int)((tune_freq / 1000) % 100), (unsigned int)((tune_freq % 1000) / 10));
	announce(freq_out, st_freq, wpm);

	free(freq_out);
}

void poll_buttons(void)
{
	static uint8_t prev_enc_state;

	// Handle buttons
	if(both_btn == HOLD)
	{
		cur_state = IDLE;
		prev_mode = cur_mode;
		cur_mode = TUNE;
		sleep_timer = cur_timer + SLEEP_DELAY;
	}
	else if(cmd_btn == PRESS)
	{
		prev_mode = cur_mode;
		cur_mode = MENU;
		cur_state = IDLE;
		sleep_timer = cur_timer + SLEEP_DELAY;
	}
	else if(msg_btn == PRESS)
	{
		count_frequency();
		sleep_timer = cur_timer + SLEEP_DELAY;
	}
	else if(msg_btn == HOLD)
	{
		// Playback message memory 1
		eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, 40);
		strcpy(announce_buffer, msg_buffer);
		cur_state = IDLE;
		prev_mode = cur_mode;
		cur_mode = PLAYBACK;
		sleep_timer = cur_timer + SLEEP_DELAY;
	}

	if(enc_btn == PRESS)
	{
		// If we are in normal tuning mode, pressing the tune knob toggles tuning rates
		if(inc_tune_state == NONE)
		{
			if(tune_rate == FAST)
			{
				tune_rate = SLOW;
				tune_step = DDS_20HZ;
				tune_freq_step = 5;
				sleep_timer = cur_timer + SLEEP_DELAY;
				debounce(TRUE);
				announce("S", ST_LOW, 25);
			}
			else
			{
				tune_rate = FAST;
				tune_step = DDS_100HZ;
				tune_freq_step = 25;
				sleep_timer = cur_timer + SLEEP_DELAY;
				debounce(TRUE);
				announce("S", ST_HIGH, 25);
			}
		}
		// Otherwise if we are in RIT or XIT, pressing the tune knob toggles between the two VFOs
		else
		{
			if(tune_reg == REG_0)
			{
				tune_reg = REG_1;
				announce("T", ST_LOW, 25);
			}
			else
			{
				tune_reg = REG_0;
				announce("R", ST_LOW, 25);
			}
		}
	}
	else if(enc_btn == HOLD)
	{
		// Rotate through the 3 states
		inc_tune_state++;
		if(inc_tune_state > 2)
			inc_tune_state = NONE;

		switch(inc_tune_state)
		{
			case RIT:
				RIT_LED_DDR |= _BV(RIT_LED);
				//RIT_LED_PORT |= _BV(RIT_LED);
				tune_rate = SLOW;
				tune_step = DDS_20HZ;
				tune_freq_step = 5;
				dds_it_freq_word = dds_freq_word;
				tune_dds(dds_it_freq_word, REG_0, FALSE);
				tune_dds(dds_it_freq_word, REG_1, FALSE);
				tune_reg = REG_0;
				set_dds_freq_reg(tune_reg);
				it_delta = 0;
				debounce(TRUE);
				sleep_timer = cur_timer + SLEEP_DELAY;
				announce("R", ST_HIGH, 25);
				break;

			case XIT:
				RIT_LED_DDR |= _BV(RIT_LED);
				//RIT_LED_PORT |= _BV(RIT_LED);
				led_toggle = cur_timer + XIT_BLINK;
				tune_rate = FAST;
				tune_step = DDS_100HZ;
				tune_freq_step = 25;
				tune_dds(dds_it_freq_word, REG_0, FALSE);
				tune_dds(dds_it_freq_word, REG_1, FALSE);
				tune_reg = REG_1;
				set_dds_freq_reg(tune_reg);
				it_delta = 0;
				debounce(TRUE);
				sleep_timer = cur_timer + SLEEP_DELAY;
				announce("X", ST_HIGH, 25);
				break;

			case NONE:
			default:
				RIT_LED_DDR &= ~(_BV(RIT_LED));
				RIT_LED_PORT &= ~(_BV(RIT_LED));
				tune_rate = FAST;
				tune_step = DDS_100HZ;
				tune_freq_step = 25;
				dds_freq_word = dds_it_freq_word;
				tune_dds(dds_freq_word, REG_0, FALSE);
				tune_dds(dds_freq_word, REG_1, FALSE);
				tune_reg = REG_0;
				set_dds_freq_reg(tune_reg);
				it_delta = 0;
				debounce(TRUE);
				sleep_timer = cur_timer + SLEEP_DELAY;
				announce("O", ST_HIGH, 25);
				break;
		}
		/*
		if(rit_enable == FALSE)
		{
			RIT_LED_DDR |= _BV(RIT_LED);
			RIT_LED_PORT |= _BV(RIT_LED);
			rit_enable = TRUE;
			dds_rit_freq_word = dds_freq_word;
			tune_dds(dds_rit_freq_word, REG_1, FALSE);
			debounce(TRUE);
			sleep_timer = cur_timer + SLEEP_DELAY;
		}
		else
		{

			RIT_LED_DDR &= ~(_BV(RIT_LED));
			RIT_LED_PORT &= ~(_BV(RIT_LED));
			rit_enable = FALSE;
			dds_freq_word = dds_rit_freq_word;
			tune_dds(dds_freq_word, REG_0, FALSE);
			debounce(TRUE);
			sleep_timer = cur_timer + SLEEP_DELAY;
		} */
	}

	// Handle encoder
	uint8_t cur_enc_state = 0;

	// Set bits representing current encoder state
	if(enc_a)
		cur_enc_state += 0x02;
	if(enc_b)
		cur_enc_state += 0x01;

	// If the current state is different from previous state, the encoder has moved
	if(cur_enc_state != prev_enc_state)
	{
		prev_enc_state = (prev_enc_state >> 1) & 0x01;
		sleep_timer = cur_timer + SLEEP_DELAY;

		// Compare current B state to previous A state
		if((prev_enc_state ^ (cur_enc_state & 0x01)) == 1)
		{
			// Don't allow tuning if we are on the locked VFO
			if((inc_tune_state == RIT && tune_reg == REG_0) || (inc_tune_state == XIT && tune_reg == REG_1) || (inc_tune_state == NONE))
			{

				if(tune_freq > LOWER_FREQ_LIMIT)
				{
					dds_freq_word -= tune_step;
					if(inc_tune_state == XIT || inc_tune_state == RIT)
						it_delta -= tune_freq_step;
					else
						tune_freq -= tune_freq_step;
					tune_dds(dds_freq_word, tune_reg, FALSE);
					set_dds_freq_reg(tune_reg);
				}
				else
				{
					announce("L", ST_HIGH, 25);
					debounce(TRUE);
				}
			}
		}
		else
		{
			// Don't allow tuning if we are on the locked VFO
			if((inc_tune_state == RIT && tune_reg == REG_0) || (inc_tune_state == XIT && tune_reg == REG_1) || (inc_tune_state == NONE))
			{

				// Tune up as long as we are not at upper limit
				if(tune_freq < UPPER_FREQ_LIMIT)
				{
					dds_freq_word += tune_step;
					if(inc_tune_state == XIT || inc_tune_state == RIT)
						it_delta += tune_freq_step;
					else
						tune_freq += tune_freq_step;
					tune_dds(dds_freq_word, tune_reg, FALSE);
					set_dds_freq_reg(tune_reg);
				}
				else
				{
					announce("U", ST_HIGH, 25);
					debounce(TRUE);
				}
			}
			/*
			if(tune_freq == LOWER_FREQ_LIMIT)
				announce("L", ST_HIGH, 23);
			else if(tune_freq == UPPER_FREQ_LIMIT)
				announce("U", ST_HIGH, 23);
				*/
		}
	}

	prev_enc_state = cur_enc_state;
}

void tune_dds(uint32_t dds_word, enum FREQREG reg, enum BOOL init)
{
	uint16_t dds_word_high, dds_word_low, freq_reg;

	if(reg == REG_1)
		freq_reg = 0x8000;
	else
		freq_reg = 0x4000;

	dds_word_low = (uint16_t)((dds_word & 0x3FFF) + freq_reg);
	dds_word_high = (uint16_t)(((dds_word >> 14) & 0x3FFF) + freq_reg);

	if(init == TRUE)
		send_dds_word(0x2100);

	// Send frequency word LSB
	send_dds_word(dds_word_low);

	// Send frequency word MSB
	send_dds_word(dds_word_high);

	if(init == TRUE)
	{
		// Send phase
		send_dds_word(0xC000);

		// Exit reset
		send_dds_word(0x2000);
	}
}

/*
void init_dds(uint32_t dds_word, enum FREQREG reg)
{
	uint16_t dds_word_high, dds_word_low, freq_reg;

	if(reg == REG_1)
		freq_reg = 0x8000;
	else
		freq_reg = 0x4000;

	dds_word_low = (uint16_t)((dds_word & 0x3FFF) + freq_reg);
	dds_word_high = (uint16_t)(((dds_word >> 14) & 0x3FFF) + freq_reg);

	// Control register
	//if(reg == REG_1)
		//send_dds_word(0x2900);
	//else
		send_dds_word(0x2100);

	// Send frequency word LSB
	send_dds_word(dds_word_low);

	// Send frequency word MSB
	send_dds_word(dds_word_high);

	// Send phase
	send_dds_word(0xC000);

	// Exit reset
	send_dds_word(0x2000);
}
*/

void send_dds_word(uint16_t dds_word)
{
	SPI_PORT |= _BV(SPI_SCK);
	SPI_PORT &= ~(_BV(SPI_FSYNC));
	SPDR = (uint8_t)((dds_word >> 8) & 0xFF);
	while(!(SPSR & (1<<SPIF)));
	SPDR = (uint8_t)(dds_word & 0xFF);
	while(!(SPSR & (1<<SPIF)));
	SPI_PORT |= _BV(SPI_FSYNC);
}

void set_dds_freq_reg(enum FREQREG reg)
{
	// Control register
	if(reg == REG_1)
		send_dds_word(0x2800);
	else
		send_dds_word(0x2000);
}

void set_st_freq(uint32_t st_freq)
{
	st_tune_word = st_freq  * ST_REFCLK; // A way to avoid 64-bit math, ST_REFCLK is 1/(2^32/REFCLK)
}

int main(void)
{
	//static uint32_t cur_timer = 0;
	static uint32_t rec_timeout;
	static uint8_t cur_character = '\0';
	static uint8_t rec_input, rec_count;
	static char * cur_char_p;
	static char * cur_menu_p;
	static char * cur_menu;
	static char * text_buffer;
	static uint8_t val_index;

	announce_buffer = malloc(ANNOUNCE_BUFFER_SIZE);
	memset(announce_buffer, '\0', ANNOUNCE_BUFFER_SIZE);
	cur_char_p = announce_buffer;

	text_buffer = malloc(MSG_BUFFER_SIZE);
	memset(text_buffer, '\0', MSG_BUFFER_SIZE);

	init();

	announce("CC", st_freq, 15);

	// Main event loop
	while(1)
	{
		// Latch the current time
		// MUST disable interrupts during this read or there will be an occasional corruption of cur_timer
		cli();
		cur_timer = timer;
		sei();

		// Handle the current mode
		switch(cur_mode)
		{
		case SK:
			default_mode = SK;
			poll_buttons();

			switch(cur_state)
			{
			case IDLE:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;
				/*
				if(allow_sleep == TRUE)
					mute_on = FALSE;
				else
					mute_on = TRUE;
					*/

				if(dit_active == TRUE)
				{
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = UINT32_MAX;
					cur_state_end = UINT32_MAX;
					cur_state = KEYDOWN;
				}
				else
				{
					cur_state = IDLE;
				}
				break;

			case KEYDOWN:
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
				{
					key_down = FALSE;
					sidetone_on = FALSE;
					mute_on = FALSE;
				}
				else
				{
					key_down = TRUE;
					sidetone_on = TRUE;
					mute_on = TRUE;
				}

				if(dit_active == FALSE)
				{
					cur_state = EXIT;
					cur_state_end = cur_timer + MUTE_OFF_DELAY;
					tx_end = cur_timer;
				}
				else
					tx_end = UINT32_MAX;
				break;

			case EXIT:
				key_down = FALSE;
				sidetone_on = FALSE;

				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
					mute_on = FALSE;
				else
					mute_on = TRUE;

				if(cur_timer >= cur_state_end)
					cur_state = IDLE;
				break;

			default:
				break;
			}

			/*
			// Go to sleep
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			cli();
			if((cur_mode == KEYER) && (cur_state == IDLE) && (cur_timer > sleep_timer))
			{
				MUTE_PORT &= ~(_BV(MUTE));
				PCICR = _BV(PCIE2);
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
			}
			sei();
			*/

			break;

		case KEYER:
			default_mode = KEYER;
			poll_buttons();

			// Handle KEYER state conditions
			switch(cur_state)
			{
			case IDLE:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;
				mute_end = cur_timer;

				// Dit paddle only
				if((dit_active == TRUE) && (dah_active == FALSE))
				{
					prev_state = IDLE;
					cur_state = DIT;
					next_state = IDLE;
					cur_state_end = cur_timer + dit_length;
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = cur_state_end;
					mute_start = cur_timer;
					mute_end = UINT32_MAX;
				}
				// Dah paddle only
				else if((dah_active == TRUE) && (dit_active == FALSE))
				{
					prev_state = IDLE;
					cur_state = DAH;
					next_state = IDLE;
					cur_state_end = cur_timer + (dit_length * 3);
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = cur_state_end;
					mute_start = cur_timer;
					mute_end = UINT32_MAX;
				}
				// Dit and dah paddle at same time (rare case)
				else if((dit_active == TRUE) && (dah_active == TRUE) && (next_state == IDLE))
				{
					prev_state = IDLE;
					cur_state = DIT;
					next_state = DAH;
					cur_state_end = cur_timer + dit_length;
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = cur_state_end;
					mute_start = cur_timer;
					mute_end = UINT32_MAX;
				}
				else
				{
					cur_state = IDLE;
				}

				break;

			case DIT:
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
				{
					key_down = FALSE;
					sidetone_on = FALSE;
					mute_on = FALSE;
				}
				else
				{
					key_down = TRUE;
					sidetone_on = TRUE;
					mute_on = TRUE;
				}

				if(cur_timer > cur_state_end)
				{
					prev_state = DIT;
					cur_state = DITDELAY;
					cur_state_end = cur_timer + dit_length;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}

				if((dah_active == TRUE) && (next_state == IDLE))
					next_state = DAH;

				break;

			case DAH:
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
				{
					key_down = FALSE;
					sidetone_on = FALSE;
					mute_on = FALSE;
				}
				else
				{
					key_down = TRUE;
					sidetone_on = TRUE;
					mute_on = TRUE;
				}

				if(cur_timer > cur_state_end)
				{
					prev_state = DAH;
					cur_state = DITDELAY;
					cur_state_end = cur_timer + dit_length;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}

				if((dit_active == TRUE) && (next_state == IDLE))
					next_state = DIT;

				break;

			case DITDELAY:
				if(cur_timer > cur_state_end)
				{
					if(next_state == DIT)
					{
						cur_state = DIT;
						cur_state_end = cur_timer + dit_length;
						tx_start = cur_timer + TX_ON_DELAY;
						tx_end = cur_state_end;
						mute_start = cur_timer;
						mute_end = UINT32_MAX;
					}
					else if(next_state == DAH)
					{
						cur_state = DAH;
						cur_state_end = cur_timer + (dit_length * 3);
						tx_start = cur_timer + TX_ON_DELAY;
						tx_end = cur_state_end;
						mute_start = cur_timer;
						mute_end = UINT32_MAX;
					}
					else
					{
						cur_state = IDLE;
						cur_state_end = cur_timer;
					}

					prev_state = DITDELAY;
					next_state = IDLE;
				}

				if((dit_active == TRUE) && (prev_state == DAH) && (next_state == IDLE))
					next_state = DIT;
				else if((dah_active == TRUE) && (prev_state == DIT) && (next_state == IDLE))
					next_state = DAH;

				key_down = FALSE;
				sidetone_on = FALSE;
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
					mute_on = FALSE;
				else
					mute_on = TRUE;
				break;

			case EXIT:
				key_down = FALSE;
				sidetone_on = FALSE;
				//mute_on = TRUE;
				mute_on = FALSE;

				if(cur_timer > cur_state_end)
				{
					cur_state = IDLE;
				}

				sleep_timer = cur_timer + SLEEP_DELAY;
				break;

			default:
				break;
			}

			/*
			// Go to sleep
			cli();
			if((cur_mode == KEYER) && (cur_state == IDLE) && (cur_timer > sleep_timer))
			{
				MUTE_PORT &= ~(_BV(MUTE));
				//SIDETONE_DDR &= ~(_BV(SIDETONE));
				PCICR = _BV(PCIE2);
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
			}
			sei();
			*/

			break;

		case ANNOUNCE:
			switch(cur_state)
			{
			case IDLE:
				// If this is the first time thru the ANNOUNCE loop, get the first character
				if((cur_char_p == announce_buffer) && (cur_character == '\0'))
				{
					cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
				}

				// Get the current element in the current character
				if(cur_character != '\0')
				{
					if(cur_character == 0b10000000 || cur_character == 0b11111111)	// End of character marker or SPACE
					{
						// Set next state based on whether EOC or SPACE
						if(cur_character == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = DAHDELAY;
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * 7);
							cur_state = DAHDELAY;
						}

						// Grab next character, set state to inter-character delay
						cur_char_p++;

						// If we read a NULL from the announce buffer, set cur_character to NULL,
						// otherwise set to correct morse character
						if((*cur_char_p) == '\0')
							cur_character = '\0';
						else
							cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
					}
					else
					{
						// Mask off MSb, set cur_element
						if((cur_character & 0b10000000) == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = DAH;
						}
						else
						{
							cur_state_end = cur_timer + dit_length;
							cur_state = DIT;
						}

						// Shift left to get next element
						cur_character = cur_character << 1;
					}
				}
				else
				{
					// Clear the announcement buffer and set buffer pointer back to beginning
					strcpy(announce_buffer, "");
					cur_char_p = announce_buffer;
					cur_character = '\0';

					// Set back into previous mode
					st_freq = prev_st_freq;
					set_st_freq(st_freq);

					wpm = prev_wpm;
					set_wpm(wpm);

					cur_mode = prev_mode;
					cur_state = prev_state;
					cur_state_end = prev_state_end;
				}
				break;

			case DIT:
			case DAH:
				if(cur_timer > cur_state_end)
				{
					cur_state_end = cur_timer + dit_length;
					cur_state = DITDELAY;
				}

				key_down = FALSE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case DITDELAY:
			case DAHDELAY:
			case WORDDELAY:
				if(cur_timer > cur_state_end)
					cur_state = IDLE;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			default:
				break;
			}
			break;

		case TUNE:
			switch(cur_state)
			{
			case IDLE:
				tx_start = cur_timer + TX_ON_DELAY;
				tx_end = UINT32_MAX;
				cur_state_end = UINT32_MAX;
				mute_start = cur_timer;
				mute_end = UINT32_MAX;

				cur_state = KEYDOWN;
				break;

			case KEYDOWN:
				key_down = TRUE;
				sidetone_on = TRUE;
				mute_on = TRUE;

				if(ANYBUTTON)
				{
					cur_state = ENDKEYDOWN;
					cur_state_end = cur_timer + MUTE_OFF_DELAY;
					tx_end = cur_timer;
				}
				break;

			case ENDKEYDOWN:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;

				if(cur_timer >= cur_state_end)
				{
					mute_on = FALSE;
					cur_state = IDLE;
					cur_mode = default_mode;
				}
				break;

			default:
				break;
			}
			break;

		case MENU:
			switch(cur_state)
			{
			case IDLE:
				// Point to the beginning of the menu
				cur_menu_p = menu;
				cur_state = MENUANNOUNCE;
				break;

			case MENUANNOUNCE:
				cur_menu = malloc(2);
				memset(cur_menu, '\0', 2);

				// Get the menu char (just 1 from the array)
				memcpy(cur_menu, cur_menu_p, 1);

				// Set menu input expiration
				cur_state_end = cur_timer + MENU_EXPIRATION;

				// Set next state
				cur_state = MENUINPUT;

				// Announce the menu item
				announce(cur_menu, st_freq, wpm);

				free(cur_menu);
				break;

			case MENUINPUT:
				text_buffer = malloc(20);
				memset(text_buffer, '\0', 20);

				// Wait for input
				if(cur_timer < cur_state_end)
				{
					// If CMD/FREQ pressed, advance to next menu item
					if(cmd_btn == PRESS)
					{
						debounce(TRUE);
						cur_menu_p++;
						// If at end of menu, return to previous mode
						if((*cur_menu_p) == '\0')
						{
							cur_state = IDLE;
							cur_mode = default_mode;

							//set_st_freq(ST_LOW);
							announce("X", ST_LOW, wpm);
						}
						else
							cur_state = MENUANNOUNCE;
					}
					// If MSG/OK pressed, select this menu item
					else if(msg_btn == PRESS)
					{
						// need to clear the button buffer
						debounce(TRUE);

						switch(*cur_menu_p)
						{
						// Change keyer speed
						case 'S':
							cur_state_end = cur_timer + MENU_EXPIRATION;
							cur_mode = SETWPM;

							announce("R", st_freq, wpm);
							break;

						// Read WPM
						case 'W':
							cur_state = IDLE;
							cur_mode = default_mode;

							sprintf(text_buffer, "%d", wpm);
							announce(text_buffer, st_freq, wpm);
							break;

						// Record keyer memory
						case 'R':
							cur_state = INIT;
							cur_mode = RECORD;

							announce("R", st_freq, wpm);
							break;

						// Read voltage
						case 'V':
							cur_state = IDLE;
							cur_mode = default_mode;

							read_voltage();
							break;

						// Toggle keyer/straight key mode
						case 'K':
							if(default_mode == KEYER)
							{
								default_mode = SK;
								cur_state = IDLE;
								cur_mode = default_mode;
								eeprom_busy_wait();
								eeprom_write_byte(&ee_keyer, FALSE);

								announce("S", st_freq, wpm);
							}
							else
							{
								default_mode = KEYER;
								cur_state = IDLE;
								cur_mode = default_mode;
								eeprom_busy_wait();
								eeprom_write_byte(&ee_keyer, TRUE);

								announce("K", st_freq, wpm);
							}
							break;

						default:
							break;
						}
					}
				}
				else // Bail out of menu if past menu expiration
				{
					cur_state = IDLE;
					cur_mode = default_mode;

					// Send "X" to indicate expiration
					//set_st_freq(ST_LOW);
					announce("X", ST_LOW, wpm);
				}

				free(text_buffer);
				break;

			default:
				cur_state = IDLE;
				cur_mode = default_mode;
				break;
			}
			break;

		case SETWPM:
			if(cur_timer < cur_state_end)
			{
				if(cmd_btn == PRESS)
				{
					if(wpm < MAX_WPM)
						wpm++;
					set_wpm(wpm);
					cur_state_end = cur_timer + MENU_EXPIRATION;
					announce("I", st_freq, wpm);
				}
				else if(msg_btn == PRESS)
				{
					if(wpm > MIN_WPM)
						wpm--;
					set_wpm(wpm);
					cur_state_end = cur_timer + MENU_EXPIRATION;
					announce("I", st_freq, wpm);
				}
			}
			else // done setting WPM, announce current setting
			{
				// Save WPM in EEPROM
				eeprom_busy_wait();
				eeprom_write_byte(&ee_wpm, wpm);

				cur_state = IDLE;
				cur_mode = default_mode;

				sprintf(text_buffer, "%d", wpm);
				announce(text_buffer, st_freq, wpm);
			}
			break;

		// Consolidate with ANNOUNCE code
		case PLAYBACK:
			// Cancel playback if any button pressed
			if(ANYBUTTON)
			{
				// Clear the announcement buffer and set buffer pointer back to beginning
				strcpy(announce_buffer, "");
				cur_char_p = announce_buffer;
				cur_character = '\0';

				// Set back into previous mode
				mute_end = cur_timer;
				cur_mode = prev_mode;
				cur_state = prev_state;
				cur_state_end = prev_state_end;
			}

			switch(cur_state)
			{
			case IDLE:
				// If this is the first time thru the PLAYBACK loop, get the first character
				if((cur_char_p == announce_buffer) && (cur_character == '\0'))
				{
					cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
				}

				// Get the current element in the current character
				if(cur_character != '\0')
				{
					if(cur_character == 0b10000000 || cur_character == 0b11111111)	// End of character marker or SPACE
					{
						// Set next state based on whether EOC or SPACE
						if(cur_character == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = DAHDELAY;
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * 7);
							cur_state = DAHDELAY;
						}

						// Grab next character, set state to inter-character delay
						cur_char_p++;

						// If we read a NULL from the announce buffer, set cur_character to NULL,
						// otherwise set to correct morse character
						if((*cur_char_p) == '\0')
							cur_character = '\0';
						else
							cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
					}
					else
					{
						// Mask off MSb, set cur_element
						if((cur_character & 0b10000000) == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = DAH;
							tx_start = cur_timer + TX_ON_DELAY;
							tx_end = cur_state_end;
							mute_start = cur_timer;
							mute_end = UINT32_MAX;
						}
						else
						{
							cur_state_end = cur_timer + dit_length;
							cur_state = DIT;
							tx_start = cur_timer + TX_ON_DELAY;
							tx_end = cur_state_end;
							mute_start = cur_timer;
							mute_end = UINT32_MAX;
						}

						// Shift left to get next element
						cur_character = cur_character << 1;
					}
				}
				else
				{
					// Clear the announcement buffer and set buffer pointer back to beginning
					strcpy(announce_buffer, "");
					cur_char_p = announce_buffer;
					cur_character = '\0';

					// Set back into previous mode
					mute_end = cur_timer;
					cur_mode = prev_mode;
					cur_state = prev_state;
					cur_state_end = prev_state_end;
				}
				break;

			case DIT:
			case DAH:
				if(cur_timer > cur_state_end)
				{
					cur_state_end = cur_timer + dit_length;
					cur_state = DITDELAY;
				}

				key_down = TRUE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case DITDELAY:
			case DAHDELAY:
			case WORDDELAY:
				if(cur_timer > cur_state_end)
					cur_state = IDLE;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			default:
				break;
			}
			break;
		/*
		case RECORD:
			switch(cur_state)
			{
			case INIT:
				mute_start = cur_timer;
				mute_end = UINT32_MAX;

				// Initialize the current recorded character
				rec_input = 0;
				rec_count = 0;
				rec_timeout = UINT32_MAX;

				memset(text_buffer, '\0', MSG_BUFFER_SIZE);

				cur_state = IDLE;
				break;

			case IDLE:
				// Dit paddle only
				if((dit_active == TRUE) && (dah_active == FALSE))
				{
					prev_state = IDLE;
					cur_state = DIT;
					next_state = IDLE;
					cur_state_end = cur_timer + dit_length;
					rec_timeout = cur_timer + REC_EXPIRATION;

					// Add this element to the recorded character
					rec_count++;
					if(rec_count >= 6)
						next_state = VALIDATECHAR;

				}
				// Dah paddle only
				else if((dah_active == TRUE) && (dit_active == FALSE))
				{
					prev_state = IDLE;
					cur_state = DAH;
					next_state = IDLE;
					cur_state_end = cur_timer + (dit_length * 3);
					rec_timeout = cur_timer + REC_EXPIRATION;

					// Add this element to the recorded character
					rec_input = rec_input + (0b10000000 >> rec_count);
					rec_count++;
					if(rec_count >= 6)
						next_state = VALIDATECHAR;
				}
				// Dit and dah paddle at same time (rare case)
				else if((dit_active == TRUE) && (dah_active == TRUE) && (next_state == IDLE))
				{
					prev_state = IDLE;
					cur_state = DIT;
					next_state = DAH;
					cur_state_end = cur_timer + dit_length;
					rec_timeout = cur_timer + REC_EXPIRATION;

					// Add this element to the recorded character
					rec_input = rec_input + (0b10000000 >> (rec_count + 1));
					rec_count += 2;
					if(rec_count >= 6)
						next_state = VALIDATECHAR;
				}

				//else
				//{
				//	cur_state = IDLE;
				//	cur_state_end = cur_timer;
				//}

				// Handle character record timeout
				// Need to handle SPACE
				if((cur_timer > rec_timeout))// && (rec_input > 0))
					cur_state = VALIDATECHAR;

				// If CMD is pressed, we are done recording
				if(cmd_btn == PRESS)
					cur_state = EXIT;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			case DIT:
				if(cur_timer > cur_state_end)
				{
					prev_state = DIT;
					cur_state = DITDELAY;
					cur_state_end = cur_timer + dit_length;
				}

				if((dah_active == TRUE) && (next_state == IDLE))
					next_state = DAH;

				key_down = FALSE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case DAH:
				if(cur_timer > cur_state_end)
				{
					prev_state = DAH;
					cur_state = DITDELAY;
					cur_state_end = cur_timer + dit_length;
				}

				if((dit_active == TRUE) && (next_state == IDLE))
					next_state = DIT;

				key_down = FALSE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case DITDELAY:
				if(cur_timer > cur_state_end)
				{
					if(next_state == DIT)
					{
						cur_state = DIT;
						cur_state_end = cur_timer + dit_length;
					}
					else if(next_state == DAH)
					{
						cur_state = DAH;
						cur_state_end = cur_timer + (dit_length * 3);
					}
					else if(next_state == VALIDATECHAR)
						cur_state = VALIDATECHAR;
					else
						cur_state = IDLE;

					prev_state = DITDELAY;
					next_state = IDLE;
				}

				if((dit_active == TRUE) && (prev_state == DAH) && (next_state == IDLE))
					next_state = DIT;
				else if((dah_active == TRUE) && (prev_state == DIT) && (next_state == IDLE))
					next_state = DAH;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			case VALIDATECHAR:
				// Compare recorded character with the Morse Code table

				// If rec_input is 0, dump to invalid

				// Tack a trailing "1" onto rec_input to indicate end of character
				rec_input = rec_input + (0b10000000 >> rec_count);

				for(val_index = MORSE_CHAR_START; val_index <= 'Z'; val_index++)
				{
					if(rec_input == pgm_read_byte(&morsechar[val_index]))
					{
						// Add recorded character to text buffer
						char temp_str[2] = {val_index, '\0'};
						//temp_str[0] = val_index;
						//temp_str[1] = '\0';
						strcat(text_buffer, temp_str);

						// Reinitialize the current recorded character
						rec_input = 0;
						rec_count = 0;
						cur_state = IDLE;

						// Indicate successful entry
						//pwm_delay = HIGHFREQ_PWM_DELAY;
						//announce("E");
					}
				}

				// If no match, the character isn't valid. Toss it out and announce error
				// No match if rec_input is not reset to 0
				if(rec_input != 0)
				{
					// Reinitialize the current recorded character
					rec_input = 0b10000000;
					rec_count = 0;
					cur_state = IDLE;

					// Indicate an error
					//set_st_freq(ST_LOW);
					announce("X", ST_LOW, wpm);
				}

				cur_state = IDLE;

				break;

			case EXIT:
				// Write the memory to EEPROM
				eeprom_update_block((const void*)&text_buffer, (void*)&ee_msg_mem_1, 40);

				// Unmute and reset back to default mode
				mute_end = cur_timer;
				mute_on = FALSE;
				cur_state = IDLE;
				cur_mode = default_mode;

				// Announce successful recording
				announce("R", st_freq, wpm);
				break;

			default:
				break;
			}
			break;
		*/
		default:
			break;
		} // END switch(cur_mode)

	}
}

