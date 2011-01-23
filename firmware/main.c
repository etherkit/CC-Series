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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/eeprom.h>
//#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
//#include <util/delay.h>

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

#define PADDLE_DAH				PD4			// Dah paddle
#define PADDLE_DAH_PORT			PORTD		// Dah paddle port
#define	PADDLE_DAH_DDR			DDRD		// Dah paddle DDR
#define PADDLE_DAH_PIN			PIND		// Dah paddle pin

#define CMD_BUTTON				PD2			// Command button
#define CMD_BUTTON_PORT			PORTD
#define CMD_BUTTON_DDR			DDRD
#define CMD_BUTTON_PIN			PIND

#define MUTE					PC5			// Mute output
#define MUTE_PORT				PORTC
#define MUTE_DDR				DDRC

#define TX						PC4			// Transmit output
#define TX_PORT					PORTC
#define TX_DDR					DDRC

//#define SIDETONE_PERIOD			7			// 600 Hz freq = 1.66 ms period, (1.66 ms/100 us)/2 = 8 (-1 since count from 0)
#define TIMER2_COUNT			249			// 4 us clk period * 250 ticks (0-249)= 1 ms Timer2 CTC A overflow
#define PWM_DELAY				5			// PWM sidetone freq is 2 kHz, /5 to set to ~650 kHz
#define DEBOUNCE_TIME			5			// Amount of captures for keybounce
#define DEFAULT_WPM				12			// Default keyer WPM
#define TX_ON_DELAY				1			// TX sequence delay time (in 1 ms increments)
#define MUTE_OFF_DELAY			100			// Mute off delay time (in 1 ms increments)
#define ANNOUNCE_BUFFER_SIZE	40			// Buffer size for announce string
#define IF_FREQ					4916000UL	// IF frequency
#define FC_GATE_TIME			100 		// Frequency counter gate time (in 1 ms increments)

// State machine and other enumerations
enum BOOL {FALSE, TRUE};
enum STATE {IDLE, DIT, DAH, DITDELAY, DAHDELAY, WORDDELAY, KEYDOWN, ENDKEYDOWN};
enum MODE {KEYER, SK, ANNOUNCE};

// Global variable defs
uint16_t dit_length;
uint8_t wpm;
enum STATE prev_state, cur_state, next_state;
enum BOOL cmd_active;
char * announce_buffer;

// Global variables used in ISRs
volatile uint32_t timer;
volatile uint16_t fc_ovf, fc_count, fc_period;
volatile uint8_t ind;
volatile unsigned long freq;
volatile enum BOOL sidetone_on = FALSE;
volatile enum BOOL mute_on = FALSE;
volatile enum BOOL key_down = FALSE;
volatile enum BOOL dit_active, dah_active;
volatile enum BOOL fc_done = FALSE;
volatile enum MODE prev_mode, cur_mode;
volatile uint32_t tx_start, tx_end, mute_start, mute_end, fc_count_end;

// Function prototypes
void set_wpm(uint8_t);
void init(void);
void debounce(void);
void announce(char * msg);
void read_voltage(void);
void count_frequency(void);


ISR(TIMER0_OVF_vect)
{
	static uint8_t pwm_delay;

	if(sidetone_on == TRUE)
	{
		if(pwm_delay++ >= PWM_DELAY)
		{
		OCR0A = pgm_read_byte(&sinewave[ind++]);
		if(ind >= PWM_ARRAY_SIZE)
			ind = 0;

		pwm_delay = 0;
		}
	}
	else
	{
		pwm_delay = 0;
		if(OCR0A > 0)
			OCR0A--;
	}
}

ISR(TIMER1_OVF_vect)
{
	fc_ovf++;
}

// Timer2 ISR
ISR(TIMER2_COMPA_vect)
{
	//static uint8_t ind;
	uint8_t sreg;

	cli();

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

	// If the sidetone is on, generate it by flipping port every half-period
	/*
	if((++tone_counter >= SIDETONE_PERIOD) && (sidetone_on == TRUE))
	{
		SIDETONE_PORT ^= _BV(SIDETONE);
		tone_counter = 0;
	}
	else if(sidetone_on == FALSE)
		SIDETONE_PORT &= ~(_BV(SIDETONE));
		*/

	// Handle mute
	if(((timer > mute_start) && (timer < mute_end)) || (mute_on == TRUE))
		MUTE_PORT |= _BV(MUTE);
	else
		MUTE_PORT &= ~(_BV(MUTE));

	// Handle transmit
	if((key_down == TRUE) && (timer < tx_end) && (timer > tx_start))
		TX_PORT |= _BV(TX);
	else
		TX_PORT &= ~(_BV(TX));

	// Need to consider timer overflow
	timer++;

	debounce();

}

void init(void)
{
	// Disable interrupts
	cli();

	// Osc is 8 MHz, configure system clock for /1
	CLKPR = _BV(CLKPCE);
	CLKPR = 0;

	// Setup Timer0 as fast PWM
	TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00); // Set for Fast PWM mode, output on OC0A
	TCCR0B = _BV(CS00); // Prescaler /1
	TIMSK0 |= _BV(TOIE0); // Enable Timer0 CTC overflow interrupt
	//OCR0A = 249; // Fires every 1 ms

	// Setup Timer1 as frequency counter (stopped for now)
	TCCR1A = 0; // Normal mode
	//TCCR1B = 0; // No counting for now
	TCCR1B = _BV(CS12) | _BV(CS11) | _BV(CS10); // Ext. clock source on T1, rising edge
	TIMSK1 = _BV(TOIE1); // Enable overflow interrupt

	// Setup Timer2 as main event timer, 4 us tick
	TCCR2A = _BV(WGM21); // Set for CTC mode
	TCCR2B = _BV(CS22); // Prescaler /64
	TIMSK2 |= _BV(OCIE2A); // Enable Timer2 CTC interrupt
	OCR2A = TIMER2_COUNT; // Timer2 CTC A value

	// Setup ADC
	ADCSRA |= _BV(ADPS2) | _BV(ADEN); // Prescaler /16, enable ADC
	ADMUX = _BV(REFS0) | _BV(ADLAR);  // AREF ref voltage, left adjust result, read channel 0

	// Configure output ports
	SIDETONE_DDR |= _BV(SIDETONE);
	MUTE_DDR |= _BV(MUTE);
	TX_DDR |= _BV(TX);

	// Configure input ports
	PADDLE_DIT_DDR &= ~(_BV(PADDLE_DIT));
	PADDLE_DIT_PORT |= _BV(PADDLE_DIT); // Enable pull-up

	PADDLE_DAH_DDR &= ~(_BV(PADDLE_DAH));
	PADDLE_DAH_PORT |= _BV(PADDLE_DAH); // Enable pull-up

	CMD_BUTTON_DDR &= ~(_BV(CMD_BUTTON));
	CMD_BUTTON_PORT |= _BV(CMD_BUTTON); // Enable pull-up

	// Initialize global variables
	prev_state = IDLE;
	cur_state = IDLE;
	next_state = IDLE;
	timer = 0;
	wpm = DEFAULT_WPM;
	set_wpm(wpm);

	// Enable interrupts
	sei();
}

void set_wpm(uint8_t new_wpm)
{
	// Dit length in milliseconds is 1200 ms / WPM
	// then divide that by the 100 us per timer tick (dividing by 0.1 ms, so multiply by 10)
	dit_length = (1200 / new_wpm);
}

void debounce(void)
{
	static uint8_t dit_on_count, dah_on_count, dit_off_count, dah_off_count, cmd_on_count, cmd_off_count;

	// Debounce buttons
	if(bit_is_clear(PADDLE_DIT_PIN, PADDLE_DIT))
	{
		if(dit_on_count < DEBOUNCE_TIME)
			dit_on_count++;
		dit_off_count = 0;
	}
	else
	{
		if(dit_off_count < DEBOUNCE_TIME)
			dit_off_count++;
		dit_on_count = 0;
	}


	if(bit_is_clear(PADDLE_DAH_PIN, PADDLE_DAH))
	{
		if(dah_on_count < DEBOUNCE_TIME)
			dah_on_count++;
		dah_off_count = 0;
	}
	else
	{
		if(dah_off_count < DEBOUNCE_TIME)
			dah_off_count++;
		dah_on_count = 0;
	}

	if(bit_is_clear(CMD_BUTTON_PIN, CMD_BUTTON))
	{
		cmd_on_count++;
		//cmd_off_count = 0;
	}
	else
	{
		//cmd_off_count++;
		cmd_on_count = 0;
	}

	if(dit_on_count >= DEBOUNCE_TIME)
		dit_active = TRUE;
	if(dit_off_count >= DEBOUNCE_TIME)
		dit_active = FALSE;

	if(dah_on_count >= DEBOUNCE_TIME)
		dah_active = TRUE;
	if(dah_off_count >= DEBOUNCE_TIME)
		dah_active = FALSE;

	if(cmd_on_count >= DEBOUNCE_TIME)
		cmd_active = TRUE;
	else
	//if(cmd_off_count >= DEBOUNCE_TIME)
		cmd_active = FALSE;

	//dit_active = (dit_count > DEBOUNCE_TIME) ? TRUE : FALSE;
	//dah_active = (dah_count > DEBOUNCE_TIME) ? TRUE : FALSE;
}

void announce(char * msg)
{
	// Need to convert to uppercase
	strupr(msg);
	// Need buffer overflow checking here
	strcpy(announce_buffer, msg);
	cur_state = IDLE;
	prev_mode = cur_mode;
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

	// Full scale reading at uC is 15.7 V
	// Well use fixed point numbers, so full scale is 157 * 0.1 V
	vcc = (vcc_mon * 157) / 256;

	// Format for output
	sprintf(vcc_out, "%dR%d", vcc / 10, vcc % 10);

	announce(vcc_out);

	free(vcc_out);
}

void count_frequency(void)
{
	//uint8_t sreg;
	//uint32_t temp_timer;
	//unsigned long freq;
	//uint16_t temp_count;
	char *freq_out;

	freq_out = malloc(15);
	//gate_time = FC_GATE_TIME;

	// Latch the current time
	// MUST disable interrupts during this read or there will be an occasional corruption of temp_timer
	//cli();
	//temp_timer = timer;
	//sei();
	// Set the expiration of the counter based on gate time
	//fc_count_end = temp_timer + FC_GATE_TIME;

	// Clear the counter then start it
	//TCNT1 = 0;
	//fc_period = 0;
	//fc_ovf = 0;
	//fc_done = FALSE;

	//TCCR1B = _BV(CS12) | _BV(CS11) | _BV(CS10); // Ext. clock source on T1, rising edge

	// Wait for counting to complete
	//do {} while(fc_done == FALSE);

	// Turn counter back off
	//TCCR1B = 0;

	// Latch in the counter value and calculate frequency
	/*
	sreg = SREG;
	fc_count = TCNT1;
	SREG = sreg;
	*/

	//freq = (((fc_ovf * 0x10000) + fc_count)) + IF_FREQ; // Need to calculate that constant "10"

	// Format and output frequency
	sprintf(freq_out, "%lu", freq);
	announce(freq_out);

	free(freq_out);
}

int main(void)
{
	static uint32_t cur_state_end = 0;
	static uint32_t cur_timer = 0;
	static uint8_t cur_character = '\0';
	static char * cur_char_p;

	announce_buffer = malloc(ANNOUNCE_BUFFER_SIZE + 1);
	strcpy(announce_buffer, "");
	cur_char_p = announce_buffer;

	init();

	// Check to see if we should startup in straight key mode
	for (uint8_t i = 0; i < DEBOUNCE_TIME + 10; i++)
		debounce();

	if((dah_active == TRUE) && (dit_active == FALSE))
		cur_mode = SK;
	else
		cur_mode = KEYER;

	announce("NT7S");

	// Main event loop
	while(1)
	{
		// Latch the current time
		// MUST disable interrupts during this read or there will be an occasional corruption of cur_timer
		cli();
		cur_timer = timer;
		sei();

		// Conserve power
		power_adc_disable();
		power_spi_disable();
		power_twi_disable();
		power_usart0_disable();

		// Go to sleep
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();

		// Latch button and key states
		//debounce();

		// Handle the current mode
		switch(cur_mode)
		{
		case SK:
			switch(cur_state)
			{
			case IDLE:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;

				if(dit_active == TRUE)
				{
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = UINT32_MAX;
					cur_state_end = UINT32_MAX;
					cur_state = KEYDOWN;
				}
				else
					cur_state = IDLE;
				break;

			case KEYDOWN:
				key_down = TRUE;
				sidetone_on = TRUE;
				mute_on = TRUE;

				if(dit_active == FALSE)
				{
					cur_state = ENDKEYDOWN;
					cur_state_end = cur_timer + MUTE_OFF_DELAY;
					tx_end = cur_timer;
				}
				else
					tx_end = UINT32_MAX;
				break;

			case ENDKEYDOWN:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;

				if(cur_timer >= cur_state_end)
					cur_state = IDLE;
				break;

			default:
				break;
			}
			// Handle command button
			if(cmd_active == TRUE)
				count_frequency();

			break;

		case KEYER:
			// Handle KEYER state conditions
			switch(cur_state)
			{
			case IDLE:
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
					cur_state_end = cur_timer;
				}

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;
				break;

			case DIT:
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

				key_down = TRUE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case DAH:
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

				key_down = TRUE;
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

			default:
				break;
			}

			// Handle command button
			if(cmd_active == TRUE)
				count_frequency();

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
					cur_mode = prev_mode;
					cur_state = IDLE;
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

		default:
			break;
		} // END switch(cur_mode)


	}
}

