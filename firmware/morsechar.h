/*
 * morsechar.h
 *
 *  Created on: Nov 29, 2010
 *      Author: jason
 */

#ifndef MORSECHAR_H_
#define MORSECHAR_H_

#include <stdint.h>
#include <avr/pgmspace.h>

// The morsechar array maps to the standard ASCII table starting at ASCII 32 (SPACE).
//
// A dit is represented by a "0" bit, while a dah is represented by a "1" bit.
//
// Characters are encoded with the most significant "bit" first so that the byte can be left-shifted
// to read out each element. Each character must be terminated in a "1" so that the reading algorithm
// knows to end when the byte == 0b10000000.

#define MORSE_CHAR_START			32

const uint8_t morsechar[] PROGMEM = {
		0b11111111,		// Special code for SPACE
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b11100000,		// Minus sign (indicated by "M" in this implementation)
		0b10000000,		// Not implemented
		0b10010100,		// "/" Slash
		0b11111100,		// "0"
		0b01111100,		// "1"
		0b00111100,		// "2"
		0b00011100,		// "3"
		0b00001100,		// "4"
		0b00000100,		// "5"
		0b10000100,		// "6"
		0b11000100,		// "7"
		0b11100100,		// "8"
		0b11110100,		// "9"
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10000000,		// Not implemented
		0b10001100,		// "=" BT prosign/Equal sign
		0b10000000,		// Not implemented
		0b00110010,		// "?" Question mark
		0b10000000,		// Not implemented
		0b01100000,		// "A"
		0b10001000,		// "B"
		0b10101000,		// "C"
		0b10010000,		// "D"
		0b01000000,		// "E"
		0b00101000,		// "F"
		0b11010000,		// "G"
		0b00001000,		// "H"
		0b00100000,		// "I"
		0b01111000,		// "J"
		0b10110000,		// "K"
		0b01001000,		// "L"
		0b11100000,		// "M"
		0b10100000,		// "N"
		0b11110000,		// "O"
		0b01101000,		// "P"
		0b11011000,		// "Q"
		0b01010000,		// "R"
		0b00010000,		// "S"
		0b11000000,		// "T"
		0b00110000,		// "U"
		0b00011000,		// "V"
		0b01110000,		// "W"
		0b10011000,		// "X"
		0b10111000,		// "Y"
		0b11001000};	// "Z"

#endif /* MORSECHAR_H_ */
