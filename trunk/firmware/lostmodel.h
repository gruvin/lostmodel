/* Filename: lostmodel.h
 * Project: Lost Model Alarm for AVR ATmega88P
 * Author: Bryan Rentoul (aka Gruvin)
 * Creation Date: 2014-11-22
 * Tabsize: 2
 * Copyright: (c) 2014 Gruvin9X Project and Bryan J. Rentoul aka Gruvin
 * License: GNU GPL v3 (see License.txt)
 * This Revision: $Id$
 *
 * This file is part of Gruvin9X Lost Model Alarm.
 *
 * Gruvin9X Lost Model Alarm is free software: you can redistribute it 
 * and/or modify it under the terms of the GNU General Public License 
 * as published by the Free Software Foundation, either version 3 of 
 * the License, or(at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program in the file Lincese.txt.  If not, please
 * see <http://www.gnu.org/licenses/>
 */
#ifndef _LOSTMODEL_H
#define _LOSTMODEL_H

#define DEBUG 1

// CW dit length (arbitrary units)
#define DITLENGTH 90 // milli-seconds

// TODO: prototype version. Pins changed for production version ...
//       These are NOT the pins specified in the production electronic schematic
#define SIG_INP   PINB
#define SIG_PORT  PORTB
#define SIG_DDR   DDRB
#define SIG_BIT   0 // D-

#define PIEZO_PORT PORTD
#define PIEZO_DDR  DDRD
#define PIEZO_BIT  5 // LED01

#define LED1_PORT PORTC
#define LED1_DDR  DDRC
#define LED1_BIT  2 // LED03 (not used in prototype)

#define LED2_PORT PORTC
#define LED2_DDR  DDRC
#define LED2_BIT  3 // LED02

#define LED3_PORT PORTC
#define LED3_DDR  DDRC
#define LED3_BIT  4 // LED01

#define LED_ON(a) LED ## a ## _PORT |= (1<<LED ## a ## _BIT)
#define LED_OFF(a) LED ## a ## _PORT &= ~(1<<LED ## a ## _BIT)
#define LED_TOGGLE(a) LED ## a ## _PORT ^= (1<<LED ## a ## _BIT)

// input pulse width measurement
// Timer 1 counter clock = 1MHz
#define MIDPOINT 1500 // uS
#define PULSE_TIMEOUT (23000 / 250) // 23mS, in 250uS increments

// PWM piezo output volume
#ifdef DEBUG
#define PWM_DUTY_CYCLE 3
#define PWM_DUTY_CYCLE_QUIET 2
#else
#define PWM_DUTY_CYCLE 27
#define PWM_DUTY_CYCLE_QUIET 10
#endif

#define true (-1)
#define false 0


#endif
