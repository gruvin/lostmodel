#ifndef _MAIN_H
#define _MAIN_H

// CW dit length (arbitrary units)
#define DITLENGTH 8

// TODO: prototype version. Pins changed for production version ...
#define SIG_INP   PIND
#define SIG_PORT  PORTD
#define SIG_DDR   DDRD
#define SIG_BIT   3 // D-

#define PIEZO_PORT PORTD
#define PIEZO_DDR  DDRD
#define PIEZO_BIT  5 // LED01

#define LED1_PORT PORTC
#define LED1_DDR  DDRC
#define LED1_BIT  3 // LED05 (not used in prototype)

#define LED2_PORT PORTD
#define LED2_DDR  DDRD
#define LED2_BIT  4 // LED02

#define LED3_PORT PORTC
#define LED3_DDR  DDRC
#define LED3_BIT  5 // LED03

#define LED_ON(a) LED ## a ## _PORT |= (1<<LED ## a ## _BIT)
#define LED_OFF(a) LED ## a ## _PORT &= ~(1<<LED ## a ## _BIT)

#define MIDPOINT 1400

#define true (-1)
#define false 0

// Run Modes
#define NORMAL 0
#define INACTIVITY 1



#endif
