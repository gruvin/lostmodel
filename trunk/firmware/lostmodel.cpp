/* Filename: lostmodel.cpp
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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h> // abs()
#include "lostmodel.h"

void beeperOn()
{
  LED_ON(3);

  // PWM ON
  TCCR0A |= (0b10<<COM0B0);
}

void beeperOff()
{
  LED_OFF(3);

  TCCR0A &= ~(0b11<<COM0B0);
}

const unsigned char morseTable[47] PROGMEM = {
  0b11110011, // , ASCII 44(dec)
  0,
  0b11010101, // .
  0b10110010, // /
  0b10111111, // 0
  0b10101111, // 1
  0b10100111, // 2
  0b10100011, // 3
  0b10100001, // 4
  0b10100000, // 5
  0b10110000, // 6
  0b10111000, // 7
  0b10111100, // 8
  0b10111110, // 9
  0,          // :
  0,          // ;
  0,          // <
  0b10110001, // =
  0,          // >
  0b11001100, // ?
  0,          // @
  0b01001000, // A
  0b10010000, // B
  0b10010100, // C
  0b01110000, // D
  0b00100000, // E
  0b10000100, // F
  0b01111000, // G
  0b10000000, // H
  0b01000000, // I
  0b10001110, // J
  0b01110100, // K
  0b10001000, // L
  0b01011000, // M
  0b01010000, // N
  0b01111100, // O
  0b10001100, // P
  0b10011010, // Q
  0b01101000, // R
  0b01100000, // S
  0b00110000, // T
  0b01100100, // U
  0b10000010, // V
  0b01101100, // W
  0b10010010, // X
  0b10010110, // Y
  0b10011000  // Z
}; // morse table


enum morseState_t { 
  END = 0,
  DIT,
  DAH,
  BITGAP,
  LETTERGAP,
  SPACE,
  NEXTBIT,
  NEXTCHAR,
  START,
  STOP
} morseState = STOP;

char *morseString;
// morse for LM - di'dah'di'dit, dah dah (lost model)
static inline void morseStateMachine() 
{
  static unsigned int timer;
  static unsigned char bitCount;
  static unsigned char letterBits;
  static unsigned char charIndex;
  static char thisChar;

  switch (morseState)
  {
    case DIT:
      if (timer == 0)
        beeperOn();
      if (++timer > DITLENGTH) 
      {
        timer = 0;
        morseState = BITGAP;
      }
      break;

    case DAH:
      if (timer == 0)
        beeperOn();
      if (++timer > DITLENGTH * 3)
      {
        timer = 0;
        morseState = BITGAP;
      }
      break;

    case BITGAP:
      if (timer == 0) 
        beeperOff();
      if (++timer > DITLENGTH) morseState = NEXTBIT;
      break;

    case LETTERGAP:
      if (timer == 0) 
        beeperOff();
      if (++timer > DITLENGTH * 2) morseState = NEXTCHAR;
      break;

    case SPACE:
      if (timer == 0) 
        beeperOff();
      if (++timer > DITLENGTH * 6) morseState = NEXTCHAR;
      break;

    case NEXTBIT:

      timer = 0;

      if (letterBits & 0b10000000)
        morseState = DAH;
      else
        morseState = DIT;

      letterBits <<= 1; // shift bits left 1 position
      if (bitCount-- == 0)
        morseState = LETTERGAP;

      break;

    case NEXTCHAR:

      thisChar = pgm_read_byte(&morseString[charIndex++]);

      if (thisChar == ' ') 
      {
        timer = 0;
        morseState = SPACE;
      }
      else if ( (thisChar == '\0') || (thisChar < ',') || (thisChar > 'Z') ) // end of string or invalid character?  
        morseState = STOP;
      else
      {
        letterBits = pgm_read_byte(&morseTable[thisChar-44]);
        bitCount = (letterBits >> 5);
        if (bitCount < 6)
          letterBits <<= 3;
        else
          letterBits <<= 2;

        morseState = NEXTBIT;
      }
      break;


    case START:
      beeperOff();
      timer = 0;
      charIndex = 0;
      morseState = NEXTCHAR;
      break;

    case STOP:
      break; // do nothing

    default:
      morseState = STOP;
      break;

  } // switch
      
}


#define PULSE_INVALID (-1)
volatile int sigPulseWidth;
volatile unsigned int pulseTimeoutCounter = 0;
// this ISR called every 250uS
ISR(TIMER2_COMPA_vect)
{
  if (pulseTimeoutCounter++ > PULSE_TIMEOUT)
  {
    sigPulseWidth = PULSE_INVALID;  // signal no pulse detected
    pulseTimeoutCounter--; // prevent counter incrementing further
    LED_OFF(2);
  }

  static unsigned char count = 0;
  if ((count++ % 4) == 3)  // call the Morse state machine every 1 millisecond 
    morseStateMachine();
}

ISR(TIMER1_CAPT_vect)
{
  // an edge detection event has occured on the ICP1(PB0) pin
  unsigned int icr;

  icr = ICR1; // capture counter value as at this edge trigger

  static int c = 0;
  if (TCCR1B & (1<<ICES1)) // trigger was high going edge
  {
    TCNT1 = 0;             // reset counter
    TCCR1B &= ~(1<<ICES1); // set next trigger to low going edge
  }
  else // trigger was low going edge
  {
    if ((c++ % 33) == 0) LED_TOGGLE(2); // DEBUG
    if ((icr > 800) && (icr < 2200)) 
    {
      pulseTimeoutCounter = 0;
      sigPulseWidth = icr;
    }
    else
    {
      sigPulseWidth = PULSE_INVALID; // invalid pulse width
      LED_OFF(2);
    }
    TCCR1B |= (1<<ICES1); // set next trigger to high going edge
  }

}

void morseStart()
{
  if (morseState != STOP) return;
  morseState = START;
}

void morseStop()
{
  morseState = STOP;
  beeperOff();
}

void storeRunMode(unsigned char newmode)
{
  eeprom_write_byte(0x00, newmode);
}

int main(void)
{
  enum { INIT = 0, WAIT_READY, READY, RUNNING, PROGRAM } runState = INIT;
  enum runMode_t { NORMAL = 0, INACTIVITY } runMode = NORMAL;

  /**************************
   ****     MAIN LOOP     ****
   **************************/
  while (1)
  {

    switch (runState) 
    {
      case INIT: 
      {
        cli();
        wdt_reset();

        wdt_enable(WDTO_4S);  // 4 second watchdog timer. (Brown Out Detector fuses are set for 2.7V)

        LED1_DDR |= (1<<LED1_BIT); LED1_PORT &= ~(1<<LED1_BIT);     // output
        LED2_DDR |= (1<<LED2_BIT); LED2_PORT &= ~(1<<LED2_BIT);     // output
        LED3_DDR |= (1<<LED3_BIT); LED3_PORT &= ~(1<<LED3_BIT);     // output
        PIEZO_DDR |= (1<<PIEZO_BIT); PIEZO_PORT &= ~(1<<PIEZO_BIT); // output (PWM on timer 0)
        SIG_DDR &= ~(1<<SIG_BIT); SIG_PORT |= (1<<SIG_BIT);         // input with pullup (ICR1)

        // set up PWM output on OC0B pin (timer 0)
        TCCR0A = (0b01<<WGM00); // 8-bit Phase Correct PWM mode, output on OC0B (PD5) ...
        TCCR0B = (1<<WGM02) | (0b010<<CS00); // ... clk/8 with TOP(OCR0A)=180 / 2 gives ~2.8KHz

        // start piezo tone out at a lower, quieter frequency and duty cycle
        OCR0A = 255; 
        OCR0B = PWM_DUTY_CYCLE_QUIET;

        // set up timer 1 for input capture compare on ICP1 pin
        TCCR1A = (0b00<<WGM10);
        TCCR1B = (0b010<<CS10); // clk/8 for 1MHz counting
        TCCR1B |= (1<<ICES1);   // look for rising edge first
        TCNT1 = 0;
        sigPulseWidth = PULSE_INVALID; 
        TIMSK1 = (1<<ICIE1);    // input capture interrupt enabled

        // set up timer 2 for the Morse code sate machine
        TCCR2A = (0x10<<WGM20); // CTC mode
        TCCR2B = (0b010<<CS20); // CLK / 8 (1Mhz counter rate)
        TCNT2 = 0;
        OCR2A = 249;            // counter TOP (250uS, 8-bit timer)
        TIMSK2 = (1<<OCIE2A);   // enable timer 2 OC interrupt (every 1ms) 

        
        sei(); // gloabl interrupt enable

        // retrieve stored Run Mode from EEPROM
        runMode = (enum runMode_t)eeprom_read_byte(0x00);
        if (runMode > INACTIVITY) 
        {
          runMode = NORMAL; // sanitize
          storeRunMode(runMode); // store
        }

        // check for programming mode request by input pulse varying widely over
        // over 10 samples. Skipped if a BOD or WDT reset occurred
        /*
        if (MCUSR & WDRF)
        {
          MCUSR &= ~(1<<WDRF); // reset the WDRF flag
          wdt_reset();
          wdt_enable(WDTO_4S); // just in case
        }
        else if (MCUSR & BORF)
          MCUSR &= ~(1<<BORF);         // reset the BORF flag
        else
          */
        {
          _delay_ms(1000);             // allow one second for the receiver to boot up
          unsigned int pulseSum = 0;
          unsigned int lastPW = 0;
          int count;
          for (count = 0; count < 10; count++)
          {
            int thisPulse = sigPulseWidth;
            if (lastPW == 0) lastPW = thisPulse;
            if (thisPulse > 0)
            {
              pulseSum += abs(thisPulse - lastPW);
              lastPW = thisPulse;
            }
            _delay_ms(50);
          }
          if ((pulseSum / count) > 200) // enter programming mode if average pulse delta large enough 
          {
            runState = PROGRAM;
            break;
          }
        }
        runState = WAIT_READY;
        break;
      }

      case WAIT_READY: 
      {
        LED_ON(1);

        if ((sigPulseWidth < 0) && (morseState == STOP))
        {
          morseString = (char *)PSTR("E   ");
          morseStart();
        }
        else if (morseState == STOP)
        {
          if (runMode == NORMAL)
            morseString = (char *)PSTR("R N ");
          else
            morseString = (char *)PSTR("R I ");
          morseStart();
          LED_OFF(1);
          runState = READY;
        }
        break;
      }

      case READY: {
        if (morseState == STOP)
        {
          // Set beeper frqeuency to ~2.8KHz at 15% duty cycle = LOUD!
          OCR0A = 180;
          OCR0B = PWM_DUTY_CYCLE;

          // emit a single, loud "BIP!"
          if (sigPulseWidth < MIDPOINT)
          {
            beeperOn();
            _delay_ms(100);
            beeperOff();
          }

          runState = RUNNING;
        }
        break;
      }

      case RUNNING: 
      {
        switch (runMode)
        {
          case /* runMode is */NORMAL:
          {
            if (sigPulseWidth > 0) // if pulse did not time out
            {
              if (sigPulseWidth > MIDPOINT)
              {
                morseString = (char *)PSTR("LOST "); 
                morseStart();    // start bleeting loud morse tones
              }
              else
                morseStop();
            } 
            else
            {
              morseStop();
              runState = WAIT_READY; // we lost the pulse signal TODO: should optionally alarm instead?
            }
          
            break;
          }

          case /* runMode is */INACTIVITY:
          {
            static int last_sigPulseWidth = 0;
            static int last_sigPulseWidthAverage = 0;
            static int sigPulseWidthAverage;
            static unsigned int inactiveTimer = 0;
            
            if (sigPulseWidth > 0) // if pulse did not time out
            {
              int thisPulse = sigPulseWidth;
              if (last_sigPulseWidth == 0) last_sigPulseWidth = thisPulse;
              sigPulseWidthAverage = ((last_sigPulseWidth * 7) + thisPulse) / 8; // some noise filtering
              last_sigPulseWidth = thisPulse;

              if (last_sigPulseWidthAverage == 0)
                last_sigPulseWidthAverage = sigPulseWidthAverage;

              if ( (sigPulseWidthAverage > (last_sigPulseWidthAverage-20)) 
                  && (sigPulseWidthAverage < (last_sigPulseWidthAverage+20/*TODO sensitivity*/)) )
              {
                if (inactiveTimer++ > 600/*TODO -- how long to wait for inactivity before alarming*/)
                {
                  inactiveTimer--; // don't let it keep climbing
                  if (morseState == STOP)
                  {
                    morseString = (char *)PSTR("LOST "); 
                    morseStart();   // start bleeting loud morse tones
                  }
                }
              }
              else
              {
                inactiveTimer = 0;
                morseStop();
              }
              last_sigPulseWidthAverage = sigPulseWidthAverage;

            }
            else
            {
              morseStop();
              runState = WAIT_READY; // we lost the pulse signal TODO: should optionally alarm instead?
            }

          }
        }

        break; // case RUNNING
      }

      case PROGRAM: 
      {
        static enum { 
          WAIT, // wait for pulse to go above MIDPOINT before entering program mode
          ENTER, 
          ASK_NORMAL, 
          ANS_NORMAL, 
          ASK_INACTIVE, 
          ANS_INACTIVE, 
          OK, 
          EXIT 
        } pgmState = WAIT;
        static int pgmTimer = 0;


        switch (pgmState)
        {
          case WAIT:
            if (sigPulseWidth < (MIDPOINT * 1.17)) // at least 3/4 stick needed (allowing for first 1ms being included)
            { 
              if (morseState == STOP)
              {
                LED_TOGGLE(2);
                morseString = (char *)PSTR("W  ");
                morseStart();
              }
            }
            else if (pgmTimer++ > 100)
            {
              pgmTimer = 0;
              morseStop();
              pgmState = ENTER;
            }
            break;

          case ENTER:
            LED_ON(1);
            morseString = (char *)PSTR("P  ");
            morseStart();
            pgmState = ASK_NORMAL;
            break;

          case ASK_NORMAL:
            if (morseState == STOP) // wait for morse code to finish sounding
            {
              pgmTimer = 0;
              morseString = (char *)PSTR("N? ");
              morseStart();
              pgmState = ANS_NORMAL;
            }
            break;

          case ANS_NORMAL:
            if (morseState == STOP) // wait for morse code to finish sounding
            {
              if (pgmTimer++ < 200) // did they say yes?
              {
                if (sigPulseWidth < MIDPOINT)
                {
                  storeRunMode(NORMAL);
                  pgmState = OK;
                }
              } else
                pgmState = ASK_INACTIVE;
            }
            break;

          case ASK_INACTIVE:
            if (morseState == STOP) // wait for morse code to finish sounding
            {
              pgmTimer = 0;
              morseString = (char *)PSTR("I? ");
              morseStart();
              pgmState = ANS_INACTIVE;
            }
            break;

          case ANS_INACTIVE:
            if (morseState == STOP) // wait for morse code to finish sounding
            {
              if (pgmTimer++ < 200) // did they say yes?
              {
                if (sigPulseWidth < MIDPOINT)
                {
                  storeRunMode(INACTIVITY);
                  pgmState = OK;
                }
              } else
                pgmState = ASK_NORMAL;
            }
            break;

          case OK:
            morseString = (char *)PSTR("OK ");
            morseStart();
            pgmState = EXIT;
            break;

          case EXIT:
            if (morseState == STOP) // wait for morse code to finish sounding
              runState = INIT;
            break;
        }

        break;
      }

      default:
        runState = INIT;

    } // main loop switch

    _delay_ms(10); // XXX TODO: fudge to slow asynch counters for PROGRAM and INACTIVITY modes down

    wdt_reset();
  } // MAIN LOOP
}

