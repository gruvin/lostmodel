/* Name: main.c
 * Project: Lost Model Alarm for AVR ATmega88P
 * Author: Bryan Rentoul (aka Gruvin)
 * Creation Date: 2014-11-22
 * Tabsize: 2
 * Copyright: (c) 2014 Gruvin9X Project and Bryan J. Rentoul aka Gruvin
 * License: GNU GPL v2 (see License.txt)
 * This Revision: $Id$
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h> // abs()
#include "lostmodel.h"

int pulseInWidth()
{
  unsigned int width;         // pulse width in while loop iterations, which should be exactly 10 clock cycles
  unsigned long numloops;
  
  numloops = 0;
  // wait for any previous pulse to end
  while (SIG_INP & (1<<SIG_BIT))
    if (numloops++ > (MIDPOINT * 2)) return -1;

  // wait for pulse to start
  while (!(SIG_INP & (1<<SIG_BIT)))
    if (numloops++ > ((unsigned long)MIDPOINT * 14)) return -1; // MIDPOINT ~= 1500us. Full PPM sweep over 8 channels is 23ms.

  // wait for pulse to end
  width = 0;
  while (SIG_INP & (1<<SIG_BIT))
    if (width++ > (MIDPOINT * 3)) return -1;

  return width; 
}

void beeperOn()
{
  LED_ON(2);

  // PWM ON
  TCCR0A |= (0b10<<COM0B0);
}

void beeperOff()
{
  LED_OFF(2);

  TCCR0A &= ~(0b11<<COM0B0);
}

const char morse[47][7] PROGMEM  = {
  { 2, 2, 1, 1, 2, 2, 0 },// , ASCII = 44
  { 0 },                  // -
  { 1, 2, 1, 2, 1, 2, 0}, // .
  { 2, 1, 1, 2, 1, 0 },   // /
  { 2, 2, 2, 2, 2, 0 },   // 0
  { 1, 2, 2, 2, 2, 0 },   // 1
  { 1, 1, 2, 2, 2, 0 },   // 2
  { 1, 1, 1, 2, 2, 0 },   // 3
  { 1, 1, 1, 1, 2, 0 },   // 4
  { 1, 1, 1, 1, 1, 0 },   // 5
  { 2, 1, 1, 1, 1, 0 },   // 6
  { 2, 2, 1, 1, 1, 0 },   // 7
  { 2, 2, 2, 1, 1, 0 },   // 8
  { 2, 2, 2, 2, 1, 0 },   // 9
  { 0 },                  // :
  { 0 },                  // ;
  { 0 },                  // <
  { 2, 1, 1, 1, 2, 0 },   // =
  { 0 },                  // >
  { 1, 1, 2, 2, 1, 1, 0 },// ?
  { 0 },                  // @
  { 1, 2, 0 },            // A
  { 2, 1, 1, 1, 0 },      // B
  { 2, 1, 2, 1, 0 },      // C
  { 2, 1, 1, 0 },         // D
  { 1, 0 },               // E
  { 1, 1, 2, 1, 0 },      // F
  { 2, 2, 1, 0 },         // G
  { 1, 1, 1, 1, 0 },      // H
  { 1, 1, 0 },            // I
  { 1, 2, 2, 2, 0 },      // J
  { 2, 1, 2, 0 },         // K
  { 1, 2, 1, 1, 0 },      // L
  { 2, 2, 0 },            // M
  { 2, 1, 0 },            // N
  { 2, 2, 2, 0 },         // O
  { 1, 2, 2, 1, 0 },      // P
  { 2, 2, 1, 2, 0 },      // Q
  { 1, 2, 1, 0 },         // R
  { 1, 1, 1, 0 },         // S
  { 2, 0 },               // T
  { 1, 1, 2, 0 },         // U
  { 1, 1, 1, 2, 0 },      // V
  { 1, 2, 2, 0 },         // W
  { 2, 1, 1, 2, 0 },      // X
  { 2, 1, 2, 2, 0 },      // Y
  { 2, 2, 1, 1, 0 }       // Z
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
void morseStateMachine()
{
  static unsigned int timer;
  static unsigned char bitIndex;
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

      morseState = (morseState_t)pgm_read_byte(&morse[thisChar-44][bitIndex++]);

      if ((morseState == END) || (bitIndex > sizeof(morse[1]))) {
        morseState = LETTERGAP;
        bitIndex = 0;
      };

      break;

    case NEXTCHAR:

      thisChar = pgm_read_byte(&morseString[charIndex++]);

      if (thisChar == ' ') 
      {
        timer = 0;
        morseState = SPACE;
      }
      else if ((thisChar == '\0') || (thisChar < ',') || (thisChar > 'Z'))
        morseState = STOP;
      else
        morseState = NEXTBIT;

      break;


    case START:
      beeperOff();
      timer = 0;
      bitIndex = 0;
      charIndex = 0;
      morseState = NEXTCHAR;
      break;

    case STOP:
      beeperOff();
      break; // do nothing

    default:
      morseState = STOP;
      break;

  } // switch
      
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

void setRunMode(unsigned char newmode)
{
  eeprom_write_byte(0x00, newmode);
}

int main(void)
{
  enum { INIT = 0, WAIT_READY, READY, RUNNING, PROGRAM } runState = INIT;
  int pw = 0; // store last sampled pulse width

  /**************************
   ****     MAIN LOOP     ****
   **************************/
  while (1)
  {

    unsigned char runMode = NORMAL;

    switch (runState) 
    {
      case INIT: 
      {
        wdt_enable(WDTO_4S);  // 4 second watchdog timer. (Brown Out Detector fuses are set for 2.7V)

        LED1_DDR |= (1<<LED1_BIT); LED1_PORT &= ~(1<<LED1_BIT); // output
        LED2_DDR |= (1<<LED2_BIT); LED2_PORT &= ~(1<<LED2_BIT); // output
        LED3_DDR |= (1<<LED3_BIT); LED3_PORT &= ~(1<<LED3_BIT); // output
        PIEZO_DDR |= (1<<PIEZO_BIT); PIEZO_PORT &= ~(1<<PIEZO_BIT); // output (PWM on timer 0)
        SIG_DDR &= ~(1<<SIG_BIT); SIG_PORT |= (1<<SIG_BIT); // input with pullup

        // set up PWM output on OC0B pin (timer 0)
        TCCR0A = (0b01<<WGM00); // 8-bit Phase Correct PWM mode, output on OC0B (PD5) ...
        TCCR0B = (1<<WGM02) | (0b010<<CS00); // ... clk/8 with TOP(OCR0A)=180 / 2 gives ~2.8KHz

        // start beep out at lower, quieter frequency and duty cycle
        OCR0A = 255; 
        OCR0B = PWM_DUTY_CYCLE_QUIET;

        runMode = eeprom_read_byte(0x00);
        if (runMode > INACTIVITY) 
        {
          runMode = NORMAL; // sanitize
          setRunMode(runMode); // store
        }

        // check for programming mode request by input pulse varying widely at least four times
        // over period of 400ms, starting one second after start-up.
        // TODO XXX: skip this if BOD or WDT reset occurred
        _delay_ms(1000);
        int pw1 = pulseInWidth();
        _delay_ms(100);
        int pw2 = pulseInWidth();
        _delay_ms(100);
        int pw3 = pulseInWidth();
        _delay_ms(100);
        int pw4 = pulseInWidth();
        _delay_ms(100);
        int pw5 = pulseInWidth();
        if ( (abs(pw2 - pw1) > 130) && (abs(pw3 - pw2) > 130) && (abs(pw4 - pw3) > 130) && (abs(pw5 - pw4) > 130) ) 
          runState = PROGRAM;
        else
          runState = WAIT_READY;
        break;
      }

      case WAIT_READY: 
      {
        LED_ON(3);

        if ((pw < 0) && (morseState == STOP))
        {
          beeperOn();
          _delay_ms(100);
          beeperOff();
          _delay_ms(900);
        }
        else if (morseState == STOP)
        {
          if (runMode == NORMAL)
            morseString = (char *)PSTR("R N ");
          else
            morseString = (char *)PSTR("R I ");
          morseStart();
          LED_OFF(3);
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
          if (pw < MIDPOINT)
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
            if (pw > 0) // if pulse did not time out
            {
              if (pw > MIDPOINT) // approximately 1500us (mid-servo) using internal 8MHZ MCU clock
              {
                morseString = (char *)PSTR("LOST "); 
                morseStart();    // start bleeting loud morse tones
              }
              else
                morseStop();
            } else
              morseStop();
          
            break;
          }

          case /* runMode is */INACTIVITY:
          {
            static int last_pw = 0;
            static int last_pwAverage = 0;
            static int pwAverage;
            static unsigned int inactiveTimer = 0;
            
            if (pw > 0) // if pulse did not time out
            {
              if (last_pw == 0) last_pw = pw;
              pwAverage = ((last_pw * 7) + pw) / 8; // some noise filtering
              last_pw = pw;

              if (last_pwAverage == 0) last_pwAverage = pwAverage;
              if ( (pwAverage > (last_pwAverage-10)) && (pwAverage < (last_pwAverage+10/*TODO sensitivity*/)) )
              {
                if (inactiveTimer++ > 10000/*TODO -- how long to wait for inactivity before alarming*/)
                {
                  inactiveTimer--; // don't let it keep climbing
                  morseString = (char *)PSTR("LOST "); 
                  morseStart();   // start bleeting loud morse tones
                }
              }
              else
              {
                inactiveTimer = 0;
                morseStop();
              }
              last_pwAverage = pwAverage;

            } else // pulse timed out
              morseStop();

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
            if (pw < MIDPOINT) 
            { 
              if (morseState == STOP)
              {
                  beeperOn();
                  _delay_ms(300);
                  beeperOff();
                  _delay_ms(700);
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
            morseString = (char *)PSTR("PGM   ");
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
                if (pw < MIDPOINT)
                {
                  setRunMode(NORMAL);
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
                if (pw < MIDPOINT)
                {
                  setRunMode(INACTIVITY);
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

    morseStateMachine();

    pw = pulseInWidth();

    wdt_reset();
  } // MAIN LOOP
}

