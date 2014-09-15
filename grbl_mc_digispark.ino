/*
  esc spindle controller for grbl
  by ms@ms-ite.de
  
  - POC code!!
  - controlled via i2c
  - connects to a rotary encoder + switch for speed/manual stop
  - outputs a servo signal (1-2ms, 50Hz) for esc control
  - no led feedback (cut trace) as i2c uses digisparks led pin
*/

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/twi.h>
#include <util/delay.h>

#include <TinyWireS.h>
 
#define F_CPU 16500000

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

#define PIN_SDA 0
#define PIN_SCK 2

#define PIN_SERVO 1

#define PIN_UP 3
#define PIN_DOWN 4
#define PIN_MUTE 5

#define PIN_INPUT_MASK ((1<<PIN_UP) | (1<<PIN_MUTE) | (1<<PIN_DOWN))

#define DEBOUNCE_TICKS 2

#define SERVO_PULSE_1MS 125      // 1ms
#define SERVO_PULSE_2MS 250   // 1ms
#define SERVO_PAUSE_CYCLES 17             // cycles of 125/1ms ints

#define RPM_OFF 0
#define RPM_SLOW 15
#define RPM_MAX 125

#define BLINK_TICKS_MANUAL 5
#define BLINK_TICKS_MASTER 1

#define NOP 0
#define UP 1
#define DOWN 2

//-----------------------------------------------------------

enum SequencerState_t {
  PULSE,
  PAUSE,
  POST
};

volatile byte state_machine = POST;
volatile byte servo_pause = 0;

volatile byte pin_status = 0xff;

volatile int _rpm_master = RPM_OFF;
volatile int _rpm_slave = RPM_OFF;

volatile boolean _forceOff = false;

//int _blinkCount = 0;
//int _blinkPhase = false;

void setup( void) {

  sei();
    
  // timer mode, 128 prescale: 16M/128
  TCCR1 = (0<< CTC1) | (0<< PWM1A) | (0<< COM1A1) | (0<< COM1A0) | (1<< CS13) | (0<< CS12) | (0<< CS11) | (0<< CS10);
  GTCCR = (0<< PWM1B) | (0<< COM1B1) | (0<< COM1B0) | (0<< FOC1B) | (0<< FOC1A);
  
  TIMSK |= ( 1 << OCIE1A);
  
  OCR1A = SERVO_PULSE_1MS;
//  OCR1B = SERVO_PULSE_FIXED;

  TCNT1 = 0;
  
//  TCCR0A = (1<< COM0A1) | (0<< COM0A0) | (1<< WGM01) | (1<< WGM00);
//  TCCR0B = (0<< WGM02) | (0<< CS02) | (0<< CS01) | (1<< CS00);

  DDRB  = (0<< DDB5) | (0<< DDB4) | (0<< DDB3) | (1<< DDB2) | (1<< DDB1) | (1<< DDB0);  // pins 0 ans 1 as outputs, others as input
  PORTB = (1<< PIN_DOWN) | (1<< PIN_MUTE) | (1<< PIN_UP) | (1<< PIN_SCK) | (0<< PIN_SERVO) | (0<< PIN_SDA);  // enable pullup for inputs

  // if it's unconnected (pulled high) or enabled by default, we stop it
//  _forceOff = ((PINB & (1<< PIN_MASTER)) == HIGH);
  
  PCMSK = PIN_INPUT_MASK;
  GIMSK |= ( 1 << PCIE);

  TinyWireS.begin( 0x5c);                // join i2c bus with address $5c for SpeedControl
  TinyWireS.onReceive(receiveEvent); // register event
  TinyWireS.onRequest(requestEvent); // register event
}

void loop( void) { 
  byte cnt = 40;
  
  // do not go into ESC set up mode
  // output off for 4s
  while( cnt > 0) {
    setLED( _rpm_slave, true, true);
    _delay_ms( 100);
    cnt--;
  }

  // make ESC detect digispark, do not enter ESC setup, allow ESC to scale to pulse/pause ratio
  // output on for 1s
  _rpm_slave = RPM_MAX;
  cnt = 10;
  while( cnt > 0) {
    setLED( _rpm_slave, true, false);
    _delay_ms( 100);
    cnt--;
  }
  
  // now loop
  _rpm_slave = RPM_OFF; 
  while( true) {
    setLED( _rpm_master * 2, _forceOff /* || ((PINB & (1<< PIN_MASTER)) == HIGH) */, _forceOff);
    tws_delay( 100);
  }
}

void receiveEvent( uint8_t howMany) {
  int rpm = 0; // receive byte as a character

  while( TinyWireS.available() > 0) {
    rpm = TinyWireS.receive();
  }
  
  if ( rpm > RPM_MAX) _rpm_master = RPM_MAX;
  else if ( rpm < RPM_OFF) _rpm_master = RPM_OFF;
  else _rpm_master = rpm;
}

void requestEvent() {
  TinyWireS.send( _rpm_master);
}

byte gamma_correction(byte input) {
  unsigned int multiplied = input * input;
  return multiplied / 256;
}

byte encoder( byte input) {
  byte res = 0;
  
  if ( input & ( 1<< PIN_UP)) res |= 0x02;
  if ( input & ( 1<< PIN_DOWN)) res |= 0x01;
  
  return res;
}

// level change interrupt on 
ISR( PCINT0_vect) {
  
  //      00 01 10 11
  //  00  .  -  +  .
  //  01  +  .  .  -
  //  11  .  +  -  .
  //  10  -  .  .  +
  
  const byte enc_states[] = { NOP, DOWN, UP, NOP, 
                              UP, NOP, NOP, DOWN, 
                              DOWN, NOP, NOP, UP, 
                              NOP, UP, DOWN, NOP};

  static byte encoder_pos = 0;
  
  byte pin = PINB & PIN_INPUT_MASK;
  
  // should always be true as we're triggered only on edges
  if ( pin ^ pin_status) {
    pin_status = pin;

    if (( pin & (1<< PIN_MUTE)) == LOW) _forceOff = !_forceOff;
   
    byte enc = encoder( pin);
    
    if ( enc != encoder_pos) {
      enc |= encoder_pos << 2;
      
      switch ( enc_states[ enc]) {
        case UP:
          if ( _rpm_master < RPM_MAX) _rpm_master++;
          else _rpm_master = RPM_MAX;
        break;
        
        case DOWN:
          if ( _rpm_master > RPM_OFF) _rpm_master--;
          else _rpm_master = RPM_OFF;
        break;
        
        default:
          ;
      }
    
      encoder_pos = enc & 0x03;
    }
  }
}

// timer interrupt for pwm signal generation
ISR(TIM1_COMPA_vect) {
    switch( state_machine) {
      case PULSE:
        OCR1A = SERVO_PULSE_1MS + _rpm_slave;
        PORTB |= (1 << PIN_SERVO);
        state_machine = PAUSE;
      break;

      case PAUSE:
        OCR1A = SERVO_PULSE_2MS - _rpm_slave;
        PORTB &= ~( 1 << PIN_SERVO);

        servo_pause = 0;
        state_machine = POST;
      break;

      case POST:
        OCR1A = SERVO_PULSE_1MS;          
        if ( servo_pause >= SERVO_PAUSE_CYCLES) {
          
          _rpm_slave = ( _forceOff /*|| ((PINB & (1<< PIN_MASTER)) == LOW)*/) ? 0 : _rpm_master;
          state_machine = PULSE;
        } else servo_pause++;
      break;
      
      default:
        state_machine = POST;
  
        OCR1A = SERVO_PULSE_1MS;
        servo_pause = 0;
    }
    
    TCNT1 = 0;
}//end ISR TIM0_COMPA_vect

void setLED( byte val, boolean doBlink, boolean freq) {
  /*
  if ( doBlink) {
    if ( _blinkCount <= 0) {
      _blinkPhase = ! _blinkPhase;
      _blinkCount = freq ? BLINK_TICKS_MANUAL : BLINK_TICKS_MASTER;
    } else _blinkCount--;
    
    OCR0A = _blinkPhase ? gamma_correction( val) : 0;
  } else {
    OCR0A = gamma_correction( val);
  }
  */
}

