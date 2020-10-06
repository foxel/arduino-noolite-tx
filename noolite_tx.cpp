/*
  NooliteTX - Library for transmitting NooLite RF codes
  Created by Andrey F. Kupreychik (c) 2017
*/

#include "Arduino.h"
#include "noolite_tx.h"
#include <avr/power.h>

//usec
#define PERIOD 500

void __setup_timer(void);

static volatile uint8_t _tx_signal = 0;
static volatile uint8_t _tx_phase = 0;
static volatile uint8_t _tx_ready = 0;
static volatile uint8_t _tx_pin = 0;

// this is a timer compare handler to change output pin state
#if defined(TIMER1_OVF_vect)
SIGNAL(TIMER1_OVF_vect)
#elif defined(TIM1_OVF_vect)
SIGNAL(TIM1_OVF_vect)
#else
  #error cannot find timer overflow vector
#endif
{
  if (_tx_phase) {
    digitalWrite(_tx_pin, _tx_signal);
  } else {
    digitalWrite(_tx_pin, LOW);
  }
  
  switch (_tx_phase) {
    case 2: 
      _tx_signal = !_tx_signal;
      break;
    case 1:
      _tx_ready = 1;
      break;
    case 0:
      // disable timer interrupt
      #if defined(TIMSK)
        TIMSK &= ~(1<<TOIE1);
      #else
        TIMSK1 &= ~(1<<TOIE1);
      #endif
  }
  _tx_phase--;
}

void send_bit(uint8_t pin, uint8_t value) {
  _tx_pin = pin;
  _tx_signal = !value;
  _tx_phase = 2;
  _tx_ready = 0;
  
  #if defined(TIMSK)
    TIMSK |= (1<<TOIE1);
  #else
    TIMSK1 |= (1<<TOIE1);
  #endif

  while (!_tx_ready);
}

void send_sequence(uint8_t pin, uint8_t count, uint8_t * sequence, uint8_t shift) {
  send_bit(pin, 1);
  
  for (uint8_t j = shift; j < 8; j++) {
    send_bit(pin, bitRead(sequence[0], j));
  }

  for (uint8_t i = 1; i < count; ++i) {
    for (uint8_t j = 0; j < 8; j++) {
      send_bit(pin, bitRead(sequence[i], j));
    }
  }

  delayMicroseconds(PERIOD * 3);
}

void send_preable(uint8_t pin, uint8_t len) {
  for (uint8_t i = 0; i < len; ++i) {
    send_bit(pin, 1);
  }

  delayMicroseconds(PERIOD * 3);
}


// Automatically generated CRC function
// polynomial: 0x131, bit reverse algorithm
uint8_t crc8_maxim(uint8_t *data, uint8_t len, uint8_t crc)
{
  static const uint8_t table[256] PROGMEM = {
    0x00U,0x5EU,0xBCU,0xE2U,0x61U,0x3FU,0xDDU,0x83U,
    0xC2U,0x9CU,0x7EU,0x20U,0xA3U,0xFDU,0x1FU,0x41U,
    0x9DU,0xC3U,0x21U,0x7FU,0xFCU,0xA2U,0x40U,0x1EU,
    0x5FU,0x01U,0xE3U,0xBDU,0x3EU,0x60U,0x82U,0xDCU,
    0x23U,0x7DU,0x9FU,0xC1U,0x42U,0x1CU,0xFEU,0xA0U,
    0xE1U,0xBFU,0x5DU,0x03U,0x80U,0xDEU,0x3CU,0x62U,
    0xBEU,0xE0U,0x02U,0x5CU,0xDFU,0x81U,0x63U,0x3DU,
    0x7CU,0x22U,0xC0U,0x9EU,0x1DU,0x43U,0xA1U,0xFFU,
    0x46U,0x18U,0xFAU,0xA4U,0x27U,0x79U,0x9BU,0xC5U,
    0x84U,0xDAU,0x38U,0x66U,0xE5U,0xBBU,0x59U,0x07U,
    0xDBU,0x85U,0x67U,0x39U,0xBAU,0xE4U,0x06U,0x58U,
    0x19U,0x47U,0xA5U,0xFBU,0x78U,0x26U,0xC4U,0x9AU,
    0x65U,0x3BU,0xD9U,0x87U,0x04U,0x5AU,0xB8U,0xE6U,
    0xA7U,0xF9U,0x1BU,0x45U,0xC6U,0x98U,0x7AU,0x24U,
    0xF8U,0xA6U,0x44U,0x1AU,0x99U,0xC7U,0x25U,0x7BU,
    0x3AU,0x64U,0x86U,0xD8U,0x5BU,0x05U,0xE7U,0xB9U,
    0x8CU,0xD2U,0x30U,0x6EU,0xEDU,0xB3U,0x51U,0x0FU,
    0x4EU,0x10U,0xF2U,0xACU,0x2FU,0x71U,0x93U,0xCDU,
    0x11U,0x4FU,0xADU,0xF3U,0x70U,0x2EU,0xCCU,0x92U,
    0xD3U,0x8DU,0x6FU,0x31U,0xB2U,0xECU,0x0EU,0x50U,
    0xAFU,0xF1U,0x13U,0x4DU,0xCEU,0x90U,0x72U,0x2CU,
    0x6DU,0x33U,0xD1U,0x8FU,0x0CU,0x52U,0xB0U,0xEEU,
    0x32U,0x6CU,0x8EU,0xD0U,0x53U,0x0DU,0xEFU,0xB1U,
    0xF0U,0xAEU,0x4CU,0x12U,0x91U,0xCFU,0x2DU,0x73U,
    0xCAU,0x94U,0x76U,0x28U,0xABU,0xF5U,0x17U,0x49U,
    0x08U,0x56U,0xB4U,0xEAU,0x69U,0x37U,0xD5U,0x8BU,
    0x57U,0x09U,0xEBU,0xB5U,0x36U,0x68U,0x8AU,0xD4U,
    0x95U,0xCBU,0x29U,0x77U,0xF4U,0xAAU,0x48U,0x16U,
    0xE9U,0xB7U,0x55U,0x0BU,0x88U,0xD6U,0x34U,0x6AU,
    0x2BU,0x75U,0x97U,0xC9U,0x4AU,0x14U,0xF6U,0xA8U,
    0x74U,0x2AU,0xC8U,0x96U,0x15U,0x4BU,0xA9U,0xF7U,
    0xB6U,0xE8U,0x0AU,0x54U,0xD7U,0x89U,0x6BU,0x35U,
  };
    
  while (len > 0)
  {
    crc = pgm_read_byte(&table[*data ^ (uint8_t)crc]);
    data++;
    len--;
  }
  return crc;
}


uint8_t calc_checksum(uint8_t count, uint8_t * sequence) {
  uint8_t data[] = {0,0,0,0,0,0,0,0,0,0};
  uint8_t mask  ;
  // first byte from 1 to 5 bit (0-based)
  
  for (uint8_t byte_n=0; byte_n < count; ++byte_n) {
    for (uint8_t i=0; i < 8; ++i) {
      if (bitRead(sequence[byte_n], i)) {
        mask = 1 << i;  
        data[byte_n] |= mask;
      }
    }
  }
  
  return crc8_maxim(data, count, 0);
}


NooliteTX::NooliteTX(uint8_t pin, uint16_t addr, uint8_t repeats) {
  _pin = pin;
  _addr_lo = lowByte(addr);
  _addr_hi = highByte(addr);
  _flip = 0;
  _repeats = repeats;
}

NooliteTX::NooliteTX(uint8_t pin, uint16_t addr) {
  NooliteTX(pin, addr, 3);
}



void NooliteTX::send_command(uint8_t cmd, uint8_t * argv, uint8_t argc) {
  pinMode(_pin, OUTPUT);
  __setup_timer();

  uint8_t payload[] = {0,0,0,0,0,0,0,0,0,0};
  uint8_t len = 0;
  uint8_t shift = 0;
  uint8_t fmt = 0;

  if (argc == 0) {
    fmt = 0;
  } else if (argc == 1) {
    fmt = 1;
  } else if (argc == 2) {
    fmt = 2;
  } else if (argc == 4) {
    fmt = 3;
  }

  if (cmd > 15) {
    shift = 7;
    payload[len++] = (_flip & 1) << 7;
    payload[len++] = cmd;
    fmt = fmt | 4;
  } else {
    shift = 3;
    payload[len++] = ((cmd & 0x0f) << 4) | (_flip & 1) << 3;
  }

  for (uint8_t i = 0; i < argc; i++) {
    payload[len++] = argv[i];
  }

  payload[len++] = _addr_lo;
  payload[len++] = _addr_hi;
  payload[len++] = fmt;
  
  byte checksum_val = calc_checksum(len, payload);
  payload[len++] = checksum_val;

  send_preable(_pin, len*8);
  for (uint8_t i = 0; i < _repeats; ++i) {
    send_sequence(_pin, len, payload, shift);
  }

  _flip = !_flip;
}

void NooliteTX::send_command(uint8_t cmd) {
  this->send_command(cmd, {}, 0);
}


// preparing timer
void __setup_timer(void) {
  #if defined(power_timer1_enable)
    power_timer1_enable();
  #endif

  #if defined(TCCR1) // 8-bit TIMER1 with 4 bit RF_TIMER_PRESCALER (e.g. TINYx5)
    // Turn off Clear on Compare Match, turn off PWM A, disconnect the timer from the output pin, stop the clock
    TCCR1 = 0;
    // Turn off PWM A, disconnect the timer from the output pin, no Force Output Compare Match, no RF_TIMER_PRESCALER Reset
    GTCCR &= ~((1<<PWM1B) | (1<<COM1B1) | (1<<COM1B0) | (1<<FOC1B) | (1<<FOC1A) | (1<<PSR1));
    // Disable all Timer1 interrupts
    TIMSK &= ~((1<<OCIE1A) | (1<<OCIE1B) | (1<<TOIE1));
    // Clear the Timer1 interrupt flags
    TIFR |= ((1<<OCF1A) | (1<<OCF1B) | (1<<TOV1));
    // disadle PLL source for timer1 if available
    #if defined(PLLCSR)
      PLLCSR &= ~(1<<PCKE);
    #endif

    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    OCR1C = 0;
    
    // set CTC and PWM
    TCCR1 |= (1<<CTC1) | (1<<PWM1A);
    // set RF_TIMER_PRESCALER and counter
    #if F_CPU <= 1000000L
      #define RF_TIMER_PRESCALER 2
      TCCR1 |= (1<<CS11);
    #elif F_CPU <= 2000000L
      #define RF_TIMER_PRESCALER 4
      TCCR1 |= (1<<CS11) | (1<<CS10);
    #elif F_CPU <= 4000000L
      #define RF_TIMER_PRESCALER 8
      TCCR1 |= (1<<CS12);
    #elif F_CPU <= 8000000L
      #define RF_TIMER_PRESCALER 16
      TCCR1 |= (1<<CS12) | (1<<CS10);
    #elif F_CPU <= 16000000L
      #define RF_TIMER_PRESCALER 32
      TCCR1 |= (1<<CS12) | (1<<CS11);
    #elif F_CPU <= 32000000L
      #define RF_TIMER_PRESCALER 64
      TCCR1 |= (1<<CS12) | (1<<CS11) | (1<<CS10);
    #else
      #error "CPU frequesncy higher than 32 MHz is not expected"
    #endif
    
    OCR1C = (F_CPU / RF_TIMER_PRESCALER) / (1000000L / PERIOD);
  #elif defined(TCCR1E) // 8-10-bit timer (e.g. TINYx61
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1D = 0;
    TCCR1E = 0;
    // Disable all Timer1 interrupts
    TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) | (1<<OCIE1D));
    // Clear the Timer1 interrupt flags
    TIFR |= ((1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<OCF1D));
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    // disadle PLL source for timer1 if available
    #if defined(PLLCSR)
      PLLCSR &= ~(1<<PCKE);
    #endif

    // set RF_TIMER_PRESCALER and counter
    #if F_CPU <= 1000000L
      #define RF_TIMER_PRESCALER 2
      TCCR1B |= (1<<CS11);
    #elif F_CPU <= 2000000L
      #define RF_TIMER_PRESCALER 4
      TCCR1B |= (1<<CS11) | (1<<CS10);
    #elif F_CPU <= 4000000L
      #define RF_TIMER_PRESCALER 8
      TCCR1B |= (1<<CS12);
    #elif F_CPU <= 8000000L
      #define RF_TIMER_PRESCALER 16
      TCCR1B |= (1<<CS12) | (1<<CS10);
    #elif F_CPU <= 16000000L
      #define RF_TIMER_PRESCALER 32
      TCCR1B |= (1<<CS12) | (1<<CS11);
    #elif F_CPU <= 32000000L
      #define RF_TIMER_PRESCALER 64
      TCCR1B |= (1<<CS12) | (1<<CS11) | (1<<CS10);
    #else
      #error "CPU frequesncy higher than 32 MHz is not expected"
    #endif
    
    OCR1C = (F_CPU / RF_TIMER_PRESCALER) / (1000000L / PERIOD);
  #else // 16-bit timer (default)
    // Turn off Input Capture Noise Canceler, Input Capture Edge Select on Falling, stop the clock
    // Disconnect the timer from the output pins, Set Waveform Generation Mode to Normal
    TCCR1A = 0;
    TCCR1B = 0;
    #if defined(TIMSK)
      // Disable all Timer1 interrupts
      TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) | (1<<ICIE1));
      // Clear the Timer1 interrupt flags
      TIFR |= ((1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<ICF1));
    #elif defined(TIMSK1)
      // Disable all Timer1 interrupts
      TIMSK1 &= ~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) | (1<<ICIE1));
      // Clear the Timer1 interrupt flags
      TIFR1 |= ((1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<ICF1));
    #endif
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    
    // set Fast PWM CTC mode
    TCCR1A |= (1<<WGM11) | (1<<WGM10);
    TCCR1B |= (1<<WGM13) | (1<<WGM12);
    // set RF_TIMER_PRESCALER and counter
    #if F_CPU <= 500000L
      #define RF_TIMER_PRESCALER 1
      TCCR1B |= (1<<CS10);
    #elif F_CPU <= 4000000L
      #define RF_TIMER_PRESCALER 8
      TCCR1B |= (1<<CS11);
    #elif F_CPU <= 32000000L
      #define RF_TIMER_PRESCALER 64
      TCCR1B |= (1<<CS11) | (1<<CS10);
    #else
      #error "CPU frequesncy higher than 32 MHz is not expected"
    #endif
    
    OCR1A = (F_CPU / RF_TIMER_PRESCALER) / (1000000L / PERIOD);

  #endif

}
