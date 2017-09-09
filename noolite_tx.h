/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef NooliteTX_h
#define NooliteTX_h

#include "Arduino.h"

class NooliteTX
{
  public:
    NooliteTX(int pin, uint16_t addr);
    void send_command(uint8_t cmd, uint8_t * argv, uint8_t argc);
    void send_command(uint8_t cmd);
  private:
    uint8_t _pin;
    uint8_t _addr_lo;
    uint8_t _addr_hi;
    uint8_t _flip;
};

#endif
