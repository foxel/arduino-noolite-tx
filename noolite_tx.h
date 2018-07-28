/*
  NooliteTX - Library for transmitting NooLite RF codes
  Created by Andrey F. Kupreychik (c) 2017
*/
#ifndef NooliteTX_h
#define NooliteTX_h

class NooliteTX
{
  public:
    NooliteTX(uint8_t pin, uint16_t addr);
    void send_command(uint8_t cmd, uint8_t * argv, uint8_t argc);
    void send_command(uint8_t cmd);
  private:
    uint8_t _pin;
    uint8_t _addr_lo;
    uint8_t _addr_hi;
    uint8_t _flip;
};

#endif
