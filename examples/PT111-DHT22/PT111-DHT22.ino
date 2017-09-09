#include <noolite_tx.h>
#include <Sleep_n0m1.h>
#include <avr/power.h>
#include <SimpleDHT.h>

#define DHTPIN      16
#define RFPIN       14
#define BINDPIN     2
#define SENSOR_ADDR 0xAA56

SimpleDHT22 dht22;
NooliteTX tx(RFPIN, SENSOR_ADDR);
Sleep sleep;

void setup()
{
  delay(1000);
  // bind
  pinMode(BINDPIN, INPUT_PULLUP);
  if (digitalRead(BINDPIN) == LOW) {
    tx.send_command(15);
  }
  
  delay(20000);
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_timer3_disable();
  power_usart1_disable();
  // disable USB
  power_usb_disable();
  USBCON |= (1 << FRZCLK);             // Freeze the USB Clock              
  PLLCSR &= ~(1 << PLLE);              // Disable the USB Clock (PPL) 
  USBCON &=  ~(1 << USBE  );           // Disable the USB  
  sleep.pwrDownMode(); //set sleep mode
}

void loop()
{
  float humidity = 0;
  float temp = 0;

  if (dht22.read2(DHTPIN, &temp, &humidity, NULL) == SimpleDHTErrSuccess) {
    int deci_temp = int(temp*10);
    byte deci_hum = byte(humidity);

    uint8_t tx_args[] = {
      deci_temp & 0xff,
      (deci_temp >> 8) & 0x0f | 0x20,
      deci_hum,
      0xff // can be some analog sensor
    };
  
    tx.send_command(21, tx_args, 4);
  }


  sleep.sleepDelay(20000);
}
