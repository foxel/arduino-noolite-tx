#include <noolite_tx.h>
#include <SimpleDHT.h>

#define DHTPIN      16
#define RFPIN       14
#define BINDPIN     2
#define SENSOR_ADDR 0xAA56

SimpleDHT22 dht22;
NooliteTX tx(RFPIN, SENSOR_ADDR);

void setup()
{
  // bind
  // this will send BIND command
  pinMode(BINDPIN, INPUT_PULLUP);
  if (digitalRead(BINDPIN) == LOW) {
    tx.send_command(15);
  }
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

  delay(20000);
}
