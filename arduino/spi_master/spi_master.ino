#include <SPI.h>

#define PIN_BUTTON 8

/*
SPI_CLOCK_DIV2 - 8MHz max
*/

#define CMD_LED_CTRL 0x50
#define CMD_SENSOR_READ 0x51
#define CMD_ID_READ 0x52
#define CMD_MSG_WRITE 0x53

char msg[] = "hello arduino!";

void setup(void) {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // SPI master
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 16 / 8 = 2MHz
  digitalWrite(SS, HIGH);

  Serial.println("Master initialized");
}

void send(uint8_t data) {
  SPDR = data;
  __asm("nop");
  while (!(SPSR & (1 << SPIF)))
    ;
}

uint8_t receive() {
  while (!(SPSR & (1 << SPIF)))
    ;
  uint8_t data = SPDR;
  __asm("nop");
  return data;
}


void command(uint8_t cmd) {
  uint8_t data;

  data = SPI.transfer(cmd);
  Serial.print("cmd: ");
  Serial.print(String(cmd, HEX));
  Serial.print(" -> ");
  Serial.print(String(data, HEX));
  delay(1);
}

uint8_t argument(uint8_t arg) {
  uint8_t data;

  data = SPI.transfer(arg);
  Serial.print(", arg: ");
  Serial.print(String(arg, HEX));
  Serial.print(" -> ");
  Serial.println(String(data, HEX));
  return data;
}

void loop(void) {
  uint8_t data, len;

  // 1. wait for PIN_BUTTON low
  while (digitalRead(PIN_BUTTON))
    ;
  delay(200);

  // put SS low
  digitalWrite(SS, LOW);
  delayMicroseconds(20);

  command(CMD_LED_CTRL);
  argument(1);
  delay(500);

  command(CMD_SENSOR_READ);
  argument(0);
  delay(500);

  command(CMD_LED_CTRL);
  argument(0);
  delay(500);
  
  command(CMD_ID_READ);
  len = argument(0);
  Serial.print("id: ");
  for (int i = 0; i < len; i ++) 
    Serial.print((char)SPI.transfer(0));
  Serial.println();

  command(CMD_MSG_WRITE);
  for (int i = 0; i < sizeof(msg); i ++) 
    SPI.transfer(msg[i]);
  Serial.println();

  // put SS high
  digitalWrite(SS, HIGH);
}
