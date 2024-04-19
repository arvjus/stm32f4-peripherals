#include <SPI.h>
#include <string.h>

/*
can receive 8MHz max
*/

#define PIN_LED 9
#define PIN_SENSOR A0

#define CMD_LED_CTRL 0x50
#define CMD_SENSOR_READ 0x51
#define CMD_ID_READ 0x52
#define CMD_MSG_WRITE 0x53

#define NONE 0xff

volatile uint8_t cmd = 0, sensor_value = 0, led_value = NONE, msg_received = false;
volatile uint8_t *ptr = 0;

char id[] = "12345";

char msg[500];
uint8_t len;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  // SPI slave
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  SPCR |= _BV(SPE);
  SPDR = 0; // this will be sent on 1st cmd
  SPI.attachInterrupt();

  Serial.println("Slave initialized");
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

// Note SPDR value sent back before this call
ISR(SPI_STC_vect) {
  uint8_t rxval = SPDR;
  if (!cmd) {
    cmd = rxval;
    if (cmd == CMD_SENSOR_READ) {
      SPDR = sensor_value;
    } else if (cmd == CMD_ID_READ) {
      SPDR = strlen(id);
      ptr = id;
    } else if (cmd == CMD_MSG_WRITE) {
      SPDR = 0;
      ptr = msg;
    } else {
      SPDR = 0;
    }
  } else {
    if (cmd == CMD_LED_CTRL) {
      led_value = rxval;
    } else if (cmd == CMD_ID_READ) {
      SPDR = *ptr;
      if (*ptr ++)
        return;
    } else if (cmd == CMD_MSG_WRITE) {
      SPDR = 0;
      *ptr = rxval;
      if (*ptr ++)
        return;
      msg_received = true;
    }
    cmd = 0;
    SPDR = 0; // this will be sent on next cmd
  }
}

void loop() {
  sensor_value = map(analogRead(PIN_SENSOR), 0, 1024, 0, 255);
  if (led_value != NONE) {
      digitalWrite(PIN_LED, led_value);
      led_value = NONE;
  }
  if (msg_received) {
    Serial.print("msg: ");
    Serial.println(msg);
    msg_received = false;
  }
}
