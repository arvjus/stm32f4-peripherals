// Wire Slave Transmitter and receiver
//Uno, Ethernet A4 (SDA), A5 (SCL)
#include <Wire.h>

uint8_t active_command = 0xff;
char name_msg[32] = "Welcome to arduino\n";

#define SLAVE_ADDR 0x68

uint8_t get_len_of_data(void) {
  return (uint8_t)strlen(name_msg);
}

void setup() {
  // Start the I2C Bus as Slave on address 9
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.begin(9600);
  Serial.println("Waiting for data from master");
}

//write
void receiveEvent(int bytes) {
  active_command = Wire.read();  // read one character from the I2C
}

//read
void requestEvent() {
  if (active_command == 0X51) {
    uint8_t len = get_len_of_data();
    Wire.write(&len, 1);
    active_command = 0xff;
  }


  if (active_command == 0x52) {
    Wire.write(name_msg, get_len_of_data());
    active_command = 0xff;
  }
}

void loop() {
}
