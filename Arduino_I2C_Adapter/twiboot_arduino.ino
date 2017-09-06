/**
   SIASFU - Simple I2C Adapter for SensorDot Firmware Update.

   Copyright (C) 2017  Blair Wyatt

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.



   For use with the SensorDot Bootloader.
   Performs everything in a state based structure.
   Each transaction has the following format:
   
   | 1byte | 1byte |  1byte  |  1byte  |num_bytes| 0xFF  |
   |ADDRESS|COMMAND|RET_BYTES|NUM_BYTES|  BYTES  |  END  |
   
   RET_BYTES is number of bytes expected to return if performing a read command,
   leave as 0 if doing a write command.
   Every incoming byte triggers a state change.
   Num_bytes can be number of bytes to write, or number of bytes in register to read.
   If end byte not received after num bytes, then it's assumed that the transaction failed.
   Adapter will timeout if a complete transaction doesn't arrive on the serial interface
   within a specified timeframe.

*/

#include <i2c_t3.h> //Teensy 3 I2C library.

#define I2C_WRITE       0x00
#define I2C_READ        0x01
#define BUFFER_SIZE     128

#define WAIT_STATE      0x00
#define ADDRESS_STATE   0x01
#define COMMAND_STATE   0x02
#define RET_LEN_STATE   0x03
#define LENGTH_STATE    0x04
#define BYTES_STATE     0x05
#define TRANSMIT_STATE  0x06
#define RECEIVE_STATE   0x07
#define END_STATE       0x08

#define END_BYTE        0xff


uint8_t num_bytes = 0;

uint8_t bytes_count = 0;
uint8_t data_buffer[BUFFER_SIZE];
uint8_t incoming_data = 0;
uint8_t return_num_bytes = 0;

uint8_t address = 0;
int8_t i2c_command = -1;

#define TIMEOUT 100000
uint32_t timeout = TIMEOUT;

uint8_t state = ADDRESS_STATE;

const int ledPin = 13;

void setup() {

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(115200);

  //Stop bus locking up when I2C glitches occur.
  Wire.setDefaultTimeout(200000);

  // initialize the LED pin as an output.
  pinMode(ledPin, OUTPUT);
}

void loop() {
  while (!Serial) {
    // Wait for serial port
  }

  if (Serial.available()) {
    // Read character from serial port
    incoming_data = Serial.read();
    digitalWrite(ledPin, HIGH);   // set the LED on
    timeout = TIMEOUT;
    handleData(incoming_data);
  } else {
    timeout--;
  }

  if (state == ADDRESS_STATE) digitalWrite(ledPin, LOW); // set the LED off

  if (timeout == 0)
  {
    digitalWrite(ledPin, LOW);    // set the LED off
    state = ADDRESS_STATE;
    timeout = TIMEOUT;
  }
}

void handleData(uint8_t data) {
  if (state == ADDRESS_STATE) {
    address = data;
    num_bytes = 0;
    state = COMMAND_STATE;
  }
  else if (state == COMMAND_STATE) {
    i2c_command = data;
    state = RET_LEN_STATE;
  }
  else if (state == RET_LEN_STATE) {
    return_num_bytes = data;
    state = LENGTH_STATE;
  }
  else if (state == LENGTH_STATE) {
    num_bytes = data;
    state = BYTES_STATE;
    bytes_count = 0;
    if (num_bytes == 0) state = END_STATE;
  }
  else if (state == BYTES_STATE && bytes_count < num_bytes) {
    data_buffer[bytes_count] = data;
    bytes_count++;
    if (bytes_count == num_bytes) state = END_STATE;
  }
  else if (state == END_STATE) {
    if (data == END_BYTE)
    {
      handleWireTrans();
    }
    state = ADDRESS_STATE;
  }
}

void handleWireTrans() {
  //Flush wire
  while (Wire.available()) {
    Wire.read();
  }

  //if (num_bytes > 0) {
  Wire.beginTransmission(address);
  for (int i = 0; i < num_bytes; i++) {
    Wire.write(data_buffer[i]);
  }
  Wire.endTransmission();
  //}
  delay(2);
  //Repeated Start for Read Register
  if (i2c_command == I2C_READ) {
    Wire.requestFrom(address, return_num_bytes);
    while (Wire.available())   // slave may send less than requested
    {
      char c = Wire.read();    // receive a byte as character
      Serial.print(c);         // print the character
      Serial.flush();
    }
  }
}

