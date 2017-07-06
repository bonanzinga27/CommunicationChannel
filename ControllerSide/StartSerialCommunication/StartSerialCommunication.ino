// this is where we will put our data
#include "Print.h"
unsigned char output[] = {};
unsigned char status_frame[] = {};
uint8_t crc[2] = {0x00, 0x00};
uint16_t full_crc = 0x0000;

#define HEADER1     0xFF
#define HEADER2     0xFF
#define HEADER3     0xFD

#define ACK         0x01
#define NACK        0x02

#define CRC16 0x1021

uint16_t gen_crc16(uint8_t *data, uint16_t size_data) {
  uint16_t out = 0;
  int bits_read = 0, bit_flag;
  /* Sanity check: */
  if (data == NULL)
    return 0;

  while (size_data > 0)
  {
    bit_flag = out >> 15;
    /* Get next bit: */
    out <<= 1;
    out |= (*data >> bits_read) & 1; // item a) work from the least significant bits
    /* Increment bit counter: */
    bits_read++;
    if (bits_read > 7)
    {
      bits_read = 0;
      data++;
      size_data--;
    }

    /* Cycle check: */
    if (bit_flag)
      out ^= CRC16;

  }
  // item b) "push out" the last 16 bits
  int i;
  for (i = 0; i < 16; ++i) {
    bit_flag = out >> 15;
    out <<= 1;
    if (bit_flag)
      out ^= CRC16;
  }
  // item c) reverse the bits
  uint16_t crc = 0;
  uint16_t ii = 0x8000;
  uint16_t j = 0x0001;
  uint16_t c;
  while (ii != 0) {
    c = ii & out;
    if (c) {
      crc |= j;
    }
    ii = ii >> 1;
    j = j << 1;
  }
  return crc;
}

void clear_buffer() {
  while (Serial.available() > 0)
    Serial.read();
}

boolean header_recognition() {
  boolean valid_header = false;
  int count = 0;
  unsigned char first_header = 0x00;
  unsigned char header;
  while (valid_header == false && count < 3) {
    header = Serial.read();
    delay(10);
    if (first_header == 0x00 && header == 0xFF) {
      first_header = header;
    } else if (first_header == 0xFF && header == 0xFD) {
      valid_header = true;
    }
    count++;
  }
  return valid_header;
}

bool data_reconstruction() {
  int buff_length = 0;
  int i;
  unsigned char* ptr;
  uint16_t received_crc;
  full_crc = 0x0000;
  if (header_recognition()) {
    buff_length = Serial.read();
    delay(10);
    for (i = 0; i < buff_length; i++) {
      output[i] = Serial.read();
      delay(10);
    }
    ptr = &output[0];
    received_crc = gen_crc16(ptr, buff_length - 2);
    full_crc = full_crc^output[buff_length-2];
    full_crc = full_crc^((uint16_t)output[buff_length-1]<<8);
    Serial.write(full_crc == received_crc);
    return true;
  } else {
    return false;
  }
}

int packaging(unsigned char id, unsigned char ack) {
  status_frame[0] = HEADER1;
  status_frame[1] = HEADER2;
  status_frame[2] = HEADER3;
  status_frame[3] = 0x02;
  status_frame[4] = id;
  status_frame[5] = ack;
  return sizeof(status_frame);
}

void action_to_perform() {
  unsigned char ack;
  int frame_length;
  /*
     Do something based on output[2] because it contain the next instruction to be executed
     Just choose to send and ack or nack
  */
  ack = ACK;                          // hard coding for test
  frame_length = packaging(output[0], ack);
  print_data(status_frame, 6);
}

void print_data(unsigned char data[], int frame_length) {
  /*for (int i = 0; i < frame_length; i++) {
    Serial.write(data[i]);
    //delay(2000);                   //used to test that read call is blocking
    }*/
  Serial.write(crc[0]);
  Serial.write(crc[1]);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino is ready");
  clear_buffer();
}

void loop() {
  while (Serial.available() > 0) {
    data_reconstruction();
    print_data(output, 6);
    //action_to_perform();
  }
}
