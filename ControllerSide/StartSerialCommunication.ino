// this is where we will put our data
unsigned char output[] = {};
unsigned char status_frame[] = {};

#define HEADER1     0xFF
#define HEADER2     0xFF
#define HEADER3     0xFD

#define ACK         0x01
#define NACK        0x02

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

void data_reconstruction() {
  int buff_length=0;
  int i;
  buff_length = Serial.read();
  delay(10);  
  for (i = 0; i < buff_length; i++) {
    output[i] = Serial.read();
    delay(10);
  }
  return;
}

int packaging(unsigned char id, unsigned char ack){
  status_frame[0] = HEADER1;
  status_frame[1] = HEADER2;
  status_frame[2] = HEADER3;
  status_frame[3] = 0x02;
  status_frame[4] = id;
  status_frame[5] = ack;
  return sizeof(status_frame);
}

void action_to_perform(){
  unsigned char ack;
  int frame_length;
  /*
   * Do something based on output[2] because it contain the next instruction to be executed
   * Just choose to send and ack or nack
   */
  ack = ACK;                          // hard coding for test
  frame_length = packaging(output[0],ack);
  print_data(status_frame,6);
}

void print_data(unsigned char data[],int frame_length){
  for(int i = 0; i< frame_length; i++){
    Serial.write(data[i]);
    //delay(2000);                   //used to test that read call is blocking
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino is ready");
  clear_buffer();
}

void loop() {
  while (Serial.available() > 0) {
    if ( header_recognition()) {
     data_reconstruction();
     //print_data(output,6);
     action_to_perform();
    }
  }
}
