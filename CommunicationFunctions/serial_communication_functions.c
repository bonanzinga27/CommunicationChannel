/** Created by Andrea Bonanzinga on 7/6/17. **/

#include "serial_communication_functions.h"

int start_connection(int speed, int wait){
    //unsigned char portname = portname_selection();
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_ASYNC);
    if (fd < 0) {
        perror("error");
        return -1;
    }
    fcntl(fd, F_SETFL, 0);
    if (set_interface_attribs(fd, speed, 0, 0)) {           // set speed to 9.600 bps, 8n1 (no parity) and non blocking
        printf("Error setting interface attributes");
        return -1;
    }
    if( tcflush(fd,TCIOFLUSH) == 0)
        printf("%s successfully flushed.\n", "/dev/ttyACM0");
    sleep(wait);
    return fd;
}

int set_interface_attribs (int fd, int speed, int parity, int should_block) {
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0) {
        perror("error from tcgetattr-");
        return -1;
    }
    cfsetospeed (&tty, (speed_t)speed);                      // Output speed
    cfsetispeed (&tty, (speed_t)speed);                      // Input speed
    /* Control option */
    tty.c_cflag |= (CLOCAL | CREAD);                // Don't change owner of the port and enable receiver
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // Mask character size and set it to 8-bit
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    /* Local option */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // no signaling chars, no echo,
    /* Input option */
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_iflag |= (IXON | IXOFF | IXANY);          // Sotware flow control
    /* Output control */
    tty.c_oflag = 0;                                // No remapping, no delays
    /* Control characters */
    tty.c_cc[VMIN]  = should_block ? (cc_t)6 : (cc_t)0;         // Minimum number of character to be read
    tty.c_cc[VTIME] = 1;                            // Minimum time to wait for every character read (tenth of a second)
    if (tcsetattr (fd, TCSAFLUSH, &tty) != 0) {
        perror("error from tcsetattr-");
        return -1;
    }
    printf("Serial inferface correctly configured\n");
    return 0;
}

void clear_buffer(int fd){
    int n=0;
    uint8_t* buf;
    do{
        read(fd,&buf,1);
    }while(n>0);
}

void transmit_instruction_frame(int fd, int id, uint8_t instruction, uint8_t params[], uint8_t values [], int params_size){
    uint8_t content_length = ( uint8_t )(4 + 2 * params_size);
    uint8_t frame[ HEADER_LENGTH + content_length +1 ];

    frame[0]= HEADER1;
    frame[1]= HEADER2;
    frame[2]= HEADER3;
    frame[3]= content_length;                                         //Length
    frame[4]= ( uint8_t )id;                                    //ID
    frame[5] = instruction;
    int j = 5;
    for(int i = 0; i < params_size; i++) {
        j++;
        frame[j] = params[i];
        j++;
        frame[j] = values[i];
    }

    uint8_t* data = &frame[4];
    uint16_t crc = gen_crc16(data,4);
    uint8_t crc_high, crc_low;
    crc_high = (uint8_t) (crc >> 8);
    crc_low = (uint8_t )((uint16_t )(crc_high << 8) ^ crc);
    j++;
    frame[j] = crc_low;
    j++;
    frame[j] = crc_high;
    /*printf("\nCRC is: %X\n",crc);
    for( int i = 0; i< sizeof(frame); i++) {
        printf("%X ",frame[i]);
    }*/
    write(fd, frame, sizeof(frame));
    return;
}

_Bool receive_status_frame(int fd, int id, uint8_t instruction[]){
    int count = 0;
    int buff_length;
    uint8_t* buf;
    uint8_t first_header = 0x00;
    _Bool valid_header = false;

    while (valid_header == false && count < 3) {
        read(fd, &buf,1);
        if (first_header == 0x00 && ( uint8_t )buf ==  0xFF) {
            first_header = ( uint8_t )buf;
        } else if (first_header == 0xFF && ( uint8_t )buf == 0xFD) {
            valid_header = true;
        }
        count++;
    }
    if ( count == 4 )
        return false;
    read(fd,&buff_length,1);
    for(int i = 0; i < buff_length; i++){
        read(fd, &instruction[i],1);
        printf("%X ",instruction[i]);
    }
    if( instruction[1] == ACK && instruction[0] == id) {              // Instruction[1] contain the acknowledgment information meanwhile instruction[0] contain the ID
        printf("\nInstruction sent successfully\n");
        return true;
    }else{
        printf("\nError occur: retrying\n");
        return false;
    }
}

void read_for_test(int fd){
    ssize_t n=0;
    unsigned char buf;
    do{
        n = read(fd,&buf,1);
        printf("%X ",buf);
    }while(n>0);
}