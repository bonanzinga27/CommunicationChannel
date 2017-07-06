/** Created by Andrea Bonanzinga on 7/6/17. **/

#include "serial_communication_functions.h"

int start_connection(){
    //unsigned char portname = portname_selection();
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_ASYNC);
    if (fd < 0) {
        perror("error");
        return -1;
    }
    fcntl(fd, F_SETFL, 0);
    set_interface_attribs(fd, B9600, 0, 0);             // set speed to 9.600 bps, 8n1 (no parity) and non blocking
    if( tcflush(fd,TCIOFLUSH) == 0)
        printf("%s successfully flushed.\n", "/dev/ttyACM0");
    sleep(2);
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
    tty.c_cc[VTIME] = 5;                            // Minimum time to wait for every character read (tenth of a second)
    if (tcsetattr (fd, TCSAFLUSH, &tty) != 0) {
        perror("error from tcsetattr-");
        return -1;
    }
    return 0;
}

void clear_buffer(int fd){
    tcflush(fd,TCIOFLUSH);
}

void transmit_instruction_frame(int fd, int id, int instruction, unsigned char params[], unsigned char values [], unsigned char params_size){

    unsigned char frame_length = (unsigned char)(4 + 2 * (int)params_size);      // Viene considerato anche id e istruzione
    unsigned char frame[ HEADER_LENGTH + frame_length +1 ];
    unsigned char instruction_type = 0x00;
    //printf("Frame length is: %d\n",HEADER_LENGTH + frame_length);

    frame[0]= HEADER1;
    frame[1]= HEADER2;
    frame[2]= HEADER3;
    frame[3]= frame_length;                                         //Length
    frame[4]= (unsigned char)id;                                    //ID
    /*
     * QUESTO SWITCH VA RIVISTO/ELIMINATO
     */
    switch(instruction){
        case 1:
            instruction_type = DIRECTION;
            break;
        case 2:
            instruction_type = TURN;
            break;
        case 3:
            instruction_type = RESET;
            break;
        default:
            printf("Sicuro di non aver sbagliato?");
            break;
    }
    frame[5] = instruction_type;
    int j = 5;
    //printf("j is %d",j);
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

    printf("CRC is: %X\n",crc);
    /*printf("CRC high is: %X\n",crc_high);
    printf("CRC low is: %X\n",crc_low);*/
    /*printf("\nI'm sending: ");
    for( int i = 0; i< sizeof(frame); i++) {
        printf("%X ",frame[i]);
    }*/
    write(fd, frame, sizeof(frame));
    return;
}

_Bool receive_status_frame(int fd,int id,unsigned char instruction[]){

    int count = 0;
    int buff_length;
    unsigned char* buf;
    unsigned char first_header = 0x00;
    _Bool valid_header = false;
    while (valid_header == false && count < 4) {
        read(fd, &buf,1);
        if (first_header == 0x00 && (unsigned char)buf ==  0xFF) {
            first_header = (unsigned char)buf;                                // ELIMINARE L'ASTERISCO SE DA PROBLEMI
        } else if (first_header == 0xFF && (unsigned char)buf == 0xFD) {
            valid_header = true;
        }
        count++;
    }
    if ( count == 4 )
        return NEGATIVE_ACK;

    read(fd,&buff_length,1);
    //printf("Buff Length is: %X\n",buff_length);
    for(int i = 0; i < buff_length; i++){
        read(fd, &instruction[i],1);
        printf("%X ",instruction[i]);
    }
    printf("\nI'm returning %X",instruction[1]);
    if( instruction[1] == POSITIVE_ACK)
        return true;                                          // Instruction[1] contain the acknowledgment information
    return false;
}
