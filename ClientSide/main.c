#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


/***************************************************** INSTRUCTION FRAME ***************************************************/
/**______________________________________________________________________________________________________________________ **/
/**|          |          |      |               |           |           |          |           |          |              |**/
/**|  HEADER  |  LENGTH  |  ID  |  INSTR. TYPE  |  PARAM 1  |  VALUE 1  |  . . .   |  PARAM N  |  VALUE N |  16 BIT CRC  |**/
/**----------------------------------------------------------------------------------------------------------------------|**/

/********************* STATUS FRAME ********************/
/**___________________________________________________**/
/**|          |          |      |            |       |**/
/**|  HEADER  |  LENGTH  |  ID  |  ACK_TYPE  |  CRC  |**/
/**---------------------------------------------------**/


/**  HEADER STRUCTURE  **/
#define     HEADER1                 0xFF
#define     HEADER2                 0xFF
#define     HEADER3                 0XFD
#define     HEADER_LENGTH           0X03

/**  INSTRUCTION TYPE  **/
#define     DIRECTION               0x49
#define     TURN                    0x02
#define     RESET                   0x03

/**  INSTRUCTION PARAMS  **/
#define     TEMPORIZED              0x41
#define     DELAYED                 0x02
#define     INSTANTANEOUS           0x03
#define     DEGREE                  0x04

#define STOP_COMMAND            0x00
#define FORWARD_COMMAND         0X01
#define RIGHT_COMMAND           0X02
#define LEFT_COMMAND            0X03
#define BACK_COMMAND            0X04

/**  ACK TYPE  **/
#define POSITIVE_ACK            0X01
#define NEGATIVE_ACK            0X02

/**  CRC-CCIT 16 BIT POLYNOMIAL   **/
#define CRC16 0x1021

uint16_t gen_crc16(uint8_t *data, uint16_t size) {
    uint16_t out = 0;
    int bits_read = 0, bit_flag;
    /* Sanity check: */
    if(data == NULL)
        return 0;
    while(size > 0)
    {
        bit_flag = out >> 15;
        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits
        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;
    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out)
            crc |= j;
    }

    return crc;
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

/*
 * unsigned char portname_selection(){
    unsigned char pName[256];
    unsigned char default_port[256] = "/dev/ttyACM0";
    printf("Type the serial port for the communication (type default for /dev/ttyACM0:)\n");
    scanf("%s", &pName);
    if (strcmp(pName,"default") == 0){
        return "/dev/ttyACM0";
    }else{
        return pName;
    }
}
 */

void clear_buffer(int fd){
    tcflush(fd,TCIOFLUSH);
}

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

int instruction_menu(void){
    int s;
    printf("\nType the action to perform: \n");
    printf("1) Go forward\n");
    printf("2) Go right\n");
    printf("3) Go left\n");
    printf("4) Go behind\n");
    printf("Other to Exit\n");
    scanf("%d",&s);
    return s;
}

/*int param_menu(void){

}*/

void transmit_instruction_frame(int fd, int id, int instruction, unsigned char params[], unsigned char values [], unsigned char params_size){

    unsigned char frame_length = (unsigned char)(5 + 2 * (int)params_size);      // Viene considerato anche id e istruzione
    unsigned char frame[ HEADER_LENGTH + frame_length ];
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
    printf("\nI'm sending: ");
    for( int i = 0; i< sizeof(frame); i++) {
        printf("%X ",frame[i]);
    }
    write(fd, frame, sizeof(frame));
    return;
}

/*_Bool header_reconstruction(int fd){
    *//*
     * TO BE IMPLEMENTED
     *//*
}*/

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
    return instruction[1];                                          // Instruction[1] contain the acknowledgment information
}

int main() {
    int fd = start_connection();                                    // File descriptor
    int s;                                                          // Menu variable
    int id = 0x43;                                                  // Starting IDENTIFICATOR
    unsigned char frame[]={};                                       // Frame array
    unsigned char params[] = { TEMPORIZED };                        // Parameters
    unsigned char values[] = { 0X4F };                              // Values of the parameters
    unsigned char params_size = (unsigned char) sizeof(params);     // Parameters size
    _Bool isAck = false;                                            // Return true if an ACK is received
    if(fd < 0){                                                     // Sanity check
        perror("Something goes wrong");
        return -1;
    }

    do {
        s = instruction_menu();
        if( s >= 1 && s <= 4 ) {
                clear_buffer(fd);
                transmit_instruction_frame(fd, id, s, params, values, params_size);
                usleep(100);
                receive_status_frame(fd, id, frame);
        }
        id++;
    }while(s >= 1 && s <= 4 );
    close(fd);
    return 0;
}
