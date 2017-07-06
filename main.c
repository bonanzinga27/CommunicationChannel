/** Created by Andrea Bonanzinga on 7/6/17. **/

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include "CommunicationFunctions/serial_communication_functions.h"

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
            do {
                clear_buffer(fd);
                transmit_instruction_frame(fd, id, s, params, values, params_size);
                usleep(100);
                /*int n=0;
                unsigned char buf;
                do{
                    n = read(fd,&buf,1);
                    printf("%X ",buf);
                }while(n>0);*/
                isAck = receive_status_frame(fd, id, frame);
            }while(!isAck);
        }
        id++;
    }while(s >= 1 && s <= 4 );
    close(fd);
    return 0;
}