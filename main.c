/** Created by Andrea Bonanzinga on 7/6/17. **/

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include "CommunicationFunctions/serial_communication_functions.h"

uint8_t instruction_menu(void){
    int s;
    printf("\nType the action to perform: \n");
    printf("1) Camminata lenta\n");
    printf("2) Trotto\n");
    printf("3) Galoppo\n");
    printf("Other to Exit\n");
    scanf("%d",&s);
    return (uint8_t )s;
}

int main() {
    int fd = start_connection(B115200,2);                     // File descriptor
    int id = 0x01, retransmission;                            // Starting IDENTIFICATOR
    uint8_t frame[]={};                                       // Frame array

    /** Impostazione manuale dei parametri da inviare, eccetto per values che verrà popolato dall'utente **/
    uint8_t instruction = SET_LAPLACIAN;
    uint8_t params[] = { LAPLACIAN_NUMBER };                  // Parameters
    uint8_t values[] = { };                                   // Values of the parameters

    uint8_t params_size = (uint8_t) sizeof(params);           // Parameters size
    _Bool isAck = true;                                       // Return true if an ACK is received
    if(fd < 0){                                               // Sanity check
        perror("Something goes wrong");
        return -1;
    }

    /** Durante il primo do_while ci si assicura che il valore introdotto dall'utente sia corretto **/
    do {
        values[0] = instruction_menu();
        if( values[0] >= 0x01 && values[0] <= 0x03 ) {
            retransmission = 0;

            /** Nel secondo do_while ci si assicura che sia arrivato un ACK.
             *  Nel caso in cui si riceva un NACK, si ritenta la transmissione per un numero limitato di volte
             *  specificato dalla costante THRESHOLD.
             */
            do {
                printf("Transmission\n");
                clear_buffer(fd);
                transmit_instruction_frame(fd, id, instruction, params, values, params_size);
                retransmission ++;
                isAck = receive_status_frame(fd, id, frame);

                /* In fase di test, decommentare la funzione read_for_test e commmentare la receive_status_frame */
                //isAck = read_for_test(fd);
            }while(!isAck && retransmission < THRESHOLD);
            if(retransmission >= THRESHOLD){
                printf("Impossible to sent data, fix problem before retrying");
                return -1;
            }
        }
        id++;
    }while(values[0] >= 0x01 && values[0] <= 0x03 );

    close(fd);
    return 0;
}
