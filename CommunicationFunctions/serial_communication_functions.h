/** Created by Andrea Bonanzinga on 7/6/17. **/

#ifndef COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_FUNCTIONS_H
#define COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_FUNCTIONS_H

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>
#include <stdbool.h>
#include "../CRC_16-CCITT/crc_16ccitt.h"
#include "../CommunicationProtocol/protocol_keywords.h"

int start_connection(int speed, unsigned int wait);
int set_interface_attribs (int fd, int speed, int parity, int should_block);
void clear_buffer(int fd);
void transmit_instruction_frame(int fd, int id, uint8_t instruction, uint8_t params[], uint8_t values [], int params_size);
_Bool receive_status_frame(int fd,int id, uint8_t instruction[]);
_Bool read_for_test(int fd);

#endif //COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_FUNCTIONS_H