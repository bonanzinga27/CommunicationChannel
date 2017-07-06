/** Created by Andrea Bonanzinga on 7/6/17. **/

#ifndef COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_FUNCTIONS_H
#define COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_FUNCTIONS_H

int set_interface_attribs (int fd, int speed, int parity, int should_block);
int start_connection();
void clear_buffer(int fd);
void transmit_instruction_frame(int fd, int id, int instruction, unsigned char params[], unsigned char values [], unsigned char params_size);
_Bool receive_status_frame(int fd,int id,unsigned char instruction[]);

#endif //COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_FUNCTIONS_H
