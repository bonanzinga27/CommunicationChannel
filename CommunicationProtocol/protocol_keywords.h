/** Created by Andrea Bonanzinga on 7/6/17. **/

#ifndef COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_H
#define COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_H

/******************************************************* INSTRUCTION FRAME ***********************************************************/
/**________________________________________________________________________________________________________________________________ **/
/**|          |          |      |               |           |           |          |           |          |                        |**/
/**|  HEADER  |  LENGTH  |  ID  |  INSTR. TYPE  |  PARAM 1  |  VALUE 1  |  . . .   |  PARAM N  |  VALUE N |  CRC_LOW  |  CRC_HIGH  |**/
/**--------------------------------------------------------------------------------------------------------------------------------|**/

/********************************** STATUS FRAME ************************/
/**____________________________________________________________________**/
/**|          |          |      |            |                        |**/
/**|  HEADER  |  LENGTH  |  ID  |  ACK_TYPE  |  CRC_LOW  |  CRC_HIGH  |**/
/**--------------------------------------------------------------------**/

/**  HEADER STRUCTURE  **/
#define     HEADER1                 0xFF
#define     HEADER2                 0xFF
#define     HEADER3                 0XFD
#define     HEADER_LENGTH           0X03

/**  INSTRUCTION TYPE  **/
#define     STRAIGHT                0x01
#define     ROTATE                  0x02
#define     RESET                   0x03

/**  INSTRUCTION PARAMS  **/
#define     INSTANTANEOUS           0x01
#define     TEMPORIZED              0x02
#define     DELAYED                 0x03
#define     DEGREE                  0x04

/**  ACK TYPE  **/
#define     ACK                     0X06
#define     NACK                    0X15

#endif //COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_H
