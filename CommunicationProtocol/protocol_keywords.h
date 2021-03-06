/** Created by Andrea Bonanzinga on 7/6/17. **/

#ifndef COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_H
#define COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_H

/*************************************************** INSTRUCTION FRAME ***************************************************************/
/**________________________________________________________________________________________________________________________________ **/
/**|          |          |      |               |           |           |          |           |          |           |            |**/
/**|  HEADER  |  LENGTH  |  ID  |  INSTR. TYPE  |  PARAM 1  |  VALUE 1  |  . . .   |  PARAM N  |  VALUE N |  CRC_LOW  |  CRC_HIGH  |**/
/**--------------------------------------------------------------------------------------------------------------------------------|**/

/********************************** STATUS FRAME ************************/
/**____________________________________________________________________**/
/**|          |          |      |            |           |            |**/
/**|  HEADER  |  LENGTH  |  ID  |  ACK_TYPE  |  CRC_LOW  |  CRC_HIGH  |**/
/**--------------------------------------------------------------------**/

/** Parole chiave utilizzati dal protocollo per la comunicazione
 * NB: affinchè vi si una perfetta sincronizzazione con il ricevitore, esso deve contenere gli stessi valori.
 */

/**  HEADER STRUCTURE  **/
#define     HEADER1                 0xFF
#define     HEADER2                 0xFF
#define     HEADER3                 0XFD
#define     HEADER_LENGTH           0X03
#define     ID_LENGTH               0x01
#define     INSTRUCTION_LENGTH      0x01
#define     CRC_LENGTH              0x02

/**  INSTRUCTION TYPE  **/
#define     STRAIGHT                0x01
#define     ROTATE                  0x02
#define     SET_LAPLACIAN           0x03
#define     RESET                   0x04

/**  INSTRUCTION PARAMS  **/
#define     INSTANTANEOUS           0x01
#define     TEMPORIZED              0x02
#define     DELAYED                 0x03
#define     DEGREE                  0x04
#define     LAPLACIAN_NUMBER        0x05

/**  ACK TYPE  **/
#define     ACK                     0X06
#define     NACK                    0X15

#define     THRESHOLD               0x05
#endif //COMMUNICATIONCHANNEL_SERIAL_COMMUNICATION_H

