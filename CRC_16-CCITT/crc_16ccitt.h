//
// Created by root on 7/6/17.
//

#ifndef COMMUNICATIONCHANNEL_CRC_16CCITT_H
#define COMMUNICATIONCHANNEL_CRC_16CCITT_H

#include <stdint.h>
#include <stdio.h>
#define CRC16 0x1021

uint16_t gen_crc16(uint8_t *data, uint16_t size);


#endif //COMMUNICATIONCHANNEL_CRC_16CCITT_H
