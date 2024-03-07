/*
 * NEC_Decode.h
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#ifndef INC_NEC_DECODE_H_
#define INC_NEC_DECODE_H_

#include <stdbool.h>
#include <stdint.h>

#define DMA_LEN 1
#define DATA_LEN 21

typedef struct {
    uint8_t flip, address, command;
} NEC;

void NEC_Init(NEC* handle);
bool NEC_Read(NEC* handle);
uint8_t NEC_GetAddress(NEC* handle);
uint8_t NEC_GetCommand(NEC* handle);

#endif /* INC_NEC_DECODE_H_ */
