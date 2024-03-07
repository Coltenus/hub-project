/*
 * NEC_Decode.c
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#include "NEC_Decode.h"
#include <stdio.h>
#include "main.h"

void println(const char*);
void DelayMS(uint16_t ms);
void DelayUS(uint16_t us);

uint8_t NEC_GetAddress(NEC* handle) {
    return handle->address;
}

uint8_t NEC_GetCommand(NEC* handle) {
    uint8_t res = handle->command, mask = 0b11111111, check = 0b10000000;

    if(!handle->flip) res = mask ^ res;
    if((check & res) >> 7) res = mask ^ res;

    return res;
}

bool NEC_Read(NEC* handle) {
    if(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin)) return false;
    DelayUS(7900);//8424);
    handle->flip = HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin);
    handle->address = 0;
    handle->command = 0;
    for(int count=0;count<8;count++){
        DelayUS(888);
        handle->address <<= 1;
        if(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin))
            handle->address |= 0x01;
    }
    for(int count=0;count<8;count++){
        DelayUS(888);
        handle->command <<= 1;
        if(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin))
            handle->command |= 0x01;
    }
    return true;
}

void NEC_Init(NEC* handle) {
    handle->address = 0;
    handle->command = 0;
    handle->flip = 0;
}
