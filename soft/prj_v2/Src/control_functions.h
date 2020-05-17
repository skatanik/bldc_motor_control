#pragma once
#include "main.h"
#include "bsp_functions.h"

typedef struct
{
    uint16_t rawPosition;
	uint16_t rawCurrent[4];
    uint16_t rawCurrentA;
    uint16_t rawCurrentB;
    uint16_t rawCurrentC;

    uint16_t pwmChannel1Val;
    uint16_t pwmChannel2Val;
    uint16_t pwmChannel3Val;

    uint8_t sendRawPosition;
    uint8_t sendPwmChannel1Val;
    uint8_t sendPwmChannel2Val;
    uint8_t sendPwmChannel3Val;

} globalState_typedef;

void initControl();
void updateControl();
void composeRegularMessage(uint8_t * data, uint16_t * size);
