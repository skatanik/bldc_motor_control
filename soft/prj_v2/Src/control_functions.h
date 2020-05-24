#pragma once
#include "main.h"
#include "bsp_functions.h"

typedef struct
{
	uint8_t adcDataReady;
    uint16_t rawPosition;
	uint16_t rawCurrent[4];
    uint16_t rawCurrentA;
    uint16_t rawCurrentB;
    uint16_t rawCurrentC;
    uint16_t currentAfiltPrev;
    uint16_t currentBfiltPrev;
    uint16_t currentCfiltPrev;
    uint16_t currentAfilt;
    uint16_t currentBfilt;
    uint16_t currentCfilt;
    int16_t currentAscaled;
    int16_t currentBscaled;
    int16_t currentCscaled;
    uint16_t currentAscaler;
    uint16_t currentBscaler;
    uint16_t currentCscaler;
    uint16_t currentAoffset;
    uint16_t currentBoffset;
    uint16_t currentCoffset;

    uint16_t pwmChannel1Val;
    uint16_t pwmChannel2Val;
    uint16_t pwmChannel3Val;

    uint8_t sendRawPosition;
    uint8_t sendPwmChannel1Val;
    uint8_t sendPwmChannel2Val;
    uint8_t sendPwmChannel3Val;

	uint16_t electricalAngle;

	uint8_t receivedData [256];
	uint8_t dataReady;

    // control values
    uint16_t desiredSpeed;
    uint8_t runningEnabled;
    uint8_t sendDataEnabled;

} globalState_typedef;

void globalStateInit(void);
void initControl(void);
void updateControl(void);
void composeRegularMessage(uint8_t * data, uint16_t * size);
void updateCalc(void);
