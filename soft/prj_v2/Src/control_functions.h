#pragma once
#include "main.h"
#include "bsp_functions.h"
#include <arm_math.h>

typedef struct
{
	uint8_t adcDataReady;

    // current calc
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
    int32_t currentAscaled;
    int32_t currentBscaled;
    int32_t currentCscaled;
    uint16_t currentAscaler;
    uint16_t currentBscaler;
    uint16_t currentCscaler;
    uint16_t currentAoffset;
    uint16_t currentBoffset;
    uint16_t currentCoffset;

    // clark
    int32_t alfaI;
    int32_t betaI;
    int32_t thetaCos;
    int32_t thetaSin;

    // park
    int32_t currQ;
    int32_t currD;

    // errors
    int32_t errorQ;
    int32_t errorD;

    // pid Q
    arm_pid_instance_q31 PIDQ;
    int32_t currQres;

    // pid D
    arm_pid_instance_q31 PIDD;
    int32_t currDres;

    //inv Park
    int32_t alfaV;
    int32_t betaV;

    // phase and amplitude
    int32_t abPhase;
    int32_t abAmpl;


    uint16_t pwmChannel1Val;
    uint16_t pwmChannel2Val;
    uint16_t pwmChannel3Val;

    // position
    uint16_t rawPosition;
	int32_t electricalAngle;

    // communication
	uint8_t receivedData [256];
	uint8_t dataReady;

    // control values
    uint16_t desiredSpeed;
    uint8_t runningEnabled;
    uint8_t sendDataEnabled;
    uint8_t sendRawPosition;
    uint8_t sendPwmChannel1Val;
    uint8_t sendPwmChannel2Val;
    uint8_t sendPwmChannel3Val;
    int32_t desiredCurrQ;

} globalState_typedef;

void globalStateInit(void);
void initControl(void);
void updateControl(void);
void composeRegularMessage(uint8_t * data, uint16_t * size);
void updateCalc(void);

