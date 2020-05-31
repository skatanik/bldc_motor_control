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
    int32_t desiredCurrQ;
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
    uint32_t abAmpl;

    uint16_t pwmChannel1Val;
    uint16_t pwmChannel2Val;
    uint16_t pwmChannel3Val;

    // position
    uint16_t rawPosition;
	int32_t electricalAngle;
	uint16_t posOffset;

    // communication
	uint8_t receivedData [256];
	uint8_t dataReady;

    // control values
    uint16_t desiredSpeed;
    uint8_t runningEnabled;
    uint8_t sendDataEnabled;
    uint8_t sendRawPosition;
    uint8_t sendCurrentA;
    uint8_t sendCurrentB;
    uint8_t sendCurrentC;
    uint8_t sendAlfaI;
    uint8_t sendBetaI;
    uint8_t sendCurrQ;
    uint8_t sendCurrD;
    uint8_t sendErrorQ;
    uint8_t sendErrorD;
    uint8_t sendDesiredCurrQ;
    uint8_t sendCurrQres;
    uint8_t sendCurrDres;
    uint8_t sendAlfaV;
    uint8_t sendBetaV;
    uint8_t sendAbPhase;
    uint8_t sendAbAmpl;
	uint8_t testModeEn;
	uint16_t testAmpl;

} globalState_typedef;

void globalStateInit(void);
void initControl(void);
void updateControl(void);
void composeRegularMessage(uint8_t * data, uint16_t * size);
void updateCalc(void);

