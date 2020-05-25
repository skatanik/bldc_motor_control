#pragma once

#include "main.h"
#include <stdint.h>
#include "control_functions.h"

//#define ARM_MATH_CM4

#define PWM_PERIOD  (uint16_t)4250
#define MAIN_IFACE_UART

void bspStart(void);
void setPWM1(uint16_t val);
void setPWM2(uint16_t val);
void setPWM3(uint16_t val);
void sendUARTArray(uint8_t * data, uint16_t size);
void sendData(uint8_t * data, uint16_t size);
void startDataReceiving(uint8_t * data, uint16_t size);
int getPositionData(uint8_t * pos);
void cordicSetSin(void);
void cordicSetSin32(void);
void cordicSetPhase32(void);

