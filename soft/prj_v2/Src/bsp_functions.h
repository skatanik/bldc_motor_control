#pragma once

#include "main.h"
#include <stdint.h>
#include "control_functions.h"

#define PWM_PERIOD  8500
#define MAIN_IFACE_UART

void bspStart();
void setPWM1(uint16_t val);
void setPWM2(uint16_t val);
void setPWM3(uint16_t val);
void sendUARTArray(uint8_t * data, uint16_t size);
void sendData(uint8_t * data, uint16_t size);
void startDataReceiving(uint8_t * data, uint16_t size);
int getPositionData(uint8_t * pos);