#include "control_functions.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

uint16_t pulse_ch4 = 0;
float Uv_ampl = 0;
float Uv_ang = 0;
float rot_ang_global = 0;

uint8_t sector_number;
float Beta_ang;
float MAX_MODULATION = 0.8660254;
float coeff_timing = 1.7320508;
float PWM_MAX_VAL = PWM_PERIOD;
float eq_first_part;
float t_a, t_b, t_zero;
uint16_t t1, t2, t3, t4;
uint8_t rawPosData[2];

uint16_t raw_angle;
float res_angle;
int cnt;

extern uint16_t arm_A_raw_current;
extern uint16_t arm_B_raw_current;
extern uint16_t arm_C_raw_current;
uint8_t messageString[128] = {0};
uint16_t messageLength;

globalState_typedef globalState;

void initControl()
{
    bspStart();
    globalState.pwmChannel1Val = 0;
    globalState.pwmChannel2Val = 0;
    globalState.pwmChannel3Val = 0;
    globalState.sendPwmChannel1Val = 1;
    globalState.sendPwmChannel2Val = 1;
    globalState.sendPwmChannel3Val = 1;
    globalState.sendRawPosition = 1;
    Uv_ampl = 0.8;
    cnt = 0;
    messageLength = 0;

}

void updateControl()
{
    HAL_Delay(1);
    Uv_ang += 5;
	if(Uv_ang >= 360)
		Uv_ang = 0;

    /* Calculate Beta and sector */
    sector_number = (uint8_t)floor(Uv_ang/60.0) + 1;
    Beta_ang = Uv_ang - 60.0*(sector_number-1);

    /* Calculate t_a, t_b, t_0*/
    eq_first_part = coeff_timing * PWM_MAX_VAL * Uv_ampl;

    t_a = eq_first_part * sin((60.0-Beta_ang)*(float)0.0035367765);
    t_b = eq_first_part * sin((Beta_ang)*(float)0.0035367765);
    t_zero = PWM_MAX_VAL - t_a - t_b;

    // t_a     = Uv_ampl * PWM_MAX_VAL * sin((60.0-Beta_ang)*(float)0.0035367765);
    // t_b     = Uv_ampl * PWM_MAX_VAL * sin((Beta_ang)*(float)0.0035367765);
    // t_zero  = PWM_MAX_VAL - t_a - t_b;

    if(t_zero < 0)
        t_zero = 0;

    /* Set pulse values */
    t1 = (uint16_t) floor(t_a + t_b + t_zero/2.0);
    t2 = (uint16_t) floor(t_b + t_zero/2.0);
    t3 = (uint16_t) floor(t_a + t_zero/2.0);
    t4 = (uint16_t) floor(t_zero/2.0);

    switch (sector_number)
    {
    case 1:
        globalState.pwmChannel1Val = t1;
        globalState.pwmChannel2Val = t2;
        globalState.pwmChannel3Val = t4;
		pulse_ch4 = globalState.pwmChannel2Val;
        break;
    case 2:
        globalState.pwmChannel1Val = t3;
        globalState.pwmChannel2Val = t1;
        globalState.pwmChannel3Val = t4;
		pulse_ch4 = globalState.pwmChannel1Val;
        break;
    case 3:
        globalState.pwmChannel1Val = t4;
        globalState.pwmChannel2Val = t1;
        globalState.pwmChannel3Val = t2;
		pulse_ch4 = globalState.pwmChannel3Val;
        break;
    case 4:
        globalState.pwmChannel1Val = t4;
        globalState.pwmChannel2Val = t3;
        globalState.pwmChannel3Val = t1;
		pulse_ch4 = globalState.pwmChannel2Val;
        break;
    case 5:
        globalState.pwmChannel1Val = t2;
        globalState.pwmChannel2Val = t4;
        globalState.pwmChannel3Val = t1;
		pulse_ch4 = globalState.pwmChannel1Val;
        break;
    case 6:
        globalState.pwmChannel1Val = t1;
        globalState.pwmChannel2Val = t4;
        globalState.pwmChannel3Val = t3;
		pulse_ch4 = globalState.pwmChannel3Val;
        break;

    default:
        globalState.pwmChannel1Val = t1;
        globalState.pwmChannel2Val = t2;
        globalState.pwmChannel3Val = t4;
		pulse_ch4 = t2;
        break;
    }


    /* Set timer pulse */
    setPWM1(globalState.pwmChannel1Val);
    setPWM2(globalState.pwmChannel2Val);
    setPWM3(globalState.pwmChannel3Val);

    /* Read encoder */

    getPositionData(rawPosData);
	globalState.rawPosition = (uint16_t)(rawPosData[0] << 8) + rawPosData[1];
	res_angle = 360.0 / 4096.0 * globalState.rawPosition;

	if(cnt == 1)
	{
//		sprintf(messageString,"%f \n", res_angle);
//		sprintf(messageString,"%d %d %d %d\n", buff_data[0], buff_data[1], buff_data[2], buff_2[0]);
//		sprintf(messageString,"%d; %d; %d;\n", arm_A_raw_current, arm_B_raw_current, arm_C_raw_current);
        composeRegularMessage(messageString, &messageLength);
        sendData(messageString, messageLength);
		cnt = 0;
	}
	cnt ++;

}

void composeRegularMessage(uint8_t * data, uint16_t * size)
{
    uint8_t index = 0;
    data[index++] = (uint8_t)0x48;

	if(globalState.sendRawPosition)
    {
//        data[index++] = (uint8_t)1;
//        data[index++] = (uint8_t)2;
        data[index++] = (uint8_t)(globalState.rawPosition >> 8);
        data[index++] = (uint8_t)globalState.rawPosition;
    }

    if(globalState.sendPwmChannel1Val)
    {
//        data[index++] = (uint8_t)2;
//        data[index++] = (uint8_t)2;
        data[index++] = (uint8_t)(globalState.rawCurrentA >> 8);
        data[index++] = (uint8_t)globalState.rawCurrentA;
    }

    if(globalState.sendPwmChannel2Val)
    {
//        data[index++] = (uint8_t)3;
//        data[index++] = (uint8_t)2;
        data[index++] = (uint8_t)(globalState.rawCurrentB >> 8);
        data[index++] = (uint8_t)globalState.rawCurrentB;
    }

    if(globalState.sendPwmChannel3Val)
    {
//        data[index++] = (uint8_t)4;
//        data[index++] = (uint8_t)2;
        data[index++] = (uint8_t)(globalState.rawCurrentC >> 8);
        data[index++] = (uint8_t)globalState.rawCurrentC;
    }
    *size = index;

}

