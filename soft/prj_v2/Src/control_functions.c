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
float coeff_timing = 2.0/1.7320508;
uint16_t PWM_MAX_VAL = PWM_PERIOD;
float eq_first_part;
float t_a, t_b, t_zero;
uint16_t t1, t2, t3, t4;
uint8_t rawPosData[2];

uint16_t raw_angle;
float res_angle;
int cnt;

uint16_t fp_Vamp;
uint32_t fp_tConst = 0x1279a; // q2.16 2/sqrt(3)
uint32_t fp_firstPart;
int16_t sinB;
int16_t cosB;
int16_t sin60mB;
int16_t cos60mB;
uint32_t fp_t_a;
uint32_t fp_t_b;
uint32_t fp_t_zero;
int16_t fp_sixtyDeg = 10922;
uint16_t fp_betaAng;
uint16_t fp_Uv_ang;
uint32_t fp_cordic_inp;

extern uint16_t arm_A_raw_current;
extern uint16_t arm_B_raw_current;
extern uint16_t arm_C_raw_current;
uint8_t messageString[128] = {0};
uint16_t messageLength;

globalState_typedef globalState;
uint8_t rcvMsgInd = 0;
uint8_t controlByte = 0;
uint16_t dataBytes = 0;

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
    Uv_ampl = 0.3;
    cnt = 0;
    messageLength = 0;

}

void updateControl()
{
    HAL_Delay(1);

	fp_Vamp = 0x4fff ; // q16 amplitude 0xDDB2 max
    fp_Uv_ang += 182*5; // 1 degree 182
	if(fp_Uv_ang >= 0xFFFF)
		fp_Uv_ang = 0;

	if(fp_Uv_ang < fp_sixtyDeg)
    {
        sector_number = 1;
        fp_betaAng = fp_Uv_ang;
    } else if(fp_Uv_ang > fp_sixtyDeg && fp_Uv_ang < 2*fp_sixtyDeg)
    {
        sector_number = 2;
        fp_betaAng = fp_Uv_ang - fp_sixtyDeg;
    } else if(fp_Uv_ang > 2*fp_sixtyDeg && fp_Uv_ang < 3*fp_sixtyDeg)
    {
        sector_number = 3;
        fp_betaAng = fp_Uv_ang - 2*fp_sixtyDeg;
    } else if(fp_Uv_ang > 3*fp_sixtyDeg && fp_Uv_ang < 4*fp_sixtyDeg)
    {
        sector_number = 4;
        fp_betaAng = fp_Uv_ang - 3*fp_sixtyDeg;
    } else if(fp_Uv_ang > 4*fp_sixtyDeg && fp_Uv_ang < 5*fp_sixtyDeg)
    {
        sector_number = 5;
        fp_betaAng = fp_Uv_ang - 4*fp_sixtyDeg;
    } else if(fp_Uv_ang > 5*fp_sixtyDeg)
    {
        sector_number = 6;
        fp_betaAng = fp_Uv_ang - 5*fp_sixtyDeg;
    }



    /* Calculate Beta and sector */
    // sector_number = (uint8_t)floor(Uv_ang/60.0) + 1;
    // Beta_ang = Uv_ang - 60.0*(sector_number-1);
/*
11 pole pairs
32.727272 degree per electrical rotation
mechanical 0.0909 degree per one electrical degree
1 digit = 0.087890625 degree
*/
	fp_cordic_inp = (0x7fff << 16) + fp_betaAng;

	LL_CORDIC_WriteData(CORDIC, fp_cordic_inp);
	while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

    sinB = (uint16_t)LL_CORDIC_ReadData(CORDIC);

	fp_cordic_inp = (0x7fff << 16) + (fp_sixtyDeg - fp_betaAng);
    LL_CORDIC_WriteData(CORDIC, fp_cordic_inp);
	while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

    sin60mB = (uint16_t)LL_CORDIC_ReadData(CORDIC);

    fp_firstPart = (fp_Vamp*fp_tConst) >> 16; // q2.16
    fp_t_a = (PWM_MAX_VAL*((sin60mB * fp_firstPart)>> 15)) >> 16 ;
    fp_t_b = (PWM_MAX_VAL*((sinB * fp_firstPart)>> 15)) >> 16 ;
    fp_t_zero = PWM_MAX_VAL - fp_t_a - fp_t_b;

    t1 = (uint16_t) (fp_t_a + fp_t_b + (fp_t_zero >> 1));
    t2 = (uint16_t) (fp_t_b + (fp_t_zero >> 1));
    t3 = (uint16_t) (fp_t_a + (fp_t_zero >> 1));
    t4 = (uint16_t) (fp_t_zero >> 1);

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
//    setPWM1(globalState.pwmChannel1Val);
//    setPWM2(globalState.pwmChannel2Val);
//    setPWM3(globalState.pwmChannel3Val);

    /* Read encoder */

//    getPositionData(rawPosData);
//	globalState.rawPosition = (uint16_t)(rawPosData[0] << 8) + rawPosData[1];
//	res_angle = 360.0 / 4096.0 * globalState.rawPosition;

	if(cnt == 1)
	{
//		sprintf(messageString,"%f \n", res_angle);
//		sprintf(messageString,"%d %d %d %d\n", buff_data[0], buff_data[1], buff_data[2], buff_2[0]);
//		sprintf(messageString,"%d; %d; %d;\n", arm_A_raw_current, arm_B_raw_current, arm_C_raw_current);
//		globalState.rawCurrentA = globalState.pwmChannel1Val;
//		globalState.rawCurrentB = globalState.pwmChannel2Val;
//		globalState.rawCurrentC = globalState.pwmChannel3Val;
		globalState.rawCurrentA = globalState.rawCurrent[0];
		globalState.rawCurrentB = globalState.rawCurrent[3];
		globalState.rawCurrentC = globalState.rawCurrent[1];
        composeRegularMessage(messageString, &messageLength);
//        sendData(messageString, messageLength);
		cnt = 0;
	}
	cnt ++;

	if(globalState.dataReady)
	{
		globalState.dataReady = 0;

		while(rcvMsgInd != 63)
		{
			if(globalState.receivedData[rcvMsgInd++] == 0x49)
			{
                controlByte = globalState.receivedData[rcvMsgInd++];
                dataBytes = globalState.receivedData[rcvMsgInd++] * 256 + globalState.receivedData[rcvMsgInd];
				if(dataBytes >> 7) // write
				{
                    switch (dataBytes & 0x7f)
                    {
                    case 0x00:
                        globalState.runningEnabled = dataBytes;
                        break;
                    case 0x01:
                        globalState.desiredSpeed = dataBytes;
                        break;
                    case 0x02:
                        globalState.sendDataEnabled = dataBytes;
                        break;
                    default:
                        break;
                    }
				} else // read
                {

                }
			}
		}
	}

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

