#include "control_functions.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_MOD_VECTOR_VAL  (uint16_t) 0xDDB2
#define SQRT3_M1        (uint16_t)0x93CD // 16 bit  1/sqrt(3) = 0,5773502691896 * 2^16

uint16_t pulse_ch4 = 0;
float Uv_ampl = 0;
float Uv_ang = 0;
float rot_ang_global = 0;

uint8_t sector_number;
float Beta_ang;
float MAX_MODULATION = 0.8660254;
float coeff_timing = 2.0/1.7320508;
float eq_first_part;
float t_a, t_b, t_zero;
uint16_t t1, t2, t3, t4;
uint8_t rawPosData[2];

int16_t raw_angle;
float res_angle;
int cnt;

/*swpwv*/
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
uint16_t currAscaled;
uint16_t currBscaled;
uint16_t currCscaled;

/*uart*/
uint8_t messageString[128] = {0};
uint16_t messageLength;

globalState_typedef globalState;
uint8_t rcvMsgInd = 0;
uint8_t controlByte = 0;
uint16_t dataBytes = 0;

uint32_t val_before;
uint32_t val_after;

uint8_t ind,ind2;

uint8_t adcInterruptCnt = 0;

void globalStateInit(void)
{
    globalState.pwmChannel1Val = 0;
    globalState.pwmChannel2Val = 0;
    globalState.pwmChannel3Val = 0;
    globalState.sendPwmChannel1Val = 1;
    globalState.sendPwmChannel2Val = 1;
    globalState.sendPwmChannel3Val = 1;
    globalState.sendRawPosition = 1;
	globalState.desiredSpeed = 0;
	globalState.runningEnabled = 0;
	globalState.adcDataReady = 0;
    globalState.currentAscaler = 0xF0B7; // 0.9403 *2^16
    globalState.currentBscaler = 0xD3CA; //0.8273
    globalState.currentCscaler = 0xFFFF;
    globalState.currentAfiltPrev = 0;
    globalState.currentBfiltPrev = 0;
    globalState.currentCfiltPrev = 0;
    globalState.currentAoffset = 1896; // 1880
    globalState.currentBoffset = 1896;
    globalState.currentCoffset = 1896;

}

void initControl()
{
    globalStateInit();
	bspStart();
    Uv_ampl = 0.3;
    cnt = 0;
    messageLength = 0;

	fp_Vamp = 0x8fff ; // q16 amplitude 0xDDB2 max
	fp_Uv_ang = 0;

}

void updateControl()
{


	/* Read encoder */
	// 4096/ 11 = 372.37
	// 97 - first zero angle

	if(getPositionData(rawPosData))
	{
		globalState.rawPosition = (uint16_t)(rawPosData[0] << 8) + rawPosData[1];

		if(globalState.rawPosition < 97)
		{
			raw_angle = 4095 - (96 - globalState.rawPosition);
		}
		else
			raw_angle = globalState.rawPosition - 97;

		for(ind = 0; ind < 10; ind++)
		{
			raw_angle -= 372;
			if(raw_angle < 0)
			{
				raw_angle += 186;
				break;
			}
		}
        globalState.electricalAngle = (raw_angle * 0xB02C0B); // (2^31) / 186  = 11 545 611; in q31 = 0xB02C0B
	}

	if(cnt == 400 && globalState.sendDataEnabled)
	{
		// globalState.rawCurrentA = globalState.pwmChannel1Val;
		// globalState.rawCurrentB = globalState.pwmChannel2Val;
		// globalState.rawCurrentC = globalState.pwmChannel3Val;

		composeRegularMessage(messageString, &messageLength);
		sendData(messageString, messageLength);
		cnt = 0;
	} else if(cnt > 1000)
	{
		cnt = 0;
	}
	cnt ++;

	if(globalState.dataReady)
	{
		globalState.dataReady = 0;

		if(globalState.receivedData[rcvMsgInd++] == 0x49)
		{
			controlByte = globalState.receivedData[rcvMsgInd++];
			dataBytes = globalState.receivedData[rcvMsgInd] * 256 + globalState.receivedData[rcvMsgInd+1];
			if(controlByte >> 7) // write
			{
				switch (controlByte & 0x7f)
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

				rcvMsgInd = 0;
			} else // read
			{
				rcvMsgInd = 0;
			}
		}
	}
}

void updateCalc(void)
{
	if(adcInterruptCnt == 1)
	{
		adcInterruptCnt = 0;
		return;
	}
	else
		adcInterruptCnt++;
	/*
	97 - 452 - 834 - 1232 - 1615 - 1971 - 2320 - 2687 - 3078 - 3474 - 3843 - 97
	  355   382   398    383    356    349    367    391    396    369    349
	*/
    val_after = TIM2->CNT;

    globalState.rawCurrentA = globalState.rawCurrent[0];
    globalState.rawCurrentB = globalState.rawCurrent[3];
    globalState.rawCurrentC = globalState.rawCurrent[1];
    /* filter currents */
    globalState.currentAfilt = ((globalState.currentAfiltPrev * 0xE000) >> 16) + (globalState.rawCurrentA >> 3); // 0.875*prev + 0.125*curr | 0.75 = 0xD999
    globalState.currentBfilt = ((globalState.currentBfiltPrev * 0xE000) >> 16) + (globalState.rawCurrentB >> 3); // 0.875*prev + 0.125*curr | 0.75 = 0xD999
    globalState.currentCfilt = ((globalState.currentCfiltPrev * 0xE000) >> 16) + (globalState.rawCurrentC >> 3); // 0.875*prev + 0.125*curr | 0.75 = 0xD999

	globalState.currentAfiltPrev = globalState.currentAfilt;
    globalState.currentBfiltPrev = globalState.currentBfilt;
    globalState.currentCfiltPrev = globalState.currentCfilt;

    /* scale A, B, C currents and convert to int32_t type */

    globalState.currentAscaled = (((globalState.currentAfilt - globalState.currentAoffset) * globalState.currentAscaler)); //
    globalState.currentBscaled = (((globalState.currentBfilt - globalState.currentBoffset) * globalState.currentBscaler)); //
    globalState.currentCscaled = (((globalState.currentCfilt - globalState.currentCoffset) * globalState.currentCscaler)); //

    /* Clark */
    // globalState.alfaI = globalState.currentAscaled;
    // globalState.betaI = ((globalState.currentBscaled - globalState.currentCscaled) * SQRT3_M1) >> 16;

    arm_clarke_q31(globalState.currentAscaled, globalState.currentBscaled, &globalState.alfaI, &globalState.betaI);

    /* Park */

    // calculate sin and cos
    cordicSetSin32();
    LL_CORDIC_WriteData(CORDIC, globalState.electricalAngle);

    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

    globalState.thetaSin = LL_CORDIC_ReadData(CORDIC);
    globalState.thetaCos = LL_CORDIC_ReadData(CORDIC);

    // globalState.currQ = __SSAT(((globalState.alfaI * globalState.thetaCos) >> 15) - ((globalState.betaI * globalState.thetaSin) >> 15), 15);
    // globalState.currD = __SSAT(((globalState.alfaI * globalState.thetaSin) >> 15) + ((globalState.betaI * globalState.thetaCos) >> 15), 15);

    // calc Park transform
    arm_park_q31(globalState.alfaI, globalState.betaI, &globalState.currQ, &globalState.currD, globalState.thetaSin, globalState.thetaCos);

    /* Calculate errors */
    globalState.errorD = -globalState.currD;
    globalState.errorQ = __SSAT((globalState.desiredCurrQ - globalState.currQ), 32);

    /* PID D */
    globalState.currDres = arm_pid_q31(&globalState.PIDD, globalState.errorD);

    /* PID Q */
    globalState.currQres = arm_pid_q31(&globalState.PIDQ, globalState.errorQ);

    /* Inv Park */
    arm_inv_park_q31(globalState.currQres, globalState.currDres, &globalState.alfaV, &globalState.betaV, globalState.thetaSin, globalState.thetaCos);

    cordicSetPhase32();
    LL_CORDIC_WriteData(CORDIC, globalState.alfaV);
    LL_CORDIC_WriteData(CORDIC, globalState.betaV);

    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

    globalState.abPhase = LL_CORDIC_ReadData(CORDIC);
    globalState.abAmpl  = LL_CORDIC_ReadData(CORDIC);

    /* Alfa, Beta -> Phase , Angle */
    fp_Uv_ang   = (uint16_t)(((int16_t)(globalState.abPhase >> 16)) + 0x7FFF);  // changing int32_t representation to uint16_t representation
    fp_Vamp     = (uint16_t)(globalState.abAmpl >> 15);

    if(fp_Vamp > 0xDDB2)
        fp_Vamp = 0xDDB2;

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

	/*
	11 pole pairs
	32.727272 degree per electrical rotation
	mechanical 0.0909 degree per one electrical degree
	1 digit = 0.087890625 degree
	*/
	/* SVPWM part */

    fp_cordic_inp = (0x7fff << 16) + fp_betaAng;

	cordicSetSin();

    LL_CORDIC_WriteData(CORDIC, fp_cordic_inp);

    fp_cordic_inp = (0x7fff << 16) + (fp_sixtyDeg - fp_betaAng);

    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

    sinB = (uint16_t)LL_CORDIC_ReadData(CORDIC);

    LL_CORDIC_WriteData(CORDIC, fp_cordic_inp);

    fp_firstPart = (fp_Vamp*fp_tConst) >> 16; // q2.16

    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

    sin60mB = (uint16_t)LL_CORDIC_ReadData(CORDIC);

    fp_t_a = (PWM_PERIOD*((sin60mB * fp_firstPart)>> 15)) >> 16 ;
    fp_t_b = (PWM_PERIOD*((sinB * fp_firstPart)>> 15)) >> 16 ;
    fp_t_zero = PWM_PERIOD - fp_t_a - fp_t_b;

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
        break;
    case 2:
        globalState.pwmChannel1Val = t3;
        globalState.pwmChannel2Val = t1;
        globalState.pwmChannel3Val = t4;
        break;
    case 3:
        globalState.pwmChannel1Val = t4;
        globalState.pwmChannel2Val = t1;
        globalState.pwmChannel3Val = t2;
        break;
    case 4:
        globalState.pwmChannel1Val = t4;
        globalState.pwmChannel2Val = t3;
        globalState.pwmChannel3Val = t1;
        break;
    case 5:
        globalState.pwmChannel1Val = t2;
        globalState.pwmChannel2Val = t4;
        globalState.pwmChannel3Val = t1;
        break;
    case 6:
        globalState.pwmChannel1Val = t1;
        globalState.pwmChannel2Val = t4;
        globalState.pwmChannel3Val = t3;
        break;

    default:
        globalState.pwmChannel1Val = t1;
        globalState.pwmChannel2Val = t2;
        globalState.pwmChannel3Val = t4;
        break;
    }

/* SVPWM part END */

    /* Set timer pulse */
    if(globalState.runningEnabled)
    {
        setPWM1(globalState.pwmChannel1Val);
        setPWM2(globalState.pwmChannel2Val);
        setPWM3(globalState.pwmChannel3Val);

        fp_Uv_ang += globalState.desiredSpeed; // 1 degree 182 digits (360 degree - 2pi rad - 0xffff)

        if(fp_Uv_ang >= 0xFFFF)
            fp_Uv_ang = 0;
    }
    else
    {
        setPWM1(0);
        setPWM2(0);
        setPWM3(0);
    }

	val_before = TIM2->CNT;

	val_after = val_before - val_after;

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
        data[index++] = (uint8_t)(globalState.currentAscaled >> 8);
        data[index++] = (uint8_t)globalState.currentAscaled;
    }

    if(globalState.sendPwmChannel2Val)
    {
//        data[index++] = (uint8_t)3;
//        data[index++] = (uint8_t)2;
        data[index++] = (uint8_t)(globalState.currentBscaled >> 8);
        data[index++] = (uint8_t)globalState.currentBscaled;
    }

    if(globalState.sendPwmChannel3Val)
    {
//        data[index++] = (uint8_t)4;
//        data[index++] = (uint8_t)2;
        data[index++] = (uint8_t)(globalState.currentCscaled >> 8);
        data[index++] = (uint8_t)globalState.currentCscaled;
    }
    *size = index;

}

