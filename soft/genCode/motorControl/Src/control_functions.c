#include "control_functions.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>

// #include "parameters_conversion.h"

#define MAX_MOD_VECTOR_VAL  (uint16_t) 0xDDB2
#define SQRT3_M1        (uint16_t)0x93CD // 16 bit  1/sqrt(3) = 0,5773502691896 * 2^16

extern I2C_HandleTypeDef hi2c1;

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
    // globalState.pwmChannel1Val = 0;
    // globalState.pwmChannel2Val = 0;
    // globalState.pwmChannel3Val = 0;
    // globalState.sendCurrentA = 0;
    // globalState.sendCurrentB = 0;
    // globalState.sendCurrentC = 0;
    // globalState.sendAlfaI = 0;
    // globalState.sendBetaI = 0;
    // globalState.sendCurrQ = 0;
    // globalState.sendCurrD = 0;
    // globalState.sendErrorQ = 0;
    // globalState.sendErrorD = 0;
    // globalState.sendDesiredCurrQ = 0;
    // globalState.sendCurrQres = 0;
    // globalState.sendCurrDres = 0;
    // globalState.sendAlfaV = 0;
    // globalState.sendBetaV = 0;
    // globalState.sendAbPhase = 0;
    // globalState.sendAbAmpl = 0;
	// globalState.desiredSpeed = 0;
	// globalState.runningEnabled = 0;
	// globalState.adcDataReady = 0;
    // globalState.currentAscaler = 0xF0B7; // 0.9403 *2^16
    // globalState.currentBscaler = 0xD3CA; //0.8273
    // globalState.currentCscaler = 0xFFFF;
    // globalState.currentAfiltPrev = 0;
    // globalState.currentBfiltPrev = 0;
    // globalState.currentCfiltPrev = 0;
    // globalState.currentAoffset = 1896; // 1880
    // globalState.currentBoffset = 1896;
    // globalState.currentCoffset = 1906;
    // globalState.testModeEn = 0;
    // globalState.testAmpl = 0x1fff;

    globalState.posOffset = 282;
    // globalState.sendRawPosition = 0;

    // PID_HandleInit(&globalState.PID_D);
    // PID_HandleInit(&globalState.PID_Q);
}

/*
void initControl()
{
    globalStateInit();
	bspStart();
    Uv_ampl = 0.3;
    cnt = 0;
    messageLength = 0;

	fp_Vamp = 0x4fff ; // q16 amplitude 0xDDB2 max
	fp_Uv_ang = 0;

}
*/
int getPositionData(uint8_t * pos)
{
    if(HAL_I2C_Mem_Read_DMA(&hi2c1, 0x36<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, (pos), 2) == HAL_OK)
    {
        return 1;
    } else
    {
        return 0;
    }
}

void updateControl()
{


	/* Read encoder */
	// 4096/ 11 = 372.37
	// globalState.posOffset - first zero angle

	if(getPositionData(rawPosData))
	{
		globalState.rawPosition = (uint16_t)(rawPosData[0] << 8) + rawPosData[1];

		if(globalState.rawPosition < globalState.posOffset)
		{
			raw_angle = 4095 - (globalState.posOffset - 1 - globalState.rawPosition);
		}
		else
			raw_angle = globalState.rawPosition - globalState.posOffset;

		for(ind = 0; ind < 11; ind++)
		{
			raw_angle -= 372;
			if(raw_angle < 0)
			{
				raw_angle += 186;
				break;
			}
		}
        globalState.electricalAngle = (raw_angle * 0xB0); // (2^15) / 186  = 176; in q16 = 0xB0
	}

	// if(cnt == 400 && globalState.sendDataEnabled)
	// {
	// 	// globalState.rawCurrentA = globalState.pwmChannel1Val;
	// 	// globalState.rawCurrentB = globalState.pwmChannel2Val;
	// 	// globalState.rawCurrentC = globalState.pwmChannel3Val;

	// 	composeRegularMessage(messageString, &messageLength);
	// 	if(messageLength > 3)
	// 		sendData(messageString, messageLength);
	// 	cnt = 0;
	// } else if(cnt > 1000)
	// {
	// 	cnt = 0;
	// }
	// cnt ++;

	// if(globalState.dataReady)
	// {
	// 	globalState.dataReady = 0;

	// 	if(globalState.receivedData[rcvMsgInd++] == 0x49)
	// 	{
	// 		controlByte = globalState.receivedData[rcvMsgInd++];
	// 		dataBytes = globalState.receivedData[rcvMsgInd] * 256 + globalState.receivedData[rcvMsgInd+1];
	// 		if(controlByte >> 7) // write
	// 		{
	// 			switch (controlByte & 0x7f)
	// 			{
	// 			case 0x00:
	// 				globalState.runningEnabled = dataBytes;
    //                 if(globalState.testModeEn)
    //                     fp_Uv_ang = 0;
	// 				break;
	// 			case 0x01:
	// 				globalState.desiredSpeed = dataBytes;
	// 				break;
	// 			case 0x02:
	// 				globalState.sendDataEnabled = dataBytes;
	// 				break;
    //             case 0x03:
	// 				HAL_NVIC_SystemReset();
	// 				break;
    //             case 0x04:
    //                 globalState.PIDD.Kd = dataBytes << 6;
    //                 arm_pid_init_q31(&globalState.PIDD, 0);
    //                 break;
    //             case 0x05:
    //                 globalState.PIDD.Ki = dataBytes << 6;
    //                 arm_pid_init_q31(&globalState.PIDD, 0);
    //                 break;
    //             case 0x06:
    //                 globalState.PIDD.Kp = dataBytes << 5;
    //                 arm_pid_init_q31(&globalState.PIDD, 0);
    //                 break;
    //             case 0x07:
    //                 globalState.PIDQ.Kd = dataBytes << 6;
    //                 arm_pid_init_q31(&globalState.PIDQ, 0);
    //                 break;
    //             case 0x08:
    //                 globalState.PIDQ.Ki = dataBytes << 6;
    //                 arm_pid_init_q31(&globalState.PIDQ, 0);
    //                 break;
    //             case 0x09:
    //                 globalState.PIDQ.Kp = dataBytes << 6;
    //                 arm_pid_init_q31(&globalState.PIDQ, 0);
    //                 break;
    //             case 0x0A:
    //                 globalState.desiredCurrQ = (int32_t)(dataBytes << 16);
    //                 break;
    //             case 0x0B:
    //                 globalState.sendCurrentA = dataBytes;
    //                 break;
    //             case 0x0C:
    //                 globalState.sendCurrentB = dataBytes;
    //                 break;
    //             case 0x0D:
    //                 globalState.sendCurrentC = dataBytes;
    //                 break;
    //             case 0x0E:
    //                 globalState.sendRawPosition = dataBytes;
    //                 break;
    //             case 0x0F:
    //                 globalState.sendAlfaI = dataBytes;
    //                 break;
    //             case 0x10:
    //                 globalState.sendBetaI = dataBytes;
    //                 break;
    //             case 0x11:
    //                 globalState.sendCurrQ = dataBytes;
    //                 break;
    //             case 0x12:
    //                 globalState.sendCurrD = dataBytes;
    //                 break;
    //             case 0x13:
    //                 globalState.sendErrorQ = dataBytes;
    //                 break;
    //             case 0x14:
    //                 globalState.sendErrorD = dataBytes;
    //                 break;
    //             case 0x15:
    //                 globalState.sendDesiredCurrQ = dataBytes;
    //                 break;
    //             case 0x16:
    //                 globalState.sendCurrQres = dataBytes;
    //                 break;
    //             case 0x17:
    //                 globalState.sendCurrDres = dataBytes;
    //                 break;
    //             case 0x18:
    //                 globalState.sendAlfaV = dataBytes;
    //                 break;
    //             case 0x19:
    //                 globalState.sendBetaV = dataBytes;
    //                 break;
    //             case 0x1A:
    //                 globalState.sendAbAmpl = dataBytes;
    //                 break;
    //             case 0x1B:
    //                 globalState.sendAbPhase = dataBytes;
    //                 break;
    //             case 0x1C:
    //                 globalState.testModeEn = dataBytes;
    //                 break;
    //             case 0x1D:
    //                 globalState.posOffset = dataBytes;
    //                 break;
    //             case 0x1E:
    //                 globalState.testAmpl = dataBytes;
    //                 break;

	// 			default:
	// 				break;
	// 			}

	// 			rcvMsgInd = 0;
	// 		} else // read
	// 		{
	// 			rcvMsgInd = 0;
	// 		}
	// 	}
	// }
}


// void updateCalc(void)
// {
// 	if(adcInterruptCnt == 1)
// 	{
// 		adcInterruptCnt = 0;
// 		return;
// 	}
// 	else
// 		adcInterruptCnt++;
// 	/*
// 	97 - 452 - 834 - 1232 - 1615 - 1971 - 2320 - 2687 - 3078 - 3474 - 3843 - 97
// 	  355   382   398    383    356    349    367    391    396    369    349
// 	*/
//     val_after = TIM2->CNT;

//     globalState.rawCurrentA = globalState.rawCurrent[0];
//     globalState.rawCurrentB = globalState.rawCurrent[3];
//     globalState.rawCurrentC = globalState.rawCurrent[1];
//     /* filter currents */
//     globalState.currentAfilt = ((globalState.currentAfiltPrev * 0xE000) >> 16) + (globalState.rawCurrentA >> 3); // 0.875*prev + 0.125*curr | 0.75 = 0xD999
//     globalState.currentBfilt = ((globalState.currentBfiltPrev * 0xE000) >> 16) + (globalState.rawCurrentB >> 3); // 0.875*prev + 0.125*curr | 0.75 = 0xD999
//     globalState.currentCfilt = ((globalState.currentCfiltPrev * 0xE000) >> 16) + (globalState.rawCurrentC >> 3); // 0.875*prev + 0.125*curr | 0.75 = 0xD999

// 	globalState.currentAfiltPrev = globalState.currentAfilt;
//     globalState.currentBfiltPrev = globalState.currentBfilt;
//     globalState.currentCfiltPrev = globalState.currentCfilt;

//     /* scale A, B, C currents and convert to int32_t type */

//     globalState.currentAscaled = (((globalState.currentAfilt - globalState.currentAoffset) * globalState.currentAscaler)); //
//     globalState.currentBscaled = (((globalState.currentBfilt - globalState.currentBoffset) * globalState.currentBscaler)); //
//     globalState.currentCscaled = (((globalState.currentCfilt - globalState.currentCoffset) * globalState.currentCscaler)); //

//     /* Clark */
//     // globalState.alfaI = globalState.currentAscaled;
//     // globalState.betaI = ((globalState.currentBscaled - globalState.currentCscaled) * SQRT3_M1) >> 16;

//     arm_clarke_q31(globalState.currentAscaled, globalState.currentBscaled, &globalState.alfaI, &globalState.betaI);

//     /* Park */

//     // calculate sin and cos
//     cordicSetSin32();
//     LL_CORDIC_WriteData(CORDIC, globalState.electricalAngle);
// 	LL_CORDIC_WriteData(CORDIC, (int32_t)((1<<31)-1));

// //    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

//     globalState.thetaSin = LL_CORDIC_ReadData(CORDIC);
//     globalState.thetaCos = LL_CORDIC_ReadData(CORDIC);

//     // globalState.currQ = __SSAT(((globalState.alfaI * globalState.thetaCos) >> 15) - ((globalState.betaI * globalState.thetaSin) >> 15), 15);
//     // globalState.currD = __SSAT(((globalState.alfaI * globalState.thetaSin) >> 15) + ((globalState.betaI * globalState.thetaCos) >> 15), 15);

//     // calc Park transform
//     arm_park_q31(globalState.alfaI, globalState.betaI, &globalState.currQ, &globalState.currD, globalState.thetaSin, globalState.thetaCos);

//     /* Calculate errors */
//     globalState.errorD = -globalState.currD;
//     globalState.errorQ = __SSAT((globalState.desiredCurrQ - globalState.currQ), 32);

//     /* PID D */
//     globalState.currDres = arm_pid_q31(&globalState.PIDD, globalState.errorD);

//     /* PID Q */
//     globalState.currQres = arm_pid_q31(&globalState.PIDQ, globalState.errorQ);

//     /* Inv Park */
//     arm_inv_park_q31(globalState.currQres, globalState.currDres, &globalState.alfaV, &globalState.betaV, globalState.thetaSin, globalState.thetaCos);

//     cordicSetPhase32();
//     LL_CORDIC_WriteData(CORDIC, globalState.betaV);
//     LL_CORDIC_WriteData(CORDIC, globalState.alfaV);

// //    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

//     globalState.abPhase = LL_CORDIC_ReadData(CORDIC);
//     globalState.abAmpl  = LL_CORDIC_ReadData(CORDIC);

//     FOCVars.UserIdref = 0;
//     IqdRef.q = globalState.desiredCurrQ;
//     IqdRef.d = FOCVars.UserIdref;
//     FOCVars.Iqdref = IqdRef;

//     FOC_CurrController();

//     /* Alfa, Beta -> Phase , Angle */
//     if(!globalState.testModeEn)
//     {
//         fp_Uv_ang   = (uint16_t)(((int16_t)(globalState.abPhase >> 16)) + 0x7FFF);  // changing int32_t representation to uint16_t representation
//         fp_Vamp     = (uint16_t)__SSAT((((uint64_t)globalState.abAmpl * 0xDDB2) >> 19), 16);
//     }
//     else
//     {
//         fp_Vamp = globalState.testAmpl;
//     }

//     if(fp_Uv_ang < fp_sixtyDeg)
//     {
//         sector_number = 1;
//         fp_betaAng = fp_Uv_ang;
//     } else if(fp_Uv_ang > fp_sixtyDeg && fp_Uv_ang < 2*fp_sixtyDeg)
//     {
//         sector_number = 2;
//         fp_betaAng = fp_Uv_ang - fp_sixtyDeg;
//     } else if(fp_Uv_ang > 2*fp_sixtyDeg && fp_Uv_ang < 3*fp_sixtyDeg)
//     {
//         sector_number = 3;
//         fp_betaAng = fp_Uv_ang - 2*fp_sixtyDeg;
//     } else if(fp_Uv_ang > 3*fp_sixtyDeg && fp_Uv_ang < 4*fp_sixtyDeg)
//     {
//         sector_number = 4;
//         fp_betaAng = fp_Uv_ang - 3*fp_sixtyDeg;
//     } else if(fp_Uv_ang > 4*fp_sixtyDeg && fp_Uv_ang < 5*fp_sixtyDeg)
//     {
//         sector_number = 5;
//         fp_betaAng = fp_Uv_ang - 4*fp_sixtyDeg;
//     } else if(fp_Uv_ang > 5*fp_sixtyDeg)
//     {
//         sector_number = 6;
//         fp_betaAng = fp_Uv_ang - 5*fp_sixtyDeg;
//     }

// 	/*
// 	11 pole pairs
// 	32.727272 degree per electrical rotation
// 	mechanical 0.0909 degree per one electrical degree
// 	1 digit = 0.087890625 degree
// 	*/
// 	/* SVPWM part */

//     fp_cordic_inp = (0x7fff << 16) + fp_betaAng;

// 	cordicSetSin();

//     LL_CORDIC_WriteData(CORDIC, fp_cordic_inp);

// //    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

//     sinB = (uint16_t)LL_CORDIC_ReadData(CORDIC);

// 	fp_cordic_inp = (0x7fff << 16) + (fp_sixtyDeg - fp_betaAng);

//     LL_CORDIC_WriteData(CORDIC, fp_cordic_inp);

// //    while(!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)){}

// 	fp_firstPart = (fp_Vamp*fp_tConst) >> 16; // q2.16

//     sin60mB = (uint16_t)LL_CORDIC_ReadData(CORDIC);

//     fp_t_a = (PWM_PERIOD*((sin60mB * fp_firstPart)>> 15)) >> 16 ;
//     fp_t_b = (PWM_PERIOD*((sinB * fp_firstPart)>> 15)) >> 16 ;
//     fp_t_zero = PWM_PERIOD - fp_t_a - fp_t_b;

//     t1 = (uint16_t) (fp_t_a + fp_t_b + (fp_t_zero >> 1));
//     t2 = (uint16_t) (fp_t_b + (fp_t_zero >> 1));
//     t3 = (uint16_t) (fp_t_a + (fp_t_zero >> 1));
//     t4 = (uint16_t) (fp_t_zero >> 1);

//     switch (sector_number)
//     {
//     case 1:
//         globalState.pwmChannel1Val = t1;
//         globalState.pwmChannel2Val = t2;
//         globalState.pwmChannel3Val = t4;
//         break;
//     case 2:
//         globalState.pwmChannel1Val = t3;
//         globalState.pwmChannel2Val = t1;
//         globalState.pwmChannel3Val = t4;
//         break;
//     case 3:
//         globalState.pwmChannel1Val = t4;
//         globalState.pwmChannel2Val = t1;
//         globalState.pwmChannel3Val = t2;
//         break;
//     case 4:
//         globalState.pwmChannel1Val = t4;
//         globalState.pwmChannel2Val = t3;
//         globalState.pwmChannel3Val = t1;
//         break;
//     case 5:
//         globalState.pwmChannel1Val = t2;
//         globalState.pwmChannel2Val = t4;
//         globalState.pwmChannel3Val = t1;
//         break;
//     case 6:
//         globalState.pwmChannel1Val = t1;
//         globalState.pwmChannel2Val = t4;
//         globalState.pwmChannel3Val = t3;
//         break;

//     default:
//         globalState.pwmChannel1Val = t1;
//         globalState.pwmChannel2Val = t2;
//         globalState.pwmChannel3Val = t4;
//         break;
//     }

// /* SVPWM part END */

//     /* Set timer pulse */
//     if(globalState.runningEnabled)
//     {
//         setPWM1(globalState.pwmChannel1Val);
//         setPWM2(globalState.pwmChannel2Val);
//         setPWM3(globalState.pwmChannel3Val);

//         if(globalState.testModeEn)
//         {
//             fp_Uv_ang += globalState.desiredSpeed; // 1 degree 182 digits (360 degree - 2pi rad - 0xffff)

//             if(fp_Uv_ang >= 0xFFFF)
//                fp_Uv_ang = 0;
//         }
//     }
//     else
//     {
//         setPWM1(0);
//         setPWM2(0);
//         setPWM3(0);
//     }

// 	val_before = TIM2->CNT;

// 	val_after = val_before - val_after;

// }

// void composeRegularMessage(uint8_t * data, uint16_t * size)
// {
//     uint8_t index = 0;
//     int16_t curr16bit;

//     if(globalState.sendCurrentA)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x0B;
//         curr16bit = globalState.currentAscaled >> 16; // globalState.pwmChannel1Val;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendCurrentB)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x0C;
//         curr16bit = globalState.currentBscaled >> 16; // globalState.pwmChannel2Val;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendCurrentC)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x0D;
//         curr16bit = globalState.currentCscaled >> 16; // globalState.pwmChannel3Val;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendRawPosition)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x0E;
//         data[index++] = (uint8_t)(globalState.rawPosition >> 8);
//         data[index++] = (uint8_t)globalState.rawPosition;
//     }

//     if(globalState.sendAlfaI)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x0F;
//         curr16bit = globalState.alfaI >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendBetaI)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x10;
//         curr16bit = globalState.betaI >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendCurrQ)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x11;
//         curr16bit = globalState.currQ >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendCurrD)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x12;
//         curr16bit = globalState.currD >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendErrorQ)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x13;
//         curr16bit = globalState.errorQ >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendErrorD)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x14;
//         curr16bit = globalState.errorD >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendDesiredCurrQ)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x15;
//         curr16bit = globalState.desiredCurrQ >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendCurrQres)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x16;
//         curr16bit = globalState.currQres >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendCurrDres)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x17;
//         curr16bit = globalState.currDres >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendAlfaV)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x18;
//         curr16bit = globalState.alfaV >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendBetaV)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x19;
//         curr16bit = globalState.betaV >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendAbAmpl)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x1A;
//         curr16bit = (uint16_t)(globalState.abAmpl >> 16); //fp_Vamp;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//     if(globalState.sendAbPhase)
//     {
//         data[index++] = (uint8_t)0x48;
//         data[index++] = 0x1B;
//         curr16bit = globalState.abPhase >> 16;
//         data[index++] = (uint8_t)(curr16bit >> 8);
//         data[index++] = (uint8_t)curr16bit;
//     }

//    if(globalState.sendPwmChannel1Val)
//    {
//        data[index++] = (uint8_t)(globalState.pwmChannel1Val >> 8);
//        data[index++] = (uint8_t)globalState.pwmChannel1Val;
//    }

//    if(globalState.sendPwmChannel2Val)
//    {
//        data[index++] = (uint8_t)(globalState.pwmChannel2Val >> 8);
//        data[index++] = (uint8_t)globalState.pwmChannel2Val;
//    }

//    if(globalState.sendPwmChannel3Val)
//    {
//        data[index++] = (uint8_t)(globalState.pwmChannel3Val >> 8);
//        data[index++] = (uint8_t)globalState.pwmChannel3Val;
//    }
//     *size = index;

// }

