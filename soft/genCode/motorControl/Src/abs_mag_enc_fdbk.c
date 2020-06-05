
#include "stm32g4xx_hal.h"
#include "abs_mag_enc_fdbk.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
extern ABS_ENCODER_Handle_t ABS_ENCODER_M1;

void ABS_ENC_Init( ABS_ENCODER_Handle_t * pHandle )
{
    TIM_TypeDef * TIMx = pHandle->TIMx;
    uint8_t BufferSize;
    uint8_t Index;

    /* Reset counter */
    LL_TIM_SetCounter ( TIMx, 0 );
    LL_TIM_EnableUpdateEvent(TIMx);
    LL_TIM_SetAutoReload(TIMx, pHandle->pollPeriodTicks);
    LL_TIM_SetPrescaler(TIMx, pHandle->timPrescaler);

    /*Calculations of convenience*/
//    pHandle->U32MAXdivPulseNumber = UINT32_MAX / ( uint32_t )( pHandle->PulseNumber );
//    pHandle->SpeedSamplingFreqUnit = pHandle->SpeedSamplingFreqHz * SPEED_UNIT;

    LL_TIM_ClearFlag_UPDATE ( TIMx );
    LL_TIM_EnableIT_UPDATE ( TIMx );

    /* Enable the counting timer*/
    LL_TIM_EnableCounter ( TIMx );

}

bool ABS_ENC_CalcAvrgMecSpeedUnit( ABS_ENCODER_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
    uint16_t newPos;
    int16_t digitDiff;
    float digitSpeed;

    newPos = pHandle->rawPosition;
    digitDiff = newPos - pHandle->prevRawPosition;
    pHandle->prevRawPosition = newPos;

    *pMecSpeedUnit = pHandle->SpeedSamplingFreqUnit;
    digitSpeed = ((float)digitDiff / 4096.0) * pHandle->SpeedSamplingFreqHz * SPEED_UNIT;
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)round(digitSpeed);

}


/*****************************************************************
 *              CALLBACKS
 * **************************************************************/

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    ABS_ENCODER_M1.rawPosition = (uint16_t)(ABS_ENCODER_M1.rawData[0] << 8) + ABS_ENCODER_M1.rawData[1];
    int16_t raw_angle;
    int ind;

    if(ABS_ENCODER_M1.rawPosition < ABS_ENCODER_M1.posOffset)
    {
        raw_angle = 4095 - (ABS_ENCODER_M1.posOffset - 1 - ABS_ENCODER_M1.rawPosition);
    }
    else
        raw_angle = ABS_ENCODER_M1.rawPosition - ABS_ENCODER_M1.posOffset;

    for(ind = 0; ind < 11; ind++)
    {
        raw_angle -= 372;
        if(raw_angle < 0)
        {
            raw_angle += 186;
            break;
        }
    }
    ABS_ENCODER_M1._Super.hElAngle = (raw_angle * 0xB0); // (2^15) / 186  = 176; in q16 = 0xB0
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // add handler when there is no encoder detected
    if(HAL_I2C_Mem_Read_DMA(&hi2c1, 0x36<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, ABS_ENCODER_M1.rawData, 2) == HAL_OK)
    {
        return;
    } else
    {
        return;
    }
}
