#ifndef __ABS_ENCODER_SPEEDNPOSFDBK_H
#define __ABS_ENCODER_SPEEDNPOSFDBK_H

#include "speed_pos_fdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;

  TIM_TypeDef * TIMx;   /*!< Timer used for ENCODER sensor management.*/

  uint32_t SpeedSamplingFreqUnit; /*!< Frequency at which motor speed is to be
                                   computed. It must be equal to the frequency
                                   at which function SPD_CalcAvrgMecSpeedUnit
                                   is called.*/
//  int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /*!< Buffer used to store
//                                        captured variations of timer counter*/


  uint32_t U32MAXdivPulseNumber;       /*! <It stores U32MAX/hPulseNumber*/

  uint16_t SpeedSamplingFreqHz;        /*! <Frequency (Hz) at which motor speed
                                        is to be computed. */

  /* SW Settings */
  uint16_t PulseNumber; /*!< Number of pulses per revolution, provided by each
                              of the two encoder signals, multiplied by 4 */

  volatile uint16_t TimerOverflowNb;    /*!< Number of overflows occurred since
                                        last speed measurement event*/
  uint16_t PreviousCapture;            /*!< Timer counter value captured during
                                        previous speed measurement event*/

  FunctionalState RevertSignal; /*!< To be enabled if measured speed is opposite
                                     to real one (ENABLE/DISABLE)*/

  uint8_t SpeedBufferSize; /*!< Size of the buffer used to calculate the average
                             speed. It must be <= 16.*/


  bool SensorIsReliable;            /*!< Flag to indicate sensor/decoding is not
                                         properly working.*/

  uint8_t ICx_Filter;                  /*!< Input Capture filter selection */

  volatile uint8_t DeltaCapturesIndex; /*! <Buffer index*/

  bool TimerOverflowError;              /*!< true if the number of overflow
                                        occurred is greater than 'define'
                                        ENC_MAX_OVERFLOW_NB*/

    uint8_t rawData[2];
    uint16_t rawPosition;
    uint16_t prevRawPosition;
    uint16_t posOffset;
    int16_t mecAngle;
    int16_t prevMecAngle;
    uint16_t pollPeriodTicks;
    uint16_t timPrescaler;
    uint16_t prevSpeed;

} ABS_ENCODER_Handle_t;

void ABS_ENC_Init( ABS_ENCODER_Handle_t * pHandle );
void ABS_ENC_Clear( ABS_ENCODER_Handle_t * pHandle );
bool ABS_ENC_CalcAvrgMecSpeedUnit( ABS_ENCODER_Handle_t * pHandle, int16_t * pMecSpeedUnit );
// void ABS_ENC_SetMecAngle( ABS_ENCODER_Handle_t * pHandle, int16_t hMecAngle );

#endif
