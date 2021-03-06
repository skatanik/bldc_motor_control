#include "bsp_functions.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;

extern DAC_HandleTypeDef hdac1;

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint16_t arm_A_raw_current;
uint16_t arm_B_raw_current;
uint16_t arm_C_raw_current;

extern globalState_typedef globalState;

void bspStart(void)
{
	HAL_UART_Receive_DMA(&huart1, globalState.receivedData, 4);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);
	HAL_COMP_Start(&hcomp4);
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

//	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_TIM_Base_Start(&htim2);

	HAL_Delay(10);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)globalState.rawCurrent, 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&globalState.rawCurrent[3], 1);

	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT);

//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_PERIOD-30);
}

void setPWM1(uint16_t val)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val);
}

void setPWM2(uint16_t val)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, val);
}

void setPWM3(uint16_t val)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, val);
}

void sendUARTArray(uint8_t * data, uint16_t size)
{
    HAL_UART_Transmit_DMA(&huart1, data, size);
}

void sendData(uint8_t * data, uint16_t size)
{
#ifdef MAIN_IFACE_UART
    sendUARTArray(data, size);
#else

#endif
}

void startDataReceiving(uint8_t * data, uint16_t size)
{
    HAL_UART_Receive_DMA(&huart1, data, size);
}

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

/**
 *  CALLBACKS
 */

//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//	if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
//    {
//        globalState.rawCurrentA = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//		globalState.rawCurrentC = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
//
////		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
//    }
//	if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
//    {
//       globalState.rawCurrentB = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//    }
//}HAL_ADC_ConvCpltCallback

void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	globalState.dataReady = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	updateCalc();
}

void cordicSetSin(void)
{
	LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_SINE,   /* cosine function */
                           LL_CORDIC_PRECISION_6CYCLES, /* max precision for q1.31 cosine */
                           LL_CORDIC_SCALE_0,           /* no scale */
                           LL_CORDIC_NBWRITE_1,         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                           LL_CORDIC_NBREAD_1,          /* Two output data: cosine, then sine */
                           LL_CORDIC_INSIZE_16BITS,     /* q1.16 format for input data */
                           LL_CORDIC_OUTSIZE_16BITS);   /* q1.16 format for output data */
}

void cordicSetSin32(void)
{
	LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_SINE,   /* cosine function */
                           LL_CORDIC_PRECISION_6CYCLES, /* max precision for q1.31 cosine */
                           LL_CORDIC_SCALE_0,           /* no scale */
                           LL_CORDIC_NBWRITE_2,         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                           LL_CORDIC_NBREAD_2,          /* Two output data: cosine, then sine */
                           LL_CORDIC_INSIZE_32BITS,     /* q1.16 format for input data */
                           LL_CORDIC_OUTSIZE_32BITS);   /* q1.16 format for output data */
}

void cordicSetPhase32(void)
{
	LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_PHASE,   /* cosine function */
                           LL_CORDIC_PRECISION_6CYCLES, /* max precision for q1.31 cosine */
                           LL_CORDIC_SCALE_0,           /* no scale */
                           LL_CORDIC_NBWRITE_2,         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                           LL_CORDIC_NBREAD_2,          /* Two output data: cosine, then sine */
                           LL_CORDIC_INSIZE_32BITS,     /* q1.32 format for input data */
                           LL_CORDIC_OUTSIZE_32BITS);   /* q1.32 format for output data */
}
