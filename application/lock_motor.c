#include "stm32f0xx.h"
#include "lock_motor.h"
#include "lock_config.h"
#include "main.h"
#include "slave_lock_task.h"

ADC_HandleTypeDef hadc;
uint16_t   aADCxConvertedData[8] = {0x00};

DMA_HandleTypeDef hdma_adc;

extern uint16_t motor_drag_value;
static void _lock_motor_stop(void);

//void XferCpltCallback(struct __DMA_HandleTypeDef * hdma)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t AD_vaule = 0;
	uint8_t i;
	for(i=0;i<8;i++)
	{
		AD_vaule += aADCxConvertedData[i];
	}
	AD_vaule >>= 3;
	if(AD_vaule > motor_drag_value)
	{
		HAL_ADC_Stop_DMA(hadc);
		HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_LOCK_PIN, GPIO_PIN_SET); //brake stop
		HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_UNLOCK_PIN, GPIO_PIN_SET);	
		osal_set_event(slave_lock_task_id,MOTOR_STOP_EVENT);
	}
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hadc->Instance==ADC1)
    {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_ADC1_CLK_ENABLE();

        /**ADC GPIO Configuration
        PA0     ------> ADC_IN0
        */
        GPIO_InitStruct.Pin  = MOTOR_FEEDBACK_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(MOTOR_FEEDBACK_PORT, &GPIO_InitStruct);

        /* USER CODE BEGIN ADC1_MspInit 1 */
    /* ADC1 DMA Init */
    /* ADC Init */
		hdma_adc.Instance = DMA1_Channel1;
		hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc.Init.Mode = DMA_CIRCULAR;
		hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
		
		if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
		{
		  Error_Handler();
		}
//hdma_adc.XferCpltCallback = XferCpltCallback;
		__HAL_LINKDMA(hadc,DMA_Handle,hdma_adc);
        /* USER CODE END ADC1_MspInit 1 */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

    if(hadc->Instance==ADC1)
    {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC1_CLK_DISABLE();

        /**ADC GPIO Configuration
        PA5     ------> ADC_IN5
        */
        HAL_GPIO_DeInit(MOTOR_FEEDBACK_PORT, MOTOR_FEEDBACK_PIN);

        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    }
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

void lock_motor_adc_init(void)
{
    ADC_ChannelConfTypeDef sConfig;
	MX_DMA_Init();
    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = ENABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        //_Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        //_Error_Handler(__FILE__, __LINE__);
    }
	if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
	{
		Error_Handler();
	}	
	if (HAL_ADC_Start_DMA(&hadc,
						(uint32_t *)aADCxConvertedData,
						8
					   ) != HAL_OK)
	{
		Error_Handler();
	} 
	
	 //HAL_ADC_Start(&hadc);
}



//uint8_t motor_feedback_pin_adc(uint8_t time,uint32_t *value)
//{
//    uint32_t total_value = 0;

//    uint8_t i;

//    for(i=0; i<(time+1); i++)
//    {
//        HAL_ADC_Start(&hadc);

//        HAL_ADC_PollForConversion(&hadc, 100);

//        while (!((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC));

//        /*##-6- Get the converted value of regular channel  ########################*/
//        if ( i!= 0)
//        {
//            total_value += HAL_ADC_GetValue(&hadc);
//        }

//        HAL_ADC_Stop(&hadc);
//    }
//    if ( value )
//    {
//        *value = total_value  / time;
//    }
//    return TRUE;
//}

static void _lock_motor_on(int side)
{
    GPIO_PinState LeftPinState,RightPinState;
    LeftPinState  = (side == 0) ? GPIO_PIN_SET   : GPIO_PIN_RESET;
    RightPinState = (side == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_LOCK_PIN, LeftPinState);
    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_UNLOCK_PIN, RightPinState);

}

static void _lock_motor_stop(void)
{

    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_LOCK_PIN,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_UNLOCK_PIN, GPIO_PIN_RESET);

    //HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_LOCK_PIN,   GPIO_PIN_SET);
    //HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_UNLOCK_PIN, GPIO_PIN_SET);
}


void door_lock_motor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = MOTOR_LOCK_PIN | MOTOR_UNLOCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;//GPIO_PULLUP;
    HAL_GPIO_Init(LOCK_MOTOR_PORT, &GPIO_InitStruct);

    _lock_motor_stop();    
}

void door_lock_open(void)
{
    uint8_t unlock_type  = get_unlock_type();
    _lock_motor_stop();
    _lock_motor_on(unlock_type == LEFT_UNLOCK_DOOR ? 0 : 1 );
}
void door_lock_close()
{
    uint8_t unlock_type  = get_unlock_type();
    _lock_motor_stop();
    _lock_motor_on(unlock_type == LEFT_UNLOCK_DOOR ? 1 : 0 );
}

void door_motor_stop(void)
{
    _lock_motor_stop();
}

void door_motor_brake(void)
{
    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_LOCK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_UNLOCK_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_LOCK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOCK_MOTOR_PORT, MOTOR_UNLOCK_PIN, GPIO_PIN_RESET);
}

