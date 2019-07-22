#include "main.h"
#include "stm32f0xx.h"
#include "doorbell.h"


void door_bell_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;//GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
    /*
    	GPIO_InitStruct.Pin   = GPIO_PIN_14| GPIO_PIN_15;
    	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    	GPIO_InitStruct.Pull  = GPIO_PULLUP;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/

}

void door_bell_play(void)
{
	door_bell_init();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
    halTimerDelay(150);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
}

