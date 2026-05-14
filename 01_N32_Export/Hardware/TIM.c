#include "TIM.h"

void TIM5_Init(void)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM5, ENABLE);

    TIM_TimeBaseInitType TIM5_InitStructure;
    TIM_InitTimBaseStruct(&TIM5_InitStructure);
    TIM5_InitStructure.CntMode = TIM_CNT_MODE_UP;
    TIM5_InitStructure.Prescaler = 71;
    TIM5_InitStructure.ClkDiv = TIM_CLK_DIV1;
    TIM5_InitStructure.Period = 999;

    TIM_InitTimeBase(TIM5, &TIM5_InitStructure);
    TIM_ConfigInt(TIM5, TIM_INT_UPDATE, ENABLE);

    NVIC_InitType NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Enable(TIM5, ENABLE);
}