/*
 * usart1.c
 *
 *  Created on: Sep 4, 2022
 *      Author: Christian Mahlburg
 */

#include "stm32u5xx_ll_usart.h"

/**
 * @brief Simple uart transmit in blocking mode.
 */
void usart1_transmit(const char* msg)
{
    LL_USART_EnableDirectionTx(USART1);
    LL_USART_ClearFlag_TC(USART1);

    while(*msg != 0)
    {
        LL_USART_TransmitData8(USART1, *msg);
        while(!LL_USART_IsActiveFlag_TC(USART1));
        LL_USART_ClearFlag_TC(USART1);
        msg++;
    }
    LL_USART_DisableDirectionTx(USART1);
}
