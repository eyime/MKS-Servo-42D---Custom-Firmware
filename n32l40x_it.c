#include "n32l40x_it.h"
#include "n32l40x.h"

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
/* NTFx CODE END */

}
/* NTFx CODE START */
/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    /* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles Memory Manage exception.
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
/* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles Bus Fault exception.
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
/* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles Usage Fault exception.
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
/* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
/* NTFx CODE END */

}
/* NTFx CODE START */
/**
 * @brief  This function handles Debug Monitor exception.
 */
void DebugMon_Handler(void)
{
/* NTFx CODE END */

}
/* NTFx CODE START */
/**
 * @brief  This function handles SysTick Handler.
 */
// Defined in foc_time.c
//void SysTick_Handler(void)
//{
/* NTFx CODE END */
//}
/* NTFx CODE START(USART1_IRQHandler)*/
/**
* @brief  This function handles USART1IRQHandler.
*/
// Defined in foc_comm.c
//void USART1_IRQHandler(void)
//{
/* NTFx CODE END */

/* NTFx CODE START */
//    if (USART_GetIntStatus(USART1, USART_INT_TXDE))
//    {
/* NTFx CODE END */

//    }
/* NTFx CODE START */
//    if (USART_GetIntStatus(USART1, USART_INT_RXDNE))
//    {
/* NTFx CODE END */

//    }
/* NTFx CODE START */
//    if (USART_GetIntStatus(USART1, USART_INT_OREF)||
//        USART_GetIntStatus(USART1, USART_INT_NEF)||
//        USART_GetIntStatus(USART1, USART_INT_FEF)||
//        USART_GetIntStatus(USART1, USART_INT_PEF))
//    {
        /*clear IT flag*/
        /*Read the sts register first,and the read the DAT register to clear the all error flag*/
//        (void)USART1->STS;
//        (void)USART1->DAT;
/* NTFx CODE END */

//    }

/* NTFx CODE START */
//}
/* NTFx CODE END(USART1_IRQHandler)*/

