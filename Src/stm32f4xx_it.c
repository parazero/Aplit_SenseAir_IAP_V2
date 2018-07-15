/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "includes.h"

extern uint32_t msTicks,msTicks_Send_to_Modem;
extern uint32_t Counter1Sec;
extern uint32_t Counter1Min;
extern uint32_t CounterNoGPS;
void InsertCharToString_UART2(char InChar); // catch string at UART2
extern UART_HandleTypeDef huart5;
extern bool Catched_String_UART1;
extern bool Catched_String_UART2;
extern bool Catched_String_UART5;

//input buffer & pointers UART1 & UART5
char aRxBufferCh5='5';
#define InBuf5SizeDef 2000
char InBuf5[InBuf5SizeDef];
uint16_t InBuf5InPtr=0;

char aRxBufferCh1='1';
#define InBuf1SizeDef 2000
char InBuf1[InBuf1SizeDef];
uint16_t InBuf1InPtr=0;

char aRxBufferCh2='2';
#define InBuf2SizeDef 2000
char InBuf2[InBuf2SizeDef];
uint16_t InBuf2InPtr=0;

char aTxBufferCh5='5';
char aTxBufferCh1='1';
char aTxBufferCh2='2';

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  msTicks++;
  msTicks_Send_to_Modem++;
//  Counter1Sec++;
//  Counter1Min++;
  CounterNoGPS++;

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream7 global interrupt.
*/
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream7 global interrupt.
*/
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//  if (huart == &huart1)
//  { 
//    if (Catched_String_UART1 == false)
//    {
//          if (aRxBufferCh1 =='\r')  // If CR received, then set ready flag
//          {
//             InBuf1[InBuf1InPtr++] = aRxBufferCh1;
//             InBuf1[InBuf1InPtr] = 0; 
//             InBuf1InPtr = 0;
//             Catched_String_UART1 = true;
//              //return; //!!
//             goto label11; //   return;
//          }
//          if ((InBuf1InPtr < InBuf1SizeDef - 2))// && (g_imu.rx_ready != true)) // Store character
//          {	  InBuf1[InBuf1InPtr++] = aRxBufferCh1;
//          }
//          else
//          {	  InBuf1InPtr=0;
//          }
//        }        
//label11:   
//    //    if(HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBufferCh1, 1 ) != HAL_OK)   {  } //  Error_Handler();     
//      if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)&aRxBufferCh1, 1 ) != HAL_OK)   {  } //  Error_Handler();       
//  }  
  if (huart == &huart2)
  {
    
    aTxBufferCh2 = aRxBufferCh2;
  
    if (Catched_String_UART2 == false)
    {
          if (aRxBufferCh2 =='\r')  // If CR received, then set ready flag
          {
             InBuf2[InBuf2InPtr++] = aRxBufferCh2;
             InBuf2[InBuf2InPtr] = 0; 
             HAL_UART_Transmit(&huart5, (uint8_t *)&InBuf2[0], InBuf2InPtr,10); // to modem
             InBuf2InPtr = 0;
             Catched_String_UART2 = true;
             goto label22; //   return;
          }
          if ((InBuf2InPtr < InBuf2SizeDef - 2))// && (g_imu.rx_ready != true)) // Store character
          {	  InBuf2[InBuf2InPtr++] = aRxBufferCh2;
          }
          else
          {	  InBuf2InPtr=0;
          }
        }        
label22:   
    //    if(HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBufferCh2, 1 ) != HAL_OK)   {  } //  Error_Handler();     
      if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)&aRxBufferCh2, 1 ) != HAL_OK)   {  } //  Error_Handler();       
  }
  if (huart == &huart5)
  {   
      aTxBufferCh5 = aRxBufferCh5;
      
      HAL_UART_Transmit(&huart2, (uint8_t *) &aTxBufferCh5, 1,10);
      
        if (Catched_String_UART5 == false)
      {
            if (aRxBufferCh5 =='\r')  // If CR received, then set ready flag
            {
               InBuf5[InBuf5InPtr++] = aRxBufferCh5;
               InBuf5[InBuf5InPtr] = 0; 
               InBuf5InPtr = 0;
               Catched_String_UART5 = true;
                //return; //!!
               goto label55; //   return;
            }
            if ((InBuf5InPtr < InBuf5SizeDef - 2))// && (g_imu.rx_ready != true)) // Store character
            {	  InBuf5[InBuf5InPtr++] = aRxBufferCh5;
            }
            else
            {	  InBuf5InPtr=0;
            }
          }        
  label55:   
    //    if(HAL_UART_Receive_IT(&huart5, (uint8_t *)&aRxBufferCh5, 1 ) != HAL_OK)   {  } //  Error_Handler();     
      if(HAL_UART_Receive_DMA(&huart5, (uint8_t *)&aRxBufferCh5, 1 ) != HAL_OK)   {  } //  Error_Handler();       
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
//      if (huart == &huart1)
//      {   huart->RxState = HAL_UART_STATE_READY; //MX_USART2_UART_Init(); 	 
//             if(HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBufferCh1, 1 ) != HAL_OK)   {  } //  Error_Handler();
//      }  
      if (huart == &huart2)
      {	huart->RxState = HAL_UART_STATE_READY; //MX_USART2_UART_Init(); 	 
             if(HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBufferCh2, 1 ) != HAL_OK)   {  } //  Error_Handler();    
      }
      if (huart == &huart5)
      { huart->RxState = HAL_UART_STATE_READY; //MX_USART2_UART_Init(); 	 
        if(HAL_UART_Receive_IT(&huart5, (uint8_t *)&aRxBufferCh5, 1 ) != HAL_OK)   {  } //  Error_Handler();  }
      }

}



void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  
//  if (hdma==&hdma_usart1_rx)
//  {    	// InBuf1[InBuf1InPtr++] = aRxBufferCh1;	if (InBuf1InPtr>InBuf1Size) InBuf1InPtr=0; 
//	 if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)&aRxBufferCh1, 1 ) != HAL_OK)   {  } //  Error_Handler();
//  }  

//  if (hdma==&hdma_usart2_rx)
//  {	
//	 if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)&aRxBufferCh2, 1 ) != HAL_OK)   {  } //  Error_Handler();    
//  }
  
//  if (hdma==&hdma_uart5_rx)
//  {  
//	//InBuf5[InBuf5InPtr++] = aRxBufferCh5; if (InBuf5InPtr>InBuf5Size) InBuf5InPtr=0;
//	 if(HAL_UART_Receive_DMA(&huart5, (uint8_t *)&aRxBufferCh5, 1 ) != HAL_OK)   {  } //  Error_Handler();
//  }
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
