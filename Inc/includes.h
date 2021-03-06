
#ifndef INC_INCLUDES_H_
#define INC_INCLUDES_H_
#define ONE_G 9.80665

#include "stdarg.h"
#include "stdio.h"
#include "stddef.h"
#include "string.h"
#include "types.h"
#include "stdint.h"
#include "flash_if.h"
#include "menu.h"
#include "ymodem.h"

#define M_PI   3.14159265358979323846264338327950288

extern UART_HandleTypeDef huart1;  // to PC
extern UART_HandleTypeDef huart2;  // to SmartAir
//extern UART_HandleTypeDef UartHandle;
#define UartHandle huart2

extern char term_buffer[3000];
typedef  void (*pFunction)(void);
extern pFunction JumpToApplication;
extern uint32_t JumpAddress;

#define false (0)
#define true  (1)
#define bool int8_t

#define PRINT_COM1() { HAL_UART_Transmit(&huart1, (u8*) term_buffer, strlen(term_buffer), 10); }
#define PRINT() {HAL_UART_Transmit(&huart2, (uint8_t *) &term_buffer, strlen(term_buffer),5);}

extern __IO uint32_t flashdestination;

#endif /* INC_INCLUDES_H_ */
