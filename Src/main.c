
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal_crc.h"

#include "includes.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "common.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef UartHandle;


#define Sense_UID1 (*(__I uint32_t *) 0x1FFF7A10) 
#define Sense_UID2 (*(__I uint32_t *) 0x1FFF7A14) 
#define Sense_UID3 (*(__I uint32_t *) 0x1FFF7A18) 
float Smart_Version_ID;
float Sense_Version_ID = 2.21;

uint32_t  Smart_UID1, Smart_UID2, Smart_UID3;
char term_buffer[3000];

//#define G 9.80665
uint32_t msTicks = 0, msTicks_Send_to_Modem = 0;
uint32_t Counter1Sec;
uint32_t Counter1Min;
uint32_t CounterNoGPS;

//#define ARM_MATH_CM4
//#define __FPU_PRESENT 1

int Wait_Answer(long int wait);

//#define STD_arr_len 20
//        float AX_arr[STD_arr_len];
//        float AY_arr[STD_arr_len];
//        float AZ_arr[STD_arr_len];
//        float AX_Std = 0,AY_Std=0,AZ_Std=0;
//        int A_arr_cnt=0;

// AT commands
char ATcommandCheckGPS[] = "at^sgpsc?\r\n";
char ATcommandCheckGPSEngine[] = "at^sgpsc:\"Engine\"\r\n";
char ATcommandEnableGPS[] = "at^sgpsc=\"Engine\",3\r\n";
char ATcommandDisableGPS_GPGSV[] = "at^SGPSC=\"Nmea/Data\",\"GSV\",\"off\"\r\n";
char ATcommandEnableGPS_GPRMC[] = "at^SGPSC=\"Nmea/Data\",\"RMC\",\"on\"\r\n";

//input buffer & pointers UART1 & UART5
char * status = "1";
char * id = "1";

extern char aRxBufferCh5;
extern char InBuf5[];
extern uint16_t InBuf5InPtr;

extern char aRxBufferCh1;
extern char InBuf1[];
extern uint16_t InBuf1InPtr;

extern char aRxBufferCh2;
extern char InBuf2[];
extern uint16_t InBuf2InPtr;

bool Catched_String_UART1 = false;
bool Catched_String_UART2 = false;
bool Catched_String_UART5 = false;
float roll_calib = 0, pitch_calib = 0;
void InsertCharToString_UART2(char InChar); // catch string at UART2
char pbuf_IN_UART2[500];
uint16_t pbuf_IN_UART2_ptr = 0;

uint16_t Great_Status=0;
uint16_t Destination=0; 			//  where to send data from modem
uint16_t emergency_state=0;
//--------------------

uint8_t pos, start, Catching_Gps_Message = 0, Got_Gps_Message = 0,
		Got_RMC_Message = 0, Got_GGA_Message = 0;
char pbuf[200] = "";
uint16_t pbuf_ptr = 0;
char header[100] = "";

uint8_t gprmc_rdy = 0;
uint8_t gpgga_rdy = 0;

uint16_t SentCnt = 0; // counter of sent GPS messages to cloud

float GPS_lat, GPS_lon, angle, Hangle, lat, lon, lat_seconds, lon_seconds, HDOP,
	  height, velocity;

char height_units, HangleC, ModeC, Valid, lat_sign, lon_sign;

int GPS_hour, GPS_min, GPS_sec, GPS_ms, GPS_day, GPS_month, GPS_year,
		lat_degrees, lat_minutes, lon_degrees, lon_minutes, precision,
		satellites;
bool need_smart_cfg = 1;

typedef struct sEE_DATA
{
  uint32_t status;
  float att_roll_lim;
  float att_pitch_lim;
  float att_rate_lim;
  float att_rate; 
  float yaw_rate;
  float height_lim;
  float fall_value;
  float pyro_res;
  uint8_t count_fall;
  uint8_t rc;
  uint8_t emr_bat_ena;
  uint8_t num_of_cell;
  uint8_t ec;
  uint16_t motor_delay;
  float sea_level;
  uint16_t dst;                              // where to send data from modem
  uint8_t use_sd_card;
  uint16_t pyro_test_freq;
  char Password[30];
  char Log_file_name[255];
} tEE_DATA;
tEE_DATA ee;

char ResultStringToCloud[500];
char StringToSmart[500];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

pFunction JumpToApplication;
uint32_t JumpAddress;
uint32_t FlashProtection = 0;
uint8_t aFileName[FILE_NAME_LENGTH];

/* Private function prototypes -----------------------------------------------*/
int SerialDownload(bool move);
void SerialUpload(void);

extern char aRxBufferCh1;
extern char aRxBufferCh2;
extern char aRxBufferCh5;


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Download a file via serial port
  * @param  None
  * @retval None
  */
    

void Jump_to_main_programm(void)
{
      JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
      JumpToApplication = (pFunction) JumpAddress;
      __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
      JumpToApplication();  
}    


 /* calculating CRC */
        
u32 revbit(u32 data)
 {
   asm("rbit r0,r0");
   return data;
 };

u32 CalcCRC32(u8 *buffer,u32 size)
 {
   u32 i, j;
   u32 ui32;
   //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);

   CRC->CR=1;

   asm("NOP");asm("NOP");asm("NOP");//delay for hardware ready

   i = size >> 2;

   while(i--)
   {
     ui32=*((u32 *)buffer);

     buffer += 4;

     ui32=revbit(ui32);//reverse the bit order of input data

     CRC->DR=ui32;
   }

   ui32=CRC->DR;

   ui32=revbit(ui32);//reverse the bit order of output data

   i = size & 3;

   while(i--)
   {
     ui32 ^= (u32)*buffer++;

     for(j=0; j<8; j++)
       if (ui32 & 1)
         ui32 = (ui32 >> 1) ^ 0xEDB88320;
       else
         ui32 >>= 1;
   }

   ui32^=0xffffffff;//xor with 0xffffffff

   return ui32;//now the output is compatible with windows/winzip/winrar
 };

void vprint(const char *fmt, va_list argp) {
	char string[300];
	if (0 < vsprintf(string, fmt, argp)) // build string
			{
		HAL_UART_Transmit(&huart1, (u8*) string, strlen(string), 500); // send message via UART
	}
}

void send_to_PC_UART1(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

void vprint2(const char *fmt, va_list argp) {
	char string[300];
	if (0 < vsprintf(string, fmt, argp)) // build string
	{
		HAL_UART_Transmit(&huart2, (u8*) string, strlen(string), 500); // send message via UART
	}
}

void send_to_smartair(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	vprint2(fmt, argp);
	va_end(argp);
}
void vprint3(const char *fmt, va_list argp) {
	char string[300];
	if (0 < vsprintf(string, fmt, argp)) // build string
			{
		HAL_UART_Transmit(&huart5, (u8*) string, strlen(string), 500); // send message via UART
	}
}

void send_to_modem(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	vprint3(fmt, argp);
	va_end(argp);
}

void InsertCharToString_UART2(char InChar) // catch string at UART2
{       if (!Catched_String_UART2) 
        {    
          pbuf_IN_UART2[pbuf_IN_UART2_ptr++] = InChar;
          if (InChar=='\r')
              { Catched_String_UART2 = true;
                pbuf_IN_UART2[pbuf_IN_UART2_ptr]=0;
                pbuf_IN_UART2_ptr = 0;
              }
          else
          { Catched_String_UART2 = false; 
            if (pbuf_IN_UART2_ptr>=500) 
              { pbuf_IN_UART2_ptr=0; 
              }
          }
	}
}


//uint32_t acu_File[3000];
//uint32_t ccu_File[3000];
extern uint32_t msTicks,msTicks_Send_to_Modem;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;

extern bool Catched_String_UART5;
//char aRxBufferCh5='5';
//#define InBuf5SizeDef 2000
extern char InBuf5[];
extern uint16_t InBuf5InPtr;
uint32_t acuCRC =0 ,ccuCRC = 0;
uint32_t acu_Size = 0,ccu_Size = 0;

//char term_buffer[1000];

int Wait_Answer(long int wait)
{     term_buffer[0] = 0;
      msTicks = 0;
      if (wait<20000) wait = 20000;
      while(msTicks<wait)
      {
          if (Catched_String_UART5 == true)
          {   
    //        if ((strlen(term_buffer)+strlen(InBuf5)) > sizeof(term_buffer)) { term_buffer[0] = 0; }
    //            strcat(term_buffer,InBuf5);
    //            //HAL_UART_Transmit(&huart1, (uint8_t *) &InBuf5, strlen(InBuf5), 5); // send buf to PC via uart1
    //            //Catched_String_UART5 = false;
                Catched_String_UART5 = false;
                if (strstr((char*)InBuf5,"OK")!=NULL) break;
                if (strstr((char*)InBuf5,"ERR")!=NULL) break;
                if (strstr((char*)InBuf5,"^SYSSTART")!=NULL) break;
                if (strstr((char*)InBuf5,"CONNECT")!=NULL) break;


    //            if (strstr((char*)InBuf5,"OK")!=NULL) break;
    //            if (strstr((char*)InBuf5,"ERR")!=NULL) break;
    //            if (strstr((char*)InBuf5,"^SYSSTART")!=NULL) break;
    //            if (strstr((char*)InBuf5,"CONNECT")!=NULL) break;
            }
      }
      HAL_Delay(300);

//      strcat(term_buffer,"\n\0");
//      HAL_UART_Transmit(&huart2, (uint8_t *) &term_buffer, strlen(term_buffer), 50); // send buf to PC via uart1
//      //HAL_Delay(50);
//      HAL_UART_Transmit(&huart1, (uint8_t *) &term_buffer, strlen(term_buffer), 50); // send buf to PC via uart1
//      //HAL_Delay(50);
}

int AT_init_Modem()
{
   
//            at^sgpsc="Engine",0
//                      at^sgpsc="Engine",3 
//             AT^SFSA="ls", "a:/"
//            AT^SFSA="rename", "a:/MQTT.jar", "MQT.ja"
//
//            AT^SFSA="remove", "a:/MQTT.jad"
//            AT^SFSA="remove", "a:/MQTT.jar"
//              AT^SFSA="remove", AT^SJAM=2,"http://192.168.244.2:49363/MQTT.jad",""
//      AT^SJAM=3,"http://192.168.244.2:49363/MQTT.jad",""
//            AT^SJAM=4                                 list installed apps 
//            AT^SJAM=5                                 list running apps

//            AT^SJAM=0,"a:/MQTT.jar",""                install
//
//            AT^SJAM=1,"a:/MQTT.jar",""                start
//            
//            AT^SJAM=2,"a:/MQTT.jar",""                stop
//            
//            AT^SJAM=3,"a:/MQTT.jar",""                uninstall
//           
//            at^sgpsc="Engine",0 
//
//            AT^SJDL=1,330,"MQTT.jad" upload file

 //                         AT^SJDL=1,50,"test.jad"  
//            AT^SJDL=2,30,"MQTT.jad" delete file
//                AT^SJDL=2,30,"MQTT.jar"
//            AT^SJDL=1,228283,"MQTT.jar"
//                      SPOW set sleep mode
//            AT^SGPSC?
 //  AT^SCFG="Serial/Interface/Allocation","0","0"
//AT^SCFG="Serial/Interface/Allocation",0
//                                              AT^SGPSC="Nmea/Output","on"             AT^SGPSC="Nmea/Output","off"
//            AT^SGPSC="Nmea/Interface","asc0"
//      AT^SGPSC="Nmea/Interface","usb5"
//            AT^SGPSC="Nmea/Interface","local"
//            at^sgpsc?
//                      ate 0   AT commanf echo OFF         
//        at^spow? 
//              at^spow=1,0,0    enable asc0
// ate 0                                  
                     //   AT^SCFG="Userware/Autostart",0
  
//                              AT^SCFG="Userware/Autostart/Delay",10
                          
//AT^SCFG="GPIO/mode/DCD0","std" (enables DCD0 line for ASC0)
//AT^SCFG="GPIO/mode/DTR0","std" (enables DTR0 line for ASC0)
//AT^SCFG="GPIO/mode/DSR0","std" (enables DSR0 line for ASC0)
//
//AT^SCFG="GPIO/mode/DCD0","gpio" 
//AT^SCFG="GPIO/mode/DTR0","gpio" 
//AT^SCFG="GPIO/mode/DSR0","gpio" 
//
//AT^SCFG="Userware/Stdout","ASC0",,,,"off"
//AT^SCFG="Userware/Stdout","USB1",,,,"off"
//AT+CMUX?                        ???????????????????????????? "Multiplexer User's Guide" [4] which
//                          
//             AT^SISX=<service>, <conProfileId>, <address>[, <request>[, <timelimit>]]             
//AT^SISX="Ping", 6, "8.8.8.8",3 AT^SISX=?
//  AT^SISX="HostByName",3,"www.nist.gov"
//AT^SISX="Ping",0,8.8.8.8,4,2000
//AT^SICS=0,apn,"internet.golantelecom.co.il"

//  AT^SCFG="Radio/OutputPowerReduction","0"
// AT^SJMSEC?
//!!      Wait_Answer2(10000);
//
      HAL_UART_Transmit(&huart5, (uint8_t *) "ate 1\r\n",sizeof("ate 0\r\n"),10);                                        // ok
      Wait_Answer(5000);

      HAL_UART_Transmit(&huart5, (uint8_t *) "at^spow=1,0,0\r\n",sizeof("at^spow=1,0,0\r\n"),10);                       // ok
      Wait_Answer(1000);      

      HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SGPSC=\"Engine\",\"0\"\r" ,strlen("AT^SGPSC=\"Engine\",\"0\"\r"),20);   //  GPS on
      Wait_Answer(5000);      

      HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJAM=2,\"a:/MQTT.jar\",\"\"\r\n",sizeof("AT^SJAM=2,\"a:/MQTT.jar\",\"\"\r\n"),100);  
      Wait_Answer(40000);        
      
////      HAL_UART_Transmit(&huart5, (uint8_t *) "at^sgpsc?\r\n",sizeof("at^sgpsc?\r\n"),1);                                        // ok
////      Wait_Answer(10000);
//      
////      HAL_UART_Transmit(&huart5, (uint8_t *) "at^scfg?\r\n",sizeof("at^scfg?\r\n"),1);                                        // ok
////      Wait_Answer(10000);

}


//void MX_UART5_Init(void);
//void MX_USART1_UART_Init(void);
//void MX_USART2_UART_Init(void);
//

int SerialDownload(bool move)
{
  uint8_t number[11] = {0};
  uint32_t size = 0;
 
  COM_StatusTypeDef result;
 // sprintf(term_buffer,"\r\nSenseAir waiting for SmartAir starts to send new FW for SenseAir\r\n"); PRINT();

  HAL_UART_DMAStop(&huart5);
  HAL_UART_DMAStop(&huart2);

  result = Ymodem_Receive( &size );
  
  sprintf(term_buffer,"\r\nSenseAir Ymodem recieve ended.\r\n"); PRINT();
  
  if (HAL_UART_Receive_DMA(&huart5, (uint8_t *) &aRxBufferCh5, 1) != HAL_OK) {	} //Error_Handler();
  if (HAL_UART_Receive_DMA(&huart2, (uint8_t *) &aRxBufferCh2, 1) != HAL_OK) {	} //Error_Handler();
  
  
  if (result == COM_OK)
  { 
        if (strstr((char*)aFileName,"ACU.bin")!=NULL) 
        {
          // DELETE sertificates 

            AT_init_Modem();      
          
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC?\r\n",sizeof("AT^SJMSEC?\r\n"),100);  
            Wait_Answer(20000);        

            HAL_UART_Transmit(&huart5, (uint8_t *) "at^sjmsec=cmd,\"0600B1000000\"\r\n",sizeof("at^sjmsec=cmd,\"0600B1000000\"\r\n"),100);  
            Wait_Answer(20000);        
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT+CFUN=1,1\r\n",sizeof("AT+CFUN=1,1\r\n"),100);          // reboot
            sprintf(term_buffer,"\r\nRestarting modem..\r\n"); PRINT();
            Wait_Answer(10000);            
            AT_init_Modem();      

            HAL_UART_Transmit(&huart5, (uint8_t *) "at^sjmsec=cmd,\"060091000000\"\r\n",sizeof("at^sjmsec=cmd,\"060091000000\"\r\n"),100);  
            Wait_Answer(10000);        
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT+CFUN=1,1\r\n",sizeof("AT+CFUN=1,1\r\n"),100);          // reboot
            sprintf(term_buffer,"\r\nRestarting modem..\r\n"); PRINT();
            Wait_Answer(10000);            
            AT_init_Modem();      
            
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SFSA=\"remove\",\"a:/ACU.bin\"\r\n",sizeof("AT^SFSA=\"remove\",\"a:/ACU.bin\"\r\n"),100);  
            Wait_Answer(10000);        
            
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SFSA=\"remove\",\"a:/CCU.bin\"\r\n",sizeof("AT^SFSA=\"remove\",\"a:/CCU.bin\"\r\n"),100);  
            Wait_Answer(10000);        

            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC?\r\n",sizeof("AT^SJMSEC?\r\n"),100);  
            Wait_Answer(10000);        
            
                  
      // SEND ACU.FILE
            acu_Size = size; //acu_File = (uint8_t*)DOWNLOAD_ADDRESS;
            acuCRC = CalcCRC32((uint8_t*)DOWNLOAD_ADDRESS,acu_Size);
            sprintf(term_buffer,"AT^SJDL=1,%d,\"ACU.bin\"\r\n",acu_Size);// PRINT();                // prepare to send file
            HAL_Delay(150);
            HAL_UART_Transmit(&huart5, (uint8_t *) term_buffer,strlen(term_buffer),100);   
            Wait_Answer(10000);      

            sprintf(term_buffer,"\r\nSending ACU file. size: %d CRC: 0x%0x\r\n",acu_Size,acuCRC); PRINT();
            HAL_Delay(150);      
            HAL_UART_Transmit(&huart5, (uint8_t *) DOWNLOAD_ADDRESS,(acu_Size),10000);         
            Wait_Answer(10000);      

            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC=\"file\",\"ACU.bin\"\r\n",sizeof("AT^SJMSEC=\"file\",\"ACU.bin\"\r\n"),100);  
            Wait_Answer(10000);            
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT+CFUN=1,1\r\n",sizeof("AT+CFUN=1,1\r\n"),100);           // reboot modem
            sprintf(term_buffer,"\r\nRestarting modem..\r\n"); PRINT();
            Wait_Answer(10000);            
      
            AT_init_Modem();      

            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC?\r\n",sizeof("AT^SJMSEC?\r\n"),100);  
            Wait_Answer(10000);        
            
            sprintf(term_buffer,"\n\rSenseAir Modem ACU sertificate loaded\n\r");
            HAL_UART_Transmit(&huart2, (uint8_t *) &term_buffer, strlen(term_buffer), 100); // send buf to PC via uart1
                
        }
        else
        if (strstr((char*)aFileName,"CCU.bin")!=NULL)           
        {   
          
            AT_init_Modem();      
                      
      // SEND CCU.FILE
            ccu_Size = size; //ccu_File = DOWNLOAD_ADDRESS;
            ccuCRC = CalcCRC32((uint8_t*)DOWNLOAD_ADDRESS,ccu_Size);
            sprintf(term_buffer,"AT^SJDL=1,%d,\"CCU.bin\"\r\n",ccu_Size);// PRINT();                // prepare to send  file
            HAL_Delay(150);
            HAL_UART_Transmit(&huart5, (uint8_t *) term_buffer,strlen(term_buffer),100);   
            Wait_Answer(10000);      

            sprintf(term_buffer,"\r\nSending CCU file. size: %d CRC: 0x%0x\r\n",ccu_Size,ccuCRC); PRINT();
            HAL_Delay(150);      
            HAL_UART_Transmit(&huart5, (uint8_t *) DOWNLOAD_ADDRESS,(ccu_Size),10000);         
            Wait_Answer(10000);      

            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC=\"file\",\"CCU.bin\"\r\n",sizeof("AT^SJMSEC=\"file\",\"CCU.bin\"\r\n"),100);  
            Wait_Answer(10000);            
            
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT+CFUN=1,1\r\n",sizeof("AT+CFUN=1,1\r\n"),100);           // reboot modem
            sprintf(term_buffer,"\r\nRestarting modem..\r\n"); PRINT();
            Wait_Answer(10000);            
            AT_init_Modem();      
            
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC=\"cmd\",\"0B00310001000500020001\"\r\n",sizeof("AT^SJMSEC=\"cmd\",\"0B00310001000500020001\"\r\n"),100);  
            Wait_Answer(10000);            
            HAL_UART_Transmit(&huart5, (uint8_t *) "AT+CFUN=1,1\r\n",sizeof("AT+CFUN=1,1\r\n"),100);
            sprintf(term_buffer,"\r\nRestarting modem..\r\n"); PRINT();
            Wait_Answer(10000);            

            AT_init_Modem();      

            HAL_UART_Transmit(&huart5, (uint8_t *) "AT^SJMSEC?\r\n",sizeof("AT^SJMSEC?\r\n"),100);  
            Wait_Answer(10000);                  

            sprintf(term_buffer,"\r\nSenseAir Modem CCU sertificate loaded\r\n"); PRINT();

        }    
        else
        {
            if (move)
            {
              FLASH_If_Erase(APPLICATION_ADDRESS,USER_FLASH_SIZE);
              if (FLASH_If_Write(APPLICATION_ADDRESS, (uint32_t*) DOWNLOAD_ADDRESS, size/4+1) == FLASHIF_OK)
              { 
                    sprintf(term_buffer,"\r\nLoading SenseAir FW from SmartAir completed successfully. Name: %s Size: %d bytes.\r\n",aFileName,size); PRINT();
                    uint32_t AA = 0xAAAAAAAA;
                    uint32_t FF = 0xFFFFFFFF;
                    uint32_t* ee_flash_base = (uint32_t*)FLAG_ADDRESS;  
            //        uint32_t Res = *ee_flash_base;
            //        if (Res==FF)
                      { 
                        FLASH_If_Erase(FLAG_ADDRESS,4);
                        if (FLASH_If_Write(FLAG_ADDRESS, &FF,1) == FLASHIF_OK)                                        
                        { sprintf(term_buffer,"\n\rSenseAir Flag \"Need update modem\" succesfully written.\r\n"); PRINT();
                        }
                        else
                        { sprintf(term_buffer,"\n\rSenseAir Error. Flag \"Need update modem\" not written.\r\n"); PRINT();
                        }
                      }
                    HAL_UART_DMAStop(&huart5);
                    HAL_UART_DMAStop(&huart1);
                    HAL_UART_DMAStop(&huart2);
                    Jump_to_main_programm(); 
              }
              else /* An error occurred while writing to Flash memory */
              { sprintf(term_buffer,"\r\nSenseAir Error moving FW to start addr\r\n"); PRINT();      
              }
            }
              //sprintf(term_buffer,"\r\nLoading SenseAir FW from SmartAit completed successfully. Name: %s Size: %d bytes.",aFileName,size); PRINT();
       }
  }
  else if (result == COM_LIMIT)
  {
    sprintf(term_buffer,"\r\nSenseAir The FW size is higher than the allowed space memory!\r\n"); PRINT();
  }
  else if (result == COM_DATA)
  {
    sprintf(term_buffer,"\r\nSenseAir Verification failed!\r\n"); PRINT();
  }
  else if (result == COM_ABORT)
  {
    sprintf(term_buffer,"\r\n\n\nSenseAir Aborted by user.\r\n"); PRINT();
  }
  else
  {
    sprintf(term_buffer,"\r\nSenseAir Failed to receive the file!\r\n"); PRINT();
  }
  return result;
}

/**
  * @brief  Upload a file via serial port.
  * @param  None
  * @retval None
  */
void SerialUpload(void)
{
  uint8_t status = 0;

//  sprintf(term_buffer,"\r\nSelect Receive File"); PRINT();

  HAL_UART_Receive(&UartHandle, &status, 1, RX_TIMEOUT);
  if ( status == CRC16)
  {
    /* Transmit the flash image through ymodem protocol */
    status = Ymodem_Transmit((uint8_t*)DOWNLOAD_ADDRESS, (const uint8_t*)"UploadedFlashImage.bin", USER_FLASH_SIZE);

    if (status != 0)
    {
      sprintf(term_buffer,"\r\nSenseAir Error occurred while Transmitting File\r\n"); PRINT();
    }
    else
    {
      sprintf(term_buffer,"\r\nSenseAir file uploaded successfully\r\n"); PRINT();
    }
  }
}





/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void init_modem() {
	uint16_t b0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	if (b0 == 0)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}
 



 /* calculating CRC */
//        
//void CRC32(uint8_t* Result, uint8_t* pTemp,uint16_t pTemp_bytes)
//{     uint32_t tmpCrc;
//
//      tmpCrc = HAL_CRC_Calculate(&hcrc, (uint32_t*)pTemp, (pTemp_bytes - 4) / 4);
//      if(pTemp_bytes % 4 > 0)
//      {   uint8_t i;
//          tmpCrc = 0;
//          for(i = 0; i > (pTemp_bytes % 4); i++)
//          {   tmpCrc = tmpCrc << 0x08;
//              tmpCrc |= pTemp[pTemp_bytes - 4 + i]; 
//          }
//          tmpCrc = HAL_CRC_Accumulate(&hcrc, &tmpCrc, 1);
//      }
//      Result[0] = (uint8_t)tmpCrc;
//      Result[1] = (uint8_t)(tmpCrc >> 0x08);
//      Result[2] = (uint8_t)(tmpCrc >> 0x10);
//      Result[3] = (uint8_t)(tmpCrc >> 0x18);
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
//        perepheral_sensors_init();
        
//#define __RELEASE__        
#ifdef __RELEASE__
        FLASH_OBProgramInitTypeDef OptionsBytesStruct; 
        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock(); 
//        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); 
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);  
        HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
        if((OptionsBytesStruct.RDPLevel) == OB_RDP_LEVEL_0)
        { OptionsBytesStruct.OptionType= OPTIONBYTE_RDP;
          OptionsBytesStruct.RDPLevel   = OB_RDP_LEVEL_1;
          if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
          { sprintf(term_buffer,"\r\nOPB set error."); //strcat(start_buffer,term_buffer); 
            PRINT();
          }
          else
          { sprintf(term_buffer,"\r\nOPB set OK."); //strcat(start_buffer,term_buffer); 
            PRINT();
          }
          HAL_FLASH_OB_Launch();
        }
        HAL_FLASH_OB_Lock();
#endif        


//        Wait_Answer(10000);     
       
//	if (HAL_UART_Receive_DMA(&huart1, (uint8_t *) &aRxBufferCh1, 1) != HAL_OK) {	} //Error_Handler();
//	if (HAL_UART_Receive_DMA(&huart2, (uint8_t *) &aRxBufferCh2, 1) != HAL_OK) {	} //Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      
        uint32_t* ee_flash_base = (uint32_t*)FLAG_ADDRESS;  
        uint32_t Res = *ee_flash_base;

	while (1) 
        {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
          if (Res==0xFFFFFFFF)
          { 
            init_modem(); // Init Modem

            if (HAL_UART_Receive_DMA(&huart5, (uint8_t *) &aRxBufferCh5, 1) != HAL_OK) {	} //Error_Handler();
//	    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *) &aRxBufferCh1, 1) != HAL_OK) {	} //Error_Handler();
            if (HAL_UART_Receive_DMA(&huart2, (uint8_t *) &aRxBufferCh2, 1) != HAL_OK) {	} //Error_Handler();

            UartHandle = huart2;
            HAL_Delay(7250); // how long or till Ack?
      
            HAL_UART_Transmit(&huart5, (uint8_t *) "at^spow=1,0,0\r\n",sizeof("at^spow=1,0,0\r\n"),10);                       // ok

            __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
            uint32_t SDLcnt = 0;
            while(1) 
            { if(Catched_String_UART2 == true)
                { if (strstr((char*)InBuf2,"GET FILE")!=NULL) 
                    {
                      while (SDLcnt++<12000)
                      {   HAL_Delay(300);
                          if (SerialDownload(true)==COM_OK) break;
                      }
                    }
                  Catched_String_UART2 = false;
                }
            }
          }
          else
          {     sprintf(term_buffer,"\r\nSenseAir bootloader. Start main program execution."); PRINT();
                HAL_UART_DMAStop(&huart5);
                Jump_to_main_programm(); 
          }
        }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB13   ------> SPI2_SCK
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC4 PC5 
                           PC6 PC7 PC8 PC9 
                           PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA10 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11 
                           PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
