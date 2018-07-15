/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bma2x2_support.c
* Date: 2016/03/11
* Revision: 1.0.3 $
*
* Usage: Sensor Driver support file for  BMA2x2 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
//!!
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "includes.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include <string.h>

#include "../../Perepheral_drivers/Inc/bma2x2.h"
#define BMA2x2_API

extern SPI_HandleTypeDef hspi1;
/*----------------------------------------------------------------------------*
*	The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMA2x2_API

/* \Brief: The function is used as SPI bus write
 * \Return : Status of the SPI write
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *      will data is going to be written
 * \param reg_data : It is a value hold in the array,
 *	will be used for write the value into the register
 * \param cnt : The no of byte of data to be write
 */
s8 BMA2x2_SPI_bus_write(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/* \Brief: The function is used as SPI bus read
 * \Return : Status of the SPI read
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *   will data is going to be read
 * \param reg_data : This data read from the sensor, which is hold in an array
 * \param cnt : The no of byte of data to be read */
s8 BMA2x2_SPI_bus_read(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/

s8 SPI_routine(struct bma2x2_t  *bma2x2);
#endif
/********************End of I2C/SPI function declarations*******************/
/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BMA2x2_delay_msek(u32 msek);
/*!
 *	@brief This function is an example for delay
 *	@param : None
 *	@return : communication result
 */
s32 bma2x2_data_readout_template(struct bma2x2_t* bma2x2);
/*----------------------------------------------------------------------------*
*  struct bma2x2_t parameters can be accessed by using bma2x2
 *	bma2x2_t having the following parameters
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*
*  V_BMA2x2RESOLUTION_u8R used for selecting the accelerometer resolution
 *	12 bit
 *	14 bit
 *	10 bit
*----------------------------------------------------------------------------*/
extern u8 V_BMA2x2RESOLUTION_u8R;
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bma2x2_data_readout_template(struct bma2x2_t* bma2x2)
{
	/*Local variables for reading accel x, y and z data*/
	s16	accel_x_s16, accel_y_s16, accel_z_s16 = BMA2x2_INIT_VALUE;

	/* bma2x2acc_data structure used to read accel xyz data*/
	struct bma2x2_accel_data sample_xyz;
	/* bma2x2acc_data_temp structure used to read
		accel xyz and temperature data*/
	struct bma2x2_accel_data_temp sample_xyzt;
	/* Local variable used to assign the bandwidth value*/
	u8 bw_value_u8 = BMA2x2_INIT_VALUE;
	/* Local variable used to set the bandwidth value*/
	u8 banwid = BMA2x2_INIT_VALUE;
	/* status of communication*/
	s32 com_rslt = ERROR;


/*********************** START INITIALIZATION ************************
  *	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bma2x2 API*/
	#ifdef BMA2x2_API

	SPI_routine(bma2x2);
	#endif
 /*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
 *-------------------------------------------------------------------------*/
	com_rslt = bma2x2_init(bma2x2);

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	NORMAL mode is set from the register 0x11 and 0x12
 *	0x11 -> bit 5,6,7 -> set value as 0
 *	0x12 -> bit 5,6 -> set value as 0
 *	data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL
 *	For the Normal/standby/Low power 2 mode Idle time
		of at least 2us(micro seconds)
 *	required for read/write operations*/
	/* Set the power mode as NORMAL*/
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
/*	Note:
	* For the Suspend/Low power1 mode Idle time of
		at least 450us(micro seconds)
	* required for read/write operations*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the bandwidth of the sensor input
	value have to be given
	bandwidth is set from the register 0x10 bits from 1 to 4*/
	bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
	com_rslt += bma2x2_set_bw(bw_value_u8);

	/* This API used to read back the written value of bandwidth*/
	com_rslt += bma2x2_get_bw(&banwid);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*---------------------------------------------------------------------*/
	/* Read the accel X data*/
	com_rslt += bma2x2_read_accel_x(&accel_x_s16);
	/* Read the accel Y data*/
	com_rslt += bma2x2_read_accel_y(&accel_y_s16);
	/* Read the accel Z data*/
	com_rslt += bma2x2_read_accel_z(&accel_z_s16);

	/* accessing the bma2x2acc_data parameter by using sample_xyz*/
	/* Read the accel XYZ data*/
	com_rslt += bma2x2_read_accel_xyz(&sample_xyz);

	/* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
	/* Read the accel XYZT data*/
	com_rslt += bma2x2_read_accel_xyzt(&sample_xyzt);

/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "DEEP SUSPEND"
 *	DEEP SUSPEND mode is set from the register 0x11
 *	0x11 -> bit 5 -> set value as 1
 *	the device reaches the lowest power consumption only
 *	interface selection is kept alive
 *	No data acquisition is performed
 *	by using the below API able to set the power mode as DEEPSUSPEND*/
 /* Set the power mode as DEEPSUSPEND*/
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return com_rslt;
}

s32 bma2x2_data_readout(struct bma2x2_t* bma2x2, struct bma2x2_accel_data_temp *xyzt)
{
	/*Local variables for reading accel x, y and z data*/
//	s16	accel_x_s16, accel_y_s16, accel_z_s16 = BMA2x2_INIT_VALUE;

	/* bma2x2acc_data structure used to read accel xyz data*/
//	struct bma2x2_accel_data sample_xyz;
	/* bma2x2acc_data_temp structure used to read
		accel xyz and temperature data*/
	struct bma2x2_accel_data_temp sample_xyzt;
	/* Local variable used to assign the bandwidth value*/
	u8 bw_value_u8 = BMA2x2_INIT_VALUE;
	/* Local variable used to set the bandwidth value*/
	u8 banwid = BMA2x2_INIT_VALUE;
	/* status of communication*/
	s32 com_rslt = ERROR;


/*********************** START INITIALIZATION ************************
  *	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bma2x2 API*/
	#ifdef BMA2x2_API

//	SPI_routine(bma2x2);
	#endif
 /*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
 *-------------------------------------------------------------------------*/
//	com_rslt = bma2x2_init(bma2x2);

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	NORMAL mode is set from the register 0x11 and 0x12
 *	0x11 -> bit 5,6,7 -> set value as 0
 *	0x12 -> bit 5,6 -> set value as 0
 *	data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL
 *	For the Normal/standby/Low power 2 mode Idle time
		of at least 2us(micro seconds)
 *	required for read/write operations*/
	/* Set the power mode as NORMAL*/
//	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
/*	Note:
	* For the Suspend/Low power1 mode Idle time of
		at least 450us(micro seconds)
	* required for read/write operations*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the bandwidth of the sensor input
	value have to be given
	bandwidth is set from the register 0x10 bits from 1 to 4*/
//!!!!	com_rslt += bma2x2_set_bw(BMA2x2_BW_7_81HZ);

	/* This API used to read back the written value of bandwidth*/
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*---------------------------------------------------------------------*/
	/* Read the accel X data*/
//	com_rslt += bma2x2_read_accel_x(&accel_x_s16);
//	*ax = accel_x_s16;
	/* Read the accel Y data*/
//	com_rslt += bma2x2_read_accel_y(&accel_y_s16);
//	*ay = accel_y_s16;
	/* Read the accel Z data*/
//	com_rslt += bma2x2_read_accel_z(&accel_z_s16);
//	*az = accel_z_s16;

	/* accessing the bma2x2acc_data parameter by using sample_xyz*/
	/* Read the accel XYZ data*/
//	com_rslt += bma2x2_read_accel_xyz(&sample_xyz);
//	*xyz = sample_xyz;
	/* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
	/* Read the accel XYZT data*/
	com_rslt += bma2x2_read_accel_xyzt(&sample_xyzt);
	*xyzt = sample_xyzt;
/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "DEEP SUSPEND"
 *	DEEP SUSPEND mode is set from the register 0x11
 *	0x11 -> bit 5 -> set value as 1
 *	the device reaches the lowest power consumption only
 *	interface selection is kept alive
 *	No data acquisition is performed
 *	by using the below API able to set the power mode as DEEPSUSPEND*/
 /* Set the power mode as DEEPSUSPEND*/
//	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return com_rslt;
}

s32 bma2x2_data_readout_gyro(struct bma2x2_t* bma2x2, struct bma2x2_accel_data *gxyz)
{
	/*Local variables for reading accel x, y and z data*/
//	s16	gyro_x_s16, gyro_y_s16, gyro_z_s16 = BMA2x2_INIT_VALUE;

	/* bma2x2acc_data structure used to read accel xyz data*/
//	struct bma2x2_accel_data sample_xyz;
	/* bma2x2acc_data_temp structure used to read
		accel xyz and temperature data*/
//	struct bma2x2_accel_data_temp sample_xyzt;
	/* Local variable used to assign the bandwidth value*/
	u8 bw_value_u8 = BMA2x2_INIT_VALUE;
	/* Local variable used to set the bandwidth value*/
	u8 banwid = BMA2x2_INIT_VALUE;
	/* status of communication*/
	s32 com_rslt = ERROR;


/*********************** START INITIALIZATION ************************
  *	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bma2x2 API*/
	#ifdef BMA2x2_API

	SPI_routine(bma2x2);
	#endif
 /*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
 *-------------------------------------------------------------------------*/
	com_rslt = bma2x2_init(bma2x2);

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	NORMAL mode is set from the register 0x11 and 0x12
 *	0x11 -> bit 5,6,7 -> set value as 0
 *	0x12 -> bit 5,6 -> set value as 0
 *	data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL
 *	For the Normal/standby/Low power 2 mode Idle time
		of at least 2us(micro seconds)
 *	required for read/write operations*/
	/* Set the power mode as NORMAL*/
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
/*	Note:
	* For the Suspend/Low power1 mode Idle time of
		at least 450us(micro seconds)
	* required for read/write operations*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the bandwidth of the sensor input
	value have to be given
	bandwidth is set from the register 0x10 bits from 1 to 4*/
	bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
	com_rslt += bma2x2_set_bw(bw_value_u8);

	/* This API used to read back the written value of bandwidth*/
	com_rslt += bma2x2_get_bw(&banwid);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*---------------------------------------------------------------------*/
	/* Read the gyro X data*/
//	com_rslt += bma2x2_read_accel_x(&gyro_x_s16);
//	*gx = gyro_x_s16;
	/* Read the gyro Y data*/
//	com_rslt += bma2x2_read_accel_y(&gyro_y_s16);
//	*gy = gyro_y_s16;
	/* Read the gyro Z data*/
//	com_rslt += bma2x2_read_accel_z(&gyro_z_s16);
//	*gz = gyro_z_s16;

	/* accessing the bma2x2acc_data parameter by using sample_xyz*/
	/* Read the accel XYZ data*/
	com_rslt += bma2x2_read_accel_xyz(gxyz);

	/* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
	/* Read the accel XYZT data*/
//	com_rslt += bma2x2_read_accel_xyzt(&sample_xyzt);

/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "DEEP SUSPEND"
 *	DEEP SUSPEND mode is set from the register 0x11
 *	0x11 -> bit 5 -> set value as 1
 *	the device reaches the lowest power consumption only
 *	interface selection is kept alive
 *	No data acquisition is performed
 *	by using the below API able to set the power mode as DEEPSUSPEND*/
 /* Set the power mode as DEEPSUSPEND*/
//	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return com_rslt;
}



#ifdef BMA2x2_API

/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bma2x2_t
 *--------------------------------------------------------------------------*/
s8 SPI_routine(struct bma2x2_t  *bma2x2)
{
/*--------------------------------------------------------------------------*
 *  By using bma2x2 the following structure parameter can be accessed
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bma2x2->bus_write = BMA2x2_SPI_bus_write;
	bma2x2->bus_read = BMA2x2_SPI_bus_read;
	bma2x2->delay_msec = BMA2x2_delay_msek;

	return BMA2x2_INIT_VALUE;
}

/************** I2C/SPI buffer length ******/

#define SPI_BUFFER_LEN 5
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE	0x80

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using SPI
*	Use either  SPI based on your need
*
*-----------------------------------------------------------------------*/

static s8 SPI_SendData(uint16_t gpio_pin, uint8_t* data, u8 size, uint32_t timeout)
{

  s32 iError = BMA2x2_INIT_VALUE;
   HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_RESET);
   iError = HAL_SPI_Transmit(&hspi1, data, size, timeout);
   while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
   HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_SET);
   return (s8)iError;
}

static s8 SPI_ReadData(uint16_t gpio_pin, uint8_t* data, u8 size, uint32_t timeout)
{

   s32 iError = BMA2x2_INIT_VALUE;
   HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_RESET);
   iError = HAL_SPI_Receive(&hspi1, data, size, timeout);
   while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
   HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_SET);
   return (s8)iError;
}


/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *          will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *          which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 BMA2x2_SPI_bus_read(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN] = {BMA2x2_INIT_VALUE};
	u8 stringpos;
	memset(array,0,SPI_BUFFER_LEN -1);
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as 0)*/
	array[BMA2x2_INIT_VALUE] = reg_addr|BMA2x2_SPI_BUS_READ_CONTROL_BYTE;
	/*read routine is initiated register address is mask with 0x80*/
	iError = SPI_ReadData(dev_addr, array, cnt+1, HAL_MAX_DELAY);
	/*
	* Please take the below function as your reference for
	* read the data using SPI communication
	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
	* add your SPI read function here
	* iError is an return value of SPI read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done in the SPI read
	* and write string function
	* For more information please refer data sheet SPI communication:
	*/
	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) { //wrong chipID but seems valid gyro data
		*(reg_data + stringpos) = array[stringpos +
		BMA2x2_BUS_READ_WRITE_ARRAY_INDEX];

//	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt + 1; stringpos++) {//wrong gyro data but valid chipID
//		*(reg_data + stringpos) = array[stringpos*2 ] & 0x0f << 4 | array[stringpos*2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] & 0x0f;

	}
	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
*               will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMA2x2_SPI_bus_write(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN * 2];
	u8 stringpos = BMA2x2_INIT_VALUE;

	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done:
		because it ensure the
		0 and 1 of the given value
		It is done only for 8bit operation*/
		array[stringpos * 2] = (reg_addr++) &
		BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE;
		array[stringpos * 2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
		*(reg_data + stringpos);

	}
	/* Please take the below function as your reference
	 * for write the data using SPI communication
	 * add your SPI write function here.
	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	iError = SPI_SendData(dev_addr,array, cnt*2, HAL_MAX_DELAY);

	return (s8)iError;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMA2x2_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	HAL_Delay(msek);
}
#endif


BMA2x2_RETURN_FUNCTION_TYPE bma2x2_spi_init(struct bma2x2_t *bma2x2, uint16_t GPIO_Pin)
{
	  BMA2x2_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	  bma2x2->dev_addr = GPIO_Pin;
	  SPI_routine(bma2x2);
	  com_rslt = bma2x2_init(bma2x2);
	  u8 range = BMA2x2_INIT_VALUE;
	  bma2x2_set_range(BMA2x2_RANGE_4G);
	  bma2x2_get_range(&range);
	  com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
	  return com_rslt;
}
