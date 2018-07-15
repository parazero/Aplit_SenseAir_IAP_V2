/*
 * ms5611_support.c
 *
 *  Created on: 8 баев„ 2017
 *      Author: user
 */

#include "../../Perepheral_drivers/Inc/ms5611.h"
#include "includes.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"
#include "string.h"
#include "types.h"
#include "stdint.h"

#include "types.h"

extern SPI_HandleTypeDef hspi2;

/************** I2C/SPI buffer length ******/

#define SPI_BUFFER_LEN 5
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE	0x80

u8 transfer(u8 byte);

s8 MS5611_SPI_bus_write(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s8 MS5611_SPI_bus_read(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s8 SPI_routine_MS5611(struct ms5611_t* ms5611);

s32 MS5611_data_readout(struct ms5611_t* ms5611);

void MS5611_delay_msek(u32 msek);

u16 MS5611_SPI_bus_readREG16(u16 dev_addr, u8 reg_addr, u16 *reg_data, u8 cnt);
u32 MS5611_SPI_bus_readREG24(u16 dev_addr, u8 reg_addr, u32 *reg_data, u8 cnt);

s8 SPI_SendData_MS(uint16_t gpio_pin, uint8_t* data, u8 size, uint32_t timeout);

s8 SPI_ReadData_MS(uint16_t gpio_pin, uint8_t* data, u8 size, uint32_t timeout);

s8 SPI_routine_MS5611(struct ms5611_t* ms5611) {
	ms5611->bus_write = MS5611_SPI_bus_write;
	ms5611->bus_read = MS5611_SPI_bus_read;
	ms5611->bus_readREG16 = MS5611_SPI_bus_readREG16;
	ms5611->bus_readREG24 = MS5611_SPI_bus_readREG24;
	ms5611->delay_msec = MS5611_delay_msek;
	ms5611->set_filter = set_filter;
	ms5611->update = update;
	ms5611->present = false;
	ms5611->valid = false;
	ms5611->state_baro_counter = 0;
	return MS5611_INIT_VALUE;
}

s8 SPI_SendData_MS(uint16_t gpio_pin, uint8_t* data, u8 size, uint32_t timeout) {
	s32 iError = MS5611_INIT_VALUE;
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_RESET);
	iError = HAL_SPI_Transmit(&hspi2, data, size, timeout);
	while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_SET);
	return (s8) iError;
}

s8 SPI_ReadData_MS(uint16_t gpio_pin, uint8_t* data, u8 size, uint32_t timeout) {

	s32 iError = MS5611_INIT_VALUE;
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_RESET);
	iError = HAL_SPI_Receive(&hspi2, data, size, timeout);
	while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY)
		;
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_SET);
	return (s8) iError;
}

u16 SPI_ReadData_MSREG16(u16 gpio_pin, u8 reg) {
	u8 b0, b1;
	u8 addr = reg;
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_RESET);
	transfer(addr);
	b1 = transfer(0);
	b0 = transfer(0);
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_SET);
	return (((u16) b1 << 8) | b0);

}

u32 SPI_ReadData_MSREG24(u16 gpio_pin, u8* data, u8 size, u32 timeout) {
	u8 b0, b1, b2;
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_RESET);
	transfer(0x00);
	b2 = transfer(0);
	b1 = transfer(0);
	b0 = transfer(0);
	HAL_GPIO_WritePin(GPIOC, gpio_pin, GPIO_PIN_SET);
	return (((u32) b2) << 16) | (((u32) b1) << 8) | (b0);
}

s8 MS5611_SPI_bus_read(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	s32 iError = MS5611_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN] = { 0xFF };
	u8 stringpos;
	memset(array, 0, SPI_BUFFER_LEN - 1);
	/*	For the SPI mode only 7 bits of register addresses are used.
	 The MSB of register address is declared the bit what functionality it is
	 read/write (read as 1/write as 0)*/
	array[MS5611_INIT_VALUE] = reg_addr | BMA2x2_SPI_BUS_READ_CONTROL_BYTE;
	/*read routine is initiated register address is mask with 0x80*/
	iError = SPI_ReadData_MS(dev_addr, array, cnt + 1, HAL_MAX_DELAY);
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
	for (stringpos = MS5611_INIT_VALUE; stringpos < cnt; stringpos++) { //wrong chipID but seems valid gyro data
		*(reg_data + stringpos) = array[stringpos +
		BMA2x2_BUS_READ_WRITE_ARRAY_INDEX];
	}
	return (s8) iError;
}

u16 MS5611_SPI_bus_readREG16(u16 dev_addr, u8 reg_addr, u16 *reg_data, u8 cnt) {
	s32 iError = MS5611_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN] = { MS5611_INIT_VALUE };
//	u8 stringpos;
	memset(array, 0, SPI_BUFFER_LEN - 1);

	array[MS5611_INIT_VALUE] = reg_addr;
	/*read routine is initiated register address is mask with 0x80*/
	u16 Cal_C[7] = { (u16) MS5611_INIT_VALUE };
//	MS561101BA_PROM_READ();
	memset(Cal_C, 0, 7);
	for (int var = 0; var < 8; var++) {
		array[MS5611_INIT_VALUE] = array[MS5611_INIT_VALUE] + var * 2;
//		*(reg_data + var) = SPI_ReadData_MSREG16(dev_addr, &array, cnt + 1,
//				HAL_MAX_DELAY);
	}
	//!!return (u16) Cal_C;
	return (u16) Cal_C[0];        

}

u32 MS5611_SPI_bus_readREG24(u16 dev_addr, u8 reg_addr, u32 *reg_data, u8 cnt) {
	u32 result = *reg_data;
	s32 iError = MS5611_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN] = { MS5611_INIT_VALUE };
	memset(array, 0, SPI_BUFFER_LEN - 1);

	array[MS5611_INIT_VALUE] = reg_addr;
	result = SPI_ReadData_MSREG24(dev_addr, (u8*)&array, cnt + 1, HAL_MAX_DELAY); //!result = SPI_ReadData_MSREG24(dev_addr, &array, cnt + 1, HAL_MAX_DELAY);
	*reg_data = result;
	return result;
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
s8 MS5611_SPI_bus_write(u16 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	s32 iError = MS5611_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN * 2];
	u8 stringpos = MS5611_INIT_VALUE;

	for (stringpos = MS5611_INIT_VALUE; stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done:
		 because it ensure the
		 0 and 1 of the given value
		 It is done only for 8bit operation*/
		array[stringpos * 2] = (reg_addr++) &
		BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE;
		array[stringpos * 2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] = *(reg_data
				+ stringpos);

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
	iError = SPI_SendData_MS(dev_addr, array, cnt * 2, HAL_MAX_DELAY);

	return (s8) iError;
}

void MS5611_delay_msek(u32 msek) {
	/*Here you can write your own delay routine*/
	HAL_Delay(msek);
}

MS5611_RETURN_FUNCTION_TYPE ms5611_spi_init(struct ms5611_t *ms5611,
		uint16_t GPIO_Pin) {

	MS5611_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	ms5611->dev_addr = GPIO_Pin;

	SPI_routine_MS5611(ms5611);
        MS561101BA_Init(ms5611);
	//!!com_rslt = MS561101BA_Init(ms5611);
	//!!return com_rslt;
}

s32 ms5611_data_readout(float *altitude_out, float *air_pressure_out) {
	update();
	*altitude_out = altitude;
	*air_pressure_out = air_pressure;
	return 0;
}
