#include "../../Perepheral_drivers/Inc/ms5611.h"
#include "includes.h"
#include <stdbool.h>
extern SPI_HandleTypeDef hspi2;

s64 t1;
s64 t2;
s64 ofs1;
s64 ofs2;
s64 sens1;
s64 sens2;
s64 aux;
s32 dt;
s32 raw_pressure;
s32 raw_temperature;
float iir_coeff;
float iir_diff_coeff;
float pressure;
extern float altitude;
float abs_neg_hgt;
float rel_neg_hgt;
float air_pressure;
float bias;
float init_neg_hgt;
u16 c1, c2, c3, c4, c5, c6;
s16 temperature;
bool pressure_sample;
//bool present;
//bool valid;
bool integrity;
u8 state_baro_counter;




//float baro_buffer[10];
//#define baro_buffer_num   10
u8 exchange_Pres_num[8];
u8 exchange_Temp_num[8];

//cBARO_MS5611 g_baro;
//struct ms5611_t* p_ms5611;
u16 Cal_C[8];
u32 D1_Pres = 8442380, D2_Temp = 0;
float Pressure = 0;
float dT = 0, Temperature = 0, Temperature2 = 0;
double OFF = 0, SENS = 0;

static float aslRaw = 0;
static float aslAlpha = 0.95;

float aslZero = 0;
float asl = 0;
float vspeedAsl = 0;

float altitude = 0;

static struct ms5611_t *p_ms5611;

#define ERROR -1
#define SUCCESS 0

#define CMD_MS5611_ROM_C1 (0xA2)
#define CMD_MS5611_ROM_C2 (0xA4)
#define CMD_MS5611_ROM_C3 (0xA6)
#define CMD_MS5611_ROM_C4 (0xA8)
#define CMD_MS5611_ROM_C5 (0xAA)
#define CMD_MS5611_ROM_C6 (0xAC)
#define CMD_MS5611_ROM_CRC (0xAE)
#define CMD_MS5611_CONV_D1 (0x48)
#define CMD_MS5611_CONV_D2 (0x58)

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdbool.h>
extern SPI_HandleTypeDef hspi2;

// Constant values for height calculations

#define RHO_15 (1.225f)
#define MF_G (9.80665f)
#define P_0 (1013.25f)
#define CMD_MS5611_RESET (0x1E)
#define CMD_MS5611_ROM_SETUP (0xA0)
#define CMD_MS5611_ROM_C1 (0xA2)
#define CMD_MS5611_ROM_C2 (0xA4)
#define CMD_MS5611_ROM_C3 (0xA6)
#define CMD_MS5611_ROM_C4 (0xA8)
#define CMD_MS5611_ROM_C5 (0xAA)
#define CMD_MS5611_ROM_C6 (0xAC)
#define CMD_MS5611_ROM_CRC (0xAE)
#define CMD_MS5611_CONV_D1 (0x48)
#define CMD_MS5611_CONV_D2 (0x58)
#define EXTRA_PRECISION (5)

//void MS611_Write(uint8_t reg)
//{
////	ms5611_cs = 0;
//	SPI2_ReadWriteByte(reg);
////	ms5611_cs = 1;
//}

//uint8_t SPI2_ReadWriteByte(uint8_t TxData)
//{
//	uint8_t retry=0;
//	uint8_t temp;
//
//	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
//	{
//		retry++;
//		if(retry>200) return 0;
//	}
//	SPI_SendData_MS(SPI2, TxData);
//	retry=0;
//
//
//
//	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
//	{
//		retry++;
//		if(retry>200) return 0;
//	}
//
//	temp = SPI_I2S_ReceiveData(SPI2);
//
//	return temp;
//}

void set_filter(float iir) {
	iir_coeff = iir;
	iir_diff_coeff = 1.0f - iir_coeff;
}



static s8 SPI_SendData_MS(uint16_t gpio_pin, uint8_t* data, u8 size,
		uint32_t timeout) {
	s32 iError = 0;
	HAL_GPIO_WritePin(GPIOB, gpio_pin, GPIO_PIN_RESET);
	iError = HAL_SPI_Transmit(&hspi2, data, size, timeout);
	while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY)
		;
	HAL_GPIO_WritePin(GPIOB, gpio_pin, GPIO_PIN_SET);
	return (s8) iError;
}

static s8 SPI_ReadData_MS(u16 gpio_pin, u8* data, u8 size, uint32_t timeout) {
	s32 iError = 0;
	HAL_GPIO_WritePin(GPIOB, gpio_pin, GPIO_PIN_RESET);
	iError = HAL_SPI_Receive(&hspi2, data, size, timeout);
	while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY)
		;
	HAL_GPIO_WritePin(GPIOB, gpio_pin, GPIO_PIN_SET);
	return (s8) iError;
}

//uint16_t MS611_Read_16bit(uint8_t reg)
//{
//	uint8_t temp1, temp2;
//	uint16_t value = 0;
//
//	ms5611_cs = 0;
//	SPI2_ReadWriteByte(reg);
//	temp1 = SPI2_ReadWriteByte(0x00);
//	temp2 = SPI2_ReadWriteByte(0x00);
//	ms5611_cs = 1;
//
//	value = (temp1<<8) + temp2;
//
//	return value;
//}

//uint32_t MS611_Read_24bit(uint8_t reg)
//{
//	uint8_t temp1, temp2, temp3;
//	uint32_t value = 0;
//
//	ms5611_cs = 0;
//	SPI2_ReadWriteByte(reg);
//	temp1 = SPI2_ReadWriteByte(0x00);
//	temp2 = SPI2_ReadWriteByte(0x00);
//	temp3 = SPI2_ReadWriteByte(0x00);
//	ms5611_cs = 1;
//
//	value = (temp1<<16) + (temp2<<8) + temp3;
//
//	return value;
//}

//void MS561101BA_READ(struct ms5611_t *ms5611) {
//	u16 data_u16 = MS5611_INIT_VALUE;
//	u8 data_u8 = MS5611_INIT_VALUE;
//	ms5611->MS5611_BUS_READ_FUNC_REG16
//	(ms5611->dev_addr,
//			0xA2, &data_u16, MS5611_GEN_READ_WRITE_LENGTH);
//	HAL_Delay(5);
//	ms5611->MS5611_BUS_WRITE_FUNC
//	(ms5611->dev_addr,
//			CMD_MS5611_CONV_D2, &data_u8, MS5611_GEN_READ_WRITE_LENGTH);
//	HAL_Delay(11);
//
//	for (int i = 0; i < 50; i++) {
//		ms5611_update(ms5611);
//		HAL_Delay(11);
//	}
//
//	ms5611_calculate();
//}

//#define ms5611_updatemode 0

void update(void) {
	if (pressure_sample) {
		raw_pressure = spi_read24();
		pressure = (float) (((double) raw_pressure * (double) sens1 / 2097152.0
				- (double) ofs1) / 32768.0 / 100.0);
		air_pressure = air_pressure * iir_coeff + pressure * iir_diff_coeff;
		abs_neg_hgt = 100.0f * P_0 / (RHO_15 * MF_G) * logf(air_pressure / P_0);
		rel_neg_hgt = abs_neg_hgt - init_neg_hgt;
		altitude = -rel_neg_hgt;
		p_ms5611->valid = (pressure > 450.0f) && (pressure < 1200.0f);
		pressure_sample = false;
		u8 data_u8 = 0;
		p_ms5611->MS5611_BUS_WRITE_FUNC (p_ms5611->dev_addr, CMD_MS5611_CONV_D2, &data_u8, MS5611_GEN_READ_WRITE_LENGTH);
	} else {
		raw_temperature = spi_read24();
		dt = raw_temperature - ((long) c5 * 256);
		t1 = 2000 + (s64) dt * (s64) c6 / 8388608;
		ofs1 = (s64) c2 * 65536 + ((s64) c4 * (s64) dt) / 128;
		sens1 = (s64) c1 * 32768 + ((s64) c3 * (s64) dt) / 256;
		if (t1 < 2000) {
			t2 = (((s64) dt) * dt) >> 31;
			aux = (t1 - 2000) * (t1 - 2000);
			ofs2 = (5 * aux) >> 1;
			sens2 = (5 * aux) >> 2;
			t1 = t1 - t2;
			ofs1 = ofs1 - ofs2;
			sens1 = sens1 - sens2;
		}
		temperature = (s16) t1;
		pressure_sample = true;
		u8 data_u8 = 0;
		p_ms5611->MS5611_BUS_WRITE_FUNC (p_ms5611->dev_addr, CMD_MS5611_CONV_D1, &data_u8, MS5611_GEN_READ_WRITE_LENGTH);
	}
}

void asl_fliter(void) {
	asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
	altitude = asl - aslZero;
}

void MS561101BA_RESET(struct ms5611_t *ms5611) {
	u8 data_u8 = MS5611_INIT_VALUE;
	ms5611->MS5611_BUS_WRITE_FUNC
	(ms5611->dev_addr,
			MS561101BA_RST, &data_u8, MS5611_GEN_READ_WRITE_LENGTH);
}

#define SPI_BUFFER_LEN 5

u8 transfer(u8 byte) {
	hspi2.Instance->DR = byte;
	while (!(hspi2.Instance->SR & 1)) {
	}
	return ((u8) hspi2.Instance->DR);
}

u32 spi_read24(void)
{
  u8 b0,b1,b2;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//  GPIO_13_CLR(GPIOC);
  transfer(0x00);
  b2=transfer(0);
  b1=transfer(0);
  b0=transfer(0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//  GPIO_13_SET(GPIOC);
  return(((u32)b2)<<16)|(((u32)b1)<<8)|(b0);
}
u16 SPI_ReadData_MSREG16(u16 gpio_pin, u8 reg);

void MS561101BA_PROM_READ(struct ms5611_t *ms5611) {  // Read Calibration Data C
	c1 = SPI_ReadData_MSREG16(ms5611->dev_addr, CMD_MS5611_ROM_C1);
	c2 = SPI_ReadData_MSREG16(ms5611->dev_addr, CMD_MS5611_ROM_C2);
	c3 = SPI_ReadData_MSREG16(ms5611->dev_addr, CMD_MS5611_ROM_C3);
	c4 = SPI_ReadData_MSREG16(ms5611->dev_addr, CMD_MS5611_ROM_C4);
	c5 = SPI_ReadData_MSREG16(ms5611->dev_addr, CMD_MS5611_ROM_C5);
	c6 = SPI_ReadData_MSREG16(ms5611->dev_addr, CMD_MS5611_ROM_C6);
}

//
//MS5611_RETURN_FUNCTION_TYPE MS561101BA_Init(struct ms5611_t* ms5611)
//{
//	/*  Variable used to return value of
//		communication routine*/
//		MS5611_RETURN_FUNCTION_TYPE com_rslt = ERROR;
//		u8 data_u8 = MS5611_INIT_VALUE;
//		u8 config_data_u8 = MS5611_INIT_VALUE;
//		/* assign bma2x2 ptr */
//		p_ms5611 = ms5611;
//		/* read Chip Id */
//		com_rslt = p_ms5611->MS5611_BUS_READ_FUNC
//		(p_ms5611->dev_addr,
//		MS5611_CHIP_ID_REG, &data_u8, MS5611_GEN_READ_WRITE_LENGTH);
//		p_ms5611->chip_id = data_u8;    /* get bit slice */
//		/* read the fifo config register and update
//		the value to the fifo_config*/
////		com_rslt += bma2x2_read_reg(BMA2x2_FIFO_MODE_REG,
////		&config_data_u8, BMA2x2_GEN_READ_WRITE_LENGTH);
////		p_bma2x2->fifo_config = config_data_u8;
//		return com_rslt;
//}

void MS561101BA_Init(struct ms5611_t *ms5611) {

	p_ms5611 = ms5611;
	MS561101BA_RESET(p_ms5611);

	HAL_Delay(10); //must wait for 2.8ms, I set 5ms, enough.
	MS561101BA_PROM_READ(p_ms5611);
	HAL_Delay(5);
	u8 data_u8 = 0;
	p_ms5611->MS5611_BUS_WRITE_FUNC (p_ms5611->dev_addr, CMD_MS5611_CONV_D2, &data_u8, MS5611_GEN_READ_WRITE_LENGTH);
	pressure_sample = false;
	pressure = 0.0f;
	temperature = 0;
	altitude = 0.0f;
	p_ms5611->present = ((c1 > 0x0000) && (c1 < 0xFFFF));
	if (p_ms5611->present) {
		p_ms5611->set_filter(0.0f);
		for (u8 i = 0; i < 13; i++) {
			HAL_Delay(25);
			p_ms5611->update();
		}
		init_neg_hgt = abs_neg_hgt;
		HAL_Delay(25);
		p_ms5611->update();
	}
}

