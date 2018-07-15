#ifndef __MS5611_H__
#define __MS5611_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "types.h"
#include "string.h"
#include "types.h"
#include "stdint.h"
#include "includes.h"
// imp_temp_out
#include <stdbool.h>
#include <math.h>

#define  MS5611_INIT_VALUE ((u8)0)
#define  MS561101BA_D1 0x40 
#define  MS561101BA_D2 0x50 
#define  MS561101BA_RST 0x1E 

#define MS5611_CHIP_ID_REG MS5611_CHIP_ID_ADDR
#define MS5611_GEN_READ_WRITE_LENGTH ((u8)1)
#define MS5611_CHIP_ID_ADDR (0x00)
//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
//#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define  MS561101BA_ADC_RD 0x00 
#define  MS561101BA_PROM_RD 0xA0 
#define  MS561101BA_PROM_CRC 0xAE

#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958 //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f

#define ms5611_pmin 20
#define ms5611_pmax 1200
#define ms5611_tmin -40
#define ms5611_tmax 85

//void MS561101BA_PROM_READ(struct ms5611_t *ms5611);
//uint32_t MS561101BA_DO_CONVERSION(uint8_t command);
//void MS561101BA_Init(void);
//void ms5611_update(struct ms5611_t *ms5611);
void ms5611_calculate(void);
void asl_fliter(void);
//void MS561101BA_RESET(struct ms5611_t *ms5611);

#define MS5611_WR_FUNC_PTR s8(*bus_write)\
(u16, u8, u8 *, u8)
#define MS5611_RD_FUNC_PTR s8(*bus_read)\
(u16, u8, u8 *, u8)

//!!
//#define MS5611_RDRG16_FUNC_PTR s8(*bus_readREG16)\
//(u16, u8, u16 *, u8)
//#define MS5611_RDRG24_FUNC_PTR s8(*bus_readREG24)\
//(u16, u8, u32 *, u8)
#define MS5611_RDRG16_FUNC_PTR u16(*bus_readREG16)\
(u16, u8, u16 *, u8)
#define MS5611_RDRG24_FUNC_PTR u32(*bus_readREG24)\
(u16, u8, u32 *, u8)




#define MS5611_MDELAY_DATA_TYPE            u32
#define	MS5611_RETURN_FUNCTION_TYPE        s8

#define MS5611_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)
#define MS5611_BUS_READ_FUNC_REG16(dev_addr, reg_addr, reg_data, r_len)\
bus_readREG16(dev_addr, reg_addr, reg_data, r_len)
#define MS5611_BUS_READ_FUNC_REG24(dev_addr, reg_addr, reg_data, r_len)\
bus_readREG24(dev_addr, reg_addr, reg_data, r_len)
#define MS5611_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
bus_write(dev_addr, reg_addr,reg_data, wr_len)
extern s64 t1;
extern s64 t2;
extern s64 ofs1;
extern s64 ofs2;
extern s64 sens1;
extern s64 sens2;
extern s64 aux;
extern s32 dt;
extern s32 raw_pressure;
extern s32 raw_temperature;
extern float iir_coeff;
extern float iir_diff_coeff;
extern float pressure;
extern float altitude;
extern float abs_neg_hgt;
extern float rel_neg_hgt;
extern float air_pressure;
extern float bias;
extern float init_neg_hgt;
extern u16 c1, c2, c3, c4, c5, c6;
extern s16 temperature;
extern bool pressure_sample;
//bool present;
//bool valid;
extern bool integrity;
extern u8 state_baro_counter;

void set_filter(float iir);
void update(void);
u32 spi_read24(void);

//struct cBARO_MS5611 {
//	bool * _present; // Error happens here
//	bool * _valid;
//	u8 *state_baro_counter;
//	void (*set_filter)(float);
//	void (*update)(void);
//};

//struct cBARO_MS5611 {
//	present=false;valid=false;state_baro_counter=0;
//	void (*set_filter)(float);
//	void (*update)(void);
//};

/*
struct ms5611_t {
	bool present;
	bool valid;
	u8 state_baro_counter;
	void (*set_filter)(float);
	void (*update)(void);
	u8 power_mode_u8;//< save current bma2x2 operation mode 
	u8 chip_id;// chip_id of bma2x2 
	u8 ctrl_mode_reg;// the value of power mode register 0x11
	u8 low_mode_reg;// the value of power mode register 0x12
	u16 dev_addr;//initializes bma2x2's I2C device address
	u8 fifo_config;// store the fifo configuration register
	MS5611_WR_FUNC_PTR;// function pointer to the SPI/I2C write function 
	MS5611_RD_FUNC_PTR;// function pointer to the SPI/I2C read function 
	MS5611_RDRG16_FUNC_PTR;
	MS5611_RDRG24_FUNC_PTR;
//MS5611_BRD_FUNC_PTR;// function pointer to the SPI/I2C burst read function 
	void (*delay_msec)(MS5611_MDELAY_DATA_TYPE);
	// function pointer to a pause in mili seconds functio
};
*/

struct ms5611_t { //!!
//	bool * present;
//	bool * valid;
//	u8 *state_baro_counter;
        bool present;
	bool valid;
	u8 state_baro_counter;
	void (*set_filter)(float);
	void (*update)(void);
	u8 power_mode_u8;/**< save current bma2x2 operation mode */
	u8 chip_id;/**< chip_id of bma2x2 */
	u8 ctrl_mode_reg;/**< the value of power mode register 0x11*/
	u8 low_mode_reg;/**< the value of power mode register 0x12*/
	u16 dev_addr;/**< initializes bma2x2's I2C device address*/
	u8 fifo_config;/**< store the fifo configuration register*/
	MS5611_WR_FUNC_PTR;/**< function pointer to the SPI/I2C write function */
	MS5611_RD_FUNC_PTR;/**< function pointer to the SPI/I2C read function */
	MS5611_RDRG16_FUNC_PTR;
	MS5611_RDRG24_FUNC_PTR;
//MS5611_BRD_FUNC_PTR;/**< function pointer to the SPI/I2C burst read function */
	void (*delay_msec)(MS5611_MDELAY_DATA_TYPE);
	/**< function pointer to a pause in mili seconds function
	 */
};
MS5611_RETURN_FUNCTION_TYPE ms5611_spi_init(struct ms5611_t *ms5611,
		uint16_t GPIO_Pin);
//s32 MS5611_data_readout(struct ms5611_t* ms5611);
s32 ms5611_data_readout(float *altitude_out, float *air_pressure_out);
void MS561101BA_Init(struct ms5611_t *ms5611);

#endif

