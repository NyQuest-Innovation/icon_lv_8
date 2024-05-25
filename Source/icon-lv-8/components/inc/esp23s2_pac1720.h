#ifndef _esp32s2_pac1720_h_
#define _esp32s2_pac1720_h_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"

#define PAC1720_ADDR 0x52

#define PAC1720_CH1_V_DENO 		2047.0
#define PAC1720_CH2_V_DENO 		2047.0

#define PAC1720_CH1_I_DENO 		2047.0
#define PAC1720_CH2_I_DENO 		2047.0

#define PAC1720_CH1_FSC		90.28   // ((IBUS * PAC1720_CH1_I_DENO) / VSENSE)
#define PAC1720_CH2_FSC		22.75	// ((IBUS * PAC1720_CH2_I_DENO) / VSENSE)

#define PAC1720_CH1_FSV		(40.0 - (40.0 / PAC1720_CH1_V_DENO))
#define PAC1720_CH2_FSV		(40.0 - (40.0 / PAC1720_CH2_V_DENO))

#if 1
    #define PAC1720_CH1_V_CONST		(PAC1720_CH1_FSV / PAC1720_CH1_V_DENO)
    #define PAC1720_CH2_V_CONST		(PAC1720_CH2_FSV / PAC1720_CH2_V_DENO)

    #define PAC1720_CH1_I_CONST		(PAC1720_CH1_FSC / PAC1720_CH1_I_DENO)
    #define PAC1720_CH2_I_CONST		(PAC1720_CH2_FSC / PAC1720_CH2_I_DENO)

    #define PAC1720_CH1_P_CONST		((PAC1720_CH1_FSV * PAC1720_CH1_FSC) / 65535)
    #define PAC1720_CH2_P_CONST		((PAC1720_CH2_FSV * PAC1720_CH2_FSC) / 65535)
#endif

#define PAC1720_CONFIG_ADDR					0x00
#define PAC1720_CONVERSION_RATE_ADDR		0x01
#define PAC1720_ONE_SHOT_ADDR				0x02
#define PAC1720_CH_MASK_REG_ADDR			0x03
#define PAC1720_HIGH_LIMIT_STATUS_ADDR		0x04
#define PAC1720_LOW_LIMIT_STATUS_ADDR		0x05

#define PAC1720_V_SOURCE_SAMP_CONFIG_ADDR	0x0A

#define PAC1720_CH1_VSENSE_SAMP_CONFIG_ADDR	0x0B
#define PAC1720_CH2_VSENSE_SAMP_CONFIG_ADDR	0x0C

#define PAC1720_CH1_VSENSE_HIGH_ADDR		0x0D
#define PAC1720_CH1_VSENSE_LOW_ADDR			0x0E
#define PAC1720_CH2_VSENSE_HIGH_ADDR		0x0F
#define PAC1720_CH2_VSENSE_LOW_ADDR			0x10

#define PAC1720_CH1_VSOURCE_HIGH_ADDR		0x11
#define PAC1720_CH1_VSOURCE_LOW_ADDR		0x12
#define PAC1720_CH2_VSOURCE_HIGH_ADDR		0x13
#define PAC1720_CH2_VSOURCE_LOW_ADDR		0x14

#define PAC1720_CH1_PWR_RAT_HIGH_ADDR		0x15
#define PAC1720_CH1_PWR_RAT_LOW_ADDR		0x16
#define PAC1720_CH2_PWR_RAT_HIGH_ADDR		0x17
#define PAC1720_CH2_PWR_RAT_LOW_ADDR		0x18

#define PAC1720_CH1_VSENSE_LIMIT_HIGH_ADDR	0x19
#define PAC1720_CH2_VSENSE_LIMIT_HIGH_ADDR	0x1A
#define PAC1720_CH1_VSENSE_LIMIT_LOW_ADDR	0x1B
#define PAC1720_CH2_VSENSE_LIMIT_LOW_ADDR	0x1C

#define PAC1720_CH1_VSOURCE_LIMIT_HIGH_ADDR	0x1D
#define PAC1720_CH2_VSOURCE_LIMIT_HIGH_ADDR	0x1E	
#define PAC1720_CH1_VSOURCE_LIMIT_LOW_ADDR	0x1F	
#define PAC1720_CH2_VSOURCE_LIMIT_LOW_ADDR	0x20	

#define PAC1720_PRODUCT_ID_ADDR				0xFD
#define PAC1720_MANUFACTURER_ID_ADDR		0xFE
#define PAC1720_REVISION_ADDR				0xFF



#define PAC1720_UPDATE_1_PER_SEC			(0x00)
#define PAC1720_UPDATE_2_PER_SEC			(0x01)
#define PAC1720_UPDATE_4_PER_SEC			(0x02)
#define PAC1720_UPDATE_CONT					(0x03)  // Default

#define PAC1720_CH2_VSOURCE_SAMP_2_5MS		(0x00 << 6)
#define PAC1720_CH2_VSOURCE_SAMP_5MS		(0x01 << 6)
#define PAC1720_CH2_VSOURCE_SAMP_10MS		(0x02 << 6) // Default
#define PAC1720_CH2_VSOURCE_SAMP_20MS		(0x03 << 6)

#define PAC1720_CH2_VSOURCE_AVG_DIS			(0x00 << 4) //Default
#define PAC1720_CH2_VSOURCE_AVG_2			(0x01 << 4)
#define PAC1720_CH2_VSOURCE_AVG_4			(0x02 << 4)
#define PAC1720_CH2_VSOURCE_AVG_8			(0x03 << 4)

#define PAC1720_CH1_VSOURCE_SAMP_2_5MS		(0x00 << 2)
#define PAC1720_CH1_VSOURCE_SAMP_5MS		(0x01 << 2)
#define PAC1720_CH1_VSOURCE_SAMP_10MS		(0x02 << 2) // Default
#define PAC1720_CH1_VSOURCE_SAMP_20MS		(0x03 << 2)

#define PAC1720_CH1_VSOURCE_AVG_DIS			(0x00) //Default
#define PAC1720_CH1_VSOURCE_AVG_2			(0x01)
#define PAC1720_CH1_VSOURCE_AVG_4			(0x02)
#define PAC1720_CH1_VSOURCE_AVG_8			(0x03)

#define PAC1720_VSENSE_SAMP_2_5MS			(0x00 << 4)
#define PAC1720_VSENSE_SAMP_5MS				(0x01 << 4)
#define PAC1720_VSENSE_SAMP_10MS			(0x02 << 4)
#define PAC1720_VSENSE_SAMP_20MS			(0x03 << 4)
#define PAC1720_VSENSE_SAMP_40MS			(0x04 << 4)
#define PAC1720_VSENSE_SAMP_80MS			(0x05 << 4) //Default
#define PAC1720_VSENSE_SAMP_160MS			(0x06 << 4)
#define PAC1720_VSENSE_SAMP_320MS			(0x07 << 4)

#define PAC1720_VSENSE_AVG_DIS			(0x00 << 2) //Default
#define PAC1720_VSENSE_AVG_2			(0x01 << 2)
#define PAC1720_VSENSE_AVG_4			(0x02 << 2)
#define PAC1720_VSENSE_AVG_8			(0x03 << 2)

#define PAC1720_VSENSE_RANGE_10MV		(0x00)
#define PAC1720_VSENSE_RANGE_20MV		(0x01)
#define PAC1720_VSENSE_RANGE_40MV		(0x02)
#define PAC1720_VSENSE_RANGE_80MV		(0x04) //Default


extern esp_err_t pac1720_busy();
extern void init_pac1720();
extern int8_t pac1720_write_reg16(uint8_t reg_addr, uint16_t reg_val);
extern int8_t pac1720_write_reg8(uint8_t reg_addr, uint8_t reg_val);
extern int8_t pac1720_read_reg16(uint8_t reg_addr, uint16_t *_reg_val);
extern int8_t pac1720_read_reg8(uint8_t reg_addr, uint8_t *_reg_val);
extern int8_t pac720_read_buff(uint16_t reg_addr, uint8_t *reg_buff, uint8_t reg_len);

#endif