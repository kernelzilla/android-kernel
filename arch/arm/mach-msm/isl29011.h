/*
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * Motorola 2010-Jan-26 - Initial Creation
 */
 
#ifndef _ISL29011_H
#define _ISL29011_H


#define ISL29011_NAME "isl29011"
#define ISL29011_MODULE_NAME "proximity"


/*ISL29011 register macro define here*/
#define ISL29011_COMMAND_I_REG  0x00
#define ISL29011_COMMAND_II_REG 0x01
#define ISL29011_DATA_LSB_REG   0x02
#define ISL29011_DATA_MSB_REG   0x03
#define ISL29011_INT_LT_LSB_REG 0x04 
#define ISL29011_INT_LT_MSB_REG 0x05
#define ISL29011_INT_HT_LSB_REG 0x06
#define ISL29011_INT_HT_MSB_REG 0x07
/*ISL29011 Register COMMAND macro define*/
#define ISL29011_OP_POWERDOWN  0x00
#define ISL29011_OP_ALS_ONCE   0x20
#define ISL29011_OP_IR_ONCE    0x40
#define ISL29011_OP_PROX_ONCE  0x60
#define ISL29011_OP_ALS_CON    0xa0
#define ISL29011_OP_IR_CON     0xc0
#define ISL29011_OP_PROX_CON   0xe0
#define ISL29011_OP_ALS_PROX   0xff
/*ISL29011 Register command II macro*/
#define ISL29011_RSLT_WIDTH_16 0x00
#define ISL29011_RSLT_WIDTH_12 0x04
#define ISL29011_RSLT_WIDTH_8 0x08
#define ISL29011_RSLT_WIDTH_4 0x0C
#define ISL29011_FSR_WIDTH_1K 0x00
#define ISL29011_FSR_WIDTH_4K 0x01
#define ISL29011_FSR_WIDTH_16K 0x02
#define ISL29011_FSR_WIDTH_64K 0x03
#define ISL29011_TM_INTERNAL  0x00
#define ISL29011_TM_EXTERNAL  0x10
#define ISL29011_INT_PERSIST_1 0x00
#define ISL29011_INT_PERSIST_4 0x01
#define ISL29011_INT_PERSIST_8 0x02
#define ISL29011_INT_PERSIST_16 0x03
#define ISL29011_PROX_SCHEMA_LED_AMB  0x00
#define ISL29011_PROX_SCHEMA_LED_ONLY 0x80
#define ISL29011_IR_CUR_12_5  0x00
#define ISL29011_IR_CUR_25    0x10
#define ISL29011_IR_CUR_50    0x20
#define ISL29011_IR_CUR_100   0x30
#define ISL29011_MODULAR_FREQ_DC  0x00
#define ISL29011_MODULAR_FREQ_327_7 0x40



typedef struct isl29011_para {
	uint8_t  op_mode;
	uint8_t  timing_mode;
	uint8_t  int_persist;
	uint8_t  prox_schema;
	uint8_t  mod_freq;
	uint8_t  ir_curr;
	uint8_t  resl_width;
	uint8_t  fs_range;
	uint16_t low_thres;	
	uint16_t hi_thres;
}isl29011_para_t;

typedef enum
{
    ALS_DARK,
    ALS_INDOOR_WEAK,
    ALS_INDOOR_NORMAL,
    ALS_STRONG,
    ALS_OUTDOOR_NORMAL,
    ALS_LEVEL_END
}als_level_t;



#define LGSMIO				0x29
#define LGS_IOCTL_APP_SET_MODE		_IOW(LGSMIO, 0x1, uint8_t)
#define LGS_IOCTL_APP_SET_LIGHT_RANGE		_IOW(LGSMIO, 0x2, thld_range_t)
#define LGS_IOCTL_APP_SET_PROXIMITY_RANGE		_IOW(LGSMIO, 0x3, thld_range_t)
#define LGS_IOCTL_APP_SET_SWITCH_TIME		_IOW(LGSMIO, 0x4, uint32_t)
#define LGS_IOCTL_APP_GET_SENSOR_DET_DATA   _IOR(LGSMIO, 0x5,  uint16_t)
#define LGS_IOCTL_APP_RESET_PEDOMETER   _IO(LGSMIO, 0x4)
#define LGS_IOCTL_APP_SET_RESOLUTION  _IOW(LGSMIO, 0x6, uint8_t)
#define LGS_IOCTL_APP_SET_WIDTH      _IOW(LGSMIO, 0x7, uint8_t)
#define LGS_IOCTL_APP_SET_IR_CURRENT  _IOW(LGSMIO,0x8, uint8_t)
#define LGS_IOCTL_APP_SET_TIMING_MODE  _IOW(LGSMIO,0x9,uint8_t)
#define LGS_IOCTL_APP_SET_INT_PERSIST  _IOW(LGSMIO,0xA, uint8_t)
#define LGS_IOCTL_APP_SET_PROX_SENS_SCHMEA _IOW(LGSMIO, 0xB, uint8_t)
#define LGS_IOCTL_APP_SET_SCALE_RANGE  _IOW(LGSMIO, 0xC, uint8_t)
#define LGS_IOCTL_APP_SET_ALL_PARA    _IOW(LGSMIO, 0xD, isl29011_para_t)
#define LGS_IOCTL_APP_GET_MODE      _IOR(LGSMIO, 0xE, uint8_t)
#define LGS_IOCTL_APP_SET_FAC_ADC  _IOW(LGSMIO, 0xF, uint16_t)
#endif
