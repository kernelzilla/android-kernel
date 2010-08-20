/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#ifdef __cplusplus
extern "C" {
	 
#endif	/*  */
	
#ifndef ATMEL_TS_H
#define ATMEL_TS_H
/*====================================================================================================
					INCLUDE FILES
==================================================================================================*/
#include <linux/ioctl.h>	/* for ioctl macros */
#include <touchpad.h>
	
/*==================================================================================================
					CONSTANTS
==================================================================================================*/
/* elementary defines */
#ifndef FALSE
#define	FALSE	0
#define	TRUE	(!FALSE)
#endif	/*  */
	
/* driver internals defines */
#define ATMEL_I2C_NAME "mXT224"
#define ATMEL_REG_ADDR_SIZE    2
#define ATMEL_NUM_OF_TOUCHES   4
#define ATMEL_READ_DATA_SIZE   8
	
#if 1 //#if defined (TC_TARGET_PRODUCT_sholes_tablet)
#define ATMEL_I2C_ADDR       0x4A	/* XmegaT - OBP */
#define ATMEL_BL_I2C_ADDR    0x24
#else
#define ATMEL_I2C_ADDR    0x11	/* K09 - OBP */
#endif
	
#define ATMEL_READ_WRITE_DELAY 3000
#define ATMEL_MAX_RW_SIZE      48
	
/*! @brief Number of read/write retries before giving up */
#define ATMEL_READ_RETRIES          10
	
/*! @brief Values for positions in the touch key map array */
#define ATMEL_TOUCH_KEY_X_START     0
#define ATMEL_TOUCH_KEY_X_MAX       1
#define ATMEL_TOUCH_KEY_Y_START     2
#define ATMEL_TOUCH_KEY_Y_MAX       3
#define ATMEL_TOUCH_KEY_KEYCODE     4
#define ATMEL_TOUCH_KEY_TOTAL_VAL   5
	
/*! @brief Touch button definitions */
#define ATMEL_NUM_KEYS                        4
#define ATMEL_MSG_NUM_OBPECTS                 4
#define ATMEL_KEY_RELEASE                     0
#define ATMEL_KEY_PRESS                       1
#define ATMEL_NO_BUTTON                       0
#define ATMEL_PROCESS_LIFTOFF                 1
#define ATMEL_DO_NOT_PROCESS_LIFTOFF          0
#define ATMEL_ALLOW                           0
#define ATMEL_IGNORE                          1
	
/*! @brief Haptic parameters */
#define ATMEL_HAPTIC_OFF           0
#define ATMEL_HAPTIC_ON            1
#define ATMEL_USE_TIMER            1
	
/* ***************** OBP ********************** */
#define ATMEL_BOOT_FROM_EEPROM   1
#define ATMEL_INVERT_DATA        1
#define ATMEL_SWAP_DATA          1
	
/* Defines for object information */
#define ATMEL_OBJ_INFO_REG          0x0000
#define ATMEL_OBJ_NUM_OF_INFO_BYTES 0x01
	
/* ! @brief Defines for the Report IDs */
#define OBJ_ID_RESERVED_T0                          0x00
#define OBJ_ID_RESERVED_T1                          0x01
#define OBJ_ID_DEBUG_DELTAS_T2                      0x02
#define OBJ_ID_DEBUG_REFERENCES_T3                  0x03
#define OBJ_ID_DEBUG_SIGNALS_T4                     0x04
#define OBJ_ID_GEN_MESSAGEPROCESSOR_T5              0x05
#define OBJ_ID_GEN_COMMANDPROCESSOR_T6              0x06
#define OBJ_ID_GEN_POWERCONFIG_T7                   0x07
#define OBJ_ID_GEN_ACQUIRECONFIG_T8                 0x08
#define OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9            0x09
#define OBJ_ID_TOUCH_SINGLETOUCHSCREEN_T10          0x0A
#define OBJ_ID_TOUCH_XSLIDER_T11                    0x0B
#define OBJ_ID_TOUCH_YSLIDER_T12                    0x0C
#define OBJ_ID_TOUCH_XWHEEL_T13                     0x0D
#define OBJ_ID_TOUCH_YWHEEL_T14                     0x0E
#define OBJ_ID_TOUCH_KEYARRAY_T15                   0x0F
#define OBJ_ID_PROCG_SIGNALFILTER_T16               0x10
#define OBJ_ID_PROCI_LINEARIZATIONTABLE_T17         0x11
#define OBJ_ID_PROCI_GESTURESPROCESS_T18            0x12
#define OBJ_ID_SPT_GPIOPWM_T19                      0x13
#define OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20        0x14
#define OBJ_ID_RESERVED_T21                         0x15
#define OBJ_ID_PROCG_NOISESUPPRESSION_T22           0x16
#define OBJ_ID_TOUCH_PROXIMITY_T23                  0x17
#define OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24   0x18
#define OBJ_ID_SPT_SELFTEST_T25                     0x19
#define OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27   0x1B
#define OBJ_ID_SPT_CTECONFIG_T28                    0x1C
#define OBJ_ID_CHARGER_NOISE_INT_FIX_T36            0x24
#define OBJ_ID_DEBUG_DIAGNOSTIC_T37                 0x25
#define OBJ_ID_SPT_USERDATA_T38                     0x26
#define ATMEL_INVALID_OBJ_TYPE                      0xFF
	
/*! @brief Mode definition for the panel */
#define	ATMEL_DONT_CHANGE_MODE  -2
#define	ATMEL_STANDBY_T7        (-OBJ_ID_GEN_POWERCONFIG_T7)
#define ATMEL_INITIAL_MODE       OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9
	
#define ATMEL_INITIAL            1
#define ATMEL_STANDBY            2
	
#define ATMEL_REPORT_ID_BYTE  0
	
/* OBJ_ID_GEN_COMMANDPROCESSOR_T6 message bytes */
#define ATMEL_T6_STATUS       1
#define ATMEL_T6_CHKSUM1      2
#define ATMEL_T6_CHKSUM2      3
	
/* OBJ_ID_GEN_COMMANDPROCESSOR_T6 message byte masks */
#define ATMEL_T6_STATUS_RESET   0x80
#define ATMEL_T6_STATUS_OFL     0x40
#define ATMEL_T6_STATUS_SIGERR  0x20
#define ATMEL_T6_STATUS_CAL     0x10
#define ATMEL_T6_STATUS_CFGERR  0x08
	
/* OBP_ID_TOUCH_MULTITOUCHSCREEN_T9 Message bytes */
#define ATMEL_T9_STATUS       1
#define ATMEL_T9_XPOS_MSB     2
#define ATMEL_T9_YPOS_MSB     3
#define ATMEL_T9_XYPOS_LSB    4
#define ATMEL_T9_TCH_AREA     5
#define ATMEL_T9_TCH_AMPLI    6
#define ATMEL_T9_TIMESTAMP    7

/* OBJ_ID_TOUCH_KEYARRAY_T15 Message bytes */
#define ATMEL_T15_STATUS		1
#define ATMEL_T15_KEYSTATE	2
#define ATMEL_T15_KEY_DETECT_MASK 0x80

/* OBP_ID_TOUCH_MULTITOUCHSCREEN_T9 Message byte masks */
#define ATMEL_T9_STATUS_TOUCH_MASK     0x80
#define ATMEL_T9_STATUS_PRESS_MASK     0x40
#define ATMEL_T9_STATUS_RELEASE_MASK   0x20
#define ATMEL_T9_STATUS_MOVE_MASK      0x10
	
#define ATMEL_T9_XYPOS_LSB_YPOS_MASK   0x0F
#define ATMEL_T9_XYPOS_LSB_XPOS_MASK   0xF0
#define ATMEL_T9_XYPOS_LSB_POS_SHIFT   4
	
/* OBJ_ID_TOUCH_SINGLETOUCHSCREEN_T10 message bytes */
#define ATMEL_T10_STATUS       1
#define ATMEL_T10_XPOS_MSB     2
#define ATMEL_T10_YPOS_MSB     3
#define ATMEL_T10_XYPOS_LSB    4
#define ATMEL_T10_TCH_AREA     5
#define ATMEL_T10_TCH_AMPLI    6
#define ATMEL_T10_TIMESTAMP    7
	
/* OBJ_ID_TOUCH_SINGLETOUCHSCREEN_T10 message byte masks */
#define ATMEL_T10_STATUS_TOUCH_MASK     0x80
#define ATMEL_T10_STATUS_PRESS_MASK     0x40
#define ATMEL_T10_STATUS_RELEASE_MASK   0x20
#define ATMEL_T10_STATUS_MOVE_MASK      0x10
	
#define ATMEL_T10_XYPOS_LSB_YPOS_MASK   0x0C
#define ATMEL_T10_XYPOS_LSB_XPOS_MASK   0xC0
#define ATMEL_T10_XYPOS_LSB_XPOS_SHIFT  6
#define ATMEL_T10_XYPOS_LSB_YPOS_SHIFT  2
	
#define ATMEL_MAX_OBJECT_NUM            40
	
#define ATMEL_FW_UPDATE_STATUS_BYTE     0x0
	
/*==================================================================================================
						MACROS
==================================================================================================*/
	
/** @brief This macro provides an interruptible sleep
*/
#define clk_busy_wait(time)	msleep_interruptible(time/1000)
	
/** @brief This macro provides a way to swap X and Y data
*/
#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)
	
/*==================================================================================================
						ENUMS
==================================================================================================*/
/*! @brief  This enum describes Firmware Update completed or not */
typedef enum 
{ 
   ATMEL_FW_UPDATE_NOT_DONE, 
   ATMEL_FW_UPDATE_DONE,
} ATMEL_FW_UPDATE_STATUS_T;
	
/** \brief This enum describes the Flip, Swap and Snap for X, Y data
 */
typedef enum 
{ 
   ATMEL_FLIP_X = 1UL << 0, 
   ATMEL_FLIP_Y = 1UL << 1, 
   ATMEL_SWAP_XY = 1UL << 2, 
   ATMEL_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
} ATMEL_FLIP_SWAP_SNAP_T;
	
/*==================================================================================================
					STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
	
/*! @brief Structure used to define the platform data */
struct atmel_i2c_platform_data 
{
   int (*power) (int on);
   uint32_t flags;
   uint32_t inactive_left;	/* 0x10000 = screen width */
   uint32_t inactive_right;	/* 0x10000 = screen width */
   uint32_t inactive_top;	/* 0x10000 = screen height */
   uint32_t inactive_bottom;	/* 0x10000 = screen height */
   uint32_t fuzz_x;	/* 0x10000 = screen width */
   uint32_t fuzz_y;	/* 0x10000 = screen height */
   int fuzz_p;
   int fuzz_w;
} atmel_platform_data;
	
/*! @brief Structure used to define the touch screen data */
struct atmel_ts_data 
{
   uint16_t addr;
   struct i2c_client *client;
   struct input_dev *input_dev;
   int use_irq;
   struct hrtimer timer;
   struct work_struct work;
   uint16_t max[2];
   int snap_down[2];
   int snap_up[2];
   uint32_t flags;
   int useData;
   int (*power) (int on);
   char mode;
   MOTO_TS_DATA_T touch_data;
   MOT_TOUCH_BL_MODE_STATE_T driver_state;
   ATMEL_FW_UPDATE_STATUS_T FwUpdate_Status;
   int FwVersion;
		
#ifdef CONFIG_HAS_EARLYSUSPEND
   struct early_suspend early_suspend;
		
#endif	/*  */
} atmel_touch_screen_data;
	
/*! @brief Structure used to define the HWCFG table */
typedef struct 
{
   char *var_name;
   void *data;
   unsigned int size;
} ATMEL_HWCFG_TABLE_T;
	
/*! @brief Structure used to define the touch data */
struct atmel_touch_data 
{
   int x_data[ATMEL_NUM_OF_TOUCHES];
   int y_data[ATMEL_NUM_OF_TOUCHES];
   int z_data[ATMEL_NUM_OF_TOUCHES];
   int a_data[ATMEL_NUM_OF_TOUCHES];
   int touch_area[ATMEL_NUM_OF_TOUCHES];
   int button_area[ATMEL_NUM_OF_TOUCHES];
   int ignore_touch[ATMEL_NUM_OF_TOUCHES];
   int ignore_button[ATMEL_NUM_OF_TOUCHES];
} atmel_touch_data;
	
/*! @brief Structure variable for the I2C client */
//struct i2c_client Atmel_ts_client;
	
/*! @brief Assigned value for a UINT32 bitmask */
typedef unsigned int ATMEL_KEY_BITMASK_ENTRY_T;
	
/* OBJ_ID_GEN_COMMANDPROCESSOR_T6 memory config, size=5 bytes */
#define ATMEL_T6_RESET        0
#define ATMEL_T6_BACKUPNV     1
#define ATMEL_T6_CALIBRATE    2
#define ATMEL_T6_REPORTALL    3
#define ATMEL_T6_DBGCTRL      4
#define ATMEL_T6_MEMCFG_SIZE  5
	
#define ATMEL_T6_BACKUP_CMD   0x55

typedef struct 
{
   uint8_t reset;
   uint8_t backup_nv;
   uint8_t calibrate;
   uint8_t reportall;
   uint8_t dbg_ctrl;
} OBJ_T6_MEM_CFG_T, *OBJ_T6_MEM_CFG_PTR;
	
/* OBJ_ID_GEN_POWERCONFIG_T7 memory config, size=3 bytes */
#define ATMEL_T7_IDLE_ACQ_INT   0
#define ATMEL_T7_ACTV_ACQ_INT   1
#define ATMEL_T7_ACTV_2_IDLE_TO 2

typedef struct 
{
   uint8_t idle_acq_int;
   uint8_t actv_acq_int;
   uint8_t actv_2_idle_to;
} OBJ_T7_MEM_CFG_T, *OBJ_T7_MEM_CFG_PTR;
	
/* OBJ_ID_GEN_ACQUIRECONFIG_T8 memory config, size=6 bytes */
#define ATMEL_T8_CHRGTIME     0
#define ATMEL_T8_ATCHDRIFT    1
#define ATMEL_T8_TCHDRIFT     2
#define ATMEL_T8_DRIFTST      3
#define ATMEL_T8_TCHAUTOCAL   4
#define ATMEL_T8_SYNC         5
#define ATMEL_T8_MEMCFG_SIZE  6

typedef struct 
{
   uint8_t chrg_time;
   uint8_t atch_drift;
   uint8_t tch_drift;
   uint8_t drift_state;
   uint8_t tch_autocal;
   uint8_t sync;
} OBJ_T8_MEM_CFG_T, *OBJ_T8_MEM_CFG_PTR;
	
/* OBP_ID_TOUCH_MULTITOUCHSCREEN_T9 memory configuration, size=17 bytes */
#define ATMEL_T9_CTRL         0
#define ATMEL_T9_XORIGIN      1
#define ATMEL_T9_YORIGIN      2
#define ATMEL_T9_XSIZE        3
#define ATMEL_T9_YSIZE        4
#define ATMEL_T9_AKSCFG       5
#define ATMEL_T9_BLEN         6
#define ATMEL_T9_TCHTHR       7
#define ATMEL_T9_TCHDI        8
#define ATMEL_T9_RESERVE1     9
#define ATMEL_T9_RESERVE2     10
#define ATMEL_T9_MOVHYSTI     11
#define ATMEL_T9_MOVHYSTN     12
#define ATMEL_T9_MOVFILTER    13
#define ATMEL_T9_NUMTOUCH     14
#define ATMEL_T9_MRGHYST      15
#define ATMEL_T9_MRGTHR       16
#define ATMEL_T9_MEMCFG_SIZE  17

typedef struct 
{
   uint8_t ctrl;
   uint8_t xorigin;
   uint8_t yorigin;
   uint8_t xsize;
   uint8_t ysize;
   uint8_t aks_cfg;
   uint8_t blen;
   uint8_t tch_thr;
   uint8_t tch_di;
   uint16_t reserve;
   uint8_t mov_hyst_init;
   uint8_t mov_hyst_next;
   uint8_t mov_filter;
   uint8_t num_touch;
   uint8_t mrg_hyst;
   uint8_t mrg_thr;
} OBJ_T9_MEM_CFG_T, *OBJ_T9_MEM_CFG_PTR;
	
/* OBP_ID_TOUCH_MULTITOUCHSCREEN_T10 memory configuration, size=14 bytes */
#define ATMEL_T10_CTRL         0
#define ATMEL_T10_XORIGIN      1
#define ATMEL_T10_YORIGIN      2
#define ATMEL_T10_XSIZE        3
#define ATMEL_T10_YSIZE        4
#define ATMEL_T10_AKSCFG       5
#define ATMEL_T10_BLEN         6
#define ATMEL_T10_TCHTHR       7
#define ATMEL_T10_TCHDI        8
#define ATMEL_T10_RESERVE1     9
#define ATMEL_T10_RESERVE2     10
#define ATMEL_T10_MOVHYSTI     11
#define ATMEL_T10_MOVHYSTN     12
#define ATMEL_T10_MOVFILTER    13
#define ATMEL_T10_MEMCFG_SIZE  14

typedef struct 
{
   uint8_t ctrl;
   uint8_t xorigin;
   uint8_t yorigin;
   uint8_t xsize;
   uint8_t ysize;
   uint8_t aks_cfg;
   uint8_t blen;
   uint8_t tch_thr;
   uint8_t tch_di;
   uint16_t reserve;
   uint8_t mov_hyst_init;
   uint8_t mov_hyst_next;
   uint8_t mov_filter;
} OBJ_T10_MEM_CFG_T, *OBJ_T10_MEM_CFG_PTR;
	
/* OBJ_ID_PROCG_SIGNALFILTER_T16 memory configuration, size=3 bytes */
#define ATMEL_T16_SLEW        0
#define ATMEL_T16_MED         1
#define ATMEL_T16_IIR         2
#define ATMEL_T16_MEMCFG_SIZE 3

typedef struct 
{
   uint8_t slew;
   uint8_t median;
   uint8_t iir;
} OBJ_T16_MEM_CFG_T, *OBJ_T16_MEM_CFG_PTR;
	
/* OBJ_ID_PROCI_LINEARIZATIONTABLE_T17 memory configuration, size=37 bytes */
#define ATMEL_T17_CTRL        1
#define ATMEL_T17_XOFFSET     2
#define ATMEL_T17_XSEGMENT    4
#define ATMEL_T17_YOFFSET     20
#define ATMEL_T17_YSEGMENT    22
#define ATMEL_T17_MEMCFG_SIZE 37
#define ATMEL_T17_XSEGMENT_SIZE 16
#define ATMEL_T17_YSEGMENT_SIZE 16

typedef struct 
{
   uint8_t ctrl;
   uint16_t xoffset;
   uint8_t xsegment[ATMEL_T17_XSEGMENT_SIZE];
   uint16_t yoffset;
   uint8_t ysegment[ATMEL_T17_YSEGMENT_SIZE];
} OBJ_T17_MEM_CFG_T, *OBJ_T17_MEM_CFG_PTR;
	
/* OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20 memory configuration, size=11 bytes */
#define ATMEL_T20_CTRL        1
#define ATMEL_T20_XLOGRIP     2
#define ATMEL_T20_XHIGRIP     3
#define ATMEL_T20_YLOGRIP     4
#define ATMEL_T20_YHIGRIP     5
#define ATMEL_T20_MAXTCHS     6
#define ATMEL_T20_RESERVE0    7
#define ATMEL_T20_SZTHR1      8
#define ATMEL_T20_SZTHR2      9
#define ATMEL_T20_SHPTHR1     10
#define ATMEL_T20_SHPTHR2     11

typedef struct {
   uint8_t ctrl;
   uint8_t xlogrip;
   uint8_t xhigrip;
   uint8_t ylogrip;
   uint8_t yhigrip;
   uint8_t maxtchs;
   uint8_t reserve0;
   uint8_t szthr1;
   uint8_t szthr2;
   uint8_t shpthr1;
   uint8_t shpthr2;
} OBJ_T20_MEM_CFG_T, *OBJ_T20_MEM_CFG_PTR;
	
#define  ATMEL_OBJ_TBL_TYPE           0
#define  ATMEL_OBJ_TBL_START_POS1     1
#define  ATMEL_OBJ_TBL_START_POS2     2
#define  ATMEL_OBJ_TBL_OBJ_SIZE       3
#define  ATMEL_OBJ_TBL_NUM_INST       4
#define  ATMEL_OBJ_TBL_NUM_REPORTID   5
#define  ATMEL_OBJ_TBL_BLK_SIZE       6
/*! @brief Structure */
typedef struct 
{
   uint8_t type;
   uint16_t start_pos;
   uint8_t obj_size;
   uint8_t num_of_instances;
   uint8_t num_of_report_ids;
   uint8_t report_id_low;
   uint8_t report_id_high;
} ATMEL_OBJ_TABLE_ELEMENT_T, *ATMEL_OBJ_TABLE_ELEMENT_PTR;
	
#define  ATMEL_INFO_BLOCK_FAMILY_ID           0
#define  ATMEL_INFO_BLOCK_VARIANT_ID          1
#define  ATMEL_INFO_BLOCK_VERSION             2
#define  ATMEL_INFO_BLOCK_BUILD               3
#define  ATMEL_INFO_BLOCK_MATRIX_XSIZE        4
#define  ATMEL_INFO_BLOCK_MATRIX_YSIZE        5
#define  ATMEL_INFO_BLOCK_NUM_OF_OBJECT       6
	
#define  ATMEL_INFO_BLOCK_SIZE                7

typedef union 
{
   struct 
   {
      uint8_t family_id;
      uint8_t variant_id;
      uint8_t version;
      uint8_t build;
      uint8_t matrix_x_size;
      uint8_t matrix_y_size;
      uint8_t num_of_objects;
   } atmel_info_bytes;
   unsigned char atmel_info_buf[ATMEL_INFO_BLOCK_SIZE];
} U_ATMEL_INFO_BYTES_T, *U_ATMEL_INFO_BYTES_PTR;

typedef struct 
{
   uint8_t obj_id_cfg_size[ATMEL_MAX_OBJECT_NUM];
   uint8_t obj_cfgs[ATMEL_MAX_OBJECT_NUM];
   uint8_t * object[ATMEL_MAX_OBJECT_NUM];
} OBJ_ID_T, *OBJ_ID_PTR;
	
/*==================================================================================================
					GLOBAL VARIABLE DECLARATION
==================================================================================================*/
	
/*==================================================================================================
					FUNCTION PROTOTYPES
==================================================================================================*/
	
/*================================================================================================*/
#endif	/* ATMEL_TS_H */
#ifdef __cplusplus
}
#endif	/*  */

