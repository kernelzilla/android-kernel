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

#define KEY08_IOCTL_SET_IN_CALL		    1
#define KEY08_IOCTL_SET_NOT_IN_CALL	    2
#define KEY08_IOCTL_SET_BOOTLOADER_MODE	    3
#define KEY08_IOCTL_SET_NORMAL_MODE	    4
#define KEY08_IOCTL_GET_VERSION		    5
#define KEY08_IOCTL_WRITE_REGISTER	    6
#define KEY08_IOCTL_READ_REGISTER	    7
#define KEY08_IOCTL_DL_GET_STATUS	    8
#define KEY08_IOCTL_GET_MODE		    9
#define KEY08_IOCTL_END_FIRMWARE	    10
#define KEY08_IOCTL_DISPLAY_ONE_POINT	    11
#define KEY08_IOCTL_RESET		    12
#define KEY08_IOCTL_DEBUG		    13
#define KEY08_IOCTL_CALIBRATE		    14
#define KEY08_IOCTL_SETIRQ		    15
#define KEY08_IOCTL_DISABLE		    16
#define KEY08_IOCTL_ENABLE	      	    17
#define KEY08_IOCTL_READ_REGISTER16         18
#define KEY08_IOCTL_GETIRQ                  19
#define KEY08_IOCTL_GET_CMD_STATUS          20
#define KEY08_IOCTL_SET_CMD_STATUS          21
#define KEY08_IOCTL_READ_4_REGS             22
#define KEY08_IOCTL_SUSPEND                 23
#define KEY08_IOCTL_GET_POINT               24


/* OBP Specific */
#define KEY08_IOCTL_SET_MESSAGE_PTR         40
#define KEY08_IOCTL_GET_TOUCH_SENS_DATA     41
#define KEY08_IOCTL_SAVE_CONFIG             42
#define KEY08_IOCTL_CAL_STATUS              43
#define KEY08_IOCTL_GET_BAD_CRC_COUNT       44

#define OFF	FALSE
#define	ON	(!OFF)

/* Bootloader statuses */
#define	KEY08_BL_WAITING_FOR_NOTHING	    0
#define	KEY08_BL_WAITING_FOR_COMMAND	    1
#define	KEY08_BL_WAITING_FOR_DATA	    2
#define KEY08_BL_WAITING_FOR_CRC	    3
#define KEY08_BL_GOT_BAD_CRC		    4 
#define KEY08_BL_WAITING_AFTER_BAD_CRC	    5
#define KEY08_BL_WAITING_AFTER_GOOD_CRC	    6
#define KEY08_BL_WAITING_FAILED		    7
#define KEY08_BL_I2C_ERROR		    8

/* Status of driver. */
#define KEY08_MODE_UNKNOWN		    0
#define KEY08_MODE_NORMAL		    1
#define	KEY08_MODE_BOOTLOADER		    2
#define	KEY08_MODE_OBP		            3

/* Status of FIRMAWARE transfers */
#define KEY08_FM_DOWNLOAD_SUCCESS           0x00
#define KEY08_FM_DOWNLOAD_STILL_ACTIVE      0x01
#define KEY08_FM_DOWNLOAD_FAILED            0x02
#define KEY08_FM_DOWNLOAD_NOT_STARTED       0x03

/* Status of CONFIGURATION transfers */
#define KEY08_CFG_DOWNLOAD_SUCCESS          0x00
#define KEY08_CFG_DOWNLOAD_STILL_ACTIVE     0x10
#define KEY08_CFG_DOWNLOAD_FAILED           0x20
#define KEY08_CFG_DOWNLOAD_NOT_STARTED      0x30

/* States for upgrading firmware. Bootloader (BL) states*/
#define BOOTLOADER_STATE_NOT_FOUND              0
#define BOOTLOADER_STATE_NOT_FOUND_SEARCH_ADDR1 1
#define BOOTLOADER_STATE_NOT_FOUND_SEARCH_ADDR2 2
#define BOOTLOADER_STATE_FOUND_CHECK_VER        3
#define BOOTLOADER_STATE_APPCRC_FAIL            4
#define BOOTLOADER_STATE_WAIT_UNLOCK_CMD        5
#define BOOTLOADER_STATE_READ_BEFORE_WAIT_FRAME 6
#define BOOTLOADER_STATE_WAIT_FRAME             7
#define BOOTLOADER_STATE_SENDING_FRAME_DATA     8
#define BOOTLOADER_STATE_WAIT_CRC_CHECK         9
#define BOOTLOADER_STATE_WAIT_CRC_RESULT        10
#define BOOTLOADER_STATE_WAIT_CHANGE_LINE       11

#define Q51001211009AK08_TS_MAX_X 	        1023
#define Q51001211009AK08_TS_MAX_Y	    	1023

#define KEY08_MAX_X			Q51001211009AK08_TS_MAX_X
#define KEY08_MAX_Y			Q51001211009AK08_TS_MAX_Y

#define	SCREEN_X	                           320
#define	SCREEN_Y	                           480


/*! @brief Values for positions in the touch key map array */
#define QTM_OBP_TOUCH_KEY_X_START                          0        
#define QTM_OBP_TOUCH_KEY_X_MAX                            1        
#define QTM_OBP_TOUCH_KEY_Y_START                          2
#define QTM_OBP_TOUCH_KEY_Y_MAX                            3
#define QTM_OBP_TOUCH_KEY_KEYCODE                          4
#define QTM_OBP_TOUCH_KEY_TOTAL_VAL                        5

/*! @brief Touch button definitions */
#define QTM_OBP_NUM_KEYS                                  24
#define QTM_OBP_MSG_NUM_OBPECTS                            4
#define QTM_OBP_KEY_RELEASE                                0
#define QTM_OBP_KEY_PRESS                                  1


// QTM - OBP Defines
#define QTM_OBP_REG_ADDR_SIZE                              2
#define QTM_OBP_READ_DATA_SIZE                             8
#define QTM_OBP_READ_RETRIES                              10
#define QTM_OBP_READ_WRITE_DELAY                        3000


/* ! @brief Defines for the Object IDs */
#define OBJ_ID_RESERVED_T0                              0x00
#define OBJ_ID_RESERVED_T1                              0x01
#define OBJ_ID_DEBUG_DELTAS_T2                          0x02
#define OBJ_ID_DEBUG_REFERENCES_T3                      0x03
#define OBJ_ID_DEBUG_SIGNALS_T4                         0x04
#define OBJ_ID_GEN_MESSAGEPROCESSOR_T5                  0x05
#define OBJ_ID_GEN_COMMANDPROCESSOR_T6                  0x06
#define OBJ_ID_GEN_POWERCONFIG_T7                       0x07
#define OBJ_ID_GEN_ACQUIRECONFIG_T8                     0x08
#define OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9                0x09
#define OBJ_ID_TOUCH_SINGLETOUCHSCREEN_T10              0x0A
#define OBJ_ID_TOUCH_XSLIDER_T11                        0x0B
#define OBJ_ID_TOUCH_SLIDER_T12                         0x0C
#define OBJ_ID_TOUCH_XWHEEL_T13                         0x0D
#define OBJ_ID_TOUCH_YWHEEL_T14                         0x0E
#define OBJ_ID_TOUCH_KEYARRAY_T15                       0x0F
#define OBJ_ID_PROCG_SIGNALFILTER_T16                   0x10
#define OBJ_ID_PROCI_LINEARIZATIONTABLE_T17             0x11
#define OBJ_ID_PROCI_GESTURESPROCESS_T18                0x12

/*! @brief Mode definition for the panel */
#define	QTM_OBP_DONT_CHANGE_MODE                        -2
#define	QTM_OBP_STANDBY_T7        -OBJ_ID_GEN_POWERCONFIG_T7
#define QTM_OBP_INITIAL_MODE        OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9

#define QTM_OBP_INITIAL                                    1
#define QTM_OBP_STANDBY                                    2

#define QTM_OBP_REPORT_ID_BYTE                             0

/* OBJ_ID_GEN_COMMANDPROCESSOR_T6 message bytes */
#define QTM_OBP_T6_STATUS                                  1
#define QTM_OBP_T6_CHKSUM1                                 2
#define QTM_OBP_T6_CHKSUM2                                 3

/* OBJ_ID_GEN_COMMANDPROCESSOR_T6 message byte masks */
#define QTM_OBP_T6_STATUS_RESET                         0x80
#define QTM_OBP_T6_STATUS_OFL                           0x40
#define QTM_OBP_T6_STATUS_SIGERR                        0x20
#define QTM_OBP_T6_STATUS_CAL                           0x10
#define QTM_OBP_T6_STATUS_CFGERR                        0x08

/* OBJ_ID_GEN_POWERCONFIG_T7 message bytes */
#define QTM_OBP_T7_IDLE                                    0
#define QTM_OBP_T7_ACTIVE                                  1

/* OBP_ID_TOUCH_MULTITOUCHSCREEN_T9 Message bytes */
#define QTM_OBP_T9_STATUS                                  1
#define QTM_OBP_T9_XPOS_MSB                                2
#define QTM_OBP_T9_YPOS_MSB                                3
#define QTM_OBP_T9_XYPOS_LSB                               4
#define QTM_OBP_T9_TCH_AREA                                5
#define QTM_OBP_T9_TCH_AMPLI                               6
#define QTM_OBP_T9_TIMESTAMP                               7

/* OBP_ID_TOUCH_MULTITOUCHSCREEN_T9 Message byte masks */
#define QTM_OBP_T9_STATUS_TOUCH_MASK                    0x80
#define QTM_OBP_T9_STATUS_PRESS_MASK                    0x40
#define QTM_OBP_T9_STATUS_RELEASE_MASK                  0x20
#define QTM_OBP_T9_STATUS_MOVE_MASK                     0x10

#define QTM_OBP_T9_XYPOS_LSB_YPOS_MASK                  0x0F
#define QTM_OBP_T9_XYPOS_LSB_XPOS_MASK                  0xF0
#define QTM_OBP_T9_XYPOS_LSB_POS_SHIFT                     4

/* OBJ_ID_TOUCH_SINGLETOUCHSCREEN_T10 message bytes */
#define QTM_OBP_T10_STATUS                                 1
#define QTM_OBP_T10_XPOS_MSB                               2
#define QTM_OBP_T10_YPOS_MSB                               3
#define QTM_OBP_T10_XYPOS_LSB                              4
#define QTM_OBP_T10_TCH_AREA                               5
#define QTM_OBP_T10_TCH_AMPLI                              6
#define QTM_OBP_T10_TIMESTAMP                              7

/* OBJ_ID_TOUCH_SINGLETOUCHSCREEN_T10 message byte masks */
#define QTM_OBP_T10_STATUS_TOUCH_MASK                   0x80
#define QTM_OBP_T10_STATUS_PRESS_MASK                   0x40
#define QTM_OBP_T10_STATUS_RELEASE_MASK                 0x20
#define QTM_OBP_T10_STATUS_MOVE_MASK                    0x10

#define QTM_OBP_T10_XYPOS_LSB_YPOS_MASK                 0x0C
#define QTM_OBP_T10_XYPOS_LSB_XPOS_MASK                 0xC0
#define QTM_OBP_T10_XYPOS_LSB_XPOS_SHIFT                   6
#define QTM_OBP_T10_XYPOS_LSB_YPOS_SHIFT                   2


#define QTM_OBP_MAX_OBJECT_NUM                            40

#define  QTM_OBP_OBJ_TBL_TYPE                              0
#define  QTM_OBP_OBJ_TBL_START_POS1                        1
#define  QTM_OBP_OBJ_TBL_START_POS2                        2
#define  QTM_OBP_OBJ_TBL_OBJ_SIZE                          3
#define  QTM_OBP_OBJ_TBL_NUM_INST                          4
#define  QTM_OBP_OBJ_TBL_NUM_REPORTID                      5
#define  QTM_OBP_OBJ_TBL_BLK_SIZE                          6


/* OBJ_ID_GEN_COMMANDPROCESSOR_T6 memory config, size=5 bytes */
#define QTM_OBP_T6_RESET                                   0
#define QTM_OBP_T6_BACKUPNV                                1
#define QTM_OBP_T6_CALIBRATE                               2
#define QTM_OBP_T6_REPORTALL                               3
#define QTM_OBP_T6_DBGCTRL                                 4
#define QTM_OBP_T6_MEMCFG_SIZE                             5

/* OBJ_ID_TOUCH_KEYARRAY_T15 key array */
#define QTM_OBP_T15_KEY_PRESS_MASK                      0x80
#define QTM_OBP_T15_STATUS                                 1
#define QTM_OBP_T15_CHANNEL                                2
#define QTM_OBP_T15_BACK_KEY                            0x02
#define QTM_OBP_T15_HOME_KEY                            0x04
#define QTM_OBP_T15_MENU_KEY                            0x10


#define QTM_OBP_T6_BACKUP_CMD                           0x55
 
/* Info Block offsets */
#define  QTM_OBP_INFO_BLOCK_FAMILY_ID                      0
#define  QTM_OBP_INFO_BLOCK_VARIANT_ID                     1
#define  QTM_OBP_INFO_BLOCK_VERSION                        2
#define  QTM_OBP_INFO_BLOCK_BUILD                          3
#define  QTM_OBP_INFO_BLOCK_MATRIX_XSIZE                   4
#define  QTM_OBP_INFO_BLOCK_MATRIX_YSIZE                   5
#define  QTM_OBP_INFO_BLOCK_NUM_OF_OBJECT                  6

#define  QTM_OBP_INFO_BLOCK_SIZE                           7

typedef struct	a_point
{
	int	X;
	int	Y;
	int	Z;
	int	W;
	int	finger;
} A_TOUCH_POINT, *A_TOUCH_POINT_PTR;

typedef struct	a_reg
{
	int	reg;
	int	value;
} A_REG, *A_REG_PTR;


