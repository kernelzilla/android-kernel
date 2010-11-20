/* drivers/input/keyboard/minipad.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
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
 */

/******************************************************************************
* 
******************************************************************************/
#ifndef FALSE
#define FALSE    0
#define TRUE     (!FALSE)
#endif

// sw version
#define MINIPAD_VERSION                         "082709"

// uncomment the below line to enable debug build
//#define DEBUG_MINIPAD

/******************************************************************************
* mouse logic config and parameters
******************************************************************************/
#define MINIPAD_MOUSE_MOTUS_TH_X                230
#define MINIPAD_MOUSE_MOTUS_TH_Y                180

#define MINIPAD_MOUSE_ZEPPELIN_TH_X             50
#define MINIPAD_MOUSE_ZEPPELIN_TH_Y             50

#define MINIPAD_MOUSE_ZEPPELIN_X_SCALE          45 //0->100
#define MINIPAD_MOUSE_ZEPPELIN_Y_SCALE          25 //0->100

#define MINIPAD_MOUSE_MAX_X                     1023
#define MINIPAD_MOUSE_MAX_Y                     1023
#define MINIPAD_MOUSE_MIN_X                     0
#define MINIPAD_MOUSE_MIN_Y                     0

#define MINIPAD_TAP_TIME_MIN_TH                 4  // x10 milliseconds
#define MINIPAD_TAP_TIME_MAX_TH                 35 // x10 milliseconds

#define MINIPAD_TAP_BOUND_X                     90 // ignore tap when moving
#define MINIPAD_TAP_BOUND_Y                     60

#define MINIPAD_TAP_GUTTER_X                    120 // ignore tap close to edge
#define MINIPAD_TAP_GUTTER_Y                    120

#define MINIPAD_KEY_RELEASE                     FALSE
#define MINIPAD_KEY_PRESS                       TRUE

#define MINIPAD_KEY_RIGHT                       KEY_RIGHT // DPAD_RIGHT =106
#define MINIPAD_KEY_LEFT                        KEY_LEFT  // DPAD_LEFT  =105
#define MINIPAD_KEY_UP                          KEY_UP    // DPAD_UP    =103
#define MINIPAD_KEY_DOWN                        KEY_DOWN  // DPAD_DOWN  =108
#define MINIPAD_KEY_CENTER                      KEY_REPLY // DPAD_CENTER=232

/******************************************************************************
* minipad ioctl
******************************************************************************/
//#define MINIPAD_IOCTL_SET_IN_CALL             1
//#define MINIPAD_IOCTL_SET_NOT_IN_CALL         2
#define MINIPAD_IOCTL_SET_BOOTLOADER_MODE       3
#define MINIPAD_IOCTL_SET_NORMAL_MODE           4
#define MINIPAD_IOCTL_GET_VERSION               5
#define MINIPAD_IOCTL_WRITE_REGISTER            6
#define MINIPAD_IOCTL_READ_REGISTER             7
#define MINIPAD_IOCTL_DL_GET_STATUS             8
#define MINIPAD_IOCTL_GET_MODE                  9
//#define MINIPAD_IOCTL_END_FIRMWARE            10
//#define MINIPAD_IOCTL_DISPLAY_ONE_POINT       11
#define MINIPAD_IOCTL_DO_RESET                  12
#define MINIPAD_IOCTL_DO_CALIBRATION            13
#define MINIPAD_IOCTL_GET_XANDY                 14
#define MINIPAD_IOCTL_SET_ENABLE                15
#define MINIPAD_IOCTL_SET_DISABLE               16
#define MINIPAD_IOCTL_READ_REGISTER16           17
#define MINIPAD_IOCTL_START                     18
#define MINIPAD_IOCTL_SETIRQ                    19
#define MINIPAD_IOCTL_TEST                      20
#define MINIPAD_IOCTL_READ_SENS                 21
#define MINIPAD_IOCTL_SET_LPMODE_VALUE          22
#define MINIPAD_IOCTL_GETIRQ                    23
#define MINIPAD_IOCTL_GETGPIO                   24

/* Status of driver. */
#define MINIPAD_MODE_UNKNOWN                    0
#define MINIPAD_MODE_NORMAL                     1
#define MINIPAD_MODE_BOOTLOADER                 2

/******************************************************************************
* Q5199225K03
******************************************************************************/
// register adddress
#define Q5199225K03_REG_ADDR_SIZE               2

#define Q5199225K03_CHIPID_ADDR                 0
#define Q5199225K03_CODE_VERSION_ADDR           1
#define Q5199225K03_BACKUP_ADDR                 2
#define Q5199225K03_CALIBRATE_ADDR              3
#define Q5199225K03_RESET_ADDR                  4
#define Q5199225K03_REG_READ_ADDR               5
#define Q5199225K03_LP_MODE_ADDR                143
#define Q5199225K03_SENS_ADDR                   512
#define Q5199225K03_SENS_CHANNELS               12

/* Bootloader statuses */
#define Q5199225K03_CHIP_ID                     0x21
#define Q5199225K03_CHIP_ID_0X23                0x23
#define Q5199225K03_BL_I2C_ADDR                 (0x25u << 1)
#define Q5199225K03_MODE_BL_PWR_ON              0x00
#define Q5199225K03_MODE_BL_WAITING_FRAME       0x01
#define Q5199225K03_MODE_BL_FRAME_CRC_CHCK      0x02
#define Q5199225K03_MODE_BL_FRAME_CRC_FAIL      0x03
#define Q5199225K03_MODE_BL_FRAME_CRC_PASS      0x04
#define Q5199225K03_MODE_BL_APP_CRC_FAIL        0x08
//#define Q5199225K03_FW_FRAME_LEN_SIZE       0x02
//#define Q5199225K03_FW_IMAGE_VER            0x61
//#define Q5199225K03_FW_IMAGE_LEN            sizeof(Q5199225K03_fw_image)

/* Bootloader sequences */
#define Q5199225K03_BL_SEQ1                     0x00
#define Q5199225K03_BL_SEQ2                     0xDC
#define Q5199225K03_BL_SEQ3                     0xAA
#define Q5199225K03_BL_SEQ4                     0x55

// Message header
#define Q5199225K03_MSG_OBJ_FLAG_BYTE           0
#define Q5199225K03_MSG_OBJ_FLAG_MASK           0x0F

// Message type
#define Q5199225K03_DATA_MSG                    0x80
#define Q5199225K03_KEY_MSG                     0x0
#define Q5199225K03_SLIDER_MSG                  0x1
#define Q5199225K03_OMEGA_MSG                   0x2
#define Q5199225K03_8BIT_TSP_MSG                0x3
#define Q5199225K03_10BIT_TSP_MSG               0x4
#define Q5199225K03_STATUS_MSG                  0x7
#define Q5199225K03_READ_MSG                    0x8

// Status flags
#define Q5199225K03_MSG_STATUS_FLAG_BYTE        1

// Event details
#define Q5199225K03_MSG_PTR10_DATA_XL_BYTE      2
#define Q5199225K03_MSG_PTR10_DATA_XH_BYTE      3
#define Q5199225K03_MSG_PTR10_DATA_YL_BYTE      4
#define Q5199225K03_MSG_PTR10_DATA_YH_BYTE      5
#define Q5199225K03_MSG_PTR10_DATA_XYH_MASK     0x03
#define Q5199225K03_MSG_PTR10_DEVICE_INDEX      0

#define Q5199225K03_MSG_TOUCH_FLAG_BYTE         1
#define Q5199225K03_MSG_TOUCH_FLAG_SHIFT        0
#define Q5199225K03_MSG_CRUNCH_FLAG_BYTE        1
#define Q5199225K03_MSG_CRUNCH_FLAG_SHIFT       4

/******************************************************************************
* other definitions
******************************************************************************/
extern unsigned long msleep_interruptible(unsigned int);
#define clk_busy_wait(time)    msleep_interruptible(time/1000)
#define Q5199225K03_WAIT                        3000

/******************************************************************************
* minipad driver
******************************************************************************/
#define MINIPAD_TS_GPIO 83
#define MINIPAD_I2C_ADDR 0x54
#define MINIPAD_BL_I2C_ADDR 0x25

#define MINIPAD_I2C_NAME "minipad"
#define MINIPAD_BL_I2C_NAME "minipad_bl"
#define MINIPAD_PF_NAME    "minipad"

/******************************************************************************
* Structure definitions
******************************************************************************/
typedef struct a_minipad_reg
{
    int    reg;
    int    value;
} A_MINIPAD_REG, *A_MINIPAD_REG_PTR;

/******************************************************************************
* End of file
******************************************************************************/
