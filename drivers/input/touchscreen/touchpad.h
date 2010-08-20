/*
 * Copyright (C) 2009 Motorola, Inc.
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
 * Motorola 2009-Feb-20 - Initial Creation
 * Motorola 2009-March-31 - Modified the IOCTLs for generic use case
 *                          
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TOUCHPAD_H
#define TOUCHPAD_H

#include <linux/ioctl.h> /* for ioctl macros */

typedef struct
{
    int    abs_x;
    int    abs_y;
    int abs_pressure;
    int abs_tool_width;
    int btn_touch;
      
} MOTO_TS_DATA_T;


typedef enum
{
    /* Start at -20 to allow up to 20 error codes */
    MOT_TOUCH_BL_I2C_WRITE_ERROR = -20,
    MOT_TOUCH_BL_COPY_DATA_ERROR,
    MOT_TOUCH_BL_I2C_READ_ERROR,
    MOT_TOUCH_BL_DATA_WRITE_ERROR,
    MOT_TOUCH_BL_AFTER_CRC_FAIL,
    MOT_TOUCH_BL_GOT_BAD_CRC,
    MOT_TOUCH_BL_FW_VERSION_READ_ERROR,
    MOT_TOUCH_BL_APP_BAD_CRC,
    /* Status of the Boot loader allow up to 10 status messages */
    MOT_TOUCH_BL_WAITING_FOR_NOTHING = 0,
    MOT_TOUCH_BL_WAITING_FOR_STATUS,
    MOT_TOUCH_BL_WAITING_FOR_DATA,
    MOT_TOUCH_BL_WAITING_FOR_CRC,
    MOT_TOUCH_BL_AFTER_CRC_PASS,
    
    MOT_TOUCH_BL_FW_UPDATE_SUCCESS =10,
    MOT_TOUCH_BL_FW_NOT_STARTED    =0xFF,

}MOT_TOUCH_BL_MODE_STATE_T;

/* Status of IC. */
#define MOT_TOUCH_MODE_UNKNOWN        0
#define MOT_TOUCH_MODE_NORMAL         1
#define MOT_TOUCH_MODE_BOOTLOADER     2


#define MOT_TOUCH_IOCTL_BASE          88

/*! @brief Get the IOCTL command depending on IOCTL class and offset */
#define MOT_TOUCH_GET_IOCTL_OFFSET(class, offset) (class+offset)

/*! @brief  Various IOCTL classes for Touch IC are
* DEBUG: IOCTLS to retrieve any debug information from the driver 
* BOOT_SUPPORT: IOCTLS used to perform firmware upgrade
* STATUS: IOCTLS used to obtain the statusof the IC/driver
* COMMAND: IOCTLS used to command the IC to perform some operation (in NORMAL mode)
*/
#define MOT_TOUCH_IOCTL_CLASS_DEBUG                00
#define MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT         20
#define MOT_TOUCH_IOCTL_CLASS_STATUS               40
#define MOT_TOUCH_IOCTL_CLASS_COMMAND              60

/* IOCTL codes */
/*! @brief  IOCTL codes for COMMAND class  */
    
/*! @brief IOCTL to enable touchscreen
 * IOCTL name: MOT_TOUCH_IOCTL_ENABLE_TOUCH
 * IOCTL description: This ioctl is used to enable the touchpad.  Used by Eventhub to enable touchscreen.
 * IOCTL example usage:
 *     ioctl(handle,MOT_TOUCH_IOCTL_ENABLE_TOUCH) 
 */
#define MOT_TOUCH_IOCTL_ENABLE_TOUCH    _IO(MOT_TOUCH_IOCTL_BASE, MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 1))

/*! @brief IOCTL to disable touchscreen
 * IOCTL name: MOT_TOUCH_IOCTL_DISABLE_TOUCH
 * IOCTL description: This ioctl is used to disable the touchpad.  Used by Eventhub to disable touchscreen.
 * IOCTL example usage:
 *     ioctl(handle,MOT_TOUCH_IOCTL_DISABLE_TOUCH) 
 */
#define MOT_TOUCH_IOCTL_DISABLE_TOUCH   _IO(MOT_TOUCH_IOCTL_BASE,MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 2))

/*! @brief IOCTL to start the calibration of Touch IC
 * IOCTL name: MOT_TOUCH_IOCTL_IC_CALIBRATION
 * IOCTL description: This ioctl is used to start the calibration of touch IC.  Used by Eventhub to start calibration..
 * IOCTL example usage:
 *     ioctl(handle,MOT_TOUCH_IOCTL_IC_CALIBRATION) 
 */
#define MOT_TOUCH_IOCTL_IC_CALIBRATION  _IO( MOT_TOUCH_IOCTL_BASE,MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 3))

/*! @brief IOCTL to set the current touch mode
 * IOCTL name: MOT_TOUCH_IOCTL_SET_TOUCH_MODE
 * IOCTL description: This ioctl is used to set one of the following  touch mode.
 * KEY08_DONT_CHANGE_MODE  -2
 * KEY08_STANDBY           -1
 * KEY08_INITIAL_MODE       0 
 * IOCTL example usage:
 *     unsigned int char_reg = KEY08_STANDBY;
 *     ioctl(handle,MOT_TOUCH_IOCTL_SET_TOUCH_MODE,&char_reg) 
 */
#define MOT_TOUCH_IOCTL_SET_TOUCH_MODE  _IOW(MOT_TOUCH_IOCTL_BASE, MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND,4), int*)


/*! @brief  IOCTL codes for STATUS class  */

/*! @brief IOCTL to get the current touch mode
 * IOCTL name: MOT_TOUCH_IOCTL_GET_TOUCH_MODE
 * IOCTL description: This ioctl is used to get the current touch mode, which is 
 *   one of the following:
 * KEY08_DONT_CHANGE_MODE  -2
 * KEY08_STANDBY           -1
 * KEY08_INITIAL_MODE       0 
 * IOCTL example usage:
 *     unsigned int char_reg;
 *     ioctl(handle,MOT_TOUCH_IOCTL_GET_TOUCH_MODE,&char_reg) 
 *     // char_reg contains the current touch mode
 */
#define MOT_TOUCH_IOCTL_GET_TOUCH_MODE  _IOR(MOT_TOUCH_IOCTL_BASE,MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_STATUS,1), int*)

/*! @brief IOCTL to get the Firmware version
 * IOCTL name: MOT_TOUCH_IOCTL_GET_FW_VERSION
 * IOCTL description: This ioctl is used to get the current Firmware version
 * IOCTL example usage:
 *     unsigned int char_reg;
 *     ioctl(handle,MOT_TOUCH_IOCTL_GET_FW_VERSION,&char_reg) 
 *     //char_reg contains the current firmware version
 */
#define MOT_TOUCH_IOCTL_GET_FW_VERSION  _IOR(MOT_TOUCH_IOCTL_BASE, MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_STATUS,2), int*)


/*! @brief  IOCTL codes for DEBUG  class  */

/*! @brief IOCTL to get current touch data
 * IOCTL name: MOT_TOUCH_IOCTL_GET_TOUCH_DATA:
 * IOCTL description: This IOCTL is used to get the current touch data from the kernel to user space.
 * IOCTL example usage:
 *     MOT_TS_DATA_T ts_data;
 *     ioctl(handle,MOT_TOUCH_IOCTL_GET_TOUCH_DATA,&ts_data) 
 *     // ts_data contains the current touch data
 */
#define MOT_TOUCH_IOCTL_GET_TOUCH_DATA  _IOR(MOT_TOUCH_IOCTL_BASE,MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_DEBUG,1) , struct MOTO_TS_DATA*)


/*! @brief  IOCTL codes for BOOT_SUPPORT  class  */

/*! @brief IOCTL to get the state of driver in boot mode
 * IOCTL name: MOT_TOUCH_IOCTL_BL_GET_STATUS
 * IOCTL description: This ioctl is used to  get the state of driver in boot mode which
 * is of type MOT_TOUCH_BL_MODE_STATE_T
 * IOCTL example usage:
 *     MOT_TOUCH_BL_MODE_STATE_T char_reg;
 *     ioctl(handle,MOT_TOUCH_IOCTL_BL_GET_STATUS,&char_reg) 
 *     // char_reg contains the current boot mode
 */
#define MOT_TOUCH_IOCTL_BL_GET_STATUS       _IOR(MOT_TOUCH_IOCTL_BASE, MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT,1), int*)

/*! @brief get the IC mode 
* IOCTL name: MOT_TOUCH_IOCTL_GET_IC_MODE
* IOCTL Description: This ioctl is used to get the current mode of the IC, which can be
* one of the following:
* MOT_TOUCH_MODE_UNKNOWN       0    
* MOT_TOUCH_MODE_NORMAL        1
* MOT_TOUCH_MODE_BOOTLOADER    2
* IOCTL example usage:
*     unsigned int char_reg;
*     ioctl(handle,MOT_TOUCH_IOCTL_GET_IC_MODE,&char_reg) 
*     // char_reg contains the current IC mode
*/
#define MOT_TOUCH_IOCTL_GET_IC_MODE         _IOR(MOT_TOUCH_IOCTL_BASE, MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT,2), int)

/*! @brief IOCTL code for  putting the touch IC into boot mode 
* IOCTL name : MOT_TOUCH_SET_BOOT_MODE:
* IOCTL Description: This IOCTL is used to set touch IC to BOOT mode.  The parameter is
* used to decide if the entire boot sequence has to be executed or only the unlock
* command has to be sent to the IC.  IC will expect only the unlock command if it has
* found a CRC error in the existing application.
* example usage:
*     unsigned int boot_from_unlock = 1; // send only unlock command
*     ioctl(handle,MOT_TOUCH_SET_BOOT_MODE,&boot_from_unlock) 
*
*/
#define MOT_TOUCH_IOCTL_SET_BOOT_MODE       _IO(MOT_TOUCH_IOCTL_BASE,MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT,3) )

/*! @brief IOCTL code for  putting the touch IC into normal mode 
* IOCTL name : MOT_TOUCH_IOCTL_SET_NORMAL_MODE
* IOCTL Description: This IOCTL is used to set touch IC to NORMAL_MODE mode.
* example usage:
*     ioctl(handle,MOT_TOUCH_IOCTL_SET_NORMAL_MODE)
*/
#define MOT_TOUCH_IOCTL_SET_NORMAL_MODE     _IO(MOT_TOUCH_IOCTL_BASE, MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT,4))

#endif  /* TOUCHPAD_H */

#ifdef __cplusplus
}
#endif

