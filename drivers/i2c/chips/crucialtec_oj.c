/*=================================================================================================
 
    Module Name: crucialtec_oj.c
 
    DESCRIPTION: Linux driver for Crucialtec Optical Joystick
 
===================================================================================================

                       Motorola Confidential Proprietary
                   Advanced Technology and Software Operations
                (c) Copyright Motorola 2009, All Rights Reserved
  
                       Modification   Tracking
Author                    Date         Number    Description of Changes
---------------------  ------------  ----------  ---------------------------------------------------
Vinh Vo (w51099)        09/10/2009   LIBtt21549  ESD detection and fix
Vinh Vo (w51099)        07/23/2009   LIBss79054  Changing the default Operational Mode to 
                                                 Directional Keypad.  Register a misc device to the 
                                                 OJ driver to configure the valid delta, 
                                                 time delay between key press, and operational mode 
Vinh Vo (w51099)        06/08/2009   LIBss57546  Created
==================================================================================================*/

/*==================================================================================================
                                           INCLUDE FILES
==================================================================================================*/
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/board.h>
#include <mach/crucialtec_oj.h>
#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
#include <mot/esd_poll.h>
#endif

/*==================================================================================================
                                          GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
                                          LOCAL CONSTANTS
==================================================================================================*/
#define CRUCIALTEC_OJ_DEBUG_MSGS 0   // Debug message flag

// Support for Misc device
//    Enables the confiuguration of Mode, Valid Delta, and Key Press Delay via adb shell
#define CRUCIALTEC_OJ_MISC_DEVICE_SUPPORT 1     

#define CRUCIALTEC_OJ_MISC_TEMP_BUFFER_SIZE 50

#define CRUCIALTEC_OJ_MISC_DEVICE_WRITE_PARAMETER_LIMIT 3

#define CRUCIALTEC_OJ_PRODUCT_NAME    "optical_joystick"

#define CRUCIALTEC_VENDOR_ID   0x0100

#define CRUCIALTEC_OJ_MOTION_GPIO_IRQ_REQUEST_FLAG    IRQF_TRIGGER_LOW

#define CRUCIALTEC_OJ_DELAY_RESET_LINE_FROM_LOW_TO_HIGH            20  // in usec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY                 23  // in msec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_SELF_TEST                  250  // in msec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SUSPEND  100  // in msec
#define CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SOFT_RESET  100 //in msec

// If the placement of the OJ is rotated clockwise by 90 degrees then swap the x and y data
#define CRUCIALTEC_OJ_ORIENTATION_POSITION_SWAP_X_TO_Y   1

#define CRUCIALTEC_OJ_PERFORM_SELF_TEST_DURING_INIT      0

#define DPAD_UP_KEY    22    // DPAD's values are from the optical_joystick.kl file
#define DPAD_DOWN_KEY  24 
#define DPAD_RIGHT_KEY 13 
#define DPAD_LEFT_KEY  33 

// For Directional Pad Mode.  The distance traveled must be greater then this to be a valid input.
#define CRUCIALTEC_OJ_MAX_DISTANCE_TRAVELED  10 

// For Directional Pad Mode. Delay between key press (in ms)
#define CRUCIALTEC_OJ_MAX_DELAY_BETWEEN_KEY  65 

#define CRUCIALTEC_OJ_XMIN_NOMINAL  -128           // X-axis min and max values
#define CRUCIALTEC_OJ_XMAX_NOMINAL  127
#define CRUCIALTEC_OJ_YMIN_NOMINAL  -128           // Y-axis min and max values
#define CRUCIALTEC_OJ_YMAX_NOMINAL  127


/******************************************************************************
*                  Crucial OJ's Registers and Defines
******************************************************************************/

/* Product ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   PID7   |   PID6   |   PID5   |   PID4   |   PID3   |   PID2   |   PID1   |   PID0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_PRODUCT_ID              0x00


/* Revision ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   RID7   |   RID6   |   RID5   |   RID4   |   RID3   |   RID2   |   RID1   |   RID0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_REVISION_ID             0x01


/* Motion Register
 *       =========================================================================================
 * Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *       =========================================================================================
 * Field |    MOT   |  PIXRDY  | PIXFIRST |    OVF   | Reserved | Reserved | Reserved |   GPIO   |
 *       =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_MOTION                  0x02
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_MASK                         0x80   // Motion Detection
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_NO_MOTION                    0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OCCURRED                     0x80
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXRDY_MASK                         0x40   // Pixel Dump Reg is Ready
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXRDY_DATA_NOT_AVAILABLE           0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXRDY_DATA_AVAILABLE               0x40
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXFIRST_MASK                       0x20   // Pixel Grab Reg is Ready
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXFIRST_NOT_FROM_PIXEL_ZERO_ZERO   0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_PIXFIRST_IS_FROM_PIXEL_ZERO_ZER0    0x20
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OVERFLOW_MASK                0x10   // Motion Overflow
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OVERFLOW_NO_OVERFLOW         0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OVERFLOW_OCCURRED            0x10
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_GPIO_STATUS_MASK             0x01   // Motion's GPIO Status
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_GPIO_STATUS_LOW              0x00
#define CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_GPIO_STATUS_HIGH             0x01


/* X Movement is counts since last report.  Eight bit 2's complement number
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    X7    |    X6    |    X5    |    X4    |    X3    |    X2    |    X1    |    X0    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_DELTA_X                 0x03


/* Y Movement is counts since last report.  Eight bit 2's complement number
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    Y7    |    Y6    |    Y5    |    Y4    |    Y3    |    Y2    |    Y1    |    Y0    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_DELTA_Y                 0x04


#define CRUCIALTEC_OJ_REG_SQUAL                   0x05
#define CRUCIALTEC_OJ_REG_SHUTTER_UPPER           0x06
#define CRUCIALTEC_OJ_REG_SHUTTER_LOWER           0x07
#define CRUCIALTEC_OJ_REG_MAX_PIXEL               0x08
#define CRUCIALTEC_OJ_REG_PIXEL_SUM               0x09
#define CRUCIALTEC_OJ_REG_MIN_PIXEL               0x0A
#define CRUCIALTEC_OJ_REG_PIXEL_GRAB              0x0B


/* Self Test CRC Registers
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   CRC7   |   CRC6   |   CRC5   |   CRC4   |   CRC3   |   CRC2   |   CRC1   |   CRC0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_CRC0                    0x0C
#define CRUCIALTEC_OJ_REG_CRC0_SELF_TEST_VALID_VALUE    0x33

#define CRUCIALTEC_OJ_REG_CRC1                    0x0D
#define CRUCIALTEC_OJ_REG_CRC1_SELF_TEST_VALID_VALUE    0x8E

#define CRUCIALTEC_OJ_REG_CRC2                    0x0E
#define CRUCIALTEC_OJ_REG_CRC2_SELF_TEST_VALID_VALUE    0x24

#define CRUCIALTEC_OJ_REG_CRC3                    0x0F
#define CRUCIALTEC_OJ_REG_CRC3_SELF_TEST_VALID_VALUE    0x6C


/* Self Test Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved |  TESTEN  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SELF_TEST               0x10
#define CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_MASK    0x01    // Test Mode
#define CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_OFF     0x00
#define CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_ON      0x01


/* Configuration Bits Register - to set the sensor resolution
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |Resolution| Reserved | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_CONFIG_BITS             0x11
#define CRUCIALTEC_OJ_REG_CONFIG_BITS_VALUE_RESOLUTION_MASK     0x80   // Sensor Resolution
#define CRUCIALTEC_OJ_REG_CONFIG_BITS_VALUE_RESOLUTION_500_CPI  0x00
#define CRUCIALTEC_OJ_REG_CONFIG_BITS_VALUE_RESOLUTION_1000_CPI 0x80


/* LED Control Register 
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | Reserved | Reserved |   LED3   | Reserved | Reserved | Reserved |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_LED_CONTROL             0x1A
#define CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_MASK              0x08  // LED Control Mode
#define CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_NORMAL_OPERATION  0x00
#define CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_CONTINUOUS_ON     0x08


/* IO Mode Register 
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | Reserved |   Burst  | Reserved |    SPI   | Reserved |   TWI    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_IO_MODE                 0x1C
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_BURST_MODE_MASK      0x10     // Burst Mode
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_BURST_MODE_OFF       0x00
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_BURST_MODE_ON        0x10
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_SPI_MODE_MASK        0x04     // SPI Mode
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_SPI_MODE_OFF         0x00
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_SPI_MODE_ON          0x04
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_TWI_MODE_MASK        0x01     // TWI Mode
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_TWI_MODE_OFF         0x00
#define CRUCIALTEC_OJ_REG_IO_MODE_VALUE_TWI_MODE_ON          0x01


/* Observation Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |  MODE1   |  MODE0   | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_OBSERVATION             0x2E
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_MODE_MASK       0xC0   // Observation Mode
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RUN_MODE        0x00
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RESET1_MODE     0x40
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RESET2_MODE     0x80
#define CRUCIALTEC_OJ_REG_OBSERVATION_VALUE_RESET3_MODE     0xC0


/* Soft Reset Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   RST    |   RST    |   RST    |   RST    |   RST    |   RST    |   RST    |   RST    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SOFT_RESET              0x3A
#define CRUCIALTEC_OJ_REG_SOFT_RESET_VALUE_REVERT_TO_DEFAULT  0x5A  // Soft Reset value


/* Shutter maximum open time - upper and lower 8-bit
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    SM7   |    SM6   |    SM5   |    SM4   |    SM3   |    SM2   |    SM1   |    SM0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SHUTTER_MAX_HI          0x3B
#define CRUCIALTEC_OJ_REG_SHUTTER_MAX_LO          0x3C


/* Inverse Revision ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   NRID7  |   NRID6  |   NRID5  |   NRID4  |   NRID3  |   NRID2  |   NRID1  |   NRID0  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_INVERSE_REVISION_ID     0x3E


/* Inverse Product ID
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |   NPID7  |   NPID6  |   NPID5  |   NPID4  |   NPID3  |   NPID2  |   NPID1  |   NPID0  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_INVERSE_PRODUCT_ID      0x3F


/* Engine Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |  Engine  |   Speed  |  Assert/ |   XY Q   | Reserved |  Finger  | XY_Scale | Reserved |
 *      |          |          | DeAssert |          |          |          |          |          |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_ENGINE              0x60
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_MASK    0x80      // Engine Properties
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_OFF     0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_ON      0x80
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_MASK     0x40      // Speed Switching
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_OFF      0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_ON       0x40
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_MASK    0x20      // Assert and Deassert
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_OFF     0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_ON      0x20
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_MASK       0x10      // XY Quantization
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_OFF        0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_ON         0x10
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_MASK    0x04      // Finger Present Detection
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_OFF     0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_ON      0x04
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_MASK  0x02      // XY Scaling Factor
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_OFF   0x00
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_ON    0x02
#define CRUCIALTEC_OJ_REG_ENGINE_VALUE_DEFAULT         \
       (CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_ON | CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_ON)

/* Resolution Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Reserved | Reserved | WakeRES2 | WakeRES1 | WakeRES0 |   RES2   |   RES1   |   RES0   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_RESOLUTION          0x62
// Sets resolution when sensor wakes up from rest modes.
// Effective if speed switching is enabled.
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_MASK      0x38  // Wakeup resolution
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_250_CPI   0x08
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_500_CPI   0x10
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_750_CPI   0x18
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_1000_CPI  0x20
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_1250_CPI  0x28
// Sets resolution for sensor
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_MASK             0x07  // Resolution of sensor
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_250_CPI          0x01
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_500_CPI          0x02
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_750_CPI          0x03
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_1000_CPI         0x04
#define CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_1250_CPI         0x05


/* Speed Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field | Y_scale  | X_scale  | Reserved | Reserved |SP_IntVal1|SP_IntVal0| Low_cpi  | Low_cpi  |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL       0x63
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_MASK          0x80   // Y scaling factor
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_OFF           0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_ON            0x80
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_MASK          0x40   // X scaling factor
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_OFF           0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_ON            0x40
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_MASK       0x0C   // Speed switching checking interval in ms
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_4_MS       0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_8_MS       0x04
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_12_MS      0x08
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_16_MS      0x0C
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_MASK          0x02   // Sets low CPI when in speed switching
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_OFF           0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_ON_250_CPI    0x02
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_MASK         0x01   // Sets high CPI when in speed switching
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_OFF          0x00
#define CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_ON_1250_CPI  0x01


/* Speed Switching Properties for the sensor
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    ST    |    ST    |    ST    |    ST    |    ST    |    ST    |    ST    |    ST    |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_SPEED_ST12          0x64
#define CRUCIALTEC_OJ_REG_SPEED_ST12_DEFAULT_VALUE  0x08

#define CRUCIALTEC_OJ_REG_SPEED_ST21          0x65
#define CRUCIALTEC_OJ_REG_SPEED_ST21_DEFAULT_VALUE  0x06

#define CRUCIALTEC_OJ_REG_SPEED_ST23          0x66
#define CRUCIALTEC_OJ_REG_SPEED_ST23_DEFAULT_VALUE  0x40

#define CRUCIALTEC_OJ_REG_SPEED_ST32          0x67
#define CRUCIALTEC_OJ_REG_SPEED_ST32_DEFAULT_VALUE  0x08

#define CRUCIALTEC_OJ_REG_SPEED_ST34          0x68
#define CRUCIALTEC_OJ_REG_SPEED_ST34_DEFAULT_VALUE  0x48

#define CRUCIALTEC_OJ_REG_SPEED_ST43          0x69
#define CRUCIALTEC_OJ_REG_SPEED_ST43_DEFAULT_VALUE  0x0A

#define CRUCIALTEC_OJ_REG_SPEED_ST45          0x6A
#define CRUCIALTEC_OJ_REG_SPEED_ST45_DEFAULT_VALUE  0x50

#define CRUCIALTEC_OJ_REG_SPEED_ST54          0x6B
#define CRUCIALTEC_OJ_REG_SPEED_ST54_DEFAULT_VALUE  0x48


/* Assert De-assert Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |     1    |     1    | Reserved | Reserved | Reserved | ST_HIGH2 | ST_HIGH1 | ST_HIGH0 |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_AD_CTRL             0x6D
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_MASK        0x07   // Set the Assert De-Assert Control
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_LOWEST_CPI  0xC1
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_LOW_CPI     0xC2
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_MIDDLE_CPI  0xC3
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_HIGHER_CPI  0xC4
#define CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_HIGHEST_CPI 0xC5


/* For setting the HIGH/LOW speed Assert shuter threshold
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |    ATH   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_AD_ATH_HIGH         0x6E
#define CRUCIALTEC_OJ_REG_AD_ATH_HIGH_DEFAULT_VALUE   0x34

#define CRUCIALTEC_OJ_REG_AD_ATH_LOW          0x70
#define CRUCIALTEC_OJ_REG_AD_ATH_LOW_DEFAULT_VALUE    0x18

/* For setting the HIGH/LOW speed De-assert shuter threshold
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *      =========================================================================================
 *Field |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |    DTH   |
 *      =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_AD_DTH_HIGH         0x6F
#define CRUCIALTEC_OJ_REG_AD_DTH_HIGH_DEFAULT_VALUE   0x3C

#define CRUCIALTEC_OJ_REG_AD_DTH_LOW          0x71
#define CRUCIALTEC_OJ_REG_AD_DTH_LOW_DEFAULT_VALUE    0x20

/* Quantization Control Register for X and Y
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field |  YQ_ON   | YQ_DIV2  | YQ_DIV1  | YQ_DIV0  |  XQ_ON   | XQ_DIV3  | XQ_DIV2  | XQ_DIV0  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL       0x73
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_MASK              0x80   // Y Quantization Control
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_OFF               0x00
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_ON                0x80
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_MASK              0x08   // X Quantization Control
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_OFF               0x00
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_ON                0x08
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_MASK          0x70   // Y Quantization Factor
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_1   0x10   // Y Quantization Factor of 1
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_2   0x20   // Y Quantization Factor of 2
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_MASK          0x07   // X Quantization Factor
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_1   0x01   // Y Quantization Factor of 1
#define CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_2   0x02   // Y Quantization Factor of 2


/* Quantization Gradient Control Register for X and Y
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field | Reserved | Reserved | Reserved | Reserved | Reserved |  XYQ_M   |  XYQ_C1  |  XYQ_C0  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_XYQ_THRESH          0x74
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_GRADIENT_MASK    0x04   // Gradient of Linear Region
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_GRADIENT_1       0x00
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_GRADIENT_2       0x04
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_INDICATES_OFFSET_OF_LINEAR_REGION_MASK    0x03
#define CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_INDICATES_OFFSET_OF_LINEAR_REGION_DEFAULT 0x02


/* Finger Presence Detection Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field | Reserved | FPD_POL  | FPD_TH5  | FPD_TH4  | FPD_TH3  | FPD_TH2  | FPD_TH1  | FPD_TH0  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_FPD_CTRL            0x75
#define CRUCIALTEC_OJ_REG_FPD_CTRL_DEFAULT_VALUE            0x50
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_GPIO_STATUS_MASK   0x40   // FPD GPIO Status
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_GPIO_STATUS_LOW    0x00
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_GPIO_STATUS_HIGH   0x40
#define CRUCIALTEC_OJ_REG_FPD_CTRL_VALUE_THRESHOLD_MASK     0x3F   // Sets FPD threshold based on shutter value.


/* Sensor Orientation Control Register
 *      =========================================================================================
 *Bit   |     7    |     6    |     5    |     4    |     3    |     2    |     1    |     0    |
 *     =========================================================================================
 *Field | XY_SWAP  |  Y_INV   |   X_INV  | Reserved | Reserved | Reserved |  ORIENT  |  ORIENT  |
 *     =========================================================================================
 */
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL    0x77
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_MASK   0x80   // XY Swap Mask
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_OFF    0x00   // Normal sensor reporting of DX and DY
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_ON     0x80   // Swap data of DX to DY and DY to DX

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_Y_INV_MASK     0x40   // Y Invert Mask
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_Y_INV_OFF      0x00   // Normal sensor reporting DY
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_Y_INV_ON       0x40   // Invert data of DY only

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_X_INV_MASK     0x20   // X Invert Mask
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_X_INV_OFF      0x00   // Normal sensor reporting DX
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_X_INV_ON       0x20   // Invert data of DX only

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_ORIENT_MASK              0x03  // Orient Pin State 
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_ORIENT_90_DEG_CLOCKWISE  0x00
#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_ORIENT_ZERO_DEG          0x01

#define CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_DEFAULT  \
               CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_XY_SWAP_ON
      

/*==================================================================================================
                                  LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
// OJ's Operation Mode
typedef enum {
   MODE_MIN,
   MOUSE_MODE = MODE_MIN,
   DIRECTIONAL_PAD_MODE,
   JOYSTICK_MODE,
   MODE_MAX
} OJ_OPERATIONAL_MODE_T;


// Data structure for the crucialtec OJ driver
struct crucialtec_oj_data {
   uint16_t                 addr;
   struct i2c_client       *client;
   struct input_dev        *input_dev;
   uint8_t                  dev_id;
   struct work_struct       work;
   int                      motion_gpio;
   int                      shutdown_gpio;
   int                      reset_gpio;
   OJ_OPERATIONAL_MODE_T    operational_mode;
   uint8_t                  initialized;
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   uint8_t                  esd_polling;
#endif
   uint16_t                 valid_delta;
   uint16_t                 delay_between_key;
};

// Data structure for the I2c's read and write
typedef struct
{
   uint8_t reg;
   uint8_t data;
} crucialtec_oj_reg_data_t;

/*==================================================================================================
                                        LOCAL VARIABLES
==================================================================================================*/

// This contains the data for the Crucialtec Optical Joystick
static struct crucialtec_oj_data    oj_data;

// Register and data values for the default configuration
static crucialtec_oj_reg_data_t oj_check_regs[] =
{
   // Speed switching configuration
   {CRUCIALTEC_OJ_REG_SPEED_ST12, CRUCIALTEC_OJ_REG_SPEED_ST12_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST21, CRUCIALTEC_OJ_REG_SPEED_ST21_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST23, CRUCIALTEC_OJ_REG_SPEED_ST23_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST32, CRUCIALTEC_OJ_REG_SPEED_ST32_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST34, CRUCIALTEC_OJ_REG_SPEED_ST34_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST43, CRUCIALTEC_OJ_REG_SPEED_ST43_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST45, CRUCIALTEC_OJ_REG_SPEED_ST45_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_SPEED_ST54, CRUCIALTEC_OJ_REG_SPEED_ST54_DEFAULT_VALUE},
   
   // Assert/De-assert configuration
   {CRUCIALTEC_OJ_REG_AD_CTRL,     CRUCIALTEC_OJ_REG_AD_CTRL_VALUE_HIGHER_CPI},
   {CRUCIALTEC_OJ_REG_AD_ATH_HIGH, CRUCIALTEC_OJ_REG_AD_ATH_HIGH_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_AD_DTH_HIGH, CRUCIALTEC_OJ_REG_AD_DTH_HIGH_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_AD_ATH_LOW,  CRUCIALTEC_OJ_REG_AD_ATH_LOW_DEFAULT_VALUE},
   {CRUCIALTEC_OJ_REG_AD_DTH_LOW,  CRUCIALTEC_OJ_REG_AD_DTH_LOW_DEFAULT_VALUE},
   
   // Finger Presence Detect configuration
   {CRUCIALTEC_OJ_REG_FPD_CTRL,   CRUCIALTEC_OJ_REG_FPD_CTRL_DEFAULT_VALUE},

   // XY Quantization configuration
   {CRUCIALTEC_OJ_REG_QUANTIZE_CTRL, (CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_ON |
                                      CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_1 |
                                      CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_ON |
                                      CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_1 )},
   {CRUCIALTEC_OJ_REG_XYQ_THRESH, CRUCIALTEC_OJ_REG_XYQ_THRESH_VALUE_INDICATES_OFFSET_OF_LINEAR_REGION_DEFAULT},

   // LED Control configuration
   {CRUCIALTEC_OJ_REG_LED_CONTROL, CRUCIALTEC_OJ_REG_LED_CONTROL_VALUE_LED_NORMAL_OPERATION},

   {0,0}  /* Must end with 0 reg */
};

// Registers and data values for OJ's self test sequence
static crucialtec_oj_reg_data_t oj_self_test_seq[]=
{
   /* Perform a soft reset */
   { CRUCIALTEC_OJ_REG_SOFT_RESET,    CRUCIALTEC_OJ_REG_SOFT_RESET_VALUE_REVERT_TO_DEFAULT },  

   { CRUCIALTEC_OJ_REG_ENGINE,        (CRUCIALTEC_OJ_REG_ENGINE_VALUE_ENGINE_ON |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_SPEED_ON  |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_ASSERT_ON |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_XYQ_ON    |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_FINGER_ON |
                                       CRUCIALTEC_OJ_REG_ENGINE_VALUE_XY_SCALE_ON) },
                                        
   { CRUCIALTEC_OJ_REG_QUANTIZE_CTRL, (CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_ON              |
                                       CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_YQ_DIV_FACTOR_OF_2 |
                                       CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_ON              |
                                       CRUCIALTEC_OJ_REG_QUANTIZE_CTRL_VALUE_XQ_DIV_FACTOR_OF_2) },

   { CRUCIALTEC_OJ_REG_SPEED_CONTROL, (CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_ON      |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_ON      |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_8_MS |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_OFF     |
                                       CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_OFF)    },

   { CRUCIALTEC_OJ_REG_SELF_TEST,      CRUCIALTEC_OJ_REG_SELF_TEST_VALUE_TEST_ENABLE_ON},

   {0,0}  /* Must end with 0 reg */
};

/*==================================================================================================
                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
static int crucialtec_oj_read_register (struct i2c_client *client, uint8_t address, uint8_t *value);

static int crucialtec_oj_write_register (struct i2c_client *client, uint8_t address, uint8_t value);

static void crucialtec_oj_work_func (struct work_struct *work);

static irqreturn_t crucialtec_oj_irq_handler (int irq, void *dev_id);

static bool crucialtec_oj_self_test(struct i2c_client *client);

static int crucialtec_oj_toggle_reset_line(void);

static bool crucialtec_oj_check_registers(struct i2c_client *client);

static bool crucialtec_oj_init_hardware(struct i2c_client *client);

static int crucialtec_oj_probe (struct i2c_client *client, const struct i2c_device_id *id);

static int crucialtec_oj_remove (struct i2c_client *client);

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
bool crucialtec_oj_detect_esd(void);

void crucialtec_oj_check_esd(void* arg);

void crucialtec_oj_fix_esd(void);
#endif /* CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void crucialtec_oj_early_suspend(struct early_suspend *h);

static void crucialtec_oj_late_resume(struct early_suspend *h);

static struct early_suspend early_suspend_data = {
    .suspend = crucialtec_oj_early_suspend,
    .resume = crucialtec_oj_late_resume,
};
#endif

static int crucialtec_oj_suspend(struct i2c_client *client, pm_message_t msg);

static int crucialtec_oj_resume(struct i2c_client *client);


/*==================================================================================================
                                          LOCAL MACROS
==================================================================================================*/
#define DEBUG_PRINTK(format, a...)      \
   if (CRUCIALTEC_OJ_DEBUG_MSGS==1)        \
      printk(KERN_DEBUG format, ##a);   \


/*==================================================================================================
                                          LOCAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================

FUNCTION: crucialtec_oj_read_register

DESCRIPTION: 
   This function reads the value from the given register.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   uint8_t     address - addresd to read from
   uint8_t    *value   - read data from the register
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   If the return value is less than 1 then there is an error 
   else it returns the number of byte read.

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
static int 
crucialtec_oj_read_register (struct i2c_client *client, uint8_t address, uint8_t *value)
{
   uint8_t data[1];
   int rc;
   
   data[0] = address; 
   
   if ((rc = i2c_master_send(client, data, 1)) < 0)
   {
      dev_err(&client->dev, " %s(%s): i2c_master_send error %d\n", __FILE__, __FUNCTION__, rc);
      
      return (rc);
   }
   
   msleep_interruptible(1);
   
   *value = 0;
   
   if ((rc = i2c_master_recv(client, value, 1)) < 0)
   {
      dev_err(&client->dev, " %s(%s): i2c_master_recv error %d\n", __FILE__, __FUNCTION__, rc);
   }
   
   return (rc);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_write_register

DESCRIPTION: 
   This function writes the data to the given register.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   uint8_t     address - addresd to write to 
   uint8_t     value   - value to write to the address
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   If the return value is less than 1 then there is an error 
   else it returns the number of byte read.

DEPENDENCIES:
   I2c driver is ready to accept command.

SIDE EFFECTS:
   None

==================================================================================================*/
static int
crucialtec_oj_write_register (struct i2c_client *client, uint8_t address, uint8_t value)
{
   int rc;
   crucialtec_oj_reg_data_t msg;
   
   msg.reg = address;
   msg.data = value;
   
   if ((rc = i2c_master_send(client, (uint8_t *)&msg, sizeof(msg))) < 0)
   {
      dev_err(&client->dev, " %s(%s): i2c_master_send error %d\n", __FILE__, __FUNCTION__, rc);
   }
   
   return (rc);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_misc_device_open

DESCRIPTION: 
   This function checks to make sure that the OJ's device driver is initialized. 

ARGUMENTS PASSED:
   struct inode *inode
   struct file *file
 
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   If the return value is less than 1 then OJ driver is not initialized 
   else return value is 0 

DEPENDENCIES:
   OJ's driver is initialized.

SIDE EFFECTS:
   None

==================================================================================================*/
static int 
crucialtec_oj_misc_device_open (struct inode *inode, struct file *file)
{
   if (!oj_data.initialized)
      return -ENODEV;

   return 0;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_misc_device_release

DESCRIPTION: 
   This is the release function for the OJ's misc device.

ARGUMENTS PASSED:
   struct inode *inode
   struct file *file

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   Returns zero.

DEPENDENCIES:
   OJ's driver is initialized.

SIDE EFFECTS:
   None

==================================================================================================*/
static int 
crucialtec_oj_misc_device_release (struct inode *inode, struct file *file)
{
   return 0;
}

/*==================================================================================================

FUNCTION: crucialtec_oj_misc_device_read

DESCRIPTION: 
   This function reads and print out the current OJ's settings:
   Mode, Valid Delta and Delay Between Key Press.

   via adb shell
   cmd: cat /dev/oj 
 
ARGUMENTS PASSED:
   struct file *file
   char   __user *buf  -- buffer from user space
   size_t count        -- size of buffer from user space
   loff_t *ppos
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
    Returns the size of the buffer from user space.

DEPENDENCIES:
   OJ's driver is initialized.

SIDE EFFECTS:
   None

==================================================================================================*/
static ssize_t
crucialtec_oj_misc_device_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
   char temp[CRUCIALTEC_OJ_MISC_TEMP_BUFFER_SIZE];

   if (file->private_data)
      return 0;

   sprintf(temp, 
	   "OJ MODE=%d  VALID DELTA=%d  DELAY Between Key(ms)=%d\n", 
           oj_data.operational_mode, 
           oj_data.valid_delta, 
           oj_data.delay_between_key);

   if (copy_to_user(buf, temp, strlen(temp)))
      return -EFAULT;

   file->private_data = (void *)1;

   return strlen(temp);
}

/*==================================================================================================

FUNCTION: crucialtec_oj_misc_device_write

DESCRIPTION: 
   This function configures the OJ's settings via adb shell

   cmd: 
   echo "[MODE] [VALID DELTA] [DELAY BETWEEN KEY]" > /dev/oj
  
     [MODE]: 0 = Mouse, 1 = Directional Pad Mode, 2 = Joystick Mode
     [VALID DELTA]: positive integer
     [DELAY BETWEEN KEY]: positive integer

ARGUMENTS PASSED:
   struct file *fp
   char   __user *buf  -- buffer from user space
   size_t count        -- size of buffer from user space
   loff_t *pos
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   Returns the size of the buffer from user space.

DEPENDENCIES:
   OJ's driver is initialized.

SIDE EFFECTS:
   None

==================================================================================================*/
static ssize_t
crucialtec_oj_misc_device_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
   char cmd[CRUCIALTEC_OJ_MISC_TEMP_BUFFER_SIZE];

   int mode, delta, delay;

   memset(cmd, 0, CRUCIALTEC_OJ_MISC_TEMP_BUFFER_SIZE);

   if (copy_from_user((char*)cmd, buf, count))
      return -EFAULT;

   cmd[count] = '\0';
   
   if (cmd[count-1] == '\n')
   {
      cmd[count-1] = '\0';
   }
 
   if ( (sscanf(cmd, "%d %d %d", &mode, &delta, &delay)) == CRUCIALTEC_OJ_MISC_DEVICE_WRITE_PARAMETER_LIMIT )
   {
      DEBUG_PRINTK("OJ MODE=%d  VALID DELTA=%d  DELAY Between Key(ms)=%d\n",
                   mode, 
                   delta, 
                   delay);

      if ((mode >= MODE_MIN) && (mode < MODE_MAX))
      {
         oj_data.operational_mode = mode; 
      }
 
      if (delta >= 0) 
      {
         oj_data.valid_delta = delta; 
      }

      if (delay >= 0)
      {
         oj_data.delay_between_key = delay;
      } 
   }

   return count;
}


static const struct file_operations crucialtec_oj_fops = {
   .owner     = THIS_MODULE,
   .read      = crucialtec_oj_misc_device_read,
   .write     = crucialtec_oj_misc_device_write,
   .open      = crucialtec_oj_misc_device_open,
   .release   = crucialtec_oj_misc_device_release,
};


static struct miscdevice crucialtec_oj_miscdev = {
   .minor     = MISC_DYNAMIC_MINOR,
   .name      = "oj",
   .fops      = &crucialtec_oj_fops,
};


/*==================================================================================================

FUNCTION: crucialtec_oj_work_func

DESCRIPTION: 
   This function reports the data from the OJ to the input system.

ARGUMENTS PASSED:
    Pointer to the optical finger navigation's work_struct structure.
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static void
crucialtec_oj_work_func (struct work_struct *work)
{
   uint8_t motion_status = 0;
   uint8_t current_key = 0;

   int8_t delta_x = 0;
   int8_t delta_y = 0;

   struct crucialtec_oj_data *oj = container_of(work, 
                                                struct crucialtec_oj_data,
                                                work);

   struct i2c_client *client = oj->client;
   
   DEBUG_PRINTK("CRUCIALTEC OJ: Servicing the motion interrupt\n");

   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_MOTION, &motion_status);
   DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_MOTION  0x%x\n", motion_status);

   if ( (motion_status & CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_MASK) == CRUCIALTEC_OJ_REG_MOTION_VALUE_MOTION_OCCURRED )
   {
      crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_DELTA_X, &delta_x);
      DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_DELTA_X  %d\n", delta_x);
         
      crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_DELTA_Y, &delta_y);
      DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_DELTA_Y  %d\n", delta_y);

      switch(oj->operational_mode)
      {
         case MOUSE_MODE:
            // For mouse/trackball 
            // Reports the relative delta X and delta Y values
            input_report_rel(oj->input_dev, REL_X, delta_x);       // report delta x
            input_report_rel(oj->input_dev, REL_Y, delta_y);       // report delta y
            input_sync(oj->input_dev);
            msleep_interruptible(oj_data.delay_between_key);
         
            break;
         
         case JOYSTICK_MODE:
            // For Joystick
            input_report_abs(oj->input_dev, ABS_X, delta_x);       // report delta x
            input_report_abs(oj->input_dev, ABS_Y, delta_y);       // report delta y
            input_sync(oj->input_dev);
            msleep_interruptible(oj_data.delay_between_key);
            break;
            
         case DIRECTIONAL_PAD_MODE:
            // For directional pad keys
            if (delta_x != delta_y)  // no diagonal movement
            {
               // distance traveled must be greater than max 
               if ( (abs(delta_x) >= oj_data.valid_delta) || 
                    (abs(delta_y) >= oj_data.valid_delta) )   
               {
                  if (abs(delta_x) > abs(delta_y))
                  {
                     // x-asix movement
                     // positive x-asix movement -- report down key
                     // negative x-asix movement -- report up key
                     current_key = (delta_x > 0) ? DPAD_RIGHT_KEY : DPAD_LEFT_KEY;
                  }
                  else  
                  {
                     // y-axis movement
                     // positive y-axis movement -- report right key
                     // negative y-axis movement -- report left key
                     current_key = (delta_y > 0) ? DPAD_DOWN_KEY : DPAD_UP_KEY;
                  }
               }

               if (current_key != 0)
               {

                  input_report_key(oj->input_dev, current_key, 1);    // report key
                  DEBUG_PRINTK("CRUCIALTEC OJ: input report key pressed - scancode %d\n", current_key);
                  input_sync(oj->input_dev);
               }
            }            
            break;
      
         default:
            break;
      }

      // Key release message in DIRECTIONAL_PAD_MODE Mode
      if ((oj->operational_mode == DIRECTIONAL_PAD_MODE) && (current_key != 0))
      {

         input_report_key(oj->input_dev, current_key, 0);   // report key released
         input_sync(oj->input_dev);

         msleep_interruptible(oj_data.delay_between_key);
      }
   }
   
   // clear the motion register
   crucialtec_oj_write_register(client, CRUCIALTEC_OJ_REG_MOTION, 0x00);
   
   enable_irq(client->irq);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_irq_handler

DESCRIPTION: 
   This function services the Optical Finger Nagivation's Motion GPIO line.

ARGUMENTS PASSED:
   int   irq     - The interrupt request line to service.
   void *dev_id  - The pointer to the device to service.
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   IRQ_HANDLED - The interrupt request has been handled.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static irqreturn_t 
crucialtec_oj_irq_handler (int irq, void *dev_id)
{
   struct crucialtec_oj_data *oj = (struct crucialtec_oj_data *)dev_id;
   
   disable_irq(irq);
   
   schedule_work(&oj->work);
   
   return IRQ_HANDLED;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_self_test

DESCRIPTION: 
   This function performs the self-test on the OJ system.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   true  - No error in the self-test
   false - Error with the self-test

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static bool
crucialtec_oj_self_test(struct i2c_client *client)
{
   bool ret = false;

   uint8_t i = 0;

   uint8_t crc0_value, crc1_value, crc2_value, crc3_value = 0;

   crucialtec_oj_reg_data_t  msg;
   crucialtec_oj_reg_data_t *cfg;
   
   cfg = oj_self_test_seq;
   
   for (i=0; cfg[i].reg !=0; i++)
   {
      msg.reg  = cfg[i].reg;
      msg.data = cfg[i].data;
      
      if ((ret = i2c_master_send(client, (uint8_t *)&msg, sizeof(msg))) < 0)
      {
         dev_err(&client->dev, " %s(%s): i2c_master_send error %d\n", __FILE__, __FUNCTION__, ret);
      }
      
      msleep_interruptible(1);
   }

   msleep_interruptible(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_SELF_TEST);

   // Read the self-test CRC registers
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_CRC0, &crc0_value);
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_CRC1, &crc1_value);
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_CRC2, &crc2_value);
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_CRC3, &crc3_value);

   DEBUG_PRINTK("CRUCIALTEC OJ: Reg CRUCIALTEC_OJ_REG_CRC0 0x%x == 0x%x\n", 
                CRUCIALTEC_OJ_REG_CRC0_SELF_TEST_VALID_VALUE, 
                crc0_value);

   DEBUG_PRINTK("CRUCIALTEC OJ: Reg CRUCIALTEC_OJ_REG_CRC1 0x%x == 0x%x\n", 
                CRUCIALTEC_OJ_REG_CRC1_SELF_TEST_VALID_VALUE, 
                crc1_value);

   DEBUG_PRINTK("CRUCIALTEC OJ: Reg CRUCIALTEC_OJ_REG_CRC2 0x%x == 0x%x\n", 
                CRUCIALTEC_OJ_REG_CRC2_SELF_TEST_VALID_VALUE, 
                crc2_value);

   DEBUG_PRINTK("CRUCIALTEC OJ: Reg CRUCIALTEC_OJ_REG_CRC3 0x%x == 0x%x\n", 
                CRUCIALTEC_OJ_REG_CRC3_SELF_TEST_VALID_VALUE, 
                crc3_value);

   if ( (crc0_value == CRUCIALTEC_OJ_REG_CRC0_SELF_TEST_VALID_VALUE) &&
        (crc1_value == CRUCIALTEC_OJ_REG_CRC1_SELF_TEST_VALID_VALUE) &&
        (crc2_value == CRUCIALTEC_OJ_REG_CRC2_SELF_TEST_VALID_VALUE) && 
        (crc3_value == CRUCIALTEC_OJ_REG_CRC3_SELF_TEST_VALID_VALUE) )
   {
     ret = true;
   }

   return(ret);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_toggle_reset_line

DESCRIPTION: 
   This function toggles the reset line as part of the initialization procedure.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int
crucialtec_oj_toggle_reset_line(void)
{
   int err = 0;

   // Request for the use of the reset line
   err = gpio_request(oj_data.reset_gpio, "oj_rst_n");	
 
   if (err) 
   {
      printk(KERN_ERR "crucialtec_oj_toggle_reset_line: crucialtec_oj_toggle_reset_line: gpio_request failed for input %d\n", 
             oj_data.reset_gpio);
      return false;
   }

   // Set the reset line to low
   err = gpio_direction_output(oj_data.reset_gpio, 0);
 
   if (err)
   {
      printk(KERN_ERR "crucialtec_oj_toggle_reset_line: crucialtec_oj_toggle_reset_line: gpio_direction_output for input %d\n", 
             oj_data.reset_gpio);
      return false;
   }
   
   // Let the OJ's reset line stay low for awhile
   udelay(CRUCIALTEC_OJ_DELAY_RESET_LINE_FROM_LOW_TO_HIGH);
    
   // Set the reset Line to high
   err = gpio_direction_output(oj_data.reset_gpio, 1);
 
   if (err)
   {
      printk(KERN_ERR "crucialtec_oj_toggle_reset_line: crucialtec_oj_toggle_reset_line: gpio_direction_output %d\n", 
             oj_data.reset_gpio);
      return false;
   }

   // Waiting for the driver to be ready
   msleep_interruptible(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY);

   return true;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_check_registers

DESCRIPTION: 
   This function checks the default values in the OJ's registers.  This is part of the init process.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   true  - matches default values
   false - doesn't match default values

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static bool
crucialtec_oj_check_registers(struct i2c_client *client)
{
   int i;
   uint8_t value = 0;
   crucialtec_oj_reg_data_t *cfg;
   
   cfg = oj_check_regs;

   DEBUG_PRINTK("--- CRUCIALTEC OJ: Read Check Registers Configuration - start ---------->\n");
   
   for (i=0; cfg[i].reg !=0; i++)
   {
      crucialtec_oj_read_register(client, cfg[i].reg, &value);
      DEBUG_PRINTK("CRUCIALTEC OJ: Reg 0x%x - Table Value 0x%x - Read Value 0x%x\n", cfg[i].reg, cfg[i].data, value);
      
      if (cfg[i].data != value)
      {
         printk(KERN_ERR "crucialtec_oj_check_registers: default register check failure: Reg 0x%x - default value is 0x%x - read value is - 0x%x\n", 
                cfg[i].reg, cfg[i].data, value);
         return false;
      }
   }

   DEBUG_PRINTK("--- CRUCIALTEC OJ: Read Check Registers Configuration - end ----------->\n");

   return true;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_configure_registers

DESCRIPTION: 
   This function configures some of the main OJ's registers.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void
crucialtec_oj_configure_registers(struct i2c_client *client)
{
   uint8_t value = 0;

   // Set the Engine register:
   //     Optical Finger Navigation Engine -- ON 
   //     Speed Switching -- ON
   crucialtec_oj_write_register(client, 
                                CRUCIALTEC_OJ_REG_ENGINE, 
                                CRUCIALTEC_OJ_REG_ENGINE_VALUE_DEFAULT);

   // Set Resolution:
   //    Wakeup Resolution from reset mode-- 500cpi
   //    Default Resolution -- 500cpi
   crucialtec_oj_write_register(client,
                                CRUCIALTEC_OJ_REG_RESOLUTION,
                                (CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_WAKEUP_RES_500_CPI |
                                  CRUCIALTEC_OJ_REG_RESOLUTION_VALUE_RES_500_CPI) );

   // Set Speed control:
   //     Y Scaling factor -- OFF
   //     X Scaling factor -- OFF
   //     Speed switching checking interval -- 16ms
   //     Low cpi when in speed switching -- 250cpi
   //     Hi cpi when in speed switching -- Disable
   crucialtec_oj_write_register(client,
                                CRUCIALTEC_OJ_REG_SPEED_CONTROL,
                                (CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_Y_SCALE_OFF |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_X_SCALE_OFF |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_SP_INT_VAL_16_MS |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_LOW_CPI_ON_250_CPI |
                                 CRUCIALTEC_OJ_REG_SPEED_CONTROL_VALUE_HIGH_CPI_OFF) );

   // Check OJ's registers
   // crucialtec_oj_check_registers(client);

   // Read these registers after the driver configuration
   //  CRUCIALTEC_OJ_REG_MOTION (0x02), CRUCIALTEC_OJ_REG_DELTA_X (0x03), CRUCIALTEC_OJ_REG_DELTA_Y (0x04)

   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_MOTION, &value);
   DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_MOTION Reg 0x02 - Value 0x%x\n", value);
   
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_DELTA_X, &value);
   DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_DELTA_X Reg 0x03 - Value 0x%x\n", value);

   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_DELTA_Y, &value);
   DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_DELTA_Y Reg 0x04 - Value 0x%x\n", value);
   
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_ENGINE, &value);
   DEBUG_PRINTK("CRUCIALTEC OJ: Register CRUCIALTEC_OJ_REG_ENGINE Reg 0x60 - Value 0x%x\n", value);


   // Swap the X and Y values 
   if (CRUCIALTEC_OJ_ORIENTATION_POSITION_SWAP_X_TO_Y)
   {
      crucialtec_oj_write_register(client, 
                                   CRUCIALTEC_OJ_REG_ORIENTATION_CTRL, 
                                   CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_DEFAULT);
   }
}


/*==================================================================================================

FUNCTION: crucialtec_oj_init_hardware

DESCRIPTION: 
   This function initializes and configures the OJ driver.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static bool
crucialtec_oj_init_hardware(struct i2c_client *client)
{
   bool err = true;

   /////  Configure MOTION GPIO line   /////
   
   // Request for the use of the OJ's motion GPIO line
   if (gpio_request(oj_data.motion_gpio, "oj_motion_gpio"))
   {
      printk(KERN_ERR "crucialtec_oj_init_hardware: gpio_request failed for input %d\n", oj_data.motion_gpio);
      return false;
   }

   // Configure the MOTION GPIO as an input for the IRQ handler
   if (gpio_direction_input(oj_data.motion_gpio))
   {
      printk(KERN_ERR "crucialtec_oj_init_hardware: gpio_direction_input failed for input %d\n", oj_data.motion_gpio);
      return false;
   }
   

   /////   Configure SHUTDOWN GPIO line   /////

   // Set shutdown line to low
   if (gpio_request(oj_data.shutdown_gpio, "oj_shutdown_gpio"))
   {
      printk(KERN_ERR "crucialtec_oj_init_hardware: gpio_request failed for input %d\n", oj_data.shutdown_gpio);
      return false;
   }

   if (gpio_direction_output(oj_data.shutdown_gpio, 0))
   {
      printk(KERN_ERR "crucialtec_oj_init_hardware: gpio_direction_output %d\n", oj_data.shutdown_gpio);
      return false;
   }


   /////    Toggle the reset line   ////

   if (!(crucialtec_oj_toggle_reset_line()))
   {
      dev_err(&client->dev, " %s(%s): Cannot toggle the reset line \n", __FILE__, __FUNCTION__);
      return(false);
   }

   // Perform a self-test
   if (CRUCIALTEC_OJ_PERFORM_SELF_TEST_DURING_INIT)
   {
      if (!(crucialtec_oj_self_test(client)))
      {
         dev_err(&client->dev, " %s(%s): Self-test error \n", __FILE__, __FUNCTION__);
         return(false);
      }
   }
   
   // Configure the registers
   crucialtec_oj_configure_registers(client);
	
   return(err);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_probe

DESCRIPTION: 
   This function initializes and registers the optical joystick device.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   i2c_device_id *id   - pointer to the device's id
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int
crucialtec_oj_probe (struct i2c_client *client, const struct i2c_device_id *id)
{
   int err = 0;
   struct crucial_oj_platform_data *pdev = NULL;
   
   uint8_t product_id = 0;
   uint8_t revision_id = 0;   
   
   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
   {
      dev_err(&client->dev, " %s(%s): No supported I2C function \n", __FILE__, __FUNCTION__);
      err = (-ENOTSUPP);
      goto probe_error_free_gpio;
   }

   pdev = (struct crucial_oj_platform_data *)client->dev.platform_data;
   
   if (!pdev)
   {
      dev_err(&client->dev, " %s(%s): platform data not set\n", __FILE__, __FUNCTION__);
      err = -EFAULT;
      goto probe_error_free_gpio;
   }
   
   oj_data.client = client;
   
   oj_data.motion_gpio   = pdev->gpio_motion_irq;
   oj_data.shutdown_gpio = pdev->gpio_shutdown; 
   oj_data.reset_gpio    = pdev->gpio_reset;
   
   // Set the OJ's data 
   i2c_set_clientdata(client, &oj_data);
   
   // Init and configure the OJ's hardware
   if (!(crucialtec_oj_init_hardware(client)))
   {
      dev_err(&client->dev, " %s(%s): Hardware initialization error \n", __FILE__, __FUNCTION__);
      err = (-EFAULT);
      goto probe_error_free_clientdata;
   }

   // Init the work function
   INIT_WORK(&oj_data.work, crucialtec_oj_work_func);
   
   
   // Configure and Register as an Input Device //
   
   // Allocate the device
   oj_data.input_dev = input_allocate_device();
   
   if (oj_data.input_dev == NULL)
   {
      dev_err(&client->dev, " %s(%s): Cannot allocate input device \n", __FILE__, __FUNCTION__);
      err = (-ENOMEM);
      goto probe_error_free_clientdata;
   }

   // Get the Product ID and and Revision ID
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_PRODUCT_ID, &product_id);
   crucialtec_oj_read_register(client, CRUCIALTEC_OJ_REG_REVISION_ID, &revision_id);

   // Set the information of the device
   oj_data.input_dev->name           = pdev->name;
   oj_data.input_dev->id.bustype     = BUS_I2C;
   oj_data.input_dev->id.vendor      = CRUCIALTEC_VENDOR_ID; 
   oj_data.input_dev->id.product     = product_id;
   oj_data.input_dev->id.version     = revision_id;

   // Init the data we are tracking 
   oj_data.operational_mode  = DIRECTIONAL_PAD_MODE; //MOUSE_MODE;
   oj_data.valid_delta       = CRUCIALTEC_OJ_MAX_DISTANCE_TRAVELED; 
   oj_data.delay_between_key = CRUCIALTEC_OJ_MAX_DELAY_BETWEEN_KEY;

   // Configure the properties of this device for the Input system
   set_bit(EV_KEY,         oj_data.input_dev->evbit);        // device has keys
   set_bit(EV_SW,          oj_data.input_dev->evbit); 
   
   set_bit(DPAD_UP_KEY,    oj_data.input_dev->keybit);       // keys for this device
   set_bit(DPAD_DOWN_KEY,  oj_data.input_dev->keybit);
   set_bit(DPAD_RIGHT_KEY, oj_data.input_dev->keybit);
   set_bit(DPAD_LEFT_KEY,  oj_data.input_dev->keybit);

   set_bit(EV_REL,         oj_data.input_dev->evbit);        // device has relative event
   set_bit(REL_X,          oj_data.input_dev->relbit);       // relative data for this device
   set_bit(REL_Y,          oj_data.input_dev->relbit);

   set_bit(BTN_MOUSE,      oj_data.input_dev->keybit);
   
   set_bit(EV_ABS,         oj_data.input_dev->evbit);
   input_set_abs_params(oj_data.input_dev, ABS_X, -128, 127, 0, 0);
   input_set_abs_params(oj_data.input_dev, ABS_Y, -128, 127, 0, 0);

   // Register the device
   err = input_register_device(oj_data.input_dev);

   if (err)
   {
      dev_err(&client->dev, " %s(%s): Unable to register %s input device\n",
              __FILE__, __FUNCTION__, oj_data.input_dev->name);

      goto probe_error_free_device;
   }

   // Set the IRQ handler
   client->irq = gpio_to_irq(oj_data.motion_gpio);

   if (client->irq)
   {
      err = request_irq(client->irq, 
                        crucialtec_oj_irq_handler, 
                        CRUCIALTEC_OJ_MOTION_GPIO_IRQ_REQUEST_FLAG,
                        CRUCIALTEC_OJ_PRODUCT_NAME,
                        &oj_data);

      if (err == 0)
      {
         err = set_irq_wake(client->irq, 1);
      }
      else
      {
         dev_err(&client->dev, " %s(%s): Request IRQ  %d  failed\n",
                 __FILE__, __FUNCTION__, client->irq);

         goto probe_error_free_irq;         
      }
   }

   if (CRUCIALTEC_OJ_MISC_DEVICE_SUPPORT==1)
   {
      err = misc_register (&crucialtec_oj_miscdev);
      
      if (err) 
      {
         dev_err(&client->dev, " %s(%s): Unable to register %s misc device\n",
                 __FILE__, __FUNCTION__, oj_data.input_dev->name);
   
         goto probe_error_free_irq;
      }
   }

   // sync the device information
   input_sync(oj_data.input_dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend (&early_suspend_data);
#endif

   oj_data.initialized = 1; 

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   esd_poll_start(crucialtec_oj_check_esd, 0);

   oj_data.esd_polling = 1;
#endif

   return(err);



probe_error_free_irq:
   free_irq(client->irq, &oj_data);

probe_error_free_device:
   input_free_device(oj_data.input_dev);

probe_error_free_clientdata:
   i2c_set_clientdata(client, NULL);

probe_error_free_gpio:
   gpio_free(oj_data.reset_gpio);
   gpio_free(oj_data.shutdown_gpio);

   return (err);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_suspend

DESCRIPTION: 
   This function suspends the OJ device.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
   pm_message_t msg    - power management message
       
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int
crucialtec_oj_suspend(struct i2c_client *client, pm_message_t msg)
{
   struct crucialtec_oj_data *oj = i2c_get_clientdata(client);

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   if (oj_data.esd_polling == 1)
   {
      esd_poll_stop(crucialtec_oj_check_esd);
      oj_data.esd_polling = 0;
   }
#endif

   disable_irq(client->irq);
   
   cancel_work_sync(&oj->work);
   
   // set the shutdown line to high
   if (gpio_direction_output(oj_data.shutdown_gpio, 1))
   {
      printk(KERN_ERR "crucialtec_oj_suspend: gpio_direction_output %d\n", oj_data.shutdown_gpio);
      return false;
   }
   
   return 0;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_resume

DESCRIPTION: 
   This function resumes the OJ device.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int
crucialtec_oj_resume(struct i2c_client *client)
{
   // set the shutdown line to low
   if (gpio_direction_output(oj_data.shutdown_gpio, 0))
   {
      printk(KERN_ERR "crucialtec_oj_resume: gpio_direction_output %d\n", oj_data.shutdown_gpio);
      return false;
   }

   // Waiting for the driver to be ready
   msleep_interruptible(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SUSPEND);
   
   enable_irq(client->irq);

#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   if (oj_data.esd_polling == 0)
   {
      esd_poll_start(crucialtec_oj_check_esd, 0);
      oj_data.esd_polling = 1;
   }
#endif

   return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void crucialtec_oj_early_suspend (struct early_suspend *h)
{
    crucialtec_oj_suspend (oj_data.client, PMSG_SUSPEND);
}

static void crucialtec_oj_late_resume (struct early_suspend *h)
{
    crucialtec_oj_resume (oj_data.client);
}
#endif


/*==================================================================================================

FUNCTION: crucialtec_oj_remove

DESCRIPTION: 
   This function removes the OJ device from the input system.

ARGUMENTS PASSED:
   i2c_client *client  - pointer to the device's data
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
static int
crucialtec_oj_remove (struct i2c_client *client)
{
   struct crucialtec_oj_data *oj = i2c_get_clientdata(client);
   
   dev_dbg(&client->dev, "%s: enter. \n", __FUNCTION__);
   
   free_irq(client->irq, oj);
   
   gpio_free(oj_data.reset_gpio);
   
   gpio_free(oj_data.shutdown_gpio);
   
   input_unregister_device(oj->input_dev);
  
   if (CRUCIALTEC_OJ_MISC_DEVICE_SUPPORT==1)
   {
      misc_deregister(&crucialtec_oj_miscdev);
   }
 
#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
   if (oj_data.esd_polling == 1)
   {
      esd_poll_stop(crucialtec_oj_check_esd);
      oj_data.esd_polling = 0;
   }
#endif

   kfree(oj);
   
   return 0;
}



#ifdef CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY
/*==================================================================================================

FUNCTION: crucialtec_oj_detect_esd

DESCRIPTION: 
   This function determines if the OJ's registers is invalid or not.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   true  - OJ's registers is invalid
   false - OJ's registers is correct

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
bool
crucialtec_oj_detect_esd()
{
   uint8_t engine_reg_value = 0;
   uint8_t orientation_ctrl = 0;

   crucialtec_oj_read_register(oj_data.client, 
                               CRUCIALTEC_OJ_REG_ENGINE, 
                               &engine_reg_value);

   crucialtec_oj_read_register(oj_data.client, 
                               CRUCIALTEC_OJ_REG_ORIENTATION_CTRL, 
                               &orientation_ctrl);
	
	if ((engine_reg_value != CRUCIALTEC_OJ_REG_ENGINE_VALUE_DEFAULT) ||
	    (orientation_ctrl != CRUCIALTEC_OJ_REG_ORIENTATION_CTRL_VALUE_DEFAULT))
	{
	   printk (KERN_ERR "%s: invalid register value detected\n", __FUNCTION__);
		
		return true;
	}
	
	return false;
}


/*==================================================================================================

FUNCTION: crucialtec_oj_fix_esd

DESCRIPTION: 
   This function resets the OJ and configures the neccessary registers.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void
crucialtec_oj_fix_esd()
{
   struct crucialtec_oj_data *oj = i2c_get_clientdata(oj_data.client);

	// Perform a sof reset
	crucialtec_oj_write_register(oj_data.client, 
                                CRUCIALTEC_OJ_REG_SOFT_RESET, 
                                CRUCIALTEC_OJ_REG_SOFT_RESET_VALUE_REVERT_TO_DEFAULT);

   // Waiting for the driver to be ready
   msleep_interruptible(CRUCIALTEC_OJ_DELAY_FOR_SYSTEM_TO_BE_READY_AFTER_SOFT_RESET);

   // Configure the registers
	crucialtec_oj_configure_registers(oj_data.client);
}


/*==================================================================================================

FUNCTION: crucialtec_oj_fix_esd

DESCRIPTION: 
   This function checks for invalid values in the OJ's registers and correct them.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/
void 
crucialtec_oj_check_esd(void* arg)
{
   if(crucialtec_oj_detect_esd() == true)
	{
	    crucialtec_oj_fix_esd();
	}
}

#endif  /* CONFIG_CRUCIALTEC_OJ_ESD_RECOVERY */



static const struct i2c_device_id crucialtec_oj_id[] = {
   {CRUCIALTEC_OJ_PRODUCT_NAME, 0},
   {},
};


static struct 
i2c_driver crucialtec_oj_driver = {
   .id_table = crucialtec_oj_id,
#ifndef CONFIG_HAS_EARLYSUSPEND
   .suspend  = crucialtec_oj_suspend,
   .resume   = crucialtec_oj_resume,
#endif
   .probe    = crucialtec_oj_probe,
   .remove   = crucialtec_oj_remove,
   .driver   = {
      .name  = CRUCIALTEC_OJ_PRODUCT_NAME
   },
};


static int 
__devinit crucialtec_oj_init(void)
{
   return i2c_add_driver(&crucialtec_oj_driver);
}


static void 
__exit crucialtec_oj_exit(void)
{
   i2c_del_driver(&crucialtec_oj_driver);
}


subsys_initcall(crucialtec_oj_init);
module_exit(crucialtec_oj_exit);


MODULE_DESCRIPTION("CRUCIALTEC OPTICAL JOYSTICK DRIVER");
MODULE_AUTHOR("Vinh Vo <vinh.vo@motorola.com>");
MODULE_LICENSE("GPL");
