#ifndef PU_MAIN_H
#define PU_MAIN_H

/*==================================================================================================
                                                                                
                                         HEADER DESCRIPTION
                                         
                       Ported from /vobs/engine_pupd/code/pupd/hdr/pu_main.h

====================================================================================================

                                    Motorola Confidential Proprietary
                                Pan American Cellular Subscriber Group
                            (c) Copyright Motorola 2006, All Rights Reserved
     
Revision History:
                            Modification     Tracking
Author                          Date          Number     Description of Changes
-------------------------   ------------    ----------   -------------------------------------------
Rafael Traje (w20142)        04/23/2009     LIBss30959   Ported pu_main.h to Android
Harpreet Sangha (w36210)     03/30/2009     LIBss23221   Moved all handover area declarations and
                                                         API's to a common file.
Harpreet Sangha (w36210)     02/24/2009     LIBss05587   Ported the following CRs:
                                            LIBrr87392   Reimplemented handover area.
Harpreet Sangha (w36210)     02/16/2009     LIBss02247   Modified for Calgary Android.

====================================================================================================
                                           INCLUDE FILES
==================================================================================================*/


/*==================================================================================================
                                             CONSTANTS
==================================================================================================*/

/*==================================================================================================
                                               ENUMS
==================================================================================================*/


/* 
 * These power up reasons are shared between the 7XXX OEMSBL ARM9 
 * and the Android APPSBL ARM11 bootloaders via shared memory.
 */
typedef enum
{
    /*Lowest Priority */
    PU_MAIN_NO_INDICATION_YET_PU = 0,
    PU_MAIN_WDOG_TIMEOUT_PU          = 1 << 0,
    PU_MAIN_PANIC_RESET_PU           = 1 << 1,
    PU_MAIN_POWERCUT_RESET_PU        = 1 << 2,
    PU_MAIN_SOFTWARE_RESET_PU        = 1 << 3,
    PU_MAIN_SHORT_POWER_KEY_PRESS_PU = 1 << 4,
    PU_MAIN_CAR_IGNITION_PU          = 1 << 5,
    PU_MAIN_ACC_PWR_KEY_PRESS_PU     = 1 << 6,
    PU_MAIN_CHARGER_PU               = 1 << 7,
    PU_MAIN_AIRPLANE_MODE_PU         = 1 << 8,
    PU_MAIN_PWR_KEY_PRESS_PU         = 1 << 9,
    PU_MAIN_EXTERNAL_PWR_PU          = 1 << 10,
    PU_MAIN_FASTBOOT_MODE_PU         = 1 << 11, 
    PU_MAIN_RECOVERY_FORCE_MODE_PU   = 1 << 12, 
    PU_MAIN_RECOVERY_OTA_MODE_PU     = 1 << 13, 
    PU_MAIN_FOTA_OTA_MODE_PU         = 1 << 14, 
    PU_MAIN_FLASH_MODE_PU            = 1 << 15,
    /*Highest Priority */
    PU_MAIN_POWRUP_REASON_MAX
} PU_MAIN_POWERUP_REASON_TYPE;


#endif /* PU_MAIN_H */


