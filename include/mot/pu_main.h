#ifndef PU_MAIN_H
#define PU_MAIN_H

#ifdef __cplusplus                      /* allow #include in a C++ file */
extern "C" {
#endif

/*==================================================================================================
                                                                                
                                         HEADER DESCRIPTION

                       Oemsbl Version:     /vobs/engine_pupd/code/pupd/hdr/pu_main.h
                       
                       Kernel Version:     ../kernel/include/mot/pu_main.h
                       
                       Usbloader Version:  ../android/vendor/mot/calgary/boot/pu_main.h
                       
====================================================================================================

                                    Motorola Confidential Proprietary
                                Pan American Cellular Subscriber Group
                            (c) Copyright Motorola 2006, All Rights Reserved
     
Revision History:
                            Modification     Tracking
Author                          Date          Number     Description of Changes
-------------------------   ------------    ----------   -------------------------------------------
Hua Mao (w50084)             11/24/2009     LIBtt48610   Add PU_MAIN_USB_DATA_CABLE_PU
Rafael Traje (w20142)        05/13/2009     LIBss45760   Add PU_MAIN_FASTBOOT_MODE_PU
Rafael Traje (w20142)        04/22/2009     LIBss35094   Add Recovery & FOTA power up reasons
Harpreet Sangha (w36210)     03/30/2009     LIBss23221   Moved all handover area declarations and
                                                         API's to a common file.
Harpreet Sangha (w36210)     02/24/2009     LIBss05587   Ported the following CRs:
                                            LIBrr87392   Reimplemented handover area.
Harpreet Sangha (w36210)     02/16/2009     LIBss02247   Modified for Calgary Android.
Curt Scott                   01/26/2006     LIBhh80348   Merge in LIBgg32888.
Jeongsoo Kim (w18570)        06/17/2005     LIBgg32888   Added PU_IN_AIRPLANE_MODE and PU_MAIN_AIRPLANE_MODE_PU 
Stephen Dickey (wlsd10)      01/30/2004     LIBdd70216   Updated to support 80500 from PCC.
Vinh Vo                      06/20/2002     LIBbb39885   Added DL_PD_CP_OFFLINE_SUCCESS for 
                                                          the call processing offline state confirmation
                                                          for SUSPEND Test Command
Tim McCune                   01/31/2002     LIBbb18324   Added handling of HIP6 to pupd
                                                         -Added enum type PU_MAIN_PROCESSOR_TYPE
                                                         -Added global pu_boot_processor_rev
                                                         -Added PU_MAIN_PROCESSOR_TECHNOLOGY_MASK
                                                         -Added PU_MAIN_TECH_HIP6
David Steh                   05/14/2001     CSGce77285   Update CPU Exception handling for p2k 
                                                         -Add define's  PU_MAIN_RUN_LEVEL
James Stokebrand             07/27/2001     LIBbb01343   Restored the PU_MAIN_POWERUP_REASON_TYPE to UINT8
James Stokebrand             07/23/2001     LIBbb00764   Fixed the priority numbering of PU_MAIN_POWERUP_REASON_TYPE
James Stokebrand             07/17/2001     LIBbb00313   Changed PU_MAIN_POWERUP_REASON_TYPE to UINT32.
Binu Abraham                 01/31/2001     CSGce86925   Added cause DL_PD_PARTIAL_SUSPEND
w19179                       12/13/2001                  Support for P2K
Vinh Vo                      11/30/2001     LIBbb13796   Powerup into the last state on a panic, 
                                                          power cut, watchdog timeout, or abnormal reset.
Vinh Vo                      11/18/2001     LIBbb12574   Disable power regulators in charge only mode
Vinh Vo                      11/13/2001     LIBbb08901   Move definitions of panic code for CDMA1X to 
                                                         panic_codes.h in the product_config directory
Vinh Vo                      11/07/2001     LIBbb11243   Add code support for reset after the exit 

Vinh Vo                      10/16/2001     LIBaa01852   Added panic ID for QCOM's error fatal messages
Vinh Vo                      10/09/2001     LIBbb07798   Fixed code for powerdown into flash mode
Vinh Vo                      10/02/2001     LIBbb01056   Added code support for powering up from charge only mode.
Vinh Vo                      09/27/2001     CSGce77671   Added support for power cut.
Vinh Vo                      09/10/2001     LIBbb05054   Added code support for flash mode
David Steh                   05/14/2001     CSGce77285   Update CPU Exception handling for p2k 
                                                         -Add define's  PU_MAIN_RUN_LEVEL
James Stokebrand             07/27/2001     LIBbb01343   Restored the PU_MAIN_POWERUP_REASON_TYPE to UINT8
James Stokebrand             07/23/2001     LIBbb00764   Fixed the priority numbering of PU_MAIN_POWERUP_REASON_TYPE
James Stokebrand             07/17/2001     LIBbb00313   Changed PU_MAIN_POWERUP_REASON_TYPE to UINT32.
Vinh Vo                      05/23/2001     LIBaa01888   Merge PUPD code inspection changes to i8-97 merge.
Vinh Vo                      04/15/2001     CSGce92602   Power Up by PWR_END key
Vinh Vo                      03/05/2001     CSGce89915   Power UP for CDMA1X	
Binu Abraham                 01/31/2001     CSGce86925   Added cause DL_PD_PARTIAL_SUSPEND
Courtney Plater              10/19/2000     CSGce74034   Added @log spec. message types to message ids
Steve Petrie                 11/30/2000     CSGce81734   Add recursive powerdown type
Barry Wester                 11/15/2000     CSGce80542   Fix charge only mode
Barry Wester                 11/08/2000     CSGce76093   Log WDOG timeout on next powerup.
                                                         - Added suPanic Codes for PUPD.
Joel Voss                    10/26/2000     CSGce78044   Battery Issues After Flashing
Courtney Plater              10/19/2000     CSGce74034   Added @log spec. message types to message ids
Joel Voss                    10/16/2000     CSGce76632   Priority of powerup reasons is incorrect
Barry Wester                 08/30/2000     CSGce58171   Decrease PU_MAIN_POWERUP_REASON_TYPE to UINT8
Joel Voss                    10/12/2000     CSGce76365   pu_powerdown_report() called before seem_init()
Barry Wester                 08/17/2000     CSGce69540   Remove CDMA boot block dependencies
Barry Wester                 08/15/2000     CSGce68690   Change Conditional Compiles to new standard
Joel Voss                    09/01/2000     CSGce71443   Add charger powerup reason support
Barry Wester                 03/17/2000     CSGce39458   Update pu_main function prototypes.
Justin Eltoft                08/21/1998                  File Creation

PRODUCT NAMES: P2K

GENERAL DESCRIPTION: Powerup functions

This file contains the headers for the functions pu_main, pu_main_restart, pu_main_wally,
set_vector_base_register, and get_vector_base_register.

NOTE: The functions are located in specific sections of memory with the help of
the cdma_link.cmd file.

TRACEABILITY MATRIX:
   None

====================================================================================================
                                           INCLUDE FILES
==================================================================================================*/
#include <mot/mot_comdef.h>

/*==================================================================================================
                                             CONSTANTS
==================================================================================================*/
/* The PUPD Panic codes are reserved for: 0x801c0 - 0x801df */
#define PU_WATCHDOG_EXPIRATION    0x801c0

    /* Mask for determining processor technology (CDR3, HIP6, etc) */
#define PU_MAIN_PROCESSOR_TECHNOLOGY_MASK (0x07<<8)
#define PU_MAIN_TECH_HIP6 (1<<8)

/*==================================================================================================
                                              MACROS
==================================================================================================*/

/*==================================================================================================
                                               ENUMS
==================================================================================================*/

typedef enum
{
    /*Lowest Priority */
    PU_MAIN_NO_INDICATION_YET_PU = 0,
    PU_MAIN_WDOG_TIMEOUT_PU          = 1 << 0,
    PU_MAIN_PANIC_RESET_PU           = 1 << 1,
    PU_MAIN_POWERCUT_RESET_PU        = 1 << 2,
    PU_MAIN_SOFTWARE_RESET_PU        = 1 << 3,
    PU_MAIN_SHORT_POWER_KEY_PRESS_PU = 1 << 4,
    PU_MAIN_USB_DATA_CABLE_PU        = 1 << 5,
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

enum
{
    DL_PD_SOFT_RESET = 0,
    DL_PD_FULL_POWERDOWN,
    DL_PD_SUSPEND,
    DL_PD_FLASH_MODE,
    DL_PD_PARTIAL_SUSPEND
};

typedef UINT16 DL_PD_CAUSE_TYPE;

enum
{
    DL_PD_POWERDOWN_SUCCESS = 0,
    DL_PD_SUICIDE_TMR_EXPIRED
#if (MAKE_MSM5100_CHIPSET)
    ,DL_PD_CP_OFFLINE_SUCCESS
#endif /* MAKE_MSM5100_CHIPSET */
};
typedef UINT16 DL_PD_STATUS_TYPE;

/* These message id's are allocated to the device layer originally */
typedef enum
{
   DL_PD_REQ_MSG_ID = 0x83FFE,    /* @LOG DL_PD_CAUSE_TYPE */
   DL_PD_CNF_MSG_ID = 0x83FFF     /* @LOG DL_PD_CNF_MSG */
} DL_PD_MSG_ID_TYPE;

/* Powerup/down is allocated msg id's from 0x80200 to 0x8023f */
typedef enum
{
   ENGINE_DL_PU_REASON_MSG_ID = 0x80200
} ENGINE_DL_PU_REASON_MSG_ID_TYPE;

typedef enum
{
   PU_MAIN_POWERDOWN_ALWAYS = 0,
   PU_MAIN_RESET_ALWAYS,
   PU_MAIN_CHECK_IGNITION,
   PU_MAIN_ENTER_FLASH_MODE,
   PU_MAIN_PRE_SEEM_INIT_PD,
   PU_MAIN_CHECK_CHARGE_ONLY_MODE,
   PU_MAIN_EXIT_CHARGE_ONLY_MODE,
   PU_MAIN_NO_LOG_POWERDOWN
} PU_MAIN_POWERDOWN_TYPE;

/* run levels for cpu exception handler   */
typedef enum
{
   PU_MAIN_RUN_LEVEL_INITIAL = 0,
   PU_MAIN_RUN_LEVEL_SUAPI_INITED = 16,
   PU_MAIN_RUN_LEVEL_PANIC1 = 32,
   PU_MAIN_RUN_LEVEL_PANIC2,
   PU_MAIN_RUN_LEVEL_PANIC3
}PU_MAIN_RUN_LEVEL;

typedef enum
{
   PU_MAIN_PROCESSOR_PATRIOT_CDR3=0,
   PU_MAIN_PROCESSOR_PATRIOT_HIP6,
   PU_MAIN_PROCESSOR_PATRIOT_HIP7,
   PU_MAIN_PROCESSOR_WALLY
}PU_MAIN_PROCESSOR_TYPE;

/* this structure is defined by the OS and is OS dependent.  
 * It will likely need to be updated for every instance of the
 * OS that is updated.
 */
typedef struct  TCB {
  int vmc[7];                   /* Size of Task Control Block */
} TCB, *TCB_PTR;


/*==================================================================================================
                                             STRUCTURES
==================================================================================================*/
typedef struct
{
   DL_PD_CAUSE_TYPE powerdown_cause;
}  DL_PD_CAUSE_MSG;

typedef struct
{
   DL_PD_STATUS_TYPE pd_status;
}  DL_PD_CNF_MSG;

typedef struct
{
   PU_MAIN_POWERUP_REASON_TYPE powerup_reason;
}  ENGINE_DL_PU_REASON_MSG;

/*==================================================================================================
                                    GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/
extern PU_MAIN_RUN_LEVEL pu_main_run_level;
extern PU_MAIN_PROCESSOR_TYPE pu_boot_processor_rev;

/*==================================================================================================
                                        FUNCTION PROTOTYPES
==================================================================================================*/

#if (MAKE_MSM5105_CHIPSET||MAKE_MSM5100_CHIPSET)

/*==================================================================================================

FUNCTION: jump_to_flash_mode

DESCRIPTION: This function transfers the control of the phone to the bootloader and the phone 
   is placed in flash mode.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   This function will not return.

==================================================================================================*/

extern void jump_to_flash_mode(void);


/*==================================================================================================
FUNCTION: pu_check_initial_states_on_powerup

DESCRIPTION: This function is used only during power up to check on what is the condition for 
             power up.  This will check the initial states of the power key and external power upon
             power up.  It will power down the phone if there is no legitimate reason to continue 
             with power up.

ARGUMENTS PASSED:
   None
    
REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None.

SIDE EFFECTS:
   None.

==================================================================================================*/

extern void pu_check_initial_states_on_powerup(void);



/*==================================================================================================

FUNCTION: pu_send_power_up_reason_msg 

DESCRIPTION: 
   Send a power up reason msg. 

ARGUMENTS PASSED:
   None
  
RETURN VALUE:
   None

PRE-CONDITIONS:
   None
 
POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
 
extern void pu_send_power_up_reason_msg(void);

#endif /* MAKE_MSM5105_CHIPSET||MAKE_MSM5100_CHIPSET */


/*==================================================================================================

FUNCTION: pu_main_powerdown

DESCRIPTION: This is the non-panic powerdown function to be called by the device layer when
they are ready to powerdown the radio.  They must provide the type of powerdown they want,
and know that if they request a reset type powerdown, they must keep track of this and
powerup properly after the reset with the default powerup reason "PU_MAIN_SOFTWARE_RESET_PU"

ARGUMENTS PASSED:
   PU_MAIN_POWERDOWN_TYPE    pd_reason

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   This must be called after all tasks that need to be notified of powerdowns are notified.
Also, the device layer must keep track of things like reseting and powering back up into
charge only mode, as mentioned above in the description.

SIDE EFFECTS:
   None

==================================================================================================*/

extern void pu_main_powerdown(PU_MAIN_POWERDOWN_TYPE pd_reason);

/*==================================================================================================
FUNCTION:
    pu_main_reset_powerup_reason

DESCRIPTION:
   This function clears out the power-up reason, typically only used to indicate an intention to
   power-down.

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

extern void pu_main_reset_powerup_reason(void);

/*==================================================================================================

FUNCTION:
   pu_main_set_powerup_reason

DESCRIPTION:
   This function is to be used by anyone wishing to remove a reason(s) from the current power-up
   reason mask. There is no need to know what the current power-up reason is because all power-up
   reasons have a priority (greater value is higher priority) based on there value in pu_main.h.

ARGUMENTS PASSED:
   pu_reason - Power-up reason to add.

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

extern void pu_main_set_powerup_reason(PU_MAIN_POWERUP_REASON_TYPE pu_reason);

/*==================================================================================================

FUNCTION:
   pu_main_clr_powerup_reason

DESCRIPTION:
   This function is to be used by anyone wishing to remove a reason(s) from the current power-up
   reason mask. There is no need to know what the current power-up reason is because all power-up
   reasons have a priority (greater value is higher priority) based on there value in pu_main.h.

ARGUMENTS PASSED:
   pu_reason - Power-up reason to remove.

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

extern void pu_main_clr_powerup_reason(PU_MAIN_POWERUP_REASON_TYPE pu_reason);

/*==================================================================================================

FUNCTION:
   pu_main_chk_powerup_reason

DESCRIPTION:
   This function is to be used by anyone wishing to compare a reason(s) against the current
   power-up reason mask.

ARGUMENTS PASSED:
   pu_reason - Power-up reason to remove.

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   BOOLEAN - True if the passed in reason(s) are a part of the current power-up reason mask.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

extern BOOLEAN pu_main_chk_powerup_reason(PU_MAIN_POWERUP_REASON_TYPE pu_reason);

/*==================================================================================================

FUNCTION:
   pu_main_get_powerup_reason

DESCRIPTION:
   This function is to be used by anyone wishing to read the highest priority reason in the current
   power-up reason mask.

ARGUMENTS PASSED:
   None

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   pu_reason - The highest priority power-up reason in the current power-up reason mask.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

extern PU_MAIN_POWERUP_REASON_TYPE pu_main_get_powerup_reason (void);

/*==================================================================================================
FUNCTION: pu_main_set_run_level

DESCRIPTION: This function sets the run level of the subscriber unit.

INPUTS:  newlevel - new run level of the subscriber unit

OUTPUTS: None

IMPORTANT NOTES:
     The global pu_main_run_level is also updated(directly) inside suJTPalErrorException() located
     in su_palerrorexception.s.   
====================================================================================================*/
extern void pu_main_set_run_level(PU_MAIN_RUN_LEVEL);

/*==================================================================================================
FUNCTION: pu_main_get_run_level

DESCRIPTION:  This function returns the current run level of the subscriber unit.

INPUTS:  N/A

RETURN VALUE: 
 PU_MAIN_RUN_LEVEL pu_main_run_level  

IMPORTANT NOTES:  None.
 ====================================================================================================*/
extern PU_MAIN_RUN_LEVEL pu_main_get_run_level(void);
/*=============================================================================================

FUNCTION: pu_main

DESCRIPTION: This function is the power up routine for sierra hardware.

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
extern void pu_main (void);
/*================================================================================================*/

#ifdef __cplusplus
}
#endif
#endif  /* PU_MAIN_H */

