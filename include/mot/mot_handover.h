#ifndef MOT_HANDOVER_H
#define MOT_HANDOVER_H


/*==================================================================================================
                                                                               
     Header Name: mot_handover.h

     General Description: Globals and definitions for managing the handover area in RAM.
 
      Ported from: /vobs/engine_qcom_android/AMSS/products/mot/mot_handover.h

====================================================================================================
                              Motorola Confidential Proprietary
                         Advanced Technology and Software Operations
                      (c) Copyright Motorola 2007, All Rights Reserved
     

Revision History:
                            Modification     Tracking
Author                          Date          Number     Description of Changes
-------------------------   ------------    ----------   -------------------------------------------
Rafael Traje (w20142)        04/23/2009     LIBss30959   Port to Android 
Harpreet Sangha (w36210)     03/30/2009     LIBss23221   Initial creation.


====================================================================================================
                                         INCLUDE FILES
==================================================================================================*/
   
#include <mot/mot_comdef.h>
#include <mot/pu_main.h>
#include <mot/hw_config.h>

/*==================================================================================================
                                           CONSTANTS
==================================================================================================*/
/* None */


/*==================================================================================================
                                            MACROS
==================================================================================================*/
/* None */


/*==================================================================================================
                                             ENUMS
==================================================================================================*/
/* None */


/*==================================================================================================
                                 STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

typedef struct 
{
  /*********************************************************************/
  /* Variables shared between the Bootloader and Phone Code.           */
  /*********************************************************************/

  /* Universal Product ID */
  HW_CONFIG_CURRENT_UPID upid;
  /* Carrier ID */
  HW_CONFIG_CURRENT_CRID crid;

  /* Indiate the powerup reason */
  PU_MAIN_POWERUP_REASON_TYPE pu_main_powerup_reason;

} mot_handover_phone_data_t;

typedef struct 
{
  /* this is the structure containing all data that the
   * phone code hands over to itself, or to the bootloader
   */
  mot_handover_phone_data_t phone_handover;

  /* this is the checksum for every byte of data in the phone_handover
   * area.
   */
  UINT32 phone_handover_csum;


} mot_handover_data_t;

/* For backwards compatibility for phone code */
typedef mot_handover_phone_data_t PU_MAIN_PHONE_HANDOVER_PANIC_STRUCT;
typedef mot_handover_data_t PU_MAIN_HANDOVER_PANIC_STRUCT;

/* For backwards compatibility for bootloader */
typedef mot_handover_phone_data_t FL_BL_PHONE_HANDOVER_PANIC_STRUCT;
typedef mot_handover_data_t FL_BL_HANDOVER_PANIC_STRUCT;

/*==================================================================================================
                                 GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/
void mot_handover_reset_powerup_reason( void );
void mot_handover_set_powerup_reason( PU_MAIN_POWERUP_REASON_TYPE pu_reason );
void mot_handover_clr_powerup_reason( PU_MAIN_POWERUP_REASON_TYPE pu_reason );
BOOLEAN mot_handover_chk_powerup_reason( PU_MAIN_POWERUP_REASON_TYPE pu_reason );
PU_MAIN_POWERUP_REASON_TYPE mot_handover_get_powerup_reason( void );
void mot_handover_set_upid( HW_CONFIG_CURRENT_UPID upid );
HW_CONFIG_CURRENT_UPID mot_handover_get_upid( void );
void mot_handover_set_crid( HW_CONFIG_CURRENT_CRID crid );
HW_CONFIG_CURRENT_CRID mot_handover_get_crid( void );


/*================================================================================================*/


#endif  /* MOT_HANDOVER_H */
