/*==================================================================================================
                                                                               
   Module Name:  mot_handover.c

   DESCRIPTION: API's for managing the handover area in RAM.
   
   Ported from: /vobs/engine_qcom_android/AMSS/products/mot/mot_handover.c

====================================================================================================
                               Motorola Confidential Proprietary
                           Advanced Technology and Software Operations
                        (c) Copyright Motorola 2009, All Rights Reserved
  

Revision History:
                            Modification     Tracking
Author                          Date          Number     Description of Changes
-------------------------   ------------    ----------   -------------------------------------------
Rafael Traje (w20142)        04/23/2009     LIBss30959   Ported to Android
Harpreet Sangha (w36210)     03/30/2009     LIBss23221   Initial creation.


Portability: Portable to other compilers. 


====================================================================================================
                                        INCLUDE FILES
==================================================================================================*/

#include <linux/string.h>
#include <mot/mot_handover.h>
#if defined(CONFIG_MACH_CALGARY)
#include <mach/smem.h>
#elif defined(CONFIG_MACH_PITTSBURGH)
#include <mach/smem.h>
#endif /* defined(CONFIG_MACH_PITTSBURGH) */

/*==================================================================================================
                                 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
static BOOLEAN mot_handover_init( void );
static void mot_handover_reset( void );
static BOOLEAN mot_handover_validate_checksum( void );
static void mot_handover_perform_checksum( void );


/*==================================================================================================
                                     LOCAL CONSTANTS
==================================================================================================*/
/* None. */                                                            


/*==================================================================================================
                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/* None. */                                                            


/*==================================================================================================
                                        LOCAL MACROS
==================================================================================================*/
#define PU_MAIN_POWERUP_REASON mot_handover_data->phone_handover.pu_main_powerup_reason
#define PU_MAIN_UPID mot_handover_data->phone_handover.upid
#define PU_MAIN_CRID mot_handover_data->phone_handover.crid


/*==================================================================================================
                                      LOCAL VARIABLES
==================================================================================================*/
static mot_handover_data_t* mot_handover_data;
                                                            

/*==================================================================================================
                                     GLOBAL VARIABLES
==================================================================================================*/
/* None. */                                                            


/*==================================================================================================
                                     LOCAL FUNCTIONS
==================================================================================================*/

/*=====================================================================

FUNCTION: mot_handover_sum_data

DESCRIPTION: sum the data pointed to, size specified, uint32 wide.

ARGUMENTS PASSED:
   UINT8 *data_ptr: pointer to the data
   UITN32 data_size:the size to sum

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   UINT32: returns the sum, 32 bits wide.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

=======================================================================*/
static __inline UINT32 mot_handover_sum_data( UINT8 *data_ptr, UINT32 data_size )
{
  unsigned int  sum = 0;
  unsigned long offset = 0;

  /* read each byte in the block, and add it into the
   * checksum. Assumes the checksum field itself is blank!
   */
  for ( offset = 0; offset < data_size; offset ++ )
    {
      sum += data_ptr[offset];
    }

  return( sum );
}

/*==================================================================================================

FUNCTION: pu_main_handover_init

DESCRIPTION:
   Initialize handover area.

ARGUMENTS PASSED:
   None

RETURN VALUE:
   BOOLEAN - TRUE if initialization was successful, FALSE otherwise.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static BOOLEAN mot_handover_init( void )
{
  BOOLEAN ret = TRUE;

  if ( !mot_handover_data )
  {

#if defined(CONFIG_MACH_CALGARY)
    /* Allocate handover area from SMEM */
    mot_handover_data = (mot_handover_data_t *)smem_alloc(SMEM_MOT_HANDOVER, sizeof(mot_handover_data_t));
#elif defined(CONFIG_MACH_PITTSBURGH)
    /* Allocate handover area from SMEM */
    mot_handover_data = (mot_handover_data_t *)smem_alloc(SMEM_MOT_HANDOVER, sizeof(mot_handover_data_t));
#endif /* defined(CONFIG_MACH_PITTSBURGH) */

    /* If we failed to retrieve the handove area, return a failure */
    if ( !mot_handover_data )
    {
      ret = FALSE;
    }

    /* Validate checksum and reset the handover area if it fails */
    if ( ret )
    {
      if ( !mot_handover_validate_checksum() )
      {
        mot_handover_reset();
      }
    }
  }

  return ret;
}

/*==================================================================================================

FUNCTION: mot_handover_reset

DESCRIPTION:
   Clear out the data in the handover area.

ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   This function assumes that addresses are 32 bits.

==================================================================================================*/
static void mot_handover_reset( void )
{
  BOOLEAN err = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  if ( !err )
  {
    /* clear out all the data */
    memset(&mot_handover_data->phone_handover, 0, sizeof(FL_BL_PHONE_HANDOVER_PANIC_STRUCT));

    /* Reset values of UPID and CRID are non-zero */
    PU_MAIN_UPID = UPID_NO_VALUE;
    PU_MAIN_CRID = CRID_NO_VALUE;

    /* update the checksum */
    mot_handover_perform_checksum();
  }
}

/*==================================================================================================

FUNCTION: mot_handover_validate_checksum

DESCRIPTION:
   Check whether the data passed us by the phone code is valid 

ARGUMENTS PASSED:
   None

RETURN VALUE:
   BOOLEAN

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   This function assumes that addresses are 32 bits.

==================================================================================================*/
static BOOLEAN mot_handover_validate_checksum( void )
{
  BOOLEAN err = FALSE;
  BOOLEAN ret = FALSE;
  register UINT32 csum;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  /* verify the checksum is correct */
  if ( !err )
  {
    csum = mot_handover_sum_data
    (
      (UINT8 *)&mot_handover_data->phone_handover,
      sizeof(FL_BL_PHONE_HANDOVER_PANIC_STRUCT)
    );

    if ( mot_handover_data->phone_handover_csum == ( 0xFFFF-csum ) )
    {
      ret = TRUE;
    }
  }

  return ret;
}

/*==================================================================================================

FUNCTION: mot_handover_perform_checksum

DESCRIPTION:
   Check whether the data passed us by the phone code is valid 

ARGUMENTS PASSED:
   None

RETURN VALUE:
   BOOLEAN

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   This function assumes that addresses are 32 bits.

==================================================================================================*/
static void mot_handover_perform_checksum( void )
{
  BOOLEAN err = FALSE;
  register UINT32 csum;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  /* set the checksum so it is correct */
  if ( !err )
  {
    csum = mot_handover_sum_data
    (
      (UINT8 *)&mot_handover_data->phone_handover,
      sizeof(FL_BL_PHONE_HANDOVER_PANIC_STRUCT)
    );

    mot_handover_data->phone_handover_csum = ( 0xFFFF-csum );
  }
}


/*==================================================================================================
                                       GLOBAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================
FUNCTION:
    mot_handover_reset_powerup_reason

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

void mot_handover_reset_powerup_reason( void )
{
  BOOLEAN err = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  if ( !err )
  {
    /* Update power-up reason */
    PU_MAIN_POWERUP_REASON = PU_MAIN_NO_INDICATION_YET_PU;

    /* must perform a checksum on the data, if we update the phone_handover area */
    mot_handover_perform_checksum();
  }
}

/*==================================================================================================

FUNCTION:
   mot_handover_set_powerup_reason

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

void mot_handover_set_powerup_reason( PU_MAIN_POWERUP_REASON_TYPE pu_reason )
{
  BOOLEAN err = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  if ( !err )
  {
    /* Update power-up reason */
    PU_MAIN_POWERUP_REASON |= pu_reason;

    /* must perform a checksum on the data, if we update the phone_handover area */
    mot_handover_perform_checksum();
  }
}

/*==================================================================================================

FUNCTION:
   mot_handover_clr_powerup_reason

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

void mot_handover_clr_powerup_reason( PU_MAIN_POWERUP_REASON_TYPE pu_reason )
{
  BOOLEAN err = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  if ( !err )
  {
    /* Update power-up reason */
    PU_MAIN_POWERUP_REASON &= ~pu_reason;

    /* must perform a checksum on the data, if we update the phone_handover area */
    mot_handover_perform_checksum();
  }
}

/*==================================================================================================

FUNCTION:
   mot_handover_chk_powerup_reason

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

BOOLEAN mot_handover_chk_powerup_reason( PU_MAIN_POWERUP_REASON_TYPE pu_reason )
{
  BOOLEAN err = FALSE;
  BOOLEAN ret = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  if ( !err )
  {
    /* Compare power-up reason */
    if ( PU_MAIN_POWERUP_REASON & pu_reason )
    {
      ret = TRUE;
    }
  }

  return ret;
}


/*==================================================================================================

FUNCTION:
   mot_handover_get_powerup_reason

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

PU_MAIN_POWERUP_REASON_TYPE mot_handover_get_powerup_reason( void )
{
  BOOLEAN err = FALSE;
  PU_MAIN_POWERUP_REASON_TYPE ret = PU_MAIN_NO_INDICATION_YET_PU;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  if ( !err )
  {
    ret = PU_MAIN_POWRUP_REASON_MAX - 1;

    /* Retrieve the highest priority reason */
    while ( ret )
    {
      if ( ret & PU_MAIN_POWERUP_REASON )
      {
        break;
      }
      else
      {
        ret = ret >> 1;
      }
    }
  }

  return ret;
}

/*==================================================================================================

FUNCTION:
   mot_handover_set_upid

DESCRIPTION:
   This function is to be used by anyone wishing to write the UPID.

ARGUMENTS PASSED:
   upid - The Universal Product ID.

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

void mot_handover_set_upid( HW_CONFIG_CURRENT_UPID upid )
{
  BOOLEAN err = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  /* Only allow setting of UPID if it isn't already set */
  if ( !err )
  {
    if ( PU_MAIN_UPID != UPID_NO_VALUE)
    {
      err = TRUE;
    }
  }

  if ( !err )
  {
    /* Update UPID */
    PU_MAIN_UPID = upid;

    /* must perform a checksum on the data, if we update the phone_handover area */
    mot_handover_perform_checksum();
  }
}

/*==================================================================================================

FUNCTION:
   mot_handover_get_upid

DESCRIPTION:
   This function is to be used by anyone wishing to read the UPID.

ARGUMENTS PASSED:
   None

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   HW_CONFIG_CURRENT_UPID - The Universal Product ID.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

HW_CONFIG_CURRENT_UPID mot_handover_get_upid( void )
{
  BOOLEAN err = FALSE;
  HW_CONFIG_CURRENT_UPID ret = UPID_NO_VALUE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  /* Retrieve UPID */
  if ( !err )
  {
    ret = PU_MAIN_UPID;
  }

  return ret;
}

/*==================================================================================================

FUNCTION:
   mot_handover_set_crid

DESCRIPTION:
   This function is to be used by anyone wishing to write the CRID.

ARGUMENTS PASSED:
   crid - The Carrier ID.

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

void mot_handover_set_crid( HW_CONFIG_CURRENT_CRID crid )
{
  BOOLEAN err = FALSE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  /* Only allow setting of CRID if it isn't already set */
  if ( !err )
  {
    if ( PU_MAIN_CRID != CRID_NO_VALUE)
    {
      err = TRUE;
    }
  }

  if ( !err )
  {
    /* Update CRID */
    PU_MAIN_CRID = crid;

    /* must perform a checksum on the data, if we update the phone_handover area */
    mot_handover_perform_checksum();
  }
}

/*==================================================================================================

FUNCTION:
   mot_handover_get_crid

DESCRIPTION:
   This function is to be used by anyone wishing to read the CRID.

ARGUMENTS PASSED:
   None

REFERENCE ARGUMENTS PASSED:
   None

RETURN VALUE:
   HW_CONFIG_CURRENT_CRID - The Carrier ID.

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

==================================================================================================*/

HW_CONFIG_CURRENT_CRID mot_handover_get_crid( void )
{
  BOOLEAN err = FALSE;
  HW_CONFIG_CURRENT_CRID ret = CRID_NO_VALUE;

  /* Verify that the handover area is initialized */
  if ( !mot_handover_init() )
  {
    err = TRUE;
  }

  /* Retrieve CRID */
  if ( !err )
  {
    ret = PU_MAIN_CRID;
  }

  return ret;
}

/*=================================================================================================*/
