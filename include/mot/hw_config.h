

#ifndef HW_CONFIG_H
#define HW_CONFIG_H
/*===============================================================================================

  Module Name:  hw_config.h

  General Description:    
        This is the hardware configuration header file.
        
  Ported From:  /vobs/engine_qcom_android/AMSS/products/mot/hw_config.h

=================================================================================================
                            Motorola Confidential Proprietary
                       Advanced Technology and Software Operations
                    (c) Copyright Motorola 2004-2007, All Rights Reserved


  Revision History:
  Modification     Tracking
  Author                          Date          Number     Description of Changes
  -------------------------   ------------    ----------   --------------------------------------
  Rafael Traje (w20142)       04/23/2009      LIBss30959   Port to Android
  Harpreet Sangha (w36210)    03/30/2009      LIBss23221   Updated HW_CONFIG_CURRENT_UPID() and
                                                           HW_CONFIG_UPID_PHONE_MASK().
                                                           Added CRID_T, HW_CONFIG_CURRENT_CRID,
                                                           and HW_CONFIG_CURRENT_CRID().
  Harpreet Sangha (w36210)    03/23/2009      LIBss20568   Removed TEMP flags.
  Harpreet Sangha (w36210)    03/19/2009      LIBss18377   Added TEMP flag:
                                                           - GPIO and HAPI tables need to be
                                                             investigated.
  Harpreet Sangha (w36210)    11/24/2008      LIBrr42622   Added TEMP flag:
                                                           - Set Calgary's UPID to match Blaze
                                                             until all checks have been removed.

===============================================================================================*/

/******************************************************************************


     UPID_T definition description

      0 x 1  1  0  0
          ^  ^  ^  ^
          |  |  |  |
          |  |  |  +- board revision.
          |  |  +---- Reserved
          |  +------- hardware type ( 0 is kenai, 1 is paris, 2 is singapore, 3 is kyoto, 4 is diamondback and etc) 
          +---------- Chipset ( 1 is msm6550 and etc)



*****************************************************************************/
enum UPID_T {
  UPID_KENAI_P2_PORTABLE          = 0x1000,
  UPID_KENAI_P3_PORTABLE          = 0x1001,
  UPID_PARIS_GENERIC_PORTABLE     = 0x1100,
  UPID_SINGAPORE_GENERIC_PORTABLE = 0x1200,
  UPID_ATHENS_P1_MORTABLE         = 0x1401,
  UPID_ATHENS_P1_PORTABLE         = 0x1405,
  UPID_PAU_M1_MORTABLE         = 0x1501,
  UPID_PAU_P1_PORTABLE         = 0x1502,
  UPID_DAVOS_M1_MORTABLE       = 0x1600,
  UPID_CHARLOTTE_M0_MORTABLE   = 0x1800,
  UPID_CHARLOTTE_M1_MORTABLE   = 0x1801,
  UPID_CHARLOTTE_D1_PORTABLE   = 0x1810,
  UPID_CHARLOTTE_D2_PORTABLE   = 0x1811,
  UPID_CHARLOTTE_P1_PORTABLE   = 0x1812,
  UPID_WATERTOWN_P0_PORTABLE   = 0x1700,
  UPID_ENCINITAS_P1_PORTABLE   = 0x2600,
  UPID_BLAZE_M0_MORTABLE       = 0x2700,
  UPID_HALO_M1_MORTABLE        = 0x2800,
  UPID_HALO_P1_PORTABLE        = 0x2801,
  UPID_VEGAS_P0_PORTABLE       = 0x1301,
  UPID_UTOPIA_P0_PORTABLE      = 0x2100,
  UPID_UTOPIA_P1_PORTABLE      = 0x2101,
  UPID_CALGARY_P0_MORTABLE     = 0x3000,
  UPID_PITTSBURGH_P0_MORTABLE  = 0x4000,
  
  /* add new entries above this line.  make sure you add a corresponding
   * table entry to hw_config_table[] in hw_config.c
   */
  UPID_NO_VALUE                    = 0xFFFF
};

enum UPID_MASK_T {
  UPID_KENAI_MASK                 = 0x1000,
  UPID_PARIS_MASK                 = 0x1100,
  UPID_SINGAPORE_MASK             = 0x1200,
  UPID_ATHENS_MASK                = 0x1400,
  UPID_PAU_MASK                   = 0x1500,
  UPID_DAVOS_MASK                 = 0x1600,
  UPID_CHARLOTTE_MASK             = 0x1800,
  UPID_ENCINITAS_MASK             = 0x2600,
  UPID_HALO_MASK                  = 0x2800,
  UPID_WATERTOWN_MASK             = 0x1700,
  UPID_VEGAS_MASK                 = 0x1300,
  UPID_UTOPIA_MASK                = 0x2100,
  UPID_BLAZE_MASK                 = 0x2700,
  UPID_CALGARY_MASK               = 0x3000,
  UPID_PITTSBURGH_MASK            = 0x4000,
  UPID_PHONE_MASK                 = 0xFF00
};

enum CRID_T {
  CRID_ROW                        = 0x0000,
  CRID_VZW                        = 0x0001,
  CRID_SPRINT                     = 0x0002,
  
  /* add new entries above this line.  make sure you add a corresponding
   * table entry to hw_config_table[] in hw_config.c
   */
  CRID_NO_VALUE                    = 0xFFFF
};

typedef unsigned char  HW_CONFIG_CONTRAST_TYPE;
typedef unsigned char  HW_CONFIG_COLUMN_TYPE;
typedef unsigned char  HW_CONFIG_REG_RES_TYPE;
typedef unsigned char  HW_CONFIG_SBCM_CURR_TYPE;
typedef unsigned char  HW_CONFIG_SBCM_BL_OFFSET_TYPE;
typedef long int       HW_CONFIG_SBCM_TEMP_OFFSET_TYPE;
typedef unsigned long int HW_CONFIG_OPTION_TYPE;
typedef unsigned char  HW_CONFIG_CHRG_CTRL_TYPE;
typedef unsigned char  HW_CONFIG_PA_PIN_MAP_TYPE;
typedef unsigned char  HW_CONFIG_VALID_AUDIO_TYPE;
typedef unsigned char  HW_CONFIG_DISPLAY_PRODUCT_TYPE;
typedef unsigned short HW_CONFIG_CURRENT_UPID;
typedef unsigned short HW_CONFIG_CURRENT_CRID;
typedef unsigned int   HW_CONFIG_GPIO_SIG_TYPE;


#endif  /* HW_CONFIG_H */
