#ifndef _MOT_DEV_H_
#define _MOT_DEV_H_

/*==================================================================================================
                                                                               
   Module Name:  mot_dev.c

   DESCRIPTION: Drivers for managing Motorola device nodes.
   
====================================================================================================
                               Motorola Confidential Proprietary
                           Advanced Technology and Software Operations
                        (c) Copyright Motorola 2009, All Rights Reserved
  

Revision History:
                            Modification     Tracking
Author                          Date          Number     Description of Changes
-------------------------   ------------    ----------   -------------------------------------------
Harpreet Sangha (w36210)     09/22/2009     LIBtt14093   Added Version device node.
Harpreet Sangha (w36210)     04/25/2009     LIBss30959   Initial creation.

==================================================================================================*/

/*==================================================================================================
                                               MACROS
==================================================================================================*/

/*========================================= GPIO Device ==========================================*/

#define MOT_DEV_GPIO "/dev/gpio"

/*========================================= PUPD Device ==========================================*/

#define MOT_DEV_PUPD "/dev/pupd"

/*========================================= Version Device ==========================================*/

#define MOT_DEV_VERSION "/dev/version"


/*==================================================================================================
                                               ENUMS
==================================================================================================*/

/*========================================= GPIO Device ==========================================*/

enum mot_dev_gpio_cmd
{
    MOT_DEV_GPIO_CMD_GET,
    MOT_DEV_GPIO_CMD_SET,
};

/*========================================= PUPD Device ==========================================*/

enum mot_dev_pupd_cmd
{
    MOT_DEV_PUPD_CMD_RESET,
    MOT_DEV_PUPD_CMD_SET,
    MOT_DEV_PUPD_CMD_CLR,
    MOT_DEV_PUPD_CMD_CHK,
    MOT_DEV_PUPD_CMD_GET,
};

/*========================================= Version Device ==========================================*/

enum mot_dev_version_cmd
{
    MOT_DEV_VERSION_CMD_GET_HW,
};


/*==================================================================================================
                                             STRUCTURES
==================================================================================================*/

/*========================================= GPIO Device ==========================================*/

struct gpio_get_arg
{
    unsigned gpio;
    int value;
};

struct gpio_set_arg
{
    unsigned gpio;
    int value;
};

/*========================================= PUPD Device ==========================================*/

struct pupd_set_arg
{
    unsigned long reason;
};

struct pupd_clr_arg
{
    unsigned long reason;
};

struct pupd_chk_arg
{
    unsigned long reason;
    unsigned char value;
};

struct pupd_get_arg
{
    unsigned long reason;
};

/*========================================= Version Device ==========================================*/

struct pupd_get_hw_arg
{
    unsigned long upid;
    unsigned long crid;
};


#endif /* _MOT_DEV_H_ */
