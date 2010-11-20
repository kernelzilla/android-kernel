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
                                           INCLUDE FILES
==================================================================================================*/

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#if defined(CONFIG_GENERIC_GPIO)
#include <linux/gpio.h>
#else /* defined(CONFIG_GENERIC_GPIO) */
#include <asm/gpio.h>
#endif /* defined(CONFIG_GENERIC_GPIO) */
#include <mot/mot_handover.h>
#include <mot/mot_dev.h>


/*==================================================================================================
                                           LOCAL MACROS
==================================================================================================*/

#define CMD_BUF_MAX_SIZE 1024

#define FUNC_NAME(d,c) \
    d ## _ ## c

#define VAR_NAME(d,c) \
    d ## _ ## c

#define DECLARE_CMDS(d) \
    static struct dev_cmd VAR_NAME(d,cmds)[]

#define DECLARE_IOCTL(d) \
    static int FUNC_NAME(d,ioctl)(struct inode *ip, struct file *fp, unsigned int cmd, unsigned long arg)

#define DEFINE_IOCTL(d) \
    DECLARE_IOCTL(d) \
    { \
        return mot_dev_ioctl(&VAR_NAME(d,context), cmd, arg); \
    }

#define DECLARE_OPEN(d) \
    static int FUNC_NAME(d,open)(struct inode *ip, struct file *fp)

#define DEFINE_OPEN(d) \
    DECLARE_OPEN(d) \
    { \
        return mot_dev_open(&VAR_NAME(d,context)); \
    }

#define DECLARE_RELEASE(d) \
    static int FUNC_NAME(d,release)(struct inode *ip, struct file *fp)

#define DEFINE_RELEASE(d) \
    DECLARE_RELEASE(d) \
    { \
        return mot_dev_release(&VAR_NAME(d,context)); \
    }

#define DECLARE_FOPS(d) \
    static struct file_operations VAR_NAME(d,fops)

#define DEFINE_FOPS(d) \
    DECLARE_FOPS(d) = \
    { \
        .owner = THIS_MODULE, \
        .ioctl = FUNC_NAME(d,ioctl), \
        .open = FUNC_NAME(d,open), \
        .release = FUNC_NAME(d,release), \
    }

#define DECLARE_DEV(d) \
    static struct miscdevice VAR_NAME(d,dev)

#define DEFINE_DEV(d) \
    DECLARE_DEV(d) = \
    { \
        .minor = MISC_DYNAMIC_MINOR, \
        .name = #d, \
        .fops = &VAR_NAME(d,fops), \
    }

#define DECLARE_CONTEXT(d) \
    static struct dev_context VAR_NAME(d,context)

#define DEFINE_CONTEXT(d) \
    DECLARE_CONTEXT(d) = \
    { \
        .dev = &VAR_NAME(d,dev), \
        .cmds = VAR_NAME(d,cmds), \
        .cmdc = sizeof(VAR_NAME(d,cmds))/sizeof(struct dev_cmd), \
    }

#define CREATE_DEVICE(d) \
    DECLARE_IOCTL(d); \
    DECLARE_OPEN(d); \
    DECLARE_RELEASE(d); \
    DEFINE_FOPS(d); \
    DEFINE_DEV(d); \
    DEFINE_CONTEXT(d);\
    DEFINE_IOCTL(d); \
    DEFINE_OPEN(d); \
    DEFINE_RELEASE(d);


/*==================================================================================================
                             LOCAL STRUCTURES, UNIONS, ENUMS PROTOTYPES
==================================================================================================*/

/*============================================ Common ============================================*/

struct dev_cmd;
struct dev_context;


/*==================================================================================================
                                     LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*========================================= GPIO Device ==========================================*/

static int dev_gpio_cmd_get(struct dev_context *context, unsigned long arg);
static int dev_gpio_cmd_set(struct dev_context *context, unsigned long arg);

/*========================================= PUPD Device ==========================================*/

static int dev_pupd_cmd_reset(struct dev_context *context, unsigned long arg);
static int dev_pupd_cmd_set(struct dev_context *context, unsigned long arg);
static int dev_pupd_cmd_clr(struct dev_context *context, unsigned long arg);
static int dev_pupd_cmd_chk(struct dev_context *context, unsigned long arg);
static int dev_pupd_cmd_get(struct dev_context *context, unsigned long arg);

/*========================================= Version Device ==========================================*/

static int dev_version_cmd_get_hw(struct dev_context *context, unsigned long arg);

/*============================================ Common ============================================*/

static int mot_dev_ioctl(struct dev_context *context, unsigned int cmd, unsigned long arg);
static int mot_dev_open(struct dev_context *context);
static int mot_dev_release(struct dev_context *context);


/*==================================================================================================
                                  LOCAL STRUCTURES, UNIONS, ENUMS
==================================================================================================*/

/*============================================ Common ============================================*/

struct dev_cmd
{
    unsigned int cmd;
    int (*func)(struct dev_context *, unsigned long arg);
};

struct dev_context
{
    struct semaphore sem;
    struct miscdevice *dev;
    struct dev_cmd *cmds;
    unsigned int cmdc;
};


/*==================================================================================================
                                          LOCAL VARIABLES
==================================================================================================*/

/*========================================= GPIO Device ==========================================*/

DECLARE_CMDS(gpio) =
{
    {
        .cmd = MOT_DEV_GPIO_CMD_GET,
        .func = dev_gpio_cmd_get,
    },
    {
        .cmd = MOT_DEV_GPIO_CMD_SET,
        .func = dev_gpio_cmd_set,
    },
};

CREATE_DEVICE(gpio);

/*========================================= PUPD Device ==========================================*/

DECLARE_CMDS(pupd) =
{
    {
        .cmd = MOT_DEV_PUPD_CMD_RESET,
        .func = dev_pupd_cmd_reset,
    },
    {
        .cmd = MOT_DEV_PUPD_CMD_SET,
        .func = dev_pupd_cmd_set,
    },
    {
        .cmd = MOT_DEV_PUPD_CMD_CLR,
        .func = dev_pupd_cmd_clr,
    },
    {
        .cmd = MOT_DEV_PUPD_CMD_CHK,
        .func = dev_pupd_cmd_chk,
    },
    {
        .cmd = MOT_DEV_PUPD_CMD_GET,
        .func = dev_pupd_cmd_get,
    },
};

CREATE_DEVICE(pupd);

/*========================================= Version Device ==========================================*/

DECLARE_CMDS(version) =
{
    {
        .cmd = MOT_DEV_VERSION_CMD_GET_HW,
        .func = dev_version_cmd_get_hw,
    },
};

CREATE_DEVICE(version);

/*============================================ Common ============================================*/

static struct dev_context *contexts[] =
{
    &VAR_NAME(gpio,context),
    &VAR_NAME(pupd,context),
    &VAR_NAME(version,context),
};

#define MOT_DEV_CNT (sizeof(contexts)/sizeof(struct dev_context *))


/*==================================================================================================
                                          LOCAL FUNCTIONS
==================================================================================================*/

/*========================================= GPIO Device ==========================================*/

/*==================================================================================================

FUNCTION: dev_gpio_cmd_get

DESCRIPTION:
   Processes GPIO Get commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_gpio_cmd_get(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct gpio_get_arg get;

    if ( ret >= 0)
    {
        if ( copy_from_user(&get, (void __user *)arg, sizeof(get)) )
        {
            ret = -EFAULT;
        }
    }

    if ( ret >= 0 )
    {
        get.value = gpio_get_value(get.gpio);
    }
    
    if ( ret >= 0 )
    {
        if ( copy_to_user((void __user *)arg, &get, sizeof(get)) )
        {
            ret = -EFAULT;
        }
    }

    return ret;
}

/*==================================================================================================

FUNCTION: dev_gpio_cmd_set

DESCRIPTION:
   Processes GPIO Set commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_gpio_cmd_set(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct gpio_set_arg set;

    if ( ret >= 0)
    {
        if ( copy_from_user(&set, (void __user *)arg, sizeof(set)) )
        {
            ret = -EFAULT;
        }
    }

    if ( ret >= 0 )
    {
        gpio_set_value(set.gpio, set.value);
    }
    
    return ret;
}

/*========================================= PUPD Device ==========================================*/

/*==================================================================================================

FUNCTION: dev_pupd_cmd_reset

DESCRIPTION:
   Processes PUPD Reset commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_pupd_cmd_reset(struct dev_context *context, unsigned long arg)
{
    int ret = 0;

    if ( ret >= 0 )
    {
        mot_handover_reset_powerup_reason();
    }

    return ret;
}

/*==================================================================================================

FUNCTION: dev_pupd_cmd_set

DESCRIPTION:
   Processes PUPD Set commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_pupd_cmd_set(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct pupd_set_arg set;

    if ( ret >= 0)
    {
        if ( copy_from_user(&set, (void __user *)arg, sizeof(set)) )
        {
            ret = -EFAULT;
        }
    }

    if ( ret >= 0 )
    {
        mot_handover_set_powerup_reason(set.reason);
    }

    return ret;
}

/*==================================================================================================

FUNCTION: dev_pupd_cmd_clr

DESCRIPTION:
   Processes PUPD Clear commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_pupd_cmd_clr(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct pupd_clr_arg clr;

    if ( ret >= 0)
    {
        if ( copy_from_user(&clr, (void __user *)arg, sizeof(clr)) )
        {
            ret = -EFAULT;
        }
    }

    if ( ret >= 0 )
    {
        mot_handover_clr_powerup_reason(clr.reason);
    }

    return ret;
}

/*==================================================================================================

FUNCTION: dev_pupd_cmd_chk

DESCRIPTION:
   Processes PUPD Check commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_pupd_cmd_chk(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct pupd_chk_arg chk;

    if ( ret >= 0)
    {
        if ( copy_from_user(&chk, (void __user *)arg, sizeof(chk)) )
        {
            ret = -EFAULT;
        }
    }

    if ( ret >= 0 )
    {
        chk.value = mot_handover_chk_powerup_reason(chk.reason);
    }

    if ( ret >= 0 )
    {
        if ( copy_to_user((void __user *)arg, &chk, sizeof(chk)) )
        {
            ret = -EFAULT;
        }
    }

    return ret;
}

/*==================================================================================================

FUNCTION: dev_pupd_cmd_get

DESCRIPTION:
   Processes PUPD Get commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_pupd_cmd_get(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct pupd_get_arg get;

    if ( ret >= 0 )
    {
        get.reason = mot_handover_get_powerup_reason();
    }

    if ( ret >= 0 )
    {
        if ( copy_to_user((void __user *)arg, &get, sizeof(get)) )
        {
            ret = -EFAULT;
        }
    }

    return ret;
}

/*========================================= Version Device ==========================================*/

/*==================================================================================================

FUNCTION: dev_version_cmd_get_hw

DESCRIPTION:
   Processes Version Get HW commands.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   arg - Pointer to command arguements in userspace.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int dev_version_cmd_get_hw(struct dev_context *context, unsigned long arg)
{
    int ret = 0;
    struct pupd_get_hw_arg get_hw;

    if ( ret >= 0 )
    {
        get_hw.upid = mot_handover_get_upid();
        get_hw.crid = mot_handover_get_crid();
    }

    if ( ret >= 0 )
    {
        if ( copy_to_user((void __user *)arg, &get_hw, sizeof(get_hw)) )
        {
            ret = -EFAULT;
        }
    }

    return ret;
}

/*============================================ Common ============================================*/

/*==================================================================================================

FUNCTION: mot_dev_init

DESCRIPTION:
   Initializes all Motorola device nodes.

ARGUMENTS PASSED:
   None

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int __init mot_dev_init(void)
{
    int ret = 0;
    unsigned i;

    for ( i = 0 ; (ret >= 0) && (i < MOT_DEV_CNT ) ; i++ )
    {
        sema_init(&contexts[i]->sem, 1);
        ret = misc_register(contexts[i]->dev);
    }

    return ret;
}

module_init(mot_dev_init);

/*==================================================================================================

FUNCTION: mot_dev_ioctl

DESCRIPTION:
   Handles all IO Control commands on Motorola device nodes.

ARGUMENTS PASSED:
   context - Pointer to device context information.
   cmd - Command to be performed for the given device.
   arg - Argument to be passed to the given device.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int mot_dev_ioctl(struct dev_context *context, unsigned int cmd, unsigned long arg)
{
    ssize_t ret = 0;
    unsigned int i;

    if ( context )
    {
        down(&context->sem);
    }

    if (ret >= 0)
    {
        for ( i = 0 ; i < context->cmdc ; i++ )
        {
            if ( cmd == context->cmds[i].cmd )
            {
                break;
            }
        }
        if ( i >= context->cmdc )
        {
            ret = -EINVAL;
        }
    }

    if (ret >= 0)
    {
        ret = context->cmds[cmd].func(context, arg);
    }


    if ( context )
    {
        up(&context->sem);
    }

    return ret;
}

/*==================================================================================================

FUNCTION: mot_dev_open

DESCRIPTION:
   Handles all Open commands on Motorola device nodes.

ARGUMENTS PASSED:
   context - Pointer to device context information.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int mot_dev_open(struct dev_context *context)
{
    int ret = 0;

    /* Do nothing */

    return ret;
}

/*==================================================================================================

FUNCTION: mot_dev_release

DESCRIPTION:
   Handles all Release commands on Motorola device nodes.

ARGUMENTS PASSED:
   context - Pointer to device context information.

RETURN VALUE:
   int - Greater than or equal to zero on successful. Less than zero on failure.

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==================================================================================================*/
static int mot_dev_release(struct dev_context *context)
{
    int ret = 0;

    /* Do nothing */

    return ret;
}


