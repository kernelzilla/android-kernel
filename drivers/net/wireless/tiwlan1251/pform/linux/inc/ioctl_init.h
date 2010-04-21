/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/


#ifndef TI1610_IOCTL_INIT
#define TI1610_IOCTL_INIT

#include <linux/kernel.h>
#include <linux/module.h>

#include "srcApi.h"

#ifndef DRIVER_NAME
#define DRIVER_NAME "TIWLAN"
#endif

#if defined(DEBUG_MESSAGES)
#  define print_deb(fmt, arg...) printk(KERN_INFO DRIVER_NAME ": " fmt,##arg)
#else
#  define print_deb(fmt, arg...) 
#endif

#if defined(OS_INFO_MESSAGES)
#  define print_info(fmt, arg...) printk(KERN_INFO DRIVER_NAME ": " fmt,##arg)
#else
#  define print_info(fmt, arg...)
#endif

#ifndef print_err
#  define print_err(fmt, arg...) printk(KERN_ERR DRIVER_NAME ": " fmt,##arg)
#endif

/* extern const struct iw_handler_def ti1610_handler_def; */
#include <linux/netdevice.h>
#include "linux_ioctl_common.h"

#define IS_CONFIG_MGR_STARTED(d)   (((tiwlan_net_dev_t *) (d)->priv)->started)
#define TEST_CONFIG_MGR_STARTED(d)  if( !IS_CONFIG_MGR_STARTED(d))  {\
                                print_err("Error: ConfigManager not STARTED!!\n");\
                                return -EACCES;\
                            }
                                

#endif /* TI1610_IOCTL_INIT */
