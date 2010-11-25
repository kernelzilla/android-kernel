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


#include "arch_ti.h"

#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/list.h>
#include <asm/io.h>

#include "debug_module.h"
#include "esta_drv.h"
#ifdef DRIVER_PROFILING
#include "tiwlan_profile.h"
#endif
#include "osApi.h"
#include "osTIType.h"
#include "srcApi.h"
#include "whalHwRegs.h"
#include "bmtrace.h"
#include "TI_IPC_Api.h"
#include "whalBus_Defs.h"
#include "802_11Defs.h"
#include "Ethernet.h"

#define OS_PROTECT_HANDLE   0xffffeee0
/*#define DEBUG_REG_ACCESS*/

#ifdef DEBUG_REG_ACCESS
#define PRINT_REG(fmt, args...)     print_info(fmt, args)
#else
#define PRINT_REG(fmt, args...)
#endif
#define NOPRINT_REG(fmt, args...)

#ifdef ESTA_TIMER_DEBUG
#define esta_timer_log(fmt,args...)  printk(fmt, ## args)
#else
#define esta_timer_log(fmt,args...)
#endif

#define FRAG_SIZE        200

/* Wlan chip reset defines */
#define GPIO_16						16
#define GPIO_16_DIRECTION_OUTPUT	0
#define GPIO_16_CLEAR				0
#define GPIO_16_SET					1
	
#define GPIO1_IRQSTATUS1 	0xFFFBE418
#define GPIO1_DATAIN 		0xFFFBE42C

/*********************    LOCAL DECLARATIONS ************************/
static inline void os_timer_dec_use_count(timer_obj_t *tmr);
static int os_tl_timerHandlr(struct tiwlan_req *req);
static void os_timerHandlr(unsigned long parm);
static void send_frag(char* msg, int message_len, int level, int module);

BOOL use_debug_module = FALSE;

/****************************************************************************************
 *                        																*
 *						OS Report API													*       
 *																						*
 ****************************************************************************************/

/****************************************************************************************
 *                        os_setDebugMode()                                 
 ****************************************************************************************
DESCRIPTION:  	Set the Debug Mode 

INPUT:            

RETURN:			None   

NOTES:         	
*****************************************************************************************/
void os_setDebugMode(BOOL enable)
{
	use_debug_module = enable;
}


/****************************************************************************************
 *                        os_printf()                                 
 ****************************************************************************************
DESCRIPTION:  	Print formatted output. 

INPUT:          format -  Specifies the string, to be printed

RETURN:			None   

NOTES:         	
*****************************************************************************************/
void os_printf(const char *format ,...)
{
	static int from_new_line = 1;		/* Used to save the last message EOL */
	static UINT8 module = 0;			/* Used to save the last message module */
	static UINT8 level = 0;				/* Used to save the last message level */
    va_list ap;
    static char msg[500];
	char *p_msg = msg;					/* Pointer to the message */
	UINT16 message_len;					
    UINT32 sec = 0;
    UINT32 uSec = 0;
   
	/* Format the message and keep the message length */
    va_start(ap,format);
	message_len = vsnprintf(&msg[1], sizeof(msg) - 1, format, ap);

    
	if (use_debug_module)
	{
		/*********************/
		/* Use debug module */
		/*******************/

		if (msg[1] == '$')
		{
			/************************************
				  Message format:		"$XX" 
										 |||
					  message prefix ----|||
					  module index -------||
					  severity index ------|
			************************************/

			level = (msg[2] - 'A');
			module = (msg[3] - 'A');
		}
		else
		{
            send_frag(msg, message_len, level, module);
        }
    }
    else
    {
        /***********************/
        /* Use regular printk */
        /*********************/
        
        if( from_new_line )
        {
            if (msg[1] == '$')
            {
                p_msg += 4;
            }
            
            sec = os_timeStampUs(NULL);
            uSec = sec % 1000000;
            sec /= 1000000;
            
            printk(KERN_INFO DRIVER_NAME ": %d.%06d: %s",sec,uSec,p_msg);
        }
        else
        {
            printk(&msg[1]);
        }
        
        from_new_line = ( msg[message_len] == '\n' );
    }
}

static void send_frag(char* msg, int message_len, int level, int module)
{
#ifdef TIWLAN_OMAP1610 /* Dm: */
    int return_value;
    int offset = 1;
    int TmpLen;
    char* FragMsg;
    
    do
    {            
        TmpLen = min(message_len - offset, FRAG_SIZE);
        FragMsg = msg + offset - 1;
        FragMsg[0] = module;
        
        return_value = debug_module_enqueue_message(DEBUG_MODULE_TRACE_QUEUE_ID, level, FragMsg, TmpLen+1, CONTROL_CODE_TYPE_MESSAGE);

			if (return_value)
			{
				/* Message overrun */

				/* Send the overrun indication to the debug module */
				os_memoryCopy(NULL, &msg[1], "*** Message Overrun ***", strlen("*** Message Overrun ***"));
				msg[0] = 0;
				debug_module_enqueue_message(DEBUG_MODULE_TRACE_QUEUE_ID, 0, msg, (strlen("*** Message Overrun ***") + 1), CONTROL_CODE_TYPE_MESSAGE);

				/* Print overrun indication to the terminal */
				/*printk(KERN_INFO DRIVER_NAME ": %d.%06d: %s\n", sec, uSec, "**** Debug module message overrun! ****\n");*/
			}
        offset += TmpLen;
    }while (offset < message_len);
  
#endif    
}

/****************************************************************************************
 *                        																*
 *							OS DMA CALLBACK API											*
 ****************************************************************************************/
 
/****************************************************************************************
 *                        os_TNETWIF_BusTxn_Complete()                                 
 ****************************************************************************************
DESCRIPTION:    Callback directly called at an IRQ context from the SPI modue
				This should triger a tasklet_schedule so that the End of DMA will be handled
				in a tasklet  context and then be directed to the TNETWIF to call the Client callback.

INPUT:          OsContext - our adapter context.

RETURN:         None

NOTES:         	
*****************************************************************************************/
void os_TNETWIF_BusTxn_Complete(TI_HANDLE OsContext,int status)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;

    drv->dma_done = 1;
#ifdef DM_USE_WORKQUEUE
    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->timer_wake_lock );
#endif
    queue_work( drv->tiwlan_wq, &drv->tw );
#else
   	tasklet_schedule(&drv->tl);
#endif
}

/****************************************************************************************
 *                        																*
 *							OS TIMER API												*
 *																						*
 ****************************************************************************************/

/****************************************************************************************
 *                        os_timerCreate()                                 
 ****************************************************************************************
DESCRIPTION:    This function creates and initializes a timer object associated with a
                caller's pRoutine function.

ARGUMENTS:      

RETURN:         A handle of the created timer.
                TI_HANDLE_INVALID if there is insufficient memory available

NOTES:          Using the Kernel timer feature, problem is that kernel timers are one-shots.
                For timers that are periodic this abstraction layer will have to mediate
                between the callback function and the re-submission of a timer request.

*****************************************************************************************/
TI_HANDLE
os_timerCreate(
        TI_HANDLE OsContext,
        PTIMER_FUNCTION pRoutine,
        TI_HANDLE Context
        )
{
    timer_obj_t *tmr;

#ifdef ESTA_TIMER_DEBUG
    esta_timer_log("\n\n%s:%d ::os_timerCreate(%p,%p,%p)",__FUNCTION__, __LINE__,OsContext,pRoutine,Context);
#endif
    ti_nodprintf(TIWLAN_LOG_INFO, "\n----> os_timerCreate function = 0x%08x , context= 0x%08x",
                 (int)pRoutine, (int)Context);

    os_profile (OsContext, 6, 0);

    tmr = os_memoryAlloc (OsContext, sizeof(timer_obj_t));
    if (tmr == NULL)
        return(TI_HANDLE_INVALID);

    memset (tmr,0,sizeof(timer_obj_t));

    init_timer(&tmr->timer);
    INIT_LIST_HEAD(&tmr->req.list);
    tmr->timer.function = os_timerHandlr;
    tmr->timer.data = (int)tmr;
    tmr->req.drv = (tiwlan_net_dev_t *)OsContext;
    tmr->req.u.req.p1 = (UINT32)pRoutine;
    tmr->req.u.req.p2 = (UINT32)Context;
    tmr->req.u.req.f  = os_tl_timerHandlr;
    tmr->use_count = 1;

    esta_timer_log("=%p\n\n", tmr);

    return (TI_HANDLE)tmr;
}


/****************************************************************************************
 *                        os_timerDestroy()                                 
 ****************************************************************************************
DESCRIPTION:    This function destroy the timer object.

ARGUMENTS:      

RETURN:         

NOTES:          Returning the Kernel level timer_list memory allocation and the
                abstraction level timer object.
*****************************************************************************************/
VOID
os_timerDestroy(
        TI_HANDLE OsContext,
        TI_HANDLE TimerHandle
        )
{
    timer_obj_t *tmr = TimerHandle;

    os_profile (OsContext, 6, 0);

    os_timerStop (OsContext, TimerHandle);
    os_timer_dec_use_count (tmr);
}


/****************************************************************************************
 *                        os_timerStart()                                 
 ****************************************************************************************
DESCRIPTION:    This function start the timer object.

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
os_timerStart(
        TI_HANDLE OsContext,
        TI_HANDLE TimerHandle,
        UINT32 DelayMs,
        BOOL bPeriodic
        )
{
   timer_obj_t *tmr= (timer_obj_t *)TimerHandle;

   UINT32 jiffie_cnt = msecs_to_jiffies(DelayMs);

#ifdef ESTA_TIMER_DEBUG
	esta_timer_log("\n\n%s:%d ::os_timerStart(%p,%p,%u,%d)\n\n",__FUNCTION__, __LINE__,OsContext,TimerHandle,DelayMs,bPeriodic);
#endif

   tmr->req.u.req.p3 = bPeriodic;
   tmr->req.u.req.p4 = jiffie_cnt;
   tmr->timer.data    = (unsigned long)tmr;
   mod_timer(&tmr->timer, jiffies + jiffie_cnt);


   return;
}

/****************************************************************************************
 *                        os_stopTimer()                                 
 ****************************************************************************************
DESCRIPTION:    This function stop the timer object.

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
os_timerStop(
        TI_HANDLE OsContext,
        TI_HANDLE TimerHandle
        )
{
   timer_obj_t *tmr= (timer_obj_t *)TimerHandle;

   del_timer_sync(&tmr->timer);
   tmr->req.u.req.p3 = 0; /* Turn "periodic" off */
   list_del_init(&tmr->req.list);

   return;
}

/****************************************************************************************
 *                        os_periodicIntrTimerStart()                                 
 ****************************************************************************************
DESCRIPTION:    This function starts the periodic interrupt mechanism. This mode is used 
				when interrupts that usually received from the Fw is now masked, and we are
				checking for any need of Fw handling in time periods.

ARGUMENTS:		

RETURN:			

NOTES:         	Power level of the CHIP should be always awake in this mode (no ELP)
*****************************************************************************************/
VOID
os_periodicIntrTimerStart(
	TI_HANDLE OsContext
	)
{
	tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;

	mod_timer (&drv->poll_timer, jiffies + TIWLAN_IRQ_POLL_INTERVAL);
}

/****************************************************************************************
 *                        os_timeStampMs()                                 
 ****************************************************************************************
DESCRIPTION:	This function returns the number of milliseconds that have elapsed since
				the system was booted.

ARGUMENTS:		OsContext - our adapter context.

RETURN:			

NOTES:         	
*****************************************************************************************/
UINT32
os_timeStampMs(
        TI_HANDLE OsContext
        )
{
    struct timeval tv;
    do_gettimeofday(&tv);
    return tv.tv_sec*1000 + tv.tv_usec/1000;
}

/****************************************************************************************
 *                        os_timeStampUs()                                 
 ****************************************************************************************
DESCRIPTION:	This function returns the number of microseconds that have elapsed since
				the system was booted.

ARGUMENTS:		OsContext - our adapter context.
				Note that sometimes this function will be called with NULL(!!!) as argument!

RETURN:			

NOTES:         	
*****************************************************************************************/
UINT32
os_timeStampUs(
        TI_HANDLE OsContext
        )
{
    struct timeval tv;
    do_gettimeofday(&tv);
    return tv.tv_sec*1000000 + tv.tv_usec;
}

/****************************************************************************************
 *                        os_StalluSec()                                 
 ****************************************************************************************
DESCRIPTION:	This function make delay in microseconds.

ARGUMENTS:		OsContext - our adapter context.
				uSec - delay time in microseconds

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
os_StalluSec(
        TI_HANDLE OsContext,
        UINT32 uSec
        )
{
    /*UINT32 usec_now = os_timeStampUs(OsContext);
    while(os_timeStampUs(OsContext) - usec_now < uSec)
      ;
	*/
    udelay(uSec);
}

/****************************************************************************************
 *                        os_WaitComplete()                                 
 ****************************************************************************************
DESCRIPTION:    This function start waiting for the complete

ARGUMENTS:		

RETURN:			

NOTES:         	can be called only from process context, and not from tasklet
*****************************************************************************************/
VOID 
os_WaitComplete(
	TI_HANDLE OsContext
	)
{
	tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
		
	/* ti_dprintf(TIWLAN_LOG_INFO, "os_WaitComplete drv %x drv->comp %x\n",(UINT32)drv,(UINT32)&drv->comp); */
	
   	/* 
   	he tasklet should them send back the user (here)the completion event so the user could
   	go through the configuration phase 
   	*/
  	wait_for_completion(&drv->comp);
}

/****************************************************************************************
 *                        os_Complete()                                 
 ****************************************************************************************
DESCRIPTION:    This function signals to the waiting process that completion occured

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID 
os_Complete(
	TI_HANDLE OsContext
	)
{
   	tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
		
	/* ti_dprintf(TIWLAN_LOG_INFO, "os_Complete drv %x drv->comp %x\n",(UINT32)drv, (UINT32)&drv->comp); */

   	/* 
   	Call the completion routine  that will unblock the caller that was waiting on that object 
   	*/
   	complete(&drv->comp);
}



/****************************************************************************************
 *                        																*
 *							Hardware access functions	API								*
 *																						*
 ****************************************************************************************/

/****************************************************************************************
 *                        os_clearWlanReady()                                 
 ****************************************************************************************
DESCRIPTION:  	Clear the WLAN Ready Interrupt Line stored in the PIC Controller 

INPUT:          None  

RETURN:			None   

NOTES:         	
*****************************************************************************************/

__inline__ VOID
os_clearWlanReady(
	void)
{
#ifdef TIWLAN_OMAP1610
    omap_writel(4,GPIO1_IRQSTATUS1);
#endif
}


/****************************************************************************************
 *                        os_senseIrqLine()                                 
 ****************************************************************************************
DESCRIPTION:  	Read the WLAN_IRQ line 

INPUT:          void

RETURN:			Read value   

NOTES:         	
*****************************************************************************************/

__inline__  UINT32
os_senseIrqLine(
	TI_HANDLE OsContext
    )
{
#ifdef TIWLAN_OMAP1610
    return (omap_readl(GPIO1_DATAIN) & 0x4);
#else
    return 0;
#endif
}



/****************************************************************************************
 *                        os_hwGetRegistersAddr()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
PVOID
os_hwGetRegistersAddr(
        TI_HANDLE OsContext
        )
{
	return (PVOID)OS_API_REG_ADRR;
}

/****************************************************************************************
 *                        os_hwGetMemoryAddr()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
PVOID
os_hwGetMemoryAddr(
        TI_HANDLE OsContext
        )
{
	return (PVOID)OS_API_MEM_ADRR;
}

/****************************************************************************************
 *                        os_memoryGetPhysicalLow()                                 
 ****************************************************************************************
DESCRIPTION:	return the lower 32 bits of a 64bit number / address *	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
UINT32 os_memoryGetPhysicalLow (OS_PHYSICAL_ADDRESS pAddr)
{
   UINT32 res;
   res = pAddr & 0xffffffff;
   ti_dprintf(TIWLAN_LOG_ERROR, "\n 64bit low. Got 0x%x; Returning 0x%x \n", (UINT32)pAddr, res);
   return res;
}

/****************************************************************************************
 *                        os_memoryGetPhysicalHigh()                                 
 ****************************************************************************************
DESCRIPTION:	return the higher order 32 bits of a 64bit number / address *	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
UINT32 os_memoryGetPhysicalHigh (OS_PHYSICAL_ADDRESS pAddr)
{
   UINT32 res;
   res = pAddr >> 32;
   ti_dprintf(TIWLAN_LOG_ERROR, "\n 64bit high. Got 0x%x; Returning 0x%x \n", (UINT32)pAddr, res);
   return res;
}


/****************************************************************************************
 *                        																*
 *							Protection services	API										*
 *																						*
 ****************************************************************************************
 * OS protection is implemented as dummy functions because								*
 * all driver code is executed in context of a single tasklet,							*
 * except IOCTL handlers and xmition.													*
 * Protection in IOCTL handlers and hard_start_xmit is done by different				*
 * means.																				*
 ****************************************************************************************/


/****************************************************************************************
 *                        os_protectCreate()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		OsContext - our adapter context.

RETURN:			A handle of the created mutex/spinlock.
				TI_HANDLE_INVALID if there is insufficient memory available or problems
				initializing the mutex

NOTES:         	
*****************************************************************************************/
TI_HANDLE
os_protectCreate(
        TI_HANDLE OsContext
        )
{
    return (TI_HANDLE)OS_PROTECT_HANDLE;
}



/****************************************************************************************
 *                        os_protectDestroy()                                 
 ****************************************************************************************
DESCRIPTION:		

ARGUMENTS:		OsContext - our adapter context.

RETURN:			None - This had better work since there is not a return value to the user

NOTES:         	
*****************************************************************************************/
VOID
os_protectDestroy(
        TI_HANDLE OsContext,
        TI_HANDLE ProtectCtx
        )
{
    return;
}


/****************************************************************************************
 *                        os_protectLock()                                 
 ****************************************************************************************
DESCRIPTION:		

ARGUMENTS:		OsContext - our adapter context.

RETURN:			None - This had better work since there is not a return value to the user

NOTES:         	
*****************************************************************************************/
VOID
os_protectLock(
        TI_HANDLE OsContext,
        TI_HANDLE ProtectContext
        )
{
#if 1    /* uncomment for work in INDIRECT mode (HwACXAccessMethod=0) */
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
    spin_lock_irqsave(&drv->lock, drv->flags);
#endif
}


/****************************************************************************************
 *                        os_protectUnlock()                                 
 ****************************************************************************************
DESCRIPTION:		

ARGUMENTS:		OsContext - our adapter context.

RETURN:			None - This had better work since there is not a return value to the user

NOTES:         	
*****************************************************************************************/
VOID
os_protectUnlock(
        TI_HANDLE OsContext,
        TI_HANDLE ProtectContext
        )
{
#if 1    /* uncomment for work in INDIRECT mode (HwACXAccessMethod=0) */
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
    spin_unlock_irqrestore(&drv->lock, drv->flags);
#endif
}


/*-----------------------------------------------------------------------------

Routine Name:

        os_resetWakeOnGpio

Routine Description:

        set the GPIO to low after awaking the TNET from ELP.

Arguments:

        OsContext - our adapter context.


Return Value:

        None

-----------------------------------------------------------------------------*/
VOID
os_hardResetTnetw( void )
{
/*
 *  Define the OMAP GPIO registers, the TNETW reset is currently connected
 *  to GPIO 16, this logic assumes that the loading code had muxed the 
 *  GPIO 16 to the Y1 pinout.
 */
    /* direction out */
#ifdef TIWLAN_OMAP1610    
    omap_set_gpio_direction(GPIO_16, GPIO_16_DIRECTION_OUTPUT);
	
    /* clear reset WLAN chip */
    omap_set_gpio_dataout(GPIO_16, GPIO_16_CLEAR);

    /* wait for 50msec */
    mdelay(50);
    omap_set_gpio_dataout(GPIO_16, GPIO_16_SET);

    /* wait for 50msec */
    mdelay(50);
#endif /* Dm: */
#ifdef TIWLAN_MSM7000
    msm_wifi_reset(1); /* Reset active */
    msm_wifi_power(0); /* Power disable */
    msm_wifi_power(1); /* Power enable */
    msm_wifi_reset(0); /* Reset clear */
#endif
}


#ifndef GWSI_DRIVER

/****************************************************************************************
						START OF TI DRIVER API				
*****************************************************************************************/

/****************************************************************************************
 *                        os_setWakeOnGpio()                                 
 ****************************************************************************************
DESCRIPTION:	set the GPIO to high for awaking the TNET from ELP.	

ARGUMENTS:		OsContext - our adapter context.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_setWakeOnGpio(
    TI_HANDLE OsContext
    )
{
#ifdef TIWLAN_OMAP1610
    /*
    Clear ELP_REQ by GPIO_CLEAR_DATAOUT
    */
    os_resetWakeOnGpio(OsContext);

    /*
    Rising edge on ELP_REQ by GPIO_SET_DATAOUT
    */
    omap_writel(0x00000200, 0xFFFBBCF0);
#endif
}

/****************************************************************************************
 *                        os_resetWakeOnGpio()                                 
 ****************************************************************************************
DESCRIPTION:	set the GPIO to low after awaking the TNET from ELP.	

ARGUMENTS:		OsContext - our adapter context.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_resetWakeOnGpio(
    TI_HANDLE OsContext
    )
{
    /*
    Clear ELP_REQ by GPIO_CLEAR_DATAOUT
    */
#ifdef TIWLAN_OMAP1610
    omap_writel(0x00000200, 0xFFFBBCB0);
#endif
}

/****************************************************************************************
 *                        _os_memorySharedFree()                                 
 ****************************************************************************************
DESCRIPTION:		

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
_os_memorySharedFree(
    TI_HANDLE OsContext,
    PVOID pVirtual,
    UINT32 Size,
    OS_PHYSICAL_ADDRESS pPhysical
    )
{
    ti_dprintf(TIWLAN_LOG_ERROR, "\n\n\n %s is not implemented\n\n\n", __FUNCTION__);
}

/****************************************************************************************
 *                        _os_memorySharedAlloc()                                 
 ****************************************************************************************
DESCRIPTION:		

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
PVOID
_os_memorySharedAlloc(
    TI_HANDLE OsContext,
    UINT32 Size,
    OS_PHYSICAL_ADDRESS *pPhysical
    )
{
    ti_dprintf(TIWLAN_LOG_ERROR, "\n\n\n %s is not implemented\n\n\n", __FUNCTION__);
    return NULL;
}

/****************************************************************************************
 *                        os_powerStateBusy()                                 
 ****************************************************************************************
DESCRIPTION:	notify to the host that the TI_WLAN application is busy.	

ARGUMENTS:		OsContext - our adapter context.

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
os_powerStateBusy(
    TI_HANDLE OsContext
    )
{
#if 0
    ti_dprintf(TIWLAN_LOG_INFO,
               "%s(%d) - os_powerStateBusy: TI_WLAN is busy!\n",
               __FILE__,__LINE__);
#endif
}

/****************************************************************************************
 *                        os_powerStateIdle()                                 
 ****************************************************************************************
DESCRIPTION:	notify to the host that the TI_WLAN application is idle.	

ARGUMENTS:		OsContext - our adapter context.

RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
os_powerStateIdle(
    TI_HANDLE OsContext
    )
{
#if 0
    ti_dprintf(TIWLAN_LOG_INFO,
               "%s(%d) - os_powerStateIdle: TI_WLAN is idle!\n",
               __FILE__,__LINE__);
#endif
}

/****************************************************************************************
 *                        os_memoryMove()                                 
 ****************************************************************************************
DESCRIPTION:	Move memory block from pSource to pDestination	

ARGUMENTS:		OsContext - Our adapter context.
				pDestination - destination
				pSource - source
				Size - Number of characters
RETURN:			

NOTES:         	
*****************************************************************************************/
VOID
os_memoryMove(
        TI_HANDLE pOsContext,
        PVOID pDestination,
        PVOID pSource,
        UINT32 Size
        )
{
    memmove(pDestination, pSource, Size);
}


/****************************************************************************************
 *                        os_memoryMoveToHw()                                 
 ****************************************************************************************
DESCRIPTION:	This function copies data from a system-space buffer to device memory.	

ARGUMENTS:		OsContext - Our adapter context.

				pTarget - Specifies the base address within a device memory range where
                the copy should begin.

				pSource - Pointer to a system-space buffer from which this function copies
                data to the destination range.

				Size - Specifies the number of bytes to copy.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_memoryMoveToHw(
        TI_HANDLE OsContext,
        PVOID pTarget,
        PVOID pSource,
        UINT32 Size
        )
{
    print_info("\nos_memoryMoveToHw pTarget 0x%08x pSource 0x%08x Size 0x%08x",(int)pTarget, (int)pSource,(int)Size);
    print_info("\n-------------------------> Not Implemented <--------------------------------------------------- ");
}

/****************************************************************************************
 *                        os_memoryMoveFromHw()                                 
 ****************************************************************************************
DESCRIPTION:	This function copies data from a system-space buffer to device memory.	

ARGUMENTS:		OsContext - Our adapter context.

				pTarget - Pointer to a system-space buffer into which this function copies
                data from device memory.

				pSource - Specifies the base virtual address within device memory from
                which to copy the data.

				Size - Specifies the number of bytes to copy.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_memoryMoveFromHw(
        TI_HANDLE OsContext,
        PVOID pTarget,
        PVOID pSource,
        UINT32 Size
        )
{
    print_info("\nos_memoryMoveFromHw pTarget 0x%08x pSource 0x%08x Size 0x%08x",(int)pTarget, (int)pSource,(int)Size);
    print_info("\n-------------------------> Not Implemented <--------------------------------------------------- ");
}

/****************************************************************************************
 *                        os_hwReadMemRegisterUINT8()                                 
 ****************************************************************************************
DESCRIPTION:	Reads a UINT8 from a memory-mapped device register.	

ARGUMENTS:		OsContext - our adapter context.

				Register - Pointer to the memory-mapped register.

				Data - Pointer to the caller-supplied variable in which this function
                returns the UINT8 read from Register.	

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_hwReadMemRegisterUINT8(
        TI_HANDLE OsContext,
        PUCHAR Register,
        PUINT8 Data
        )
{
    *Data = *Register;
    PRINT_REG("R8: %p=0x%x\n", Register, *Data);
    return;
}

/****************************************************************************************
 *                        os_hwWriteMemRegisterUINT16()                                 
 ****************************************************************************************
DESCRIPTION:	Writes a 'unsigned short' to a memory-mapped device register.	

ARGUMENTS:		OsContext - our adapter context.

				Register - Pointer to the memory-mapped register.

				Data - Specifies the caller-supplied UINT16 that this function transfers
                to the Register.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_hwWriteMemRegisterUINT16(
        TI_HANDLE OsContext,
        PUINT16 Register,
        UINT16 Data
        )
{
    PRINT_REG("W16: %p=0x%x\n", Register, Data);
    *Register = Data;
    return;
}

/****************************************************************************************
 *                        os_hwReadMemRegisterUINT16()                                 
 ****************************************************************************************
DESCRIPTION:	Reads a UINT16 from a memory-mapped device register.	

ARGUMENTS:	    OsContext - our adapter context.

				Register - Pointer to the memory-mapped register.

				Data - Pointer to the caller-supplied variable in which this function
                returns the UINT16 read from Register.
	
RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_hwReadMemRegisterUINT16(
        TI_HANDLE OsContext,
        PUINT16 Register,
        PUINT16 Data
        )
{
    *Data = *Register;
    PRINT_REG("R16: %p=0x%x\n", Register, *Data);
    return;
}

/****************************************************************************************
 *                        os_hwWriteMemRegisterUINT32()                                 
 ****************************************************************************************
DESCRIPTION:	Writes a 'unsigned long' to a memory-mapped device register.	

ARGUMENTS:		OsContext - our adapter context.

				Register - Pointer to the memory-mapped register.

				Data - Specifies the caller-supplied UINT32 that this function transfers
                to the Register.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_hwWriteMemRegisterUINT32(
        TI_HANDLE OsContext,
        PUINT32 Register,
        UINT32 Data
        )
{
    PRINT_REG("W32: %p=0x%x\n", Register, Data);
    *Register = Data;
    return;
}


/****************************************************************************************
 *                        os_hwReadMemRegisterUINT32()                                 
 ****************************************************************************************
DESCRIPTION:	Reads a UINT32 from a memory-mapped device register.	

ARGUMENTS:		OsContext - our adapter context.

				Register - Pointer to the memory-mapped register.

				Data - Pointer to the caller-supplied variable in which this function
                returns the UINT32 read from Register.

RETURN:			None

NOTES:         	
*****************************************************************************************/
VOID
os_hwReadMemRegisterUINT32(
        TI_HANDLE OsContext,
        PUINT32 Register,
        PUINT32 Data
        )
{
    *Data = *Register;
    PRINT_REG("R32: %p=0x%x\n", Register, *Data);
    return;
}


/****************************************************************************************
 *                        os_receivePacket()                                 
 ****************************************************************************************
DESCRIPTION:		

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
BOOL
os_receivePacket(
        TI_HANDLE OsContext,
        PVOID pPacket,
        UINT16 Length
        )
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
    struct sk_buff *skb;
    mem_MSDU_T* pMsdu = (mem_MSDU_T*)pPacket;
    mem_BD_T *pCurBd;
    ti_nodprintf(TIWLAN_LOG_INFO, "os_receivePacket - Received EAPOL len-%d\n", pMsdu->firstBDPtr->length );

    bm_trace(10, Length, 0);

    if (pMsdu->firstBDPtr->length > Length) { /* Dm: Security fix */
        ti_dprintf(TIWLAN_LOG_ERROR, " TI: %s - Security Error\n", __FUNCTION__);
        return FALSE;
    }

    skb = dev_alloc_skb(Length+2);
    if(!skb) {
        print_deb(" os_receivePacket() : dev_alloc_skb failed!\n");
        configMgr_memMngrFreeMSDU(drv->adapter.CoreHalCtx, memMgr_MsduHandle(pPacket));
        drv->stats.rx_dropped++;
        return FALSE;
    }
    skb_reserve(skb, 2);

    pCurBd = pMsdu->firstBDPtr;
    while (pCurBd) {
        memcpy(skb_put(skb,pCurBd->length),pCurBd->data+pCurBd->dataOffset,pCurBd->length);
        pCurBd = pCurBd->nextBDPtr;
    }

    skb->dev = drv->netdev;
    skb->protocol = eth_type_trans(skb, drv->netdev);
    skb->ip_summed = CHECKSUM_NONE;

    drv->stats.rx_packets++;
    drv->stats.rx_bytes += skb->len;

    bm_trace(11, Length, 0);
#ifdef CONFIG_ANDROID_POWER
    drv->receive_packet = 1; /* Remember to stay awake */
#endif
    netif_rx(skb);

    configMgr_memMngrFreeMSDU(drv->adapter.CoreHalCtx, memMgr_MsduHandle(pPacket));

    bm_trace(12, Length, 0);

    return TRUE;
}

/****************************************************************************************
 *                        os_sendPacket()                                 
 ****************************************************************************************
DESCRIPTION:	send EAPOL packet from Supplicant	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
INT32
os_sendPacket(
        TI_HANDLE OsContext,
        PVOID pPacket,
        UINT16 Length
        )
{
    struct net_device *dev = (struct net_device *)OsContext;
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev);

    INT32 status;
    mem_MSDU_T* pMsdu;
    char *pMsduData;
    UINT32 packetHeaderLength;

    ti_nodprintf(TIWLAN_LOG_INFO, "os_sendPacket - Transmit EAPOL len-%x\n",Length );

   /*
    * Allocate enough place also for 802.11 header (24 bytes) and LLC (8 bytes)
    * to replace the Ethernet header (14 bytes)
    */
    if(!Length)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, " EAPOL Packet Length = 0 \n");
        return -1;
    }
	/* 
	 * Retrieve the Packet Header length 
	 * from QoS Manager (through configMgr) and RSN 
	 */
	packetHeaderLength = configMgr_getPacketHeaderLength(drv->adapter.CoreHalCtx,pPacket,TX_DATA_EAPOL_MSDU);

    /* 
	 * need to reserve enough space for header translation 
	 * in the same first Bd.
	 * Allocate enough place also for 802.11 header (24 bytes or 26 for QoS) and LLC (8 bytes)
	 * to replace the Ethernet header (14 bytes)
	 */

	status = configMgr_allocMSDU(drv->adapter.CoreHalCtx, &pMsdu,
									 Length + packetHeaderLength , OS_ABS_TX_MODULE);
	if(status != OK)
	{
	   ti_dprintf(TIWLAN_LOG_ERROR, " configMgr_allocMSDU failed !!!\n");
	   ++drv->alloc_msdu_failures;
	   return -ENOMEM;
	}

	/* 
	 * case 1: only legacy wlan header 
	 *
  	 * case 2: only QoS wlan header 
	 *
	 * case 3: only legacy wlan header with new snap
	 *
	 * case 4: only QoS wlan header with new snap
	 */
	pMsdu->firstBDPtr->dataOffset = packetHeaderLength - ETHERNET_HDR_LEN;
	pMsduData = pMsdu->firstBDPtr->data + pMsdu->firstBDPtr->dataOffset;
	memcpy(pMsduData, pPacket, Length);
	pMsdu->dataLen = Length;
	pMsdu->firstBDPtr->length = pMsdu->dataLen + pMsdu->firstBDPtr->dataOffset;
	pMsdu->freeFunc = 0;
	pMsdu->freeArgs[0] = 0;
	pMsdu->freeArgs[1] = 0;

   /*
    * Propagate Msdu through Config Manager.
    * Set DTag to 0 
	* (note that classification is further handled in the Core)
    */
    status = configMgr_sendMsdu(drv->adapter.CoreHalCtx, pMsdu, 0);

    return status;
}


#endif /* NDEF  GWSI_DRIVER*/



/*******************************************************************************************************
  
									LOCAL FUNCTIONS

********************************************************************************************************/

/*-----------------------------------------------------------------------------
Routine Name:

        os_timer_dec_use_count

Routine Description:

    This function is decrements timer use count.
    When use_count becomes 0, the timer block is destroyed
-----------------------------------------------------------------------------*/
static inline void os_timer_dec_use_count(timer_obj_t *tmr)
{
    if (unlikely(!tmr->use_count)) {
        ti_dprintf(TIWLAN_LOG_ERROR, "\n\n\n %s: attempt to delete a deleted timer %p\n\n\n", __FUNCTION__, tmr);
        tmr->use_count = 1;
    }
    if (!(--tmr->use_count))
        os_memoryFree(tmr->req.drv, tmr, sizeof(timer_obj_t));
}


/*-----------------------------------------------------------------------------

Routine Name:

        os_tl_timerHandlr

Routine Description:

    This function is called in context of the control tasklet.
    It evokes user timer handler and restarts the timer if periodic

Arguments:
    p1 - user handler
    p2 - user parameter
    p3 - periodic
    p4 - jiffies

Return Value:

    None.

Notes:

-----------------------------------------------------------------------------*/
static int os_tl_timerHandlr(struct tiwlan_req *req)
{
    timer_obj_t *tmr= (timer_obj_t *)req;
    PTIMER_FUNCTION f = (PTIMER_FUNCTION)req->u.req.p1;
    TI_HANDLE parm = (TI_HANDLE)req->u.req.p2;

    esta_timer_log("%s: req=0x%x f=0x%p parm=0x%p\n", __FUNCTION__, req, f, parm);
    ++tmr->use_count;
    f(parm);
    if (req->u.req.p3) /* Periodic ? */
        mod_timer(&tmr->timer, jiffies + req->u.req.p4);
    os_timer_dec_use_count(tmr);

    return 0;
}


/*-----------------------------------------------------------------------------

Routine Name:

        os_timerHandlr

Routine Description:

    This function is called on timer expiration in context of
    softIsr. It delegates the timer handling to the control tasklet.

Arguments:
    parm - timer object handle

Return Value:

    None.

Notes:

-----------------------------------------------------------------------------*/
static void os_timerHandlr(unsigned long parm)
{
    timer_obj_t *tmr= (timer_obj_t *)parm;
    tiwlan_net_dev_t *drv = tmr->req.drv;
    unsigned long flags;

    esta_timer_log("%s: drv=%p f=0x%x ctx=0x%x\n",
          __FUNCTION__, tmr->req.drv, tmr->req.u.req.p1, tmr->req.u.req.p2);

    spin_lock_irqsave(&drv->lock, flags);
    list_del(&tmr->req.list);
    list_add_tail(&tmr->req.list, &drv->request_q);
    spin_unlock_irqrestore(&drv->lock, flags);
#ifdef DM_USE_WORKQUEUE
    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->timer_wake_lock );
#endif
    queue_work( drv->tiwlan_wq, &drv->tw );
#else
    tasklet_schedule(&drv->tl);
#endif
}


/*-----------------------------------------------------------------------------

Routine Name:

        os_connectionStatus

Routine Description:

The eSTA-DK will call this API so the OS stack is aware that the
WLAN layer is ready to function.

Arguments:
cStatus = 1; WLAN in ready for network packets
cStatus = 0; WLAN in not ready for network packets

Return Value:

        None

-----------------------------------------------------------------------------*/

tiINT32
os_IndicateEvent(IPC_EV_DATA* pData)
{
   IPC_EVENT_PARAMS * pInParam =  (IPC_EVENT_PARAMS *)pData;
   tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)(pInParam->hUserParam);
   /*UCHAR AuthBuf[sizeof(ULONG) + sizeof(OS_802_11_AUTHENTICATION_REQUEST)];*/

   ti_nodprintf(TIWLAN_LOG_INFO, "\n  os_ConnectionStatus Event 0x%08x \n", CsStatus->Event);

   switch(pInParam->uEventType)
     {
   case IPC_EVENT_ASSOCIATED:
         if (drv->netdev != NULL) {
            netif_carrier_on(drv->netdev);
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
            queue_delayed_work(drv->tiwlan_wq, &drv->trxw, 0);
#endif
         }
         break;

       case IPC_EVENT_DISASSOCIATED:
         if (drv->netdev != NULL) {
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
            unsigned percent;

            cancel_delayed_work_sync(&drv->trxw);
            percent = ( drv->started ) ? PWRSINK_WIFI_PERCENT_BASE : 0;
#ifdef CONFIG_HTC_PWRSINK
            htc_pwrsink_set(PWRSINK_WIFI, percent);
#else
            trout_pwrsink_set(PWRSINK_WIFI, percent);
#endif
#endif
            netif_carrier_off(drv->netdev);
         }
      break;

      case IPC_EVENT_LINK_SPEED:
         drv->adapter.LinkSpeed = (*(PULONG)pData->uBuffer * 10000) / 2;
         ti_nodprintf(TIWLAN_LOG_INFO, "\n  Link Speed = 0x%08x \n",drv->adapter.LinkSpeed);
      break;

      case IPC_EVENT_AUTH_SUCC:
/*         *(PULONG)AuthBuf = os802_11StatusType_Authentication;
         memcpy((PUCHAR)&AuthBuf[sizeof(ULONG)], pData->uBuffer,sizeof(OS_802_11_AUTHENTICATION_REQUEST));*/
         ti_dprintf(TIWLAN_LOG_OTHER, "\n  Auth Succ Event from Driver to another BSSID. \n");
      break;

      case IPC_EVENT_SCAN_COMPLETE:
         ti_dprintf(TIWLAN_LOG_OTHER, "\n  Driver Event = Scan Complete. \n");
      break;

      case IPC_EVENT_TIMEOUT:
         ti_dprintf(TIWLAN_LOG_OTHER, "\n  Driver Event = Timeout. \n");
      break;

      case IPC_EVENT_CCKM_START:
         ti_dprintf(TIWLAN_LOG_OTHER, "\n  Driver Event = IPC_EVENT_CCKM_START \n");
      break;

      default:
         ti_dprintf(TIWLAN_LOG_ERROR, "\n  Unrecognized driver event. \n");
      break;

   }

   return OK;
}


/****************************************************************************/
/* The following 4 functions are debug functions that enable the user
   to set/reset GPIO_25 and GPIO_27 in the OMAP - for debug purposes.
   Note: In order to enable GPIO_25/GPIO_27 the user must enable the define
	TIWLAN_OMAP1610_CRTWIPP_GPIO_DEBUG in the esta_drv.c/osapi.c files.		*/

void os_ToggleDebugGPIO(int count)
{
#if 0
    int i,j;

	omap_writel(0x00000200, 0xFFFBBCB0 );/* 0 */
	for(i=0;i<count;i++)
	{
		omap_writel(0x00000200, 0xFFFBBCF0 );/* 1 */
		for(j=0;j<100;j++);
		omap_writel(0x00000200, 0xFFFBBCB0 );/* 0 */
		for(j=0;j<100;j++);
	}
#endif
}					
#ifdef TIWLAN_OMAP1610_CRTWIPP_GPIO_DEBUG
VOID
os_SetGpio_25(
    TI_HANDLE OsContext
    )
{
    /*
    Setting GPIO_25 by GPIO_SET_DATAOUT
    */
    omap_writel(0x00000200, 0xFFFBECF0 );
}


VOID
os_ResetGpio25(
    TI_HANDLE OsContext
    )
{
	/*
    Clear GPIO_25 by GPIO_CLEAR_DATAOUT
    */
    omap_writel(0x00000200, 0xFFFBECB0 );
}


VOID
os_SetGpio27(
    TI_HANDLE OsContext
    )
{
    /*
    Setting GPIO_27 by GPIO_SET_DATAOUT
    */
    omap_writel(0x00000800, 0xFFFBECF0 );
}


VOID
os_ResetGpio27(
    TI_HANDLE OsContext
    )
{
	/*
    Clear GPIO_27 by GPIO_CLEAR_DATAOUT
    */
    omap_writel(0x00000800, 0xFFFBECB0 );
}
#endif
/******************************************************************************/

VOID
os_disableIrq( TI_HANDLE OsContext)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
    disable_irq (drv->irq);
}

VOID
os_enableIrq( TI_HANDLE OsContext)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;
    enable_irq (drv->irq);
}

int
os_getFirmwareImage(
        TI_HANDLE OsContext,
        PUINT8 *pBuffer,
        PUINT32 Length,
        UINT8 RadioType
        )
{
   tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;

   *pBuffer = drv->firmware_image.va;
   *Length = drv->firmware_image.size;

   return OK;
}


/*-----------------------------------------------------------------------------

Routine Name:

        os_getRadioImage

Routine Description:


Arguments:


Return Value:

        OK

-----------------------------------------------------------------------------*/
int
os_getRadioImage(
        TI_HANDLE OsContext,
        PUINT8 *pBuffer,
        PUINT32 Length,
        UINT8 RadioType
        )
{

#ifdef FIRMWARE_DYNAMIC_LOAD
   tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)OsContext;

   *pBuffer = drv->eeprom_image.va;
   *Length = drv->eeprom_image.size;
#else
   extern unsigned char tiwlan_radimage[];
   extern unsigned int sizeof_tiwlan_radimage;
   *pBuffer = (PUINT8)tiwlan_radimage;
   *Length = sizeof_tiwlan_radimage;
#endif
   ti_dprintf(TIWLAN_LOG_INFO, "%s: radio type: 0x%x\n", __FUNCTION__, RadioType);

   return OK;
}


#ifdef DRIVER_PROFILING
void _os_profile (TI_HANDLE OsContext, UINT32 fn, UINT32 par)
{
    tiwlan_profile (OsContext, fn, par);
}
#endif

VOID
os_closeFirmwareImage( TI_HANDLE OsContext )
{
    return;
}

VOID
os_closeRadioImage( TI_HANDLE OsContext )
{
    return;
}
