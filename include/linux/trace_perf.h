#ifndef TRACE_PERF_H
#define TRACE_PERF_H
/*================================================================================

Header Name: trace_perf.h

General Description: for performance tracking.

==================================================================================
                     Motorola Confidential Proprietary
                Advanced Technology and Software Operations
              (c) Copyright Motorola 2008, All Rights Reserved

Revision History:
                   Modification     Tracking
Author                 Date          Number     Description of Changes
----------------   ------------    ----------   -------------------------
Motorola            1/8/2008       LIBoo51508   Initial create
Motorola            01/22/2008     LIBoo65642   Add performance track id
Motorola            03/19/2008     LIBpp22432   Add track id for MMAPI
Motorola            08/07/2008                  Change syscall id

================================================================================*/

/*================================================================================
                                      CONSTANTS
================================================================================*/
#define PERF_TRACK_THROUGH_SYSCALL

#ifdef PERF_TRACK_THROUGH_SYSCALL

#include <asm/ioctl.h>

/* Performance tracking commands*/
#define LL_CMD 'l'

#define LLEVENT		_IOW(LL_CMD, 0x01, unsigned int)

#define LLINFO		_IOW(LL_CMD, 0x02, char *)

enum {
	/* 0x00000000 to 0x00FFFFFF can be used by components for general debugging */
	
	/* 0xFF000000 to 0xFFFFFFFF used for global performance tracking */
	LTTLITE_PERF_TRACK_BASE = 0xFF000000,
	
	LTTLITE_PERF_TRACK_MOUNT_USERFS = LTTLITE_PERF_TRACK_BASE + 0x1000, /* mount userfs finished */
	
	LTTLITE_PERF_TRACK_WINDOWSSERVER_START = LTTLITE_PERF_TRACK_BASE + 0x2000, /* WindowsServer start time */
	
	LTTLITE_PERF_TRACK_SOUNDMANAGER_START = LTTLITE_PERF_TRACK_BASE + 0x2100, /* Start Sound Manager */
	
	LTTLITE_PERF_TRACK_ANIMATE_START = LTTLITE_PERF_TRACK_BASE + 0x2200, /* Start Animation */
	
	LTTLITE_PERF_TRACK_XP_START = LTTLITE_PERF_TRACK_BASE + 0x2300, /* XP start time */
	
	LTTLITE_PERF_TRACK_PHONE_START = LTTLITE_PERF_TRACK_BASE + 0x2400, /* phone start time */
	
	LTTLITE_PERF_TRACK_POWERUP_ALERT = LTTLITE_PERF_TRACK_BASE + 0x2500, /* Powerup Alert */
	
	LTTLITE_PERF_TRACK_CARRIERNAME_DISPLAY_START = LTTLITE_PERF_TRACK_BASE + 0x2600, /* HomeScreen display Carrier Nam
	e */
	LTTLITE_PERF_TRACK_HOMESCREEN_LOAD_START = LTTLITE_PERF_TRACK_BASE + 0x2700, /* Start loading Homescreen */
	
	LTTLITE_PERF_TRACK_IDLE_SCREEN_START = LTTLITE_PERF_TRACK_BASE + 0x2800, /* Reach IDLE screen */
	
	LTTLITE_PERF_TRACK_TAPIMSG_SYNC_START = LTTLITE_PERF_TRACK_BASE + 0x5000, /* First Sync msg from TAPI */
	LTTLITE_PERF_TRACK_TAPIMSG_REGISTER_SUCCESS, /* Register success msg from TAPI */
	
	LTTLITE_PERF_TRACK_BP_START = LTTLITE_PERF_TRACK_BASE + 0x10000, /* BP start time */
	LTTLITE_PERF_TRACK_BP_END, /* BP end time */
	
	LTTLITE_PERF_TRACK_MODEMSERVICE_START = LTTLITE_PERF_TRACK_BASE + 0x10100, /* Modem Service Starts */
	LTTLITE_PERF_TRACK_MODEMSERVICE_REGISTER, /* Modem Service sends Register Req */

        LTTLITE_PERF_MMAPI_TRACK_BASE          = LTTLITE_PERF_TRACK_BASE + 0x11000, /* MMAPI Trace ID Base */
        LTTLITE_PERF_MMAPI_TRACK_END           = LTTLITE_PERF_TRACK_BASE + 0x11100, /* MMAPI Trace ID End  */
};

#include <asm/unistd.h>6
#define __NR_lttlite (__NR_SYSCALL_BASE + 361)
#define TRACE_PERF(i)      syscall(__NR_lttlite, LLEVENT, i)
#define TRACE_PERF_INFO(s) syscall(__NR_lttlite, LLINFO, s)
#else /* PERF_TRACK_THROUGH_SYSCALL */
#define  TRACE_PERF(i)
#define TRACE_PERF_INFO(s)
#endif /* PERF_TRACK_THROUGH_SYSCALL */

#endif
