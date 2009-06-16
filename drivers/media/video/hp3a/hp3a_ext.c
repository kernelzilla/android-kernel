/*
 * drivers/media/video/hp3a/hp3a_ext.c
 *
 * HP Imaging/3A Driver : Exported function implementation.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>

#include "hp3a.h"
#include "hp3a_common.h"

/**
 * hp3a_ccdc_done - End of ccdc readout 3A task dispatcher .
 *
 * No return value.
 **/
void hp3a_ccdc_done(void)
{
	++g_tc.frame_count;
	hp3a_update_statistics();
}
EXPORT_SYMBOL(hp3a_ccdc_done);

/**
 * hp3a_frame_done - End of frame 3A task dispatcher .
 *
 * No return value.
 **/
void hp3a_frame_done(void)
{
	hp3a_update_hardpipe();
	hp3a_schedule_task();
}
EXPORT_SYMBOL(hp3a_frame_done);

/**
 * hp3a_stream_on - Perform stream on specific tasks.
 *
 * No return value.
 **/
void hp3a_stream_on(void)
{
	g_tc.frame_count = 0;
	g_tc.v4l2_streaming = 1;

	hp3a_enable_histogram();
	hp3a_update_hardpipe();
}
EXPORT_SYMBOL(hp3a_stream_on);

/**
 * hp3a_stream_off - Perform stream off specific tasks.
 *
 * No return value.
 **/
void hp3a_stream_off(void)
{
	g_tc.v4l2_streaming = 0;
	g_tc.raw_cap_sched_count = 0;

	hp3a_flush_queue(&g_tc.hist_hw_queue);
	hp3a_flush_queue(&g_tc.ready_stats_queue);
}
EXPORT_SYMBOL(hp3a_stream_off);
