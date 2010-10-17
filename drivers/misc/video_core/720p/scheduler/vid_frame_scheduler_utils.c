/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

#include "video_core_type.h"
#include "vid_frame_scheduler_utils.h"

/**
 * SCHED_ASSERT () - This function is a wrapper to underlying ASSERT
 * @val: value to be checked for
 * function.
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void SCHED_ASSERT(int val)
{

}				/* end of SCHED_ASSERT */

/**
 * SCHED_MIN () - This function will find minimum of two values
 * @n_x: value 1
 * @n_y: value 2
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE int SCHED_MIN(int n_x, int n_y)
{
	if (n_x < n_y)
		return n_x;
	else
		return n_y;

}				/* end of SCHED_MIN */

/**
 * SCHED_MALLOC () - This function is a wrapper to underlying malloc
 * @size: memory size to be allocated
 * function
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void *SCHED_MALLOC(int size)
{
	return kmalloc(size, GFP_KERNEL);
}				/* end of SCHED_MALLOC */

/**
 * SCHED_FREE () - This function is a wrapper to underlying memory free
 * @p_ptr: memory to be freed
 * function
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void SCHED_FREE(void *p_ptr)
{
	kfree(p_ptr);
}				/* end of SCHED_FREE */

/**
 * SCHED_MEMSET () - This function is a wrapper to underlying memory set
 * @ptr: ptr to memory
 * @val: value to be set
 * @size: memory size to be set
 * function
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void *SCHED_MEMSET(void *ptr, int val, int size)
{
	return memset(ptr, val, size);
}				/* end of SCHED_MEMSET */

/**
 * SCHED_GET_CURRENT_TIME () - This function is a wrapper to underlying get time
 * @pn_time: ptr time value in milliseconds
 * function
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status_type SCHED_GET_CURRENT_TIME(u32 *pn_time)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	*pn_time = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
	return SCHED_S_OK;

}				/* end of SCHED_GET_CURRENT_TIME */

/**
 * SCHED_CRITSEC_CREATE () - This function is a wrapper to creating a critical
 * @p_cs: ptr to a critical section type
 * section
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status_type SCHED_CRITSEC_CREATE(u32 **p_cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_CREATE */

/**
 * SCHED_CRITSEC_RELEASE () - This function is a wrapper to releasing a critical
 * @cs: critical section handle type
 * section resource
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status_type SCHED_CRITSEC_RELEASE(u32 *cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_RELEASE */

/**
 * SCHED_CRITSEC_ENTER () - This function is a wrapper to enter a critical
 * @cs: critical section handle type
 * section
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status_type SCHED_CRITSEC_ENTER(u32 *cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_ENTER */

/**
 * SCHED_CRITSEC_LEAVE () - This function is a wrapper to leave a critical
 * @cs: critical section handle type
 * section
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status_type SCHED_CRITSEC_LEAVE(u32 *cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_LEAVE */
