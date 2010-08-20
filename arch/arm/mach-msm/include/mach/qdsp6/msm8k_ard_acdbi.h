/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __DALIACDB_H__
#define __DALIACDB_H__

/* ACDB Command ID definitions */
#define ACDB_GET_DEVICE		0x0108bb92
#define ACDB_SET_DEVICE		0x0108bb93
#define ACDB_GET_STREAM		0x0108bb95
#define ACDB_SET_STREAM		0x0108bb96
#define ACDB_GET_DEVICE_TABLE	0x0108bb97
#define ACDB_GET_STREAM_TABLE	0x0108bb98


/* ACDB Error Codes */
#define ACDB_RES_SUCCESS	0
#define ACDB_RES_FAILURE	-1
#define ACDB_RES_BADPARM	-2
#define ACDB_RES_BADSTATE	-3


/* ACDB Structure Definitions */

/* These structures are passed as input argument to the */
/* acdb_ioctl function defined below */

struct acdb_cmd_device_struct {
	u32	command_id;
	u32	device_id;
	u32	network_id;
	u32	sample_rate_id;
	u32	interface_id;
	u32	algorithm_block_id;

	/* Actual Length of allocated memory pointed to by phys_addr */
	u32	total_bytes;

	/* Physical address pointing to first memory location allocated by */
	/* CAD - translated from a page aligned virtual address. */
	u32	unmapped_buf;
};

struct acbd_cmd_device_table_struct {
	u32	command_id;
	u32	device_id;
	u32	network_id;
	u32	sample_rate_id;

	/* Actual Length of allocated memory pointed to by phys_addr */
	u32	total_bytes;

	/* Physical address pointing to first memory location allocated by */
	/* CAD - translated from a page aligned virtual address. */
	u32	unmapped_buf;
};

struct acdb_result_struct {
	/* Physical address pointing to first shared memory location */
	u32	unmapped_buf;
	/* The size of data copied to phys_addr by ACDB */
	u32	used_bytes;
	/* Result of operation */
	u32	result;
};

#endif
