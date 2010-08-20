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

#ifndef __ADSP_AUDIO_ERROR_H
#define __ADSP_AUDIO_ERROR_H


/* success, i.e. no error */
#define  ADSP_AUDIO_SUCCESS				0
/* general failure */
#define  ADSP_AUDIO_EFAILED				1
/* insufficient RAM */
#define  ADSP_AUDIO_ENOMEMORY				2
/* specified class unsupported */
#define  ADSP_AUDIO_ECLASSNOTSUPPORT			3
/* version not supported */
#define  ADSP_AUDIO_EVERSIONNOTSUPPORT			4
/* object already loaded */
#define  ADSP_AUDIO_EALREADYLOADED			5
/* unable to load object */
#define  ADSP_AUDIO_EUNABLETOLOAD			6
/* unable to unload object */
#define  ADSP_AUDIO_EUNABLETOUNLOAD			7
/* alarm is pending */
#define  ADSP_AUDIO_EALARMPENDING			8
/* invalid time */
#define  ADSP_AUDIO_EINVALIDTIME			9
/* NULL class object */
#define  ADSP_AUDIO_EBADCLASS				10
/* invalid metric specified */
#define  ADSP_AUDIO_EBADMETRIC				11
/* component expired */
#define  ADSP_AUDIO_EEXPIRED				12
/* invalid state */
#define  ADSP_AUDIO_EBADSTATE				13
/* invalid parameter */
#define  ADSP_AUDIO_EBADPARM				14
/* invalid URL scheme */
#define  ADSP_ADUIO_ESCHEMENOTSUPPORTED			15
/* invalid item */
#define  ADSP_AUDIO_EBADITEM				16
/* invalid format */
#define  ADSP_AUDIO_EINVALIDFORMAT			17
/* incomplete item */
#define  ADSP_AUDIO_EINCOMPLETEITEM			18
/* insufficient flash */
#define  ADSP_AUDIO_ENOPERSISTMEMORY			19
/* API is not supported */
#define  ADSP_AUDIO_EUNSUPPORTED			20
/* privileges are insufficient for this operation */
#define  ADSP_AUDIO_EPRIVLEVEL				21
/* unable to find specified resource */
#define  ADSP_AUDIO_ERESOURCENOTFOUND			22
/* non re-entrant API re-entered */
#define  ADSP_AUDIO_EREENTERED				23
/* API called in wrong task context */
#define  ADSP_AUDIO_EBADTASK				24
/* module left memory allocated when released */
#define  ADSP_AUDIO_EALLOCATED				25
/* operation is already in progress */
#define  ADSP_AUDIO_EALREADY				26
/* ADS mutual authorization failed */
#define  ADSP_AUDIO_EADSAUTHBAD				27
/* need service programming */
#define  ADSP_AUDIO_ENEEDSERVICEPROG			28
/* bad memory pointer */
#define  ADSP_AUDIO_EMEMPTR				29
/* heap corruption */
#define  ADSP_AUDIO_EHEAP				30
/* context (system, interface, etc.) is idle */
#define  ADSP_AUDIO_EIDLE				31
/* context (system, interface, etc.) is busy */
#define  ADSP_AUDIO_EITEMBUSY				32
/* invalid subscriber ID */
#define  ADSP_AUDIO_EBADSID				33
/* no type detected/found */
#define  ADSP_AUDIO_ENOTYPE				34
/* need more data/info */
#define  ADSP_AUDIO_ENEEDMORE				35
/* ADS Capabilities do not match those required for phone */
#define  ADSP_AUDIO_EADSCAPS				36
/* driver failed to close properly */
#define  ADSP_AUDIO_EBADSHUTDOWN			37
/* destination buffer given is too small */
#define  ADSP_AUDIO_EBUFFERTOOSMALL			38
/* no such name/port/socket/service exists or valid */
#define  ADSP_AUDIO_ENOSUCH				39
/* ACK pending on application */
#define  ADSP_AUDIO_EACKPENDING				40
/* not an owner authorized to perform the operation */
#define  ADSP_AUDIO_ENOTOWNER				41
/* current item is invalid */
#define  ADSP_AUDIO_EINVALIDITEM			42
/* not allowed to perform the operation */
#define  ADSP_AUDIO_ENOTALLOWED				43
/* invalid handle */
#define  ADSP_AUDIO_EBADHANDLE				44
/* out of handles */
#define  ADSP_AUDIO_EOUTOFHANDLES			45
/* waitable call is interrupted */
#define  ADSP_AUDIO_EINTERRUPTED			46
/* no more items available - reached end */
#define  ADSP_AUDIO_ENOMORE				47
/* a CPU exception occurred */
#define  ADSP_AUDIO_ECPUEXCEPTION			48
/* Cannot change read-only object or parameter */
#define  ADSP_AUDIO_EREADONLY				49
/* Operation would block if not non-blocking; wait and try again */
#define  ADSP_AUDIO_EWOULDBLOCK				516


/* CS File System Error Codes */

/* File exists */
#define  ADSP_AUDIO_EFILEEXISTS			0x100
/* File does not exist */
#define  ADSP_AUDIO_EFILENOEXISTS		0x101
/* Directory not empty */
#define  ADSP_AUDIO_EDIRNOTEMPTY		0x102
/* Bad file name */
#define  ADSP_AUDIO_EBADFILENAME		0x103
/* Bad seek position */
#define  ADSP_AUDIO_EBADSEEKPOS			0x104
/* End of file */
#define  ADSP_AUDIO_EFILEEOF			0x105
/* File system full */
#define  ADSP_AUDIO_EFSFULL			0x106
/* File already open */
#define  ADSP_AUDIO_EFILEOPEN			0x107
/* No INODES (file slots) available */
#define  ADSP_AUDIO_EOUTOFNODES			0x108
/* Directory does not exist */
#define  ADSP_AUDIO_EDIRNOEXISTS		0x109
/* Invalid operation */
#define  ADSP_AUDIO_EINVALIDOPERATION		0x10A
/* Removeable Media Not present */
#define  ADSP_AUDIO_ENOMEDIA			0x10B
/* Directory exists */
#define  ADSP_AUDIO_EDIREXISTS			0x10C
/* File is not a Directory */
#define  ADSP_AUDIO_EFILENOTDIR			0x10D
/* File Space/Number Quota Exceeded */
#define  ADSP_AUDIO_EQUOTAEXCEEDED		0x10E
/* Quota Parameters Invalid */
#define  ADSP_AUDIO_EQUOTA			0x10F


/* OMM Error Codes */
#define ADSP_AUDIO_EENUMOUTOFSYNC		0x3c19300
#define ADSP_AUDIO_EUNEXPECTED			0x3c19301
#define ADSP_AUDIO_EITEMUNSUPPORTED		0x3c19302
#define ADSP_AUDIO_EMTUNSUPPORTED		0x3c19303
#define ADSP_AUDIO_ENOTRANSPORT			0x3c19304
#define ADSP_AUDIO_ECORRUPTED			0x3c19305
#define ADSP_AUDIO_ENOTCOMMITTED		0x3c19306
#define ADSP_AUDIO_EFLUSHING			0x3c19307
#define ADSP_AUDIO_EBROKENPIPE			0x3c19308
#define ADSP_AUDIO_ENOVALUE			0x3c19309
#define ADSP_AUDIO_ETIMEOUT			0x3c1930a
#define ADSP_AUDIO_EEXISTS			0x3c1930b


#endif


