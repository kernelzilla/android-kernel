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


#ifndef __OS_TYPES_H__
#define __OS_TYPES_H__

#include <linux/version.h>

#ifndef VOID
#define VOID        void
#define PVOID       void*
#endif

#if !defined(FALSE)
#define FALSE               0
#endif

#if !defined(TRUE)
#define TRUE                1
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#if !defined(false)
#define false               0
#endif

#if !defined(true)
#define true                1
#endif
#endif

#ifndef NULL
#if defined(__cplusplus)
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif /* NULL */

#ifndef UNUSED
#define UNUSED(p) ((void)p)
#endif

#ifndef SIZE_ARR
#define SIZE_ARR(a) (sizeof(a)/sizeof(a[0]) )
#endif
/*
#ifdef _UNICODE
#define _T  unsigned short
#else
#define _T
#endif
*/
#ifndef TI_HANDLE
typedef void*           TI_HANDLE;
#endif
#define TI_HANDLE_INVALID   NULL
#define TI_OK     0
#define TI_ERROR -1

typedef signed char                    INT8, *PINT8, *PCHAR;
typedef unsigned char       UINT8, *PUINT8, UCHAR, *PUCHAR;
typedef short           INT16, *PINT16;
typedef unsigned short      UINT16, *PUINT16;
typedef unsigned int            UINT, *PUINT;

typedef void            tiVOID, *tiPVOID;
typedef unsigned int        tiBOOL;
typedef tiBOOL    BOOLEAN, *PBOOLEAN;
typedef UINT16              tiUINT16;
typedef signed int          tiINT32;
typedef unsigned int        tiUINT32;
typedef UINT8               tiUINT8;
typedef char                ti_char;
typedef char                tiCHAR;
typedef unsigned long long  tiULONGLONG;
typedef long long           tiLONGLONG;

#ifndef _BASETSD_H_
typedef signed int      INT32, *PINT32;
typedef unsigned long   /*LARGE_INTEGER, */ULONG, *PULONG;
# ifndef _WINDOWS
typedef union _LARGE_INTEGER_T {
    struct {
        tiUINT32 LowPart;
        tiINT32  HighPart;
    };
    tiLONGLONG QuadPart;
} LARGE_INTEGER;
# endif /* _WINDOWS */

# ifndef _SUPPLICANT_
typedef unsigned int    UINT32, *PUINT32;
typedef unsigned char   BOOL, *PBOOL;
# endif /* _SUPPLICANT_ */
#endif /* _BASETSD_H_ */

/*
typedef struct _GUID {
    unsigned long  Data1;
    unsigned short Data2;
    unsigned short Data3;
    unsigned char  Data4[ 8 ];
} GUID;

#define DEFINE_GUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8) ;
*/
typedef unsigned long long      UINT64;
typedef long long               INT64;
#define CONSTANT64(x)           (x##LL)

#define STRUCT typedef struct
#define UNION typedef union
#define ENUM typedef enum

#define PACKED_STRUCT(name,content) STRUCT { content } __attribute__ ((packed)) name 
#define PACKED_STRUCT_NO_TYPEDEF(content) struct { content } __attribute__ ((packed)) 
#define PACKED_UNION(name,content) union { content } __attribute__ ((packed)) name 
#define PACKED_UNION_NO_TYPEDEF(content) union { content } __attribute__ ((packed)) 
#define ENUM_BODY(name, value) name  value,
#define PACKED_ENUM(name, list) ENUM { list(ENUM_BODY) } __attribute__ ((packed)) name


#define NWIF_MAX_QOS_CONNS 7

#define BIT_TO_BYTE_FACTOR 8

typedef struct nwif_clsfr_entry_t
{
    UINT32 ip;
    UINT16 port;
    UINT16 pri;
} NWIF_CLSFR_ENTRY;
/*The following define assumes that the structure above is 8 bytes.*/
#define OS_CLSFR_TABLE_SIZE (NWIF_MAX_QOS_CONNS * 8)

typedef UINT64 OS_PHYSICAL_ADDRESS;

#define IMPORT_TI_API

#endif /* __OS_TYPES_H__*/


