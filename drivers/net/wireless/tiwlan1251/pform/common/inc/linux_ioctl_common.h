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

#ifndef TI1610_IOCTL_COMMON_H
#define TI1610_IOCTL_COMMON_H

#include "osTIType.h"

#ifndef SIZE_ARR
#define SIZE_ARR(a)    (sizeof(a)/sizeof(a[0]))
#endif


typedef struct
{
    tiUINT32 func_id;
    void *optional_param;
} hal_print_param_t;

#define IOCTL_GET    1
#define IOCTL_SET    2
#define IOCTL_SETGET 3

typedef struct
{
    ULONG cmd;
    ULONG cmd_type; /* IOCTL_SET or IOCTL_GET */
    ULONG length;
    ULONG user_data_pointer;    /* pointer to data or data if length <= sizeof(ULONG) */
} tiioctl_req_t;


/* TIWLN_SET_INIT_INFO request parameter
 */
typedef struct
{
  tiUINT32 eeprom_image_length;
  tiUINT32 firmware_image_length;
  tiUINT32 init_file_length;
  /* eeprom image follows */
  char data[1];
  /* firmware image follows */
  /* init file follows */
} tiwlan_dev_init_t;

#endif /* TI1610_IOCTL_COMMON_H */
