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


#include <linux/errno.h>
#include <linux/string.h>

#include "arch_ti.h"
#include "osTIType.h"
#include "osApi.h"
#include "ioctl_init.h"

#include "802_11Defs.h"
#include "mlmeApi.h"

/****************************************************************************/
/*                                                                          */
/*              Definition of Constants                                     */
/*                                                                          */
/****************************************************************************/
#define SIZEOF_DOT11_MGMT_HEADER 0x18
#define SIZEOF_MLME_FRAME_INFO   0x60
#ifdef EXC_MODULE_INCLUDED
#define SIZEOF_BEACON_FRM        0x57
#else
#define SIZEOF_BEACON_FRM        0x53
#endif
#define SIZEOF_ELEMENT_HDR       0x02
#define SIZEOF_MGMT_FRAME        0x920
#define SIZEOF_ASSOC_RSP_TYPE    0x26
#define SIZEOF_AUTH_MSG_TYPE     0x0a
#define SIZEOF_DEATUH_MSG_TYPE   0x02
#define SIZEOF_DISASSOC_MSG_TYPE 0x02

#define CHECK_STRUCT_SIZE(_type,_size) \
{ \
  if (sizeof(_type) != _size) \
  { \
        print_info(KERN_INFO"\n.... ERROR in size of %s struct 0x%08x  should be 0x%08x.....\n", \
                 #_type, (int)sizeof(_type),(int)_size); \
        rc = -EINVAL; \
  } \
}

/************************************************************************
 *                        packed_strct_tst                                                              *
 ************************************************************************
DESCRIPTION: Used to test structures for the correct packed size

INPUT:      Void

OUTPUT:         Print Debug statements if the structures are not the expected size

RETURN:    0=success
           -EINVAL - failure

************************************************************************/
int packed_struct_tst (void)
{
   int rc = 0;
   print_info("\nTIWLAN: Testing sizes of packed structures...\n");
   CHECK_STRUCT_SIZE(dot11_mgmtHeader_t, SIZEOF_DOT11_MGMT_HEADER);
   CHECK_STRUCT_SIZE(dot11_eleHdr_t, SIZEOF_ELEMENT_HDR);
   CHECK_STRUCT_SIZE(beacon_probeRsp_t, SIZEOF_BEACON_FRM);
   CHECK_STRUCT_SIZE(assocRsp_t, SIZEOF_ASSOC_RSP_TYPE );
   CHECK_STRUCT_SIZE(authMsg_t, SIZEOF_AUTH_MSG_TYPE);
   CHECK_STRUCT_SIZE(deAuth_t,  SIZEOF_DEATUH_MSG_TYPE);
   CHECK_STRUCT_SIZE(disAssoc_t, SIZEOF_DISASSOC_MSG_TYPE);
   CHECK_STRUCT_SIZE(dot11_mgmtFrame_t, SIZEOF_MGMT_FRAME);
   CHECK_STRUCT_SIZE(mlmeFrameInfo_t,SIZEOF_MLME_FRAME_INFO);
   print_info("TIWLAN: packet structure size test %s\n", rc?"failed":"passed");

   return rc;
}
