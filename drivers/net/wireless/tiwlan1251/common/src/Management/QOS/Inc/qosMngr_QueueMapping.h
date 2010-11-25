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

#ifndef __qosMngr_Queue_Mapping_H__
#define __qosMngr_Queue_Mapping_H__

#include "paramOut.h"


typedef enum
{
	NonQosTag0ToAC		= QOS_AC_BE,
	NonQosTag1ToAC		= QOS_AC_BE,
	NonQosTag2ToAC		= QOS_AC_BE,
	NonQosTag3ToAC		= QOS_AC_BE,
	NonQosTag4ToAC		= QOS_AC_BE,
	NonQosTag5ToAC		= QOS_AC_BE,
	NonQosTag6ToAC		= QOS_AC_BE,
	NonQosTag7ToAC		= QOS_AC_BE,

} NonQosTagToACTableDef_e;


typedef enum
{
	WMEQosTag0ToAC		= QOS_AC_BE,
	WMEQosTag1ToAC		= QOS_AC_BK,
	WMEQosTag2ToAC		= QOS_AC_BK,
	WMEQosTag3ToAC		= QOS_AC_BE,
	WMEQosTag4ToAC		= QOS_AC_VI,
	WMEQosTag5ToAC		= QOS_AC_VI,
	WMEQosTag6ToAC		= QOS_AC_VO,
	WMEQosTag7ToAC		= QOS_AC_VO,

} WMEQosTagToACTableDef_e;



/* EXC mapping:
 * BK = Queue 0 
 * BE = queue 1
 * VI - queue 2
 * VO - queue 3
 */

typedef enum
{
	EXCAc0ToQueueIndex		= 1,
	EXCAc1ToQueueIndex		= 0,
	EXCAc2ToQueueIndex		= 2,
	EXCAc3ToQueueIndex		= 3,

} EXCAcToQueueIndexDef_e;





#endif /* __qosMngr_Queue_Mapping_H__ */

