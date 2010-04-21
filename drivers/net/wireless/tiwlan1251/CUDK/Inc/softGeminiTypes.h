/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                                                                          |**
**| Copyright 1998-2008 Texas Instruments, Inc. - http://www.ti.com/         |**
**|                                                                          |**
**| Licensed under the Apache License, Version 2.0 (the "License");          |**
**| you may not use this file except in compliance with the License.         |**
**| You may obtain a copy of the License at                                  |**
**|                                                                          |**
**|     http://www.apache.org/licenses/LICENSE-2.0                           |**
**|                                                                          |**
**| Unless required by applicable law or agreed to in writing, software      |**
**| distributed under the License is distributed on an "AS IS" BASIS,        |**
**| WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. |**
**| See the License for the specific language governing permissions and      |**
**| limitations under the License.                                           |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/

/*--------------------------------------------------------------------------*/
/* Module:		SOFT_GEMINI_TYPES.H*/
/**/
/* Purpose:		This module contains All the system definisions.*/
/**/
/*--------------------------------------------------------------------------*/
#ifndef __SOFT_GEMINI_TYPES_H__
#define __SOFT_GEMINI_TYPES_H__


#pragma pack(1)
typedef struct
{
	BOOL	state;
	UINT8	minTxRate;
} btCoexStatus_t;
#pragma pack()


#endif /* __SOFT_GEMINI_TYPES_H__  */
