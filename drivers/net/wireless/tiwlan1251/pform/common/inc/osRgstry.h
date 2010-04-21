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

#ifndef __OSRGSTRY_H_
#define __OSRGSTRY_H_

typedef struct {

	PUCHAR					ParameterName;
	NDIS_STRING				NdisParameterName;

	NDIS_PARAMETER_TYPE		ParameterType;

	BOOLEAN					RangeCheck;

	ULONG					DefaultValue;
	ULONG					MinValue;
	ULONG					MaxValue;

	ULONG					FieldOffset;
	ULONG					FieldSize;

} REGISTRY_DATA, *PREGISTRY_DATA;


VOID
regFillInitTable(
	PTIWLN_ADAPTER_T pAdapter,
	PVOID pInitTable
	);

VOID
regReadParameters(
	PTIWLN_ADAPTER_T pAdapter
	);

VOID
regWriteInstanceNumber(
	PTIWLN_ADAPTER_T pAdapter
	);

VOID
regReadIntegerParameter(
				 PTIWLN_ADAPTER_T 		pAdapter,
				 PNDIS_STRING			pParameterName,
				 ULONG					defaultValue,
				 ULONG					minValue,
				 ULONG					maxValue,
				 UCHAR					parameterSize,
				 PUCHAR					pParameter
				 );

VOID
regReadStringParameter(
				 PTIWLN_ADAPTER_T 		pAdapter,
				 PNDIS_STRING			pParameterName,
				 PCHAR					pDefaultValue,
				 USHORT					defaultLen,
				 PUCHAR					pParameter,
				 void*					pParameterSize
				 );

VOID
regReadUnicodeStringParameter(
				 PTIWLN_ADAPTER_T 		pAdapter,
				 PNDIS_STRING			pParameterName,
				 PCHAR					pDefaultValue,
				 UCHAR					defaultLen,
				 PUCHAR					pParameter,
				 PUCHAR					pParameterSize
				 );


VOID
regReadWepKeyParameter(
				 PTIWLN_ADAPTER_T 		pAdapter,
				 PUCHAR					pKeysStructure,
				 UINT8					defaultKeyId
				 );

VOID
regReadNetworkAddress(PTIWLN_ADAPTER_T pAdapter);

#ifdef TI_DBG

VOID
regReadLastDbgState(
	PTIWLN_ADAPTER_T pAdapter
	);

VOID
regWriteLastDbgState(
	PTIWLN_ADAPTER_T pAdapter
	);

#endif

// TRS:AS registry callbacks
#if defined(_WINDOWS)  
#endif /* if defined(_WINDOWS) */
//TRS end

#endif		// __OSRGSTRY_H_

