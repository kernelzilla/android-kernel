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


#ifndef _WINDOWS_TYPES_H
#define _WINDOWS_TYPES_H

#include "osTIType.h"
#include "ioctl_init.h"

/*typedef tiBOOL    BOOLEAN, *PBOOLEAN;*/
typedef UINT16    USHORT, *PUSHORT;
typedef char      CHAR;
typedef const char * LPCSTR;

#define IN
#define OUT
#define NDIS_MAX_STRING_LEN   361

typedef struct _STRING {
  USHORT  Length;
  USHORT  MaximumLength;
  PCHAR  Buffer;
} ANSI_STRING, *PANSI_STRING, UNICODE_STRING, *PUNICODE_STRING;

typedef ANSI_STRING NDIS_STRING, *PNDIS_STRING;
typedef void * NDIS_HANDLE;
typedef int NDIS_STATUS, *PNDIS_STATUS;


typedef ULONG NDIS_OID, *PNDIS_OID;
#define PCI_TYPE0_ADDRESSES             6
#define PCI_TYPE1_ADDRESSES             2
typedef LARGE_INTEGER NDIS_PHYSICAL_ADDRESS;
typedef PVOID *PDEVICE_OBJECT, *PDRIVER_OBJECT;
typedef UINT32 NDIS_MINIPORT_INTERRUPT, NDIS_MINIPORT_TIMER, NDIS_SPIN_LOCK;
typedef UINT32 PNDIS_PACKET, *PPNDIS_PACKET;

typedef enum _NDIS_PARAMETER_TYPE {
  NdisParameterInteger,
  NdisParameterHexInteger,
  NdisParameterString,
  NdisParameterMultiString,
  NdisParameterBinary
} NDIS_PARAMETER_TYPE, *PNDIS_PARAMETER_TYPE;

typedef struct {
    USHORT  Length;
    PVOID  Buffer;
} BINARY_DATA;

typedef struct _NDIS_CONFIGURATION_PARAMETER {
  NDIS_PARAMETER_TYPE  ParameterType;
  union {
    ULONG  IntegerData;
    NDIS_STRING  StringData;
    BINARY_DATA  BinaryData;
  } ParameterData;
  char StringBuffer[NDIS_MAX_STRING_LEN];
} NDIS_CONFIGURATION_PARAMETER, *PNDIS_CONFIGURATION_PARAMETER;

typedef tiUINT32 NTSTATUS;

#ifndef NDIS_STATUS_SUCCESS
# define NDIS_STATUS_SUCCESS                        ((NDIS_STATUS)0x00000000L)
# define NDIS_STATUS_PENDING                        ((NDIS_STATUS)0x00000103L)
# define NDIS_STATUS_RESET_END                      ((NDIS_STATUS)0x40010005L)
# define NDIS_STATUS_MEDIA_SPECIFIC_INDICATION      ((NDIS_STATUS)0x40010012L)
# define NDIS_STATUS_FAILURE                        ((NDIS_STATUS)0xC0000001L)
# define NDIS_STATUS_ADAPTER_NOT_FOUND              ((NDIS_STATUS)0xC0010006L)
# define NDIS_STATUS_INVALID_LENGTH                 ((NDIS_STATUS)0xC0010014L)
# define NDIS_STATUS_BUFFER_TOO_SHORT               ((NDIS_STATUS)0xC0010016L)
# define NDIS_STATUS_INVALID_OID                    ((NDIS_STATUS)0xC0010017L)
#endif        /* NDIS_STATUS_SUCCESS */

#define STATUS_SUCCESS                0
#define STATUS_INVALID_PARAMETER     -1

#define NdisZeroMemory(p, size)      os_memoryZero( NULL, p, size )
#define NdisMoveMemory(d, s, size)   os_memoryCopy( NULL, d, s, size )

NDIS_STATUS NdisUnicodeStringToAnsiString( IN OUT PANSI_STRING  DestinationString,
    IN PUNICODE_STRING  SourceString );
VOID NdisReadConfiguration( OUT PNDIS_STATUS  Status, OUT PNDIS_CONFIGURATION_PARAMETER  *ParameterValue,
    IN NDIS_HANDLE  ConfigurationHandle, IN PNDIS_STRING  Keyword, IN NDIS_PARAMETER_TYPE  ParameterType );
VOID NdisWriteConfiguration( OUT PNDIS_STATUS  Status, IN NDIS_HANDLE  ConfigurationHandle,
    IN PNDIS_STRING  Keyword, IN PNDIS_CONFIGURATION_PARAMETER  ParameterValue );
VOID NdisReadNetworkAddress( OUT PNDIS_STATUS  Status, OUT PVOID  *NetworkAddress, OUT PUINT  NetworkAddressLength,
    IN NDIS_HANDLE  ConfigurationHandle );

typedef struct _NDIS_PACKET_POOL {
  NDIS_SPIN_LOCK  SpinLock;
  struct _NDIS_PACKET *FreeList;
  UINT  PacketLength;
  UCHAR  Buffer[1];
} NDIS_PACKET_POOL, * PNDIS_PACKET_POOL;

typedef enum _NDIS_802_11_STATUS_TYPE
{
    Ndis802_11StatusType_Authentication,
    Ndis802_11StatusTypeMax    /* not a real type, defined as an upper bound*/
} NDIS_802_11_STATUS_TYPE, *PNDIS_802_11_STATUS_TYPE;

typedef UCHAR   NDIS_802_11_MAC_ADDRESS[6];

typedef struct _NDIS_802_11_STATUS_INDICATION
{
    NDIS_802_11_STATUS_TYPE StatusType;
} NDIS_802_11_STATUS_INDICATION, *PNDIS_802_11_STATUS_INDICATION;

typedef struct _NDIS_802_11_AUTHENTICATION_REQUEST
{
    ULONG Length;            /* Length of structure*/
    NDIS_802_11_MAC_ADDRESS Bssid;
    ULONG Flags;
} NDIS_802_11_AUTHENTICATION_REQUEST, *PNDIS_802_11_AUTHENTICATION_REQUEST;

typedef tiINT32 NDIS_802_11_RSSI;           /* in dBm*/

typedef struct _NDIS_802_11_TEST
{
	ULONG Length;
	ULONG Type;
	union {
		struct _AuthenticationEvent {
            NDIS_802_11_STATUS_INDICATION Status;
            NDIS_802_11_AUTHENTICATION_REQUEST Request[1];
		} AuthenticationEvent;
		NDIS_802_11_RSSI RssiTrigger;
	};
} NDIS_802_11_TEST, *PNDIS_802_11_TEST;

/* Added new encryption types*/
/* Also aliased typedef to new name*/
typedef enum _NDIS_802_11_WEP_STATUS
{
    Ndis802_11WEPEnabled,
    Ndis802_11Encryption1Enabled = Ndis802_11WEPEnabled,
    Ndis802_11WEPDisabled,
    Ndis802_11EncryptionDisabled = Ndis802_11WEPDisabled,
    Ndis802_11WEPKeyAbsent,
    Ndis802_11Encryption1KeyAbsent = Ndis802_11WEPKeyAbsent,
    Ndis802_11WEPNotSupported,
    Ndis802_11EncryptionNotSupported = Ndis802_11WEPNotSupported,
    Ndis802_11Encryption2Enabled,
    Ndis802_11Encryption2KeyAbsent,
    Ndis802_11Encryption3Enabled,
    Ndis802_11Encryption3KeyAbsent
} NDIS_802_11_WEP_STATUS, *PNDIS_802_11_WEP_STATUS,
  NDIS_802_11_ENCRYPTION_STATUS, *PNDIS_802_11_ENCRYPTION_STATUS;

#ifdef TI_DBG
#ifdef __KERNEL__
#define DbgPrint     printk
#else
#define DbgPrint     printf
#endif
#else
#define DbgPrint
#endif

#endif /* _WINDOWS_TYPES_H */
