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

/***************************************************************************/
/*																		   */
/*	  MODULE:	Ethernet.h												       */
/*    PURPOSE:								 							   */
/*																		   */
/***************************************************************************/
#ifndef _ETHERNET_H_
#define _ETHERNET_H_

#pragma pack(1)
typedef struct
{
  macAddress_t	DstAddr;
  macAddress_t	SrcAddr;
  UINT16		TypeLength;
} EthernetHeader_t;
#pragma pack()
 
#define ETHERTYPE_802_1D      0x8100

typedef struct
{
  macAddress_t	DstAddr;
  macAddress_t	SrcAddr;
  UINT16    Length;
  UINT8  	DSAP;
  UINT8     SSAP;
  UINT8     Control;
  UINT8     OUI[3];
  UINT16	Type;
} LlcSnapHeader_t;
#pragma pack()

#define MAC_ADDRESS_GROUP_BIT  ( 0x01 )		/* in byte [ 0 ] of the MAC Address*/


#define ETHERNET_HDR_LEN						14
#define IEEE802_3_HDR_LEN						14 
#define LLC_SNAP_HDR_LEN						20

#define SNAP_CHANNEL_ID							0xAA
#define LLC_CONTROL_UNNUMBERED_INFORMATION		0x03
#define ETHERNET_MAX_PAYLOAD_SIZE				1500

#define SNAP_OUI_802_1H_BYTE0					0x00
#define SNAP_OUI_802_1H_BYTE1					0x00
#define SNAP_OUI_802_1H_BYTE2					0xf8
#define SNAP_OUI_802_1H_BYTES  { SNAP_OUI_802_1H_BYTE0, SNAP_OUI_802_1H_BYTE1, SNAP_OUI_802_1H_BYTE2 }

#define SNAP_OUI_RFC1042_BYTE0					0x00
#define SNAP_OUI_RFC1042_BYTE1					0x00
#define SNAP_OUI_RFC1042_BYTE2					0x00
#define SNAP_OUI_RFC1042_LEN					3
#define SNAP_OUI_RFC1042_BYTES { SNAP_OUI_RFC1042_BYTE0, SNAP_OUI_RFC1042_BYTE1, SNAP_OUI_RFC1042_BYTE2 }


typedef enum tETHERTYPES 
{
  ETHERTYPE_APPLE_AARP = 0x80f3,
  ETHERTYPE_DIX_II_IPX = 0x8137

} ETHERTYPES, *PETHERTYPES;


static __inline BOOL IsMacAddressZero( macAddress_t *pMacAddr )
{
  return( (BOOL)( ( 0 == *                      (unsigned long *)pMacAddr ) &&
                     ( 0 == *(unsigned short *)( ( (unsigned long *)pMacAddr ) + 1 ) ) ) );

}


static __inline void ClearMacAddress( macAddress_t *pMacAddr )
{
  *              (unsigned long *)pMacAddr                 = 0;
  *(unsigned short *)( ( (unsigned long *)pMacAddr ) + 1 ) = 0;
}


static __inline BOOL IsMacAddressEqual( macAddress_t *pMacAddr1, macAddress_t *pMacAddr2 )
{
  return( (BOOL)( 
          ( *                        (unsigned long *)pMacAddr1  == 
            *                        (unsigned long *)pMacAddr2     ) &&

          ( *( (unsigned short *)( ( (unsigned long *)pMacAddr1 ) + 1 ) ) == 
            *( (unsigned short *)( ( (unsigned long *)pMacAddr2 ) + 1 ) )    ) ) );
}

static __inline void SetMacAddressBroadcast( macAddress_t *pMacAddr )
{
  *                      (unsigned long *)pMacAddr         = 0xffffffff;
  *(unsigned short *)( ( (unsigned long *)pMacAddr ) + 1 ) = 0xffff;

}


static __inline BOOL IsMacAddressGroup( macAddress_t *pMACAddr )
{
  return( pMACAddr->addr[ 0 ] & MAC_ADDRESS_GROUP_BIT );
}


static __inline BOOL IsMacAddressDirected( macAddress_t *pMACAddr )
{
  return( !IsMacAddressGroup( pMACAddr ) );
}



static __inline BOOL IsMacAddressBroadcast( macAddress_t *pMacAddr )
{
/* In WinCE an adrress that is not divided by 4 causes exception here */
   return( (BOOL)( ( 0xffff == *(unsigned short *)pMacAddr ) &&
                   ( 0xffff == *(((unsigned short *)pMacAddr) + 1 ) ) &&
                   ( 0xffff == *(((unsigned short *)pMacAddr) + 2 ) )));
/*  return( (BOOL)( ( 0xffffffff == *                      (unsigned long *)pMacAddr ) &&
                  ( 0xffff     == *(unsigned short *)( ( (unsigned long *)pMacAddr ) + 1 ) ) ) );*/

}


static __inline BOOL IsMacAddressMulticast( macAddress_t *pMACAddr )
{
  return( IsMacAddressGroup( pMACAddr ) && !IsMacAddressBroadcast( pMACAddr ) );
}



#endif
