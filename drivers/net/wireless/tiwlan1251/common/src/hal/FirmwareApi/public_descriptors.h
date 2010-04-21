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

#ifndef PUBLIC_DESCRIPTORS_H
#define PUBLIC_DESCRIPTORS_H


#include "public_types.h"


/******************************************************************************

	TX PATH
	 
    The Tx path uses a double buffer and a TxControl structure, each located at a
	fixed address in the device's memory. On startup, the host retrieves the pointers
	to these addresses. A double buffer allows for continuous data flow towards the
	device. The host keeps track of which buffer is available and alternates between
	these two buffers on a per packet basis.
    The size of each of the two buffers is large enough to hold the longest 802.3 
	packet - maximum size Ethernet packet + header + descriptor.
	TX complete indication will be received a-synchronously in a TX done cyclic buffer 
	(txDoneRing) which is composed of 16 Tx Result Descriptors structures
	and is used in a cyclic manner. refer to TxResultDescriptor_t structure.
	
    The TX (HOST) procedure is as follows:
      1. Read the Tx path status (DataOutCount in TxPathStatus).
	  2. goto 1, if not possible. 
	     i.e. if DataInCount - DataOutCount >= HwBuffer size (2 for double buffer)).
      3. Copy the packet (preceded by TxDescriptor), if possible. 
	     i.e. if DataInCount - DataOutCount < HwBuffer size (2 for double buffer).
      4. increment DataInCount.
      5. Inform the firmware by generating a firmware internal interrupt.
      6. FW will increment DataOutCount after it reads the buffer.

    The TX Complete procedure:
      1. To get a TX complete indication the host enables the Tx Complete flag in the
	     TX Descriptor Structure (Refer to the Ctrl field in TxDescriptor_t).
	  2. For each packet with a Tx Complete field set, the firmware adds the transmit
	     results to the cyclic buffer (txDoneRing) and sets both done1and done2 to 1 to
		 indicate driver ownership. 
      3. The firmware sends a Tx Complete interrupt to the host to trigger the host to
	     process the new data. Note: interrupt will be send per packet if TX complete 
		 indication was requested in TxDescriptor_t (see 1.) or per crossing Aggregation
		 threashold.
      4. After receiving the Tx Complete interrupt, the host reads the TxDescriptorDone 
	     information in a cyclic manner and clears both done1 and done2 fields.
	
******************************************************************************/

/******************************************************************************

    TxDescriptor_t

    the structure of the transmit Tx descriptor passed from the host, ahead of 
	the transmitted packet.

******************************************************************************/


/* Because we are using a bit map, It is safer to enable the exact bits instead of field assignment */
/* It is IMPORTENT to check that the bit defenitions are alligned to the bit map struct  */
#define TX_COMPLETE_REQUIRED_BIT	0x80

/*  TX attribute fields (txAttr of TxDesriptor structure)*/
/*  ******************************************************/
typedef	struct 
{
    unsigned      ratePolicy		:3; /* Rate Policy (class) index */
    unsigned      ackPolicy			:1; /* When set, no ack policy is expected*/
    unsigned      pktType			:2;	/* packet type: */
	                                    /* 0 - 802.11, */
	                                    /* 1 - 802.3,*/
	                                    /* 2 - IP, */
	                                    /* 3 - raw codec  */
    unsigned      qosFrame			:1;	/* If set, this is QoS-Null or QoS-Data*/
    unsigned      txCmpltRequired	:1;	/* If set, the FW should trigger the TxComplete*/
	                                    /* interrupt for this packet*/
    unsigned      xferPadding		:1;	/* If set, there is 2 bytes padding before  */
	                                    /* packet header*/
    unsigned      reserved			:7; /* padding*/
} TxDescCtrl_t;


#ifdef HOST_COMPILE 
typedef uint16 TxAttr_t;
#else
typedef TxDescCtrl_t TxAttr_t;
#endif

/**************************************/
/*  Double Buffer Descriptor Fields   */
/**************************************/



#define DoubleBufferDesc uint16 length; /* Length of payload, including headers.				*/  \
    TxdRateSet_t     rate;          /* A bit mask that specifies the initial rate to be 	*/  \
	                                /* used. Possible values are:							*/  \
									/* 0x0001 - 1Mbits										*/  \
	                                /* 0x0002 - 2Mbits										*/  \
		                            /* 0x0004 - 5.5Mbits									*/  \
		                            /* 0x0008 - 6Mbits										*/  \
		                            /* 0x0010 - 9Mbits										*/  \
		                            /* 0x0020 - 11Mbits										*/  \
		                            /* 0x0040 - 12Mbits										*/  \
		                            /* 0x0080 - 18Mbits										*/  \
		                            /* 0x0100 - 22Mbits										*/  \
		                            /* 0x0200 - 24Mbits										*/  \
		                            /* 0x0400 - 36Mbits										*/  \
		                            /* 0x0800 - 48Mbits										*/  \
		                            /* 0x1000 - 54Mbits										*/  \
    uint32      	 expiryTime;	/* Time (in us) the packet can stay in the device 		*/  \
	                                /* before the packet expires.							*/  \
    uint8       	 xmitQueue;		/* The index of the Tx queue used for this packet.		*/  \
	uint8       	 descID;	    /* Identifier of the packet. This ID is used by the 	*/  \
	                                /* host for identifying the Tx Result of packet.		*/  \
    TxAttr_t      	 txAttr;		/* Bitwise fields - see TxDescCtrl_tdefinition above	*/  \
    uint16      	 fragThreshold;	/* The FW should cut the packet to fragments by 		*/	\
	                                /* this size.											*/  \
    uint8       	 numMemBlks;	/* Number of HW queue blocks to allocate for this 		*/  \
	                                /* packet.												*/  \
    uint8       	 reserved;      /* for padding to 32 bits boundry.						*/  \




typedef struct 
{
	DoubleBufferDesc
}DbTescriptor;

/******************************************************************************

    TxResultDescriptor_t

    the structure of the Tx result retrieved from FW upon TX completion.

******************************************************************************/

typedef enum
{
    TX_SUCCESS              = 0,     
	TX_DMA_ERROR            = BIT_7,
	TX_DISABLED             = BIT_6,
	TX_RETRY_EXCEEDED       = BIT_5,
	TX_TIMEOUT              = BIT_4,
	TX_KEY_NOT_FOUND        = BIT_3,
	TX_ENCRYPT_FAIL         = BIT_2,
	TX_UNAVAILABLE_PRIORITY = BIT_1
} TxDescStatus_enum;

#ifdef HOST_COMPILE
typedef uint8 TxDescStatus_e;
#else
typedef TxDescStatus_enum TxDescStatus_e;
#endif

typedef struct 
{
    uint8 done1;                     /* Ownership synchronization between the host and */
	                                 /* the firmware. If done1 and done2 are cleared, */
	                                 /* owned by the FW (no info ready). */
	
    uint8 descID;                    /* Packet Identifier - same value used in the Tx */
	                                 /* descriptor.*/

    uint16 mediumUsage;              /* Total air access duration consumed by this */
	                                 /* packet, including all retrys and overheads.*/

    uint32 mediumDelay;              /* Total media delay (from 1st EDCA AIFS counter until TX Complete).  */

    uint32 fwHandlingTime;           /* The time passed from host xfer to Tx-complete.*/

    uint8 lsbSecuritySequenceNumber; /* The LS-byte of the last TKIP sequence number. */
	                                 /* Saved per queue for recovery.*/
	
    uint8 ackFailures;               /* Retry count - The number of transmissions */
	                                 /* without successful ACK reception.*/

    TxdRateSet_t actualRate;         /* The rate that succeeded getting ACK - */
	                                 /* Valid only if status=TX_SUCCESS.*/

    uint16 reserved; 
 
	TxDescStatus_e  status;          /* The status of the transmission, indicating */
	                                 /* success or one of several possible reasons for */
	                                 /* failure. Refer to TxDescStatus_enum, above.*/
	
    uint8 done2;                     /* Refer to done1.*/
} TxResultDescriptor_t;



/******************************************************************************

	RX PATH

    The Rx path uses a double buffer and an RxControl structure, each located at
	a fixed address in the device memory. On startup, the host retrieves the 
	pointers to these addresses. A double buffer allows for continuous data flow
	towards the device. The host keeps track of which buffer is available and 
	alternates between them on a per packet basis.
    The size of each of the two buffers is large enough to hold the longest 802.3
	packet.
	A dedicated control block is used to manage the flow control between the host 
	and the device. The procedure is interrupt driven.

	The RX procedure is as follows:
	1.	The device generates an interrupt each time a new packet is received and
	    the corresponding interrupt is enabled in the host. There are two different 
		interrupt sources, one for each buffer. It is possible that both interrupt 
		sources are set when both buffers in the double buffer are pending with Rx
		data.
	2.	Depending on flow control conditions, the host reads a packet from the 
	    appropriate buffer in the double buffer.
	3.	The host triggers an interrupt in the device (using the HW mechanism of EOT), 
	    indicating the address of the buffer which has been read.
	4.	The device services the complete interrupt and prepares the next Rx packet,
	    if available. In addition, the device updates the control block and issues
		an interrupt to the host.
	
******************************************************************************/

/******************************************************************************

    RxPathStatusReg_t

    The structure of the Rx Path Status register. This status register 
	represents both the status of the double-buffer (xfer protocol) and the 
	number of the pending packet in receive queue.
	
******************************************************************************/

#ifdef HOST_COMPILE
#define    RX_DESC_VALID_FCS         0x0001 
#define    RX_DESC_MATCH_RXADDR1     0x0002 
#define    RX_DESC_MCAST             0x0004 
#define    RX_DESC_STAINTIM          0x0008 
#define    RX_DESC_VIRTUAL_BM        0x0010
#define    RX_DESC_BCAST             0x0020
#define    RX_DESC_MATCH_SSID        0x0040
#define    RX_DESC_MATCH_BSSID       0x0080
#define    RX_DESC_ENCRYPTION_MASK   0x0300
#define    RX_DESC_MEASURMENT        0x0400
#define    RX_DESC_SEQNUM_MASK       0x1800
#define	   RX_DESC_MIC_FAIL			 0x2000
#define	   RX_DESC_DECRYPT_FAIL		 0x4000
typedef uint16 RxFlags_t;  
#else
typedef struct
{
    unsigned    validFcs        : 1; /* Indicates whether a received frame had a valid FCS*/

    unsigned    matchRxAddr1    : 1; /* Indicates whether a received frame contained a */
                                   /* matching receive address in Address 1.*/

    unsigned    mcast           : 1; /* Indicates whether a received frame contains a */
	                               /* unicast/directed or group receive address, bit 40*/
	                               /* of Addr1 (0 = individual, 1 = group).*/

    unsigned    StaInTIM        : 1; /* Indicates that the TIM in a Beacon frame contained*/
	                               /* a 1 in the bit position representing this STA */
	                               /* (i.e. the AP contains one or more buffered frame(s)*/
	                               /* for this STA).*/

    unsigned    virtualBM       : 1; /* If asserted, the VBM in a Beacon frame contained */
	                               /* more than 1 asserted bit*/

    unsigned    bcast           : 1; /* Indicates whether the received frames address 1 */
                                   /* is a broadcast address.*/

    unsigned    matchSSID       : 1; /* Indicates whether the received frame containing */
	                               /* a matching SSID (either broadcast or specific).*/

    unsigned    matchBSSID      : 1; /* Indicates whether the received frame containing */
	                               /* a matching BSSID (either broadcast or specific).*/

    unsigned    encryption      : 2; /* This field indicates the encryption type of the */
                                   /* packet: 00 - None, 01 - WEP, 10 - TKIP, 11 - AES*/

    unsigned    measurement     : 1; /* Indicates whether the packet was received during */
	                               /* measurement process or not*/

    unsigned    seqnum          : 2; /* Sequence number of the current frame*/

	unsigned	micFail			: 1; /* MIC Fail indication */

	unsigned	decryptFail		: 1; /* DECRYPT Fail indication */

    unsigned    reserved2       : 1;
} RxFlags_t;
#endif

/******************************************************************************

    RxIfDescriptor_t

    the structure of the Rx Descriptor recieved by HOST.

******************************************************************************/

typedef struct
{
    uint32          timestamp;     /* Timestamp in microseconds,     */
    
	uint16          length;        /* Length of payload (including headers)*/
    
	RxFlags_t       flags;         /* See RxFlags_t for details. */

    uint8           type;          /* Protocol type: */
	                               /* 0 - 802.11*/
	                               /* 1 - 802.3*/
	                               /* 2 - IP*/
	                               /* 3 - Raw Codec*/

    uint8           rate;          /* Recevied Rate:*/
                                   /* 0x0A - 1MBPS*/
                                   /* 0x14 - 2MBPS   */
		                           /* 0x37 - 5_5MBPS */
		                           /* 0x0B - 6MBPS   */
		                           /* 0x0F - 9MBPS   */
		                           /* 0x6E - 11MBPS  */
		                           /* 0x0A - 12MBPS  */
		                           /* 0x0E - 18MBPS  */
		                           /* 0xDC - 22MBPS  */
		                           /* 0x09 - 24MBPS  */
		                           /* 0x0D - 36MBPS  */
		                           /* 0x08 - 48MBPS  */
		                           /* 0x0C - 54MBPS  */

    uint8           modPre;        /* Modulation and Preamble of received packet*/
    
	uint8           chanNum;       /* The received channel*/
    
	uint8           band;          /* 0 - 2.4Ghz*/
	                               /* 1 - 5Ghz*/
    
	int8            rssi;          /* RSSI value in db */
    
	uint8           rcpi;          /* RCPI value in db */
    
	uint8           snr;           /* SNR in db*/
    
} RxIfDescriptor_t;



#endif /* PUBLIC_DESCRIPTORS_H*/



