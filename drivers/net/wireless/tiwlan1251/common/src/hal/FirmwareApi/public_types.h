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

/** \file public_types.h
 *  \brief Basic types and general macros, bit manipulations, etc.
 *
 */

#ifndef PUBLIC_TYPES_H
#define PUBLIC_TYPES_H


/******************************************************************************

    Basic definitions

******************************************************************************/
#ifndef uint8
typedef unsigned char   uint8;
#endif
#ifndef uint16
typedef unsigned short  uint16;
#endif
#ifndef uint32
typedef unsigned long int    uint32;
#endif

#ifndef int8
typedef signed char     int8;
#endif
#ifndef int16
typedef short           int16;
#endif
#ifndef int32
typedef long int        int32;
#endif

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/* !! LAC - NULL definition conflicts with the compilers version.
   I redid this definition to the ANSI version....
    #define NULL 0
*/
#if !defined( NULL )
#if defined( __cplusplus )
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/* Bool_e should be used when we need it to be a byte. */
typedef uint8           Bool_e;

/* Bool32 should be used whenever possible for efficiency */
typedef uint32          Bool32;

/* to align enum to 32/16 bits */
#define MAX_POSITIVE32 0x7FFFFFFF
#define MAX_POSITIVE16 0x7FFF


#define MAC_ADDR_SIZE 6

#ifndef HOST_COMPILE
#define HOST_COMPILE	/* temp fix for suppl build err */
#endif

#ifdef HOST_COMPILE
#else
typedef struct macAddress_t
{
    uint8 addr[MAC_ADDR_SIZE];
}macAddress_t;
#endif


#define  BIT_0    0x00000001
#define  BIT_1    0x00000002
#define  BIT_2    0x00000004
#define  BIT_3    0x00000008
#define  BIT_4    0x00000010
#define  BIT_5    0x00000020
#define  BIT_6    0x00000040
#define  BIT_7    0x00000080
#define  BIT_8    0x00000100
#define  BIT_9    0x00000200
#define  BIT_10   0x00000400
#define  BIT_11   0x00000800
#define  BIT_12   0x00001000
#define  BIT_13   0x00002000
#define  BIT_14   0x00004000
#define  BIT_15   0x00008000
#define  BIT_16   0x00010000
#define  BIT_17   0x00020000
#define  BIT_18   0x00040000
#define  BIT_19   0x00080000
#define  BIT_20   0x00100000
#define  BIT_21   0x00200000
#define  BIT_22   0x00400000
#define  BIT_23   0x00800000
#define  BIT_24   0x01000000
#define  BIT_25   0x02000000
#define  BIT_26   0x04000000
#define  BIT_27   0x08000000
#define  BIT_28   0x10000000
#define  BIT_29   0x20000000
#define  BIT_30   0x40000000
#define  BIT_31   0x80000000

#define  BIT_32   0x00000001
#define  BIT_33   0x00000002
#define  BIT_34   0x00000004
#define  BIT_35   0x00000008
#define  BIT_36   0x00000010
#define  BIT_37   0x00000020
#define  BIT_38   0x00000040
#define  BIT_39   0x00000080
#define  BIT_40   0x00000100
#define  BIT_41   0x00000200
#define  BIT_42   0x00000400
#define  BIT_43   0x00000800
#define  BIT_44   0x00001000
#define  BIT_45   0x00002000
#define  BIT_46   0x00004000
#define  BIT_47   0x00008000
#define  BIT_48   0x00010000
#define  BIT_49   0x00020000
#define  BIT_50   0x00040000
#define  BIT_51   0x00080000
#define  BIT_52   0x00100000
#define  BIT_53   0x00200000
#define  BIT_54   0x00400000
#define  BIT_55   0x00800000
#define  BIT_56   0x01000000
#define  BIT_57   0x02000000
#define  BIT_58   0x04000000
#define  BIT_59   0x08000000
#define  BIT_60   0x10000000
#define  BIT_61   0x20000000
#define  BIT_62   0x40000000
#define  BIT_63   0x80000000


/******************************************************************************

    CHANNELS, BAND & REG DOMAINS definitions

******************************************************************************/


typedef uint8 Channel_e;

typedef enum
{
    RADIO_BAND_2_4GHZ = 0,  /* 2.4 Ghz band */
    RADIO_BAND_5GHZ = 1,    /* 5 Ghz band */
    RADIO_BAND_JAPAN_4_9_GHZ = 2,
    DEFAULT_BAND = RADIO_BAND_2_4GHZ,
    INVALID_BAND = 0xFE,
    MAX_RADIO_BANDS = 0xFF
} RadioBand_enum;

#ifdef HOST_COMPILE
typedef uint8 RadioBand_e;
#else
typedef RadioBand_enum RadioBand_e;
#endif


typedef enum
{
    NO_RATE      = 0,
    RATE_1MBPS   = 0x0A,
    RATE_2MBPS   = 0x14,
    RATE_5_5MBPS = 0x37,
    RATE_6MBPS   = 0x0B,
    RATE_9MBPS   = 0x0F,
    RATE_11MBPS  = 0x6E,
    RATE_12MBPS  = 0x0A,
    RATE_18MBPS  = 0x0E,
    RATE_22MBPS  = 0xDC,
    RATE_24MBPS  = 0x09,
    RATE_36MBPS  = 0x0D,
    RATE_48MBPS  = 0x08,
    RATE_54MBPS  = 0x0C
} Rate_enum;

#ifdef HOST_COMPILE
typedef uint8 Rate_e;
#else
typedef Rate_enum Rate_e;
#endif

typedef enum
{
	RATE_INDEX_1MBPS   =  0,
	RATE_INDEX_2MBPS   =  1,
	RATE_INDEX_5_5MBPS =  2,
	RATE_INDEX_6MBPS   =  3,
	RATE_INDEX_9MBPS   =  4,
	RATE_INDEX_11MBPS  =  5,
	RATE_INDEX_12MBPS  =  6,
	RATE_INDEX_18MBPS  =  7,
	RATE_INDEX_22MBPS  =  8,
	RATE_INDEX_24MBPS  =  9,
	RATE_INDEX_36MBPS  =  10,
	RATE_INDEX_48MBPS  =  11,
	RATE_INDEX_54MBPS  =  12,
	RATE_INDEX_MAX     =  RATE_INDEX_54MBPS,
	MAX_RATE_INDEX,
	INVALID_RATE_INDEX = MAX_RATE_INDEX,
	RATE_INDEX_ENUM_MAX_SIZE = 0x7FFFFFFF
} RateIndex_e;

#define SHORT_PREAMBLE_BIT   BIT_0 /* CCK or Barker depending on the rate */
#define OFDM_RATE_BIT        BIT_6
#define PBCC_RATE_BIT        BIT_7


typedef enum
{
    CCK_LONG = 0,
    CCK_SHORT = SHORT_PREAMBLE_BIT,
    PBCC_LONG = PBCC_RATE_BIT,
    PBCC_SHORT = PBCC_RATE_BIT | SHORT_PREAMBLE_BIT,
    OFDM = OFDM_RATE_BIT
} Mod_enum;

#ifdef HOST_COMPILE
typedef  uint8 Mod_e;
#else
typedef  Mod_enum Mod_e;
#endif


typedef uint16 BasicRateSet_t;


/******************************************************************************

Transmit-Descriptor RATE-SET field definitions...

Define a new "Rate-Set" for TX path that incorporates the
Rate & Modulation info into a single 16-bit field.

TxdRateSet_t:
  b15    - Indicates Preamble type (1=SHORT, 0=LONG).
           Notes:
             Must be LONG (0) for 1Mbps rate.
             Does not apply (set to 0) for RevG-OFDM rates.
  b14    - Indicates PBCC encoding (1=PBCC, 0=not).
           Notes:
             Does not apply (set to 0) for rates 1 and 2 Mbps.
             Does not apply (set to 0) for RevG-OFDM rates.
  b13    - Unused (set to 0).
  b12-b0 - Supported Rate indicator bits as defined below.

******************************************************************************/

typedef uint16 TxdRateSet_t;


/******************************************************************************
 
    CHIP_ID definitions
 
******************************************************************************/
#define TNETW1150_PG10_CHIP_ID          0x04010101
#define TNETW1150_PG11_CHIP_ID          0x04020101
#define TNETW1150_CHIP_ID               0x04030101  /* 1150 PG2.0, 1250, 1350, 1450*/
#define TNETW1350A_CHIP_ID              0x06010101
#define TNETW1251_CHIP_ID_PG1_0         0x07010101
#define TNETW1251_CHIP_ID_PG1_1         0x07020101
#define TNETW1251_CHIP_ID_PG1_2	        0x07030101

#define IS_CHIP_PG_LESS_THEN_PG12()      ((CHIP_ID == TNETW1251_CHIP_ID_PG1_0) || (CHIP_ID == TNETW1251_CHIP_ID_PG1_1))

/******************************************************************************
Enable bits for SOC1251 PG1.2
******************************************************************************/
#define PDET_BINARY_OFFSET_EN   BIT_0
#define STOP_TOGGLE_MONADC_EN   BIT_1
#define RX_ADC_BIAS_DEC_EN      BIT_2
#define RX_LNB_AND_DIGI_GAIN_EN BIT_3


#endif /* PUBLIC_TYPES_H*/
