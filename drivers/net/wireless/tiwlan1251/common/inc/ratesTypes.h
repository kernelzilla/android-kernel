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

#ifndef RATES_TYPES_H
#define RATES_TYPES_H

typedef enum
{
    DRV_RATE_AUTO       = 0,
    DRV_RATE_1M         = 1,
    DRV_RATE_2M         = 2,
    DRV_RATE_5_5M       = 3,
    DRV_RATE_11M        = 4,
    DRV_RATE_22M        = 5,
    DRV_RATE_6M         = 6,
    DRV_RATE_9M         = 7,
    DRV_RATE_12M        = 8,
    DRV_RATE_18M        = 9,
    DRV_RATE_24M        = 10,
    DRV_RATE_36M        = 11,
    DRV_RATE_48M        = 12,
    DRV_RATE_54M        = 13,
    DRV_RATE_MAX        = 13,
    DRV_RATE_INVALID= 0xFF
} rate_e;


typedef enum
{
    DRV_RATE_MASK_AUTO          = DRV_RATE_AUTO, /*0x0000,*/
    DRV_RATE_MASK_1_BARKER      = (1<<(DRV_RATE_1M - 1)), /*0x0001,*/
    DRV_RATE_MASK_2_BARKER      = (1<<(DRV_RATE_2M - 1)), /*0x0002,*/
    DRV_RATE_MASK_5_5_CCK       = (1<<(DRV_RATE_5_5M - 1)), /*0x0004,*/
    DRV_RATE_MASK_11_CCK        = (1<<(DRV_RATE_11M - 1)), /*0x0008,*/
    DRV_RATE_MASK_22_PBCC       = (1<<(DRV_RATE_22M - 1)), /*0x0010,*/
    DRV_RATE_MASK_6_OFDM        = (1<<(DRV_RATE_6M - 1)), /*0x0020,*/
    DRV_RATE_MASK_9_OFDM        = (1<<(DRV_RATE_9M - 1)), /*0x0040,*/
    DRV_RATE_MASK_12_OFDM       = (1<<(DRV_RATE_12M - 1)), /*0x0080,*/
    DRV_RATE_MASK_18_OFDM       = (1<<(DRV_RATE_18M - 1)), /*0x0100,*/
    DRV_RATE_MASK_24_OFDM       = (1<<(DRV_RATE_24M - 1)), /*0x0200,*/
    DRV_RATE_MASK_36_OFDM       = (1<<(DRV_RATE_36M - 1)), /*0x0400,*/
    DRV_RATE_MASK_48_OFDM       = (1<<(DRV_RATE_48M - 1)), /*0x0800,*/
    DRV_RATE_MASK_54_OFDM       = (1<<(DRV_RATE_54M - 1)), /*0x1000*/
} rateMask_e;

/*GWSI_RATE*/
#define GWSI_1Mbits 	0x00000001
#define GWSI_2Mbits 	0x00000002
#define GWSI_5_5Mbits	0x00000004
#define GWSI_6Mbits	0x00000008
#define GWSI_9Mbits	0x00000010
#define GWSI_11Mbits	0x00000020
#define GWSI_12Mbits	0x00000040
#define GWSI_18Mbits	0x00000080
#define GWSI_22Mbits	0x00000100
#define GWSI_24Mbits	0x00000200
#define GWSI_36Mbits	0x00000800
#define GWSI_48Mbits	0x00001000
#define GWSI_54Mbits	0x00002000

/*HW_RATE*/
#define HW_RATE_1M			(0x0A)
#define HW_RATE_2M			(0x14)
#define HW_RATE_5_5M		(0x37)
#define HW_RATE_5_5M_PBCC	(0xB7)
#define HW_RATE_11M			(0x6E)
#define HW_RATE_11M_PBCC	(0xEE)
#define HW_RATE_22M_PBCC	(0xDC)
#define HW_RATE_6M			(0x0B)
#define HW_RATE_9M			(0x0F)
#define HW_RATE_12M			(0x0A)
#define HW_RATE_18M			(0x0E)
#define HW_RATE_24M			(0x09)
#define HW_RATE_36M			(0x0D)
#define HW_RATE_48M			(0x08)
#define HW_RATE_54M			(0x0C)

#define HW_BIT_RATE_1MBPS   0x00000001
#define HW_BIT_RATE_2MBPS   0x00000002
#define HW_BIT_RATE_5_5MBPS 0x00000004
#define HW_BIT_RATE_6MBPS   0x00000008
#define HW_BIT_RATE_9MBPS   0x00000010
#define HW_BIT_RATE_11MBPS  0x00000020
#define HW_BIT_RATE_12MBPS  0x00000040
#define HW_BIT_RATE_18MBPS  0x00000080
#define HW_BIT_RATE_22MBPS  0x00000100
#define HW_BIT_RATE_24MBPS  0x00000200
#define HW_BIT_RATE_36MBPS  0x00000400
#define HW_BIT_RATE_48MBPS  0x00000800
#define HW_BIT_RATE_54MBPS  0x00001000


#define SHORT_PREAMBLE_BIT  BIT_0               /*CCK or Barker depending on the rate*/
#define OFDM_MOD_TYPE       BIT_6
#define PBCC_MOD_TYPE       BIT_7

typedef enum
{
    MOD_PBCC = 1,
    MOD_CCK,
    MOD_OFDM
}Modulation_e;


#endif

