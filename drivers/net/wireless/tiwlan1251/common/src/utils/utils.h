/** \file utils.h
 *  \brief utils API
 *
 *  \see utils.c
 */
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

/****************************************************************************/
/*                                                                          */
/*    MODULE:   utils.h                                                     */
/*    PURPOSE:  utilities API, contains some utilites function to be used   */
/*              by the COre & HAL                                           */
/*                                                                          */
/***************************************************************************/
#ifndef __UTILS_H__
#define __UTILS_H__

#include "osTIType.h"
#include "commonTypes.h"
#include "memMngrEx.h"
#include "802_11Defs.h"

/* TODO: replace the following macros with a faster code. */
#define MAC_COPY(pOsContext,pDstMac,pSrcMac)         \
        os_memoryCopy(pOsContext, (void *)((pDstMac)->addr), (void *)((pSrcMac)->addr), MAC_ADDR_LEN)
#define MAC_EQUAL(pDstMac,pSrcMac)                   \
        ((pDstMac)->addr[0] == (pSrcMac)->addr[0] && \
         (pDstMac)->addr[1] == (pSrcMac)->addr[1] && \
         (pDstMac)->addr[2] == (pSrcMac)->addr[2] && \
         (pDstMac)->addr[3] == (pSrcMac)->addr[3] && \
         (pDstMac)->addr[4] == (pSrcMac)->addr[4] && \
         (pDstMac)->addr[5] == (pSrcMac)->addr[5])
#define MAC_BROADCAST(pMac)                          \
         ((pMac)->addr[0] == 0xFF &&                 \
          (pMac)->addr[1] == 0xFF &&                 \
          (pMac)->addr[2] == 0xFF &&                 \
          (pMac)->addr[3] == 0xFF &&                 \
          (pMac)->addr[4] == 0xFF &&                 \
          (pMac)->addr[5] == 0xFF)
#define MAC_MULTICAST(pMac) ((pMac)->addr[0] & 0x01)
#define MAC_NULL(pMac)                               \
        ((pMac)->addr[0] == 0x00 &&                  \
         (pMac)->addr[1] == 0x00 &&                  \
         (pMac)->addr[2] == 0x00 &&                  \
         (pMac)->addr[3] == 0x00 &&                  \
         (pMac)->addr[4] == 0x00 &&                  \
         (pMac)->addr[5] == 0x00)


#ifndef offsetof
#define offsetof(type, field)    ((unsigned int) (&(((type *)(0))->field)))
#endif


/* NOTE - Network byte order is BIG endian. */

static __inline unsigned short int __byte_swap_16 (unsigned short int __bsx) {
  return ((((__bsx) >> 8) & 0xff) | (((__bsx) & 0xff) << 8));
}

static __inline unsigned int __byte_swap_32 (unsigned int __bsx) {
  return ((((__bsx) & 0xff000000) >> 24) | (((__bsx) & 0x00ff0000) >>  8) |
          (((__bsx) & 0x0000ff00) <<  8) | (((__bsx) & 0x000000ff) << 24));
}


#ifdef __BYTE_ORDER_BIG_ENDIAN


#define wlan_ntohl(x)       (x)
#define wlan_ntohs(x)       (x)
#define wlan_htonl(x)       (x)
#define wlan_htons(x)       (x)

#define ENDIAN_HANDLE_WORD(x)   __byte_swap_16 (x)
#define ENDIAN_HANDLE_LONG(x)   __byte_swap_32 (x)

/* int64 handling macros */
#define INT64_LOWER(x) *(((UINT32*)&(x))+1)
#define INT64_HIGHER(x) *((UINT32*)&(x))

#else

#ifdef __BYTE_ORDER_LITTLE_ENDIAN

#define wlan_ntohl(x)       __byte_swap_32 (x)
#define wlan_ntohs(x)       __byte_swap_16 (x)
#define wlan_htonl(x)       __byte_swap_32 (x)
#define wlan_htons(x)       __byte_swap_16 (x)

#define ENDIAN_HANDLE_WORD(x)   (x)
#define ENDIAN_HANDLE_LONG(x)   (x)

/* int64 handling macros */
#define INT64_HIGHER(x) *(((UINT32*)&(x))+1)
#define INT64_LOWER(x) *((UINT32*)&(x))

/*#define COPY_UNALIGNED_WORD(srcWord, destWord)        ((UINT8 *)&destWord)[0] = ((UINT8 *)&srcWord)[0]; ((UINT8 *)&destWord)[1] = ((UINT8 *)&srcWord)[1];
#define COPY_UNALIGNED_LONG(srcLong, destLong)      ((UINT8 *)&destWord)[0] = ((UINT8 *)&srcWord)[0]; ((UINT8 *)&destWord)[1] = ((UINT8 *)&srcWord)[1];((UINT8 *)&destWord)[2] = ((UINT8 *)&srcWord)[2]; ((UINT8 *)&destWord)[3] = ((UINT8 *)&srcWord)[3];
*/
#else

#error "MUST define byte order (BIG/LITTLE ENDIAN)"

#endif
#endif

#define COPY_UNALIGNED_WORD(pDest, pSrc)           {((UINT8 *)(pDest))[0] = ((UINT8 *)(pSrc))[0];\
                                                    ((UINT8 *)(pDest))[1] = ((UINT8 *)(pSrc))[1];}

#define COPY_UNALIGNED_LONG(pDest, pSrc)           {((UINT8 *)(pDest))[0] = ((UINT8 *)(pSrc))[0];\
                                                    ((UINT8 *)(pDest))[1] = ((UINT8 *)(pSrc))[1];\
                                                    ((UINT8 *)(pDest))[2] = ((UINT8 *)(pSrc))[2];\
                                                    ((UINT8 *)(pDest))[3] = ((UINT8 *)(pSrc))[3];}

void utils_nullMemoryFree(void* pOsContext,
                          void* pMemPtr,
                          unsigned long size);

void utils_nullTimerDestroy(void* pOsContext,
                         void* pTimerHandle);

#define MAX(a,b)  (((a) > (b)) ? (a) : (b))
#define MIN(a,b)  (((a) < (b)) ? (a) : (b))

#ifndef min
# define min MIN
#endif

#ifndef max
# define max MAX
#endif


#define MAKE_BASIC_RATE(rate)                           rate |= 0x80

#define IS_BASIC_RATE(rate)                             rate & 0x80

#define IS_ACTIVE_RATE(rate)                            !(rate & 0x80)

rate_e networkToHostRate(UINT8 rate);

UINT8 hostToNetworkRate(rate_e rate);


UINT8 getMaxBasicRatefromString(UINT8 *ratesString, UINT8 len, UINT8 maxRate);

rate_e getMaxRatefromBitmap(UINT32 ratesBitMap);

UINT8 getMaxActiveRatefromString(UINT8 *ratesString, UINT8 len, UINT8 maxRate);

TI_STATUS validateNetworkRate(UINT8 rate);

UINT8 hostToUtilityRate(rate_e rate);

rate_e utilityToHostRate(UINT8 rate);

UINT8 hostRateToNumber(rate_e rate);
rate_e  RateNumberToHost(UINT8 rateIn);

void bitMapToNetworkStringRates(UINT32 suppRatesBitMap, UINT32 basicRatesBitMap,
                                UINT8 *string, UINT32 *len,
                                UINT32 *firstOFDMrateLoc);

void networkStringToBitMapSuppRates(UINT32 *bitMap, UINT8 *string, UINT32 len);
void networkStringToBitMapBasicRates(UINT32 *bitMap, UINT8 *string, UINT32 len);

UINT32 translateBasicRateValueToMask(UINT32 value, BOOL dot11a);
UINT32 translateSupportedRateValueToMask(UINT32 value, BOOL dot11a);
void validateRates(UINT32 *pBasicRateMask, UINT32 *pSuppRateMask, UINT32 *pTxRate, modulationType_e *modulation, BOOL dot11a);
rate_e calculateMaxSupportedRate(UINT32 *pSuppRateMask);
rate_e findMaxActiveRate(UINT32 ratesBitMap);
void   validateRatesVsBand(UINT32 *supportedMask, UINT32 *basicMask, BOOL dot11a);

BOOL utils_isAnySSID(ssid_t *pSsid);
BOOL utils_isJunkSSID(ssid_t *pSsid);
BOOL utils_isIESSID_Broadcast(dot11_SSID_t *pIESsid); /* routinte to check for Junk SSID in SSID IE */
void    MsduContentDump (mem_MSDU_T* pMsdu, char *str);


void HexDumpData (UINT8 *data, int datalen);
void msduContentDump (mem_MSDU_T* pMsdu, char *str);


BOOL parseIeBuffer(TI_HANDLE hOs, UINT8 *pIeBuffer, UINT16 length, UINT8 desiredIeId, UINT8 **pDesiredIe, UINT8 *pMatchBuffer, UINT8 matchBufferLen);
void TiWlanIntToStr(UINT8 number , char *string, UINT8 radix);

UINT32 getBasicRateMaskForSpecialBGchannel(void);
UINT32 getSupportedRateMaskForSpecialBGchannel(void);

int ConvertHwBitRateToAppRate(UINT32 HwRate,rate_e *AppRate);


void getMaxRate(UINT32 ratesBitMap, rate_e *rate, modulationType_e *modulation, dot11mode_e operationMode);
void getMinRate(UINT32 ratesBitMap, rate_e *rate, modulationType_e *modulation, dot11mode_e operationMode);

UINT32 reminder64( UINT64 dividee, UINT32 divider );
int ConvertHwBitRateToAppRate(UINT32 HwRate,rate_e *AppRate);
int ConvertAppRatesToBitmap(UINT16 AppRatesBitmap, UINT32 *HwRatesBitmap);
int ConvertAppRateToHwBitMapRate(UINT32 AppRate, UINT32 *HwRate);
void convert_hex_to_string(tiUINT8 *pBuffer, char *pString, tiUINT8 Size);
rate_e ConvertHwRateToDrvRate(UINT8 HwRate, BOOL bOFDMMudulation);
UINT8 ConvertDrvRate2HwRate(rate_e eRate);
RateIndex_e rateNumberToIndex(UINT8 uRate);

/* returns TI_STATUS as string */
char* convertTI_STATUS_toString(TI_STATUS status);

/*
++++++++ Profiling code ++++++++
*/
#define UTIL_DEBUG_PROFILE (0)

void convert_hex_to_string(tiUINT8 *pBuffer, char *pString, tiUINT8 Size);

/* 
* Small macro to convert Dbm units into Dbm/10 units. This macro is important
* in order to avoid over-flow of Dbm units bigger than 25
*/
#define DBM2DBMDIV10(uTxPower) \
	((uTxPower) > (MAX_TX_POWER / DBM_TO_TX_POWER_FACTOR) ? \
		MAX_TX_POWER : (uTxPower) * DBM_TO_TX_POWER_FACTOR)		

#if UTIL_DEBUG_PROFILE
typedef struct
{
    UINT32 TIWlanModuleLogName;
    UINT32 Event;
    UINT32 Param_1;
    UINT32 Param_2;
    UINT32 timeStamp;
} profileInfoElement_t;

enum
{
    PROFILE_BUFFER_SIZE = 10000
};

typedef struct
{
    profileInfoElement_t profileInfoElement[PROFILE_BUFFER_SIZE];
    UINT32 currentInfoElement;
    BOOL overlap;
} profileInfo_t;

void util_initProfile(void);

void util_recordProfile(UINT32 theTIWlanModuleLogName,
                        UINT32 theEvent,
                        UINT32 theParam_1,
                        UINT32 theParam_2);

void util_printProfile(void);
#endif /* UTIL_DEBUG_PROFILE */


#endif /* __UTILS_H__ */
