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
/**************************************************************************/
/*                                                                        */
/*   MODULE:  whalHwEeprom.h                                              */
/*   PURPOSE: Wlan hardware EEPROM access routines implemenatation        */
/*                                                                        */
/**************************************************************************/
#ifndef _WHAL_HW_EEPROM_H
#define _WHAL_HW_EEPROM_H

#include "whalCommon.h"
#include "TNETWIF.h"

typedef struct _HwEeprom_T
{
	TI_HANDLE  hTNETWIF;
	TI_HANDLE  hOs;
	TI_HANDLE  hReport;
} HwEeprom_T;

extern HwEeprom_T *whal_hwEeprom_Create(TI_HANDLE hOs);
extern int whal_hwEeprom_Destroy(HwEeprom_T *pHwEeprom);
extern int whal_hwEeprom_Config (HwEeprom_T *pHwEeprom, TI_HANDLE hTNETWIF, TI_HANDLE hReport);

#ifdef USE_SYNC_API

extern int whal_hwEeprom_GetRadioTypeAndEEPROMversion(HwEeprom_T *pHwEeprom, UINT32 *major, UINT32 *minor, UINT32 *bugfix);

/****************************
 Offsets of Wlan Hardware data 
 ****************************/
#define HW_EEPROM_AGC_TABLE_ADDR   0x18C
#define HW_EEPROM_DAC_ADDR         0x20D
#define HW_EEPROM_BIAS_ADDR        0x20E    
#define HW_EEPROM_OSC_ADDR         0x215
#define MAX_AGC_TABLE_ENTRIES       128

/*******************
 EEPROM access APIs
 *******************/
#define MAX_OSC_CAL         15
#define EEPROM_ACCESS_TO    10000   /* timeout counter */
#define INVALID_PARAMETER1      -101







/************************************************************************
*        UINT32 whal_hwEepromReadByte(UINT16 wAddr,UINT8 *pbVal)       *
*************************************************************************
* DESCRIPTION:  Reads a single byte from EEPROM.                        *
*                                                                       *
* INPUT:        wAddr - 16-bits EEPROM addrress                         *
*               pbVal - pointer the to output parameter - EEPROM value  *     
*                                                                       *
* OUTPUT:       *pbVal contains EEPROM value                            *   
*                                                                       *
* RETURN:       OK - successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32   whal_hwEepromReadByte(HwEeprom_T *pHwEeprom, UINT16 wAddr,UINT8 *pbVal);

/************************************************************************
*       UINT32 whal_hwEepromWriteByte(UINT16 wAddr,UINT8 bVal)         *
*************************************************************************
* DESCRIPTION:  Writes a single byte to EEPROM                          *  
*                                                                       *
* INPUT:        wAddr - 16-bits EEPROM addrress                         *
*               bVal  - new value                                       *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       OK - successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromWriteByte(HwEeprom_T *pHwEeprom, UINT16 wAddr,UINT8 bVal);

/************************************************************************
*     UINT32 whal_hwEepromWriteByteNoUnp(UINT16 wAddr,UINT8 bVal)      *
*************************************************************************
* DESCRIPTION:  Writes a single byte to EEPROM. The caller must         *
*               unprotect the EEPROM first. This is done for fast       *
*               programming of several data bytes.                      *
*                                                                       *
* INPUT:        wAddr - 16-bits EEPROM addrress                         *
*               bVal  - new value                                       *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       OK - successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromWriteByteNoUnp(HwEeprom_T *pHwEeprom, UINT16 wAddr,UINT8 bVal);

/************************************************************************
*           void whal_hwEepromProtect(void)                            *
*************************************************************************
* DESCRIPTION:  Set EEPROM write protection.                            *
*               Inhibits writing to the EEPROM.                         *
*                                                                       *
* INPUT:        N/A                                                     *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       N/A                                                     *
*************************************************************************/
void	   whal_hwEepromProtect(HwEeprom_T *pHwEeprom);

/************************************************************************
*           void whal_hwEepromUnprotect(void)                          *
*************************************************************************
* DESCRIPTION:  Remove EEPROM write protection.                         *
*               Enables writing to the EEPROM.                          *
*                                                                       *
* INPUT:        N/A                                                     *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       N/A                                                     *
*************************************************************************/
void	   whal_hwEepromUnprotect(HwEeprom_T *pHwEeprom);

/*************************
 EEPROM data manipulation
 *************************/

/************************************************************************
*           UINT32 whal_hwEepromGetCalValue(UINT8 *pbVal)              *
*************************************************************************
* DESCRIPTION:  Reads oscillator cal. value from EEPROM                 *
*                                                                       *
* INPUT:        N/A                                                     *
*                                                                       *
* OUTPUT:       pbVal - pointer to the output parameter                 *
*                                                                       *
* RETURN:       OK   successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromGetCalValue(HwEeprom_T *pHwEeprom, UINT8 *pbVal);

/************************************************************************
*           UINT32 whal_hwEepromSetCalValue(UINT8 bVal)                *
*************************************************************************
* DESCRIPTION:  Writes new oscillator cal. value to EEPROM              *
*                                                                       *
* INPUT:        bVal - new oscillator cal. value                        *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       OK   successful                                         *
*               NOK  timeout or invalid value                           *
*************************************************************************/
UINT32	whal_hwEepromSetCalValue(HwEeprom_T *pHwEeprom, UINT8 bVal);

/************************************************************************
*           UINT32 whal_hwEepromGetBiasValue(UINT8 *pbVal)             *
*************************************************************************
* DESCRIPTION:  Reads bias value from EEPROM                            *
*                                                                       *
* INPUT:        N/A                                                     *
*                                                                       *
* OUTPUT:       pbVal - pointer to the output parameter                 *
*                                                                       *
* RETURN:       OK   successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromGetBiasValue(HwEeprom_T *pHwEeprom, UINT8 *pbVal);

/************************************************************************
*           UINT32 whal_hwEepromSetBiasValue(UINT8 bVal)               *
*************************************************************************
* DESCRIPTION:  Writes new bias value to EEPROM                         *
*                                                                       *
* INPUT:        bVal - new bias value                                   *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       OK   successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromSetBiasValue(HwEeprom_T *pHwEeprom, UINT8 bVal);

/************************************************************************
*           UINT32 whal_hwEepromGetDACValue(UINT8 *pbVal)              *
*************************************************************************
* DESCRIPTION:  Reads DAC value from EEPROM                             *
*                                                                       *
* INPUT:        N/A                                                     *
*                                                                       *
* OUTPUT:       pbVal - pointer to the output parameter                 *
*                                                                       *
* RETURN:       OK   successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromGetDACValue(HwEeprom_T *pHwEeprom, UINT8 *pbVal);

/************************************************************************
*           UINT32 whal_hwEepromSetDACValue(UINT8 bVal)                *
*************************************************************************
* DESCRIPTION:  Writes new DAC value to EEPROM                          *
*                                                                       *
* INPUT:        bVal - new DAC value                                    *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       OK   successful                                         *
*               NOK  timeout                                            *
*************************************************************************/
UINT32	whal_hwEepromSetDACValue(HwEeprom_T *pHwEeprom, UINT8 bVal);

/************************************************************************
*           int whal_hwEepromLoadBaseBandTable(void)                   *
*************************************************************************
* DESCRIPTION:  Loads BB registers table from EEPROM                    *
*                                                                       *
* INPUT:        N/A                                                     *
*                                                                       *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       N/A                                                     *
*************************************************************************/
int		whal_hwEepromLoadBaseBandTable(HwEeprom_T *pHwEeprom);

/************************************************************************
*   UINT32 whal_hwEepromGetAGCCell(UINT8 bTableOffset, UINT8 *pbVal)   *
*************************************************************************
* DESCRIPTION:  Reads one cell from ACG table                           *
*                                                                       *
* INPUT:        bTableOffset - zero-based offset of the cell in AGC     *   
*                              table                                    *
* OUTPUT:       pbVal - pointer to the output parameter                 *       
*                                                                       *
* RETURN:       OK                                                      *
*               NOK - EEPROM write error                                *
*               INVALID_PARAMETER1 - invalid parameter 1                *  
*************************************************************************/
UINT32	whal_hwEepromGetAGCCell(HwEeprom_T *pHwEeprom, UINT8 bTableOffset, UINT8 *pbVal);

/************************************************************************
*   UINT32 whal_hwEepromSetAGCCell(UINT8 bTableOffset, UINT8 bVal)     *
*************************************************************************
* DESCRIPTION:  Writes one AGC table cell to EEPROM                     *
*                                                                       *
* INPUT:        bTableOffset - zero-based offset of the cell in AGC     *   
*                              table                                    *
*               bVal         - new cell value                           *
* OUTPUT:       N/A                                                     *
*                                                                       *
* RETURN:       OK                                                      *
*               NOK - EEPROM write error                                *
*               INVALID_PARAMETER1 - invalid parameter 1                *  
*************************************************************************/
UINT32	whal_hwEepromSetAGCCell(HwEeprom_T *pHwEeprom, UINT8 bTableOffset, UINT8 bVal);


void whal_hwEeprom_DumpEEPROM(HwEeprom_T *pHwEeprom);

#endif /* USE_SYNC_API */


#endif	/* _WHAL_HW_EEPROM_H */



