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
/*   MODULE:  whalHwEeprom.c                                              */ 
/*   PURPOSE: Wlan hardware EEPROM access routines implemenatation        */
/*                                                                        */
/**************************************************************************/
#include "whalCommon.h"
#include "whalHwDefs.h"
#include "whalHwEeprom.h"
#include "TNETWIF.h"


/****************************************************************************
 *                      whal_hwEeprom_Create()
 ****************************************************************************
 * DESCRIPTION:	Create the wlan hardware eeprom access object
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The Created object
 ****************************************************************************/
HwEeprom_T *whal_hwEeprom_Create(TI_HANDLE hOs)
{
	HwEeprom_T *pObj;

	pObj = os_memoryAlloc(hOs, sizeof(HwEeprom_T));
	if (pObj == NULL)
		return NULL;

	os_memoryZero(hOs, pObj, sizeof(HwEeprom_T));

	pObj->hOs = hOs;

	return(pObj);
}

/****************************************************************************
 *                      whal_hwEeprom_Destroy()
 ****************************************************************************
 * DESCRIPTION:	Destroy the object 
 * 
 * INPUTS:	
 *		pHwEeprom		The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwEeprom_Destroy(HwEeprom_T *pHwEeprom)
{
	if (pHwEeprom)
		os_memoryFree(pHwEeprom->hOs, pHwEeprom, sizeof(HwEeprom_T));
	return OK;
}

/****************************************************************************
 *                      whal_hwEeprom_Config()
 ****************************************************************************
 * DESCRIPTION:	Config the object 
 * 
 * INPUTS:	
 *		pHwEeprom		The object to free
 *		hTNETWIF		hardware access object
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwEeprom_Config(HwEeprom_T *pHwEeprom, TI_HANDLE hTNETWIF, TI_HANDLE hReport)
{
	pHwEeprom->hReport = hReport;
	pHwEeprom->hTNETWIF = hTNETWIF;
	return OK;
}

#ifdef USE_SYNC_API 
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
UINT32 whal_hwEepromReadByte(HwEeprom_T *pHwEeprom, UINT16 wAddr,UINT8 *pbVal)
{
	UINT32 data;   
	int i = 0x00;
   

    wAddr &= 0x07ff;
    TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_EE_ADDR_REG,(UINT32)wAddr);
    TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_EE_CTL_REG, EE_READ);
    while((TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,ACX_EE_CTL_REG,&data) & EE_READ) != 0x00)
    {
        if(i++ > EEPROM_ACCESS_TO)
        {
            return NOK; /* timeout */
        }
    }
    *pbVal = TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,ACX_EE_DATA_REG,&data) & 0xff;
    return OK;
}

/************************************************************************
*      UINT32 whal_hwEepromWriteByteNoUnp(UINT16 wAddr,UINT8 bVal)     *
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
UINT32 whal_hwEepromWriteByteNoUnp(HwEeprom_T *pHwEeprom, UINT16 wAddr,UINT8 bVal)
{
   volatile int i=0;
   UINT32 data;

   wAddr &= 0x07ff;
   TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_EE_ADDR_REG,(UINT32)wAddr);
   TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_EE_DATA_REG,bVal);
   TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_EE_CTL_REG, EE_WRITE);
   while((TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,ACX_EE_CTL_REG,&data)& EE_WRITE) != 0x00)
   {
		volatile int y=0;

   	for( ; y<100; y++) {}
      if(i++ > EEPROM_ACCESS_TO)
      {
          return NOK; /* timeout */
      }
   }

   return OK;
}

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
void whal_hwEepromProtect(HwEeprom_T *pHwEeprom)
{
   /* Set up write protect. Should be according to board type and SW patch
      rather than according to Hardware EEPROM
   */
	TNETWIF_RegIsBitSet(pHwEeprom->hTNETWIF, ACX_GPIO_OUT_REG, 1ul << 9);
}

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
void whal_hwEepromUnprotect(HwEeprom_T *pHwEeprom)
{
    /* Turn off write protect. Should be according to board type and SW patch
      rather than according to Hardware EEPROM
   */
   	TNETWIF_RegResetBitVal(pHwEeprom->hTNETWIF, ACX_GPIO_OUT_REG, 1ul << 9);
}

/************************************************************************
*          UINT32 whal_hwEepromWriteByte(UINT16 wAddr,UINT8 bVal)      *
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
UINT32 whal_hwEepromWriteByte(HwEeprom_T *pHwEeprom, UINT16 wAddr,UINT8 bVal)
{
	UINT32 retCode;
	
   whal_hwEepromUnprotect(pHwEeprom);
	os_StalluSec(pHwEeprom->hOs, 100000);
	retCode = whal_hwEepromWriteByteNoUnp(pHwEeprom, wAddr, bVal);
	os_StalluSec(pHwEeprom->hOs, 100000);
	whal_hwEepromProtect(pHwEeprom);
   return retCode;
}

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
UINT32 whal_hwEepromGetCalValue(HwEeprom_T *pHwEeprom, UINT8 *pbVal)
{
    return whal_hwEepromReadByte(pHwEeprom, HW_EEPROM_OSC_ADDR,pbVal);
}

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
UINT32 whal_hwEepromSetCalValue(HwEeprom_T *pHwEeprom, UINT8 bVal)
{
   if(bVal > MAX_OSC_CAL)
   {
      return NOK;       
   }
   
   return whal_hwEepromWriteByte(pHwEeprom, HW_EEPROM_OSC_ADDR,bVal);
}

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
UINT32 whal_hwEepromGetBiasValue(HwEeprom_T *pHwEeprom, UINT8 *pbVal)
{
    return whal_hwEepromReadByte(pHwEeprom, HW_EEPROM_BIAS_ADDR,pbVal);
}

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
UINT32 whal_hwEepromSetBiasValue(HwEeprom_T *pHwEeprom, UINT8 bVal)
{
    return whal_hwEepromWriteByte(pHwEeprom, HW_EEPROM_BIAS_ADDR,bVal);
}

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
UINT32 whal_hwEepromGetDACValue(HwEeprom_T *pHwEeprom, UINT8 *pbVal)
{
    return whal_hwEepromReadByte(pHwEeprom, HW_EEPROM_DAC_ADDR,pbVal);
}

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
UINT32 whal_hwEepromSetDACValue(HwEeprom_T *pHwEeprom, UINT8 bVal)
{
    return whal_hwEepromWriteByte(pHwEeprom, HW_EEPROM_DAC_ADDR,bVal);
}

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
int whal_hwEepromLoadBaseBandTable(HwEeprom_T *pHwEeprom)
{
   UINT8   bRegVal;
   UINT16  wTableAddress;
   UINT8   bNumberOfEntries;  
   UINT16  wTableEnd;
   UINT8   bEntrySize; 
   UINT8   bbAddr;
   UINT8   bbData;
   int addr;

   if(whal_hwEepromReadByte(pHwEeprom, 0x14a,&bRegVal) != OK)
   {
		return NOK;
   }
   
   wTableAddress = bRegVal;
   
   if(whal_hwEepromReadByte(pHwEeprom, 0x14b,&bRegVal) != OK)
   {
		return NOK;
   }
   
   wTableAddress |= bRegVal << 8;

   if(whal_hwEepromReadByte(pHwEeprom, wTableAddress,&bRegVal) != OK)
   {
   }
   
   bNumberOfEntries = bRegVal;

   if(whal_hwEepromReadByte(pHwEeprom, (UINT16)(wTableAddress+1),&bRegVal) != OK)
   {
      return NOK;
   }

   bEntrySize = bRegVal;
   
	WLAN_REPORT_INFORMATION(pHwEeprom->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("\tTable Address: 0x%x\n\tNumber of elements: 0x%x\n\tEntry Size: 0x%x\n",
                        wTableAddress,bNumberOfEntries,bEntrySize));


   wTableEnd = wTableAddress + (bEntrySize * (bNumberOfEntries + 1));
   
   for(addr = wTableAddress + 2; addr < wTableEnd; addr+= bEntrySize)
   {
      if(whal_hwEepromReadByte(pHwEeprom, (UINT16)addr,&bbAddr)!= OK)
      {
			return NOK;         
      }                        
   
      if(whal_hwEepromReadByte(pHwEeprom, (UINT16)(addr+1),&bbData)!= OK)
      {
         return NOK;         
      }

      /*
      whal_hwWritePHYReg(bbAddr,bbData);
      -- the follwing 3 statements do the same thing
      */
	   TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_PHY_ADDR_REG, bbAddr);
	   TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_PHY_DATA_REG, bbData);
	   TNETWIF_WriteRegSync(pHwEeprom->hTNETWIF, ACX_PHY_CTRL_REG, 1 /* write */);
   }
   return OK;
}/* END whal_hwEepromLoadBaseBandTable() */

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
UINT32 whal_hwEepromGetAGCCell(HwEeprom_T *pHwEeprom, UINT8 bTableOffset, UINT8 *pbVal)
{
    if(bTableOffset > MAX_AGC_TABLE_ENTRIES)
    {
        *pbVal = 0x00;
        return (UINT32)INVALID_PARAMETER1;
    }

    return whal_hwEepromReadByte(pHwEeprom, (UINT16)(HW_EEPROM_AGC_TABLE_ADDR+bTableOffset),pbVal);
}
 
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
UINT32 whal_hwEepromSetAGCCell(HwEeprom_T *pHwEeprom, UINT8 bTableOffset, UINT8 bVal)
{
    if(bTableOffset > MAX_AGC_TABLE_ENTRIES)
    {
        return (UINT32)INVALID_PARAMETER1;
    }

    return whal_hwEepromWriteByte(pHwEeprom, (UINT16)(HW_EEPROM_AGC_TABLE_ADDR+bTableOffset), bVal);
} 



/****************************************************************************
 *                      whal_hwCtrl_GetRadioTypeAndEEPROMversion()
 ****************************************************************************
 * DESCRIPTION:	
 * 
 * INPUTS:  None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwEeprom_GetRadioTypeAndEEPROMversion(HwEeprom_T *pHwEeprom, UINT32 *major, UINT32 *minor, UINT32 *bugfix)
{
	/*volatile */
UINT32    radioType;

	/*volatile */
UINT32 eectl;
	/* 
	 * Read major eeprom version - offset 5
	 */
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_ADDR, 5);
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_CTL,  0x2);
	do 
	{
		TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_CTL,&eectl);
		if (eectl == 0xffffffff)
			return -1;
	} while (eectl &0x2); 
	TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_DATA,major);
	/* 
	 * Read minor eeprom version - offset 9 
	 */
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_ADDR, 9);
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_CTL,  0x2);
	do 
	{
		TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_CTL,&eectl);
		if (eectl == 0xffffffff)
			return -1;
	} while (eectl &0x2); 
	TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_DATA,minor);

	/* 
	 * Read bugfix eeprom version - offset A 
	 */
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_ADDR, 0xA);
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_CTL,  0x2);
	do 
	{
		TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_CTL,&eectl);
		if (eectl == 0xffffffff)
			return -1;
	} while (eectl &0x2); 
	TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_DATA,bugfix);
	
	/* 
	 * Read radio type - offset 4 
	 */
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_ADDR, 4);
	TNETWIF_WriteRegSync( pHwEeprom->hTNETWIF, EE_CTL,  0x2);
	do 
	{
		TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_CTL,&eectl);
		if (eectl == 0xffffffff)
			return -1;
	} while (eectl &0x2); 

	TNETWIF_ReadRegSync(pHwEeprom->hTNETWIF,EE_DATA,&radioType);

	return(int)radioType;
}

void whal_hwEeprom_DumpEEPROM(HwEeprom_T *pHwEeprom)
{
	UINT16 wAddr;
	UINT8  bVal;

	WLAN_REPORT_REPLY(pHwEeprom->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("Dump EEPROM contents:"));
	for ( wAddr = 0; wAddr < 0x2ff; wAddr++)
	{
		if (whal_hwEepromReadByte(pHwEeprom, wAddr, &bVal) == OK)
		{
			WLAN_REPORT_REPLY(pHwEeprom->hReport, HAL_HW_CTRL_MODULE_LOG,  
							  ("\tEEPROM 0x%04X:\t0x%02X.\n", wAddr, bVal));
		} else
		{
			WLAN_REPORT_REPLY(pHwEeprom->hReport, HAL_HW_CTRL_MODULE_LOG,  
							  ("\tERROR: timeout"));
		}
	}
}

#endif /* USE_SYNC_API */
