/** \file utils.c
 *  \brief General utilities implementation
 *
 *  \see utils.h
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
/*																			*/
/*		MODULE:	utils.c														*/
/*    PURPOSE:	General utilities implementation 							*/
/***************************************************************************/
#include "osTIType.h"
#include "osApi.h"
#include "report.h"
#include "commonTypes.h"
#include "utils.h"
#include "802_11Defs.h"

#define NUM_OF_NIBBLES_IN_BYTE (2)
#define NIBBLE_SIZE_IN_BITS (4)
#define NIBBLE_MASK (0xF)

/************************************************************************
 *                        utils_nullMemoryFree							*
 ************************************************************************
DESCRIPTION: Called in order to free memory space. 
			Calls the OS free function only if the memory is occupied
                                                                                                   
INPUT:      pOsContext		-	Handle to OS
			pMemPtr			-	Pointer to the memory to be free	
			size			-	Size of memory to be free	

OUTPUT:		


RETURN:     

************************************************************************/
void utils_nullMemoryFree(void* pOsContext, 
						  void* pMemPtr, 
						  unsigned long size)
{
	if (pMemPtr != NULL)
		os_memoryFree(pOsContext, pMemPtr, size);
}

/************************************************************************
 *                        utils_nullTimerDestroy							*
 ************************************************************************
DESCRIPTION: Called in order to free OS timers. 
			Calls the OS timer destroy function only if the timer is not NULL
                                                                                                   
INPUT:      pOsContext		-	Handle to OS
			pTimerHandle	-	Pointer to timer handle	

OUTPUT:		


RETURN:     

************************************************************************/
void utils_nullTimerDestroy(void* pOsContext, 
						 void* pTimerHandle)
{
	os_timerStop(pOsContext,pTimerHandle);
	os_timerDestroy(pOsContext, pTimerHandle);
}

/************************************************************************
 *                        networkToHostRate							*
 ************************************************************************
DESCRIPTION: Translates a network rate (0x02, 0x82, 0x84, etc...) to host rate (1, 2, 3, ....)
                                                                                                   
INPUT:      rate		-	Network rate

OUTPUT:		


RETURN:     Host rate if the input rate is valid, otherwise returns 0.

************************************************************************/

#define NET_BASIC_MASK		0x80
#define NET_RATE_1M			(0x02)
#define NET_RATE_2M			(0x04)
#define NET_RATE_5_5M		(0x0B)
#define NET_RATE_11M		(0x16)
#define NET_RATE_22M		(0x2C)
#define NET_RATE_6M			(0x0C)
#define NET_RATE_9M			(0x12)
#define NET_RATE_12M		(0x18)
#define NET_RATE_18M		(0x24)
#define NET_RATE_24M		(0x30)
#define NET_RATE_36M		(0x48)
#define NET_RATE_48M		(0x60)
#define NET_RATE_54M		(0x6C)

/*#define ONLY_802_11_A*/
/*#ifdef ONLY_802_11_A
#undef NET_RATE_1M
#undef NET_RATE_2M
#undef NET_RATE_5_5M
#undef NET_RATE_11M
#undef NET_RATE_22M
#define NET_RATE_1M			(NET_RATE_6M)
#define NET_RATE_2M			(NET_RATE_12M)
#define NET_RATE_5_5M		(NET_RATE_24M)
#define NET_RATE_11M		(NET_RATE_54M)
#define NET_RATE_22M		(NET_RATE_48M)
#endif
*/

#define NET_RATE_1M_BASIC	(NET_RATE_1M   | NET_BASIC_MASK)
#define NET_RATE_2M_BASIC	(NET_RATE_2M   | NET_BASIC_MASK)
#define NET_RATE_5_5M_BASIC	(NET_RATE_5_5M | NET_BASIC_MASK)
#define NET_RATE_11M_BASIC	(NET_RATE_11M  | NET_BASIC_MASK)
#define NET_RATE_22M_BASIC	(NET_RATE_22M  | NET_BASIC_MASK)
#define NET_RATE_6M_BASIC	(NET_RATE_6M   | NET_BASIC_MASK)
#define NET_RATE_9M_BASIC	(NET_RATE_9M   | NET_BASIC_MASK)
#define NET_RATE_12M_BASIC	(NET_RATE_12M  | NET_BASIC_MASK)
#define NET_RATE_18M_BASIC	(NET_RATE_18M  | NET_BASIC_MASK)
#define NET_RATE_24M_BASIC	(NET_RATE_24M  | NET_BASIC_MASK)
#define NET_RATE_36M_BASIC	(NET_RATE_36M  | NET_BASIC_MASK)
#define NET_RATE_48M_BASIC	(NET_RATE_48M  | NET_BASIC_MASK)
#define NET_RATE_54M_BASIC	(NET_RATE_54M  | NET_BASIC_MASK)


#define FIRST_VALID_CHAR        32


rate_e networkToHostRate(UINT8 rate)
{
	switch (rate)
	{
	case NET_RATE_1M:
	case NET_RATE_1M_BASIC:
		return DRV_RATE_1M;

	case NET_RATE_2M:
	case NET_RATE_2M_BASIC:
		return DRV_RATE_2M;

	case NET_RATE_5_5M:
	case NET_RATE_5_5M_BASIC:
		return DRV_RATE_5_5M;

	case NET_RATE_11M:
	case NET_RATE_11M_BASIC:
		return DRV_RATE_11M;

	case NET_RATE_22M:
	case NET_RATE_22M_BASIC:
		return DRV_RATE_22M;

	case NET_RATE_6M:
	case NET_RATE_6M_BASIC:
		return DRV_RATE_6M;

	case NET_RATE_9M:
	case NET_RATE_9M_BASIC:
		return DRV_RATE_9M;

	case NET_RATE_12M:
	case NET_RATE_12M_BASIC:
		return DRV_RATE_12M;

	case NET_RATE_18M:
	case NET_RATE_18M_BASIC:
		return DRV_RATE_18M;

	case NET_RATE_24M:
	case NET_RATE_24M_BASIC:
		return DRV_RATE_24M;

	case NET_RATE_36M:
	case NET_RATE_36M_BASIC:
		return DRV_RATE_36M;

	case NET_RATE_48M:
	case NET_RATE_48M_BASIC:
		return DRV_RATE_48M;

	case NET_RATE_54M:
	case NET_RATE_54M_BASIC:
		return DRV_RATE_54M;

	default:
		return DRV_RATE_INVALID;
	}
}

/************************************************************************
 *                        hostToNetworkRate							*
 ************************************************************************
DESCRIPTION: Translates a host rate (1, 2, 3, ....) to network rate (0x02, 0x82, 0x84, etc...) 
                                                                                                   
INPUT:      rate		-	Host rate

OUTPUT:		


RETURN:     Network rate if the input rate is valid, otherwise returns 0.

************************************************************************/
UINT8 hostToNetworkRate(rate_e rate)
{
	switch (rate)
	{
	case DRV_RATE_1M:
		return NET_RATE_1M;

	case DRV_RATE_2M:
		return NET_RATE_2M;

	case DRV_RATE_5_5M:
		return NET_RATE_5_5M;

	case DRV_RATE_11M:
		return NET_RATE_11M;

	case DRV_RATE_22M:
		return NET_RATE_22M;

	case DRV_RATE_6M:
		return NET_RATE_6M;

	case DRV_RATE_9M:
		return NET_RATE_9M;

	case DRV_RATE_12M:
		return NET_RATE_12M;

	case DRV_RATE_18M:
		return NET_RATE_18M;

	case DRV_RATE_24M:
		return NET_RATE_24M;

	case DRV_RATE_36M:
		return NET_RATE_36M;

	case DRV_RATE_48M:
		return NET_RATE_48M;

	case DRV_RATE_54M:
		return NET_RATE_54M;

	default:
		return 0;
	}
}


/***************************************************************************
*					getMaxActiveRatefromBitmap				               *
****************************************************************************
* DESCRIPTION:	
*
* INPUTS:		hCtrlData - the object
*				
* OUTPUT:		
*
* RETURNS:		
***************************************************************************/
rate_e getMaxRatefromBitmap(UINT32		ratesBitMap)
{
	rate_e rate = DRV_RATE_1M;

	if(ratesBitMap & DRV_RATE_MASK_1_BARKER) rate = DRV_RATE_1M; 
	if(ratesBitMap & DRV_RATE_MASK_2_BARKER) rate = DRV_RATE_2M;
	if(ratesBitMap & DRV_RATE_MASK_5_5_CCK)  rate = DRV_RATE_5_5M;
	if(ratesBitMap & DRV_RATE_MASK_11_CCK)   rate = DRV_RATE_11M; 
	if(ratesBitMap & DRV_RATE_MASK_22_PBCC)  rate = DRV_RATE_22M;
	if(ratesBitMap & DRV_RATE_MASK_6_OFDM)   rate = DRV_RATE_6M;
	if(ratesBitMap & DRV_RATE_MASK_9_OFDM)   rate = DRV_RATE_9M;
	if(ratesBitMap & DRV_RATE_MASK_12_OFDM)  rate = DRV_RATE_12M;
	if(ratesBitMap & DRV_RATE_MASK_18_OFDM)  rate = DRV_RATE_18M;
	if(ratesBitMap & DRV_RATE_MASK_24_OFDM)  rate = DRV_RATE_24M;
	if(ratesBitMap & DRV_RATE_MASK_36_OFDM)  rate = DRV_RATE_36M;
	if(ratesBitMap & DRV_RATE_MASK_48_OFDM)  rate = DRV_RATE_48M;
	if(ratesBitMap & DRV_RATE_MASK_54_OFDM)  rate = DRV_RATE_54M;

	return rate;
}


/************************************************************************
 *                        getMaxBasicRate							*
 ************************************************************************
DESCRIPTION: Goes over an array of network rates and returns the max basic rate
                                                                                                   
INPUT:      pRates		-	Rate array

OUTPUT:		


RETURN:     Max basic rate (in network units)

************************************************************************/
UINT8 getMaxBasicRatefromString(UINT8 *ratesString, UINT8 len, UINT8 maxRate)
{
	UINT8 i;
	
	for (i = 0; i < len; i++)
	{
		if ((IS_BASIC_RATE(ratesString[i])) &&
			(validateNetworkRate(ratesString[i]) == OK))
		{
			maxRate = MAX(ratesString[i], maxRate);
	}
	}

	return maxRate;
}

/************************************************************************
 *                        getMaxActiveRate							*
 ************************************************************************
DESCRIPTION: Goes over an array of network rates and returns the max active rate
                                                                                                   
INPUT:      pRates		-	Rate array

OUTPUT:		


RETURN:     Max active rate (in network units)

************************************************************************/
UINT8 getMaxActiveRatefromString(UINT8 *ratesString, UINT8 len, UINT8 maxRate)
{
	UINT8 i;
	
	for (i = 0; i < len; i++)
	{
		if ((IS_ACTIVE_RATE(ratesString[i])) &&
			(validateNetworkRate(ratesString[i]) == OK))
		{
			maxRate = MAX(ratesString[i], maxRate);
		}
	}

	return maxRate;
}

/************************************************************************
 *                        validateNetworkRate							*
 ************************************************************************
DESCRIPTION: Verify that the input nitwork rate is valid
                                                                                                   
INPUT:      rate	-	input network rate

OUTPUT:		


RETURN:     OK if valid, otherwise NOK

************************************************************************/
TI_STATUS validateNetworkRate(UINT8 rate)
{
	switch (rate)
	{
	case NET_RATE_1M:
	case NET_RATE_1M_BASIC:
	case NET_RATE_2M:
	case NET_RATE_2M_BASIC:
	case NET_RATE_5_5M:
	case NET_RATE_5_5M_BASIC:
	case NET_RATE_11M:
	case NET_RATE_11M_BASIC:
	case NET_RATE_22M:
	case NET_RATE_22M_BASIC:
	case NET_RATE_6M:
	case NET_RATE_6M_BASIC:
	case NET_RATE_9M:
	case NET_RATE_9M_BASIC:
	case NET_RATE_12M:
	case NET_RATE_12M_BASIC:
	case NET_RATE_18M:
	case NET_RATE_18M_BASIC:
	case NET_RATE_24M:
	case NET_RATE_24M_BASIC:
	case NET_RATE_36M:
	case NET_RATE_36M_BASIC:
	case NET_RATE_48M:
	case NET_RATE_48M_BASIC:
	case NET_RATE_54M:
	case NET_RATE_54M_BASIC:
		return OK;

	default:
		return NOK;
	}
}

/************************************************************************
 *                        hostToUtilityRate							*
 ************************************************************************
DESCRIPTION: Translates a host rate (1, 2, 3, ....) to utility rate (2, 4, 11, 22, ....) 
                                                                                                   
INPUT:      rate		-	Host rate

OUTPUT:		


RETURN:     Utility rate if the input rate is valid, otherwise returns 0.

************************************************************************/
UINT8 hostToUtilityRate(rate_e rate)
{
	switch (rate)
	{
	case DRV_RATE_AUTO:
		return 0;

	case DRV_RATE_1M:
		return NET_RATE_1M;

	case DRV_RATE_2M:
		return NET_RATE_2M;

	case DRV_RATE_5_5M:
		return NET_RATE_5_5M;

	case DRV_RATE_11M:
		return NET_RATE_11M;

	case DRV_RATE_22M:
		return NET_RATE_22M;

	case DRV_RATE_6M:
		return NET_RATE_6M;

	case DRV_RATE_9M:
		return NET_RATE_9M;

	case DRV_RATE_12M:
		return NET_RATE_12M;

	case DRV_RATE_18M:
		return NET_RATE_18M;

	case DRV_RATE_24M:
		return NET_RATE_24M;

	case DRV_RATE_36M:
		return NET_RATE_36M;

	case DRV_RATE_48M:
		return NET_RATE_48M;

	case DRV_RATE_54M:
		return NET_RATE_54M;

	default:
		return 0;
	}
}

/************************************************************************
 *                        utilityToHostRate							*
 ************************************************************************
DESCRIPTION: Translates a utility rate (2, 4, 11, 22, ....) to host rate (1, 2, 3, ....) to
                                                                                                   
INPUT:      rate		-	Utility rate

OUTPUT:		


RETURN:     Host rate if the input rate is valid, otherwise returns 0.

************************************************************************/
rate_e utilityToHostRate(UINT8 rate)
{
	switch (rate)
	{
	case 0:
		return DRV_RATE_AUTO;

	case NET_RATE_1M:
	case NET_RATE_1M_BASIC:
		return DRV_RATE_1M;

	case NET_RATE_2M:
	case NET_RATE_2M_BASIC:
		return DRV_RATE_2M;

	case NET_RATE_5_5M:
	case NET_RATE_5_5M_BASIC:
		return DRV_RATE_5_5M;

	case NET_RATE_11M:
	case NET_RATE_11M_BASIC:
		return DRV_RATE_11M;

	case NET_RATE_22M:
	case NET_RATE_22M_BASIC:
		return DRV_RATE_22M;

	case NET_RATE_6M:
	case NET_RATE_6M_BASIC:
		return DRV_RATE_6M;

	case NET_RATE_9M:
	case NET_RATE_9M_BASIC:
		return DRV_RATE_9M;

	case NET_RATE_12M:
	case NET_RATE_12M_BASIC:
		return DRV_RATE_12M;

	case NET_RATE_18M:
	case NET_RATE_18M_BASIC:
		return DRV_RATE_18M;

	case NET_RATE_24M:
	case NET_RATE_24M_BASIC:
		return DRV_RATE_24M;

	case NET_RATE_36M:
	case NET_RATE_36M_BASIC:
		return DRV_RATE_36M;

	case NET_RATE_48M:
	case NET_RATE_48M_BASIC:
		return DRV_RATE_48M;

	case NET_RATE_54M:
	case NET_RATE_54M_BASIC:
		return DRV_RATE_54M;

	default:
		return DRV_RATE_AUTO;
	}
}


/************************************************************************
 *                        bitMapToNetworkStringRates					*
 ************************************************************************
DESCRIPTION: Converts bit map to the rates string
                                                                                                   
INPUT:      suppRatesBitMap		-	bit map of supported rates
			basicRatesBitMap	-   bit map of basic rates

OUTPUT:		string - network format rates array,
			len - rates array length
			firstOFDMrateLoc - the index of first OFDM rate in the rates array.


RETURN:     None

************************************************************************/
void bitMapToNetworkStringRates(UINT32 suppRatesBitMap, UINT32 basicRatesBitMap,
								UINT8 *string, UINT32 *len,
								UINT32 *firstOFDMrateLoc)
{

	UINT32 i = 0;

	if(suppRatesBitMap & DRV_RATE_MASK_1_BARKER)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_1_BARKER)
			string[i++] = NET_RATE_1M_BASIC;
		else 
			string[i++] = NET_RATE_1M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_2_BARKER)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_2_BARKER)
			string[i++] = NET_RATE_2M_BASIC;
		else
			string[i++] = NET_RATE_2M;
	}	
	if(suppRatesBitMap & DRV_RATE_MASK_5_5_CCK)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_5_5_CCK)
			string[i++] = NET_RATE_5_5M_BASIC;
		else
			string[i++] = NET_RATE_5_5M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_11_CCK)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_11_CCK)
			string[i++] = NET_RATE_11M_BASIC;
		else
			string[i++] = NET_RATE_11M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_22_PBCC)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_22_PBCC)
			string[i++] = NET_RATE_22M_BASIC;
		else
			string[i++] = NET_RATE_22M;
	}

	*firstOFDMrateLoc = i;
	
	if(suppRatesBitMap & DRV_RATE_MASK_6_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_6_OFDM)
			string[i++] = NET_RATE_6M_BASIC;
		else
			string[i++] = NET_RATE_6M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_9_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_9_OFDM)
			string[i++] = NET_RATE_9M_BASIC;
		else
			string[i++] = NET_RATE_9M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_12_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_12_OFDM)
			string[i++] = NET_RATE_12M_BASIC;
		else
			string[i++] = NET_RATE_12M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_18_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_18_OFDM)
			string[i++] = NET_RATE_18M_BASIC;
		else
			string[i++] = NET_RATE_18M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_24_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_24_OFDM)
			string[i++] = NET_RATE_24M_BASIC;
		else
			string[i++] = NET_RATE_24M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_36_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_36_OFDM)
			string[i++] = NET_RATE_36M_BASIC;
		else
			string[i++] = NET_RATE_36M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_48_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_48_OFDM)
			string[i++] = NET_RATE_48M_BASIC;
		else
			string[i++] = NET_RATE_48M;
	}
	if(suppRatesBitMap & DRV_RATE_MASK_54_OFDM)
	{
		if(basicRatesBitMap & DRV_RATE_MASK_54_OFDM)
			string[i++] = NET_RATE_54M_BASIC;
		else
			string[i++] = NET_RATE_54M;
	}

	*len = i;
}

/************************************************************************
 *                        networkStringToBitMapSuppRates				*
 ************************************************************************
DESCRIPTION: Converts supported rates string to the bit map
                                                                                                   
INPUT:      string		-	array of rates in the network format
			len - array length

OUTPUT:		bitMap - bit map of rates.

RETURN:     None

************************************************************************/
void networkStringToBitMapSuppRates(UINT32 *bitMap, UINT8 *string, UINT32 len)
{
	UINT32 i;

	*bitMap = 0;

	for(i=0; i<len; i++)
	{
		switch(string[i])
		{
			case NET_RATE_1M:
			case NET_RATE_1M_BASIC:
				*bitMap |= DRV_RATE_MASK_1_BARKER;
				break;
			case NET_RATE_2M:
			case NET_RATE_2M_BASIC:
				*bitMap |= DRV_RATE_MASK_2_BARKER;
				break;
			case NET_RATE_5_5M:
			case NET_RATE_5_5M_BASIC:
				*bitMap |= DRV_RATE_MASK_5_5_CCK;
				break;
			case NET_RATE_11M:
			case NET_RATE_11M_BASIC:
				*bitMap |= DRV_RATE_MASK_11_CCK;
				break;
			case NET_RATE_22M:
			case NET_RATE_22M_BASIC:
				*bitMap |= DRV_RATE_MASK_22_PBCC;
				break;
			case NET_RATE_6M:
			case NET_RATE_6M_BASIC:
				*bitMap |= DRV_RATE_MASK_6_OFDM;
				break;
			case NET_RATE_9M:
			case NET_RATE_9M_BASIC:
				*bitMap |= DRV_RATE_MASK_9_OFDM;
				break;
			case NET_RATE_12M:
			case NET_RATE_12M_BASIC:
				*bitMap |= DRV_RATE_MASK_12_OFDM;
				break;
			case NET_RATE_18M:
			case NET_RATE_18M_BASIC:
				*bitMap |= DRV_RATE_MASK_18_OFDM;
				break;
			case NET_RATE_24M:
			case NET_RATE_24M_BASIC:
				*bitMap |= DRV_RATE_MASK_24_OFDM;
				break;
			case NET_RATE_36M:
			case NET_RATE_36M_BASIC:
				*bitMap |= DRV_RATE_MASK_36_OFDM;
				break;
			case NET_RATE_48M:
			case NET_RATE_48M_BASIC:
				*bitMap |= DRV_RATE_MASK_48_OFDM;
				break;
			case NET_RATE_54M:
			case NET_RATE_54M_BASIC:
				*bitMap |= DRV_RATE_MASK_54_OFDM;
				break;
			default:
				break;
		}
	}
}	  

/************************************************************************
 *                        networkStringToBitMapBasicRates				*
 ************************************************************************
DESCRIPTION: Converts basic rates string to the bit map
                                                                                                   
INPUT:      string		-	array of rates in the network format
			len - array length

OUTPUT:		bitMap - bit map of rates.

RETURN:     None

************************************************************************/
void networkStringToBitMapBasicRates(UINT32 *bitMap, UINT8 *string, UINT32 len)
{
	UINT32 i;

	*bitMap = 0;

	for(i=0; i<len; i++)
	{
		switch(string[i])
		{
			case NET_RATE_1M_BASIC:	
				*bitMap |= DRV_RATE_MASK_1_BARKER;
				break;
			case NET_RATE_2M_BASIC:
				*bitMap |= DRV_RATE_MASK_2_BARKER;
				break;
			case NET_RATE_5_5M_BASIC:
				*bitMap |= DRV_RATE_MASK_5_5_CCK;
				break;
			case NET_RATE_11M_BASIC:
				*bitMap |= DRV_RATE_MASK_11_CCK;
				break;
			case NET_RATE_22M_BASIC:
				*bitMap |= DRV_RATE_MASK_22_PBCC;
				break;
			case NET_RATE_6M_BASIC:
				*bitMap |= DRV_RATE_MASK_6_OFDM;
				break;
			case NET_RATE_9M_BASIC:
				*bitMap |= DRV_RATE_MASK_9_OFDM;
				break;
			case NET_RATE_12M_BASIC:
				*bitMap |= DRV_RATE_MASK_12_OFDM;
				break;
			case NET_RATE_18M_BASIC:
				*bitMap |= DRV_RATE_MASK_18_OFDM;
				break;
			case NET_RATE_24M_BASIC:
				*bitMap |= DRV_RATE_MASK_24_OFDM;
				break;
			case NET_RATE_36M_BASIC:
				*bitMap |= DRV_RATE_MASK_36_OFDM;
				break;
			case NET_RATE_48M_BASIC:
				*bitMap |= DRV_RATE_MASK_48_OFDM;
				break;
			case NET_RATE_54M_BASIC:
				*bitMap |= DRV_RATE_MASK_54_OFDM;
				break;
			default:
				break;
		}
	}
}	  

void validateRates(UINT32 *pBasicRateMask, UINT32 *pSuppRateMask, 
						  UINT32 *pTxRate, modulationType_e *modulation, BOOL dot11a)
{
	rate_e maxSuppRate;

	/* Make sure that the basic rate set is included in the supported rate set */
	(*pBasicRateMask) &= (*pSuppRateMask);

	/* Ignore modulation in the Tx rate. */
	switch (*pTxRate)
	{
	case REG_RATE_AUTO_BIT:
		*pTxRate = DRV_RATE_AUTO;
		break;

	case REG_RATE_1M_BIT:
		*pTxRate = DRV_RATE_1M;
		break;

	case REG_RATE_2M_BIT:
		*pTxRate = DRV_RATE_2M;
		break;

	case REG_RATE_5_5M_CCK_BIT:
		*pTxRate = DRV_RATE_5_5M;
		break;

	case REG_RATE_11M_CCK_BIT:
		*pTxRate = DRV_RATE_11M;
		break;

	case REG_RATE_22M_PBCC_BIT:
		*pTxRate = DRV_RATE_22M;
		break;

	case REG_RATE_6M_OFDM_BIT:
		*pTxRate = DRV_RATE_6M;
		break;
	case REG_RATE_9M_OFDM_BIT:
		*pTxRate = DRV_RATE_9M;
		break;
	case REG_RATE_12M_OFDM_BIT:
		*pTxRate = DRV_RATE_12M;
		break;
	case REG_RATE_18M_OFDM_BIT:
		*pTxRate = DRV_RATE_18M;
		break;
	case REG_RATE_24M_OFDM_BIT:
		*pTxRate = DRV_RATE_24M;
		break;
	case REG_RATE_36M_OFDM_BIT:
		*pTxRate = DRV_RATE_36M;
		break;
	case REG_RATE_48M_OFDM_BIT:
		*pTxRate = DRV_RATE_48M;
		break;
	case REG_RATE_54M_OFDM_BIT:
		*pTxRate = DRV_RATE_54M;
		break;
	default:
		*pTxRate = DRV_RATE_AUTO;
		break;
	}

	/* Make sure that in dot11a mode the desired tx rate is OFDM rate */
	if(dot11a)
		if((*pTxRate < DRV_RATE_6M) && (*pTxRate != DRV_RATE_AUTO))
			*pTxRate = DRV_RATE_6M;

	/* Make sure that the Tx rate is less or equsl to the max supported rate */
	maxSuppRate = calculateMaxSupportedRate(pSuppRateMask);
	if(maxSuppRate == DRV_RATE_INVALID)
	{
		if(dot11a)
			*pTxRate = DRV_RATE_6M;
		else
			*pTxRate = DRV_RATE_1M;
	}
	else if(*pTxRate > (UINT32)maxSuppRate)
		*pTxRate = (UINT32)maxSuppRate;

	/* Handle desired modulation */
	if(maxSuppRate == DRV_RATE_22M)
		*modulation = DRV_MODULATION_PBCC;
	else if(maxSuppRate < DRV_RATE_22M)
		*modulation = DRV_MODULATION_CCK;
	else
		*modulation = DRV_MODULATION_OFDM;
}

int ConvertHwBitRateToAppRate(UINT32 HwRate,rate_e *AppRate)
{
	rate_e Rate = DRV_RATE_AUTO;
	int Stt = OK;

	switch (HwRate)
	{
		/*
		 *	The handle for 5.5/11/22 PBCC was removed !!!
		 */

	case HW_BIT_RATE_1MBPS:    						 Rate = DRV_RATE_1M;   		break;
	case HW_BIT_RATE_2MBPS:    						 Rate = DRV_RATE_2M;		break;
	case HW_BIT_RATE_5_5MBPS:   					 Rate = DRV_RATE_5_5M;		break;
	case HW_BIT_RATE_6MBPS:    						 Rate = DRV_RATE_6M;		break;
	case HW_BIT_RATE_9MBPS:    						 Rate = DRV_RATE_9M;    	break;
	case HW_BIT_RATE_11MBPS:   						 Rate = DRV_RATE_11M;		break;
	case HW_BIT_RATE_12MBPS:   						 Rate = DRV_RATE_12M;		break;
	case HW_BIT_RATE_18MBPS:   						 Rate = DRV_RATE_18M;		break;
	case HW_BIT_RATE_22MBPS:   						 Rate = DRV_RATE_22M;		break;
	case HW_BIT_RATE_24MBPS:   						 Rate = DRV_RATE_24M;		break;
	case HW_BIT_RATE_36MBPS:   						 Rate = DRV_RATE_36M;		break;
	case HW_BIT_RATE_48MBPS:   						 Rate = DRV_RATE_48M;		break;
	case HW_BIT_RATE_54MBPS:   						 Rate = DRV_RATE_54M;		break;
	default:
			Stt = NOK;
			break;
	}

	if (Stt == OK)
		*AppRate = Rate;
	else
		*AppRate = DRV_RATE_1M; 

	return (Stt);
}
int ConvertAppRateToHwBitMapRate(UINT32 AppRate, UINT32 *HwRate)
{
	UINT32 Rate = 0;
	int Stt = OK;

	switch (AppRate)
	{
		/* when rateAdaptaion in FW */
		case DRV_RATE_AUTO:                          Rate = 0; break;

		case DRV_RATE_1M:							 Rate = HW_BIT_RATE_1MBPS;     break;
		case DRV_RATE_2M:							 Rate = HW_BIT_RATE_2MBPS;     break;
		case DRV_RATE_5_5M:   						 Rate = HW_BIT_RATE_5_5MBPS;   break;
		case DRV_RATE_11M:							 Rate = HW_BIT_RATE_11MBPS;    break;
		case DRV_RATE_22M:							 Rate = HW_BIT_RATE_22MBPS;    break;
		case DRV_RATE_6M:							 Rate = HW_BIT_RATE_6MBPS;     break;
		case DRV_RATE_9M:							 Rate = HW_BIT_RATE_9MBPS;     break;
		case DRV_RATE_12M:							 Rate = HW_BIT_RATE_12MBPS;    break;
		case DRV_RATE_18M:							 Rate = HW_BIT_RATE_18MBPS;    break;
		case DRV_RATE_24M:							 Rate = HW_BIT_RATE_24MBPS;    break;
		case DRV_RATE_36M:							 Rate = HW_BIT_RATE_36MBPS;    break;
		case DRV_RATE_48M:							 Rate = HW_BIT_RATE_48MBPS;    break;
		case DRV_RATE_54M:							 Rate = HW_BIT_RATE_54MBPS;    break;
		default:
			Stt = NOK;
			break;
	}

	if (Stt == OK)
		*HwRate = Rate;
	else
		*HwRate = HW_BIT_RATE_1MBPS; 



	return (Stt);
}

int ConvertAppRatesToBitmap(UINT16 AppRatesBitmap, UINT32 *HwRatesBitmap)
{
	UINT16 RatesBitmap = 0;
   
	if (AppRatesBitmap & DRV_RATE_MASK_1_BARKER)	RatesBitmap |= HW_BIT_RATE_1MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_2_BARKER)	RatesBitmap |= HW_BIT_RATE_2MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_5_5_CCK)		RatesBitmap |= HW_BIT_RATE_5_5MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_11_CCK)		RatesBitmap |= HW_BIT_RATE_11MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_22_PBCC)		RatesBitmap |= HW_BIT_RATE_22MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_6_OFDM)		RatesBitmap |= HW_BIT_RATE_6MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_9_OFDM)		RatesBitmap |= HW_BIT_RATE_9MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_12_OFDM)		RatesBitmap |= HW_BIT_RATE_12MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_18_OFDM)		RatesBitmap |= HW_BIT_RATE_18MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_24_OFDM)		RatesBitmap |= HW_BIT_RATE_24MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_36_OFDM)		RatesBitmap |= HW_BIT_RATE_36MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_48_OFDM)		RatesBitmap |= HW_BIT_RATE_48MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_54_OFDM)		RatesBitmap |= HW_BIT_RATE_54MBPS;

	
	*HwRatesBitmap = RatesBitmap;

	return (OK);
}

/*
 * ----------------------------------------------------------------------------
 * Function : ConvertHwRateToDrvRate
 *
 * Input    :
 * Output   :
 * Process  : convert rate from Hw rate to Drv rate
 * Note(s)  : 
 * -----------------------------------------------------------------------------
 */

rate_e ConvertHwRateToDrvRate(UINT8 HwRate, BOOL bOFDMMudulation)
{
    /* 
     * This special case is done because of identical values of HW_RATE_1M & RATE_12MBPS
     * These values are Hw oriented and can't be changed. The way for distinguishing 
     * between them is using the modulation of the packet
     */
    if ( (HwRate == RATE_12MBPS) && (bOFDMMudulation) )
    {
        return DRV_RATE_12M;
    }

	switch (HwRate)
	{
		case RATE_1MBPS:	return DRV_RATE_1M;	
			
		case RATE_2MBPS:	return DRV_RATE_2M;	
		      
		case RATE_5_5MBPS:	return DRV_RATE_5_5M;	
		
		case RATE_6MBPS:	return DRV_RATE_6M;	
		
		case RATE_9MBPS:	return DRV_RATE_9M;	
		
		case RATE_11MBPS:	return DRV_RATE_11M;	
			
        /* RATE_12MBPS is covered on the top */ 

		case RATE_18MBPS:	return DRV_RATE_18M;	
		
		case RATE_22MBPS:	return DRV_RATE_22M;	
		
		case RATE_24MBPS:	return DRV_RATE_24M;	
		
		case RATE_36MBPS:	return DRV_RATE_36M;	
		
		case RATE_48MBPS:	return DRV_RATE_48M;	

        case RATE_54MBPS:	return DRV_RATE_54M;	
		
		default:
            /* Return error indication */
			return DRV_RATE_AUTO;			
	}
}

/*
* ----------------------------------------------------------------------------
* Function : ConvertHwRateToDrvRate
*
* Input    :
* Output   :
* Process  : convert rate from Drv rate to Hw rate
* Note(s)  : 
* -----------------------------------------------------------------------------
*/
UINT8 ConvertDrvRate2HwRate(rate_e eRate)
{
	switch(eRate)
	{
	case DRV_RATE_1M:	return RATE_1MBPS;

	case DRV_RATE_2M:	return RATE_2MBPS;

	case DRV_RATE_5_5M:	return RATE_5_5MBPS;

	case DRV_RATE_11M:	return RATE_11MBPS;

	case DRV_RATE_22M:	return RATE_22MBPS;

	case DRV_RATE_6M:	return RATE_6MBPS;

	case DRV_RATE_9M:	return RATE_9MBPS;

	case DRV_RATE_12M:	return RATE_12MBPS;

	case DRV_RATE_18M:	return RATE_18MBPS;

	case DRV_RATE_24M:	return RATE_24MBPS;

	case DRV_RATE_36M:	return RATE_36MBPS;

	case DRV_RATE_48M:	return RATE_48MBPS;

	case DRV_RATE_54M:	return RATE_54MBPS;

	default:
		WLAN_OS_REPORT(("ERROR: ConvertDrvRate2HwRate: Invalid input Rate = %d\n ", eRate));
		return 0;
	}
}




rate_e calculateMaxSupportedRate(UINT32 *pSuppRateMask)
{
	if((*pSuppRateMask) & DRV_RATE_MASK_54_OFDM)
		return DRV_RATE_54M;
	if((*pSuppRateMask) & DRV_RATE_MASK_48_OFDM)
		return DRV_RATE_48M;
	if((*pSuppRateMask) & DRV_RATE_MASK_36_OFDM)
		return DRV_RATE_36M;
	if((*pSuppRateMask) & DRV_RATE_MASK_24_OFDM)
		return DRV_RATE_24M;
	if((*pSuppRateMask) & DRV_RATE_MASK_22_PBCC)
		return DRV_RATE_22M;
	if((*pSuppRateMask) & DRV_RATE_MASK_18_OFDM)
		return DRV_RATE_18M;
	if((*pSuppRateMask) & DRV_RATE_MASK_12_OFDM)
		return DRV_RATE_12M;
	if((*pSuppRateMask) & DRV_RATE_MASK_11_CCK)
		return DRV_RATE_11M;
	if((*pSuppRateMask) & DRV_RATE_MASK_9_OFDM)
		return DRV_RATE_9M;
	if((*pSuppRateMask) & DRV_RATE_MASK_6_OFDM)
		return DRV_RATE_6M;
	if((*pSuppRateMask) & DRV_RATE_MASK_5_5_CCK)
		return DRV_RATE_5_5M;
	if((*pSuppRateMask) & DRV_RATE_MASK_2_BARKER)
		return DRV_RATE_2M;
	if((*pSuppRateMask) & DRV_RATE_MASK_1_BARKER)
		return DRV_RATE_1M;
	
	return DRV_RATE_INVALID;
}



/************************************************************************
 *                        hex_to_string				     				*
 ************************************************************************
DESCRIPTION: Converts hex buffer to string buffer.

NOTE:		 1. The caller has to make sure that the pString size is at 
				lease: ((Size * 2) + 1)
			 2. A string terminator is inserted into lase char of the string
************************************************************************/
void convert_hex_to_string(tiUINT8 *pBuffer, char *pString, tiUINT8 Size)
{
	int index;
	unsigned char temp_nibble;

	/* Go over pBuffer and convert it to chars */ 
	for (index = 0; index < Size; index++)
	{
		/* First nibble */
		temp_nibble = (pBuffer[index] & 0x0F);
		if (temp_nibble <= 9)
		{
			pString[(index << 1) + 1] = temp_nibble + '0';
		}
		else
		{
			pString[(index << 1) + 1] = temp_nibble - 10 + 'A';
		}

		/* Second nibble */
		temp_nibble = ((pBuffer[index] & 0xF0) >> 4);
		if (temp_nibble <= 9)
		{
			pString[(index << 1)] = temp_nibble + '0';
		}
		else
		{
			pString[(index << 1)] = temp_nibble - 10 + 'A';
		}
	}

	/* Put string terminator */
	pString[(Size * 2)] = 0;
}








UINT32 translateBasicRateValueToMask(UINT32 value, BOOL dot11a)
{
	if(!dot11a)
	{
		switch(value)
		{
			case BASIC_RATE_SET_1_2:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER;
			case BASIC_RATE_SET_1_2_5_5_11:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK;
			case BASIC_RATE_SET_UP_TO_12:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM;
			case BASIC_RATE_SET_UP_TO_18:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM;
			case BASIC_RATE_SET_UP_TO_24:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM;
			case BASIC_RATE_SET_UP_TO_36:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM;
			case BASIC_RATE_SET_UP_TO_48:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM;
			case BASIC_RATE_SET_UP_TO_54:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
			case BASIC_RATE_SET_6_12_24:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_24_OFDM;
			case BASIC_RATE_SET_1_2_5_5_6_11_12_24:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_24_OFDM;			
			default:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER;
		}
	}
	else
	{
		switch(value)
		{
			case BASIC_RATE_SET_UP_TO_12:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM;
			case BASIC_RATE_SET_UP_TO_18:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM;
			case BASIC_RATE_SET_UP_TO_24:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM;
			case BASIC_RATE_SET_UP_TO_36:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM;
			case BASIC_RATE_SET_UP_TO_48:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM;
			case BASIC_RATE_SET_UP_TO_54:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
			case BASIC_RATE_SET_6_12_24:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_24_OFDM;
			default:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_24_OFDM;
		}
	}
}

UINT32 translateSupportedRateValueToMask(UINT32 value, BOOL dot11a)
{
	if(!dot11a)
	{
		switch(value)
		{
			case SUPPORTED_RATE_SET_1_2:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER;
			case SUPPORTED_RATE_SET_1_2_5_5_11:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK;
			case SUPPORTED_RATE_SET_1_2_5_5_11_22:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_22_PBCC; 
			case SUPPORTED_RATE_SET_UP_TO_18:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_24:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_36:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_48:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_54:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
			case SUPPORTED_RATE_SET_ALL:
				return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_22_PBCC | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
			case SUPPORTED_RATE_SET_ALL_OFDM:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
		default:
			return DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_22_PBCC | DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
		}
	}
	else
	{
		switch(value)
		{
			case SUPPORTED_RATE_SET_UP_TO_18:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_24:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_36:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_48:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM;
			case SUPPORTED_RATE_SET_UP_TO_54:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;

			case SUPPORTED_RATE_SET_ALL:
			case SUPPORTED_RATE_SET_ALL_OFDM:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
			default:
				return DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_9_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_18_OFDM | DRV_RATE_MASK_24_OFDM | DRV_RATE_MASK_36_OFDM | DRV_RATE_MASK_48_OFDM | DRV_RATE_MASK_54_OFDM;
		}
	}
}

void	validateRatesVsBand(UINT32 *supportedMask, UINT32 *basicMask, BOOL dot11a)
{
	if(dot11a)
		*supportedMask &= ~(DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER | DRV_RATE_MASK_5_5_CCK | DRV_RATE_MASK_11_CCK | DRV_RATE_MASK_22_PBCC);

	*basicMask &= *supportedMask;

	if(*basicMask == 0)
	{ 	
		if(dot11a)
			*basicMask = DRV_RATE_MASK_6_OFDM | DRV_RATE_MASK_12_OFDM | DRV_RATE_MASK_24_OFDM;
		else
			*basicMask = DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER;
	}

}


rate_e findMaxActiveRate( UINT32 ratesBitMap)
{
	rate_e rate = DRV_RATE_1M;

	if(ratesBitMap & DRV_RATE_MASK_1_BARKER) rate = DRV_RATE_1M; 
	if(ratesBitMap & DRV_RATE_MASK_2_BARKER) rate = DRV_RATE_2M;
	if(ratesBitMap & DRV_RATE_MASK_5_5_CCK)  rate = DRV_RATE_5_5M;
	if(ratesBitMap & DRV_RATE_MASK_11_CCK)   rate = DRV_RATE_11M; 
	if(ratesBitMap & DRV_RATE_MASK_22_PBCC)  rate = DRV_RATE_22M;
	if(ratesBitMap & DRV_RATE_MASK_6_OFDM)   rate = DRV_RATE_6M;
	if(ratesBitMap & DRV_RATE_MASK_9_OFDM)   rate = DRV_RATE_9M;
	if(ratesBitMap & DRV_RATE_MASK_12_OFDM)  rate = DRV_RATE_12M;
	if(ratesBitMap & DRV_RATE_MASK_18_OFDM)  rate = DRV_RATE_18M;
	if(ratesBitMap & DRV_RATE_MASK_24_OFDM)  rate = DRV_RATE_24M;
	if(ratesBitMap & DRV_RATE_MASK_36_OFDM)  rate = DRV_RATE_36M;
	if(ratesBitMap & DRV_RATE_MASK_48_OFDM)  rate = DRV_RATE_48M;
	if(ratesBitMap & DRV_RATE_MASK_54_OFDM)  rate = DRV_RATE_54M;

	return rate;
}


/************************************************************************
 *                        getMaxRate									*
 ************************************************************************
DESCRIPTION: This function return the Max rate.
			 In addition return the matched modulation type
                                                                                                   
INPUT:      rateBitMap		-	The supported basic rates
			operationMode   -   Current operation mode, used only to set default rate.


OUTPUT:		rate - The max rate from the OFDM allowed rates.
			modulation - The modulation of the Max Basic rate.

RETURN:     None

************************************************************************/
void getMaxRate(UINT32 ratesBitMap, rate_e *rate, modulationType_e *modulation, dot11mode_e operationMode)
{
	rate_e maxRate = DRV_RATE_INVALID;

    /* find max rate */
    if(ratesBitMap & DRV_RATE_MASK_1_BARKER) maxRate = DRV_RATE_1M;
    if(ratesBitMap & DRV_RATE_MASK_2_BARKER) maxRate = DRV_RATE_2M;
    if(ratesBitMap & DRV_RATE_MASK_5_5_CCK)  maxRate = DRV_RATE_5_5M;
    if(ratesBitMap & DRV_RATE_MASK_11_CCK)   maxRate = DRV_RATE_11M;

    /* Ctrl and Mgmt frames should not be transmitted at 22Mbps PBCC */
	/*if(ratesBitMap & DRV_RATE_MASK_22_PBCC)   maxRate = DRV_RATE_22M;*/

	if(ratesBitMap & DRV_RATE_MASK_6_OFDM)   maxRate = DRV_RATE_6M;
	if(ratesBitMap & DRV_RATE_MASK_9_OFDM)   maxRate = DRV_RATE_9M;
	if(ratesBitMap & DRV_RATE_MASK_12_OFDM)  maxRate = DRV_RATE_12M;
	if(ratesBitMap & DRV_RATE_MASK_18_OFDM)  maxRate = DRV_RATE_18M;
	if(ratesBitMap & DRV_RATE_MASK_24_OFDM)  maxRate = DRV_RATE_24M;
	if(ratesBitMap & DRV_RATE_MASK_36_OFDM)  maxRate = DRV_RATE_36M;
	if(ratesBitMap & DRV_RATE_MASK_48_OFDM)  maxRate = DRV_RATE_48M;
	if(ratesBitMap & DRV_RATE_MASK_54_OFDM)  maxRate = DRV_RATE_54M;
	
	if(maxRate == DRV_RATE_INVALID)
	{
		/* No rate is supported */
		WLAN_OS_REPORT((" Error; The rate Bit field does not support any available rate\n"));
		
		if(operationMode == DOT11_A_MODE)
		{
			/* Default value for A band is 6M */
			maxRate = DRV_RATE_6M;
		}
		else
		{
			/* Default value for B/G band is 2M */
			maxRate = DRV_RATE_2M;
		}
	}

	/* Return the Max rate */
	*rate = maxRate;

	/* Return the matched modulation type */
	if(maxRate >= DRV_RATE_6M)
	{
		*modulation = DRV_MODULATION_OFDM;
	}
	else 
	if(maxRate >= DRV_RATE_5_5M)
	{
		*modulation = DRV_MODULATION_CCK;
	}
	else
	{
		*modulation = DRV_MODULATION_QPSK;
	}
	
	return;

}

/************************************************************************
 *                        getMinRate									*
 ************************************************************************
DESCRIPTION: This function return the Min rate.
			 In addition return the matched modulation type
                                                                                                   
INPUT:      rateBitMap		-	The supported basic rates
			operationMode   -   Current operation mode, used only to set default rate.


OUTPUT:		rate - The min rate from the OFDM allowed rates.
			modulation - The modulation of the Min Basic rate.

RETURN:     None

************************************************************************/
void getMinRate(UINT32 ratesBitMap, rate_e *rate, modulationType_e *modulation, dot11mode_e operationMode)
{
	rate_e minRate = DRV_RATE_INVALID;

    /* find min rate */
	if(ratesBitMap & DRV_RATE_MASK_54_OFDM)  minRate = DRV_RATE_54M;
	if(ratesBitMap & DRV_RATE_MASK_48_OFDM)  minRate = DRV_RATE_48M;
	if(ratesBitMap & DRV_RATE_MASK_36_OFDM)  minRate = DRV_RATE_36M;
	if(ratesBitMap & DRV_RATE_MASK_24_OFDM)   minRate = DRV_RATE_24M;
	if(ratesBitMap & DRV_RATE_MASK_18_OFDM)   minRate = DRV_RATE_18M;
	if(ratesBitMap & DRV_RATE_MASK_12_OFDM)  minRate = DRV_RATE_12M;
	if(ratesBitMap & DRV_RATE_MASK_9_OFDM)  minRate = DRV_RATE_9M;
	if(ratesBitMap & DRV_RATE_MASK_6_OFDM)  minRate = DRV_RATE_6M;

    /* Ctrl and Mgmt frames should not be transmitted at 22Mbps PBCC */
    /*if(ratesBitMap & DRV_RATE_MASK_22_PBCC)  minRate = DRV_RATE_22M;*/

	if(ratesBitMap & DRV_RATE_MASK_11_CCK)   minRate = DRV_RATE_11M;
	if(ratesBitMap & DRV_RATE_MASK_5_5_CCK)  minRate = DRV_RATE_5_5M;
	if(ratesBitMap & DRV_RATE_MASK_2_BARKER) minRate = DRV_RATE_2M;
	if(ratesBitMap & DRV_RATE_MASK_1_BARKER) minRate = DRV_RATE_1M;

	if(minRate == DRV_RATE_INVALID)
	{
		/* No rate is supported */
		WLAN_OS_REPORT((" Error; The rate Bit field does not support any available rate\n"));
		
		if(operationMode == DOT11_A_MODE)
		{
			/* Default value for A band is 6M */
			minRate = DRV_RATE_6M;
		}
		else
		{
			/* Default value for B/G band is 2M */
			minRate = DRV_RATE_2M;
		}
	}

	/* Return the Max rate */
	*rate = minRate;

	/* Return the matched modulation type */
	if(minRate >= DRV_RATE_6M)
	{
		*modulation = DRV_MODULATION_OFDM;
	}
	else 
	if(minRate >= DRV_RATE_5_5M)
	{
		*modulation = DRV_MODULATION_CCK;
	}
	else
	{
		*modulation = DRV_MODULATION_QPSK;
	}
	
	return;

}


UINT8 hostRateToNumber(rate_e rate)
{
	switch (rate)
	{
	case DRV_RATE_1M:	return 1;
	case DRV_RATE_2M:	return 2;
	case DRV_RATE_5_5M:	return 5;
	case DRV_RATE_11M:	return 11;
	case DRV_RATE_22M:	return 22;
	case DRV_RATE_6M:	return 6;
	case DRV_RATE_9M:	return 9;
	case DRV_RATE_12M:	return 12;
	case DRV_RATE_18M:	return 18;
	case DRV_RATE_24M:	return 24;
	case DRV_RATE_36M:	return 36;
	case DRV_RATE_48M:	return 48;
	case DRV_RATE_54M:	return 54;

	default:
		return 0;
	}
}

/*-----------------------------------------------------------------------------
Routine Name:    RateNumberToHost
Routine Description:
Arguments:
Return Value:    None
-----------------------------------------------------------------------------*/
rate_e  RateNumberToHost(UINT8 rateIn)
{
    switch(rateIn)
    {
    case 0x1:   return DRV_RATE_1M;
    case 0x2:   return DRV_RATE_2M;
    case 0x5:   return DRV_RATE_5_5M;
    case 0xB:   return DRV_RATE_11M;
    case 0x16:  return DRV_RATE_22M;
    case 0x6:   return DRV_RATE_6M;
    case 0x9:   return DRV_RATE_9M;
    case 0xC:   return DRV_RATE_12M;
    case 0x12:  return DRV_RATE_18M;
    case 0x18:  return DRV_RATE_24M;
    case 0x24:  return DRV_RATE_36M;
    case 0x30:  return DRV_RATE_48M;
    case 0x36:  return DRV_RATE_54M;
    default:    return DRV_RATE_6M;
    }
}

RateIndex_e rateNumberToIndex(UINT8 uRate)
{
	switch(uRate)
	{
	case 1:   return RATE_INDEX_1MBPS;
	case 2:   return RATE_INDEX_2MBPS;
	case 5:   return RATE_INDEX_5_5MBPS;
	case 6:   return RATE_INDEX_6MBPS; 
	case 9:   return RATE_INDEX_9MBPS; 
	case 11:  return RATE_INDEX_11MBPS;
	case 12:  return RATE_INDEX_12MBPS;
	case 18:  return RATE_INDEX_18MBPS;
	case 22:  return RATE_INDEX_22MBPS;
	case 24:  return RATE_INDEX_24MBPS;
	case 36:  return RATE_INDEX_36MBPS;
	case 48:  return RATE_INDEX_48MBPS;
	case 54:  return RATE_INDEX_54MBPS;
	default:
		return INVALID_RATE_INDEX;
	}
}


BOOL utils_isAnySSID(ssid_t *pSsid)
{
	if (pSsid == NULL)
	{
		return TRUE;
	}

	if (pSsid->len == 0)
	{
		return TRUE;
	}

	return FALSE;
}

BOOL utils_isJunkSSID(ssid_t *pSsid)
{
	if (pSsid == NULL)
	{
		return TRUE;
	}

	if (pSsid->len > 2)
	{
                unsigned char *ssidString = (unsigned char *)pSsid->ssidString;
		if ((ssidString[0] < FIRST_VALID_CHAR) &&
			(ssidString[1] < FIRST_VALID_CHAR) &&
			(ssidString[2] < FIRST_VALID_CHAR))
		{
			return TRUE;
		}
	}

	return FALSE;
}


BOOL utils_isIESSID_Broadcast(dot11_SSID_t *pIESsid)
{
	if ((pIESsid == NULL) || (pIESsid->hdr.eleLen==0))
	{
		return TRUE;
	}

    /* According to 802.11, Broadcast SSID should be with length 0,
        however, different vendors use invalid chanrs for Broadcast SSID. */
    if (pIESsid->serviceSetId[0] < FIRST_VALID_CHAR)
    {
        return TRUE;
    }

	return FALSE;
}

/* HEX DUMP for BDs !!! Debug code only !!! */
void HexDumpData (UINT8 *data, int datalen)
{
#ifdef TI_DBG
int j,dbuflen=0;
char dbuf[50];
static char hexdigits[16] = "0123456789ABCDEF";

	for(j=0; j < datalen;)
	{
		/* Add a byte to the line*/
		dbuf[dbuflen] =  hexdigits[(data[j] >> 4)&0x0f];
		dbuf[dbuflen+1] = hexdigits[data[j] & 0x0f];
		dbuf[dbuflen+2] = ' ';
		dbuf[dbuflen+3] = '\0';
		dbuflen += 3;
		j++;
		if((j % 16) == 0)
		{
			/* Dump a line every 16 hex digits*/
			WLAN_OS_REPORT(("%04.4x  %s\n", j-16, dbuf));
			dbuflen = 0;
		}
	}
	/* Flush if something has left in the line*/
	if(dbuflen)
		WLAN_OS_REPORT(("%04.4x  %s\n", j & 0xfff0, dbuf));
#endif
}

void msduContentDump (mem_MSDU_T* pMsdu, char *str)
{	
    INT32 msduLen;
	mem_BD_T* pCurrBd;
    
	WLAN_OS_REPORT(("%s\n", str));
	
	WLAN_OS_REPORT(("totalLen = %d\n", pMsdu->dataLen));
	WLAN_OS_REPORT(("headerLen = %d\n", pMsdu->headerLen));

	msduLen = pMsdu->dataLen;
	pCurrBd = pMsdu->firstBDPtr;
	
	while ((msduLen >= 0)&&(pCurrBd!=NULL))
	{
		WLAN_OS_REPORT(("\nBdLen = %d\n", pCurrBd->length));
		
		HexDumpData((UINT8*)(pCurrBd->data+pCurrBd->dataOffset), pCurrBd->length);
	
		msduLen -=  pCurrBd->length;
		pCurrBd =  pCurrBd->nextBDPtr;
	}

}


/**
*
* parseIeBuffer  - Parse a required information element.
*
* \b Description: 
*
* Parse an required information element
* and returns a pointer to the IE.
 * If given a matching buffer as well, returns a pointer to the first IE 
 * that matches the IE ID and the given buffer.
*
* \b ARGS:
*
*  I   - hOs - pointer to OS context
*  I   - pIeBuffer - pointer to the IE buffer  \n
*  I   - length - the length of the whole buffer
*  I   - desiredIeId - the desired IE ID 
*  O   - pDesiredIe - a pointer to the desired IE
*  I   - pMatchBuffer - a matching buffer in the IE buffer. Optional, if not required a NULL can be given. 
*  I   - matchBufferLen - the matching buffer length. Optional, if not required zero can be given. 
*  
*  
* \b RETURNS:
*
* TRUE if IE pointer was found, FALSE on failure. 
*
* \sa 
*/
BOOL parseIeBuffer(TI_HANDLE hOs, UINT8 *pIeBuffer, UINT16 length, UINT8 desiredIeId, UINT8 **pDesiredIe, UINT8 *pMatchBuffer, UINT8 matchBufferLen)
{

	dot11_eleHdr_t   *eleHdr;
	UINT8            *pCurIe;


	if (pDesiredIe!=NULL)
	{
		*pDesiredIe = NULL;
	}

	if ((pIeBuffer == NULL) || (length==0))
	{
	   return FALSE;	
	}

	pCurIe = pIeBuffer;
	
	while (length>0)
	{
		eleHdr = (dot11_eleHdr_t*)pCurIe;
		
		if (length<(eleHdr->eleLen+2))
		{
			return FALSE;
		}
		
		if (eleHdr->eleId == desiredIeId)
		{
            if ((matchBufferLen==0) || (pMatchBuffer == NULL) ||
				(!os_memoryCompare(hOs, &pCurIe[2], pMatchBuffer, matchBufferLen)))
			{
				if (pDesiredIe!=NULL)
				{
					*pDesiredIe = (UINT8*)eleHdr;
				}
				return TRUE;
			}

		}
		length -= eleHdr->eleLen+2;
		pCurIe += eleHdr->eleLen+2;
	}
	return FALSE;
}

/***************************************************************************
*							TiWlanIntToStr					               *
****************************************************************************
DESCRIPTION:	convert from UINT8 to asci string.
				NOTE: the method assume that the convert number is positive.
				debug ststus - radix == 16.
                                                                                                   
INPUT:      the number to convert.
			the radix of the number.

OUTPUT:		the convert string.
			on error return empty string.

RETURN:     void
****************************************************************************/
void TiWlanIntToStr(UINT8 theNumber , char *theString , UINT8 theRadix)
{
	int nibbleIndex;
	UINT8 temp , factor;

	for (nibbleIndex = 0 ; nibbleIndex < NUM_OF_NIBBLES_IN_BYTE ; ++nibbleIndex)
	{
		temp = theNumber;
		if ((nibbleIndex % NUM_OF_NIBBLES_IN_BYTE) == 0) /* if the upper nibble */
		{
			 temp >>= NIBBLE_SIZE_IN_BITS;
		}
		temp &= NIBBLE_MASK;

		if (temp < 10)
		{
			factor = 0x30;
		}
		else
		{
			factor = 0x37;
		}
		theString[nibbleIndex] = temp + factor;
	}
	theString[nibbleIndex] = '\0';
}


UINT32 getBasicRateMaskForSpecialBGchannel(void)
{
	return (UINT32)(translateBasicRateValueToMask(BASIC_RATE_SET_1_2_5_5_11, FALSE));
}


UINT32 getSupportedRateMaskForSpecialBGchannel(void)
{
	return (UINT32)(translateSupportedRateValueToMask(SUPPORTED_RATE_SET_1_2_5_5_11, FALSE));		
}


/***************************************************************************
*							reminder64   					               *
****************************************************************************
DESCRIPTION:	returns the reminder of a 64 bit number division by a 32
                bit number.
                                                                                                   
INPUT:      The dividee (64 bit number to divide)
			The divider (32 bit number to divide by)

OUTPUT:		
		    

RETURN:     The reminder
****************************************************************************/
UINT32 reminder64( UINT64 dividee, UINT32 divider )
{
    UINT32 divideeHigh, divideeLow, partA, partB, mod28n, mod24n, mod16n, partA8n, mod8n, mod4n;

    divideeHigh = INT64_HIGHER( dividee );
    divideeLow = INT64_LOWER( dividee );

    mod8n = 256 % divider;
    mod4n = 16 % divider;

    partA = (mod4n * (divideeHigh % divider)) % divider;
    partA8n = (partA * mod4n) % divider;
    mod16n = (partA8n * mod8n) % divider;
    mod24n = (mod8n * mod16n) % divider;
    mod28n = (mod4n * mod24n) % divider;

    partB = (mod4n * mod28n) % divider;
    return ( partB + (divideeLow % divider)) % divider;
}


/***************************************************************************
*							print_TI_STATUS   					           *
****************************************************************************
DESCRIPTION:	returns TI_STATUS as string
****************************************************************************/
char* convertTI_STATUS_toString(TI_STATUS status)
{
    switch (status)
    {
        case   OK:                           return "OK";		
        case   NOK:                          return "NOK";		
        case   PARAM_NOT_SUPPORTED:          return "PARAM_NOT_SUPPORTED";
        case   PARAM_VALUE_NOT_VALID:        return "PARAM_VALUE_NOT_VALID";
        case   CONFIGURATION_NOT_VALID:      return "CONFIGURATION_NOT_VALID";
        case   NO_SITE_SELECTED_YET:         return "NO_SITE_SELECTED_YET";
        case   RE_SCAN_NEEDED:               return "RE_SCAN_NEEDED";
        case   EXTERNAL_SET_PARAM_DENIED   : return "EXTERNAL_SET_PARAM_DENIED";
        case   EXTERNAL_GET_PARAM_DENIED   : return "EXTERNAL_GET_PARAM_DENIED";
        case   PARAM_MODULE_NUMBER_INVALID : return "PARAM_MODULE_NUMBER_INVALID";
        case   STATION_IS_NOT_RUNNING      : return "STATION_IS_NOT_RUNNING";
        case   CARD_IS_NOT_INSTALLED 	    : return "CARD_IS_NOT_INSTALLED";
        case   RX_MIC_FAILURE_ERROR        : return "RX_MIC_FAILURE_ERROR";
        case   RX_DECRYPT_FAILURE          : return "RX_DECRYPT_FAILURE";
        case   RX_STATUS_FAILURE           : return "RX_STATUS_FAILURE";
        case   TX_QUEUE_SELECTED_OK        : return "TX_QUEUE_SELECTED_OK";  
        case   NO_TX_QUEUE_SELECTED        : return "NO_TX_QUEUE_SELECTED";
        case   TX_STATUS_PENDING           : return "TX_STATUS_PENDING";
        case   TX_STATUS_NO_RESOURCES      : return "TX_STATUS_NO_RESOURCES";
        case   TX_STATUS_FAILURE           : return "TX_STATUS_FAILURE";
        case   TX_STATUS_OK                : return "TX_STATUS_OK";
        case   MAKE_CONCATENATION          : return "MAKE_CONCATENATION"; 
        case   SEND_ONE_MSDU               : return "SEND_ONE_MSDU";
        case   DO_NOT_SEND_MSDU            : return "DO_NOT_SEND_MSDU";
        case   FOUR_X_DISABLE              : return "FOUR_X_DISABLE";
        case   NO_COUNTRY                  : return "NO_COUNTRY";
        case   SCAN_ALREADY_IN_PROGRESS    : return "SCAN_ALREADY_IN_PROGRESS";
        case   NO_SCAN_IN_PROGRESS         : return "NO_SCAN_IN_PROGRESS";
        case   TX_POWER_SET_SAME_VALUE     : return "TX_POWER_SET_SAME_VALUE";
        case   CHANNEL_CHANGED             : return "CHANNEL_CHANGED";
        case   SUPPORT_IMMEDIATE_MEASUREMENT_ONLY : return "SUPPORT_IMMEDIATE_MEASUREMENT_ONLY";
        case   MEASUREMENT_TYPE_NOT_SUPPORT : return "MEASUREMENT_TYPE_NOT_SUPPORT"; 
        case   MEASUREMENT_CAN_NOT_EXECUTED_IN_PARALLEL : return "MEASUREMENT_CAN_NOT_EXECUTED_IN_PARALLEL";
        case   MEASUREMENT_REQUEST_IGNORED : return "MEASUREMENT_REQUEST_IGNORED";
        case   CANNOT_SET_MEASUREMENT_PARAM_WHEN_ACTIVATED : return "CANNOT_SET_MEASUREMENT_PARAM_WHEN_ACTIVATED";
        case   CANNOT_SET_CHANNEL_THAT_IS_NOT_SUPPORTED : return "CANNOT_SET_CHANNEL_THAT_IS_NOT_SUPPORTED";
        case   STATUS_BAD_KEY_PARAM : return "STATUS_BAD_KEY_PARAM";
        case   STATUS_RX_MIC_FAIL : return "STATUS_RX_MIC_FAIL";
        case   STATUS_FIRST_PRIMARY_SITE_SET : return "STATUS_FIRST_PRIMARY_SITE_SET";
        case   POWER_SAVE_802_11_SUCCESS  : return "POWER_SAVE_802_11_SUCCESS";
        case   POWER_SAVE_802_11_FAIL : return "POWER_SAVE_802_11_FAIL";
        case   POWER_SAVE_802_11_NOT_ALLOWED : return "POWER_SAVE_802_11_NOT_ALLOWED";
        case   PENDING : return "PENDING";
        case	SEND_COMPLETE_SUCCESS : return "SEND_COMPLETE_SUCCESS";
        case	SEND_COMPLETE_RETRY_EXCEEDED : return "SEND_COMPLETE_RETRY_EXCEEDED";
        case	SEND_COMPLETE_LIFETIME_EXCEEDED : return "SEND_COMPLETE_LIFETIME_EXCEEDED";
        case	SEND_COMPLETE_NO_LINK : return "SEND_COMPLETE_NO_LINK";
        case	SEND_COMPLETE_MAC_CRASHED : return "SEND_COMPLETE_MAC_CRASHED";
        case   POWER_SAVE_802_11_IS_CURRENT : return "POWER_SAVE_802_11_IS_CURRENT";
        case	SEND_PACKET_XFER_DONE : return "SEND_PACKET_XFER_DONE";
        case   SEND_PACKET_SUCCESS : return "SEND_PACKET_SUCCESS";
        case	SEND_PACKET_PENDING : return "SEND_PACKET_PENDING";
        case	SEND_PACKET_BUSY : return "SEND_PACKET_BUSY";
        case	SEND_PACKET_ERROR : return "SEND_PACKET_ERROR";
        case	TNETWIF_NONE : return "TNETWIF_NONE";
        case	TNETWIF_OK : return "TNETWIF_OK";
        case	TNETWIF_COMPLETE : return "TNETWIF_COMPLETE";
        case	TNETWIF_PENDING : return "TNETWIF_PENDING";
        case	TNETWIF_ERROR : return "TNETWIF_ERROR";
        case	TNETWIF_MORE : return "TNETWIF_MORE";
        default:
            WLAN_OS_REPORT(("%s No matching for TI_STATUS = %d , please add me !!!\n",__FUNCTION__ ,status));
            return "UnKnown";
    }
    
}



#if UTIL_DEBUG_PROFILE
profileInfo_t profileInfo;
profileInfo_t* pProfileInfo = &profileInfo;

void util_initProfile(void)
{
    pProfileInfo->currentInfoElement = 0;
    pProfileInfo->overlap = FALSE;
}


void util_recordProfile(UINT32 theTIWlanModuleLogName,
                        UINT32 theEvent,
                        UINT32 theParam_1,
                        UINT32 theParam_2)
{
    pProfileInfo->profileInfoElement[pProfileInfo->currentInfoElement].timeStamp = os_timeStampUs(NULL);
    pProfileInfo->profileInfoElement[pProfileInfo->currentInfoElement].TIWlanModuleLogName = theTIWlanModuleLogName;
    pProfileInfo->profileInfoElement[pProfileInfo->currentInfoElement].Event = theEvent;
    pProfileInfo->profileInfoElement[pProfileInfo->currentInfoElement].Param_1 = theParam_1;
    pProfileInfo->profileInfoElement[pProfileInfo->currentInfoElement].Param_2 = theParam_2;

    ++pProfileInfo->currentInfoElement;
    if (pProfileInfo->currentInfoElement == PROFILE_BUFFER_SIZE)
    {
        pProfileInfo->overlap = TRUE;
        pProfileInfo->currentInfoElement = 0;
    }
}

void util_printProfile(void)
{
    UINT32 index;
    UINT32 count;
    UINT32 elemCount;

    if (pProfileInfo->overlap == TRUE)
    {
        elemCount = PROFILE_BUFFER_SIZE;
        index = pProfileInfo->currentInfoElement;
    }
    else
    {
        elemCount = pProfileInfo->currentInfoElement;
        index = 0;
    }

    WLAN_OS_REPORT(("\nPrint profiling log (%d elements):\n",elemCount));

    for (count = 0 ; count < elemCount ; ++count)
    {
        WLAN_OS_REPORT(("%d  - %d.%06d   0x%04X - 0x%08X: 0x%08X 0x%08X\n",
                        index,
                        pProfileInfo->profileInfoElement[index].timeStamp/1000000,
                        pProfileInfo->profileInfoElement[index].timeStamp%1000000,
                        pProfileInfo->profileInfoElement[index].TIWlanModuleLogName,
                        pProfileInfo->profileInfoElement[index].Event,
                        pProfileInfo->profileInfoElement[index].Param_1,
                        pProfileInfo->profileInfoElement[index].Param_2));
        ++index;
    }

    WLAN_OS_REPORT(("\n\n"));

    util_initProfile();
}



#endif /* UTIL_DEBUG_PROFILE */
