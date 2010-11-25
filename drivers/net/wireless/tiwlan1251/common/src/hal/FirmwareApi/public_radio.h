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

/** \file public_radio.h
 *  \brief Contains information element defines/structures used by the TNETxxxx and host and Radio Module.
 *
 */

/*
=======================================================================================================================
                      R E V I S I O N    H I S T O R Y

  04/29/05  BRK  1. retrieved from ClearCase and added this rev. history
                 2. added two new entries to RadioParamType_e  enum
                 3. increased MAX_RADIO_PARAM_POWER_TABLE (from 20 to 56)
                    - this is sort of a kludge, struct RadioParam_t  should have used an
                      array pointer instead of an actual data block
  06/10/05  BRK  changed MAX_RADIO_PARAM_POWER_TABLE for 1251 support (sort of a KLUDGE)
  07/15/05  BRK  added RADIO_PABIAS_TABLE entry to RadioParamType_e  enum
  04/12/06  MH   Added new run-time calibration state: RFPLL_CALIBRATION_NEEDED

  Note: This code should only be edited with TAB stops set at 4
=======================================================================================================================
 */
#ifndef PUBLIC_RADIO
#define PUBLIC_RADIO

// radio parameter to set
 #ifdef TNETW1251
#define MAX_RADIO_PARAM_POWER_TABLE  4*48   // cPowLmtTbl[] max size for ABG radios
 #else
#define MAX_RADIO_PARAM_POWER_TABLE  56     // cPowLmtTbl[] max size for BG radios
 #endif
#define MAX_RADIO_PARAM_LEN          MAX_RADIO_PARAM_POWER_TABLE

#define RADIO_PARAM_POWER_TABLE_ENABLE        0x01  // mask for RADIO_PARAM_POWER_ENABLES usage
#define RADIO_PARAM_POWER_LIMIT_TABLE_ENABLE  0x02  // mask for RADIO_PARAM_POWER_ENABLES usage
#define RADIO_PARAM_POWER_ADJ_TABLE_ENABLE    0x04  // mask for RADIO_PARAM_POWER_ENABLES usage


#define NUM_OF_SUB_BANDS        5
#define NUM_OF_POWER_LEVEL      4

#ifndef HOST_IF_ENUMS_DISABLED
typedef enum RadioParamType_e
{
    RADIO_PARAM_POWER_TABLE = 1,
    RADIO_PARAM_POWER_LIMIT_TABLE,
    RADIO_PARAM_POWER_ADJ_TABLE,
    RADIO_PARAM_POWER_ENABLES,
    RADIO_PABIAS_TABLE,
	RADIO_PARAM_POWER_LEVELS,
    MAX_RADIO_PARAM_TYPE = 0x7FFFFFFF /* force this enum to be uint32 */
} RadioParamType_e;
#else
typedef uint32 RadioParamType_e;
#endif

typedef struct RadioParam_t
{
    RadioParamType_e parameterType;
    int8  parameter[MAX_RADIO_PARAM_LEN];
} RadioParam_t;


/******************************************************************************

    Name:	ACX_CAL_ASSESSMENT
	Type:	Configuration
	Access:	Write Only
	Length: 4
	Note:	OBSOLETE !!! (DO_CALIBRATION_IN_DRIVER is not defined)
	
******************************************************************************/

typedef enum
{
    RUNTIME_CALIBRATION_NOT_NEEDED = 1,
    RUNTIME_CALIBRATION_NEEDED = 2,
    RFPLL_CALIBRATION_NEEDED = 3,
    MAX_RUNTIME_CALIBRATION_OPTIONS = 0x7FFFFFFF // force this enum to be uint32 
} RadioRuntimeCalState_enum;



#ifdef HOST_COMPILE
typedef uint32 RadioRuntimeCalState_e;
#else
typedef RadioRuntimeCalState_enum RadioRuntimeCalState_e;
#endif

/************************************************************************/
/*																		*/	
/*							Commands section                            */
/*																		*/
/************************************************************************/


/******************************************************************************

	Name:	    ACX_PLT_NVS_BUFFER_UPDATE 
	TestCmdId:	TEST_CMD_PLT_GET_NVS_UPDATE_BUFFER
	Description: This PLT function provides the all information required by 
					the upper driver in order to update the NVS image.
					It received a parameter defining the type of update 
					information required and provides an array of elements defining 
					the data bytes to be written to the NVS image and the byte 
					offset in which they should be written.         
	Type:	PLT
	Access:	Read Only
	Length: 420

******************************************************************************/
#define  NVS_RESULTS_MAX_NUM_OF_TABLES		4
#define  NVS_RESULTS_MAX_UPDATE_TABLE_SIZE 100  

typedef struct resultsBuffer_t 
{
	 uint16	  size;								      /* size of table*/
     uint16   offset;							      /* offset in the binary image of the NVS file*/
     uint8    data[NVS_RESULTS_MAX_UPDATE_TABLE_SIZE];/* the actual table data */
}resultsBuffer_t;


typedef struct PltNvsResultsBuffer_t
{
	resultsBuffer_t	tables[NVS_RESULTS_MAX_NUM_OF_TABLES];	/* array of structures of type containing the tables*/
	uint8			numOfTables;				            /* number of tables needed to be updated*/
	uint8			padding[3];
}PltNvsResultsBuffer_t;


typedef struct PltGainGet_t
{
            uint8 TxGain;            //Total TX chain gain according to the current setting
            uint8 TxUpperBound;      //the max gain setting allowed
            uint8 TxLowerBound;      //the min gain setting allowed
            uint8 padding;           /* padding to 32 bit */
}PltGainGet_t;

#endif
