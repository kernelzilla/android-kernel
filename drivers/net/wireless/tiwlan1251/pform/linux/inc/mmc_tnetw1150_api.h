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


#ifndef MMC_TNETW1150_API_H
#define MMC_TNETW1150_API_H

#undef CONFIG_CEE
#define CONFIG_CEE

#include "mmc_omap_api.h"
 
/* 
   First partition is used to access first 90K of memory 
   during FW download or 90K of data memory during WLAN driver 
   regular work (0x00000-0x167FF; 0x00C800-0x22FFF - two 
   regions with overlap).
*/
#define SDIO_DOWNLOAD_PARTITION_START	(0x000000)
                                         /*  90K*/
#define SDIO_DOWNLOAD_PARTITION_SIZE	(0x016800) 
/*End of download partition is          (0x0167FF)    (90K)*/
                                    /*  60K*/
#define SDIO_MEM_PARTITION_START	(0x00F000)  
                                    /*  90K*/ 
#define SDIO_MEM_PARTITION_SIZE		(0x016800)  
/*End of first partition is             (0x025799)   (150K)*/

/* 
   Second partition is 34K of registers.
*/
                                    /*3072K*/
#define SDIO_REG_PARTITION_START	(0x300000)   
                                    /*  34K*/
#define SDIO_REG_PARTITION_SIZE		(0x008800)  
/*End of second partition is            (0x3087FF)  (3106K) */
                                         /*  90K*/ 
#define SDIO_DRIVER_REG_PARTITION_START (0x016800) 

#define AMAP_MAX_REGIONS                        4
                                /*128K-64B*/
#define AMAP_ONE_REGION  		(0x01FFC0)  

/* Status Card Register (SCR) address mapping */
typedef struct {
	unsigned int reg_size;	/* Region size */
	unsigned int scr_offset; /* Region offset in SCR address space */
} address_mapping_region;
typedef struct {
	unsigned int num_of_parts; /* number of partitions in use */
        /* h/w-dependant base addresses */
        address_mapping_region map_reg[AMAP_MAX_REGIONS]; /* CSR addresses */
} SDIO_TNETWConfigParams;
 
/*
  Initialization of TNETW memory configuration.
*/
SDIO_Status SDIO_TNETWInit(SDIO_TNETWConfigParams *);
 
/* 
  This function configures the slave SDIO device for the required 
  operation mode.
*/
SDIO_Status SDIO_TNETWConfig(SDIO_Handle, Peripheral_ConfigParams **);
 
/* 
  This function performs convertion of peripheral adddress into 17 bits 
  SDIO address. If found that the memory partition configuration must be 
  changed, the memory is re-mapped in accordance to requested address.
*/
SDIO_Address SDIO_ConvertTNETWToSDIOMaster(Peripheral_Address, SDIO_BufferLength);

/* 
  This function sets the ELP REG in the TNET
*/
SDIO_Status SDIO_TNETW_Set_ELP_Reg(SDIO_Handle sdioHandle, Peripheral_Address start_addr, unsigned int data);

/* 
This function gets the ELP REG in the TNET
*/
SDIO_Status SDIO_TNETW_Get_ELP_Reg(SDIO_Handle sdioHandle, Peripheral_Address start_addr, unsigned int *data);

#endif /* MMC_TNETW1150_API_H */
