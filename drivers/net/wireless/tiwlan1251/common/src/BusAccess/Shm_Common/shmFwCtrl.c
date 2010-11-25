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

/****************************************************************************
 *
 *   MODULE:  ShmFwCtrl.c
 *   PURPOSE: shared memory firmware control
 *
 ****************************************************************************/
#include "whalCommon.h"
#include "whalBus_Api.h"
#include "shmBus.h"
#include "TNETWIF.h"
#include "whalHwAccess.h"
#include "whalHwCtrl.h"
#include "shmFwCtrl.h"
#include "TNETW_Driver.h"
#include "CmdMBox_api.h"
#include "eventMbox_api.h"
#include "FwEvent_api.h"


/* Firmware image header size */
#define FW_HDR_SIZE 8


static TI_STATUS whal_FwCtrl_BootSm                 (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);
static TI_STATUS whal_FwCtrl_ResetSm                (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);
static TI_STATUS whal_FwCtrl_EepromlessStartBurstSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);                                                   
static TI_STATUS whal_FwCtrl_InitSequenceSm         (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);
static TI_STATUS whal_FwCtrl_LoadFwImageSm          (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);
static TI_STATUS whal_FwCtrl_FinalizeDownloadSm     (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);                                             
#ifdef USE_SYNC_API
static int       whal_FwCtrl_Reset                  (TI_HANDLE hWhalBus);
#endif


/* Handle return status inside a state machine */
#define EXCEPT(pwhalbus,status)                                 \
    switch (status) {                                           \
        case OK:                                                \
        case TNETWIF_COMPLETE:                                  \
             break;                                             \
        case TNETWIF_PENDING:                                   \
             return TNETWIF_PENDING;                            \
        default:                                                \
             whal_hwCtrl_FinalizeOnFailure (pwhalbus->hHwCtrl); \
             return TNETWIF_ERROR;                              \
    }


/* Handle return status inside an init sequence state machine  */
#define EXCEPT_I(pwhalbus,status)                               \
    switch (status) {                                           \
        case OK:                                                \
        case TNETWIF_COMPLETE:                                  \
             break;                                             \
        case TNETWIF_PENDING:                                   \
             pwhalbus->uInitSeqStatus = status;                 \
             return TNETWIF_PENDING;                            \
        default:                                                \
             whal_hwCtrl_FinalizeOnFailure (pwhalbus->hHwCtrl); \
             return TNETWIF_ERROR;                              \
    }


/* Handle return status inside a load image state machine */
#define EXCEPT_L(pwhalbus,status)                               \
    switch (status) {                                           \
        case OK:                                                \
        case TNETWIF_COMPLETE:                                  \
             break;                                             \
        case TNETWIF_PENDING:                                   \
             pwhalbus->DownloadStatus = status;                 \
             return TNETWIF_PENDING;                            \
        default:                                                \
             pwhalbus->DownloadStatus = status;                 \
             whal_hwCtrl_FinalizeOnFailure (pwhalbus->hHwCtrl); \
             return TNETWIF_ERROR;                              \
    }


/****************************************************************************
 *                      whalBus_FwCtrl_Boot()
 ****************************************************************************
 * DESCRIPTION: Download firmware code to the Hardware and run it
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whalBus_FwCtrl_Boot (TI_HANDLE hWhalBus, TI_HANDLE hHwCtrl, BootAttr_T *pBootAttr)
{ 
    whalBus_T *pWhalBus     = (whalBus_T *)hWhalBus;  
    HwCtrl_T  *pHwCtrl      = (HwCtrl_T *)hHwCtrl;  
                           
    pWhalBus->hHwCtrl = hHwCtrl;

    /* Store the pointer to the FW buffer for further use in FW download in part */
    pWhalBus->pFwBuf = (UINT8 *)pHwCtrl->uFwBuf;
    pWhalBus->uFwLastAddr = pHwCtrl->uFwAddr;
    pWhalBus->pEEPROMBuf = (UINT8 *)pHwCtrl->uEEEPROMBuf;
    pWhalBus->uEEPROMLen = pHwCtrl->uEEEPROMLen; 

    /*
     * Initialize the status of download to  pending 
     * It will be set to TNETWIF_COMPLETE at the FinalizeDownload function 
     */
    pWhalBus->DownloadStatus = TNETWIF_PENDING;

    /* Call the boot sequence state machine */
    pWhalBus->uInitStage = 0;

    os_memoryCopy (pWhalBus->hOs, &pWhalBus->BootAttr, pBootAttr, sizeof(BootAttr_T));

    whal_FwCtrl_BootSm (hWhalBus, HAL_INIT_MODULE_ID, OK);

    /*
     * If it returns the status of the StartInstance only then we can here query for the download status 
     * and then return the status up to the TNETW_Driver.
     * This return value will go back up to the TNETW Driver layer so that the init from OS will know
     * if to wait for the InitComplte or not in case of TNETWIF_ERROR.
     * This value will always be pending since the SPI is ASYNC 
     * and in SDIOa timer is set so it will be ASync also in anyway.
     */
    return pWhalBus->DownloadStatus;
}


 /****************************************************************************
 * DESCRIPTION: Firmware boot state machine
 * 
 * INPUTS:  
 *      TI_HANDLE hWhalBus  Handle to the Bus
 *      UINT8 module_id     The module id of the Init process in the TNETWIF
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK 
 ****************************************************************************/
static TI_STATUS whal_FwCtrl_BootSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T  *pWhalBus = (whalBus_T *)hWhalBus;
    BootAttr_T *pBootAttr;
    UINT8 minorMinorE2Ver = 0;

    EXCEPT (pWhalBus, status)

    switch (pWhalBus->uInitStage)
    {
    case 0:  
        pWhalBus->uInitStage ++;

        pWhalBus->uChipId = 0;

        /* Read the CHIP ID to get an indication that the bus is OK */
        status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF, 
                                     CHIP_ID, 
                                     &pWhalBus->uChipId, 
                                     module_id,
                                     (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                     hWhalBus);
        EXCEPT (pWhalBus, status)

    case 1:
        pWhalBus->uInitStage ++;

        /* This is only sanity check that the HW exists, we can continue and fail on FwLoad */
        if (pWhalBus->uChipId == CHIP_ID_1251_PG10)
        {
            WLAN_OS_REPORT(("Working on a 1251 PG 1.0 board.\n"));
        }
        else if (pWhalBus->uChipId == CHIP_ID_1251_PG11)
        {
            WLAN_OS_REPORT(("Working on a 1251 PG 1.1 board.\n"));
        }
        else if (pWhalBus->uChipId == CHIP_ID_1251_PG12)
        {
            WLAN_OS_REPORT(("Working on a 1251 PG 1.2 board.\n"));
        }
        else 
        {
            WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                               ("whalBus_FwCtrl_Boot: ERROR, Fail to identify Wlan Hardware card, ChipId(0x%x)=0x%x\n", 
                               CHIP_ID, pWhalBus->uChipId));

            WLAN_OS_REPORT (("Found unknown Chip Id = 0x%x\n", pWhalBus->uChipId));

            /*
             * NOTE: no exception because of forward compatibility
             */
        }
    
        /*
         * Soft reset 
         */
        pWhalBus->uResetStage = 0;
        pWhalBus->uSelfClearTime = 0;
        pWhalBus->uBootData = 0;
        status = whal_FwCtrl_ResetSm (pWhalBus, module_id, OK);    

        EXCEPT (pWhalBus, status)

    case 2:
        pWhalBus->uInitStage ++;

        WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, ("TNET SOFT-RESET\n"));                         

        WLAN_OS_REPORT(("Starting to process NVS...\n"));

        /*
         * Start EEPROM/NVS burst (get RadioType)
         */
        if (pWhalBus->pEEPROMBuf) 
        {
            /* NVS file exists (EEPROM-less support) */
            pWhalBus->uEEPROMCurLen = pWhalBus->uEEPROMLen;
            pWhalBus->pEEPROMCurPtr = pWhalBus->pEEPROMBuf;
            pWhalBus->uEEPROMStage = 0;

            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                              ("whal_FwCtrl_EepromlessStartBurst: EEPROM Image addr=0x%x, EEPROM Len=0x0x%x\n", 
                              pWhalBus->pEEPROMBuf, pWhalBus->uEEPROMLen));
            status = whal_FwCtrl_EepromlessStartBurstSm (hWhalBus, module_id, OK);

            EXCEPT (pWhalBus, status)
        }

    case 3: 
        pWhalBus->uInitStage ++;

        if (pWhalBus->pEEPROMBuf) 
        {
            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          ACX_EEPROMLESS_IND_REG, 
                                          pWhalBus->uFwLastAddr,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                          hWhalBus);
            EXCEPT (pWhalBus, status)
        }

    case 4:
        pWhalBus->uInitStage ++;

        if (pWhalBus->pEEPROMBuf) 
        {
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG,
                              ("DRIVER NVS BURST-READ\n"));
        }

        if (!pWhalBus->pEEPROMBuf) 
        {
            /*
             * Start ACX EEPROM
             */     
            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          ACX_REG_EE_START, 
                                          START_EEPROM_MGR,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                          hWhalBus);
            EXCEPT (pWhalBus, status)
        }

    case 5:
        pWhalBus->uInitStage ++;

        if (!pWhalBus->pEEPROMBuf) 
        {           
            /*
             * The stall is needed so the EEPROM NVS burst read will complete
             */     
            os_StalluSec (pWhalBus->hOs, 40000);

            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          ACX_EEPROMLESS_IND_REG, 
                                          USE_EEPROM,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                          hWhalBus);
            EXCEPT (pWhalBus, status)
        }

    case 6:
        pWhalBus->uInitStage ++;

        if (!pWhalBus->pEEPROMBuf) 
        {
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG,
                              ("STARTING EEPROM NVS BURST-READ\n"));
        }

        /* Read the EEPROM parameters */
        status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF, 
                                     SCR_PAD2, 
                                     &pWhalBus->uBootData,
                                     module_id,
                                     (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                     hWhalBus);
        EXCEPT (pWhalBus, status)

    case 7:
        pWhalBus->uInitStage ++;

        pBootAttr = &pWhalBus->BootAttr;
        pBootAttr->radioType = (pWhalBus->uBootData & 0x0000FF00) >> 8;
        pBootAttr->majorE2Ver = (pWhalBus->uBootData & 0x00FF0000) >> 16;

        status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF,
                                     SCR_PAD3,
                                     &pWhalBus->uBootData,
                                     module_id,
                                     (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                     hWhalBus);
        EXCEPT (pWhalBus, status)

    case 8:
        pWhalBus->uInitStage ++;

        pBootAttr = &pWhalBus->BootAttr;
        pBootAttr->minorE2Ver = (pWhalBus->uBootData & 0x00FF0000) >> 16;
        minorMinorE2Ver = (pWhalBus->uBootData & 0xFF000000) >> 24;

        if (pBootAttr->radioType == 0xffffffff)
        {
            WLAN_REPORT_FATAL_ERROR (pWhalBus->hReport, HAL_CTRL_MODULE_LOG,
                                     ("whalBus_FwCtrl_Boot: error in RadioType\n"));
            EXCEPT (pWhalBus, TNETWIF_ERROR)
        }

        WLAN_OS_REPORT(("NVS version %d.%d.%d found.\n", pBootAttr->majorE2Ver, pBootAttr->minorE2Ver, minorMinorE2Ver));
        WLAN_OS_REPORT(("Radio type is 0x%X.\n", pBootAttr->radioType));

        /* Call the restart sequence */
        pWhalBus->uInitSeqStage = 0;
        pWhalBus->uInitSeqStatus = TNETWIF_COMPLETE;
        status = whal_FwCtrl_InitSequenceSm (hWhalBus, module_id, OK);

        EXCEPT (pWhalBus, status)

    case 9:
        pWhalBus->uInitStage ++;

        WLAN_OS_REPORT(("Finished processing NVS.\n"));

        /* Download the firmware */
        status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF,
                                     ACX_REG_ECPU_CONTROL,
                                     &pWhalBus->uBootData,
                                     module_id,
                                     (TNETWIF_callback_t)whal_FwCtrl_BootSm,
                                     hWhalBus);
        EXCEPT (pWhalBus, status)

    case 10:
        pWhalBus->uInitStage = 0;

        if (pWhalBus->pFwBuf && (pWhalBus->uBootData & ECPU_CONTROL_HALT) != 0)
        {
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG,
                              ("CPU halt -> download code"));

            /* Load firmware image */ 
            pWhalBus->uLoadStage = 0;
            status = whal_FwCtrl_LoadFwImageSm (pWhalBus, module_id, OK);

            switch (status)
            {
            case TNETWIF_COMPLETE:
                /*WLAN_OS_REPORT (("Firmware successfully downloaded.\n"));*/
                break;
            case TNETWIF_PENDING:
                WLAN_OS_REPORT (("Starting to download firmware...\n"));
                break;
            default:
                WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG, ("Firmware download failed!\n"));                                   
                break;
            }

            EXCEPT (pWhalBus, status);
        }   
        else
        {
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, ("Firmware not downloaded...\n"));

            EXCEPT (pWhalBus, TNETWIF_ERROR)
        }
            
    } /* Switch end */

    return TNETWIF_COMPLETE;
}                                                  
    

/****************************************************************************
 *                      whal_FwCtrl_FinalizeDownloadSm()
 ****************************************************************************
 * DESCRIPTION: Run the Hardware firmware
 *              Wait for Init Complete
 *              Configure the Bus Access with Addresses available on the scratch pad register 
 *              Change the SDIO/SPI partitions to be able to see all the memory addresses
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
static TI_STATUS whal_FwCtrl_FinalizeDownloadSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    #define FIN_LOOP 20000

    EXCEPT (pWhalBus, status)

    while (TRUE)
    {
        switch (pWhalBus->uFinStage)
        {
        case 0:
            pWhalBus->uFinStage ++;

            /*
             * Run the firmware (I)
             */
            status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF, 
                                         ACX_REG_ECPU_CONTROL, 
                                         &pWhalBus->uFinData,
                                         module_id,
                                         (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm,
                                         hWhalBus);
            EXCEPT (pWhalBus, status);

        case 1:
            pWhalBus->uFinStage ++;

            /*
             * Run the firmware (II)
             */
            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          ACX_REG_ECPU_CONTROL, 
                                          pWhalBus->uFinData & ~ECPU_CONTROL_HALT,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm,
                                          hWhalBus);
            EXCEPT (pWhalBus, status);

        case 2:
            pWhalBus->uFinStage ++;

          #if defined(TNETW1150) && defined(RIVENDELL)    
            /* (!!!1150) added when testing with the prateekai/rivendell */
            WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                               ("whal_hwCtrl_Run: Only 1150 - wait 500 msec between FW download and run CPU\n"));
            os_StalluSec (pWhalBus->hOs, 500000);
          #endif

            WLAN_OS_REPORT (("Firmware running.\n"));

            /* 
             * CHIP ID Debug
             */     
            status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF,
                                         CHIP_ID,
                                         &pWhalBus->uFinData,
                                         module_id,
                                         (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm,
                                         hWhalBus);
            EXCEPT (pWhalBus, status);

        case 3:
            pWhalBus->uFinStage ++;
            pWhalBus->uFinLoop = 0;

            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
                              ("CHIP ID IS %x\n", pWhalBus->uFinData));
                   
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG, ("Wait init complete\n")); 

        case 4:
            /* 
             * Wait for init complete 
             */
            if (pWhalBus->uFinLoop < FIN_LOOP)
            {           
                pWhalBus->uFinStage = 5;

                os_StalluSec (pWhalBus->hOs, 50);

                /* Read interrupt status register */
                status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF, 
                                             ACX_REG_INTERRUPT_NO_CLEAR, 
                                             &pWhalBus->uFinData,
                                             module_id,
                                             (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm,
                                             hWhalBus); 
                EXCEPT (pWhalBus, status);
            }
            else
                pWhalBus->uFinStage = 6;
            continue;

        case 5:
            if (pWhalBus->uFinData == 0xffffffff) /* error */
            {
                WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                   ("Error reading hardware complete init indication\n"));

                pWhalBus->DownloadStatus = TNETWIF_ERROR;
                EXCEPT (pWhalBus, TNETWIF_ERROR);
            }

            if (IS_MASK_ON (pWhalBus->uFinData, ACX_INTR_INIT_COMPLETE))
            {
                pWhalBus->uFinStage = 6;

                /* Interrupt ACK */
                status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                              ACX_REG_INTERRUPT_ACK, 
                                              ACX_INTR_INIT_COMPLETE,
                                              module_id,
                                              (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm,
                                              hWhalBus); 
                EXCEPT (pWhalBus, status);
            }
            else
            {
                pWhalBus->uFinStage = 4;
                pWhalBus->uFinLoop ++;
            }
            continue;

        case 6:               
            pWhalBus->uFinStage = 7;

            if (pWhalBus->uFinLoop >= FIN_LOOP)
            {
                WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                   ("Timeout waiting for the hardware to complete initialization\n"));

                pWhalBus->DownloadStatus = TNETWIF_ERROR;
                EXCEPT (pWhalBus, TNETWIF_ERROR);
            }
        
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, ("Firmware init complete...\n"));

            /* 
             * There are valid addresses of the command and event mailbox 
             * on the scratch pad registers 
             */
            {
                /* Hardware config command mail box */
                TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)pWhalBus->hTnetwDrv;
                status = CmdMBox_ConfigHw (pTnetwDrv->hCmdMBox,
                                           module_id, 
                                           (fnotify_t)whal_FwCtrl_FinalizeDownloadSm, 
                                           hWhalBus);
                EXCEPT (pWhalBus, status);
            }
            continue;

        case 7:  
            pWhalBus->uFinStage = 8;

            {
                /* Hardware config event mail box */
                TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)pWhalBus->hTnetwDrv;
                status = eventMbox_ConfigHw (pTnetwDrv->hEventMbox,
                                             module_id,
                                             (fnotify_t)whal_FwCtrl_FinalizeDownloadSm, 
                                             hWhalBus);
                EXCEPT (pWhalBus, status);
            }
            continue;

        case 8: 
            pWhalBus->uFinStage = 9;

            /* Set the working partition to its "running" mode offset */
          #if defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI)
            status = TNETWIF_SetPartitionsOpt (pWhalBus->hTNETWIF, 
                                               HW_ACCESS_WORKING, 
                                               HW_ACCESS_WORK_PART0_ADDR,
                                               module_id,
                                               (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm, 
                                               hWhalBus);
             EXCEPT (pWhalBus, status);
          #endif
            continue;
        
        case 9:   
            pWhalBus->uFinStage = 10;
       
            /* 
             * In case of full asynchronous mode the firmware event must be ready 
             * to receive event from the command mailbox
             */
            {
                TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)pWhalBus->hTnetwDrv;
                UINT32      uIntVect;

				if (pWhalBus->recoveryProcess == FALSE)
                FwEvent_Config (pTnetwDrv->hFwEvent, pWhalBus->hTnetwDrv);

              #if !defined(USE_SYNC_API)
                             
                /* This makes command mailbox to work in normal mode */
                whalBus_ExitFromInitMode (hWhalBus); 

                /* Enable command complete interrupt */
                FwEvent_Enable (pTnetwDrv->hFwEvent, ACX_INTR_CMD_COMPLETE);

                /* At the driver init the interrupts must be disabled */
                os_enableIrq (pWhalBus->hOs);

              #endif

              #ifdef PRIODIC_INTERRUPT
			    /* Enable periodic interrupts. It means that every period of time the FwEvent SM will be called */
                os_periodicIntrTimerStart (pWhalBus->hOs);
              #endif

                uIntVect = FwEvent_GetEnabled (pTnetwDrv->hFwEvent);

                /* Clearing all the interrupt status register sources */
                status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                              ACX_REG_INTERRUPT_MASK, 
                                              ~uIntVect,
                                              module_id,
                                              (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm, 
                                              hWhalBus);
            }

            EXCEPT (pWhalBus, status);
            continue;
    
        case 10:
            pWhalBus->uFinStage = 11;

            /*
             * Setting the right operation of the interrupt
             * bit 5 - enable interrupt
             * bit 7 - active low 
             */
            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          HI_CFG, 
                                          HI_CFG_DEF_VAL,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_FinalizeDownloadSm, 
                                          hWhalBus);
            EXCEPT (pWhalBus, status);
            continue;

        case 11:
            pWhalBus->uFinStage = 0;

          #ifdef DEBUG_INTERRUPTS_PRINT
            WLAN_REPORT_INFORMATION (pHwIntr->hReport,
                                     HAL_HW_CTRL_MODULE_LOG,
                                     ("whal_hwIntr_EnableInterrupts(0x%08X)",
                                     pHwIntr->InterruptEnabled));
          #endif

          #if defined(HAL_ON_WIN)
            /* (!!!) Only in CardBus, add HostIfType parameter */
            /* Enable interrupt on a CardBus */
            TNETWIF_WriteRegSync (pWhalBus->hTNETWIF, FEMR, 0x8000);
          #endif

            /* 
             * The last thing to be done after the upper layers have been called 
             * is to send FINISH to the TNETWIF to end the init process 
             */    
            TNETWIF_Finish (pWhalBus->hTNETWIF, HAL_INIT_MODULE_ID, hWhalBus, NULL);

            /* Call the whal_hwCtrl_FinalizeDownload of the upper layer to finalize the download process */
            whal_hwCtrl_FinalizeDownload (pWhalBus->hHwCtrl, &pWhalBus->BootAttr);
        
            /* Set the Download Status to COMPLETE */
            pWhalBus->DownloadStatus = TNETWIF_COMPLETE;

            return TNETWIF_COMPLETE;

        } /* End switch */

    } /* End while */

}


#ifdef USE_SYNC_API

/****************************************************************************
 *                      whal_hwCtrl_Reset()
 ****************************************************************************
 * DESCRIPTION: Reset the Hardware
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static int whal_FwCtrl_Reset (TI_HANDLE hWhalBus)
{
#ifdef USE_SYNC_API

    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    UINT32 data;

#ifdef  TNETW1251

    /***************************************************************/
    /* SOFT RESET is done here - its a temporary fix */
    /***************************************************************/
    UINT32 SelfClearTime;

    /*
     * Perform Soft Reset
     */
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF,  ACX_REG_SLV_SOFT_RESET, SLV_SOFT_RESET_BIT);

    /* SOFT_RESET - Self clearing  */
    for (SelfClearTime=0; SelfClearTime<SOFT_RESET_MAX_TIME; SelfClearTime+=SOFT_RESET_STALL_TIME)
    {
        TNETWIF_ReadRegSync(pWhalBus->hTNETWIF, ACX_REG_SLV_SOFT_RESET,&data);
        if (( data & SLV_SOFT_RESET_BIT) == 0)
            break;
        os_StalluSec(pWhalBus->hOs, SOFT_RESET_STALL_TIME);     
    }

    WLAN_REPORT_INFORMATION(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_Reset: SOFT_RESET Self clearing time = %d (%d)\n", SelfClearTime, SOFT_RESET_MAX_TIME));
    if (SelfClearTime >= SOFT_RESET_MAX_TIME)
    {
        WLAN_REPORT_FATAL_ERROR(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwCtrl_Reset: ACX_REG_SLV_SOFT_RESET - Self clearing timer expired !!!\n"));
        return NOK;
    }

    /***************************************************************/
    /* SOFT RESET is done here - its a temporary fix */
    /***************************************************************/

    /* Disable Rx/Tx */
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, ENABLE, 0x0);      /* disable TX,RX */ 

    /* Auto Calibration on start Disable */
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, SPARE_A2, (UINT32)0xFFFF);

#else /* TNETW1251 */
    UINT32 SelfClearTime;

    /*
     * Halt the Acx Cpu
     */
    TNETWIF_RegIsBitSet(pWhalBus->hTNETWIF,  ACX_REG_ECPU_CONTROL, ECPU_CONTROL_HALT);

    /*
     * Reset the ACX cpu
     */
    TNETWIF_RegIsBitSet(pWhalBus->hTNETWIF,  ACX_REG_SLV_SOFT_RESET, SLV_SOFT_RESET_BIT);

    /*
     * Wait for Soft reset (Self clearing only in 1150)
     */
#if defined(TNETW1150)
    /* SOFT_RESET - Self clearing only on 1150 */
    for (SelfClearTime=0; SelfClearTime<SOFT_RESET_MAX_TIME; SelfClearTime+=SOFT_RESET_STALL_TIME)
    {
        os_StalluSec(pWhalBus->hOs, SOFT_RESET_STALL_TIME);
        
        if (((TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,ACX_REG_SLV_SOFT_RESET,&data)) & SLV_SOFT_RESET_BIT) == 0)
            break;
    }

    WLAN_REPORT_INFORMATION(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_Reset: SOFT_RESET Self clearing time = %d (%d)\n", SelfClearTime, SOFT_RESET_MAX_TIME));
    if (SelfClearTime >= SOFT_RESET_MAX_TIME)
    {
        WLAN_REPORT_FATAL_ERROR(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwCtrl_Reset: ACX_REG_SLV_SOFT_RESET - Self clearing timer expired !!!\n"));
        return NOK;
    }
#else
    os_StalluSec(pWhalBus->hOs, 10000);
    TNETWIF_RegResetBitVal(pWhalBus->hTNETWIF,  ACX_REG_SLV_SOFT_RESET, SLV_SOFT_RESET_BIT);
#endif
    
    /*
     * Start Acx Eeprom
     */
    TNETWIF_RegIsBitSet(pWhalBus->hTNETWIF,  ACX_REG_EE_START, START_EEPROM_MGR);

    /* Do Not Reduce the StallSec time !!!!! */
    os_StalluSec(pWhalBus->hOs, 40000);

#endif /* TNETW1251 */

#endif /* USE_SYNC_API*/

    return OK;
}

#endif /* USE_SYNC_API */


/****************************************************************************
 *                      whal_hwCtrl_Reset()
 ****************************************************************************
 * DESCRIPTION: Reset hardware state machine
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static TI_STATUS whal_FwCtrl_ResetSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    /***************************************************************/
    /* SOFT RESET is done here - its a temporary fix               */
    /***************************************************************/

    EXCEPT (pWhalBus, status);

    switch (pWhalBus->uResetStage)
    {
    case 0:
        /*
         * Perform soft reset
         */   
        pWhalBus->uResetStage ++;
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      ACX_REG_SLV_SOFT_RESET, 
                                      SLV_SOFT_RESET_BIT,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_ResetSm,
                                      hWhalBus);
        EXCEPT (pWhalBus, status);

    case 1:
        /* SOFT_RESET - self clearing */
        while (pWhalBus->uSelfClearTime <  SOFT_RESET_MAX_TIME) 
        {
            if (pWhalBus->uSelfClearTime != 0)
            {
                if ((pWhalBus->uBootData & SLV_SOFT_RESET_BIT) == 0)
                    break;
                os_StalluSec (pWhalBus->hOs, SOFT_RESET_STALL_TIME);     
            }

            status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF, 
                                         ACX_REG_SLV_SOFT_RESET,
                                         &pWhalBus->uBootData,
                                         module_id,
                                         (TNETWIF_callback_t)whal_FwCtrl_ResetSm,
                                         hWhalBus);

            pWhalBus->uSelfClearTime += SOFT_RESET_STALL_TIME;

            EXCEPT (pWhalBus, status);
        }

        pWhalBus->uResetStage ++;

    case 2:
        pWhalBus->uResetStage ++;

        WLAN_REPORT_INFORMATION (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                 ("whal_hwCtrl_Reset: SOFT_RESET self clearing time = %d (%d)\n", 
                                 pWhalBus->uSelfClearTime, SOFT_RESET_MAX_TIME));
        if (pWhalBus->uSelfClearTime >= SOFT_RESET_MAX_TIME)
        {
            WLAN_REPORT_FATAL_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                     ("whal_hwCtrl_Reset: ACX_REG_SLV_SOFT_RESET - Self clearing timer expired !!!\n"));
            EXCEPT (pWhalBus, TNETWIF_ERROR);
        }

    case 3:
        pWhalBus->uResetStage ++;

        /* Disable Rx/Tx */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      ENABLE, 
                                      0x0,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_ResetSm,
                                      hWhalBus);
        EXCEPT (pWhalBus, status);

    case 4:
        pWhalBus->uResetStage ++;

        /* Disable auto calibration on start */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      SPARE_A2, 
                                      0xFFFF,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_ResetSm,
                                      hWhalBus);
        return status;

    case 5:
        pWhalBus->uResetStage = 0;

        /* If the previous status was pending call the upper layer init state machine */
        whal_FwCtrl_BootSm (hWhalBus, module_id, status);
    }

    return status;
}


/****************************************************************************
 *                      whal_FwCtrl_Eepromless_StartBurst()
 ****************************************************************************
 * DESCRIPTION: prepare eepromless configuration before boot
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: 
 ****************************************************************************/
static TI_STATUS whal_FwCtrl_EepromlessStartBurstSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T* pWhalBus = (whalBus_T *)hWhalBus;
           
    EXCEPT (pWhalBus, status);

    while (TRUE)
    {
        switch (pWhalBus->uEEPROMStage)
        {
        case 0: 
            if ((pWhalBus->uEEPROMRegAddr = pWhalBus->pEEPROMCurPtr[1]) & 1)
            {
                pWhalBus->uEEPROMRegAddr &= 0xfe;
                pWhalBus->uEEPROMRegAddr |= (UINT32)pWhalBus->pEEPROMCurPtr[2] << 8;
                pWhalBus->uEEPROMBurstLen = pWhalBus->pEEPROMCurPtr[0];
                pWhalBus->pEEPROMCurPtr += 3;
                pWhalBus->uEEPROMBurstLoop = 0; 
                pWhalBus->uEEPROMStage = 1;
            }
            else
            {
                if (pWhalBus->pEEPROMCurPtr[0] == 0)
                    pWhalBus->pEEPROMCurPtr += 7;
                pWhalBus->uEEPROMCurLen -= pWhalBus->pEEPROMCurPtr - pWhalBus->pEEPROMBuf;
                pWhalBus->uEEPROMCurLen = (pWhalBus->uEEPROMCurLen + NVS_DATA_BUNDARY_ALIGNMENT - 1) & 0xfffffffc;
                pWhalBus->uEEPROMStage = 2;
            }
            continue;            

        case 1: 
            if (pWhalBus->uEEPROMBurstLoop < pWhalBus->uEEPROMBurstLen)
            {
                UINT32 val = (pWhalBus->pEEPROMCurPtr[0] | 
                              (pWhalBus->pEEPROMCurPtr[1] << 8) | 
                              (pWhalBus->pEEPROMCurPtr[2] << 16) | 
                              (pWhalBus->pEEPROMCurPtr[3] << 24));

                WLAN_REPORT_INIT (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                        ("NVS::BurstRead: *(%08x) = %x\n", pWhalBus->uEEPROMRegAddr, val));

                status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                              pWhalBus->uEEPROMRegAddr, 
                                              val,
                                              module_id,
                                              (TNETWIF_callback_t)whal_FwCtrl_EepromlessStartBurstSm,
                                              hWhalBus); 
 
                pWhalBus->uEEPROMStatus = status;
                pWhalBus->uEEPROMRegAddr += 4;
                pWhalBus->pEEPROMCurPtr += 4;
                pWhalBus->uEEPROMStage = 1;
                pWhalBus->uEEPROMBurstLoop ++;

                EXCEPT (pWhalBus, status);
            }
            else
                pWhalBus->uEEPROMStage = 0;
            continue;

        case 2: 
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                    ("Get NVS file information: NvsDataLen = %#x TableAddr %#x\n", pWhalBus->uEEPROMCurLen, pWhalBus->uFwLastAddr));
            pWhalBus->uNVSStartAddr = pWhalBus->uFwLastAddr;
            pWhalBus->uNVSNumChar = 0;
            pWhalBus->uNVSNumByte = 0;
            pWhalBus->uNVSTempWord = 0;
            pWhalBus->uEEPROMStage = 3;
    
          #if defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI)
            status = TNETWIF_SetPartitionsOpt (pWhalBus->hTNETWIF,
                                               HW_ACCESS_DOWNLOAD, 
                                               pWhalBus->uNVSStartAddr,
                                               module_id,
                                               (TNETWIF_callback_t)whal_FwCtrl_EepromlessStartBurstSm,
                                               hWhalBus);
            EXCEPT (pWhalBus, status);                   
          #endif
            continue;

        case 3:
            /*
             * Download EEPROM data to ACX internal memory
             */
            if (pWhalBus->uNVSNumChar < pWhalBus->uEEPROMCurLen)
            {
                pWhalBus->uNVSTempWord |= (*pWhalBus->pEEPROMCurPtr) << (8 * pWhalBus->uNVSNumByte);
                pWhalBus->pEEPROMCurPtr ++;
                pWhalBus->uNVSNumChar ++;

                if (++pWhalBus->uNVSNumByte > 3)
                {
                    pWhalBus->uEEPROMStage = 4;
                    pWhalBus->uNVSTempWord = ENDIAN_HANDLE_LONG (pWhalBus->uNVSTempWord);                   
                    WLAN_REPORT_INIT (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                        ("NVS::WriteTable: *(%08x) = %x\n", pWhalBus->uNVSStartAddr, pWhalBus->uNVSTempWord));
                    status = TNETWIF_WriteMemOpt (pWhalBus->hTNETWIF, 
                                                  pWhalBus->uNVSStartAddr, 
                                                  PADWRITE (&pWhalBus->uNVSTempWord), 
                                                  sizeof(pWhalBus->uNVSTempWord),
                                                  module_id,
                                                  (TNETWIF_callback_t)whal_FwCtrl_EepromlessStartBurstSm,
                                                  hWhalBus);
                    pWhalBus->uNVSStatus = status;

                    EXCEPT (pWhalBus, status);
                }
            }
            else
            {
                /* Call the upper level state machine */
                if (pWhalBus->uEEPROMStatus == TNETWIF_PENDING || 
                    pWhalBus->uNVSStatus == TNETWIF_PENDING)
                    whal_FwCtrl_BootSm (hWhalBus, module_id, status);

                return TNETWIF_COMPLETE;
            }
            continue;

        case 4:
            pWhalBus->uNVSStartAddr += 4;
            pWhalBus->uNVSTempWord = 0;
            pWhalBus->uNVSNumByte = 0;
            pWhalBus->uEEPROMStage = 3;
            continue; 
           
        } /* End switch */
 
    } /* End while */

}


/****************************************************************************
 *                      whal_FwCtrl_InitSequenceSm()
 ****************************************************************************
 * DESCRIPTION: the restart wakeup sequence state machine
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static TI_STATUS whal_FwCtrl_InitSequenceSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    static const UINT32 LUT [REF_FREQ_NUM][LUT_PARAM_NUM] = 
    {   /* INTEGER_DIVIDER   FRACTIONAL_DIVIDER   ATTN_BB   ALPHA_BB   STOP_TIME_BB   BB_PLL_LOOP_FILTER */
        {   83,             87381,                  0xB,        5,          0xF00,      3}, /* REF_FREQ_19_2*/
        {   61,             141154,                 0xB,        5,          0x1450,     2}, /* REF_FREQ_26_0*/
        {   41,             174763,                 0xC,        6,          0x2D00,     1}, /* REF_FREQ_38_4*/
        {   40,             0,                      0xC,        6,          0x2EE0,     1}, /* REF_FREQ_40_0*/
        {   47,             162280,                 0xC,        6,          0x2760,     1}  /* REF_FREQ_33_6        */
    };
    
    EXCEPT_I (pWhalBus, status);

    switch (pWhalBus->uInitSeqStage)
    {
    case 0:
        pWhalBus->uInitSeqStage ++;

        WLAN_REPORT_INIT(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, ("Starting INIT sequence\n"));           

        /* Read NVS params */
        status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF,
                                     SCR_PAD6,
                                     &pWhalBus->uScrPad6,
                                     module_id,
                                     (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                     hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 1:
        pWhalBus->uInitSeqStage ++;
        /* Read ELP_CMD */
        status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF,
                                     ELP_CMD,
                                     &pWhalBus->uElpCmd,
                                     module_id,
                                     (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                     hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 2: 
        pWhalBus->uInitSeqStage ++;

        pWhalBus->uRefFreq = pWhalBus->uScrPad6 & 0x000000FF;
    
        /******************** Set ELP configuration *********************/
    
        /*
         * Set the BB Calibration time to be 300 usec (PLL_CAL_TIME)
         */
         status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                       PLL_CAL_TIME/*0x5810*/, 
                                       0x9/*0x4*/,
                                       module_id,
                                       (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                       hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 3:
        pWhalBus->uInitSeqStage ++;

        /* PG 1.1 & 1.0: Set the clock buffer time to be 760 usec (CLK_BUF_TIME) */
        if (pWhalBus->uChipId == CHIP_ID_1251_PG10 ||
            pWhalBus->uChipId == CHIP_ID_1251_PG11)
        {
			status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      CLK_BUF_TIME/*0x5818*/, 
                                      0x19,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
		}
        /* PG 1.2: Set the clock buffer time to be 210 usec (CLK_BUF_TIME) */
		else
		{
			status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      CLK_BUF_TIME/*0x5818*/, 
                                      0x6,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
		}
        EXCEPT_I (pWhalBus, status);

    case 4:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set the clock detect feature to work in the restart wu procedure (ELP_CFG_MODE[14])
         * &
         * Select the clock source type (ELP_CFG_MODE[13:12] )
         */  
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      ELP_CFG_MODE/*0x5804*/, 
                                      ((pWhalBus->uScrPad6 & 0x0000FF00) << 4) | 0x00004000,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 5:
        pWhalBus->uInitSeqStage ++;

        /* PG 1.1 & 1.0 */
        if (pWhalBus->uChipId == CHIP_ID_1251_PG10 ||
            pWhalBus->uChipId == CHIP_ID_1251_PG11)
        {
            /* Do nothing */
        }

        /* PG 1.2: Enable the BB PLL fix. Enable the PLL_LIMP_CLK_EN_CMD */
        else
        {
			pWhalBus->uElpCmd |= 0x00000040;    

            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          ELP_CMD/*0x5808*/, 
                                          pWhalBus->uElpCmd,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                          hWhalBus);
            EXCEPT_I (pWhalBus, status);
        }


    case 6:
        pWhalBus->uInitSeqStage ++;

        /* PG 1.1 & 1.0: set the BB PLL stable time to be 30usec (PLL_STABLE_TIME) */
        if (pWhalBus->uChipId == CHIP_ID_1251_PG10 ||
            pWhalBus->uChipId == CHIP_ID_1251_PG11)
        {
            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          CFG_PLL_SYNC_CNT/*0x5820*/, 
                                          0x00,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                          hWhalBus);
        }

        /* PG 1.2: Set the BB PLL stable time to be 1000usec (PLL_STABLE_TIME) */
        else
        {
            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          CFG_PLL_SYNC_CNT/*0x5820*/, 
                                          0x20,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                          hWhalBus);
        }

        EXCEPT_I (pWhalBus, status);

    case 7:
        pWhalBus->uInitSeqStage ++;

        /* PG 1.1 & 1.0 */
        if (pWhalBus->uChipId == CHIP_ID_1251_PG10 ||
            pWhalBus->uChipId == CHIP_ID_1251_PG11)
        {
            /* Do nothing */
        }

        /* PG 1.2: read clock request time */
        else
        {
            status = TNETWIF_ReadRegOpt (pWhalBus->hTNETWIF, 
                                         CLK_REQ_TIME/*0x5814*/, 
                                         &pWhalBus->uInitData,
                                         module_id,
                                         (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                         hWhalBus);
            EXCEPT_I (pWhalBus, status);
        }

    case 8:
        pWhalBus->uInitSeqStage ++;

        /* PG 1.1 & 1.0 */
        if (pWhalBus->uChipId == CHIP_ID_1251_PG10 ||
            pWhalBus->uChipId == CHIP_ID_1251_PG11)
        {
            /* Do nothing */
        }

        /* PG 1.2: set the clock request time to be [ref_clk_settling_time-1mS] 4ms */
        else
        {
			WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
				("CLK_REQ_TIME: read = 0x%x write = 0x%x\n", 
				pWhalBus->uInitData,((pWhalBus->uInitData > 0x21) ? (pWhalBus->uInitData - 0x21) : 0 )));

            status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                          CLK_REQ_TIME/*0x5814*/, 
                                          ((pWhalBus->uInitData > 0x21) ? (pWhalBus->uInitData - 0x21) : 0 ),
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                          hWhalBus);
            EXCEPT_I (pWhalBus, status);
        }

    case 9:
        pWhalBus->uInitSeqStage ++;

        /******************** Set BB PLL configurations in RF AFE *********************/

        /*
         * Set RF_AFE_REG_3
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x58CC, 
                                      0x4B5, 
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 10:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set RF_AFE_REG_5
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x58D4, 
                                      0x50/*0x150*/,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 11:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set RF_AFE_CTRL_REG_2
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x5948, 
                                      0x11C001,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 12:
        pWhalBus->uInitSeqStage ++;

        /*
         * Change RF PLL and BB PLL divider for VCO clock  and adjust VCO bais current(RF_AFE_REG_13)
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x58F4, 
                                      0x1E,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 13:
        pWhalBus->uInitSeqStage ++;

        /******************** Set BB PLL configurations *********************/
    
        /*
         * Set integer divider according to Appendix C-BB PLL Calculations. 
         * &
         * Set dither scale to 0.
         * &
         * Enable complex zero
         * &
         * Set the location of complex zero
         * &
         * Set the order of the sigma delta to 2nd order
         * &
         *Disable the async load
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x5840, 
                                      LUT[pWhalBus->uRefFreq][LUT_PARAM_INTEGER_DIVIDER] | 0x00017000,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 14:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set fractional divider according to Appendix C-BB PLL Calculations
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x5844, 
                                      LUT[pWhalBus->uRefFreq][LUT_PARAM_FRACTIONAL_DIVIDER],
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 15:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set the initial data for the sigma delta
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x5848, 
                                      0x3039,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 16:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set the accumulator attenuation value
         * &
         * Set calibration loop1 (alpha)
         * &
         * Set calibration loop2 (beta)
         * &
         * Set calibration loop3 (gamma)
         * &
         * Set the VCO gain
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x5854, 
                                      (LUT[pWhalBus->uRefFreq][LUT_PARAM_ATTN_BB] << 16) | (LUT[pWhalBus->uRefFreq][LUT_PARAM_ALPHA_BB] << 12) | 0x1,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 17:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set the calibration stop time after holdoff time expires
         * &
         * Set settling time HOLD_OFF_TIME_BB
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x5858, 
                                      LUT[pWhalBus->uRefFreq][LUT_PARAM_STOP_TIME_BB] | 0x000A0000,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 18:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set BB PLL Loop filter capacitor3- BB_C3[2:0]
         * &
         * Set BB PLL constant leakage current to linearize PFD to 0uA- BB_ILOOPF[7:3]
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x58F8, 
                                      LUT[pWhalBus->uRefFreq][LUT_PARAM_BB_PLL_LOOP_FILTER] | 0x00000030,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 19:
        pWhalBus->uInitSeqStage ++;

        /*
         * Set regulator output voltage for n divider to 1.35- BB_REFDIV[1:0]
         * &
         * Set Charge pump current- BB_CPGAIN[4:2]
         * &
         * Set BB PLL Loop filter capacitor2- BB_C2[7:5]
         * &
         * Set gain of BB PLL auto-call to normal mode- BB_CALGAIN_3DB[8]
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      0x58F0, 
                                      0x29,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 20:
        pWhalBus->uInitSeqStage ++;

        /******************** Enable restart sequence *********************/
    
        /*
         * Enable restart wakeup sequence (ELP_CMD[0])
         */
        status = TNETWIF_WriteRegOpt (pWhalBus->hTNETWIF, 
                                      ELP_CMD/*0x5808*/, 
                                      pWhalBus->uElpCmd | 0x1,
                                      module_id,
                                      (TNETWIF_callback_t)whal_FwCtrl_InitSequenceSm,
                                      hWhalBus);
        EXCEPT_I (pWhalBus, status);

    case 21:      
        pWhalBus->uInitSeqStage = 0;

        os_StalluSec (pWhalBus->hOs, 2000);

        WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, ("End INIT sequence\n"));            

        /* Call upper layer state machine */ 
        if (pWhalBus->uInitSeqStatus == TNETWIF_PENDING)
            whal_FwCtrl_BootSm (hWhalBus, module_id, OK);

    } /* End switch */

    return TNETWIF_COMPLETE;
}


/****************************************************************************
 *                      whal_FwCtrl_LoadFwImageSm()
 ****************************************************************************
 * DESCRIPTION: Load image from the host and download into the hardware 
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/

#define ADDR_OFFS HW_ACCESS_DOWN_PART0_ADDR


static TI_STATUS whal_FwCtrl_LoadFwImageSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    EXCEPT_L (pWhalBus, status);

    while (TRUE)
    {
        switch (pWhalBus->uLoadStage)
        {
        case 0:
            pWhalBus->uLoadStage = 1; 
            /* 
             * Extract and calculate a length of the firmware image
             * Needed to avoid DWORD alignment issues          
             * Get the data length of the firmware image             
             */ 
            pWhalBus->uFwDataLen = (pWhalBus->pFwBuf[4] << 24) |
                                   (pWhalBus->pFwBuf[5] << 16) |
                                   (pWhalBus->pFwBuf[6] << 8 ) |
                                   (pWhalBus->pFwBuf[7]);

            /* Check the data length */
            if ((pWhalBus->uFwDataLen % 4) != 0)
            {
                WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG, ("FW image length\n")); 
            
            }

          #if defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI)
            status = TNETWIF_SetPartitionsOpt (pWhalBus->hTNETWIF, 
                                               HW_ACCESS_DOWNLOAD, 
                                               ADDR_OFFS,
                                               module_id,
                                               (TNETWIF_callback_t)whal_FwCtrl_LoadFwImageSm,
                                               hWhalBus);
            EXCEPT_L (pWhalBus, status);
          #endif
            continue;

        case 1:
            pWhalBus->uLoadStage = 2;
 
            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG,
                              ("Image addr=0x%x, Len=0x%x\n", 
                              pWhalBus->pFwBuf, pWhalBus->uFwLastAddr));
       
            pWhalBus->uChunkNum = 0;
            pWhalBus->uPartitionLimit = HW_ACCESS_DOWN_PART0_SIZE;

            WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG, ("DOWNLOADING !!!\n"));
            continue;
                    
        case 2:

            /* Retrieve the data that was saved for the last chunk */
          #ifdef USE_NO_CHUNK_COPY
            if (pWhalBus->uChunkNum > 0)
                os_memoryCopy (pWhalBus->hOs,
                               (void *)(pWhalBus->pFwBuf + FW_HDR_SIZE + (pWhalBus->uChunkNum - 1) * CHUNK_SIZE - TNETWIF_WRITE_OFFSET_BYTES),
                               (void *)pWhalBus->auFwTmpBuf,
                               TNETWIF_WRITE_OFFSET_BYTES);
          #endif

            /* Load firmware by chunks */
            if (pWhalBus->uChunkNum < pWhalBus->uFwDataLen / CHUNK_SIZE)
            {            
                pWhalBus->uLoadStage = 3;

              #if defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI)
                /* Change partition */
                if (ADDR_OFFS + (pWhalBus->uChunkNum + 2) * CHUNK_SIZE > pWhalBus->uPartitionLimit)
                {                
                    pWhalBus->uPartitionLimit = (ADDR_OFFS + pWhalBus->uChunkNum * CHUNK_SIZE) + HW_ACCESS_DOWN_PART0_SIZE;
                    status = TNETWIF_SetPartitionsOpt (pWhalBus->hTNETWIF, 
                                                       HW_ACCESS_DOWNLOAD, 
                                                       ADDR_OFFS + pWhalBus->uChunkNum * CHUNK_SIZE,
                                                       module_id,
                                                       (TNETWIF_callback_t)whal_FwCtrl_LoadFwImageSm,
                                                       hWhalBus);
                    EXCEPT_L (pWhalBus, status);
                                                     
                    WLAN_REPORT_INIT (pWhalBus->hReport, HAL_CTRL_MODULE_LOG,
                                      ("Change partition ADDR_OFFS = 0x%x\n", 
                                      ADDR_OFFS + pWhalBus->uChunkNum * CHUNK_SIZE));
                }
              #endif
            }
            else
                pWhalBus->uLoadStage = 4;
            continue;

        case 3:        
            pWhalBus->uLoadStage = 2;

            /* Write the data chunk of 512 bytes */

          #ifdef USE_NO_CHUNK_COPY
            /* 
             * Save the chunk trailer bytes in the temporary buffer.
             * The trailer space is used by the WSPI driver 
             */
            os_memoryCopy (pWhalBus->hOs,
                           (void *)pWhalBus->auFwTmpBuf,
                           (void *)(pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE - TNETWIF_WRITE_OFFSET_BYTES),
                           TNETWIF_WRITE_OFFSET_BYTES);
          #else
            /* Copy image chunk to temporary buffer */
            os_memoryCopy (pWhalBus->hOs,
                           (void *)&pWhalBus->auFwTmpBuf[TNETWIF_WRITE_OFFSET_BYTES],
                           (void *)(pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE),
                           CHUNK_SIZE);
          #endif 
            
            /* Load the chunk. Save TNETWIF_WRITE_OFFSET_BYTES space for WSPI bus command */
            status = TNETWIF_WriteMemOpt (pWhalBus->hTNETWIF, 
                                          ADDR_OFFS + pWhalBus->uChunkNum * CHUNK_SIZE,
                                        #ifdef USE_NO_CHUNK_COPY
                                          pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE - TNETWIF_WRITE_OFFSET_BYTES,
                                        #else 
                                          pWhalBus->auFwTmpBuf,
                                        #endif
                                          CHUNK_SIZE,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_LoadFwImageSm,
                                          hWhalBus);

            /* Increment chunk number */
            pWhalBus->uChunkNum ++;

            /* Log ERROR if the TNETWIF_WriteMemOpt returned ERROR */
            if (status == TNETWIF_ERROR)
            {
                WLAN_REPORT_ERROR (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                   ("TNETWIF_WriteMemOpt retruned status=0x%x\n", status));
            } 

            EXCEPT_L (pWhalBus, status);
            continue;

        case 4:    
            pWhalBus->uLoadStage = 5;

          #ifdef USE_NO_CHUNK_COPY
            /* 
             * Save the chunk trailer bytes in the temporary buffer.
             * The trailer space is used by the WSPI driver 
             */
            os_memoryCopy (pWhalBus->hOs,
                           (void *)pWhalBus->auFwTmpBuf,
                           (void *)(pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE - TNETWIF_WRITE_OFFSET_BYTES),
                           TNETWIF_WRITE_OFFSET_BYTES);
          #else
            /* Copy the last image chunk */
            os_memoryCopy (pWhalBus->hOs,
                           (void *)&pWhalBus->auFwTmpBuf[TNETWIF_WRITE_OFFSET_BYTES],
                           (void *)(pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE),
                           pWhalBus->uFwDataLen % CHUNK_SIZE);
          #endif

            /* Load the last chunk */
            status = TNETWIF_WriteMemOpt (pWhalBus->hTNETWIF, 
                                          ADDR_OFFS + pWhalBus->uChunkNum * CHUNK_SIZE,
                                        #ifdef USE_NO_CHUNK_COPY
                                          pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE - TNETWIF_WRITE_OFFSET_BYTES, 
                                        #else
                                          pWhalBus->auFwTmpBuf,
                                        #endif
                                          pWhalBus->uFwDataLen % CHUNK_SIZE,
                                          module_id,
                                          (TNETWIF_callback_t)whal_FwCtrl_LoadFwImageSm,
                                          hWhalBus); 

            EXCEPT_L (pWhalBus, status);
            continue;

        case 5:
            pWhalBus->uLoadStage = 0;

            /* The download has completed */ 
            WLAN_OS_REPORT (("Finished downloading firmware.\n"));

          #ifdef USE_NO_CHUNK_COPY
            /* Retrieve the data that was saved for the last chunk */
            os_memoryCopy (pWhalBus->hOs,
                           (void *)(pWhalBus->pFwBuf + FW_HDR_SIZE + pWhalBus->uChunkNum * CHUNK_SIZE - TNETWIF_WRITE_OFFSET_BYTES),
                           (void *)pWhalBus->auFwTmpBuf,
                           TNETWIF_WRITE_OFFSET_BYTES);
          #endif

            /* Finalize download (run firmware) */
            pWhalBus->uFinStage = 0;
            status = whal_FwCtrl_FinalizeDownloadSm (hWhalBus, module_id, OK);

            return status;

        } /* End switch */

    } /* End while */
        
}


UINT32 whalBus_FwCtrl_GetRadioStandByState(TI_HANDLE hWhalBus)
{
  #ifdef USE_SYNC_API
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    UINT32     data;
    
    return TNETWIF_ReadRegSync (pWhalBus->hTNETWIF,GPIO_IN,&data);
    
  #else

    return 0;

  #endif
}


int whalBus_FwCtrl_Reset(TI_HANDLE hWhalBus)
{   
  #ifdef USE_SYNC_API

    return whal_FwCtrl_Reset (hWhalBus);

  #else

    return OK;

  #endif
}


int whalBus_FwCtrl_isCardIn(TI_HANDLE hWhalBus)
{   
  #ifdef USE_SYNC_API

    /*
    UINT32 ChipId;
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,CHIP_ID,&ChipId)
    ChipId = CHIP_ID_1X50;
    */

  #endif
        
    return TRUE;
}

void whalBus_FwCtrl_Halt(TI_HANDLE hWhalBus)
{
  #ifdef USE_SYNC_API
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    /* Halt the firmware */
    TNETWIF_RegIsBitSet(pWhalBus->hTNETWIF, ACX_REG_ECPU_CONTROL, ECPU_CONTROL_HALT);
  #endif
}
