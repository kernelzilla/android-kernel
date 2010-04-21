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
 *   MODULE:  whalHwMboxCmdBit.c
 *   PURPOSE: wlan hardware BIT commands handler
 * 
 ****************************************************************************/

#include "whalCommon.h"
#include "CmdQueue_api.h"
#include "public_infoele.h"
#include "commonTypes.h"
#include "whalHwMboxCmdBit.h"



/*******************************************
 * Wlan hardware Test (BIT)
 * =================
 *
 * Tests description:
 * ==================
 * FCC           = Continuous modulated transmission (should not emit carrier)
 * TELEC         = Continuous unmodulated carrier transmission (carrier only)
 * PER_TX_STOP   = Stops the TX test in progress (FCC or TELEC).
 * ReadRegister  = Read a register value.
 * WriteRegister = Sets a register value.
* 
* Rx PER test
* ========
* PerRxStart       = Start or resume the PER measurement. This function will put the device in promiscuous mode, and resume counters update.
* PerRxStop        = Stop Rx PER measurements. This function stop counters update and make it is safe to read the PER test result.
* PerRxGetResults  = Get the last Rx PER test results.
* PerRxClear       = Clear the Rx PER test results. 
 */

enum
{
/* 0 */   TEST_MOD_QPSK,
/* 1 */   TEST_MOD_CCK,
/* 2 */   TEST_MOD_PBCC,

   TEST_MOD_NUMOF
};

enum
{
/* 0 */   TEST_MOD_LONG_PREAMBLE,
/* 1 */   TEST_MOD_SHORT_PREAMBLE
};

enum
{
/* 0 */   TEST_BAND_2_4GHZ,
/* 1 */   TEST_BAND_5GHZ,
/* 2 */   TEST_BAND_4_9GHZ
};


#define TEST_MOD_MIN_GAP           200
#define TEST_MOD_MIN_TX_BODYLEN    0
#define TEST_MOD_MAX_TX_BODYLEN    2304

#define TEST_RX_CAL_SAFE_TIME      5000  /*uSec*/

#define TEST_MOD_IS_GAP_OK(gap)     ((gap) >= TEST_MOD_MIN_GAP)

#define TEST_MOD_IS_TX_BODYLEN_OK(len)  \
   (INRANGE((len), TEST_MOD_MIN_TX_BODYLEN, TEST_MOD_MAX_TX_BODYLEN) && \
   (((len) & 3) == 0) )

#define TEST_MOD_IS_PREAMBLE_OK(p)  \
   INRANGE((p), TEST_MOD_LONG_PREAMBLE, TEST_MOD_SHORT_PREAMBLE)


void whal_hwCmdBit_RxPer_Clear_CB(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf);
void whal_hwCmdBit_RxPer_GetResults_CB(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf);
void whal_hwCmdBit_RxCal_CB(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf);
void whal_hwCmdBit_RxCal_Complete_Event_CB( TI_HANDLE objectHandle, char* str, UINT32 strLen );

/****************************************************************************
 *                      whal_hwMboxCmdBit_Create()
 ****************************************************************************
 * DESCRIPTION: Create the mailbox BIT commands object
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
HwMboxCmdBit_T *whal_hwMboxCmdBit_Create(TI_HANDLE hOs)
{
    HwMboxCmdBit_T *pObj;

    pObj = os_memoryAlloc(hOs, sizeof(HwMboxCmdBit_T));
    if (pObj == NULL)
        return NULL;

    os_memoryZero(hOs, (void *)pObj, sizeof(HwMboxCmdBit_T));

    pObj->hOs = hOs;

    return(pObj);
}

/****************************************************************************
 *                      whal_hwMboxCmdBit_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object 
 * 
 * INPUTS:  
 *      pHwMboxCmdBit       The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwMboxCmdBit_Destroy(HwMboxCmdBit_T *pHwMboxCmdBit)
{
    if (pHwMboxCmdBit)
    { 
        whalCtrl_EventMbox_Disable( pHwMboxCmdBit->hWhalCtr, HAL_EVENT_PLT_RX_CALIBRATION_COMPLETE );
        os_memoryFree(pHwMboxCmdBit->hOs, pHwMboxCmdBit, sizeof(HwMboxCmdBit_T));
    }
    return OK;
}

/****************************************************************************
 *                      whal_hwMboxCmdBit_Config()
 ****************************************************************************
 * DESCRIPTION: Configure the object 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwMboxCmdBit_Config(TI_HANDLE hWhalCtr, HwMboxCmdBit_T *pHwMboxCmdBit, TI_HANDLE hCmdQueue, TI_HANDLE hReport)
{
    pHwMboxCmdBit->hReport = hReport;
    pHwMboxCmdBit->hCmdQueue = hCmdQueue;
    pHwMboxCmdBit->hWhalCtr = hWhalCtr;

    whalCtrl_EventMbox_RegisterForEvent( hWhalCtr, 
                                         HAL_EVENT_PLT_RX_CALIBRATION_COMPLETE,
                                         (void *)whal_hwCmdBit_RxCal_Complete_Event_CB, 
                                         (void*)pHwMboxCmdBit);

    whalCtrl_EventMbox_Enable( hWhalCtr, HAL_EVENT_PLT_RX_CALIBRATION_COMPLETE );

    pHwMboxCmdBit->PltData.RxTxCal.lastStatus = OK;
    return OK;
}

/*******************
 Helper functions
 *******************/

static void errPrint_ChID(HwMboxCmdBit_T *pHwMboxCmdBit, int chID)
{
    WLAN_REPORT_REPLY(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("Channel ID (%d) is out of range, use [%d..%d] for 2.4Ghz, [%d..%d] for 5Ghz and [%d %d %d] for 4.9Ghz.\n",
         chID,
         HAL_CTRL_CALIBRATION_CHANNEL_2_4_MIN, HAL_CTRL_CALIBRATION_CHANNEL_2_4_MAX,
         HAL_CTRL_CALIBRATION_CHANNEL_5_0_MIN, HAL_CTRL_CALIBRATION_CHANNEL_5_0_MAX,
         HAL_CTRL_CALIBRATION_CHANNEL_4_9_MIN, HAL_CTRL_CALIBRATION_CHANNEL_4_9_DEF, HAL_CTRL_CALIBRATION_CHANNEL_4_9_MAX));
}

static void errPrint_BandID(HwMboxCmdBit_T *pHwMboxCmdBit, int bandID)
{
        WLAN_REPORT_REPLY(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("PLT whal_hwCmdBit_Fcc: Wrong band parameter 0x%x(must be 0x%x for 2.4Ghz or 0x%x for 5Ghz or 0x%x for 4.9Ghz).\n",
        bandID, TEST_BAND_2_4GHZ, TEST_BAND_5GHZ, TEST_BAND_4_9GHZ));
}

/* MACROS */
int VALIDATE_BAND_CHID(HwMboxCmdBit_T *pHwMboxCmdBit, int chID, int bandID) 
{
   int minVal, maxVal;
   switch(bandID)
   {
   case TEST_BAND_2_4GHZ:
       minVal = HAL_CTRL_CALIBRATION_CHANNEL_2_4_MIN;
       maxVal = HAL_CTRL_CALIBRATION_CHANNEL_2_4_MAX;
    break;                                           

   case TEST_BAND_5GHZ:
       minVal = HAL_CTRL_CALIBRATION_CHANNEL_5_0_MIN;
       maxVal = HAL_CTRL_CALIBRATION_CHANNEL_5_0_MAX;
       break;

    case TEST_BAND_4_9GHZ:
       minVal = HAL_CTRL_CALIBRATION_CHANNEL_4_9_MIN;
       maxVal = HAL_CTRL_CALIBRATION_CHANNEL_4_9_MAX;
       break;

   default:
       errPrint_BandID(pHwMboxCmdBit, bandID);
       return NOK;
   }
   if(!INRANGE(chID, minVal, maxVal))
   {
      errPrint_ChID(pHwMboxCmdBit, chID);
      return NOK;
   }
   return(OK);
}

/****************************************************************************
 *                       whal_hwCmdBit_Fcc()
 ****************************************************************************
 * DESCRIPTION: Performs FCC test
 * 
 * INPUTS: chID, rate, Modulation, preamble, band, TestMode
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCmdBit_Fcc(HwMboxCmdBit_T *pHwMboxCmdBit,
                      int chID, 
                      int rate, 
                      int preamble, 
                      int bandID,
                      int InterPacketDelay,
                      int TestMode,
                      uint32 numFrames,
                      uint32 seqNumMode,
                      uint32 frameBodySize,
                      uint8 *PeerMacAddr,
                      void *CB_Func, TI_HANDLE CB_Handle, void *CB_Buf)
{
    int Modulation;
    TestCmd_t TestCmd;
    os_memoryZero(pHwMboxCmdBit->hOs, (void *)&TestCmd, sizeof(TestCmd));
    
    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
        ("whal_hwCmdBit_Fcc:\n chID = %d\n rate = %d preamble = %d, bandID %d, InterPacketDelay %d, TestMode %d numOfFrames %d\n ", 
             chID,  rate,  preamble, 
             bandID,  InterPacketDelay,  TestMode, numFrames));

    /*Check Channel and band*/
    if (VALIDATE_BAND_CHID(pHwMboxCmdBit, chID, bandID) == NOK)
        return NOK;

    /*Check rate and set Modulation*/
    if ((rate >= 1) && (rate <= 4))
        Modulation = MOD_CCK;
    else if ((rate >= 6) && (rate <= 13))
        Modulation =  MOD_OFDM;
    else
    {
        WLAN_REPORT_REPLY(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("PLT whal_hwCmdBit_Fcc: Wrong rate parameter %d(must be 1-4 to CCK modulation  or 6-13 for OFDM modulation) .\n",
            rate));
        return NOK;
    }

     /*Set FW rate */
    TestCmd.testCmd_u.fcc.dataRate = ConvertDrvRate2HwRate((rate_e)rate);

    /*Validate preamble*/
    if ((preamble!=0) && preamble != CTL_PREAMBLE)
    {
        WLAN_REPORT_REPLY(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("PLT whal_hwCmdBit_Fcc: Wrong preamble parameter 0x%x(must be (0x%x) for long preamble or short(0x%x)) .\n",
        preamble, 0, CTL_PREAMBLE));

        return NOK;
    } 

    /*Validate test mode*/
    if ((TestMode != TEST_MODE_ZOZO_DATA) && (TestMode != TEST_MODE_RANDOM_DATA))
    {
        WLAN_REPORT_REPLY(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("PLT whal_hwCmdBit_Fcc: Wrong TestMode parameter 0x%x(must be RANDOM_DATA(0x%x) or ZOZO_DATA(0x%x)) .\n",
        TestMode, TEST_MODE_RANDOM_DATA, TEST_MODE_ZOZO_DATA));
        /*Not returning NOX for enabling GAP bit*/
    }

        /*Validate seq num mode*/
    if ((seqNumMode != TEST_SEQ_NUM_MODE_FIXED) && (seqNumMode != TEST_SEQ_NUM_MODE_INCREMENTED))
    {
        WLAN_REPORT_REPLY(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("PLT whal_hwCmdBit_Fcc: Wrong seqNumMode parameter 0x%x(must be TEST_SEQ_NUM_MODE_FIXED(0x%x) or TEST_SEQ_NUM_MODE_INCREMENTED(0x%x)) .\n",
        seqNumMode, TEST_SEQ_NUM_MODE_FIXED, TEST_SEQ_NUM_MODE_INCREMENTED));
        
        return NOK;
    }

    TestCmd.testCmd_u.fcc.seqNumMode = seqNumMode;
    TestCmd.testCmd_u.fcc.frameBodySize = frameBodySize;
    os_memoryCopy(pHwMboxCmdBit->hOs , TestCmd.testCmd_u.fcc.dest, PeerMacAddr,NUM_OF_MAC_ADDR_ELEMENTS);

    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
        ("whal_hwCmdBit_Fcc: \n seqNumMode = %d ,frameBodySize = 0x%x", 
             TestCmd.testCmd_u.fcc.seqNumMode, 
             TestCmd.testCmd_u.fcc.frameBodySize));

    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
        ("whal_hwCmdBit_Fcc: \n Dest = %02x:%02x:%02x:%02x:%02x:%02x", 
             TestCmd.testCmd_u.fcc.dest[0],
             TestCmd.testCmd_u.fcc.dest[1],
             TestCmd.testCmd_u.fcc.dest[2],
             TestCmd.testCmd_u.fcc.dest[3],
             TestCmd.testCmd_u.fcc.dest[4],
             TestCmd.testCmd_u.fcc.dest[5]));

    TestCmd.testCmd_u.fcc.channel = chID;

    switch (Modulation)
    {
    case MOD_PBCC:
        Modulation = PBCC_MODULATION_MASK;
        break;

    case MOD_CCK:
        Modulation = 0x00;
        break;

    case MOD_OFDM:
        Modulation = OFDM_MODULATION_MASK;
        break;
    }
    /*Build preamble parameter*/
    TestCmd.testCmd_u.fcc.modPreamble = preamble;
    TestCmd.testCmd_u.fcc.band = bandID;
    TestCmd.testCmd_u.fcc.modulation = Modulation;

    TestCmd.testCmd_u.fcc.testModeCtrl = TestMode;

    /* Set command fields */
    TestCmd.testCmdId = TEST_CMD_FCC; 

    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
        ("whal_hwCmdBit_Fcc: \n AcxCmd_TestFCC.channel = %d ,AcxCmd_TestFCC.dataRate = 0x%x,  AcxCmd_TestFCC.modPreamble = 0x%x, AcxCmd_TestFCC.testModeCtrl = 0x%x\n ", 
             TestCmd.testCmd_u.fcc.channel, 
             TestCmd.testCmd_u.fcc.dataRate,
             TestCmd.testCmd_u.fcc.modPreamble,
             TestCmd.testCmd_u.fcc.testModeCtrl));
    

    /*Set InterPacketDelay*/
    /*TestCmd.testCmd_u.fcc.interFrameGap = 1000 * InterPacketDelay; */ /*Convert ms to us*/
    TestCmd.testCmd_u.fcc.interFrameGap = InterPacketDelay; /*(uSec)*/

    /*Set numFrames */
    TestCmd.testCmd_u.fcc.numFrames = numFrames;

    return whal_hwCmdBit_TestCmd(pHwMboxCmdBit, CB_Func, CB_Handle, &TestCmd);
}/* END whal_hwCmdBit_Fcc() */



/****************************************************************************
 *                      whal_hwCmdBit_Telec()
 ****************************************************************************
 * DESCRIPTION: Performs TELEC test
 * 
 * INPUTS: chID   - Channel number, between  1 (MIN_CHANNEL_ID) to  14 (MAX_CHANNEL_ID).
 *         bandID - Band ID number, 0 - 2.4Ghz, 1 - 5Ghz.  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCmdBit_Telec(HwMboxCmdBit_T *pHwMboxCmdBit, int chID, int bandID, void *CB_Func, TI_HANDLE CB_Handle, void *CB_Buf)
{
    TestCmd_t   TestCmd;
    TestCmd_t  *pCmd = &TestCmd;
    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
         ("whal_hwCmdBit_Telec: chID = %d bandID = %d\n ", chID, bandID));


    /*Check Channel and band*/
    if (VALIDATE_BAND_CHID(pHwMboxCmdBit, chID, bandID) == NOK)
        return NOK;

    /* Set command fields; param1 = chID, param2 = BandID */
    os_memoryZero(pHwMboxCmdBit->hOs, (void *)pCmd, sizeof(*pCmd ));
    pCmd->testCmdId = TEST_CMD_TELEC;
    pCmd->testCmd_u.telec.channel = (UINT8)chID;
    pCmd->testCmd_u.telec.band = (UINT8)bandID;
    
    /* Send the command */ 
    return whal_hwCmdBit_TestCmd(pHwMboxCmdBit, CB_Func, CB_Handle, &TestCmd);
}/* END whal_hwCmdBit_Telec() */

/**************************
    REGISTER HANDLING
 *************************/

/****************************************************************************
 *                      whal_hwCmdBit_ReadRegister()
 ****************************************************************************
 * DESCRIPTION: Performs PLT read register
 * 
 * INPUTS: RegAddress - the address of the register to read
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK       
 ****************************************************************************/
 int whal_hwCmdBit_ReadRegister(HwMboxCmdBit_T *pHwMboxCmdBit, TI_HANDLE CB_Handle, void *CB_Func, void *CB_Buf)
{        
    ReadWriteCommand_t* pCmd = CB_Buf;

     /* Send the command */  
     return (CmdQueue_CommandWithCb(pHwMboxCmdBit->hCmdQueue, CMD_READ_MEMORY, (char *)pCmd, sizeof(*pCmd), CB_Func, CB_Handle, CB_Buf));         
}/* END whal_hwCmdBit_ReadRegister() */


/****************************************************************************
 *                      whal_hwCmdBit_WriteRegister()
 ****************************************************************************
 * DESCRIPTION: Performs PLT write register
 * 
 * INPUTS:  RegAddress - the address of the register to write to
 *          RegData - the data to write
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
 int whal_hwCmdBit_WriteRegister(HwMboxCmdBit_T *pHwMboxCmdBit, TI_HANDLE CB_Handle, void *CB_Func, void *Command_Buf)
{   
     
     
     /* Send the command */    
     if (CB_Func)
        return (CmdQueue_CommandWithCb(pHwMboxCmdBit->hCmdQueue, CMD_WRITE_MEMORY, (char *)Command_Buf, sizeof(ReadWriteCommand_t), CB_Func, CB_Handle, NULL));    
     else
        return (CmdQueue_Command(pHwMboxCmdBit->hCmdQueue, CMD_WRITE_MEMORY, (char *)Command_Buf, sizeof(ReadWriteCommand_t)));
}/* END whal_hwCmdBit_WriteRegister() */


/****************************************************************************
 *                      whal_hwCmdBit_perTxStop()
 ****************************************************************************
 * DESCRIPTION:   Packet Error Rate - stop test, no params.
 *                Does not clear statistics.
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCmdBit_perTxStop(HwMboxCmdBit_T *pHwMboxCmdBit, void *CB_Func, TI_HANDLE CB_Handle, void *CB_Buf)
{
    TestCmd_t   TestCmd;
    TestCmd_t  *pCmd = &TestCmd;

    /* Set command fields*/
    os_memoryZero(pHwMboxCmdBit->hOs, (void *)pCmd,  sizeof(*pCmd ));
    pCmd->testCmdId = TEST_CMD_PLT_FCC_TELEC_TX_STOP;
    
    /* Send the command  */
    return whal_hwCmdBit_TestCmd(pHwMboxCmdBit, CB_Func, CB_Handle, pCmd);
}/* END whal_hwCmdBit_perTxStop() */
 
 
 
 
  /****************************************************************************
  *                      whal_hwCmdBit_RxPER()
  ****************************************************************************
  * DESCRIPTION:   RX PER main function - All other RX per function are called using this function
  * INPUTS: None 
  * 
  * OUTPUT:  None
  * 
  * RETURNS: OK or NOK
 ****************************************************************************/
 
 int whal_hwCmdBit_RxPER(HwMboxCmdBit_T *pHwMboxCmdBit, PLT_RxPerCmd_e eRxPerCmd, TI_HANDLE CB_Handle, void *CB_Func)
 {     
     int ret = OK;     
     ACXErrorCounters_t ACXErrorCounters;
     TestCmd_t   TestCmd;
     TestCmd_t  *pCmd = &TestCmd;



     /*Defining a local function that will support the Rx PER process 
     in a sync mode (for the core) as well as Async mode (for GWSI, as callbacks)*/
     
     CmdQueue_InterrogateCB_t PltRxPerCBArr[PLT_RX_PER_MAX] = 
     {
         NULL,                              //PLT_RX_PER_START
         NULL,                              //PLT_RX_PER_STOP
         whal_hwCmdBit_RxPer_Clear_CB,      //PLT_RX_PER_CLEAR
         whal_hwCmdBit_RxPer_GetResults_CB  //PLT_RX_PER_GETRESULTS
     };

    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
         ("whal_hwCmdBit_RxPER eRxPerCmd=%d, CB_Handle=0x%p,  CB_Func=0x%p\n", eRxPerCmd, CB_Handle, CB_Func));

     
     /* 
     * Set information element header
     * Each call to ACX_ERROR_CNT - retrieve the F.W Rx Per and reset them.
     */
     ACXErrorCounters.EleHdr.id  = ACX_ERROR_CNT;
     ACXErrorCounters.EleHdr.len = sizeof(ACXErrorCounters) - sizeof(EleHdrStruct);
     
     if ((UINT32)eRxPerCmd >=  (UINT32)PLT_RX_PER_MAX)
         return PARAM_VALUE_NOT_VALID;
     
     pHwMboxCmdBit->PltData.RxPer.CB_Func = CB_Func;
     pHwMboxCmdBit->PltData.RxPer.CB_Handle = CB_Handle;
     pHwMboxCmdBit->PltData.RxPer.CB_RxPerCmd = eRxPerCmd;
     
     /* 
     * Send the interrogation command
     */
     if((PLT_RX_PER_GETRESULTS == eRxPerCmd) || (PLT_RX_PER_CLEAR == eRxPerCmd))
     {     
     ret = CmdQueue_CmdInterrogateWithCb(pHwMboxCmdBit->hCmdQueue,
         &ACXErrorCounters, sizeof(ACXErrorCounters), 
         (void *) PltRxPerCBArr[eRxPerCmd], pHwMboxCmdBit,
         &pHwMboxCmdBit->PltData.RxPer.ACXErrCountTable);
     return ret;
 }

     /* Set command fields*/
     os_memoryZero(pHwMboxCmdBit->hOs, (void *)pCmd,  sizeof(*pCmd ));

     if(PLT_RX_PER_START == eRxPerCmd)
     {
        pCmd->testCmdId = TEST_CMD_RX_PER_START;
        /* Send the command  */
        if (CB_Func != NULL) {
            return (CmdQueue_CommandWithCb(pHwMboxCmdBit->hCmdQueue, CMD_TEST, (char *)pCmd, sizeof(*pCmd),CB_Func , CB_Handle, NULL));
        }
        else
        {
            return (CmdQueue_Command(pHwMboxCmdBit->hCmdQueue, CMD_TEST, (char *)pCmd, sizeof(*pCmd) ));
        }
     }

     if(PLT_RX_PER_STOP == eRxPerCmd)
     {
        pCmd->testCmdId = TEST_CMD_RX_PER_STOP;
        /* Send the command  */
        if (CB_Func != NULL) {
           return (CmdQueue_CommandWithCb(pHwMboxCmdBit->hCmdQueue, CMD_TEST, (char *)pCmd, sizeof(*pCmd),CB_Func, CB_Handle, NULL));
        }
        else
        {
            return (CmdQueue_Command(pHwMboxCmdBit->hCmdQueue, CMD_TEST, (char *)pCmd, sizeof(*pCmd) ));
        }
     }

     return NOK;
 }
 
 /****************************************************************************
 *                      whal_hwCmdBit_RxPer_Clear_CB()
 ****************************************************************************
 * DESCRIPTION:   Clear the RX per counters
 *.
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
 void whal_hwCmdBit_RxPer_Clear_CB(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf)
 {
     HwMboxCmdBit_T *pHwMboxCmdBit = (HwMboxCmdBit_T*)objectHandle;
     CmdQueue_InterrogateCB_t CB_Func;
     
     /*Add the latest F.W. counter result to the total RX per counters*/
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.FCSErrorCount   = 0;
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.TotalFrameCount = 0; 
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.PLCPErrorCount  = 0; 
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.SeqNumMissCountRef = pHwMboxCmdBit->PltData.RxPer.ACXErrCountTable.seqNumMissCount; /* set as reference point for the mesurements */

     
     /* Call the saved CB function (if any)*/
     if (pHwMboxCmdBit->PltData.RxPer.CB_Func)
     {
         CB_Func = (CmdQueue_InterrogateCB_t)(pHwMboxCmdBit->PltData.RxPer.CB_Func);
         CB_Func(pHwMboxCmdBit->PltData.RxPer.CB_Handle, MboxStatus, &pHwMboxCmdBit->PltData.RxPer);
     }
 }
 
 
 /****************************************************************************
 *                      whal_hwCmdBit_RxPer_GetResults_CB()
 ****************************************************************************
 * DESCRIPTION:   Returns the accumulated counters.
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
  void whal_hwCmdBit_RxPer_GetResults_CB(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf)
 {
     HwMboxCmdBit_T *pHwMboxCmdBit = (HwMboxCmdBit_T*)objectHandle;
     CmdQueue_InterrogateCB_t CB_Func;
     
         
     /*Accumulate the RX PER counters */
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.FCSErrorCount   += (UINT16)pHwMboxCmdBit->PltData.RxPer.ACXErrCountTable.FCSErrorCount;
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.PLCPErrorCount  += (UINT16)pHwMboxCmdBit->PltData.RxPer.ACXErrCountTable.PLCPErrorCount;
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.TotalFrameCount += (UINT16)pHwMboxCmdBit->PltData.RxPer.ACXErrCountTable.validFrameCount;
     pHwMboxCmdBit->PltData.RxPer.PltRxPer.SeqNumMissCount  = (UINT16)(pHwMboxCmdBit->PltData.RxPer.ACXErrCountTable.seqNumMissCount - 
                                                              pHwMboxCmdBit->PltData.RxPer.PltRxPer.SeqNumMissCountRef);          


     /* Call the saved CB function (if any)*/
     if (pHwMboxCmdBit->PltData.RxPer.CB_Func)
     {
         CB_Func = (CmdQueue_InterrogateCB_t)(pHwMboxCmdBit->PltData.RxPer.CB_Func);
         CB_Func(pHwMboxCmdBit->PltData.RxPer.CB_Handle, MboxStatus, &(pHwMboxCmdBit->PltData.RxPer));
     }
 }
 
 

/****************************************************************************
 *                      whal_hwCmdBit_TestCmd()
 ****************************************************************************
 * DESCRIPTION:   
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCmdBit_TestCmd(HwMboxCmdBit_T *pHwMboxCmdBit, void *CB_Func, TI_HANDLE CB_Handle, TestCmd_t* pTestCmd_Buf)
{
     int bIsCBfuncNecessary=TRUE;

     WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
     ("%s CB_Handle=0x%p, CB_Func=0x%p, pTestCmd_Buf=%p\n",__FUNCTION__, CB_Handle, CB_Func, pTestCmd_Buf));

     if (NULL == pTestCmd_Buf)
     {
         WLAN_REPORT_ERROR(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
         ("%s pTestCmd_Buf = NULL!!!\n",__FUNCTION__));

         return NOK;
     }

     WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
     ("%s pTestCmd_Buf->testCmdId=%d\n",__FUNCTION__, pTestCmd_Buf->testCmdId));

     WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
     ("%s pTestCmd_Buf (HEX DUMP):\n",__FUNCTION__)); 
     WLAN_REPORT_HEX_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
         (PUINT8)pTestCmd_Buf, sizeof(TestCmd_t));


     switch((TestCmdID_enum)pTestCmd_Buf->testCmdId)
     {
     case TEST_CMD_FCC:                          
     case TEST_CMD_TELEC:                       
     case TEST_CMD_PLT_FCC_TELEC_TX_STOP:
     case TEST_CMD_RADIO_TUNE:
        bIsCBfuncNecessary = FALSE;
        break;

     case TEST_CMD_PLT_GAIN_ADJUST:         
     case TEST_CMD_PLT_TXPOWER_CAL_START:       
     case TEST_CMD_PLT_TXPOWER_CAL_STOP:        
     case TEST_CMD_PLT_GAIN_GET:                 
     case TEST_CMD_PLT_GET_NVS_UPDATE_BUFFER:    
         bIsCBfuncNecessary = TRUE; 
         break;

     case TEST_CMD_PLT_RX_CALIBRATION:
         /* Mark that a calibration operation is pending */
         pHwMboxCmdBit->PltData.RxTxCal.lastStatus = PENDING;
         /* Save the CB for the command response (GWSI oriented) */
         pHwMboxCmdBit->PltData.RxTxCal.CB_Func = CB_Func;
         pHwMboxCmdBit->PltData.RxTxCal.CB_Handle = CB_Handle;

         CB_Func = (void*)whal_hwCmdBit_RxCal_CB;
         CB_Handle = (TI_HANDLE)pHwMboxCmdBit;
         bIsCBfuncNecessary = TRUE; 
         break;

     default:
     WLAN_REPORT_WARNING(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
     ("%s Unsupported TestCmdId (%d)\n",__FUNCTION__, pTestCmd_Buf->testCmdId));
         break;
     }

    /* Send the command */
    if (CB_Func)
    {
         return (CmdQueue_CommandWithCb(pHwMboxCmdBit->hCmdQueue, CMD_TEST, (char *)pTestCmd_Buf, sizeof(*pTestCmd_Buf),
                                        CB_Func, CB_Handle, (void*)pTestCmd_Buf));    
    }
    else
    {
        if (bIsCBfuncNecessary)
            return NOK;
        else
            return (CmdQueue_Command(pHwMboxCmdBit->hCmdQueue, CMD_TEST, (char *)pTestCmd_Buf, sizeof(*pTestCmd_Buf) ));    
    }
     
}/* END whal_hwCmdBit_TestCmd() */


/****************************************************************************
 *                      whal_hwCmdBit_RxCal_CB()
 ****************************************************************************
 * DESCRIPTION:   Returns the accumulated counters.
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
void whal_hwCmdBit_RxCal_CB(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf)
{
    HwMboxCmdBit_T *pHwMboxCmdBit = (HwMboxCmdBit_T*)objectHandle;
    CmdQueue_InterrogateCB_t CB_Func;

    WLAN_REPORT_INFORMATION(pHwMboxCmdBit->hReport, HAL_HW_CTRL_MODULE_LOG, 
        ("%s MboxStatus = 0x%x\n",
       __FUNCTION__, MboxStatus));

    /* If there was an error in the RX Calibration command, mark the NOK status here rather than
        from the RX calibration complete event callback function (the event won't be sent)*/
    if (OK != MboxStatus)
    {
        pHwMboxCmdBit->PltData.RxTxCal.lastStatus = NOK;
    }

    /* Call the saved CB function (if any)*/
    if (pHwMboxCmdBit->PltData.RxTxCal.CB_Func)
    {
        CB_Func = (CmdQueue_InterrogateCB_t)(pHwMboxCmdBit->PltData.RxTxCal.CB_Func);
        CB_Func(pHwMboxCmdBit->PltData.RxTxCal.CB_Handle, MboxStatus, &pHwMboxCmdBit->PltData.RxTxCal);
    }

}
 

 /****************************************************************************
 *                      whal_hwCmdBit_RxCal_Complete_Event_CB()
 ****************************************************************************
 * DESCRIPTION:   Returns the accumulated counters.
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
void whal_hwCmdBit_RxCal_Complete_Event_CB( TI_HANDLE objectHandle, char* str, UINT32 strLen )
{
    HwMboxCmdBit_T *pHwMboxCmdBit = (HwMboxCmdBit_T*)objectHandle;

    /* mark the status as completed */
    pHwMboxCmdBit->PltData.RxTxCal.lastStatus = OK;
}

 /****************************************************************************
 *                      whal_hwCmdBit_GetPltRxCalibrationStatus()
 ****************************************************************************
 * DESCRIPTION:   Returns whether the last RX calibration is pending.
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void whal_hwCmdBit_GetPltRxCalibrationStatus( TI_HANDLE objectHandle, TI_STATUS* pLastStatus  )
{
    HwMboxCmdBit_T *pHwMboxCmdBit = (HwMboxCmdBit_T*)objectHandle;

    /* return the status of the last RX calibration */
    *pLastStatus = pHwMboxCmdBit->PltData.RxTxCal.lastStatus;
}
