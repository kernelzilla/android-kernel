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
 *   MODULE:  shmDebug.c
 *   PURPOSE: Handle Debug requests
 *
 ****************************************************************************/
#include "whalCommon.h"
#include "whalBus_Api.h"
#include "shmBus.h"
#include "TNETWIF.h"


#ifdef USE_SYNC_API

/****************************************************************************
 *                      shmDebug_registerDump()
 ****************************************************************************
 * DESCRIPTION:
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: void
 *
 * NOTES:
 ****************************************************************************/
void shmDebug_registerDump(TI_HANDLE hWhalBus)
{
	shmDebug_macRegisterDump(hWhalBus);
	shmDebug_phyRegisterDump(hWhalBus);
}

void shmDebug_phyRegisterDump(TI_HANDLE hWhalBus)
{
	UINT32 regIndex;
	int regInterval;
	UINT32  RegValue;
	UINT32  phyRegMap[56] = {
		0x0800, 0x0807, 0x1000, 0x100c, 0x1010, 0x1019, 0x1020, 0x1032,
		0x1040, 0x104c, 0x1050, 0x1077, 0x1201, 0x1334, 0x1400, 0x143f,
		0x1500, 0x153F, 0x1800, 0x1802, 0x1c00, 0x1cff, 0x1e00, 0x1eff,
		0x2000, 0x20ff, 0x2840, 0x28b3, 0x2a00, 0x2a02, 0x2c01, 0x2c02,
		0x2c10, 0x2c6f, 0x2d00, 0x2f3f, 0x3000, 0x30dd, 0x3100, 0x311f,
		0x3200, 0x321f, 0x3030, 0x303f, 0x4800, 0x4bb8, 0x4bc0, 0x4bd7,
		0x4be1, 0x4bed, 0x4bf0, 0x4d7f, 0x4e00, 0x4fe0, 0x4ff0, 0x4fff
	};

	WLAN_OS_REPORT(("shmDebug_registerDump : Dumping PHY registers \n"));
	for (regInterval = 0; regInterval < 56; regInterval += 2)
	{
		WLAN_OS_REPORT(("shmDebug_registerDump : Dumping PHY registers from 0x%x to 0x%x\n",
						phyRegMap[regInterval], phyRegMap[regInterval+1]));
		for (regIndex = phyRegMap[regInterval]; regIndex <= phyRegMap[regInterval+1]; regIndex++)
		{
#if defined(TNETW1150)
			RegValue = whalBus_PhyRegRead(hWhalBus, regIndex+0x3c0000);
#else
			RegValue = whalBus_PhyRegRead(hWhalBus, regIndex);
#endif
			WLAN_OS_REPORT(("Phy addr = 0x%x, Phy data = 0x%x\n", regIndex, RegValue));
		}
	}

	WLAN_OS_REPORT(("shmDebug_registerDump : FINISHED Dumping PHY registers \n"));
}

#if defined (TNETW1150)

void shmDebug_macRegisterDump(TI_HANDLE hWhalBus)
{
	WLAN_OS_REPORT(("shmDebug_macRegisterDump : Not implemented in TNETW1150 \n"));
}

#else

#define START_REG_ADDR	0x0
#define LAST_REG_ADDR	0xa80

void shmDebug_macRegisterDump(TI_HANDLE hWhalBus)
{
	whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

	UINT32 regIndex;
	UINT32  RegValue;


	WLAN_OS_REPORT(("shmDebug_registerDump : Dumping MAC registers \n"));

	for (regIndex = START_REG_ADDR; regIndex < LAST_REG_ADDR; regIndex += 4)
	{
		RegValue = whalBus_MacRegRead(hWhalBus, regIndex);
		WLAN_OS_REPORT(("Mac addr = 0x%x, Mac data = 0x%x\n", regIndex, RegValue));
	}
}

#endif

int shmDebug_PrintRxRegs(TI_HANDLE hWhalBus)
{
	whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("shmDebug_PrintRxRegs ----MAC--------------\n"));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_FRM_CNT      (0x%08X) = 0x%08X, 0x%08X\n", RX_FRM_CNT,
         whalBus_MacRegRead(hWhalBus, RX_FRM_CNT), 
			whalBus_MacRegRead(hWhalBus, RX_FRM_CNT)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("CONS_FCS_ERR_CNT(0x%08X) = 0x%08X, 0x%08X\n", CONS_FCS_ERR_CNT,
			whalBus_MacRegRead(hWhalBus, CONS_FCS_ERR_CNT), 
			whalBus_MacRegRead(hWhalBus, CONS_FCS_ERR_CNT)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("FCS_ERR_CNT     (0x%08X) = 0x%08X, 0x%08X\n", FCS_ERR_CNT,
			whalBus_MacRegRead(hWhalBus, FCS_ERR_CNT), 
			whalBus_MacRegRead(hWhalBus, FCS_ERR_CNT)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PLCP_ERR_CNT    (0x%08X) = 0x%08X, 0x%08X\n", PLCP_ERR_CNT,
			whalBus_MacRegRead(hWhalBus, PLCP_ERR_CNT), 
			whalBus_MacRegRead(hWhalBus, PLCP_ERR_CNT)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_FRM_PTR      (0x%08X) = 0x%08X, 0x%08X\n", RX_FRM_PTR,
			whalBus_MacRegRead(hWhalBus, RX_FRM_PTR), 
			whalBus_MacRegRead(hWhalBus, RX_FRM_PTR)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_FRM_CTL      (0x%08X) = 0x%08X, 0x%08X\n", RX_FRM_CTL,
			whalBus_MacRegRead(hWhalBus, RX_FRM_CTL), 
			whalBus_MacRegRead(hWhalBus, RX_FRM_CTL)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_SEQ_CTL      (0x%08X) = 0x%08X, 0x%08X\n", RX_SEQ_CTL,
			whalBus_MacRegRead(hWhalBus, RX_SEQ_CTL), 
			whalBus_MacRegRead(hWhalBus, RX_SEQ_CTL)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_STATUS       (0x%08X) = 0x%08X, 0x%08X\n", RX_STATUS,
			whalBus_MacRegRead(hWhalBus, RX_STATUS), 
			whalBus_MacRegRead(hWhalBus, RX_STATUS)));

#if defined (TNETW1150)
	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_HEAD_PTR     (0x%08X) = 0x%08X, 0x%08X\n", RX_HEAD_PTR,
			whalBus_MacRegRead(hWhalBus, RX_HEAD_PTR), 
			whalBus_MacRegRead(hWhalBus, RX_HEAD_PTR)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_TAIL_PTR     (0x%08X) = 0x%08X, 0x%08X\n", RX_TAIL_PTR,
			whalBus_MacRegRead(hWhalBus, RX_TAIL_PTR), 
			whalBus_MacRegRead(hWhalBus, RX_TAIL_PTR)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("RX_CURR_PTR     (0x%08X) = 0x%08X, 0x%08X\n", RX_CURR_PTR,
			whalBus_MacRegRead(hWhalBus, RX_CURR_PTR), 
			whalBus_MacRegRead(hWhalBus, RX_CURR_PTR)));
#endif

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("shmDebug_PrintRxRegs ----PHY--------------\n"));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_11A_HDR_ERR (0x%08X) = 0x%08X, 0x%08X\n", 0x3c400a,
			whalBus_PhyRegRead(hWhalBus, 0x3c400a), 
			whalBus_PhyRegRead(hWhalBus, 0x3c400a)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_FSM_STATUS  (0x%08X) = 0x%08X, 0x%08X\n", 0x3c4017,
			whalBus_PhyRegRead(hWhalBus, 0x3c4017), 
			whalBus_PhyRegRead(hWhalBus, 0x3c4017)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_BAD_PKT_CNT (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2010,
			whalBus_PhyRegRead(hWhalBus, 0x3c2010), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2010)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_RXT_CFG     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2002,
			whalBus_PhyRegRead(hWhalBus, 0x3c2002), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2002)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_LS_RAM_OVR  (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2020,
			whalBus_PhyRegRead(hWhalBus, 0x3c2020), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2020)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_LS_FFT_ERR  (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2027,
			whalBus_PhyRegRead(hWhalBus, 0x3c2027), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2027)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_DATA_STATE  (0x%08X) = 0x%08X, 0x%08X\n", 0x3c201f,
			whalBus_PhyRegRead(hWhalBus, 0x3c201f), 
			whalBus_PhyRegRead(hWhalBus, 0x3c201f)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_SEQ_CFG     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c301a,
			whalBus_PhyRegRead(hWhalBus, 0x3c301a), 
			whalBus_PhyRegRead(hWhalBus, 0x3c301a)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_SYM_NUM     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2805,
			whalBus_PhyRegRead(hWhalBus, 0x3c2805), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2805)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_RUNBUSY     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2800,
			whalBus_PhyRegRead(hWhalBus, 0x3c2800), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2800)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PTH_ACC_0   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2f00,
			whalBus_PhyRegRead(hWhalBus, 0x3c2f00), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2f00)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PTH_ACC_1   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2f01,
			whalBus_PhyRegRead(hWhalBus, 0x3c2f01), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2f01)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PTH_ACC_2   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2f02,
			whalBus_PhyRegRead(hWhalBus, 0x3c2f02), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2f02)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PTH_ACC_3   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2f03,
			whalBus_PhyRegRead(hWhalBus, 0x3c2f03), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2f03)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PTH_ACC_4   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2f04,
			whalBus_PhyRegRead(hWhalBus, 0x3c2f04), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2f04)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PLCP_SIGNAL (0x%08X) = 0x%08X, 0x%08X\n", 0x3c4002,
			whalBus_PhyRegRead(hWhalBus, 0x3c4002), 
			whalBus_PhyRegRead(hWhalBus, 0x3c4002)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_PLCP_LENGTH (0x%08X) = 0x%08X, 0x%08X\n", 0x3c4003,
			whalBus_PhyRegRead(hWhalBus, 0x3c4003), 
			whalBus_PhyRegRead(hWhalBus, 0x3c4003)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_11B_HDR     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c400b,
			whalBus_PhyRegRead(hWhalBus, 0x3c400b), 
			whalBus_PhyRegRead(hWhalBus, 0x3c400b)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_TRANS_LENGTH(0x%08X) = 0x%08X, 0x%08X\n", 0x3c4018,
			whalBus_PhyRegRead(hWhalBus, 0x3c4018), 
			whalBus_PhyRegRead(hWhalBus, 0x3c4018)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_FFT_START   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c1802,
			whalBus_PhyRegRead(hWhalBus, 0x3c1802), 
			whalBus_PhyRegRead(hWhalBus, 0x3c1802)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_OUT_RAM_IACC(0x%08X) = 0x%08X, 0x%08X\n", 0x3c1E00,
			whalBus_PhyRegRead(hWhalBus, 0x3c1E00), 
			whalBus_PhyRegRead(hWhalBus, 0x3c1E00)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_RLS_SLOPE   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c2807,
			whalBus_PhyRegRead(hWhalBus, 0x3c2807), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2807)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_RLS_SLOPE_SH(0x%08X) = 0x%08X, 0x%08X\n", 0x3c2808,
			whalBus_PhyRegRead(hWhalBus, 0x3c2808), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2808)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_RLS_INTERCEP(0x%08X) = 0x%08X, 0x%08X\n", 0x3c2809,
			whalBus_PhyRegRead(hWhalBus, 0x3c2809), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2809)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_CHANRESP_ICO(0x%08X) = 0x%08X, 0x%08X\n", 0x3c2840,
			whalBus_PhyRegRead(hWhalBus, 0x3c2840), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2840)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_CHANRESP_ICO(0x%08X) = 0x%08X, 0x%08X\n", 0x3c2880,
			whalBus_PhyRegRead(hWhalBus, 0x3c2880), 
			whalBus_PhyRegRead(hWhalBus, 0x3c2880)));

	return OK;
}

int shmDebug_PrintTxRegs(TI_HANDLE hWhalBus)
{
	whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("shmDebug_PrintTxRegs ----MAC--------------\n"));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("TX_STATUS       (0x%08X) = 0x%08X, 0x%08X\n", TX_STATUS,
			whalBus_MacRegRead(hWhalBus, TX_STATUS),
			whalBus_MacRegRead(hWhalBus, TX_STATUS)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("TX_STATE        (0x%08X) = 0x%08X, 0x%08X\n", TX_STATE,
			whalBus_MacRegRead(hWhalBus, TX_STATE),
			whalBus_MacRegRead(hWhalBus, TX_STATE)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("NEXT_SEQ_NUM    (0x%08X) = 0x%08X, 0x%08X\n", NEXT_SEQ_NUM,
			whalBus_MacRegRead(hWhalBus, NEXT_SEQ_NUM),
			whalBus_MacRegRead(hWhalBus, NEXT_SEQ_NUM)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("BCN_SEQ_NUM     (0x%08X) = 0x%08X, 0x%08X\n", BCN_SEQ_NUM,
			whalBus_MacRegRead(hWhalBus, BCN_SEQ_NUM),
			whalBus_MacRegRead(hWhalBus, BCN_SEQ_NUM)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("For Ping/Pong/Pang registers use print register list in the \n"));
	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("Addresses: 0x%08X/0x%08X/0x%08X\n", TX_PING0, TX_PONG0, TX_PANG0));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("shmDebug_PrintTxRegs ----PHY--------------\n"));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_TX_STATUS   (0x%08X) = 0x%08X, 0x%08X\n", 0x3c1002,
			whalBus_PhyRegRead(hWhalBus, 0x3c1002),
			whalBus_PhyRegRead(hWhalBus, 0x3c1002)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_TX_MODE     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c1008,
			whalBus_PhyRegRead(hWhalBus, 0x3c1008),
			whalBus_PhyRegRead(hWhalBus, 0x3c1008)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("PHY_TX_CTRL     (0x%08X) = 0x%08X, 0x%08X\n", 0x3c1001,
			whalBus_PhyRegRead(hWhalBus, 0x3c1001),
			whalBus_PhyRegRead(hWhalBus, 0x3c1001)));

	return OK;
}

int shmDebug_PrintScrPadRegs(TI_HANDLE hWhalBus)
{
	whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("shmDebug_PrintScrPadRegs ---------------------\n"));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD0        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD0,
			whalBus_MacRegRead(hWhalBus, SCR_PAD0),
			whalBus_MacRegRead(hWhalBus, SCR_PAD0)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD1        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD1,
			whalBus_MacRegRead(hWhalBus, SCR_PAD1),
			whalBus_MacRegRead(hWhalBus, SCR_PAD1)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD2        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD2,
			whalBus_MacRegRead(hWhalBus, SCR_PAD2),
			whalBus_MacRegRead(hWhalBus, SCR_PAD2)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD3        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD3,
			whalBus_MacRegRead(hWhalBus, SCR_PAD3),
			whalBus_MacRegRead(hWhalBus, SCR_PAD3)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD4        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD4,
			whalBus_MacRegRead(hWhalBus, SCR_PAD4),
			whalBus_MacRegRead(hWhalBus, SCR_PAD4)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD5        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD5,
			whalBus_MacRegRead(hWhalBus, SCR_PAD5),
			whalBus_MacRegRead(hWhalBus, SCR_PAD5)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD6        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD6,
			whalBus_MacRegRead(hWhalBus, SCR_PAD6),
			whalBus_MacRegRead(hWhalBus, SCR_PAD6)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD7        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD7,
			whalBus_MacRegRead(hWhalBus, SCR_PAD7),
			whalBus_MacRegRead(hWhalBus, SCR_PAD7)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD8        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD8,
			whalBus_MacRegRead(hWhalBus, SCR_PAD8),
			whalBus_MacRegRead(hWhalBus, SCR_PAD8)));

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("SCR_PAD9        (0x%08X) = 0x%08X, 0x%08X\n", SCR_PAD9,
			whalBus_MacRegRead(hWhalBus, SCR_PAD9),
			whalBus_MacRegRead(hWhalBus, SCR_PAD9)));

	return OK;
}

int shmDebug_PrintListRegs(TI_HANDLE hWhalBus, UINT32 RegAddr)
{
	whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
	
	int i;

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
		("shmDebug_PrintListRegs ---------------------\n"));

	for (i=0; i<8; i++, RegAddr+=16)
	{
		WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_CTRL_MODULE_LOG, 
			("REGS (Base=0x%08X) = 0x%08X, 0x%08X, 0x%08X, 0x%08X\n", RegAddr,
				whalBus_MacRegRead(hWhalBus, RegAddr+0),
				whalBus_MacRegRead(hWhalBus, RegAddr+4),
				whalBus_MacRegRead(hWhalBus, RegAddr+8),
                whalBus_MacRegRead(hWhalBus, RegAddr+12)));
	}

	return OK;
}
                            
void shmDebug_MemPrint(TI_HANDLE hWhalBus, UINT32 MemAddr)
{
	whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

	UINT8 HostBuf[256];
	UINT8 *pBuf;
	int i;

	whalBus_MemCopyFrom(hWhalBus, (UINT8*)MemAddr ,(char *)(&HostBuf[0]), 256);

	WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
					  ("shmDebug_MemPrint ========================================\n"));

	for (i=0, pBuf=HostBuf; i<256; i+=16, pBuf+=16)
	{
		WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
						  ("PrintBuf: %08x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", 
						   MemAddr+i, 
						   pBuf[0], pBuf[1], pBuf[2], pBuf[3], pBuf[4], pBuf[5], pBuf[6], pBuf[7], 
						   pBuf[8], pBuf[9], pBuf[10], pBuf[11], pBuf[12], pBuf[13], pBuf[14], pBuf[15]));
	}
}

#endif /* USE_SYNC_API */

