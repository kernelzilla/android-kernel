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
 *   MODULE:  ShmFwCtrl.h
 *   PURPOSE: Firmware control  object
 *
 ****************************************************************************/

#ifndef _SHM_FW_CTRL_H
#define _SHM_FW_CTRL_H

#define REF_FREQ_19_2                       0
#define REF_FREQ_26_0                       1
#define REF_FREQ_38_4                       2
#define REF_FREQ_40_0                       3
#define REF_FREQ_33_6                       4
#define REF_FREQ_NUM                        5

#define LUT_PARAM_INTEGER_DIVIDER           0
#define LUT_PARAM_FRACTIONAL_DIVIDER        1
#define LUT_PARAM_ATTN_BB                   2
#define LUT_PARAM_ALPHA_BB                  3
#define LUT_PARAM_STOP_TIME_BB              4
#define LUT_PARAM_BB_PLL_LOOP_FILTER        5
#define LUT_PARAM_NUM                       6

#define ACX_EEPROMLESS_IND_REG              (SCR_PAD4)
#define USE_EEPROM                          0
#define SOFT_RESET_MAX_TIME                 1000000
#define SOFT_RESET_STALL_TIME               1000
#define NVS_DATA_BUNDARY_ALIGNMENT          4

#define SHMFWCTRL_XTAL_USED                 0x3 
#define SHMFWCTRL_XTAL_CLK_REQ_TIME         0x3E

#define SHMFWCTRL_SCRPAD6_REF_FREQ_MASK 0x000000FF
#define SHMFWCTRL_SCRPAD6_CLK_TYPE 		0x0000FF00
#define SHMFWCTRL_PLL_CAL_TIME 			0x5810
#define SHMFWCTRL_PLL_CAL_TIME_VAL		0x9
#define SHMFWCTRL_CLK_REQ_TIME			0x5814
#define SHMFWCTRL_CLK_BUF_TIME			0x5818
#define SHMFWCTRL_CLK_BUF_TIME_VAL		0x6
#define SHMFWCTRL_PLL_STABLE_TIME		0x5820
#define SHMFWCTRL_PLL_STABLE_TIME_VAL	0x0
#define SHMFWCTRL_ELP_CFG_MODE			0x5804
#define SHMFWCTRL_ELP_CFG_MODE_CLK_DETECT	0x00004000
#define SHMFWCTRL_RF_AFE_REG_3			0x58CC
#define SHMFWCTRL_RF_AFE_REG_3_VAL		0x4B5
#define SHMFWCTRL_RF_AFE_REG_5			0x58D4
#define SHMFWCTRL_RF_AFE_REG_5_VAL		0x50
#define SHMFWCTRL_RF_AFE_CTRL_REG_2		0x5948
#define SHMFWCTRL_RF_AFE_CTRL_REG_2_VAL	0x11C001
#define SHMFWCTRL_RF_AFE_REG_13			0x58F4
#define SHMFWCTRL_RF_AFE_REG_13_VAL		0x1E

#define SHMFWCTRL_PLL_BB_REG_0			0x5840
#define SHMFWCTRL_PLL_BB_REG_0_VAL		0x00017000

#define SHMFWCTRL_PLL_BB_REG_1			0x5844

#define SHMFWCTRL_PLL_BB_REG_2			0x5848
#define SHMFWCTRL_PLL_BB_REG_2_VAL		0x3039

#define SHMFWCTRL_PLL_BB_REG_5 			0x5854
#define SHMFWCTRL_PLL_BB_REG_5_VAL		0x1

#define SHMFWCTRL_PLL_BB_REG_6 			0x5858
#define SHMFWCTRL_PLL_BB_REG_6_VAL		0x000A0000

#define SHMFWCTRL_RF_AFE_REG_14 			0x58F8
#define SHMFWCTRL_RF_AFE_REG_14_VAL		0x00000030

#define SHMFWCTRL_RF_AFE_REG_12			0x58F0
#define SHMFWCTRL_RF_AFE_REG_12_VAL		0x29

#define SHMFWCTRL_ELP_CMD					0x5808
#define SHMFWCTRL_ELP_CMD_VAL			0x1

#define SHMFWCTRL_XTAL_CLK_REQ_TIME_SHORT 0x3E
#define SHMFWCTRL_XTAL_CLK_REQ_TIME_LONG 0xA4

/************************************************************************
 * external Functions
 ************************************************************************/


#endif /* _SHM_FW_CTRL_H */
