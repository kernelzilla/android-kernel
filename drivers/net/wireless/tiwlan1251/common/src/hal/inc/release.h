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
/*   MODULE:  release.h                                                   */                                                                                                             
/*   PURPOSE: release specific definitions file.                          */                                                                                                             
/*                                                                        */                                                                                                             
/**************************************************************************/                                                                                                             
                                                                                                             
                                                                                                             
#ifndef _HAL_RELEASE_H_                                                                                                             
#define _HAL_RELEASE_H_                                                                                                             
                                                                                                             
#define SW_VERSION_MAJOR	1
#define SW_VERSION_MINOR	0
#define SW_VERSION_PATCH	0
#define SW_VERSION_BUILD	5
                                                                                                    
#define SW_VERSION_STR	"1.0.0.5"
                                                                                                    
#define SW_RELEASE_MONTH	11
#define SW_RELEASE_DAY		16
#define SW_RELEASE_YEAR		2004


#if 0
/*
----------------------------------------------------------------------------
Hal Rev 1.0.0.5
	- Optimization, Save bus access transactions on slave mode
		- HwIntr, Trigger by writing trigger register and not by read/modify/write
		- SetMpduInfo, Write Ctl/Ctl2 once, Clear OWN here 
		- SetMpduInfo, Use local descriptor and prepare for SDIO
		- TxQueue, don't check OWN in allocation
		- TxBufs, Save access to block header, use exsiting value from CheckSize
	- Fix Linux compilation problems
	- Fix eepromless problems
	- Don't use Self clearing on eeprom burst read

Hal Rev 1.0.0.4
	- Change Trace Buffer Defaults to send UDP trace and Print UDP trace on TSDT,
	  On every FW trace .
	- Add Build\whal.dsp project file
	- Fix access to SCR_PAD5 (only from BusAccess)
	- Fix sending list of MSDU issue on USB

Hal Rev 1.0.0.3 
    - Set default channel according to Radio Band and reg domain, The firmware
	  is not handle enableTx on channel 0 anymore
	- Merge from WSP7.2 HAL to get same baseline
		- Add CMD_DEBUG_TRACE command to dynamically stop trace report 
		- Add new health report bit for sequece number error - handle for now ASSERT(0)
		- Add STAT_MSDU_DROP tx status
		- Change USB Rx/Tx descriptors definition

Hal Rev 1.0.0.2 
    - Move Hal API h-files from common/src/inc into hal/Export_Inc

Hal Rev 1.0.0.1 
    - First release after moving the wlan-hal to vob
	- The wlan-hal vob contain hal and BusAccess directories except the USB. 
    - The base lable is WSP7.1.0.19
	- Add this file.


  
-----------------------------------------------------------------------------
*/	
#endif	  






#if 0

/*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

  The release notes below is for bus access developement 
  HAL1.0 branch stared only from WSP7.1.0.19 which already have most of 
  the Hal/busAccess changes

=====================================
Build WSP7.1.0.19
=====================================

	Label DEV_WSP7.1.0.19_Eng_Drop - Merged from REL_WSP7.0.0.29_GA
	Change Rx/Tx descriptors due to the new TX definitions
	Add support of SwitchChannel event for the measurement
	Add 1150 changes for Win98 build and CUDK
	Move Privacy module from RSN to HAL
	Use firmware 2.4.0.27 

=====================================
Build WSP7.1.0.18
=====================================

	Label DEV_WSP7.1.0.18_Eng_Drop
	Filter for Msdu from other BSS with moreFrag bit
	Merge from Label DEV_WSP7.0.0.25_BFT
	    Clean to mbox queue callbacks on recovery
	    Track owner for RX packet (Hal, Rx, MLME, OS) for check msdu leak
	    TxDescFlag contain now VO_SYNC_TRIG and DISASSOC_SYNC_TRIG
	    Change factor of host descriptors from 2 to 4
	    Set lastBDPtr to NULL in EXC (fix CKIP defrag bug)
	    Enable RxReset and recovry on TxStuck and mailbox
	    WSP 7.0.0.25 is after merge from STA_DK 4.1.0.42
	Use Firmware 2.4.0.24 (meged from 2.3.0.28 + AES fixes)
	Fix CheckMailbox initialization
	Fix MSDU ownership check when tx in the rx context (WSP only)

=====================================
Build WSP7.1.0.17
=====================================

	Label DEV_WSP7.1.0.17_Eng_Drop	
	Merge read/write mem/reg via mbox from Wsp7.2
	Disable access to SCR_PAD5 on USB configuration stage 
	Add sending the trace buffer to UDP port

=====================================
Build WSP7.1.0.16
=====================================

	Label DEV_WSP7.1.0.16_Eng_Drop	
	Firmware 2.4.0.20
		fix for calibration on DCR (register 0x2001 setting)
		Fix Frag+AES
		Fix channel 14 disable OFDM
		Add Read/Write memory
	Clean Security objects
	Remove unused siteMgr_disableLna
	Clean 1150 registers file
	Fix ifdefs for USB
	Merge from Label DEV_WSP7.0.0.19_BFT

=====================================
Build WSP7.1.0.15 
=====================================

	Label DEV_WSP7.1.0.15_Eng_Drop	
	Merge from DEV_WSP_7.0.0.15_BFT
	Move debug reg/mem print into bus access
	Add setting of SCR_PAD5 in USB
	Add support for trace buffer in USB
	Add fix for check size per queue in USB
	Merge the fix for CKIP low rate problem
	Merge fix for trace debug and move utils to whal
	Firmware 2.4.0.15

=====================================
Build WSP7.1.0.13 
=====================================

	Label DEV_WSP7.1.0.13_Eng_Drop	
	Remove the use of Driver workspace 
	Remove disable OFDM in channel 14 - moved to the firmware
	Call energy detection (CCA threshold) through mailbox and not to register 
	Merge USB make fixes
	No voice cookie in the USB mode (if needed - add field in the tx desc)
	Move direct registers access into bus access
	Use the new DtimPeriod information element
	Auto calibration disable on start (fix the problem in the firmware) - 
	Auto calibration on SPARE_A2 and not A1
	USB return always not in Radio Stand By 
	Firmware 2.4.0.15 - AES fix, Chan 14 fix, Fix Calib on strat


=====================================
Build WSP7.1.0.12 
=====================================

	Remove TNETW1100 support
	Merge USB objects

=====================================
Build WSP7.1.0.11 
=====================================

	Label DEV_WSP7.1.0.11_Eng_Drop	
	Merge from DEV_7.0.0.14_BFT
	Re-arrange master/slave defines
	Remove UseTxInterrupt flag
	Add 16 bytes for security slave mode in one place

=====================================
Build WSP7.1.0.10 
=====================================

	Label DEV_WSP7.1.0.10_Eng_Drop	
	Fix for radio DCR calibration Issue (in the firmware) 
	Integration fixes on trace buffer
	Add whalRecovery, whalDebug, whalRadio files for whalCtrl object
	Disable overide tx power level table on 1150
	Split whalHwRegs, add tnetwCommon.h
	Move TxDesc, TxQueue and rx/tx h-files to common
	Remove MASTER_MODE from MboxCmd
	Fix print rx registers
	Firmware 2.4.0.9

=====================================
Build WSP7.1.0.9 
=====================================

	Label DEV_WSP7.1.0.9_Eng_Drop	
	Merge from WSP_7.0.0.13
	Create new TnetwServices directory with 1100/1130/1150 registers and macro definitions
	Create new FirmwareApi directory with all firmware h-files
	Re-arrange shmFwCtrl object and eepromless boot
	Tx modules handle static Recovery counter NumEmptyCount

	Merge Event mailbox 
	Fix reg domain check error on WBR a/b/g radio

	Add LNA from Event mailbox
	Don't disable Auto calibration at startup (shmFwCtrl)
	Add trace buffer print function in HwAccess
	Set defualt recovery to ENABLE 
	Firmware 2.4.0.6

=====================================
Build WSP7.1.0.8
=====================================

	Label DEV_WSP7.1.0.8_Eng_Drop	
	Merge from 7.0.0.8

=====================================
Build WSP7.1.0.6 
=====================================

	Merge Bus access fixes from private branches

=====================================
Build WSP7.1.0.4 
=====================================

	Merge from 1150 branch

=====================================
Build WSP7.1.0.2 
=====================================

	Merge 1150 from WSP 6.9.0.14
	Firmware 2.3.0.8

=====================================
Build WSP7.1.0.1
=====================================

	Based on DEV_WSP7.0.0.1_BFT
	Bus access component :
	   Create new whalBus_Api.h file which export the bus access API 
	   Create new whalBus.c file which manage the bus access 
	   Move HwAccesss, HwIntr, HwEeprom and HwMbox objects from hw_ctrl to whalBus
	   Create new shmFwCtrl object to handle the firmware boot 
	   Create new shmUtils file to support common general utilities
	   Move Rx/Tx objects for the slave from hw_data to shm_slave directory
	   Move Rx/Tx objects for the master from hw_data to shm_master directory
	   Same h-file for Rx/Tx Master/Slave
	   whalHwTxQueue and whalHwDesc objects still have master mode code
	   ConfigHw into Rx/Tx Mater/Slave (from whalHwCtrl)
	Hal changes :
	   Move the creation of HwAccess, HwIntr, HwMbox, HwEeprom Trace to Bus access
	   Replace the HwAccess, HwIntr objects functions with whalBus functions
	   Replace the usage of HwRx, HwTx objects handle and functions with whalBus
	   Move Firmware download and eepromless functions into shmFwCtrl
	   Remove Info mailbox
	   Call whalBus to config queue addresses (ConfigHw)
	   Move Utilities (print buffer, …) from whalParams to shmUtils
	   Move DmaParams definitions to bus access 
	System
	   Add bus access objects into source.mak
	   Add bus access include path to the makefile
	   Add DSP file

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/
#endif

#endif /* _HAL_RELEASE_H_ */                                                                                                    
                                                                                                       
                                                                                                        
