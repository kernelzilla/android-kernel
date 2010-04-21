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

#ifndef PUBLIC_HOST_INT_H
#define PUBLIC_HOST_INT_H

#include "public_types.h"

                
/*************************************************************************

    Host Interrupt Register (WiLink -> Host)
 
**************************************************************************/

#define ACX_INTR_RX0_DATA			BIT_0   /* RX packet is ready in Xfer buffer #0*/
#define ACX_INTR_TX_RESULT			BIT_1   /* TX result(s) are in the TX complete buffer  */
#define ACX_INTR_TX_XFR				BIT_2   /* OBSOLETE*/
#define ACX_INTR_RX1_DATA			BIT_3   /* RX packet is ready in Xfer buffer #1*/
#define ACX_INTR_EVENT_A			BIT_4   /* Event was entered to Event MBOX #A*/
#define ACX_INTR_EVENT_B	        BIT_5   /* Event was entered to Event MBOX #B */
#define ACX_INTR_WAKE_ON_HOST       BIT_6   /* OBSOLETE */
#define ACX_INTR_TRACE_A	        BIT_7   /* Trace meassge on MBOX #A*/
#define ACX_INTR_TRACE_B            BIT_8   /* Trace meassge on MBOX #B*/
#define ACX_INTR_CMD_COMPLETE       BIT_9   /* Command processing completion*/
#define ACX_INTR_INIT_COMPLETE		BIT_14	/* Init sequence is done*/

#define ACX_INTR_ALL                0xFFFFFFFF


/*************************************************************************

    Interrupt Trigger Register (Host -> WiLink)              
  
**************************************************************************/

/******** Hardware to Embedded CPU Interrupts - first 32-bit register set ********/ 
   
#define INTR_TRIG_CMD       BIT_0   /* Host Command Interrupt. Setting this bit masks*/
                                    /* the interrupt that the host issues to inform*/
                                    /* the FW that it has sent a command*/
                                    /* to the Wlan hardware Command Mailbox.*/
    
#define INTR_TRIG_EVENT_ACK BIT_1   /* Host Event Acknowlegde Interrupt. The host */
                                    /* sets this bit to acknowledge that it received*/
                                    /* the unsolicited information from the event*/
                                    /* mailbox.*/
                                       
#define INTR_TRIG_TX_PROC0 BIT_2    /* The host sets this bit to inform the Wlan */
                                    /* FW that a TX packet is in the XFER */
                                    /* Buffer #0.*/
                                       
#define INTR_TRIG_RX_PROC0 BIT_3    /* The host sets this bit to inform the FW */
                                    /* that it read a packet from RX XFER */
                                    /* Buffer #0.*/

#define INTR_TRIG_DEBUG_ACK BIT_4 
    
#define INTR_TRIG_STATE_CHANGED BIT_5
        

/******** Hardware to Embedded CPU Interrupts - second 32-bit register set ********/

#define INTR_TRIG_RX_PROC1 BIT_17     /* The host sets this bit to inform the FW */
                                     /* that it read a packet from RX XFER */
                                     /* Buffer #1.  */

#define INTR_TRIG_TX_PROC1 BIT_18    /* The host sets this bit to inform the Wlan */
                                     /* hardware that a TX packet is in the XFER */
                                     /* Buffer #1.*/

#endif

