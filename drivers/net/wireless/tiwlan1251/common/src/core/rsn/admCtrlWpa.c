/** \file admCtrl.c
 *  \brief Admission control API implimentation
 *
 *  \see admCtrl.h
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

/****************************************************************************
 *                                                                          *
 *   MODULE:  Admission Control	    		                                *
 *   PURPOSE: Admission Control Module API                              	*
 *                                                                          *
 ****************************************************************************/

#include "osApi.h"

#include "utils.h"
#include "paramOut.h"
#include "paramIn.h"
#include "mlmeApi.h"
#include "802_11Defs.h"
#include "DataCtrl_Api.h"
#include "report.h"

#include "utils.h"

#include "rsn.h"
#include "admCtrl.h"
#include "admCtrlWpa.h"
#include "admCtrlWpa2.h"


#ifdef EXC_MODULE_INCLUDED
#include "admCtrlExc.h"
#include "excMngr.h"
#endif

/* Constants */
#define MAX_NETWORK_MODE 2
#define MAX_WPA_CIPHER_SUITE 7



/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* Local functions definitions */

/* Global variables */

static UINT8 wpaIeOuiIe[3] = { 0x00, 0x50, 0xf2};

static BOOL broadcastCipherSuiteValidity[MAX_NETWORK_MODE][MAX_WPA_CIPHER_SUITE]=
{
    /* RSN_IBSS */  {
/* NONE		  */    FALSE,
/* WEP40	  */    FALSE,
/* TKIP		  */    TRUE,
/* AES_WRAP	  */    TRUE,
/* AES_CCMP	  */    TRUE,
/* WEP104     */    FALSE,
/* CKIP       */    FALSE},

    /* RSN_INFRASTRUCTURE */  {
/* NONE		  */    FALSE,
/* WEP		  */    TRUE,
/* TKIP		  */    TRUE,
/* AES_WRAP	  */    TRUE,
/* AES_CCMP	  */    TRUE,
/* WEP104     */    TRUE,
/* CKIP       */    TRUE}
};

/** WPA admission table. Used to verify admission parameters to an AP */
/* table parameters:
    Max unicast cipher in the IE
    Max broadcast cipher in the IE
    Encryption status 
*/
typedef struct
{
    TI_STATUS        status;
    cipherSuite_e    unicast;
    cipherSuite_e    broadcast;
    UINT8            evaluation; 
} admCtrlWpa_validity_t;

static admCtrlWpa_validity_t    admCtrlWpa_validityTable[MAX_WPA_CIPHER_SUITE][MAX_WPA_CIPHER_SUITE][MAX_WPA_CIPHER_SUITE] =
{
/* AP unicast NONE */ {
        /* AP multicast NONE */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP40 */ { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP40 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP40 */ { OK,  RSN_CIPHER_NONE, RSN_CIPHER_WEP ,1},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ OK,  RSN_CIPHER_NONE, RSN_CIPHER_WEP104 ,1}},
        /* AP multicast TKIP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { OK,  RSN_CIPHER_NONE, RSN_CIPHER_TKIP ,2},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WRAP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { OK,  RSN_CIPHER_NONE, RSN_CIPHER_AES_WRAP ,3},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast CCMP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK,  RSN_CIPHER_NONE, RSN_CIPHER_AES_CCMP ,3},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP104 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP40 */ { OK,  RSN_CIPHER_NONE, RSN_CIPHER_WEP ,1},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ OK,  RSN_CIPHER_NONE, RSN_CIPHER_WEP104 ,1}}},
/* AP unicast WEP */  {
        /* AP multicast NONE */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { OK, RSN_CIPHER_WEP, RSN_CIPHER_WEP ,1},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK, RSN_CIPHER_WEP, RSN_CIPHER_WEP ,1},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast TKIP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WRAP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WRAP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast CCMP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP104 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}}},
/* AP unicast TKIP */  {
        /* AP multicast NONE */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { OK,  RSN_CIPHER_TKIP, RSN_CIPHER_WEP  ,4},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast TKIP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { OK,  RSN_CIPHER_TKIP, RSN_CIPHER_TKIP ,7},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WRAP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast CCMP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP104 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { OK,  RSN_CIPHER_TKIP, RSN_CIPHER_WEP104 ,4},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}}},
/* AP unicast AES_WRAP */ {
        /* AP multicast NONE */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP40 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { OK,  RSN_CIPHER_AES_WRAP, RSN_CIPHER_WEP ,5},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast TKIP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { OK,  RSN_CIPHER_AES_WRAP, RSN_CIPHER_TKIP ,6},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WRAP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { OK,  RSN_CIPHER_AES_WRAP, RSN_CIPHER_AES_WRAP ,8},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast CCMP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP104 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { OK,  RSN_CIPHER_AES_WRAP, RSN_CIPHER_WEP104 ,5},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}}},
/* AP unicast AES_CCMP */ {
        /* AP multicast NONE */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK,  RSN_CIPHER_AES_CCMP, RSN_CIPHER_WEP ,5},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast TKIP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK,  RSN_CIPHER_AES_CCMP, RSN_CIPHER_TKIP ,6},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WRAP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast CCMP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK, RSN_CIPHER_AES_CCMP, RSN_CIPHER_AES_CCMP ,7},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK,  RSN_CIPHER_AES_CCMP, RSN_CIPHER_WEP104 ,5},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}}},
/* AP unicast WEP104 */  {
        /* AP multicast NONE */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast TKIP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WRAP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast CCMP */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP104 */{ NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0}},
        /* AP multicast WEP104 */ {
            /* STA NONE */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA WEP */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA TKIP */  { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA AES */   { NOK, RSN_CIPHER_NONE, RSN_CIPHER_NONE ,0},
            /* STA CCMP */  { OK, RSN_CIPHER_WEP104, RSN_CIPHER_WEP104 ,1},
            /* STA WEP104 */{ OK, RSN_CIPHER_WEP104, RSN_CIPHER_WEP104 ,1}}}


};

/* Function prototypes */
TI_STATUS admCtrlWpa_parseIe(admCtrl_t *pAdmCtrl, UINT8 *pWpaIe, wpaIeData_t *pWpaData);
UINT16 admCtrlWpa_buildCapabilities(UINT16 replayCnt);
UINT32  admCtrlWpa_parseSuiteVal(admCtrl_t *pAdmCtrl, UINT8* suiteVal,wpaIeData_t *pWpaData,UINT32 maxVal);
TI_STATUS admCtrlWpa_checkCipherSuiteValidity (cipherSuite_e unicastSuite, cipherSuite_e broadcastSuite, cipherSuite_e encryptionStatus);
static TI_STATUS admCtrlWpa_get802_1x_AkmExists (admCtrl_t *pAdmCtrl, BOOL *wpa_802_1x_AkmExists);


/**
*
* admCtrlWpa_config  - Configure EXC admission control.
*
* \b Description: 
*
* Configure EXC admission control.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrlWpa_config(admCtrl_t *pAdmCtrl)
{
    TI_STATUS           status;
    rsn_paeConfig_t     paeConfig;

    /* check and set admission control default parameters */
    pAdmCtrl->authSuite =   RSN_AUTH_OPEN;
    if (pAdmCtrl->unicastSuite == RSN_CIPHER_NONE)
    {
        pAdmCtrl->unicastSuite = RSN_CIPHER_TKIP;
    }
    if (pAdmCtrl->broadcastSuite == RSN_CIPHER_NONE)
    {
        pAdmCtrl->broadcastSuite = RSN_CIPHER_TKIP;
    }

    /* set callback functions (API) */
    pAdmCtrl->getInfoElement = admCtrlWpa_getInfoElement;
    pAdmCtrl->setSite = admCtrlWpa_setSite;
    pAdmCtrl->evalSite = admCtrlWpa_evalSite;

    pAdmCtrl->getPmkidList      = admCtrl_nullGetPMKIDlist;
    pAdmCtrl->setPmkidList      = admCtrl_nullSetPMKIDlist;
    pAdmCtrl->resetPmkidList    = admCtrl_resetPMKIDlist;
    pAdmCtrl->getPreAuthStatus = admCtrl_nullGetPreAuthStatus;
	pAdmCtrl->startPreAuth	= admCtrl_nullStartPreAuth;
    pAdmCtrl->get802_1x_AkmExists = admCtrlWpa_get802_1x_AkmExists;

    /* set cipher suite */                                  
    switch (pAdmCtrl->externalAuthMode)
    {
    case RSN_EXT_AUTH_MODE_WPA:
    case RSN_EXT_AUTH_MODE_WPAPSK:
        /* The cipher suite should be set by the External source via 
        the Encryption field*/
        pAdmCtrl->keyMngSuite = RSN_KEY_MNG_802_1X;
        break;
    case RSN_EXT_AUTH_MODE_WPANONE:
        pAdmCtrl->keyMngSuite = RSN_KEY_MNG_NONE;
        /* Not supported */
    default:
        return NOK;
    }


    paeConfig.authProtocol = pAdmCtrl->externalAuthMode;
    paeConfig.unicastSuite = pAdmCtrl->unicastSuite;
    paeConfig.broadcastSuite = pAdmCtrl->broadcastSuite;
    paeConfig.keyExchangeProtocol = pAdmCtrl->keyMngSuite;
	/* set default PAE configuration */
    status = pAdmCtrl->pRsn->setPaeConfig(pAdmCtrl->pRsn, &paeConfig);

    return status;
}




TI_STATUS admCtrlWpa_dynamicConfig(admCtrl_t *pAdmCtrl,wpaIeData_t *pWpaData)
{
    TI_STATUS           status;
    rsn_paeConfig_t     paeConfig;


    /* set callback functions (API) */
    pAdmCtrl->getInfoElement = admCtrlWpa_getInfoElement;

    switch (pAdmCtrl->externalAuthMode)
    {
    case RSN_EXT_AUTH_MODE_WPA:
    case RSN_EXT_AUTH_MODE_WPAPSK:
        /* The cipher suite should be set by the External source via 
        the Encryption field*/
        pAdmCtrl->keyMngSuite = RSN_KEY_MNG_802_1X;
        break;
    case RSN_EXT_AUTH_MODE_WPANONE:
        pAdmCtrl->keyMngSuite = RSN_KEY_MNG_NONE;
        /* Not supported */
    default:
        return NOK;
    }


    paeConfig.authProtocol = pAdmCtrl->externalAuthMode;
    paeConfig.unicastSuite = pWpaData->unicastSuite[0];
    paeConfig.broadcastSuite = pWpaData->broadcastSuite;
    paeConfig.keyExchangeProtocol = pAdmCtrl->keyMngSuite;
	/* set default PAE configuration */
    status = pAdmCtrl->pRsn->setPaeConfig(pAdmCtrl->pRsn, &paeConfig);

    return status;
}

/**
*
* admCtrlWpa_getInfoElement - Get the current information element.
*
* \b Description: 
*
* Get the current information element.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - pIe - IE buffer \n
*  I   - pLength - length of IE \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/

TI_STATUS admCtrlWpa_getInfoElement(admCtrl_t *pAdmCtrl, UINT8 *pIe, UINT8 *pLength)
{
    wpaIePacket_t       localWpaPkt;
    wpaIePacket_t       *pWpaIePacket;
    UINT8               length;
#ifdef FOUR_ALIGNMENT
	UINT16				tempInt;
#endif

    if (pIe==NULL)
    {
        *pLength = 0;
        return NOK;
    }

    
    /* Check validity of WPA IE */
    if (!broadcastCipherSuiteValidity[pAdmCtrl->networkMode][pAdmCtrl->broadcastSuite])
    {   /* check Group suite validity */                                          
        *pLength = 0;
        return NOK;
    }
   
    
    if (pAdmCtrl->unicastSuite == RSN_CIPHER_WEP)
    {   /* check pairwise suite validity */                                       
        *pLength = 0;
        return NOK;
    }

    /* Build Wpa IE */
    pWpaIePacket = &localWpaPkt;
    os_memoryZero(pAdmCtrl->hOs, pWpaIePacket, sizeof(wpaIePacket_t));
    pWpaIePacket->elementid= WPA_IE_ID;
    os_memoryCopy(pAdmCtrl->hOs, (void *)pWpaIePacket->oui, wpaIeOuiIe, 3);
    pWpaIePacket->ouiType = WPA_OUI_DEF_TYPE;
#ifndef FOUR_ALIGNMENT
	pWpaIePacket->version = ENDIAN_HANDLE_WORD(WPA_OUI_MAX_VERSION);
#else
    /* required for WinCe, when the pointer is not even */
	tempInt = ENDIAN_HANDLE_WORD(WPA_OUI_MAX_VERSION);
	os_memoryCopy(pAdmCtrl->hOs, (UINT8*)&pWpaIePacket->version, &tempInt, sizeof(pWpaIePacket->version));
#endif


    length = sizeof(wpaIePacket_t)-2;
    /* check defaults */
    if (pAdmCtrl->replayCnt==1)
    {
        length -= 2; /* 2: capabilities + 4: keyMng suite, 2: keyMng count*/
#if 0 /* The following was removed since there are APs which do no accept
	the default WPA IE */
		if (pAdmCtrl->externalAuthMode == RSN_EXT_AUTH_MODE_WPA)
		{
			length -= 6; /* 2: capabilities + 4: keyMng suite, 2: keyMng count*/
			if (pAdmCtrl->unicastSuite == RSN_CIPHER_TKIP)
			{
				length -= 6; /* 4: unicast suite, 2: unicast count */
				if (pAdmCtrl->broadcastSuite == RSN_CIPHER_TKIP)
				{
					length -= 4;  /* broadcast suite */
				}
			}
		}
#endif
	}
    pWpaIePacket->length = length;
    *pLength = length+2;

    
	
	if (length>=WPA_IE_MIN_DEFAULT_LENGTH)
    {   /* build Capabilities */
        pWpaIePacket->capabilities = ENDIAN_HANDLE_WORD(admCtrlWpa_buildCapabilities(pAdmCtrl->replayCnt));
	}

	if (length>=WPA_IE_MIN_KEY_MNG_SUITE_LENGTH(1))
	{
        /* build keyMng suite */
#ifndef FOUR_ALIGNMENT        
		pWpaIePacket->authKeyMngSuiteCnt = ENDIAN_HANDLE_WORD(0x0001);
#else
        /* required for WinCe, when the pointer is not even */
        tempInt =  ENDIAN_HANDLE_WORD(0x0001);
        os_memoryCopy (pAdmCtrl->hOs,(UINT8*)&pWpaIePacket->authKeyMngSuiteCnt, &tempInt, sizeof(pWpaIePacket->authKeyMngSuiteCnt)); 
#endif

        os_memoryCopy(pAdmCtrl->hOs, (void *)pWpaIePacket->authKeyMngSuite, wpaIeOuiIe, 3);
        
        switch (pAdmCtrl->externalAuthMode)
        {
        case RSN_EXT_AUTH_MODE_OPEN:
        case RSN_EXT_AUTH_MODE_SHARED_KEY:
        case RSN_EXT_AUTH_MODE_AUTO_SWITCH:
            pWpaIePacket->authKeyMngSuite[3] = WPA_IE_KEY_MNG_NONE;
            break;
		case RSN_EXT_AUTH_MODE_WPA:
			{
#ifdef EXC_MODULE_INCLUDED
				UINT8	akmSuite[DOT11_OUI_LEN+1];

				if (admCtrlExc_getCckmAkm(pAdmCtrl, akmSuite))
				{
					os_memoryCopy(pAdmCtrl->hOs, (PVOID)pWpaIePacket->authKeyMngSuite, akmSuite, DOT11_OUI_LEN+1);
				}
				else
#endif
				{
					pWpaIePacket->authKeyMngSuite[3] = WPA_IE_KEY_MNG_801_1X;
				}
			}

            break;

        case RSN_EXT_AUTH_MODE_WPAPSK:
            pWpaIePacket->authKeyMngSuite[3] = WPA_IE_KEY_MNG_PSK_801_1X;
            break;
        default:
            pWpaIePacket->authKeyMngSuite[3] = WPA_IE_KEY_MNG_NONE;
            break;
        }
         
    }

    
    if (length>=WPA_IE_MIN_PAIRWISE_SUITE_LENGTH)
    {  
 
#ifdef EXC_MODULE_INCLUDED       
        if ((pAdmCtrl->pRsn->paeConfig.unicastSuite==RSN_CIPHER_CKIP) || (pAdmCtrl->pRsn->paeConfig.broadcastSuite==RSN_CIPHER_CKIP))
        {
           admCtrlExc_getWpaCipherInfo(pAdmCtrl,pWpaIePacket);
        }
        else
#endif
        {
        
            /* build pairwise suite */
    #ifndef FOUR_ALIGNMENT        
		    pWpaIePacket->pairwiseSuiteCnt = ENDIAN_HANDLE_WORD(0x0001);
    #else
            /* required for WinCe, when the pointer is not even */
            tempInt =  ENDIAN_HANDLE_WORD(0x0001);
            os_memoryCopy (pAdmCtrl->hOs,(UINT8*)&pWpaIePacket->pairwiseSuiteCnt, &tempInt, sizeof(pWpaIePacket->pairwiseSuiteCnt));
    #endif

            os_memoryCopy(pAdmCtrl->hOs, (void *)pWpaIePacket->pairwiseSuite, wpaIeOuiIe, 3);
            pWpaIePacket->pairwiseSuite[3] = (UINT8)pAdmCtrl->pRsn->paeConfig.unicastSuite;
       
            if (length>=WPA_IE_GROUP_SUITE_LENGTH)
            {   /* build group suite */
                os_memoryCopy(pAdmCtrl->hOs, (void *)pWpaIePacket->groupSuite, wpaIeOuiIe, 3);
                pWpaIePacket->groupSuite[3] = (UINT8)pAdmCtrl->pRsn->paeConfig.broadcastSuite;
            }
        }
    }
    os_memoryCopy(pAdmCtrl->hOs, (UINT8*)pIe, (UINT8*)pWpaIePacket, sizeof(wpaIePacket_t));
    return OK;

}
/**
*
* admCtrlWpa_setSite  - Set current primary site parameters for registration.
*
* \b Description: 
*
* Set current primary site parameters for registration.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - pRsnData - site's RSN data \n
*  O   - pAssocIe - result IE of evaluation \n
*  O   - pAssocIeLen - length of result IE of evaluation \n
*  
* \b RETURNS:
*
*  OK on site is aproved, NOK on site is rejected.
*
* \sa 
*/
TI_STATUS admCtrlWpa_setSite(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, UINT8 *pAssocIe, UINT8 *pAssocIeLen)
{
    TI_STATUS           status;
    paramInfo_t         param;
    whalParamInfo_t     whalParam;
    wpaIeData_t         wpaData;
    cipherSuite_e       encryptionStatus;
    admCtrlWpa_validity_t *pAdmCtrlWpa_validity=NULL;
    UINT8                 *pWpaIe;
    UINT8                 index;

    *pAssocIeLen = 0;

    if (pRsnData==NULL)
    {
        return NOK;
    }
    if (pRsnData->pIe==NULL)
    {
		/* configure the MLME module with the 802.11 OPEN authentication suite, 
			THe MLME will configure later the authentication module */
		param.paramType = MLME_LEGACY_TYPE_PARAM;
		param.content.mlmeLegacyAuthType = AUTH_LEGACY_OPEN_SYSTEM;
		status = mlme_setParam(pAdmCtrl->hMlme, &param);
		if (status != OK)
		{
			return status;
		}

        return OK;
    }

#ifdef EXC_MODULE_INCLUDED
	/* Check if Aironet IE exists */
	admCtrlExc_setExtendedParams(pAdmCtrl, pRsnData);
#endif /*EXC_MODULE_INCLUDED*/

    /* Check if any-WPA mode is supported and WPA2 info elem is presented */
    /* If yes - perform WPA2 set site  procedure                          */
    if(pAdmCtrl->WPAMixedModeEnable && pAdmCtrl->WPAPromoteFlags)
    {
        if((admCtrl_parseIe(pAdmCtrl, pRsnData, &pWpaIe, RSN_IE_ID)== OK) &&
           (pWpaIe != NULL))
        {
           status = admCtrlWpa2_setSite(pAdmCtrl, pRsnData,  pAssocIe, pAssocIeLen);
           if(status == OK)
               return status;
        }
    }
    
	status = admCtrl_parseIe(pAdmCtrl, pRsnData, &pWpaIe, WPA_IE_ID);
	if (status != OK)                                                         
	{                                                                                    
		return status;                                                        
	}
    status = admCtrlWpa_parseIe(pAdmCtrl, pWpaIe, &wpaData);
    if (status != OK)
    {
        return status;
    }
    if ((wpaData.unicastSuite[0]>=MAX_WPA_CIPHER_SUITE) ||
        (wpaData.broadcastSuite>=MAX_WPA_CIPHER_SUITE) ||
        (pAdmCtrl->unicastSuite>=MAX_WPA_CIPHER_SUITE))
    {
        return NOK;
    }

    pAdmCtrl->encrInSw = wpaData.excKp;
    pAdmCtrl->micInSw = wpaData.excMic; 

    /*Because ckip is a proprietary encryption for Cisco then a different validity check is needed */
    if(wpaData.broadcastSuite == RSN_CIPHER_CKIP || wpaData.unicastSuite[0] ==  RSN_CIPHER_CKIP)
    {
        pAdmCtrl->getCipherSuite(pAdmCtrl, &encryptionStatus);
	/*Funk supplicant can support CCKM only if it configures the driver to TKIP encryption. */
        if (encryptionStatus != RSN_CIPHER_TKIP) 
            return NOK;
        if (pAdmCtrl->encrInSw)
            pAdmCtrl->excSupport = TRUE;
    }
    else
    {
        /* Check validity of Group suite */
        if (!broadcastCipherSuiteValidity[pAdmCtrl->networkMode][wpaData.broadcastSuite])
        {   /* check Group suite validity */                                          
            return NOK;
        }

        pAdmCtrl->getCipherSuite(pAdmCtrl, &encryptionStatus);
        for (index=0; index<wpaData.unicastSuiteCnt; index++)
        {
            pAdmCtrlWpa_validity = &admCtrlWpa_validityTable[wpaData.unicastSuite[index]][wpaData.broadcastSuite][encryptionStatus];
            if (pAdmCtrlWpa_validity->status ==OK)
            {
                break;
            }
        }

        if (pAdmCtrlWpa_validity->status !=OK)
        {
            return pAdmCtrlWpa_validity->status;
        }
   
        /* set cipher suites */
        wpaData.unicastSuite[0] = pAdmCtrlWpa_validity->unicast ;/*wpaData.unicastSuite[0];*/
        wpaData.broadcastSuite = pAdmCtrlWpa_validity->broadcast; /*wpaData.broadcastSuite;*/
    }
    /* set external auth mode according to the key Mng Suite */
    switch (wpaData.KeyMngSuite[0])
    {
    case WPA_IE_KEY_MNG_NONE:
        pAdmCtrl->externalAuthMode = RSN_EXT_AUTH_MODE_OPEN;
        break;
	case WPA_IE_KEY_MNG_801_1X:
#ifdef EXC_MODULE_INCLUDED
	case WPA_IE_KEY_MNG_CCKM:
#endif
        pAdmCtrl->externalAuthMode = RSN_EXT_AUTH_MODE_WPA;
        break;
    case WPA_IE_KEY_MNG_PSK_801_1X:
        pAdmCtrl->externalAuthMode = RSN_EXT_AUTH_MODE_WPAPSK;
        break;
    default:
        pAdmCtrl->externalAuthMode = RSN_EXT_AUTH_MODE_OPEN;
        break;
    }

      

#ifdef EXC_MODULE_INCLUDED
	param.paramType = EXC_CCKM_EXISTS;
	param.content.excCckmExists = (wpaData.KeyMngSuite[0]==WPA_IE_KEY_MNG_CCKM) ? TRUE : FALSE;
	excMngr_setParam(pAdmCtrl->hExcMngr, &param);
#endif
    /* set replay counter */
    pAdmCtrl->replayCnt = wpaData.replayCounters;

    *pAssocIeLen = pRsnData->ieLen;
    if (pAssocIe != NULL)
    {
        os_memoryCopy(pAdmCtrl->hOs, pAssocIe, &wpaData, sizeof(wpaIeData_t));
    }


    /* Now we configure the MLME module with the 802.11 legacy authentication suite, 
        THe MLME will configure later the authentication module */
    param.paramType = MLME_LEGACY_TYPE_PARAM;
#ifdef EXC_MODULE_INCLUDED
	if (pAdmCtrl->networkEapMode!=OS_EXC_NETWORK_EAP_OFF)
    {
        param.content.mlmeLegacyAuthType = AUTH_LEGACY_RESERVED1;
    }
	else
#endif
	{
		param.content.mlmeLegacyAuthType = AUTH_LEGACY_OPEN_SYSTEM;
	}
    
    
    status = mlme_setParam(pAdmCtrl->hMlme, &param);
    if (status != OK)
    {
        return status;
    }

    param.paramType = RX_DATA_EAPOL_DESTINATION_PARAM;
    param.content.rxDataEapolDestination = OS_ABS_LAYER;
    status = rxData_setParam(pAdmCtrl->hRx, &param);
    if (status != OK)
    {
        return status;
    }

	/* Configure privacy status in HAL so that HW is prepared to recieve keys */
	whalParam.paramType = HAL_CTRL_RSN_SECURITY_MODE_PARAM;   
	whalParam.content.rsnEncryptionStatus = (halCtrl_CipherSuite_e)wpaData.unicastSuite[0];
	status = whalCtrl_SetParam(pAdmCtrl->pRsn->hWhalCtrl, &whalParam);
	if (status != OK)
	{
		return status;
	}

#ifdef EXC_MODULE_INCLUDED
	    
	/* set MIC and KP in HAL  */
    whalParam.paramType = HAL_CTRL_RSN_EXC_SW_ENC_ENABLE_PARAM; 
    whalParam.content.rsnExcSwEncFlag = wpaData.excKp;
    status = whalCtrl_SetParam(pAdmCtrl->pRsn->hWhalCtrl, &whalParam);
    if (status != OK)
    {
        return status;
    }
    whalParam.paramType = HAL_CTRL_RSN_EXC_MIC_FIELD_ENABLE_PARAM; 
    whalParam.content.rsnExcMicFieldFlag = wpaData.excMic;
    status = whalCtrl_SetParam(pAdmCtrl->pRsn->hWhalCtrl, &whalParam);
    
    if (status != OK)
    {
        return status;
    }
#endif /*EXC_MODULE_INCLUDED*/

    /* re-config PAE */
    status = admCtrlWpa_dynamicConfig(pAdmCtrl,&wpaData);
    if (status != OK)
    {
        return status;
    }


    return status;
}

/**
*
* admCtrlWpa_evalSite  - Evaluate site for registration.
*
* \b Description: 
*
* evaluate site RSN capabilities against the station's cap.
* If the BSS type is infrastructure, the station matches the site only if it's WEP status is same as the site
* In IBSS, it does not matter
*
* \b ARGS:
*
*  I   - pAdmCtrl - Context \n
*  I   - pRsnData - site's RSN data \n
*  O   - pEvaluation - Result of evaluation \n
*  
* \b RETURNS:
*
*  OK 
*
* \sa 
*/
TI_STATUS admCtrlWpa_evalSite(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, bssType_e bssType, UINT32 *pEvaluation)
{
    TI_STATUS               status;
    wpaIeData_t             wpaData;
    admCtrlWpa_validity_t   admCtrlWpa_validity;
    cipherSuite_e           encryptionStatus;
    UINT8                   *pWpaIe;
    UINT8                   index;

    *pEvaluation = 0;

    if (pRsnData==NULL)
    {
        return NOK;
    }
    if (pRsnData->pIe==NULL)
    {
        return NOK;
    }
    
    if (bssType != BSS_INFRASTRUCTURE)
    {
        return NOK;
    }

    /* Set initial values for admCtrlWpa_validity as none*/
    admCtrlWpa_validity = admCtrlWpa_validityTable[RSN_CIPHER_NONE][RSN_CIPHER_NONE][RSN_CIPHER_NONE];
   
    /* Check if WPA-any mode is supported and WPA2 info elem is presented */
    /* If yes - perform WPA2 site evaluation                              */
    if(pAdmCtrl->WPAMixedModeEnable && pAdmCtrl->WPAPromoteFlags)
    {
        if((admCtrl_parseIe(pAdmCtrl, pRsnData, &pWpaIe, RSN_IE_ID)== OK)  &&
           (pWpaIe != NULL))
        {
            status = admCtrlWpa2_evalSite(pAdmCtrl, pRsnData,  bssType, pEvaluation);
            if(status == OK)
                return status;
        }
    }
	
	status = admCtrl_parseIe(pAdmCtrl, pRsnData, &pWpaIe, WPA_IE_ID);
	if (status != OK)                                                         
	{                                                                                    
		return status;                                                        
	}
    status = admCtrlWpa_parseIe(pAdmCtrl, pWpaIe, &wpaData);
    if (status != OK)
    {
        return status;
    }

    /* check keyMngSuite validity */
    switch (wpaData.KeyMngSuite[0])
    {
    case WPA_IE_KEY_MNG_NONE:
        status = (pAdmCtrl->externalAuthMode <= RSN_EXT_AUTH_MODE_AUTO_SWITCH) ? OK : NOK;
        break;
    case WPA_IE_KEY_MNG_801_1X:
#ifdef EXC_MODULE_INCLUDED
	case WPA_IE_KEY_MNG_CCKM:
		/* CCKM is allowed only in 802.1x auth */
#endif
        status = (pAdmCtrl->externalAuthMode == RSN_EXT_AUTH_MODE_WPA) ? OK : NOK;
        break;
    case WPA_IE_KEY_MNG_PSK_801_1X:
        status = (pAdmCtrl->externalAuthMode == RSN_EXT_AUTH_MODE_WPAPSK) ? OK : NOK;
        break;
    default:
        status = NOK;
        break;
    }
    if (status != OK)
    {
        return status;
    }

    /*Because ckip is a proprietary encryption for Cisco then a different validity check is needed */
    if(wpaData.broadcastSuite == RSN_CIPHER_CKIP || wpaData.unicastSuite[0] ==  RSN_CIPHER_CKIP)
    {
        pAdmCtrl->getCipherSuite(pAdmCtrl, &encryptionStatus);
        if (encryptionStatus != RSN_CIPHER_TKIP) 
            return NOK;
    }
    else
    {
        /* Check cipher suite validity */
        pAdmCtrl->getCipherSuite(pAdmCtrl, &encryptionStatus);
        for (index=0; index<wpaData.unicastSuiteCnt; index++)
        {
            admCtrlWpa_validity = admCtrlWpa_validityTable[wpaData.unicastSuite[index]][wpaData.broadcastSuite][encryptionStatus];
            if (admCtrlWpa_validity.status ==OK)
            {
                break;
            }
        }

        if (admCtrlWpa_validity.status!=OK)
        {
            return admCtrlWpa_validity.status;
        }
    
        wpaData.broadcastSuite  = admCtrlWpa_validity.broadcast;
        wpaData.unicastSuite[0] = admCtrlWpa_validity.unicast;
        *pEvaluation = admCtrlWpa_validity.evaluation;
    }

    /* Check privacy bit if not in mixed mode */
    if (!pAdmCtrl->mixedMode)
    {   /* There's no mixed mode, so make sure that the privacy Bit matches the privacy mode*/
        if (((pRsnData->privacy) && (wpaData.unicastSuite[0]==RSN_CIPHER_NONE)) ||
            ((!pRsnData->privacy) && (wpaData.unicastSuite[0]>RSN_CIPHER_NONE)))
        {
            *pEvaluation = 0;
        }
    }

    /* always return OK */
    return OK;
}


/**
*
* admCtrlWpa_parseIe  - Parse an WPA information element.
*
* \b Description: 
*
* Parse an WPA information element. 
* Builds a structure of the unicast adn broadcast cihper suites,
* the key management suite and the capabilities.
*
* \b ARGS:
*
*  I   - pAdmCtrl - pointer to admCtrl context
*  I   - pWpaIe - pointer to WPA IE buffer  \n
*  O   - pWpaData - capabilities structure
*  
*  
* \b RETURNS:
*
* OK on success, NOK on failure. 
*
* \sa 
*/
TI_STATUS admCtrlWpa_parseIe(admCtrl_t *pAdmCtrl, UINT8 *pWpaIe, wpaIeData_t *pWpaData)
{

    wpaIePacket_t   *wpaIePacket = (wpaIePacket_t*)pWpaIe;
    UINT8           *curWpaIe;
    UINT8           curLength = WPA_IE_MIN_LENGTH;

    WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                            ("Wpa_IE: DEBUG: admCtrlWpa_parseIe\n\n"));

    if ((pWpaData == NULL) || (pWpaIe == NULL))
    {
        return NOK;
    }

    if ((wpaIePacket->length < WPA_IE_MIN_LENGTH) ||
        (wpaIePacket->elementid != WPA_IE_ID) ||
        (wpaIePacket->ouiType > WPA_OUI_MAX_TYPE) || (ENDIAN_HANDLE_WORD(wpaIePacket->version) > WPA_OUI_MAX_VERSION) ||               
        (os_memoryCompare(pAdmCtrl->hOs, (PUINT8)wpaIePacket->oui, wpaIeOuiIe, 3)))
    {
        WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                ("Wpa_ParseIe Error: length=0x%x, elementid=0x%x, ouiType=0x%x, version=0x%x, oui=0x%x, 0x%x, 0x%x\n", 
                                 wpaIePacket->length,wpaIePacket->elementid,
                                 wpaIePacket->ouiType, wpaIePacket->version,
                                 wpaIePacket->oui[0], wpaIePacket->oui[1],wpaIePacket->oui[2]));

        return NOK; 
    }
    /* Set default values */
    pWpaData->broadcastSuite = RSN_CIPHER_TKIP;
    pWpaData->unicastSuiteCnt = 1;
    pWpaData->unicastSuite[0] = RSN_CIPHER_TKIP;
    pWpaData->KeyMngSuiteCnt = 1;
    pWpaData->KeyMngSuite[0] = (rsn_keyMngSuite_e)WPA_IE_KEY_MNG_801_1X;
    pWpaData->bcastForUnicatst = 1;
    pWpaData->replayCounters = 1;

    pWpaData->excKp = FALSE;
    pWpaData->excMic = FALSE;


    /* Group Suite */
    if (wpaIePacket->length >= WPA_IE_GROUP_SUITE_LENGTH)
    {
        pWpaData->broadcastSuite = (cipherSuite_e)admCtrlWpa_parseSuiteVal(pAdmCtrl, (UINT8 *)wpaIePacket->groupSuite,pWpaData,RSN_CIPHER_WEP104);
        curLength = WPA_IE_GROUP_SUITE_LENGTH;
        WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                ("Wpa_IE: GroupSuite%x, broadcast %x \n",
                                 wpaIePacket->groupSuite[3], pWpaData->broadcastSuite));
    } else
    {
        return OK;
    }
    /* Unicast Suite */ 
    if (wpaIePacket->length >= WPA_IE_MIN_PAIRWISE_SUITE_LENGTH)
    {
        UINT16 pairWiseSuiteCnt = ENDIAN_HANDLE_WORD(wpaIePacket->pairwiseSuiteCnt);
        BOOL   cipherSuite[MAX_WPA_UNICAST_SUITES]={FALSE, FALSE, FALSE, FALSE, FALSE, FALSE , FALSE};
        INT32  index, unicastSuiteIndex=0;

        curWpaIe = (UINT8*)&(wpaIePacket->pairwiseSuite);
        for (index=0; (index<pairWiseSuiteCnt) && (wpaIePacket->length >= (WPA_IE_MIN_PAIRWISE_SUITE_LENGTH+(index+1)*4)); index++)
        {
            cipherSuite_e   curCipherSuite;

            curCipherSuite = (cipherSuite_e)admCtrlWpa_parseSuiteVal(pAdmCtrl, curWpaIe,pWpaData,RSN_CIPHER_WEP104);
            WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                    ("Wpa_IE: pairwiseSuite %x , unicast %x \n",
                                     curWpaIe[3], curCipherSuite));

            if ((curCipherSuite!=RSN_CIPHER_UNKNOWN) && (curCipherSuite<MAX_WPA_UNICAST_SUITES))
            {
                cipherSuite[curCipherSuite] =  TRUE;
            }
            curWpaIe +=4; 
        }
        for (index=MAX_WPA_UNICAST_SUITES-1; index>=0; index--)
        {
            if (cipherSuite[index])
            {
                pWpaData->unicastSuite[unicastSuiteIndex] = (cipherSuite_e)index;
                WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                ("Wpa_IE: unicast %x \n", pWpaData->unicastSuite[unicastSuiteIndex]));
                unicastSuiteIndex++;
            }
        }
        pWpaData->unicastSuiteCnt = unicastSuiteIndex;
        curLength = WPA_IE_MIN_KEY_MNG_SUITE_LENGTH(pairWiseSuiteCnt);
        
    } else
    {
        return OK;
    }
    /* KeyMng Suite */
    if (wpaIePacket->length >= curLength)
    {
        UINT16              keyMngSuiteCnt = ENDIAN_HANDLE_WORD(*curWpaIe);
        UINT16              index;
        rsn_keyMngSuite_e   maxKeyMngSuite = (rsn_keyMngSuite_e)WPA_IE_KEY_MNG_NONE;

        curWpaIe +=2;
        pAdmCtrl->wpaAkmExists = FALSE;
        for (index=0; (index<keyMngSuiteCnt) && (wpaIePacket->length >= (curLength+index*4)); index++)
        {
            rsn_keyMngSuite_e curKeyMngSuite;

#ifdef EXC_MODULE_INCLUDED
            curKeyMngSuite = (rsn_keyMngSuite_e)admCtrlExc_parseCckmSuiteVal(pAdmCtrl, curWpaIe);
			if (curKeyMngSuite == WPA_IE_KEY_MNG_CCKM)
			{	/* CCKM is the maximum AKM */
				maxKeyMngSuite =  curKeyMngSuite;
			}
			else
#endif
			{
				curKeyMngSuite = (rsn_keyMngSuite_e)admCtrlWpa_parseSuiteVal(pAdmCtrl, curWpaIe,pWpaData,WPA_IE_KEY_MNG_PSK_801_1X);
			}
            WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                    ("Wpa_IE: authKeyMng %x , keyMng %x \n",
                                     curWpaIe[3], curKeyMngSuite));

            if ((curKeyMngSuite>maxKeyMngSuite) && (curKeyMngSuite!=WPA_IE_KEY_MNG_NA)
				&& (curKeyMngSuite!=WPA_IE_KEY_MNG_CCKM))
            {
                maxKeyMngSuite =  curKeyMngSuite;
            }
            if (curKeyMngSuite==WPA_IE_KEY_MNG_801_1X)
            {   /* If 2 AKM exist, save also the second priority */
                pAdmCtrl->wpaAkmExists = TRUE;
            }

            curWpaIe +=4; 
        }
        pWpaData->KeyMngSuite[0] = maxKeyMngSuite;
        curLength += (index-1)*4;
        WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                ("Wpa_IE: keyMng %x \n",  pWpaData->KeyMngSuite[0]));

    } else
    {
        return OK;
    }
    /* Parse capabilities */
    if (wpaIePacket->length >= (curLength+2))
    {
        UINT16 capabilities = ENDIAN_HANDLE_WORD(*((UINT16 *)curWpaIe));

        pWpaData->bcastForUnicatst = (capabilities & WPA_GROUP_4_UNICAST_CAPABILITY_MASK) >> WPA_REPLAY_GROUP4UNI_CAPABILITY_SHIFT;
        pWpaData->replayCounters =   (capabilities & WPA_REPLAY_COUNTERS_CAPABILITY_MASK) >> WPA_REPLAY_COUNTERS_CAPABILITY_SHIFT;
        switch (pWpaData->replayCounters)
        {
        case 0: pWpaData->replayCounters=1;
            break;
        case 1: pWpaData->replayCounters=2;
            break;
        case 2: pWpaData->replayCounters=4;
            break;
        case 3: pWpaData->replayCounters=16;
            break;
        default: pWpaData->replayCounters=0;
            break;
        }
        WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
                                ("Wpa_IE: capabilities %x, bcastForUnicatst %x, replayCounters %x\n", 
                                 capabilities, pWpaData->bcastForUnicatst, pWpaData->replayCounters));

    }


    return OK;

}


UINT16 admCtrlWpa_buildCapabilities(UINT16 replayCnt)
{
    UINT16 capabilities=0;
    /* Bit1: group key for unicast */
    capabilities = 0;
    capabilities = capabilities << WPA_REPLAY_GROUP4UNI_CAPABILITY_SHIFT;
    /* Bits 2&3: Replay counter */
    switch (replayCnt)
    {
    case 1:  replayCnt=0;
        break;
    case 2:  replayCnt=1;
        break;
    case 4:  replayCnt=2;
        break;
    case 16: replayCnt=3;
        break;
    default: replayCnt=0;
        break;
    }

    capabilities |= replayCnt << WPA_REPLAY_COUNTERS_CAPABILITY_SHIFT;
    return 	capabilities;

}


UINT32  admCtrlWpa_parseSuiteVal(admCtrl_t *pAdmCtrl, UINT8* suiteVal, wpaIeData_t *pWpaData, UINT32 maxVal)
{
    UINT32  suite;

    if ((pAdmCtrl==NULL) || (suiteVal==NULL))
    {
        return RSN_CIPHER_UNKNOWN;
    }
    if (!os_memoryCompare(pAdmCtrl->hOs, suiteVal, wpaIeOuiIe, 3))
    {
        suite =  (cipherSuite_e)((suiteVal[3]<=maxVal) ? suiteVal[3] : RSN_CIPHER_UNKNOWN); 
    } else
    {
#ifdef EXC_MODULE_INCLUDED
        suite = admCtrlExc_WpaParseSuiteVal(pAdmCtrl,suiteVal,pWpaData);
#else
        suite = RSN_CIPHER_UNKNOWN;
#endif
    }
    return 	suite;
}


TI_STATUS admCtrlWpa_checkCipherSuiteValidity (cipherSuite_e unicastSuite, cipherSuite_e broadcastSuite, cipherSuite_e encryptionStatus)
{
    cipherSuite_e maxCipher;

    maxCipher = (unicastSuite>=broadcastSuite) ? unicastSuite : broadcastSuite ;
    if (maxCipher != encryptionStatus)
    {
        return NOK;
    }
    if ((unicastSuite != RSN_CIPHER_NONE) && (broadcastSuite>unicastSuite))
    {
        return NOK;
    }
    return OK;
}

static TI_STATUS admCtrlWpa_get802_1x_AkmExists (admCtrl_t *pAdmCtrl, BOOL *wpa_802_1x_AkmExists)
{
    *wpa_802_1x_AkmExists = pAdmCtrl->wpaAkmExists;
    return OK;
}



