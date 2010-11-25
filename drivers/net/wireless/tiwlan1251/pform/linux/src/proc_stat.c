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


/* Dm: #include <linux/config.h> */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/proc_fs.h>

#include "healthMonitor.h"
#include "whalCtrl.h" 
#include "osTIType.h"
#include "configMgr.h"

int proc_stat_res_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data);
static struct proc_dir_entry *res; 

int proc_stat_init(TI_HANDLE     *pHandle)
{
    configMgr_t         *pConfigManager  = (configMgr_t *)pHandle;
    res = proc_mkdir("tiwlan", NULL);
    create_proc_read_entry("tiwlan0_proc_stat", 0, res, proc_stat_res_read_proc, pConfigManager->hHealthMonitor);
    return 0;
}

int proc_stat_res_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len=0;
    healthMonitor_t  *pHealthMonitor = (healthMonitor_t*) data;
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)pHealthMonitor->hHalCtrl;
    whalCtrl_hwStatus_t *pHwStatus = &pWhalCtrl->pHwCtrl->HwStatus;

    count -= 80; /* some reserve */

    len += (len<count)?sprintf(page+len,"-------------- STA Health Configuration ---------------\n"):0;
    len += (len<count)?sprintf(page+len,"Full recovery enabled      = %d\n",pHealthMonitor->bFullRecoveryEnable):0;
    len += (len<count)?sprintf(page+len,"Timer interval             = %d msec\n",pHealthMonitor->timerInterval):0;
    len += (len<count)?sprintf(page+len,"\n"):0;


    len += (len<count)?sprintf(page+len,"-------------- STA Health Failure Statistics ---------------\n"):0;
    len += (len<count)?sprintf(page+len,"Health test perfomred      = %d\n",pHealthMonitor->numOfHealthTests):0;
    len += (len<count)?sprintf(page+len,"Full recovery performed    = %d\n",pHealthMonitor->numOfRecoveryPerformed):0;
    len += (len<count)?sprintf(page+len,"No scan complete failure   = %d\n",pHealthMonitor->recoveryTriggersNumber[ NO_SCAN_COMPLETE_FAILURE ]):0;
    len += (len<count)?sprintf(page+len,"Mailbox failure            = %d\n",pHealthMonitor->recoveryTriggersNumber[ MBOX_FAILURE ]):0;
    len += (len<count)?sprintf(page+len,"HW awake failure           = %d\n",pHealthMonitor->recoveryTriggersNumber[ HW_AWAKE_FAILURE ]):0;
    len += (len<count)?sprintf(page+len,"Bus error           		= %d\n",pHealthMonitor->recoveryTriggersNumber[ BUS_ERROR ]):0;
    len += (len<count)?sprintf(page+len,"Device error               = %d\n",pHealthMonitor->recoveryTriggersNumber[ DEVICE_ERROR ]):0;
    len += (len<count)?sprintf(page+len,"Tx Stuck Errors            = %d\n",pHealthMonitor->recoveryTriggersNumber[ TX_STUCK ]):0;
    len += (len<count)?sprintf(page+len,"Disconnect timeouts        = %d\n",pHealthMonitor->recoveryTriggersNumber[ DISCONNECT_TIMEOUT ]):0;
	len += (len<count)?sprintf(page+len,"Power save failures        = %d\n",pHealthMonitor->recoveryTriggersNumber[ POWER_SAVE_FAILURE ]):0;
	len += (len<count)?sprintf(page+len,"measurement failures       = %d\n",pHealthMonitor->recoveryTriggersNumber[ MEASUREMENT_FAILURE ]):0;

    len += (len<count)?sprintf(page+len,"--------------- whalCtrl_PrintHwStatus ---------------\n\n"):0;
    len += (len<count)?sprintf(page+len,"NumMboxErrDueToPeriodicBuiltInTestCheck = %d\n", pHwStatus->NumMboxErrDueToPeriodicBuiltInTestCheck):0;
    len += (len<count)?sprintf(page+len,"NumMboxFailures = %d\n", pHwStatus->NumMboxFailures):0;
    len += (len<count)?sprintf(page+len, "\n"):0;
    *eof = 1;
    return len;
}

int proc_stat_destroy(void)
{
    remove_proc_entry("tiwlan0_proc_stat", res);
    remove_proc_entry("tiwlan", NULL);
    return 0;
}





