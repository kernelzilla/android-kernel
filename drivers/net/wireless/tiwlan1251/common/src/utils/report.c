/** \file report.c
 *  \brief report module implementation
 *
 *  \see report.h
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

/****************************************************************************************************/
/*                                                                                                  */
/*      MODULE:     report.c                                                                        */
/*      PURPOSE:    report module implementation                                                    */
/*                                                                                                  */
/****************************************************************************************************/
#include "osTIType.h"
#include "osApi.h"
#include "report.h"
#include "commonTypes.h"
#include "paramIn.h"
#include "utils.h"


/************************************************************************
 *                        report_create                             *
 ************************************************************************
DESCRIPTION: Report module creation function, called by the config mgr in creation phase
                performs the following:
                -   Allocate the report handle

INPUT:      hOs -           Handle to OS        


OUTPUT:     

RETURN:     Handle to the report module on success, NULL otherwise

************************************************************************/
TI_HANDLE report_create(TI_HANDLE hOs)
{
    report_t *pReport;

    pReport = os_memoryAlloc(hOs, sizeof(report_t));
    if(!pReport)
        return NULL;

    pReport->hOs = hOs;

    return(pReport);
}

/************************************************************************
 *                        report_config                                 *
 ************************************************************************
DESCRIPTION: Report module configuration function, called by the config mgr in configuration phase
                performs the following:
                -   Reset & init local variables
                -   Resets all report modules bits
                -   Resets all severities bits
                -   Init the description strings

INPUT:      hReport -   Report handle
            hOs     -   OS handle


OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS report_config(TI_HANDLE hReport, TI_HANDLE hOs, reportInitParams_t * init_params)
{
    report_t *pReport = (report_t *)hReport;

    pReport->hOs = hOs;

    os_memoryZero(NULL, pReport->SeverityTable, sizeof(pReport->SeverityTable));
    os_memoryZero(NULL, pReport->ModuleTable, sizeof(pReport->ModuleTable));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CONFIG_MGR_MODULE_LOG]), "CONFIG_MGR", sizeof("CONFIG_MGR"));
    
    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SME_SM_MODULE_LOG]), "SME_SM    ", sizeof("SME_SM    "));
    
    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SITE_MGR_MODULE_LOG]), "SITE_MGR  ", sizeof("SITE_MGR  "));
    
    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CONN_MODULE_LOG]), "CONN      ", sizeof("CONN      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[MLME_SM_MODULE_LOG]), "MLME_SM   ", sizeof("MLME_SM   "));
    
    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[AUTH_MODULE_LOG]), "AUTH      ", sizeof("AUTH      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[ASSOC_MODULE_LOG]), "ASSOC     ", sizeof("ASSOC     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[RX_DATA_MODULE_LOG]), "RX_DATA   ", sizeof("RX_DATA   "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TX_DATA_MODULE_LOG]), "TX_DATA   ", sizeof("TX_DATA   "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CTRL_DATA_MODULE_LOG]), "CTRL_DATA ", sizeof("CTRL_DATA "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[RSN_MODULE_LOG]), "RSN       ", sizeof("RSN       "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[HAL_RX_MODULE_LOG]), "HAL_RX    ", sizeof("HAL_RX    "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[HAL_TX_MODULE_LOG]), "HAL_TX    ", sizeof("HAL_TX    "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[HAL_CTRL_MODULE_LOG]), "HAL_CTRL  ", sizeof("HAL_CTRL  "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[HAL_SECURITY_MODULE_LOG]), "HAL_SEC   ", sizeof("HAL_SEC   "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[MEM_MGR_MODULE_LOG]), "MEM_MGR   ", sizeof("MEM_MGR   "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[REPORT_MODULE_LOG]), "REPORT   ", sizeof("REPORT   "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SITE_UPDATE_MODULE_LOG]), "SITE_UPDATE", sizeof("SITE_UPDATE"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[REGULATORY_DOMAIN_MODULE_LOG]), "REGULATORY_DOMAIN      ", sizeof("REGULATORY_DOMAIN      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[MEASUREMENT_MNGR_MODULE_LOG]), "MEASUREMENT MNGR ", sizeof("MEASUREMENT MNGR "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[MEASUREMENT_SRV_MODULE_LOG]), "MEASUREMENT SRV  ", sizeof("MEASUREMENT SRV  "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SOFT_GEMINI_MODULE_LOG]), "SOFT_GEMINI      ", sizeof("SOFT_GEMINI      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SC_MODULE_LOG]), "SC_MODULE_LOG      ", sizeof("SC_MODULE_LOG      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[EXC_MANAGER_MODULE_LOG]), "EXC_MANAGER      ", sizeof("EXC_MANAGER      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[ROAMING_MANAGER_MODULE_LOG]), "ROAMING_MANAGER      ", sizeof("ROAMING_MANAGER      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[QOS_MANAGER_MODULE_LOG]), "QOS_MANAGER      ", sizeof("QOS_MANAGER      "));
    
    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TRAFFIC_ADM_CTRL_MODULE_LOG]), "TRAFFIC_ADM_CTRL    ", sizeof("TRAFFIC_ADM_CTRL    "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[POWER_MANAGER_MODULE_LOG]), "POWER_MANAGER  ", sizeof("POWER_MANAGER  "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[POWER_CONTROL_MODULE_LOG]), "POWER_CONTROL  ", sizeof("POWER_CONTROL  "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[POWER_SERVER_MODULE_LOG]), "POWER_SERVER  ", sizeof("POWER_SERVER  "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[ELP_MODULE_LOG]), "ELP            ", sizeof("ELP            "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SCR_MODULE_LOG]), "SCR              ", sizeof("SCR              "));    

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SCAN_SRV_MODULE_LOG]), "SCAN SERVICE     ", sizeof("SCAN SERVICE     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SCAN_CNCN_MODULE_LOG]), "SCAN CONCENTRATOR", sizeof("SCAN CONCENTRATOR"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[SCAN_MNGR_MODULE_LOG]), "SCAN MANAGER     ", sizeof("SCAN MANAGER     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[GWSI_ADAPT_MODULE_LOG]), "GWSI_ADAPT     ", sizeof("GWSI_ADAPT     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[GWSI_ADAPT_CB_MODULE_LOG]), "GWSI_ADAPT_CB     ", sizeof("GWSI_ADAPT_CB     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CORE_ADAPT_MODULE_LOG]), "CORE_ADAPT     ", sizeof("CORE_ADAPT     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TX_HW_QUEUE_MODULE_LOG]), "TX HW QUEUE      ", sizeof("TX HW QUEUE      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TX_CTRL_BLK_MODULE_LOG]), "TX CTRL BLK      ", sizeof("TX CTRL BLK      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TX_RESULT_MODULE_LOG]), "TX RESULT      ", sizeof("TX RESULT      "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TNETW_IF_MODULE_LOG]), "TNETW IF     ", sizeof("TNETW IF     "));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TNETW_ARBITER_MODULE_LOG]), "TNETW ARBITER", sizeof("TNETW ARBITER"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CURR_BSS_MODULE_LOG]), "CURR BSS", sizeof("CURR BSS"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[FW_EVENT_MODULE_LOG]), "FW EVENT", sizeof("FW EVENT"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CMD_MBOX_MODULE_LOG]), "CMD MBOX", sizeof("CMD MBOX"));

	os_memoryCopy(hOs, (void *)(pReport->moduleDesc[CMDQUEUE_MODULE_LOG]), "CMD QUEUE", sizeof("CMD QUEUE"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[EVENT_MBOX_MODULE_LOG]), "EVENT MBOX", sizeof("EVENT MBOX"));

    os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TNETW_DRV_MODULE_LOG]), "TNETW DRV", sizeof("TNETW DRV"));

	os_memoryCopy(hOs, (void *)(pReport->moduleDesc[TNETW_XFER_MODULE_LOG]), "TX XFER      ", sizeof("TX XFER      "));

	os_memoryCopy(hOs, (void *)(pReport->moduleDesc[RECOVERY_MGR_MODULE_LOG]), "RECOVERY MGR", sizeof("RECOVERY MGR"));

	os_memoryCopy(hOs, (void *)(pReport->moduleDesc[RECOVERY_CTRL_MODULE_LOG]), "RECOVERY CTRL", sizeof("RECOVERY CTRL"));

	os_memoryCopy(hOs, (void *)(pReport->moduleDesc[HW_INIT_MODULE_LOG]), "HW INIT", sizeof("HW INIT"));

    report_setReportModuleTable( hReport, (tiUINT8 *)init_params->ModuleTable);
    report_setReportSeverityTable( hReport, (tiUINT8 *)init_params->SeverityTable);
    
    return OK;
}

/************************************************************************
 *                        report_unLoad                                 *
 ************************************************************************
DESCRIPTION: report module unload function, called by the config mgr in the unload phase
                performs the following:
                -   Free all memory allocated by the module

INPUT:      hReport -   report handle.      


OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS report_unLoad(TI_HANDLE hReport)
{
    report_t *pReport = (report_t *)hReport;

#if defined(TIWLN_WINCE30) && defined(TI_DBG)
    closeMyFile();
#endif

    utils_nullMemoryFree(pReport->hOs, pReport, sizeof(report_t));
    return OK;
}




/**
*
* report_setReportModulet
*
* \b Description:
*
* Sets the relevant bit in the reportModule variable.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - module_index - Report module index in the table \n
*
* \b RETURNS:
*
*  None.
*/
void report_setReportModule(TI_HANDLE hReport, tiUINT8 module_index)
{
    ((report_t *)hReport)->ModuleTable[module_index] = 1;
}


/**
*
* report_clearReportModule
*
* \b Description:
*
* Clears the relevant bit in the reportModule variable.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - module_index - Report module index in the table \n

*
* \b RETURNS:
*
*  None.
*/
void report_clearReportModule(TI_HANDLE hReport, tiUINT8 module_index)
{
    ((report_t *)hReport)->ModuleTable[module_index] = 0;
}


/**
*
* report_getReportModuleValue
*
* \b Description:
*
* Returns the value of reportModule.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - module array - set index  \n
*
* \b RETURNS:
*
*  Report module value
*/
void report_getReportModuleTable(TI_HANDLE hReport, tiUINT8 *pModules)
{
    tiUINT8 index;

    os_memoryCopy(NULL, (void *)pModules, (void *)(((report_t *)hReport)->ModuleTable), sizeof(((report_t *)hReport)->ModuleTable));

    for (index = 0; index < sizeof(((report_t *)hReport)->ModuleTable); index++)
    {
        pModules[index] += '0';
    }
}


/**
*
* report_setReportModuleTable
*
* \b Description:
*
* Sets the value of reportModule.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - module array - set index  \n

*
* \b RETURNS:
*
*  None.
*/
void report_setReportModuleTable(TI_HANDLE hReport, tiUINT8 *pModules)
{
    tiUINT8 index;

    for (index = 0; index < sizeof(((report_t *)hReport)->ModuleTable); index++)
    {
        pModules[index] -= '0';
    }

    os_memoryCopy(NULL, (void *)(((report_t *)hReport)->ModuleTable), (void *)pModules, sizeof(((report_t *)hReport)->ModuleTable));
}


/**
*
* report_setReportSeverity
*
* \b Description:
*
* Sets the relevant bit in the reportSeverity variable.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - severity_index - Report severity index in the table \n
*
* \b RETURNS:
*
*  None.
*/
void report_setReportSeverity(TI_HANDLE hReport, tiUINT8 severity_index)
{
   ((report_t *)hReport)->SeverityTable[severity_index] = 0;
}


/**
*
* report_clearReportSeverityBit
*
* \b Description:
*
* Clears the relevant bit in the reportSeverity variable.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - severity_index - Report severity index in the table \n
*
* \b RETURNS:
*
*  None.
*/
void report_clearReportSeverity(TI_HANDLE hReport, tiUINT8 severity_index)
{
   ((report_t *)hReport)->SeverityTable[severity_index] = 1;
}


/**
*
* report_getReportSeverityValue
*
* \b Description:
*
* Returns the value of reportSeverity.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - severity_array 
*
* \b RETURNS:
*
*  None
*/
void report_getReportSeverityTable(TI_HANDLE hReport, tiUINT8 *pSeverities)
{
    tiUINT8 index;

    os_memoryCopy(NULL, (void *)pSeverities, (void *)(((report_t *)hReport)->SeverityTable), sizeof(((report_t *)hReport)->SeverityTable));

    for (index = 0; index < sizeof(((report_t *)hReport)->SeverityTable); index++)
    {
        pSeverities[index] += '0';
    }
}

/**
*
* report_setReportSeverityValue
*
* \b Description:
*
* Sets the value of reportSeverity.
*
* \b ARGS:
*
*  I   - hReport - Report handle  \n
*  I   - severity_array 
*
* \b RETURNS:
*
*  None.
*/
void report_setReportSeverityTable(TI_HANDLE hReport, tiUINT8 *pSeverities)
{
    tiUINT8 index;

    for (index = 0; index < sizeof(((report_t *)hReport)->SeverityTable); index++)
    {
        pSeverities[index] -= '0';
    }

    os_memoryCopy(NULL, (void *)(((report_t *)hReport)->SeverityTable), (void *)pSeverities, sizeof(((report_t *)hReport)->SeverityTable));
}


/***********************************************************************
 *                        report_setParam                                   
 ***********************************************************************
DESCRIPTION: Report set param function, called by the following:
                -   config mgr in order to set a parameter from the OS abstraction layer.
                -   Form inside the driver

INPUT:      hReport -   Report handle.
            pParam  -   Pointer to the parameter        

OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS report_setParam(TI_HANDLE         hReport,
                          whalParamInfo_t   *pParam)
{
    switch((externalParam_e)pParam->paramType)
    {
    case REPORT_MODULE_ON_PARAM:
        report_setReportModule(hReport, pParam->content.ModuleTable[0]);
        break;

    case REPORT_MODULE_OFF_PARAM:
        report_clearReportModule(hReport, pParam->content.ModuleTable[0]);
        break;

    case REPORT_MODULE_TABLE_PARAM:
        report_setReportModuleTable(hReport, (tiUINT8 *)pParam->content.ModuleTable);
        break;

    case REPORT_SEVERITY_ON_PARAM:
        report_setReportSeverity(hReport, pParam->content.SeverityTable[0]);
        break;

    case REPORT_SEVERITY_OFF_PARAM:
        report_clearReportSeverity(hReport, pParam->content.SeverityTable[0]);
        break;

    case REPORT_SEVERITY_TABLE_PARAM:
        report_setReportSeverityTable(hReport, (tiUINT8 *)pParam->content.SeverityTable);
        break;
        
    case REPORT_PPMODE_VALUE_PARAM:
        os_setDebugMode((BOOL)pParam->content.reportPPMode);
        break;
    default:
        WLAN_REPORT_ERROR(hReport, REPORT_MODULE_LOG, ("Set param, Params is not supported, %d\n", pParam->paramType));
        return PARAM_NOT_SUPPORTED;
    }

    return OK;
}

/***********************************************************************
 *                        report_getParam                                   
 ***********************************************************************
DESCRIPTION: Report get param function, called by the following:
            -   config mgr in order to get a parameter from the OS abstraction layer.
            -   from inside the driver

INPUT:      hReport -   Report handle.
            pParam  -   Pointer to the parameter        

OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS report_getParam(TI_HANDLE         hReport,
                          whalParamInfo_t   *pParam)
{

    switch((externalParam_e)pParam->paramType)
    {
    case REPORT_MODULE_TABLE_PARAM:
        report_getReportModuleTable(hReport, (tiUINT8 *)pParam->content.ModuleTable);
        break;

    case REPORT_SEVERITY_TABLE_PARAM:
        report_getReportSeverityTable(hReport, (tiUINT8 *)pParam->content.SeverityTable);
        break;

    default:
        WLAN_REPORT_ERROR(hReport, REPORT_MODULE_LOG, ("Get param, Params is not supported, %d\n", pParam->paramType));
        return PARAM_NOT_SUPPORTED;
    }

    return OK;
}


/***********************************************************************
 *                        report_dummy                                   
 ***********************************************************************
DESCRIPTION: Dummy function used to solve warning problems 
             when REPORT_LOG flag is disabled

INPUT:      

OUTPUT:     

RETURN:     

************************************************************************/
void report_dummy (const char* fmt, ...)
{
}
