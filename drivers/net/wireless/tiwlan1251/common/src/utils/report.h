/** \file report.h
 *  \brief Report module API
 *
 *  \see report.c
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

/***************************************************************************/
/*                                                                          */
/*    MODULE:   report.h                                                    */
/*    PURPOSE:  Report module internal header API                           */
/*                                                                          */
/***************************************************************************/
#ifndef __REPORT_H__
#define __REPORT_H__

#include "osTIType.h"
#include "commonTypes.h"
#include "osApi.h"
#include "utils.h"

/* report handle */
    
#define MAX_STRING_LEN              128 

typedef struct 
{
    TI_HANDLE       hOs;
    tiUINT8         SeverityTable[WLAN_MAX_SEVERITIES];
    tiUINT8         ModuleTable[WLAN_MAX_LOG_MODULES];
    char            moduleDesc[WLAN_MAX_LOG_MODULES][MAX_STRING_LEN];
} report_t;


/* The report mechanism is like that:
    Each module ahas a report flag. Each severity has a severity flag.
    Only if bits are enabled, the message is printed */
/* The modules which have their report flag enable are indicated by a bit map in the reportModule 
    variable contained in the report handle*/
/* The severities which have are enabled are indicated by a bit map in the reportSeverity
    variable contained in the report handle*/

#ifdef REPORT_LOG


#define WLAN_REPORT_INIT(hReport, module, msg)                        \
    do { if (hReport && (((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_INIT])       && (((report_t *)hReport)->ModuleTable[module]))  \
       { os_report ("$B%c%s,INIT:",             ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_INFORMATION(hReport, module, msg)                 \
    do { if (hReport && (((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_INFORMATION]) && (((report_t *)hReport)->ModuleTable[module])) \
       { os_report ("$C%c%s,INFORMATION:",      ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_WARNING(hReport, module, msg)                     \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_WARNING])                                                      \
       { os_report ("$D%c%s,WARNING:",          ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_ERROR(hReport, module, msg)                       \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_ERROR])                                                        \
       { os_report ("$E%c%s,ERROR:",            ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_FATAL_ERROR(hReport, module, msg)                 \
    do { if (hReport && (((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_FATAL_ERROR]) && (((report_t *)hReport)->ModuleTable[module])) \
       { os_report ("$F%c%s,FATAL ERROR:",      ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_SM(hReport, module, msg)                          \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_SM])                                                           \
       { os_report ("$G%c%s,SM:",               ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_CONSOLE(hReport, module, msg)                     \
    do { if (hReport && (((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_CONSOLE])    && (((report_t *)hReport)->ModuleTable[module]))  \
       { os_report ("$H%c%s,CONSOLE:",          ((char)module + 'A'),    ((report_t *)hReport)->moduleDesc[module]); os_report msg; } } while(0)
#define WLAN_REPORT_DEBUG_RX(hReport, msg)                            \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_DEBUG_RX])                                                     \
       { os_report ("$AA,DEBUG RX:");                                                                                os_report msg; } } while(0)
#define WLAN_REPORT_DEBUG_TX(hReport, msg)                            \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_DEBUG_TX])                                                     \
       { os_report ("$AA,DEBUG TX:");                                                                                os_report msg; } } while(0)
#define WLAN_REPORT_DEBUG_CONTROL(hReport, msg)                       \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_DEBUG_CONTROL])                                                \
       { os_report ("$AA,DEBUG CONTROL:");                                                                           os_report msg; } } while(0)
#define WLAN_REPORT_GWSI_RECORDING(hReport, msg)                      \
    do { if (hReport &&  ((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_GWSI_RECORDING])                                               \
       { os_report ("$AA,GWSI RECORDING:");                                                                          os_report msg; } } while(0)
#define WLAN_REPORT_HEX_INFORMATION(hReport, module, data, datalen)   \
    do { if (hReport && (((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_INFORMATION]) && (((report_t *)hReport)->ModuleTable[module])) \
       { HexDumpData(data, datalen); } } while(0)
#define WLAN_REPORT_MSDU_INFORMATION(hReport, module, pMsdu, title)   \
    do { if (hReport && (((report_t *)hReport)->SeverityTable[WLAN_SEVERITY_INFORMATION]) && (((report_t *)hReport)->ModuleTable[module])) \
       { msduContentDump(pMsdu, title); } } while(0)
#define WLAN_OS_REPORT(msg)                                           \
    do { os_report("$AA"); os_report msg;} while(0)

#ifdef INIT_MESSAGES
#define WLAN_INIT_REPORT(msg)                                         \
    do { os_report("$AA"); os_report msg;} while(0)
#else
#define WLAN_INIT_REPORT(msg) 
#endif

#else   /* REPORT_LOG */


/* NOTE: Keep a dummy report function call to treat compilation warnings */


#define WLAN_REPORT_INIT(hReport,module,msg)                          \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_INFORMATION(hReport,module,msg)                   \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_WARNING(hReport,module,msg)                       \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_ERROR(hReport,module,msg)                         \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_FATAL_ERROR(hReport,module,msg)                   \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_SM(hReport,module,msg)                            \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_CONSOLE(hReport,module,msg)                       \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_HEX_INFORMATION(hReport,module,data,datalen)      \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_DEBUG_RX(hReport,msg)                             \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_DEBUG_TX(hReport,msg)                             \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_DEBUG_CONTROL(hReport,msg)                        \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_GWSI_RECORDING(hReport,msg)                       \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_REPORT_MSDU_INFORMATION(hReport,module,pMsdu,title)      \
    do { if (hReport == NULL) { } } while (0)
#define WLAN_OS_REPORT(msg)                                           \
    do { } while (0)
#define WLAN_INIT_REPORT(msg)                                         \
    do { } while (0)

#endif  /* REPORT_LOG */
                            

/* report module prototypes */
TI_HANDLE report_create (TI_HANDLE hOs);
TI_STATUS report_config (TI_HANDLE hReport, TI_HANDLE hOs, reportInitParams_t * init_params);
TI_STATUS report_unLoad (TI_HANDLE hReport);

void report_setReportModule (TI_HANDLE hReport, tiUINT8 module_index);
void report_clearReportModule (TI_HANDLE hReport, tiUINT8 module_index);
void report_getReportModuleTable (TI_HANDLE hReport, tiUINT8 *pModules); 
void report_setReportModuleTable (TI_HANDLE hReport, tiUINT8 *pModules); 
void report_setReportSeverity (TI_HANDLE hReport, tiUINT8 severity_index); 
void report_clearReportSeverity (TI_HANDLE hReport, tiUINT8 severity_index); 
void report_getReportSeverityTable (TI_HANDLE hReport, tiUINT8 *pSeverities);
void report_setReportSeverityTable (TI_HANDLE hReport, tiUINT8 *pSeverities);
TI_STATUS report_setParam (TI_HANDLE hReport, whalParamInfo_t *pParam);                   
TI_STATUS report_getParam (TI_HANDLE hReport, whalParamInfo_t *pParam); 
void report_dummy (const char* fmt, ...);                          

#endif /* __REPORT_H__ */
