/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/hardirq.h>
#include <asm/bug.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
#include <linux/semaphore.h>
#include <mach/resource.h>
#else 
#include <asm/semaphore.h>
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))	
#include <asm/arch/resource.h>
#endif 
#endif 

#include "sgxdefs.h"
#include "services_headers.h"
#include "sysinfo.h"
#include "sgxapi_km.h"
#include "sysconfig.h"
#include "sgxinfokm.h"
#include "syslocal.h"

#define	HZ_TO_MHZ(m) ((m) / 1000000)

#if defined(SGX_DYNAMIC_TIMING_INFO)
static inline IMG_UINT32 scale_by_rate(IMG_UINT32 val, IMG_UINT32 rate1, IMG_UINT32 rate2)
{
	if (rate1 >= rate2)
	{
		return val * (rate1 / rate2);
	}

	return val / (rate2 / rate1);
}

static inline IMG_UINT32 scale_prop_to_SGX_clock(IMG_UINT32 val, IMG_UINT32 rate)
{
	return scale_by_rate(val, rate, SYS_SGX_CLOCK_SPEED);
}

static inline IMG_UINT32 scale_inv_prop_to_SGX_clock(IMG_UINT32 val, IMG_UINT32 rate)
{
	return scale_by_rate(val, SYS_SGX_CLOCK_SPEED, rate);
}

IMG_VOID SysGetSGXTimingInformation(SGX_TIMING_INFORMATION *psTimingInfo)
{
	IMG_UINT32 rate;

#if defined(NO_HARDWARE)
	rate = SYS_SGX_CLOCK_SPEED;
#else
	PVR_ASSERT(gpsSysSpecificData->bSGXClocksEnabled);

	rate = clk_get_rate(gpsSysSpecificData->psSGX_FCK);
	PVR_ASSERT(rate != 0);
#endif
	psTimingInfo->ui32CoreClockSpeed = rate;
	psTimingInfo->ui32HWRecoveryFreq = scale_prop_to_SGX_clock(SYS_SGX_HWRECOVERY_TIMEOUT_FREQ, rate);
	psTimingInfo->ui32uKernelFreq = scale_prop_to_SGX_clock(SYS_SGX_PDS_TIMER_FREQ, rate); 
	psTimingInfo->ui32ActivePowManLatencyms = SYS_SGX_ACTIVE_POWER_LATENCY_MS; 
}
#endif 

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))	
#if !defined(SGX_DYNAMIC_TIMING_INFO)
#error "SGX_DYNAMIC_TIMING_INFO must be defined for this platform"
#endif

static IMG_INT vdd2_post_func(struct notifier_block *n, IMG_UINT32 event, IMG_VOID *ptr)
{
	PVR_UNREFERENCED_PARAMETER(n);
	PVR_UNREFERENCED_PARAMETER(event);
	PVR_UNREFERENCED_PARAMETER(ptr);

	if (gpsSysSpecificData->bSGXClocksEnabled && gpsSysSpecificData->bSGXInitComplete)
	{
#if defined(DEBUG)
		IMG_UINT32 rate;

		rate = clk_get_rate(gpsSysSpecificData->psSGX_FCK);

		PVR_ASSERT(rate != 0);

		PVR_TRACE(("%s: SGX clock rate: %dMHz", __FUNCTION__, HZ_TO_MHZ(rate)));
#endif
		PVRSRVDevicePostClockSpeedChange(gpsSysSpecificData->psSGXDevNode->sDevId.ui32DeviceIndex, IMG_TRUE, IMG_NULL);
	}

	return 0;
}

static IMG_INT vdd2_pre_func(struct notifier_block *n, IMG_UINT32 event, IMG_VOID *ptr)
{
	PVR_UNREFERENCED_PARAMETER(n);
	PVR_UNREFERENCED_PARAMETER(event);
	PVR_UNREFERENCED_PARAMETER(ptr);

	if (gpsSysSpecificData->bSGXClocksEnabled && gpsSysSpecificData->bSGXInitComplete)
	{
#if 0
		IMG_UINT32 rate;

		rate = clk_get_rate(gpsSysSpecificData->psSGX_FCK);

		PVR_ASSERT(rate != 0);

		PVR_TRACE(("%s: SGX clock rate: %dMHz", __FUNCTION__, HZ_TO_MHZ(rate)));
#endif
		PVRSRVDevicePreClockSpeedChange(gpsSysSpecificData->psSGXDevNode->sDevId.ui32DeviceIndex, IMG_TRUE, IMG_NULL);
	}

	return 0;
}

static struct notifier_block vdd2_pre = {
	vdd2_pre_func,
	 NULL
};

static struct notifier_block vdd2_post = {
	vdd2_post_func,
	 NULL
};

#ifdef K27
static struct constraint_id cnstr_id_vdd2 = {
	.type = RES_OPP_CO,
	.data = (IMG_VOID *)"vdd2_opp"
};
#endif

static IMG_VOID RegisterConstraintNotifications(IMG_VOID)
{
	PVR_TRACE(("Registering constraint notifications"));

#ifdef K27
	constraint_register_pre_notification(gpsSysSpecificData->pVdd2Handle, &vdd2_pre,
						max_vdd2_opp+1);

	constraint_register_post_notification(gpsSysSpecificData->pVdd2Handle, &vdd2_post,
						max_vdd2_opp+1);

#endif
	PVR_TRACE(("VDD2 constraint notifications registered"));
}

static IMG_VOID UnRegisterConstraintNotifications(IMG_VOID)
{
	PVR_TRACE(("Unregistering constraint notifications"));

#ifdef K27
	constraint_unregister_pre_notification(gpsSysSpecificData->pVdd2Handle, &vdd2_pre,
						max_vdd2_opp+1);

	constraint_unregister_post_notification(gpsSysSpecificData->pVdd2Handle, &vdd2_post,
						max_vdd2_opp+1);
#endif
}

#endif 

PVRSRV_ERROR EnableSGXClocks(SYS_DATA *psSysData)
{
#if !defined(NO_HARDWARE)
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;
	IMG_UINT32 rate;
	IMG_INT res;

	
	if (psSysSpecData->bSGXClocksEnabled)
	{
		return PVRSRV_OK;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "EnableSGXClocks: Enabling SGX Clocks"));

#if defined(DEBUG)
	rate = clk_get_rate(psSysSpecData->psMPU_CK);
	PVR_DPF((PVR_DBG_MESSAGE, "CPU Clock is %dMhz", HZ_TO_MHZ(rate)));
#endif

	res = clk_enable(psSysSpecData->psSGX_FCK);
	if (res < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSGXClocks: Couldn't enable SGX functional clock (%d)", res));
		return PVRSRV_ERROR_GENERIC;
	}

	res = clk_enable(psSysSpecData->psSGX_ICK); 
	if (res < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSGXClocks: Couldn't enable SGX interface clock (%d)", res));

		clk_disable(psSysSpecData->psSGX_FCK);
		return PVRSRV_ERROR_GENERIC;
	}

	rate = clk_get_rate(psSysSpecData->psSGX_FCK);
	if(rate < SYS_SGX_CLOCK_SPEED)
	{
		PVR_TRACE(("SGX Functional Clock rate is %dMhz. Attempting to set to %dMhz", HZ_TO_MHZ(rate), HZ_TO_MHZ(SYS_SGX_CLOCK_SPEED)));
		res = clk_set_rate(psSysSpecData->psSGX_FCK, SYS_SGX_CLOCK_SPEED);
		if (res < 0)
		{
//			PVR_DPF((PVR_DBG_WARNING, "EnableSGXClocks: Couldn't set SGX Functional Clock rate (%d)", res));
		}
	}
	PVR_DPF((PVR_DBG_MESSAGE, "SGX Functional Clock rate is %dMhz", HZ_TO_MHZ(clk_get_rate(psSysSpecData->psSGX_FCK))));

	
	psSysSpecData->bSGXClocksEnabled = IMG_TRUE;

#else	
	PVR_UNREFERENCED_PARAMETER(psSysData);
#endif	
	return PVRSRV_OK;
}


IMG_VOID DisableSGXClocks(SYS_DATA *psSysData)
{
#if !defined(NO_HARDWARE)
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

	
	if (!psSysSpecData->bSGXClocksEnabled)
	{
		return;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "DisableSGXClocks: Disabling SGX Clocks"));

	if (psSysSpecData->psSGX_ICK)
	{
		clk_disable(psSysSpecData->psSGX_ICK); 
	}

	if (psSysSpecData->psSGX_FCK)
	{
		clk_disable(psSysSpecData->psSGX_FCK);
	}

	
	psSysSpecData->bSGXClocksEnabled = IMG_FALSE;

#else	
	PVR_UNREFERENCED_PARAMETER(psSysData);
#endif	
}

PVRSRV_ERROR EnableSystemClocks(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;
	struct clk *psCLK;
	IMG_INT res;
#if !defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	PVRSRV_ERROR eError;
#endif
#if defined(DEBUG) || defined(TIMING)
	IMG_INT rate;
	struct clk *sys_ck;
	IMG_CPU_PHYADDR     TimerRegPhysBase;
	IMG_HANDLE hTimerEnable;
	IMG_UINT32 *pui32TimerEnable;

#endif	

	PVR_TRACE(("EnableSystemClocks: Enabling System Clocks"));

	if (!psSysSpecData->bSysClocksOneTimeInit)
	{
		psCLK = clk_get(NULL, "core_ck");
		if (IS_ERR(psCLK))
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSsystemClocks: Couldn't get Core Clock"));
			goto ExitError;
		}
		psSysSpecData->psCORE_CK = psCLK;

		psCLK = clk_get(NULL, "sgx_fck");
		if (IS_ERR(psCLK))
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSsystemClocks: Couldn't get SGX Functional Clock"));
			goto ExitError;
		}
		psSysSpecData->psSGX_FCK = psCLK;

		psCLK = clk_get(NULL, "sgx_ick");
		if (IS_ERR(psCLK))
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get SGX Interface Clock"));
			goto ExitError;
		}
		psSysSpecData->psSGX_ICK = psCLK;

#if defined(DEBUG)
		psCLK = clk_get(NULL, "mpu_ck");
		if (IS_ERR(psCLK))
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get MPU Clock"));
			goto ExitError;
		}
		psSysSpecData->psMPU_CK = psCLK;
#endif
		res = clk_set_parent(psSysSpecData->psSGX_FCK, psSysSpecData->psCORE_CK);
		if (res < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't set SGX parent clock (%d)", res));
			goto ExitError;
		}
	
		psSysSpecData->bSysClocksOneTimeInit = IMG_TRUE;
	}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))
#ifdef K27
	psSysSpecData->pVdd2Handle = constraint_get("pvrsrvkm", &cnstr_id_vdd2);
	if (IS_ERR(psSysSpecData->pVdd2Handle))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get VDD2 constraint handle"));
		goto ExitError;
	}

	RegisterConstraintNotifications();
#endif
#endif

#if !defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	
	eError = EnableSGXClocks(psSysData);
	if (eError != PVRSRV_OK)
	{
		goto ExitUnRegisterConstraintNotifications;
	}
#endif

#if defined(DEBUG) || defined(TIMING)
	
	psCLK = clk_get(NULL, "gpt11_fck");
	if (IS_ERR(psCLK))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get GPTIMER11 functional clock"));
		goto ExitDisableSGXClocks;
	}
	psSysSpecData->psGPT11_FCK = psCLK;
	
	psCLK = clk_get(NULL, "gpt11_ick");
	if (IS_ERR(psCLK))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get GPTIMER11 interface clock"));
		goto ExitDisableSGXClocks;
	}
	psSysSpecData->psGPT11_ICK = psCLK;

	sys_ck = clk_get(NULL, "sys_ck");
	if (IS_ERR(sys_ck))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get System clock"));
		goto ExitDisableSGXClocks;
	}

	if(clk_get_parent(psSysSpecData->psGPT11_FCK) != sys_ck)
	{
		PVR_TRACE(("Setting GPTIMER11 parent to System Clock"));
		res = clk_set_parent(psSysSpecData->psGPT11_FCK, sys_ck);
		if (res < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't set GPTIMER11 parent clock (%d)", res));
		goto ExitDisableSGXClocks;
		}
	}

	rate = clk_get_rate(psSysSpecData->psGPT11_FCK);
	PVR_TRACE(("GPTIMER11 clock is %dMHz", HZ_TO_MHZ(rate)));
	
	res = clk_enable(psSysSpecData->psGPT11_FCK);
	if (res < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't enable GPTIMER11 functional clock (%d)", res));
		goto ExitDisableSGXClocks;
	}

	res = clk_enable(psSysSpecData->psGPT11_ICK);
	if (res < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't enable GPTIMER11 interface clock (%d)", res));
		goto ExitDisableGPT11FCK;
	}
	
	
	TimerRegPhysBase.uiAddr = SYS_OMAP3430_GP11TIMER_TSICR_SYS_PHYS_BASE;
	pui32TimerEnable = OSMapPhysToLin(TimerRegPhysBase,
                  4,
                  PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
                  &hTimerEnable);

	if (pui32TimerEnable == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: OSMapPhysToLin failed"));
		goto ExitDisableGPT11ICK;
	}

	rate = *pui32TimerEnable;
	if(!(rate & 4))
	{
		PVR_TRACE(("Setting GPTIMER11 mode to posted (currently is non-posted)"));
		
		
		*pui32TimerEnable = rate | 4;
	}

	OSUnMapPhysToLin(pui32TimerEnable,
		    4,
		    PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
		    hTimerEnable);

	
	TimerRegPhysBase.uiAddr = SYS_OMAP3430_GP11TIMER_ENABLE_SYS_PHYS_BASE;
	pui32TimerEnable = OSMapPhysToLin(TimerRegPhysBase,
                  4,
                  PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
                  &hTimerEnable);

	if (pui32TimerEnable == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: OSMapPhysToLin failed"));
		goto ExitDisableGPT11ICK;
	}

	
	*pui32TimerEnable = 3;

	OSUnMapPhysToLin(pui32TimerEnable,
		    4,
		    PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
		    hTimerEnable);

#endif 

	return PVRSRV_OK;

#if defined(DEBUG) || defined(TIMING)
ExitDisableGPT11ICK:
	clk_disable(psSysSpecData->psGPT11_ICK);
ExitDisableGPT11FCK:
	clk_disable(psSysSpecData->psGPT11_FCK);
ExitDisableSGXClocks:
#if !defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	DisableSGXClocks(psSysData);
#endif
#endif	
#if !defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
ExitUnRegisterConstraintNotifications:
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))
	UnRegisterConstraintNotifications();
#ifdef K27
	constraint_put(psSysSpecData->pVdd2Handle);
#endif
#endif	
ExitError:
	return PVRSRV_ERROR_GENERIC;
}

IMG_VOID DisableSystemClocks(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;
#if defined(DEBUG) || defined(TIMING)
	IMG_CPU_PHYADDR TimerRegPhysBase;
	IMG_HANDLE hTimerDisable;
	IMG_UINT32 *pui32TimerDisable;
#endif	

	PVR_TRACE(("DisableSystemClocks: Disabling System Clocks"));

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))

	UnRegisterConstraintNotifications();

#endif
	
	DisableSGXClocks(psSysData);

#if defined(DEBUG) || defined(TIMING)
	
	TimerRegPhysBase.uiAddr = SYS_OMAP3430_GP11TIMER_ENABLE_SYS_PHYS_BASE;
	pui32TimerDisable = OSMapPhysToLin(TimerRegPhysBase,
				4,
				PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
				&hTimerDisable);
	
	if (pui32TimerDisable == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "DisableSystemClocks: OSMapPhysToLin failed"));
	}
	else
	{
		*pui32TimerDisable = 0;
		
		OSUnMapPhysToLin(pui32TimerDisable,
				4,
				PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
				hTimerDisable);
	}

	clk_disable(psSysSpecData->psGPT11_ICK);

	clk_disable(psSysSpecData->psGPT11_FCK);

#endif 
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))

#ifdef K27
	constraint_put(psSysSpecData->pVdd2Handle);
#endif

#endif	
}
