#
ccflags-y = -DLINUX -D__linux__ -Idrivers/gpu/pvr -Idrivers/gpu/pvr/omap3430 \
	-DANDROID \
	-DPVR_BUILD_DIR="\"omap3430_android\"" \
	-DSGX_DYNAMIC_TIMING_INFO \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSERVICES4 \
	-DPVR2D_VALIDATE_INPUT_PARAMS \
	-DPVR_PROC_USE_SEQ_FILE \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_PERCONTEXT_PB \
	-DTRANSFER_QUEUE \
	-DSYS_USING_INTERRUPTS \
	-DSUPPORT_HW_RECOVERY \
	-DPVR_SECURE_HANDLES \
	-DPVR_SECURE_FD_EXPORT \
	-DSUPPORT_SGX_EVENT_OBJECT \
	-DLDM_PLATFORM \
	-DPVR2D_ALT_2DHW \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	-DPVR_LINUX_MISR_USING_WORKQUEUE \
	-DSUPPORT_CPU_CACHED_BUFFERS \
	-DSUPPORT_CACHEFLUSH_ON_ALLOC \
	-DSUPPORT_MEMINFO_IDS \
	-DDISPLAY_CONTROLLER=omaplfb \
	-DSYS_SGX_ACTIVE_POWER_LATENCY_MS=100 \
	-DPVR_BUILD_DATE=""

ccflags-$(CONFIG_PVR_APM) += -DSUPPORT_ACTIVE_POWER_MANAGEMENT

ccflags-$(CONFIG_SGX_EDM_TRACING) += -DPVRSRV_USSE_EDM_STATUS_DEBUG

ccflags-$(CONFIG_SGX_MK_TRACE) += -DPVRSRV_DUMP_MK_TRACE

ccflags-$(CONFIG_SGX_RELEASE_LOGGING) += -DPVRSRV_NEED_PVR_DPF \
					 -DPVRSRV_NEED_PVR_ASSERT

ccflags-$(CONFIG_SGX_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\""  -DRELEASE

ccflags-$(CONFIG_SGX_BUILD_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\""  -DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-DDLL_METRIC=1

ccflags-$(CONFIG_SGX) += -Idrivers/gpu/pvr/sgx \
	-DSGX_CORE_REV=121 \
	-DSUPPORT_SGX \
	-DSGX530 -DSUPPORT_SGX530

pvr_common-y := \
	osfunc.o \
	mutils.o \
	mmap.o \
	module.o \
	pdump.o \
	proc.o \
	pvr_bridge_k.o \
	pvr_debug.o \
	mm.o \
	mutex.o \
	event.o \
	osperproc.o \
	buffer_manager.o \
	devicemem.o \
	deviceclass.o \
	handle.o \
	hash.o \
	metrics.o \
	pvrsrv.o \
	queue.o \
	ra.o \
	resman.o \
	power.o \
	mem.o \
	pdump_common.o \
	bridged_support.o \
	bridged_pvr_bridge.o \
	perproc.o \
	lists.o \
	omap3430/sysconfig.o \
	omap3430/sysutils.o


sgx-y    :=  \
	sgx/bridged_sgx_bridge.o \
	sgx/sgxinit.o \
	sgx/sgxpower.o \
	sgx/sgxreset.o \
	sgx/sgxutils.o \
	sgx/sgxkick.o \
	sgx/sgxtransfer.o \
	sgx/mmu.o \
	sgx/pb.o

omaplfb-y := \
	omaplfb/omaplfb_displayclass.o \
	omaplfb/omaplfb_linux.o

obj-$(CONFIG_SGX)	+= pvr_common.o sgx.o
obj-$(CONFIG_PVR_OMAPLFB) += omaplfb.o
