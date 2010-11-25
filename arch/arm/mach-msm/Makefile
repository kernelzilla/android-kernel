obj-y += io.o irq.o timer.o dma.o memory.o
obj-y += devices.o
obj-y += proc_comm.o
obj-y += vreg.o mpp.o
obj-y += vreg.o
obj-y += vboost.o
obj-y += clock.o clock-pcom.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += remote_spinlock.o modem_notifier.o
obj-y += rpc_hsusb.o rpc_pmapp.o rpc_fsusb.o
obj-y += socinfo.o
obj-y += cpufreq.o
obj-y += nohlt.o
obj-y += pmic.o

obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += acpuclock-8x50.o
obj-$(CONFIG_CPU_V6) += idle-v6.o
obj-$(CONFIG_CPU_V7) += idle-v7.o
obj-$(CONFIG_MSM_JTAG_V7) += jtag-v7.o

obj-$(CONFIG_MACH_MOT) += board-mot-7x01a.o mot_battery_info.o keypad-mot.o switch-mot.o timed_vibrator.o mot_akm_cal.o misc_helper.o
ifdef CONFIG_MACH_MOT
obj-$(CONFIG_PUPD) += powerup_info.o
endif
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MSM_SMD) += smd.o
obj-$(CONFIG_MSM_SMD_LOGGING) += smem_log.o
obj-$(CONFIG_MSM_SMD_TTY) += smd_tty.o
obj-$(CONFIG_MSM_SMD_QMI) += smd_qmi.o
obj-$(CONFIG_MSM_SMD_CTL) += smd_ctl2.o
obj-$(CONFIG_MSM_SMD_NMEA) += smd_nmea.o
obj-$(CONFIG_DEBUG_FS) += pmic_debugfs.o
obj-$(CONFIG_MSM_RESET_MODEM) += reset_modem.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_clients.o
obj-$(CONFIG_MSM_RPC_PING) += ping_mdm_rpc_client.o
obj-$(CONFIG_MSM_RPC_OEM_RAPI) += oem_rapi_client.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM7KV2_AUDIO) += qdsp5v2/
obj-$(CONFIG_MSM_RPCSERVER_HANDSET) += rpc_server_handset.o
obj-$(CONFIG_MSM_HW3D) += hw3d.o
ifdef CONFIG_MSM_N_WAY_SMSM
	obj-$(CONFIG_PM) += pm2.o
else
	obj-$(CONFIG_PM) += pm.o
endif
obj-$(CONFIG_MSM_DMA_TEST) += dma_test.o
#obj-$(CONFIG_SURF_FFA_GPIO_KEYPAD) += keypad-surf-ffa.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-panel.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-keypad.o fish_battery.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o htc_acoustic.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o board-trout-wifi.o
obj-$(CONFIG_MACH_TROUT) += devices_htc.o
obj-$(CONFIG_MACH_QSD8X50_SURF) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_FFA) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_COMET) += board-comet.o
obj-$(CONFIG_TROUT_H2W) += board-trout-h2w.o
obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o
obj-$(CONFIG_TROUT_PWRSINK) += htc_pwrsink.o
ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_KERNEL_MOTOROLA) += board-mot-7x27.o switch-mot.o mot_akm_cal.o board-mot-kpd.o
obj-$(CONFIG_MACH_CALGARY) += timed_vibrator.o
obj-$(CONFIG_MACH_PITTSBURGH) += msm_vibrator.o
else #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_ARCH_MSM7X27) += board-msm7x27.o
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_ARCH_MSM7X30) += board-msm7x30.o clock-7x30.o

obj-$(CONFIG_QSD_AUDIO) += qdsp6/

obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-keypad.o board-sapphire-panel.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o board-sapphire-wifi.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o msm_vibrator.o
obj-$(CONFIG_MACH_SAPPHIRE) += devices_htc.o

obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o

obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
ifeq ($(CONFIG_MACH_CALGARY),y)
EXTRA_CFLAGS += -Iinit
endif
