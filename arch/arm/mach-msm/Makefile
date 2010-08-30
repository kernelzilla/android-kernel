obj-y += io.o irq.o timer.o dma.o memory.o
obj-y += devices.o
obj-y += proc_comm.o
obj-y += vreg.o mpp.o
obj-y += vreg.o
obj-y += vboost.o
obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += remote_spinlock.o modem_notifier.o
obj-y += rpc_hsusb.o rpc_pmapp.o
obj-y += socinfo.o
obj-y += cpufreq.o
obj-y += nohlt.o

obj-$(CONFIG_ARCH_MSM) += acpuclock.o
obj-$(CONFIG_ARCH_MSM) += pmic.o
obj-$(CONFIG_ARCH_QSD) += acpuclock-8x50.o sirc.o
obj-$(CONFIG_ARCH_QSD) += pmic.o
obj-$(CONFIG_CPU_V6) += idle-v6.o
obj-$(CONFIG_CPU_V7) += idle-v7.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o smem_log.o
obj-$(CONFIG_MSM_SMD) += smd_ctl2.o smd_loopback.o smd_nmea.o
obj-$(CONFIG_MSM_RESET_MODEM) += reset_modem.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_RPCSERVER_HANDSET) += rpc_server_handset.o
obj-$(CONFIG_MSM_HW3D) += hw3d.o
ifdef CONFIG_MSM_N_WAY_SMSM
	obj-$(CONFIG_PM) += pm2.o
else
	obj-$(CONFIG_PM) += pm.o
endif
obj-$(CONFIG_MSM_DMA_TEST) += dma_test.o
obj-$(CONFIG_SURF_FFA_GPIO_KEYPAD) += keypad-surf-ffa.o

obj-$(CONFIG_PUPD) += powerup_info.o

obj-$(CONFIG_MACH_MOT) += board-mot.o keypad-mot.o switch-mot.o timed_vibrator.o mot_akm_cal.o misc_helper.o mot_battery_info.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o
obj-$(CONFIG_MACH_MSM7201A_FFA) += board-halibut.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o htc_acoustic.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o board-trout-wifi.o
obj-$(CONFIG_MACH_QSD8X50_SURF) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_FFA) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_COMET) += board-comet.o
obj-$(CONFIG_TROUT_H2W) += board-trout-h2w.o
obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o
obj-$(CONFIG_TROUT_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_MACH_MSM7X25_SURF) += board-msm7x25.o
obj-$(CONFIG_MACH_MSM7X25_FFA) += board-msm7x25.o
obj-$(CONFIG_MACH_QST1500_SURF) += board-msm7x25.o
obj-$(CONFIG_MACH_QST1500_FFA) += board-msm7x25.o
obj-$(CONFIG_MACH_QST1600_SURF) += board-msm7x25.o
obj-$(CONFIG_MACH_QST1600_FFA) += board-msm7x25.o
obj-$(CONFIG_MACH_MSM7X27_SURF) += board-msm7x25.o
obj-$(CONFIG_MACH_MSM7X27_FFA) += board-msm7x25.o

obj-$(CONFIG_QSD_AUDIO) += qdsp6/

obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-keypad.o board-sapphire-panel.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o board-sapphire-wifi.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o msm_vibrator.o

obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o

obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
