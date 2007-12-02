obj-y += io.o idle.o irq.o timer.o dma.o
obj-y += devices.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += acpuclock.o
obj-y += clock.o clock-7x01a.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o

obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_MSM_CPU_FREQ_SCREEN) += cpufreq.o


obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o
obj-$(CONFIG_MACH_TROUT) += board-dream.o
