ifeq (y, $(findstring y, $(CONFIG_MACH_PITTSBURGH) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_MSM_CAMERA) += msm_camera.o msm_camera_probe.o msm_v4l2.o
else
obj-$(CONFIG_MSM_CAMERA) += msm_camera.o msm_v4l2.o
endif
obj-$(CONFIG_MSM_CAMERA_FLASH) += flash.o
obj-$(CONFIG_ARCH_MSM_ARM11) += msm_vfe7x.o msm_io7x.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += msm_vfe8x.o msm_vfe8x_proc.o msm_io8x.o
obj-$(CONFIG_MT9T013) += mt9t013.o mt9t013_reg.o
obj-$(CONFIG_MT9D112) += mt9d112.o mt9d112_reg.o
obj-$(CONFIG_MT9P012) += mt9p012_reg.o
obj-$(CONFIG_MSM_CAMERA_AF_FOXCONN) += mt9p012_fox.o
obj-$(CONFIG_MSM_CAMERA_AF_BAM) += mt9p012_bam.o
obj-$(CONFIG_MSM_CAMERA_AF_KM) += mt9p012_km.o
obj-$(CONFIG_S5K3E2FX) += s5k3e2fx.o
obj-$(CONFIG_VB6801) += vb6801.o
