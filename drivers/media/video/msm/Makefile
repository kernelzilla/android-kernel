obj-$(CONFIG_MT9T013) += mt9t013.o mt9t013_reg.o
obj-$(CONFIG_MT9D112) += mt9d112.o mt9d112_reg.o
obj-$(CONFIG_MT9P012) += mt9p012_fox.o mt9p012_reg.o
ifeq ($(CONFIG_MSM_CAMERA_OLD),y)
obj-$(CONFIG_MSM_CAMERA) += msm_camera_old.o
obj-$(CONFIG_ARCH_MSM7X00A) += msm_vfe7x_old.o msm_io7x.o
else
obj-$(CONFIG_MSM_CAMERA) += msm_camera.o
obj-$(CONFIG_ARCH_MSM7X00A) += msm_vfe7x.o msm_io7x.o
endif
obj-$(CONFIG_MSM_CAMERA_V4L2) += msm_v4l2.o
obj-$(CONFIG_S5K3E2FX) += s5k3e2fx.o
obj-$(CONFIG_ARCH_QSD8X50) += msm_vfe8x.o msm_vfe8x_proc.o msm_io8x.o
