obj-y += adsp.o adsp_driver.o adsp_info.o
ifneq ($(CONFIG_ARCH_MSM7X30),y)
obj-y += adsp_video_verify_cmd.o
obj-y += adsp_videoenc_verify_cmd.o
obj-y += adsp_jpeg_verify_cmd.o adsp_jpeg_patch_event.o
obj-y += adsp_vfe_verify_cmd.o adsp_vfe_patch_event.o
obj-y += adsp_lpm_verify_cmd.o
obj-y += audio_out.o audio_in.o audio_mp3.o audmgr.o audpp.o audrec.o
obj-y += audio_evrc.o audio_qcelp.o audio_amrnb.o audio_aac.o audio_amrnb_in.o
ifeq (y, $(findstring y, $(CONFIG_MACH_PITTSBURGH) $(CONFIG_MACH_MOT)))
obj-y += audio_wma.o
else
obj-y += audio_wma.o audio_voicememo.o audio_pcm.o
endif
obj-y += snd.o
endif
