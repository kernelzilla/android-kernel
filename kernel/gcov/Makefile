#
# Makefile for GCOV profiling kernel module
#

# 2006-03-07, anderson.lizardo@gmail.com:
# Disable coverage for the gcov-kernel module itself. In the ARM platform, it
# generates a soft-lockup while trying to read the data from /proc/gcov.
KBUILD_CFLAGS := $(KBUILD_CFLAGS_NOGCOV)

obj-$(CONFIG_GCOV_PROFILE) += gcov-core.o
obj-$(CONFIG_GCOV_PROC)	   += gcov-proc.o

CFLAGS_gcov-core.o := -DGCOV_SRC_PATH='"$(srctree)"' -DGCOV_OBJ_PATH='"$(objtree)"'
