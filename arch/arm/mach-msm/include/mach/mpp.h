/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_MPP_H
#define __ARCH_ARM_MACH_MSM_MPP_H

struct mpp {
	const char *name;
	unsigned id;
	int status;
};

/* Digital Logical Output Level */
enum {
	MPP_DLOGIC_LVL_MSME,
	MPP_DLOGIC_LVL_MSMP,
	MPP_DLOGIC_LVL_RUIM,
	MPP_DLOGIC_LVL_MMC,
	MPP_DLOGIC_LVL_VDD,
};

/* Digital Logical Output Control Value */
enum {
	MPP_DLOGIC_OUT_CTRL_LOW,
	MPP_DLOGIC_OUT_CTRL_HIGH,
	MPP_DLOGIC_OUT_CTRL_MPP,	/* MPP Output = MPP Input */
	MPP_DLOGIC_OUT_CTRL_NOT_MPP,	/* MPP Output = Inverted MPP Input */
};


#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_LAJOLLA)
typedef enum
{
  PM_MPP__I_SINK__LEVEL_5mA,
  PM_MPP__I_SINK__LEVEL_10mA,
  PM_MPP__I_SINK__LEVEL_15mA,
  PM_MPP__I_SINK__LEVEL_20mA,
  PM_MPP__I_SINK__LEVEL_25mA,
  PM_MPP__I_SINK__LEVEL_30mA,
  PM_MPP__I_SINK__LEVEL_35mA,
  PM_MPP__I_SINK__LEVEL_40mA,
  PM_MPP__I_SINK__LEVEL_INVALID
} pm_mpp_i_sink_level_type;

typedef enum
{
  PM_MPP__I_SINK__SWITCH_DIS,
  PM_MPP__I_SINK__SWITCH_ENA,
  PM_MPP__I_SINK__SWITCH_ENA_IF_MPP_HIGH,
  PM_MPP__I_SINK__SWITCH_ENA_IF_MPP_LOW,
  PM_MPP__I_SINK__SWITCH_INVALID
} pm_mpp_i_sink_switch_type;

int mpp_config_analog_sink(struct mpp *mpp, unsigned config);

#endif

#define MPP_CFG(level, control) ((((level) & 0x0FFFF) << 16) | \
				 ((control) & 0x0FFFFF))

struct mpp *mpp_get(struct device *dev, const char *id);
int mpp_config_digital_out(struct mpp *mpp, unsigned config);

#endif
