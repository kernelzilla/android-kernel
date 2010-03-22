/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Qualcomm PMIC8058 driver header file
 *
 */

#include <linux/irq.h>
#include <linux/mfd/core.h>

/* PM8058 interrupt numbers */
#define PM8058_FIRST_IRQ	PMIC8058_IRQ_BASE

#define PM8058_FIRST_GPIO_IRQ	PM8058_FIRST_IRQ
#define PM8058_FIRST_MPP_IRQ	(PM8058_FIRST_GPIO_IRQ + NR_PMIC8058_GPIO_IRQS)
#define PM8058_FIRST_MISC_IRQ	(PM8058_FIRST_MPP_IRQ + NR_PMIC8058_MPP_IRQS)

#define	PM8058_IRQ_KEYPAD	(PM8058_FIRST_MISC_IRQ)
#define	PM8058_IRQ_KEYSTUCK	(PM8058_FIRST_MISC_IRQ + 1)
#define PM8058_IRQ_CHGVAL	(PM8058_FIRST_MISC_IRQ + 2)

#define PM8058_IRQS		NR_PMIC8058_IRQS

#define PM8058_GPIOS		NR_PMIC8058_GPIO_IRQS
#define PM8058_MPPS		NR_PMIC8058_MPP_IRQS

/* IRQ# for GPIO number = 1 .. PM8058_GPIOS */
#define PM8058_IRQ_GPIO(n)	(PM8058_FIRST_GPIO_IRQ + (n) - 1)

#define PM8058_MAX_SUBDEVICES	16

struct pm8058_chip;

struct pm8058_platform_data {
	/* This table is only needed for misc interrupts. */
	unsigned int	pm_irqs[PM8058_IRQS];	/* block*8 + bit-pos */
	int 		(*init)(struct pm8058_chip *pm_chip);

	int		num_subdevs;
	struct mfd_cell sub_devices[PM8058_MAX_SUBDEVICES];
};

/* GPIO parameters */
/* direction */
#define	PM_GPIO_DIR_OUT			0x01
#define	PM_GPIO_DIR_IN			0x02
#define	PM_GPIO_DIR_BOTH		(PM_GPIO_DIR_OUT | PM_GPIO_DIR_IN)

/* output_buffer */
#define	PM_GPIO_OUT_BUF_OPEN_DRAIN	1
#define	PM_GPIO_OUT_BUF_CMOS		0

/* pull */
#define	PM_GPIO_PULL_UP_30		0
#define	PM_GPIO_PULL_UP_1P5		1
#define	PM_GPIO_PULL_UP_31P5		2
#define	PM_GPIO_PULL_UP_1P5_30		3
#define	PM_GPIO_PULL_DN			4
#define	PM_GPIO_PULL_NO			5

/* vin_sel: Voltage Input Select */
#define	PM_GPIO_VIN_VPH			0
#define	PM_GPIO_VIN_BB			1
#define	PM_GPIO_VIN_S3			2
#define	PM_GPIO_VIN_L3			3
#define	PM_GPIO_VIN_L7			4
#define	PM_GPIO_VIN_L6			5
#define	PM_GPIO_VIN_L5			6
#define	PM_GPIO_VIN_L2			7

/* out_strength */
#define	PM_GPIO_STRENGTH_NO		0
#define	PM_GPIO_STRENGTH_HIGH		1
#define	PM_GPIO_STRENGTH_MED		2
#define	PM_GPIO_STRENGTH_LOW		3

/* function */
#define	PM_GPIO_FUNC_NORMAL		0
#define	PM_GPIO_FUNC_PAIRED		1
#define	PM_GPIO_FUNC_1			2
#define	PM_GPIO_FUNC_2			3
#define	PM_GPIO_DTEST1			4
#define	PM_GPIO_DTEST2			5
#define	PM_GPIO_DTEST3			6
#define	PM_GPIO_DTEST4			7

struct pm8058_gpio {
	int		direction;
	int		output_buffer;
	int		output_value;
	int		pull;
	int		vin_sel;	/* 0..7 */
	int		out_strength;
	int		function;
	int		inv_int_pol;	/* invert interrupt polarity */
};

int pm8058_read(struct pm8058_chip *pm_chip, u16 addr, u8 *values,
		unsigned int len);
int pm8058_write(struct pm8058_chip *pm_chip, u16 addr, u8 *values,
		 unsigned int len);

int pm8058_gpio_config(int gpio, struct pm8058_gpio *param);

int pm8058_gpio_config_h(struct pm8058_chip *pm_chip, int gpio,
			 struct pm8058_gpio *param);
int pm8058_gpio_set_direction(struct pm8058_chip *pm_chip,
			      unsigned gpio, int direction);
int pm8058_gpio_set(struct pm8058_chip *pm_chip, unsigned gpio, int value);
int pm8058_gpio_get(struct pm8058_chip *pm_chip, unsigned gpio);

int pm8058_mpp_get(struct pm8058_chip *pm_chip, unsigned mpp);

int pm8058_gpio_config_kypd_drv(int gpio_start, int num_gpios);
int pm8058_gpio_config_kypd_sns(int gpio_start, int num_gpios);

int pm8058_rev_is_a0(struct pm8058_chip *pm_chip);
int pm8058_rev_is_b0(struct pm8058_chip *pm_chip);

