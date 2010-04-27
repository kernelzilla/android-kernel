/*
 *  isa1200.h - ISA1200 Haptic Motor driver
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_ISA1200_H
#define __LINUX_ISA1200_H

#define ISA1200_SCTRL0		0x00
#define ISA1200_THSWRST		(1 << 7)
#define ISA1200_EXT2DIV		(1 << 4)
#define ISA1200_LDOADJ_24V	(0x9 << 0)
#define ISA1200_LDOADJ_25V	(0xa << 0)
#define ISA1200_LDOADJ_26V	(0xb << 0)
#define ISA1200_LDOADJ_27V	(0xc << 0)
#define ISA1200_LDOADJ_28V	(0xd << 0)
#define ISA1200_LDOADJ_29V	(0xe << 0)
#define ISA1200_LDOADJ_30V	(0xf << 0)
#define ISA1200_LDOADJ_31V	(0x0 << 0)
#define ISA1200_LDOADJ_32V	(0x1 << 0)
#define ISA1200_LDOADJ_33V	(0x2 << 0)
#define ISA1200_LDOADJ_34V	(0x3 << 0)
#define ISA1200_LDOADJ_35V	(0x4 << 0)
#define ISA1200_LDOADJ_36V	(0x5 << 0)
#define ISA1200_LDOADJ_MASK	(0xf << 0)
#define ISA1200_HCTRL0		0x30
#define ISA1200_HAPDREN		(1 << 7)
#define ISA1200_OVEREN		(1 << 6)
#define ISA1200_OVERHL		(1 << 5)
#define ISA1200_HAPDIGMOD_PWM_IN	(1 << 3)
#define ISA1200_HAPDIGMOD_PWM_GEN	(2 << 3)
#define ISA1200_PLLMOD		(1 << 2)
#define ISA1200_PWMMOD_DIVIDER_128	(0 << 0)
#define ISA1200_PWMMOD_DIVIDER_256	(1 << 0)
#define ISA1200_PWMMOD_DIVIDER_512	(2 << 0)
#define ISA1200_PWMMOD_DIVIDER_1024	(3 << 0)
#define ISA1200_HCTRL1		0x31
#define ISA1200_EXTCLKSEL	(1 << 7)
#define ISA1200_BIT6_ON		(1 << 6)
#define ISA1200_MOTTYP_ERM	(1 << 5)
#define ISA1200_MOTTYP_LRA	(0 << 5)
#define ISA1200_PLLEN		(1 << 4)
#define ISA1200_SMARTEN		(1 << 3)
#define ISA1200_SMARTONT	(1 << 2)
#define ISA1200_SMARTOFFT_16	(0 << 0)
#define ISA1200_SMARTOFFT_32	(1 << 0)
#define ISA1200_SMARTOFFT_64	(2 << 0)
#define ISA1200_SMARTOFFT_100	(3 << 0)
#define ISA1200_HCTRL2		0x32
#define ISA1200_HSWRST		(1 << 7)
#define ISA1200_SESTMOD		(1 << 2)
#define ISA1200_SEEN		(1 << 1)
#define ISA1200_SEEVENT		(1 << 0)
#define ISA1200_HCTRL3		0x33
#define ISA1200_PPLLDIV_MASK	(0xf0)
#define ISA1200_PPLLDIV_SHIFT	(4)
#define ISA1200_PPLLDIV_1	(0x0)
#define ISA1200_PPLLDIV_2	(0x1)
#define ISA1200_PPLLDIV_4	(0x2)
#define ISA1200_PPLLDIV_8	(0x3)
#define ISA1200_PPLLDIV_16	(0x4)
#define ISA1200_PPLLDIV_32	(0x5)
#define ISA1200_PPLLDIV_64	(0x6)
#define ISA1200_PPLLDIV_128	(0x7)
#define ISA1200_WPLLDIV_MASK	(0x0f)
#define ISA1200_WPLLDIV_SHIFT	(0)
#define ISA1200_WPLLDIV_1	(0x0)
#define ISA1200_WPPLLDIV_2	(0x1)
#define ISA1200_WPPLLDIV_4	(0x2)
#define ISA1200_WPPLLDIV_8	(0x3)
#define ISA1200_WPPLLDIV_16	(0x4)
#define ISA1200_WPPLLDIV_32	(0x5)
#define ISA1200_WPPLLDIV_64	(0x6)
#define ISA1200_WPPLLDIV_128	(0x7)
#define ISA1200_HCTRL4		0x34
#define ISA1200_HCTRL5		0x35
#define ISA1200_HCTRL6		0x36
#define ISA1200_HCTRL7		0x37
#define ISA1200_HCTRL8		0x38
#define ISA1200_HCTRL9		0x39
#define ISA1200_PLLP_SHIFT	(4)
#define ISA1200_PLLP_MASK	(0xf0)
#define ISA1200_PLLS_SHIFT	(0)
#define ISA1200_PLLS_MASK	(0x0f)
#define ISA1200_HCTRLA		0x3A
#define ISA1200_PLLMM_MASK	(0xfc)
#define ISA1200_PLLMM_SHIFT	(2)
#define ISA1200_PLLMS_MASK	(0x03)
#define ISA1200_PLLMS_SHIFT	(0)
#define ISA1200_HCTRLB		0x3B
#define ISA1200_HCTRLC		0x3C
#define ISA1200_HCTRLD		0x3D

#define PWM_HAPTIC_PERIOD	44640
#define PWM_HAPTIC_DEFAULT_LEVEL	99

#endif /* __LINUX_ISA1200_H */
