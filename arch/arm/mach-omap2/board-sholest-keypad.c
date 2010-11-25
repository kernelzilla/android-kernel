/*
 * arch/arm/mach-omap2/board-sholest-keypad.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_event.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>
#include <mach/board-sholest.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

static unsigned int sholest_col_gpios[] = { 43, 56, 57 };
static unsigned int sholest_row_gpios[] = { 37, 39 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(sholest_row_gpios) + (row))

static const unsigned short sholest_keymap[ARRAY_SIZE(sholest_col_gpios) *
                                           ARRAY_SIZE(sholest_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEDOWN,
    [KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	
	[KEYMAP_INDEX(1, 0)] = KEY_CAMERA-1,    /* camera 1 key, steal KEY_HP*/
	[KEYMAP_INDEX(1, 1)] = KEY_CAMERA,      /* "camera 2" key */

	[KEYMAP_INDEX(2, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 1)] = KEY_MACRO
};

static struct gpio_event_matrix_info sholest_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = sholest_keymap,
	.output_gpios = sholest_col_gpios,
	.input_gpios = sholest_row_gpios,
#ifndef CONFIG_ARM_OF
	.sw_fixup = fixup,
#endif
	.noutputs = ARRAY_SIZE(sholest_col_gpios),
	.ninputs = ARRAY_SIZE(sholest_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry sholest_keypad_switch_map[] = {
	{ GPIO_SILENCE_KEY,	SW_HEADPHONE_INSERT }
};

static struct gpio_event_input_info sholest_keypad_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = sholest_keypad_switch_map,
	.keymap_size = ARRAY_SIZE(sholest_keypad_switch_map)
};

static struct gpio_event_info *sholest_keypad_info[] = {
	&sholest_keypad_matrix_info.info,
	&sholest_keypad_switch_info.info,
};

static struct gpio_event_platform_data sholest_keypad_data = {
	.name = "sholes-keypad",
	.info = sholest_keypad_info,
	.info_count = ARRAY_SIZE(sholest_keypad_info)
};

static struct platform_device sholest_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &sholest_keypad_data,
	},
};

#ifdef CONFIG_ARM_OF
static int __init sholest_dt_kp_init(void)
{
	struct device_node *kp_node;
	const void *kp_prop;

	if ((kp_node = of_find_node_by_path(DT_PATH_KEYPAD))) {
		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_ROWS, NULL)))
			sholest_keypad_matrix_info.ninputs = \
				*(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_COLS, NULL)))
			sholest_keypad_matrix_info.noutputs = \
				*(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_ROWREG, NULL)))
			sholest_keypad_matrix_info.input_gpios = \
				(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_COLREG, NULL)))
			sholest_keypad_matrix_info.output_gpios = \
				(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_MAPS, NULL)))
			sholest_keypad_matrix_info.keymap = \
				(unsigned short *)kp_prop;

		kp_prop = of_get_property(kp_node, DT_PROP_KEYPAD_NAME, NULL);
		if (kp_prop != NULL)
			sholest_keypad_data.name = kp_prop;

		of_node_put(kp_node);
	}

	return kp_node ? 0 : -ENODEV;
}
#endif

static int __init sholest_init_keypad(void)
{
#ifdef CONFIG_ARM_OF
	if (sholest_dt_kp_init())
		printk(KERN_INFO "Keypad: using non-dt configuration\n");
#endif

	/* keypad rows */
	omap_cfg_reg(K4_34XX_GPIO37);
	omap_cfg_reg(R3_34XX_GPIO39);

	/* keypad columns */
	omap_cfg_reg(K3_34XX_GPIO43_OUT);
	omap_cfg_reg(R8_34XX_GPIO56_OUT);
	omap_cfg_reg(P8_34XX_GPIO57_OUT);

	/* switches */
	omap_cfg_reg(AB2_34XX_GPIO177);
	omap_cfg_reg(AH17_34XX_GPIO100);

	return platform_device_register(&sholest_keypad_device);
}

device_initcall(sholest_init_keypad);
