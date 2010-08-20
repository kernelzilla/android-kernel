/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/gpio_event.h>

#include <asm/mach-types.h>

/* don't turn this on without updating the ffa support */
#define SCAN_FUNCTION_KEYS 0

/* FFA:
 36: KEYSENSE_N(0)
 37: KEYSENSE_N(1)
 38: KEYSENSE_N(2)
 39: KEYSENSE_N(3)
 40: KEYSENSE_N(4)

 31: KYPD_17
 32: KYPD_15
 33: KYPD_13
 34: KYPD_11
 35: KYPD_9
 41: KYPD_MEMO
*/

/*sesame*/
static unsigned int keypad_row_gpios[] = {
	36, 37, 38, 39, 40, 41, 42
};

static unsigned int keypad_col_gpios[] = { 29, 30, 31, 32, 33, 34, 35 };

static unsigned int keypad_row_gpios_8k_ffa[] = {31, 32, 33, 34, 35, 36};
static unsigned int keypad_col_gpios_8k_ffa[] = {38, 39, 40, 41, 42};

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))
#define FFA_8K_KEYMAP_INDEX(row, col) ((row)* \
				ARRAY_SIZE(keypad_col_gpios_8k_ffa) + (col))

static const unsigned short keypad_keymap_surf[ARRAY_SIZE(keypad_col_gpios) *
					  ARRAY_SIZE(keypad_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 2)] = KEY_A,
	[KEYMAP_INDEX(0, 3)] = KEY_X,
	[KEYMAP_INDEX(0, 4)] = KEY_I,
	[KEYMAP_INDEX(0, 5)] = KEY_MAIL,   /* need change to @ */
	[KEYMAP_INDEX(0, 6)] = KEY_Q,

	[KEYMAP_INDEX(1, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_W,
	[KEYMAP_INDEX(1, 3)] = KEY_E,
	[KEYMAP_INDEX(1, 4)] = KEY_M,
	[KEYMAP_INDEX(1, 5)] = KEY_P,
	[KEYMAP_INDEX(1, 6)] = KEY_DOT,


	[KEYMAP_INDEX(2, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_G,
	[KEYMAP_INDEX(2, 3)] = KEY_C,
	[KEYMAP_INDEX(2, 4)] = KEY_Z,
	[KEYMAP_INDEX(2, 5)] = KEY_B,
	[KEYMAP_INDEX(2, 6)] = KEY_KPPLUS,


	[KEYMAP_INDEX(3, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(3, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(3, 2)] = KEY_K,
	[KEYMAP_INDEX(3, 3)] = KEY_N,
	[KEYMAP_INDEX(3, 4)] = KEY_T,
	[KEYMAP_INDEX(3, 5)] = KEY_O,
	[KEYMAP_INDEX(3, 6)] = KEY_H,

	[KEYMAP_INDEX(4, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(4, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(4, 2)] = KEY_SLASH,
	[KEYMAP_INDEX(4, 3)] = KEY_V,
	[KEYMAP_INDEX(4, 4)] = KEY_Y,
	[KEYMAP_INDEX(4, 5)] = KEY_F,
	[KEYMAP_INDEX(4, 6)] = KEY_L,

	[KEYMAP_INDEX(5, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(5, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(5, 2)] = KEY_S,
	[KEYMAP_INDEX(5, 3)] = KEY_D,
	[KEYMAP_INDEX(5, 4)] = KEY_U,
	[KEYMAP_INDEX(5, 5)] = KEY_R,
	[KEYMAP_INDEX(5, 6)] = KEY_J,

	[KEYMAP_INDEX(6, 0)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(6, 1)] = KEY_LEFTALT,
	[KEYMAP_INDEX(6, 2)] = KEY_SPACE,
	[KEYMAP_INDEX(6, 3)] = KEY_F1,  /* Need change to SYM */
	[KEYMAP_INDEX(6, 4)] = KEY_ENTER,
	[KEYMAP_INDEX(6, 5)] = KEY_SEARCH,
	[KEYMAP_INDEX(6, 6)] = KEY_BACKSPACE

};

static const unsigned short keypad_keymap_ffa[ARRAY_SIZE(keypad_col_gpios) *
					      ARRAY_SIZE(keypad_row_gpios)] = {
	/*[KEYMAP_INDEX(0, 0)] = ,*/
	/*[KEYMAP_INDEX(0, 1)] = ,*/
	[KEYMAP_INDEX(0, 2)] = KEY_1,
	[KEYMAP_INDEX(0, 3)] = KEY_SEND,
	[KEYMAP_INDEX(0, 4)] = KEY_LEFT,

	[KEYMAP_INDEX(1, 0)] = KEY_3,
	[KEYMAP_INDEX(1, 1)] = KEY_RIGHT,
	[KEYMAP_INDEX(1, 2)] = KEY_VOLUMEUP,
	/*[KEYMAP_INDEX(1, 3)] = ,*/
	[KEYMAP_INDEX(1, 4)] = KEY_6,

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,      /* A */
	[KEYMAP_INDEX(2, 1)] = KEY_BACK,      /* B */
	[KEYMAP_INDEX(2, 2)] = KEY_0,
	[KEYMAP_INDEX(2, 3)] = 228,           /* KEY_SHARP */
	[KEYMAP_INDEX(2, 4)] = KEY_9,

	[KEYMAP_INDEX(3, 0)] = KEY_UP,
	[KEYMAP_INDEX(3, 1)] = 232, /* KEY_CENTER */ /* i */
	[KEYMAP_INDEX(3, 2)] = KEY_4,
	/*[KEYMAP_INDEX(3, 3)] = ,*/
	[KEYMAP_INDEX(3, 4)] = KEY_2,

	[KEYMAP_INDEX(4, 0)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(4, 1)] = KEY_SOUND,
	[KEYMAP_INDEX(4, 2)] = KEY_DOWN,
	[KEYMAP_INDEX(4, 3)] = KEY_8,
	[KEYMAP_INDEX(4, 4)] = KEY_5,

	/*[KEYMAP_INDEX(5, 0)] = ,*/
	[KEYMAP_INDEX(5, 1)] = 227,           /* KEY_STAR */
	[KEYMAP_INDEX(5, 2)] = 230, /*SOFT2*/ /* 2 */
	[KEYMAP_INDEX(5, 3)] = KEY_MENU,      /* 1 */
	[KEYMAP_INDEX(5, 4)] = KEY_7,
};

#define QSD8x50_FFA_KEYMAP_SIZE (ARRAY_SIZE(keypad_col_gpios_8k_ffa) * \
			ARRAY_SIZE(keypad_row_gpios_8k_ffa))

static const unsigned short keypad_keymap_8k_ffa[QSD8x50_FFA_KEYMAP_SIZE] = {

	[FFA_8K_KEYMAP_INDEX(0, 0)] = KEY_VOLUMEDOWN,
	/*[KEYMAP_INDEX(0, 1)] = ,*/
	[FFA_8K_KEYMAP_INDEX(0, 2)] = KEY_DOWN,
	[FFA_8K_KEYMAP_INDEX(0, 3)] = KEY_8,
	[FFA_8K_KEYMAP_INDEX(0, 4)] = KEY_5,

	[FFA_8K_KEYMAP_INDEX(1, 0)] = KEY_UP,
	[FFA_8K_KEYMAP_INDEX(1, 1)] = KEY_CLEAR,
	[FFA_8K_KEYMAP_INDEX(1, 2)] = KEY_4,
	/*[KEYMAP_INDEX(1, 3)] = ,*/
	[FFA_8K_KEYMAP_INDEX(1, 4)] = KEY_2,

	[FFA_8K_KEYMAP_INDEX(2, 0)] = KEY_HOME,      /* A */
	[FFA_8K_KEYMAP_INDEX(2, 1)] = KEY_BACK,      /* B */
	[FFA_8K_KEYMAP_INDEX(2, 2)] = KEY_0,
	[FFA_8K_KEYMAP_INDEX(2, 3)] = 228,           /* KEY_SHARP */
	[FFA_8K_KEYMAP_INDEX(2, 4)] = KEY_9,

	[FFA_8K_KEYMAP_INDEX(3, 0)] = KEY_3,
	[FFA_8K_KEYMAP_INDEX(3, 1)] = KEY_RIGHT,
	[FFA_8K_KEYMAP_INDEX(3, 2)] = KEY_VOLUMEUP,
	/*[KEYMAP_INDEX(3, 3)] = ,*/
	[FFA_8K_KEYMAP_INDEX(3, 4)] = KEY_6,

	[FFA_8K_KEYMAP_INDEX(4, 0)] = 232,		/* OK */
	[FFA_8K_KEYMAP_INDEX(4, 1)] = KEY_SOUND,
	[FFA_8K_KEYMAP_INDEX(4, 2)] = KEY_1,
	[FFA_8K_KEYMAP_INDEX(4, 3)] = KEY_SEND,
	[FFA_8K_KEYMAP_INDEX(4, 4)] = KEY_LEFT,

	/*[KEYMAP_INDEX(5, 0)] = ,*/
	[FFA_8K_KEYMAP_INDEX(5, 1)] = 227,           /* KEY_STAR */
	[FFA_8K_KEYMAP_INDEX(5, 2)] = 230, /*SOFT2*/ /* 2 */
	[FFA_8K_KEYMAP_INDEX(5, 3)] = KEY_MENU,      /* 1 */
	[FFA_8K_KEYMAP_INDEX(5, 4)] = KEY_7,
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info surf_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_surf,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *surf_keypad_info[] = {
	&surf_keypad_matrix_info.info
};

static struct gpio_event_platform_data surf_keypad_data = {
	.name		= "surf_keypad",
	.info		= surf_keypad_info,
	.info_count	= ARRAY_SIZE(surf_keypad_info)
};

struct platform_device keypad_device_surf = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &surf_keypad_data,
	},
};

/* 8k FFA keypad platform device information */
static struct gpio_event_matrix_info keypad_matrix_info_8k_ffa = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_8k_ffa,
	.output_gpios	= keypad_row_gpios_8k_ffa,
	.input_gpios	= keypad_col_gpios_8k_ffa,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_8k_ffa),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_8k_ffa),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *keypad_info_8k_ffa[] = {
	&keypad_matrix_info_8k_ffa.info
};

static struct gpio_event_platform_data keypad_data_8k_ffa = {
	.name		= "8k_ffa_keypad",
	.info		= keypad_info_8k_ffa,
	.info_count	= ARRAY_SIZE(keypad_info_8k_ffa)
};

struct platform_device keypad_device_8k_ffa = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &keypad_data_8k_ffa,
	},
};

/* 7k FFA keypad platform device information */
static struct gpio_event_matrix_info keypad_matrix_info_7k_ffa = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_ffa,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *keypad_info_7k_ffa[] = {
	&keypad_matrix_info_7k_ffa.info
};

static struct gpio_event_platform_data keypad_data_7k_ffa = {
	.name		= "7k_ffa_keypad",
	.info		= keypad_info_7k_ffa,
	.info_count	= ARRAY_SIZE(keypad_info_7k_ffa)
};

struct platform_device keypad_device_7k_ffa = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &keypad_data_7k_ffa,
	},
};

