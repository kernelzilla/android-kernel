/* arch/arm/mach-msm/rpc_server_handset.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <mach/msm_handset.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>

#ifdef CONFIG_MACH_MOT
 struct input_dev *msm_keypad_get_input_dev(void);
#else
 #include "keypad-surf-ffa.h"
#endif

#define HS_SERVER_PROG 0x30000062
#define HS_SERVER_VERS 0x00010001

#define RPC_KEYPAD_NULL_PROC 0
#define RPC_KEYPAD_PASS_KEY_CODE_PROC 2
#define RPC_KEYPAD_SET_PWR_KEY_STATE_PROC 3

#define HS_PWR_K		0x6F	/* Power key */
#define HS_END_K		0x51	/* End key or Power key */
#define HS_STEREO_HEADSET_K	0x82
#define HS_HEADSET_SWITCH_K	0x84
#define HS_REL_K		0xFF	/* key release */

#define KEY(hs_key, input_key) ((hs_key << 24) | input_key)

static const uint32_t hs_key_map[] = {
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_LAJOLLA)
	KEY(HS_END_K, KEY_POWER),
#else
	KEY(HS_PWR_K, KEY_POWER),
	KEY(HS_END_K, KEY_END),
	KEY(HS_STEREO_HEADSET_K, SW_HEADPHONE_INSERT),
#endif
	KEY(HS_HEADSET_SWITCH_K, KEY_MEDIA),
	0
};

static struct input_dev *kpdev;
static struct input_dev *hsdev;

static int hs_find_key(uint32_t hscode)
{
	int i, key;

	key = KEY(hscode, 0);

	for (i = 0; hs_key_map[i] != 0; i++) {
		if ((hs_key_map[i] & 0xff000000) == key)
			return hs_key_map[i] & 0x00ffffff;
	}
	return -1;
}

static void
report_headset_switch(struct input_dev *dev, int key, int value)
{
	struct msm_handset *hs = input_get_drvdata(dev);

	input_report_switch(dev, key, value);
	switch_set_state(&hs->sdev, value);
	input_sync(dev);
}

/*
 * tuple format: (key_code, key_param)
 *
 * old-architecture:
 * key-press = (key_code, 0)
 * key-release = (0xff, key_code)
 *
 * new-architecutre:
 * key-press = (key_code, 0)
 * key-release = (key_code, 0xff)
 */
static void report_hs_key(uint32_t key_code, uint32_t key_parm)
{
	int key, temp_key_code;

	if (key_code == HS_REL_K)
		key = hs_find_key(key_parm);
	else
		key = hs_find_key(key_code);

	temp_key_code = key_code;

	if (key_parm == HS_REL_K)
		key_code = key_parm;

	switch (key) {
	case KEY_POWER:
	case KEY_END:
		if (!kpdev) {
			printk(KERN_ERR "%s: No input device for reporting "
					"pwr/end key press\n", __func__);
			return;
		}
		input_report_key(kpdev, key, (key_code != HS_REL_K));
		input_sync(kpdev);
		break;
	case SW_HEADPHONE_INSERT:
		if (!hsdev) {
			printk(KERN_ERR "%s: No input device for reporting "
					"handset events\n", __func__);
			return;
		}
		report_headset_switch(hsdev, key, (key_code != HS_REL_K));
		break;
	case KEY_MEDIA:
		if (!hsdev) {
			printk(KERN_ERR "%s: No input device for reporting "
					"handset events\n", __func__);
			return;
		}
		input_report_key(hsdev, key, (key_code != HS_REL_K));
		input_sync(hsdev);
		break;
	case -1:
		printk(KERN_ERR "%s: No mapping for remote handset event %d\n",
				 __func__, temp_key_code);
		break;
	default:
		printk(KERN_ERR "%s: Unhandled handset key %d\n", __func__,
				key);
	}
}

static int handle_hs_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	struct rpc_keypad_pass_key_code_args {
		uint32_t key_code;
		uint32_t key_parm;
	};

	kpdev = msm_keypad_get_input_dev();
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_LAJOLLA)
	hsdev = msm_get_handset_input_dev();
#endif 
	switch (req->procedure) {
	case RPC_KEYPAD_NULL_PROC:
		return 0;

	case RPC_KEYPAD_PASS_KEY_CODE_PROC: {
		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code, args->key_parm);

		return 0;
	}

	case RPC_KEYPAD_SET_PWR_KEY_STATE_PROC:
		/* This RPC function must be available for the ARM9
		 * to function properly.  This function is redundant
		 * when RPC_KEYPAD_PASS_KEY_CODE_PROC is handled. So
		 * input_report_key is not needed.
		 */
		return 0;
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_SERVER_PROG,
	.vers		= HS_SERVER_VERS,
	.rpc_call	= handle_hs_rpc_call,
};

static int __init hs_rpc_server_init(void)
{
	return msm_rpc_create_server(&hs_rpc_server);
}
module_init(hs_rpc_server_init);
