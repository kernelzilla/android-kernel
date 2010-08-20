#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <asm/gpio.h>

#include <linux/reboot.h>
#include <linux/syscalls.h>
#include "board-mot.h"

#define    MORRISON_SLIDER_DETECT_GPIO 	42

#define    MORRISON_KEY_HOME_GPIO     	83
#define    MORRISON_KEY_BACK_GPIO     	94
#define    MORRISON_KEY_CENTER_GPIO   	20
#define    MORRISON_EMU_SEND_END_DET  	21

#define    MORRISON_KEY_POWER_GPIO    	0	/*  no such thing yet */

#define    MOTUS_TABLET_MODE_GPIO   	20
#define    MOTUS_SLIDER_DETECT_GPIO 	42

static struct gpio_event_direct_entry morrison_kpd_map[] = {
	{ MORRISON_KEY_HOME_GPIO,              KEY_HOME    },
	{ MORRISON_KEY_BACK_GPIO,              KEY_BACK    },
	{ MORRISON_KEY_CENTER_GPIO,            KEY_MENU    },
};
#if 0
static struct gpio_event_direct_entry motus_kpd_map[] = {
/* Nothing in P1 */
};
#endif
/*
	Morrison slider keys/switches
*/
static struct gpio_event_direct_entry morrison_kpd_switch_map[] = {
	{ MORRISON_SLIDER_DETECT_GPIO,       SW_LID       }
};

static struct gpio_event_input_info morrison_kpd_nav_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = morrison_kpd_map,
	.keymap_size = ARRAY_SIZE(morrison_kpd_map)
};

static struct gpio_event_input_info morrison_kpd_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = morrison_kpd_switch_map,
	.keymap_size = ARRAY_SIZE(morrison_kpd_switch_map)
};

static struct gpio_event_info *morrison_kpd_info[] = {
	&morrison_kpd_nav_info.info,
	&morrison_kpd_switch_info.info,
};

static struct gpio_event_platform_data morrison_kpd_data = {
	.name = "morrison-kpd",
	.info = morrison_kpd_info,
	.info_count = ARRAY_SIZE(morrison_kpd_info)
};

static struct platform_device morrison_kpd_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &morrison_kpd_data,
	},
};

/* FIXME!!! */
#if 0
static struct platform_device zeppelin_kpd_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &morrison_kpd_data,
	},
};

/*
	Motus slider keys/switches
*/
static struct gpio_event_input_info motus_kpd_nav_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = motus_kpd_map,
	.keymap_size = ARRAY_SIZE(motus_kpd_map)
};

static struct gpio_event_direct_entry motus_kpd_switch_map[] = {
	{ MOTUS_TABLET_MODE_GPIO ,  SW_TABLET_MODE  },
	{ MOTUS_SLIDER_DETECT_GPIO, SW_LID          }
};

static struct gpio_event_input_info motus_kpd_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = motus_kpd_switch_map,
	.keymap_size = ARRAY_SIZE(motus_kpd_switch_map)
};

static struct gpio_event_info *motus_kpd_info[] = {
	&motus_kpd_nav_info.info,
	&motus_kpd_switch_info.info,
};

static struct gpio_event_platform_data motus_kpd_data = {
	.name = "motus-kpd",
	.info = motus_kpd_info,
	.info_count = ARRAY_SIZE(motus_kpd_info)
};

static struct platform_device motus_kpd_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &motus_kpd_data,
	},
};
#endif

/* eddie */

int reset_requested;
static void deferred_restart(struct work_struct *dummy)
{
	sys_sync();
	kernel_restart(NULL);
}
static DECLARE_WORK(restart_work, deferred_restart);

static int alt;
static int ctrl;
static int del;
static int CTRL_ALT_DEL[3];

static void keyreset_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value) {
    if (code == CTRL_ALT_DEL[1])
	alt = value;

    if (code == CTRL_ALT_DEL[0])
	ctrl = value;

    if (code == CTRL_ALT_DEL[2])
	del = value;

    if (reset_requested && !alt && !ctrl && !del)
	/* all buttons released */
	reset_requested++;

    if (alt && ctrl && del) {
	if (reset_requested > 1)
		/* ctrl-alt-del was released and then pressed again */
		panic("Restart required (sys sync failed!)");
	else {
		reset_requested++;
		schedule_work(&restart_work);
	}
    }
}

static int keyreset_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id) {
	int ret;
	struct input_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "keyreset";

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	pr_info("adp5588: using input dev %s for key reset\n", dev->name);
	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keyreset_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keyreset_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, keyreset_ids);

static struct input_handler keyreset_handler = {
	.event = keyreset_event,
	.connect = keyreset_connect,
	.disconnect = keyreset_disconnect,
	.name =	"keyreset_3keys",
	.id_table =	keyreset_ids,
};
/* ~eddie */



/* Exported API */

int __init keypad_mot_init(void)
{
#if 0
    if (machine_is_motus()) {
	if (mot_hw_rev < 0x30) 	{ /* P2 keyboard layout */

		CTRL_ALT_DEL[0] = 16;
		CTRL_ALT_DEL[1] = 71;
		CTRL_ALT_DEL[2] = 77;
	}
		else if (mot_hw_rev < 0x0400) {/* P3 keypad layout */

		CTRL_ALT_DEL[0] = 16;
		CTRL_ALT_DEL[1] = 57;
		CTRL_ALT_DEL[2] = 77;
		} else {
		CTRL_ALT_DEL[0] = 16;
		CTRL_ALT_DEL[1] = 71;
		CTRL_ALT_DEL[2] = 77;
	}
		input_register_handler(&keyreset_handler);
		return platform_device_register(&motus_kpd_device);
	}

	else if (machine_is_zeppelin())

		return platform_device_register(&zeppelin_kpd_device);

#endif
	if (machine_is_morrison()) {
		CTRL_ALT_DEL[0] = 0x3e;
		CTRL_ALT_DEL[1] = 0x47;
		CTRL_ALT_DEL[2] = 0x4d;
		input_register_handler(&keyreset_handler);
		return platform_device_register(&morrison_kpd_device);
	} else	{

		printk(KERN_ERR "Unrecognized machine type, can't uninit keypad\n");
		return -1;
	}
}
static void __exit keypad_mot_exit(void)
{
#if 0
    if (machine_is_motus()) {
	input_unregister_handler(&keyreset_handler);
	platform_device_unregister(&motus_kpd_device);

	} else if (machine_is_zeppelin())
		platform_device_unregister(&zeppelin_kpd_device);

#endif
	if (machine_is_morrison()) {
		input_unregister_handler(&keyreset_handler);
		platform_device_unregister(&morrison_kpd_device);
	} else 	{
		printk(KERN_ERR "Unrecognized machine type, can't init keypad\n");
		return;
	}

}

/*
 * This driver MUST start late, or it will break logic in Android.
 */
late_initcall(keypad_mot_init);
module_exit(keypad_mot_exit);


MODULE_AUTHOR("Vladimir Karpovich <Vladimir.Karpovich@motorola.com>");
MODULE_DESCRIPTION("Motorola GPIO keypad driver");
MODULE_LICENSE("GPL");
