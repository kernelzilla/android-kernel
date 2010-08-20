/*
 * arch/arm/mach-omap2/board-mapphone-spi.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/leds-ld-cpcap-disp.h>
#include <linux/leds-cpcap-kpad.h>
#include <linux/leds-cpcap-display.h>
#include <linux/leds-cpcap-button.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/resource.h>
#include <mach/omap34xx.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

/*
 * CPCAP devcies are common for different HW Rev.
 *
 */
static struct platform_device cpcap_3mm5_device = {
	.name   = "cpcap_3mm5",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

#ifdef CONFIG_CPCAP_USB
static struct platform_device cpcap_usb_device = {
	.name           = "cpcap_usb",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_usb_det_device = {
	.name   = "cpcap_usb_det",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};
#endif /* CONFIG_CPCAP_USB */

#ifdef CONFIG_TTA_CHARGER
static struct platform_device cpcap_tta_det_device = {
	.name   = "cpcap_tta_charger",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};
#endif


static struct platform_device cpcap_rgb_led = {
	.name           = LD_MSG_IND_DEV,
	.id             = -1,
	.dev.platform_data  = NULL,
};

#ifdef CONFIG_SOUND_CPCAP_OMAP
static struct platform_device cpcap_audio_device = {
	.name           = "cpcap_audio",
	.id             = -1,
	.dev.platform_data  = NULL,
};
#endif

#ifdef CONFIG_LEDS_AF_LED
static struct platform_device cpcap_af_led = {
	.name           = LD_AF_LED_DEV,
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
       },
};
#endif

static struct platform_device cpcap_bd7885 = {
	.name           = "bd7885",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
       },
};

static struct platform_device cpcap_vio_active_device = {
	.name		= "cpcap_vio_active",
	.id		= -1,
	.dev		= {
		.platform_data = NULL,
	},
};

static struct platform_device *cpcap_devices[] = {
	&cpcap_rgb_led,
#ifdef CONFIG_CPCAP_USB
	&cpcap_usb_device,
	&cpcap_usb_det_device,
#endif
#ifdef CONFIG_SOUND_CPCAP_OMAP
	&cpcap_audio_device,
#endif
	&cpcap_3mm5_device,
#ifdef CONFIG_TTA_CHARGER
	&cpcap_tta_det_device,
#endif
#ifdef CONFIG_LEDS_AF_LED
	&cpcap_af_led,
#endif
	&cpcap_bd7885
};


/*
 * CPCAP devcies whose availability depends on HW
 *
 */
static struct platform_device  cpcap_kpad_led = {
	.name           = CPCAP_KPAD_DEV,
	.id             = -1,
	.dev            = {
	.platform_data  = NULL,
	},
};

static struct platform_device ld_cpcap_kpad_led = {
	.name           = LD_KPAD_DEV,
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_button_led = {
	.name           = CPCAP_BUTTON_DEV,
	.id             = -1,
	.dev            = {
	.platform_data  = NULL,
	},
};

static struct disp_button_config_data btn_data;

static struct platform_device ld_cpcap_disp_button_led = {
	.name           = LD_DISP_BUTTON_DEV,
	.id             = -1,
	.dev            = {
	.platform_data  = &btn_data,
	},
};

static struct platform_device cpcap_display_led = {
	.name           = CPCAP_DISPLAY_DRV,
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_lm3554 = {
	.name           = "flash-torch",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_lm3559 = {
	.name           = "flash-torch-3559",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

#ifdef CONFIG_ARM_OF
static int is_ld_cpcap_kpad_on(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_KPAD_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_KPAD_LED);
		return -ENODEV;
	}

	prop = of_get_property(node, "tablet_kpad_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_TABLET_KPAD_LED);
			of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int is_cpcap_kpad_on(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_KPAD_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_KPAD_LED);
		return -ENODEV;
	}

	prop = of_get_property(node, "ruth_kpad_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_RUTH_KPAD_LED);
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int cpcap_button_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_HOME_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_HOME_LED);
		return -ENODEV;
}

	prop = of_get_property(node, "ruth_button_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_RUTH_BUTTON);
			of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int ld_cpcap_disp_button_init(void)
{
	struct device_node *node;
	const void *prop;
	struct disp_button_config_data *pbtn_data = &btn_data;
	u8 device_available, ret;

	ret = -ENODEV;

	node = of_find_node_by_path(DT_HOME_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
							DT_HOME_LED);
		goto err;
	}

	prop = of_get_property(node, "tablet_button_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_TABLET_BUTTON);
		goto err;
	}
	/*
	 * Hard Keys do not light
	 * making it difficult to see in low/no light
	 */
	if (device_available <= 0) {
		pr_err("Tablet button led device not present: %d\n",
							device_available);
		goto err;
	}

	prop = of_get_property(node, "button_duty_cycle", NULL);
	if (prop)
		pbtn_data->duty_cycle = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_duty_cycle");
		goto err;
	}

	prop = of_get_property(node, "button_cpcap_mask", NULL);
	if (prop)
		pbtn_data->cpcap_mask = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_cpcap_mask");
		goto err;
	}

	prop = of_get_property(node, "button_current", NULL);
	if (prop)
		pbtn_data->led_current = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_current");
		goto err;
	}

	prop = of_get_property(node, "button_register", NULL);
	if (prop)
		pbtn_data->reg = *(enum cpcap_reg *)prop;
	else {
		pr_err("Read property %s error!\n", "button_register");
		goto err;
	}

	prop = of_get_property(node, "button_register2", NULL);
	if (prop == NULL) {
		pr_err("Read property %s error!\n", "button_register2");
		pbtn_data->reg2 = 0;

		/* NOTE: Fail to read reg2 NOT mean everything wrong*/
		goto done;
	}

	pbtn_data->reg2 = *(enum cpcap_reg *)prop;
	prop = of_get_property(node, "button_cpcap_abmode_mask", NULL);
	if (prop)
		pbtn_data->abmode_cpcap_mask = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n",
				"button_cpcap_abmode_mask");
		goto err;
	}

	prop = of_get_property(node, "button_pwm", NULL);
	if (prop)
		pbtn_data->pwm = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_pwm");
		goto err;
	}

	prop = of_get_property(node, "button_fade_time", NULL);
	if (prop)
		pbtn_data->fade_time = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_fade_time");
		goto err;
	}

	prop = of_get_property(node, "button_fade_en", NULL);
	if (prop)
		pbtn_data->fade_en = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_fade_time");
		goto err;
	}

	prop = of_get_property(node, "button_abmode", NULL);
	if (prop)
		pbtn_data->abmode = *(u16 *)prop;
	else {
		pr_err("Read property %s error!\n", "button_abmode");
		goto err;
	}

done:
	ret = 1;
err:
	if (node != NULL)
		of_node_put(node);
	return ret;
}

static int is_disp_led_on(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_BACKLIGHT);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_BACKLIGHT);
		return -ENODEV;
	}

	prop = of_get_property(node, "ruth_lcd", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_RUTH_LCD);
			of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int led_cpcap_lm3554_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_LM3554);
	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "device_available", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", "device_available");
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int led_cpcap_lm3559_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_LM3559);
	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "device_available", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", "device_available");
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int is_cpcap_vio_supply_converter(void)
{
	struct device_node *node;
	const void *prop;
	int size;

	node = of_find_node_by_path(DT_PATH_CPCAP);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_CPCAP_VIO_SUPPLY_CONVERTER,
				&size);
		if (prop && size)
			return *(u8 *)prop;
	}
	/* The converter is existing by default */
	return 1;
}

#endif /* CONFIG_ARM_OF */


void __init mapphone_cpcap_client_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_device_register(cpcap_devices[i]);

	if (is_cpcap_kpad_on() > 0)
		cpcap_device_register(&cpcap_kpad_led);

	if (is_ld_cpcap_kpad_on() > 0)
		cpcap_device_register(&ld_cpcap_kpad_led);

	if (ld_cpcap_disp_button_init() > 0)
		cpcap_device_register(&ld_cpcap_disp_button_led);

	if (cpcap_button_init() > 0)
		cpcap_device_register(&cpcap_button_led);

	if (is_disp_led_on() > 0)
		cpcap_device_register(&cpcap_display_led);

	if (led_cpcap_lm3554_init() > 0)
		cpcap_device_register(&cpcap_lm3554);

	if (led_cpcap_lm3559_init() > 0)
		cpcap_device_register(&cpcap_lm3559);

	if (!is_cpcap_vio_supply_converter())
		cpcap_device_register(&cpcap_vio_active_device);
}
