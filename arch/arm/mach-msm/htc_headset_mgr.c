/*
 *
 * /arch/arm/mach-msm/htc_headset_mgr.c
 *
 * HTC headset manager driver.
 *
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>

#include <asm/atomic.h>
#include <asm/mach-types.h>

#include <mach/board.h>
#include <mach/vreg.h>
#include <mach/atmega_microp.h>

#include <mach/htc_headset_mgr.h>

#define DRIVER_NAME "HS_MGR"

static struct workqueue_struct *button_wq;
static void button_35mm_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(button_35mm_work, button_35mm_do_work);

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len);

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_RPC_SERVER_PROG,
	.vers		= HS_RPC_SERVER_VERS,
	.rpc_call	= hs_mgr_rpc_call,
};

struct button_work {
	struct delayed_work key_work;
	int key_code;
};

static struct htc_headset_mgr_info *hi;
static struct hs_notifier_func hs_mgr_notifier;

static void init_next_driver(void)
{
	int i = hi->driver_init_seq;

	if (!hi->pdata.headset_devices_num)
		return;

	if (i < hi->pdata.headset_devices_num) {
		hi->driver_init_seq++;
		platform_device_register(hi->pdata.headset_devices[i]);
	}
}

void hs_notify_driver_ready(char *name)
{
	SYS_MSG("%s ready", name);
	init_next_driver();
}

void hs_notify_hpin_irq(void)
{
	hi->hpin_jiffies = jiffies;
	SYS_MSG("HPIN IRQ");
}

int hs_hpin_stable(void)
{
	unsigned long last_hpin_jiffies = 0;
	unsigned long unstable_jiffies = 1.2 * HZ;

	last_hpin_jiffies = hi->hpin_jiffies;

	if (time_before_eq(jiffies, last_hpin_jiffies + unstable_jiffies))
		return 0;

	return 1;
}

int headset_notifier_register(struct headset_notifier *notifier)
{
	if (!notifier->func) {
		SYS_MSG("NULL register function");
		return 0;
	}

	switch (notifier->id) {
	case HEADSET_REG_REMOTE_ADC:
		SYS_MSG("Register REMOTE_ADC notifier");
		hs_mgr_notifier.remote_adc = notifier->func;
		break;
	case HEADSET_REG_RPC_KEY:
		SYS_MSG("Register RPC_KEY notifier");
		hs_mgr_notifier.rpc_key = notifier->func;
		break;
	case HEADSET_REG_MIC_STATUS:
		SYS_MSG("Register MIC_STATUS notifier");
		hs_mgr_notifier.mic_status = notifier->func;
		break;
	case HEADSET_REG_MIC_BIAS:
		SYS_MSG("Register MIC_BIAS notifier");
		hs_mgr_notifier.mic_bias_enable = notifier->func;
		break;
	case HEADSET_REG_MIC_SELECT:
		SYS_MSG("Register MIC_SELECT notifier");
		hs_mgr_notifier.mic_select = notifier->func;
		break;
	case HEADSET_REG_KEY_INT_ENABLE:
		SYS_MSG("Register KEY_INT_ENABLE notifier");
		hs_mgr_notifier.key_int_enable = notifier->func;
		break;
	case HEADSET_REG_KEY_ENABLE:
		SYS_MSG("Register KEY_ENABLE notifier");
		hs_mgr_notifier.key_enable = notifier->func;
		break;
	default:
		SYS_MSG("Unknown register ID");
		return 0;
	}

	return 1;
}

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len)
{
	struct hs_rpc_server_args_key *args_key;

	DBG_MSG("");

	switch (req->procedure) {
	case HS_RPC_SERVER_PROC_NULL:
		SYS_MSG("RPC_SERVER_NULL");
		break;
	case HS_RPC_SERVER_PROC_KEY:
		args_key = (struct hs_rpc_server_args_key *)(req + 1);
		args_key->adc = be32_to_cpu(args_key->adc);
		SYS_MSG("RPC_SERVER_KEY ADC = %u (0x%X)",
			args_key->adc, args_key->adc);
		if (hs_mgr_notifier.rpc_key) {
			wake_lock_timeout(&hi->hs_wake_lock,
					  HS_WAKE_LOCK_TIMEOUT);
			hs_mgr_notifier.rpc_key(args_key->adc);
		} else
			SYS_MSG("RPC_KEY notify function doesn't exist");
		break;
	default:
		SYS_MSG("Unknown RPC procedure");
		return -EINVAL;
	}

	return 0;
}

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

void button_pressed(int type)
{
	SYS_MSG("[H2W] button_pressed %d", type);
	atomic_set(&hi->btn_state, type);
	input_report_key(hi->input, type, 1);
	input_sync(hi->input);
}

void button_released(int type)
{
	SYS_MSG("[H2W] button_released %d", type);
	atomic_set(&hi->btn_state, 0);
	input_report_key(hi->input, type, 0);
	input_sync(hi->input);
}

void headset_button_event(int is_press, int type)
{
	if (!hs_hpin_stable()) {
		SYS_MSG("The HPIN is unstable, SKIP THE BUTTON EVENT.");
		return;
	}

	if (!is_press)
		button_released(type);
	else if (!atomic_read(&hi->btn_state))
		button_pressed(type);
}

static void set_35mm_hw_state(int state)
{
	if (hi->mic_bias_state != state && hs_mgr_notifier.mic_bias_enable) {
		hs_mgr_notifier.mic_bias_enable(state);
		hi->mic_bias_state = state;
		if (state) /* Wait for MIC bias stable */
			msleep(HS_DELAY_MIC_BIAS);
	}

	if (hs_mgr_notifier.mic_select)
		hs_mgr_notifier.mic_select(state);

	if (hs_mgr_notifier.key_enable)
		hs_mgr_notifier.key_enable(state);

	if (hs_mgr_notifier.key_int_enable)
		hs_mgr_notifier.key_int_enable(state);
}

#if 0
static void insert_h2w_35mm(int *state)
{
	int mic = HEADSET_NO_MIC;

	SYS_MSG("Insert USB 3.5mm headset");
	set_35mm_hw_state(1);

	if (hs_mgr_notifier.mic_status)
		mic = hs_mgr_notifier.mic_status();

	if (mic == HEADSET_NO_MIC) {
		/* without microphone */
		*state |= BIT_HEADSET_NO_MIC;
		hi->h2w_35mm_status = HTC_35MM_NO_MIC;
		SYS_MSG("11pin_3.5mm without microphone");
	} else { /* with microphone */
		*state |= BIT_HEADSET;
		hi->h2w_35mm_status = HTC_35MM_MIC;
		SYS_MSG("11pin_3.5mm with microphone");
	}
}

static void remove_h2w_35mm(void)
{
	SYS_MSG("Remove USB 3.5mm headset");

	set_35mm_hw_state(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->h2w_35mm_status = HTC_35MM_UNPLUG;
}
#endif /* #if 0 */

static void button_35mm_do_work(struct work_struct *w)
{
	int key;
	struct button_work *work;

	work = container_of(w, struct button_work, key_work.work);
	hi->key_level_flag = work->key_code;

	if (!hi->is_ext_insert && !hi->h2w_35mm_status) {
		kfree(work);
		SYS_MSG("3.5mm headset is plugged out, skip report key event");
		return;
	}

	if (hi->key_level_flag) {
		switch (hi->key_level_flag) {
		case 1:
			SYS_MSG("3.5mm RC: Play Pressed");
			key = HS_MGR_KEYCODE_MEDIA;
			break;
		case 2:
			SYS_MSG("3.5mm RC: BACKWARD Pressed");
			key = HS_MGR_KEYCODE_BACKWARD;
			break;
		case 3:
			SYS_MSG("3.5mm RC: FORWARD Pressed");
			key = HS_MGR_KEYCODE_FORWARD;
			break;
		default:
			SYS_MSG("3.5mm RC: WRONG Button Pressed");
			return;
		}
		headset_button_event(1, key);
	} else { /* key release */
		if (atomic_read(&hi->btn_state))
			headset_button_event(0, atomic_read(&hi->btn_state));
		else
			SYS_MSG("3.5mm RC: WRONG Button Release");
	}

	wake_lock_timeout(&hi->headset_wake_lock, 1.5 * HZ);

	kfree(work);
}

static void enable_metrico_headset(int enable)
{
	if (enable && !hi->metrico_status) {
#if 0
		enable_mos_test(1);
#endif
		hi->metrico_status = 1;
		SYS_MSG("Enable metrico headset");
	}

	if (!enable && hi->metrico_status) {
#if 0
		enable_mos_test(0);
#endif
		hi->metrico_status = 0;
		SYS_MSG("Disable metrico headset");
	}
}

static void remove_headset(void)
{
	int state;

	wake_lock_timeout(&hi->headset_wake_lock, 2.5 * HZ);

	DBG_MSG();
	/*To solve the insert, remove, insert headset problem*/
	if (time_before_eq(jiffies, hi->insert_jiffies))
		msleep(800);
	if (hi->is_ext_insert) {
		SYS_MSG("Skip 3.5mm headset plug out!!!");
		return;
	}

	SYS_MSG("Remove 3.5mm headset");
	set_35mm_hw_state(0);

	/* For HW Metrico lab test */
	if (hi->metrico_status)
		enable_metrico_headset(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->ext_35mm_status = HTC_35MM_UNPLUG;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);

#if 0
	if (hi->cable_in1 && !gpio_get_value(hi->cable_in1)) {
		state &= ~BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
		queue_delayed_work(detect_wq, &detect_h2w_work,
				   HS_DELAY_ZERO_JIFFIES);
	} else {
		state &= ~MASK_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
	}
#else
	state &= ~MASK_35MM_HEADSET;
	switch_set_state(&hi->sdev, state);
#endif

	mutex_unlock(&hi->mutex_lock);
}

static void insert_headset(void)
{
	int state;
	int i, mic1, mic2;

	DBG_MSG();
	hi->insert_jiffies = jiffies + HZ;

	wake_lock_timeout(&hi->headset_wake_lock, 1.5 * HZ);

	if (hi->is_ext_insert) {
		SYS_MSG("Insert 3.5mm headset");
		set_35mm_hw_state(1);

		mic1 = mic2 = HEADSET_NO_MIC;
		if (hs_mgr_notifier.mic_status) {
			if (hi->ext_35mm_status == HTC_35MM_NO_MIC ||
			    hi->h2w_35mm_status == HTC_35MM_NO_MIC)
				for (i = 0; i < 10; i++) {
					mic1 = hs_mgr_notifier.mic_status();
					msleep(HS_DELAY_MIC_DETECT);
					mic2 = hs_mgr_notifier.mic_status();
					if (mic1 == mic2)
						break;
				}
			else
				mic1 = mic2 = hs_mgr_notifier.mic_status();
		}

		/* For HW Metrico lab test */
		if (mic2 == HEADSET_METRICO && !hi->metrico_status)
			enable_metrico_headset(1);

		mutex_lock(&hi->mutex_lock);
		state = switch_get_state(&hi->sdev);
		state &= ~MASK_HEADSET;
		if (mic2 == HEADSET_NO_MIC || mic1 != mic2) {
			state |= BIT_HEADSET_NO_MIC;
			SYS_MSG("3.5mm_headset without microphone");
		} else {
			state |= BIT_HEADSET;
			SYS_MSG("3.5mm_headset with microphone");
		}

		state |= BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
		if (state & BIT_HEADSET_NO_MIC)
			hi->ext_35mm_status = HTC_35MM_NO_MIC;
		else
			hi->ext_35mm_status = HTC_35MM_MIC;
		mutex_unlock(&hi->mutex_lock);
	}
}

int htc_35mm_remote_notify_insert_ext_headset(int insert)
{
	if (hi) {
		mutex_lock(&hi->mutex_lock);
		hi->is_ext_insert = insert;
		mutex_unlock(&hi->mutex_lock);

		SYS_MSG(" %d", hi->is_ext_insert);
		if (!hi->is_ext_insert)
			remove_headset();
		else
			insert_headset();
	}
	return 1;
}

int htc_35mm_remote_notify_button_status(int key_level)
{
	struct button_work *work;

	if (hi->ext_35mm_status == HTC_35MM_NO_MIC ||
		hi->h2w_35mm_status == HTC_35MM_NO_MIC) {
		SYS_MSG("MIC re-detection");
		msleep(HS_DELAY_MIC_DETECT);
		insert_headset();
	} else if (!hs_hpin_stable()) {
		SYS_MSG("The HPIN is unstable, SKIP THE BUTTON EVENT.");
		return 1;
	} else {
		work = kzalloc(sizeof(struct button_work), GFP_KERNEL);
		if (!work) {
			SYS_MSG("Failed to allocate button memory");
			return 1;
		}
		work->key_code = key_level;
		INIT_DELAYED_WORK(&work->key_work, button_35mm_do_work);
		queue_delayed_work(button_wq, &work->key_work,
				   HS_JIFFIES_BUTTON);
	}

	return 1;
}

static void usb_headset_detect(int type)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);

	switch (type) {
	case USB_NO_HEADSET:
		SYS_MSG("Remove USB_HEADSET");
		hi->usb_headset.type = USB_NO_HEADSET;
		hi->usb_headset.status = STATUS_DISCONNECTED;
		state &= ~MASK_USB_HEADSET;
		break;
	case USB_AUDIO_OUT:
		SYS_MSG("Insert USB_AUDIO_OUT");
		hi->usb_headset.type = USB_AUDIO_OUT;
		hi->usb_headset.status = STATUS_CONNECTED_ENABLED;
		state |= BIT_USB_AUDIO_OUT;
		break;
	default:
		SYS_MSG("Unknown headset type");
	}

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
}

void headset_ext_detect(int type)
{
	switch (type) {
	case H2W_NO_HEADSET:
		/* Release Key */
	case H2W_HEADSET:
	case H2W_35MM_HEADSET:
	case H2W_REMOTE_CONTROL:
	case H2W_USB_CRADLE:
	case H2W_UART_DEBUG:
	case H2W_TVOUT:
		break;
	case USB_NO_HEADSET:
		/* Release Key */
	case USB_AUDIO_OUT:
		usb_headset_detect(type);
		break;
	default:
		SYS_MSG("Unknown headset type");
	}
}

void headset_ext_button(int headset_type, int key_code, int press)
{
	SYS_MSG("Headset %d, Key %d, Press %d", headset_type, key_code, press);
	headset_button_event(press, key_code);
}

int switch_send_event(unsigned int bit, int on)
{
	unsigned long state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(bit);

	if (on)
		state |= bit;

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static ssize_t tty_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	mutex_lock(&hi->mutex_lock);
	s += sprintf(s, "%d\n", hi->tty_enable_flag);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t tty_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_TTY_FULL | BIT_TTY_VCO | BIT_TTY_HCO);

	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		hi->tty_enable_flag = 1;
		switch_set_state(&hi->sdev, state | BIT_TTY_FULL);
		mutex_unlock(&hi->mutex_lock);
		SYS_MSG("Enable TTY FULL");
		return count;
	}
	if (count == (strlen("vco_enable") + 1) &&
	   strncmp(buf, "vco_enable", strlen("vco_enable")) == 0) {
		hi->tty_enable_flag = 2;
		switch_set_state(&hi->sdev, state | BIT_TTY_VCO);
		mutex_unlock(&hi->mutex_lock);
		SYS_MSG("Enable TTY VCO");
		return count;
	}
	if (count == (strlen("hco_enable") + 1) &&
	   strncmp(buf, "hco_enable", strlen("hco_enable")) == 0) {
		hi->tty_enable_flag = 3;
		switch_set_state(&hi->sdev, state | BIT_TTY_HCO);
		mutex_unlock(&hi->mutex_lock);
		SYS_MSG("Enable TTY HCO");
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->tty_enable_flag = 0;
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
		SYS_MSG("Disable TTY");
		return count;
	}
	SYS_MSG("tty_enable_flag_store: invalid argument");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(tty, 0666, tty_flag_show, tty_flag_store);

static ssize_t fm_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_FM_HEADSET | BIT_FM_SPEAKER);

	if (count == (strlen("fm_headset") + 1) &&
	   strncmp(buf, "fm_headset", strlen("fm_headset")) == 0) {
		hi->fm_flag = 1;
		state |= BIT_FM_HEADSET;
		SYS_MSG("Enable FM HEADSET");
	} else if (count == (strlen("fm_speaker") + 1) &&
	   strncmp(buf, "fm_speaker", strlen("fm_speaker")) == 0) {
		hi->fm_flag = 2;
		state |= BIT_FM_SPEAKER;
		SYS_MSG("Enable FM SPEAKER");
	} else if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->fm_flag = 0 ;
		SYS_MSG("Disable FM");
	} else {
		mutex_unlock(&hi->mutex_lock);
		SYS_MSG("fm_enable_flag_store: invalid argument");
		return -EINVAL;
	}
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return count;
}

static ssize_t fm_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	mutex_lock(&hi->mutex_lock);
	if (hi->fm_flag == 0)
		show_str = "disable";
	if (hi->fm_flag == 1)
		show_str = "fm_headset";
	if (hi->fm_flag == 2)
		show_str = "fm_speaker";

	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}
static DEVICE_ACCESSORY_ATTR(fm, 0666, fm_flag_show, fm_flag_store);

static int register_common_headset(struct htc_headset_mgr_info *h2w)
{
	int ret = 0;
	hi = h2w;

	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		goto err_create_class;
	}

	hi->tty_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		goto err_create_tty_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret)
		goto err_create_tty_device_file;

	hi->fm_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		goto err_create_fm_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret)
		goto err_create_fm_device_file;

	return 0;

err_create_fm_device_file:
	device_unregister(hi->fm_dev);
err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);
err_create_tty_device_file:
	device_unregister(hi->tty_dev);
err_create_tty_device:
	class_destroy(hi->htc_accessory_class);
err_create_class:

	return ret;
}

static void unregister_common_headset(struct htc_headset_mgr_info *h2w)
{
	hi = h2w;
	device_remove_file(hi->tty_dev, &dev_attr_tty);
	device_unregister(hi->tty_dev);
	device_remove_file(hi->fm_dev, &dev_attr_fm);
	device_unregister(hi->fm_dev);
	class_destroy(hi->htc_accessory_class);
}

static int htc_35mm_probe(struct platform_device *pdev)
{
	int ret;

	struct htc_headset_mgr_platform_data *pdata = pdev->dev.platform_data;

	SYS_MSG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_headset_mgr_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	hi->pdata.driver_flag = pdata->driver_flag;
	hi->pdata.headset_devices_num = pdata->headset_devices_num;
	hi->pdata.headset_devices = pdata->headset_devices;

	hi->driver_init_seq = 0;

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	hi->hpin_jiffies = jiffies;
	hi->usb_headset.type = USB_NO_HEADSET;
	hi->usb_headset.status = STATUS_DISCONNECTED;

	hi->ext_35mm_status = 0;
	hi->h2w_35mm_status = 0;
	hi->is_ext_insert = 0;
	hi->mic_bias_state = 0;
	hi->key_level_flag = -1;

	atomic_set(&hi->btn_state, 0);

	hi->tty_enable_flag = 0;
	hi->fm_flag = 0;
	hi->mic_switch_flag = 1;

	mutex_init(&hi->mutex_lock);

	wake_lock_init(&hi->headset_wake_lock, WAKE_LOCK_SUSPEND, "headset");

	hi->sdev.name = "h2w";
	hi->sdev.print_name = h2w_print_name;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	button_wq = create_workqueue("button");
	if (button_wq  == NULL) {
			ret = -ENOMEM;
			goto err_create_button_work_queue;
	}

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "h2w headset";
	set_bit(EV_SYN, hi->input->evbit);
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_END, hi->input->keybit);
	set_bit(KEY_MUTE, hi->input->keybit);
	set_bit(KEY_VOLUMEDOWN, hi->input->keybit);
	set_bit(KEY_VOLUMEUP, hi->input->keybit);
	set_bit(KEY_NEXTSONG, hi->input->keybit);
	set_bit(KEY_PLAYPAUSE, hi->input->keybit);
	set_bit(KEY_PREVIOUSSONG, hi->input->keybit);
	set_bit(KEY_MEDIA, hi->input->keybit);
	set_bit(KEY_SEND, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
	goto err_register_input_dev;

	ret = register_common_headset(hi);
	if (ret)
		goto err_register_common_headset;

	if (hi->pdata.driver_flag & DRIVER_HS_MGR_RPC_SERVER) {
		/* Create RPC server */
		ret = msm_rpc_create_server(&hs_rpc_server);
		if (ret < 0) {
			SYS_MSG("Failed to create RPC server");
			goto err_create_rpc_server;
		}
		SYS_MSG("Create RPC server successfully");
	}

	hs_notify_driver_ready(DRIVER_NAME);

	SYS_MSG("--------------------");

	return 0;

err_create_rpc_server:

err_register_common_headset:
	input_unregister_device(hi->input);

err_register_input_dev:
	input_free_device(hi->input);

err_request_input_dev:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	switch_dev_unregister(&hi->sdev);

err_switch_dev_register:

	SYS_MSG("H2W: Failed to register driver");

	return ret;
}

static int htc_35mm_remove(struct platform_device *pdev)
{
	DBG_MSG();

#if 0
	if ((switch_get_state(&hi->sdev) & MASK_HEADSET) != 0)
		remove_headset();
#endif

	unregister_common_headset(hi);
	input_unregister_device(hi->input);
	destroy_workqueue(button_wq);
	switch_dev_unregister(&hi->sdev);

	return 0;
}

static struct platform_driver htc_35mm_driver = {
	.probe		= htc_35mm_probe,
	.remove		= htc_35mm_remove,
	.driver		= {
		.name		= "HTC_HEADSET_MGR",
		.owner		= THIS_MODULE,
	},
};


static int __init htc_35mm_init(void)
{
	DBG_MSG();

	return platform_driver_register(&htc_35mm_driver);
}

static void __exit htc_35mm_exit(void)
{
	DBG_MSG();

	platform_driver_unregister(&htc_35mm_driver);
}

module_init(htc_35mm_init);
module_exit(htc_35mm_exit);

MODULE_DESCRIPTION("HTC headset manager driver");
MODULE_LICENSE("GPL");
