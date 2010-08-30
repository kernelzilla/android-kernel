/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <mach/pmic.h>

#include "smd_rpcrouter.h"

#define TRACE_SPEAKER 0

#if TRACE_SPEAKER
#define SPEAKER(x...) printk(KERN_INFO "[SPEAKER] " x)
#else
#define SPEAKER(x...) do {} while (0)
#endif

/* rpc related */
#define PMIC_RPC_TIMEOUT (5*HZ)

#define SPEAKER_PDEV_NAME	"rs00010001:00000000"
#define SPEAKER_RPC_PROG	0x30000061
#define SPEAKER_RPC_VER		0x00010001

#define SET_LED_INTENSITY_PROC 16
#define FLASH_LED_SET_CURRENT_PROC 17

#define SPEAKER_CMD_PROC 20
#define SET_SPEAKER_GAIN_PROC 21

#define MIC_EN_PROC 28
#define MIC_IS_EN_PROC 29
#define MIC_SET_VOLT_PROC 30
#define MIC_GET_VOLT_PROC 31
#define SPKR_EN_RIGHT_CHAN_PROC 32
#define SPKR_IS_RIGHT_CHAN_EN_PROC 33
#define SPKR_EN_LEFT_CHAN_PROC 34
#define SPKR_IS_LEFT_CHAN_EN_PROC 35
#define SET_SPKR_CONFIGURATION_PROC 36
#define GET_SPKR_CONFIGURATION_PROC 37
#define SPKR_GET_GAIN_PROC 38
#define SPKR_IS_EN_PROC 39

/* error bit flags defined by modem side */
#define PM_ERR_FLAG__PAR1_OUT_OF_RANGE		(0x0001)
#define PM_ERR_FLAG__PAR2_OUT_OF_RANGE		(0x0002)
#define PM_ERR_FLAG__PAR3_OUT_OF_RANGE		(0x0004)
#define PM_ERR_FLAG__PAR4_OUT_OF_RANGE		(0x0008)
#define PM_ERR_FLAG__PAR5_OUT_OF_RANGE		(0x0010)

#define PM_ERR_FLAG__ALL_PARMS_OUT_OF_RANGE   	(0x001F) /* all 5 previous */

#define PM_ERR_FLAG__SBI_OPT_ERR		(0x0080)
#define PM_ERR_FLAG__FEATURE_NOT_SUPPORTED	(0x0100)

struct std_rpc_req {
	struct rpc_request_hdr req;
	uint32_t value;
};

struct std_rpc_reply {
	struct rpc_reply_hdr hdr;
	uint32_t result;
};

struct get_value_rep {
	struct std_rpc_reply reply_hdr;
	uint32_t MoreData;
	uint32_t value;
};

struct std_rpc_req2 {
	struct rpc_request_hdr hdr;
	uint32_t value1;
	uint32_t value2;
};

struct set_spkr_configuration_req {
	struct rpc_request_hdr hdr;
	uint32_t MoreData;
	struct spkr_config_mode type;
};

struct get_spkr_configuration_rep {
	struct std_rpc_reply reply_hdr;
	uint32_t MoreData;
	struct spkr_config_mode type;
};

static struct msm_rpc_endpoint *endpoint;

static int check_and_connect(void)
{
	if (endpoint != NULL)
		return 0;

	endpoint = msm_rpc_connect(SPEAKER_RPC_PROG, SPEAKER_RPC_VER, 0);
	if (endpoint == NULL) {
		return -ENODEV;
	} else if (IS_ERR(endpoint)) {
		int rc = PTR_ERR(endpoint);
		printk(KERN_ERR "%s: init rpc failed! rc = %d\n",
		       __func__, rc);
		endpoint = NULL;
		return rc;
	}
	return 0;
}

static int modem_to_linux_err(u16 err_netendian)
{
	u32 err;
	if (!err_netendian)
		return 0;

	err = be32_to_cpu(err_netendian);
	if (err & PM_ERR_FLAG__ALL_PARMS_OUT_OF_RANGE)
		return -EINVAL;	/* PM_ERR_FLAG__PAR[1..5]_OUT_OF_RANGE */

	if (err & PM_ERR_FLAG__SBI_OPT_ERR)
		return -EIO;

	if (err & PM_ERR_FLAG__FEATURE_NOT_SUPPORTED)
		return -ENOSYS;

	return -EPERM;
}

static int do_remote_value(const uint32_t set_value,
			   uint32_t * const get_value,
			   const uint32_t proc)
{
	struct std_rpc_req req;
	struct std_rpc_reply std_rep;
	struct get_value_rep rep;
	void *rep_ptr;
	int rep_size, rc = check_and_connect();

	if (rc) /* connect problem */
		return rc;

	if (get_value != NULL) { /* get value */
		req.value = cpu_to_be32(1); /* output_pointer_not_null */
		rep_size = sizeof(rep);
		rep_ptr = &rep;
	} else { /* set value */
		req.value = cpu_to_be32(set_value);
		rep_size = sizeof(std_rep);
		rep_ptr = &std_rep;
	}
	rc = msm_rpc_call_reply(endpoint, proc,
				&req, sizeof(req),
				rep_ptr, rep_size,
				PMIC_RPC_TIMEOUT);
	if (rc < 0)
		return rc;

	if (get_value != NULL) { /* get value */
		if (!rep.reply_hdr.result) {
			if (!rep.MoreData)
				return -ENOMSG;

			*get_value = be32_to_cpu(rep.value);
		}
		rc = modem_to_linux_err(rep.reply_hdr.result);
	} else {
		rc = modem_to_linux_err(std_rep.result);
	}
	return rc;
}

int spkr_en_right_chan(const unsigned char enable)
{
	return do_remote_value(enable, NULL, SPKR_EN_RIGHT_CHAN_PROC);
}
EXPORT_SYMBOL(spkr_en_right_chan);

int spkr_is_right_chan_en(unsigned char * const enabled)
{
	uint32_t word_enabled;
	int rc;

	if (enabled == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	rc = do_remote_value(0, &word_enabled, SPKR_IS_RIGHT_CHAN_EN_PROC);
	if (!rc)
		*enabled = (unsigned char)word_enabled;
	return rc;
}
EXPORT_SYMBOL(spkr_is_right_chan_en);

int spkr_en_left_chan(const unsigned char enable)
{
	return do_remote_value(enable, NULL, SPKR_EN_LEFT_CHAN_PROC);
}
EXPORT_SYMBOL(spkr_en_left_chan);

int spkr_is_left_chan_en(unsigned char * const enabled)
{
	uint32_t word_enabled;
	int rc;

	if (enabled == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	rc = do_remote_value(0, &word_enabled, SPKR_IS_LEFT_CHAN_EN_PROC);
	if (!rc)
		*enabled = (unsigned char)word_enabled;
	return rc;
}
EXPORT_SYMBOL(spkr_is_left_chan_en);

int spkr_is_en(const enum spkr_left_right left_right,
	       unsigned char * const enabled)
{
	if (left_right >= SPKR_OUT_OF_RANGE)
		return -EINVAL;

	return left_right == LEFT_SPKR ?
		spkr_is_left_chan_en(enabled) :
		spkr_is_right_chan_en(enabled);
}
EXPORT_SYMBOL(spkr_is_en);

static int do_std_rpc_req2(struct get_value_rep *rep, uint32_t proc,
				uint32_t value1, uint32_t value2)
{
	struct std_rpc_req2 req;
	int rc;

	rc = check_and_connect();
	if (rc) {
		printk(KERN_ERR "%s: can't make rpc connection!\n", __func__);
		return rc;
	}

	req.value1 = cpu_to_be32(value1);
	req.value2 = cpu_to_be32(value2);

	rc = msm_rpc_call_reply(endpoint, proc,
				    &req, sizeof(req),
				    rep, sizeof(*rep),
				    PMIC_RPC_TIMEOUT);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
			__func__, proc, rc);
		return rc;
	}

	return modem_to_linux_err(rep->reply_hdr.result);
}

int spkr_get_gain(const enum spkr_left_right left_right,
		  enum spkr_gain * const gain)
{
	struct get_value_rep rep;
	int rc;

	if (left_right >= SPKR_OUT_OF_RANGE)
		return -EINVAL;

	if (gain == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	rc = do_std_rpc_req2(&rep, SPKR_GET_GAIN_PROC, (uint32_t)left_right, 1);

	if (!rc && !rep.reply_hdr.result) {
		if (!rep.MoreData)
			return -ENOMSG;

		*gain = (enum spkr_gain)be32_to_cpu(rep.value);
	}

	return rc;
}
EXPORT_SYMBOL(spkr_get_gain);

int set_speaker_gain(const enum spkr_gain speaker_gain)
{
	if (speaker_gain >= SPKR_GAIN_OUT_OF_RANGE)
		return -EINVAL;

	return do_remote_value(speaker_gain, NULL, SET_SPEAKER_GAIN_PROC);
}
EXPORT_SYMBOL(set_speaker_gain);

int speaker_cmd(const enum spkr_cmd cmd)
{
	if (cmd >= SPKR_CMD_OUT_OF_RANGE)
		return -EINVAL;

	return do_remote_value(cmd, NULL, SPEAKER_CMD_PROC);
}
EXPORT_SYMBOL(speaker_cmd);

int set_spkr_configuration(const struct spkr_config_mode * const t)
{
	struct set_spkr_configuration_req req;
	struct std_rpc_reply rep;
	int i, rc;

	if (t == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	rc = check_and_connect();
	if (rc)
		return rc;

	for (i = 0; i < sizeof(*t)/sizeof(uint32_t); i++)
		((uint32_t *)&req.type)[i] = cpu_to_be32(((uint32_t *)t)[i]);

	req.MoreData = cpu_to_be32(1);
	rc = msm_rpc_call_reply(endpoint, SET_SPKR_CONFIGURATION_PROC,
				&req, sizeof(req),
				&rep, sizeof(rep),
				PMIC_RPC_TIMEOUT);
	if (rc < 0)
		return rc;

	return modem_to_linux_err(rep.result);
}
EXPORT_SYMBOL(set_spkr_configuration);

int get_spkr_configuration(struct spkr_config_mode * const t)
{
	struct std_rpc_req req;
	struct get_spkr_configuration_rep rep;
	int rc;

	if (t == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	rc = check_and_connect();
	if (rc)
		return rc;

	req.value = cpu_to_be32(1); /* output_pointer_not_null */
	rc = msm_rpc_call_reply(endpoint, GET_SPKR_CONFIGURATION_PROC,
				    &req, sizeof(req),
				    &rep, sizeof(rep),
				    PMIC_RPC_TIMEOUT);
	if (rc < 0)
		return rc;

	if (!rep.reply_hdr.result) {
		int i;

		if (!rep.MoreData)
			return -ENOMSG;

		for (i = 0; i < sizeof(*t)/sizeof(uint32_t); i++)
			((uint32_t *)t)[i] =
				be32_to_cpu(((uint32_t *)&rep.type)[i]);
	}

	return modem_to_linux_err(rep.reply_hdr.result);
}
EXPORT_SYMBOL(get_spkr_configuration);

int mic_en(const unsigned char enable)
{
	return do_remote_value(enable, NULL, MIC_EN_PROC);
}
EXPORT_SYMBOL(mic_en);

int mic_is_en(unsigned char * const enabled)
{
	uint32_t word_enabled;
	int rc;

	if (enabled == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	rc = do_remote_value(0, &word_enabled, MIC_IS_EN_PROC);
	if (!rc)
		*enabled = (unsigned char)word_enabled;

	return rc;
}
EXPORT_SYMBOL(mic_is_en);

int mic_set_volt(const enum mic_volt type)
{
	if (type >= MIC_VOLT_OUT_OF_RANGE)
		return -EINVAL;

	return do_remote_value(type, NULL, MIC_SET_VOLT_PROC);
}
EXPORT_SYMBOL(mic_set_volt);

int mic_get_volt(enum mic_volt * const voltage)
{
	if (voltage == NULL)
		return modem_to_linux_err(PM_ERR_FLAG__PAR1_OUT_OF_RANGE);

	return do_remote_value(0, voltage, MIC_GET_VOLT_PROC);
}
EXPORT_SYMBOL(mic_get_volt);

/* Cannot use 'current' as the parameter name because 'current' is defined as
 * a macro to get a pointer to the current task.
 */
int flash_led_set_current(const uint16_t milliamps)
{
	return do_remote_value(milliamps, NULL, FLASH_LED_SET_CURRENT_PROC);
}
EXPORT_SYMBOL(flash_led_set_current);

int set_led_intensity(const enum ledtype type, int level)
{
	struct get_value_rep rep;

	if (type >= LED_TYPE_OUT_OF_RANGE)
		return -EINVAL;

	return do_std_rpc_req2(&rep, SET_LED_INTENSITY_PROC,
				(uint32_t)type, level);
}
EXPORT_SYMBOL(set_led_intensity);

#if defined(CONFIG_DEBUG_FS)
static void debugfs_log_return_status(const int caller_rc,
				      const char * const caller__func__,
				      const u64 caller_val)
{
	if (!caller_rc)
		printk(KERN_INFO "%s: succeeded, val %llu\n",
			caller__func__, caller_val);
	else
		printk(KERN_ERR "%s: ERROR! val %llu, rc:%d(%#x)\n",
			caller__func__, caller_val, caller_rc, caller_rc);
}

static int debugfs_spkr_en_chan(void *data, u64 val)
{
	int rc = ((enum spkr_left_right)data) == LEFT_SPKR ?
		  spkr_en_left_chan((const unsigned char)val) :
		  spkr_en_right_chan((const unsigned char)val);

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}

static int debugfs_spkr_is_chan_en(void *data, u64 *val)
{
	unsigned char enabled;
	int rc = ((enum spkr_left_right)data) == LEFT_SPKR ?
		  spkr_is_left_chan_en(&enabled) :
		  spkr_is_right_chan_en(&enabled);

	if (!rc)
		*val = (u64)enabled;

	debugfs_log_return_status(rc, __func__, *val);
	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(debugfs_spkr_en_chan_fops,
			debugfs_spkr_is_chan_en,
			debugfs_spkr_en_chan,
			"%llu\n");

static int debugfs_spkr_get_gain(void *data, u64 *val)
{
	enum spkr_gain gain;
	int rc = spkr_get_gain((enum spkr_left_right)data, &gain);

	if (!rc)
		*val = (u64)gain;

	debugfs_log_return_status(rc, __func__, *val);
	return rc;
}

static int debugfs_set_speaker_gain(void *data, u64 val)
{
	int rc = set_speaker_gain((enum spkr_gain)val);

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(debugfs_spkr_gain_fops,
			debugfs_spkr_get_gain,
			debugfs_set_speaker_gain,
			"%llu\n");

static int debugfs_speaker_cmd(void *data, u64 val)
{
	int rc = speaker_cmd((const enum spkr_cmd)val);

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(debugfs_speaker_cmd_fops,
			NULL,
			debugfs_speaker_cmd,
			"%llu\n");

static int debugfs_spkr_configuration_open(struct inode *inode,
					   struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

#define debugfs_speaker_configuration_format_str \
	"%u,%u,%u,%u,%u,%u,%u,%u\n" \
	"is_right_chan_en: %u\n" \
	"is_left_chan_en: %u\n" \
	"is_right_left_chan_added: %u\n" \
	"is_stereo_en: %u\n" \
	"is_usb_with_hpf_20hz: %u\n" \
	"is_mux_bypassed: %u\n" \
	"is_hpf_en: %u\n" \
	"is_sink_curr_from_ref_volt_cir_en: %u\n"

static ssize_t debugfs_spkr_get_configuration(struct file *filp,
					      char __user *ubuf,
					      size_t cnt, loff_t *ppos)
{
	struct spkr_config_mode type;
	int rc = get_spkr_configuration(&type);

	if (!rc) {
		/* Hopefully, 100 additional chars is enough. The number is
		 * calculated by having 16 elements with up to 5 digits each
		 * and then doubling it for paranoia's sake.  In any case,
		 * snprintf will truncate, so no danger of oops or panic.
		 */
		char speaker_configuration_dump_buf[
			sizeof(debugfs_speaker_configuration_format_str) + 200];
		int bytes_copied = snprintf(speaker_configuration_dump_buf,
				sizeof(speaker_configuration_dump_buf),
				debugfs_speaker_configuration_format_str,
				type.is_right_chan_en,
				type.is_left_chan_en,
				type.is_right_left_chan_added,
				type.is_stereo_en,
				type.is_usb_with_hpf_20hz,
				type.is_mux_bypassed,
				type.is_hpf_en,
				type.is_sink_curr_from_ref_volt_cir_en,
				type.is_right_chan_en,
				type.is_left_chan_en,
				type.is_right_left_chan_added,
				type.is_stereo_en,
				type.is_usb_with_hpf_20hz,
				type.is_mux_bypassed,
				type.is_hpf_en,
				type.is_sink_curr_from_ref_volt_cir_en);

		if (bytes_copied > cnt) {
			rc = -EINVAL;
			goto out_err;
		}

		rc = (int)simple_read_from_buffer(ubuf, cnt, ppos,
				speaker_configuration_dump_buf, bytes_copied);
		if (rc < 0)
			goto out_err;

		printk(KERN_INFO "%s: succeeded, (%d)%d bytes copied\n%s",
			__func__, rc, bytes_copied,
			speaker_configuration_dump_buf);
	}
out_err:
	if (rc < 0)
		printk(KERN_ERR "%s: ERROR! rc: %d(%#x)\n", __func__, rc, rc);

	return rc;
}

#define debugfs_spkr_set_configuration_format_str "%u,%u,%u,%u,%u,%u,%u,%u"

static ssize_t debugfs_spkr_set_configuration(struct file *filp,
		const char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	char buf[100]; /* 8 unsigned ints + paranoia */
	struct spkr_config_mode t;
	int rc;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

	rc = sscanf(buf, debugfs_spkr_set_configuration_format_str,
		    &t.is_right_chan_en,
		    &t.is_left_chan_en,
		    &t.is_right_left_chan_added,
		    &t.is_stereo_en,
		    &t.is_usb_with_hpf_20hz,
		    &t.is_mux_bypassed,
		    &t.is_hpf_en,
		    &t.is_sink_curr_from_ref_volt_cir_en);

	if (rc < 8) {
		printk(KERN_ERR "%s: snprintf failed arg convert"
				" after arg #%d on buf %s\n",
			__func__, rc, buf);
		return -EINVAL;
	}

	rc = set_spkr_configuration(&t);
	if (!rc)
		printk(KERN_INFO "%s: succeeded, %s\n", __func__, buf);
	else
		printk(KERN_ERR
			"%s: set_spkr_configuration error %d(%#x), buf: %s\n",
			__func__, rc, rc, buf);

	return rc < 0 ? rc : cnt;
}

static struct file_operations debugfs_spkr_configuration_fops = {
	.open = debugfs_spkr_configuration_open,
	.read = debugfs_spkr_get_configuration,
	.write = debugfs_spkr_set_configuration,
};

static int debugfs_mic_en(void *data, u64 val)
{
	int rc = mic_en((const unsigned char)val);

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}

static int debugfs_mic_is_en(void *data, u64 *val)
{
	unsigned char enabled;
	int rc = mic_is_en(&enabled);

	if (!rc)
		*val = (u64)enabled;

	debugfs_log_return_status(rc, __func__, *val);
	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(debugfs_mic_en_fops,
			debugfs_mic_is_en,
			debugfs_mic_en,
			"%llu\n");

static int debugfs_mic_set_volt(void *data, u64 val)
{
	int rc = mic_set_volt((const enum mic_volt)val);

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}

static int debugfs_mic_get_volt(void *data, u64 *val)
{
	enum mic_volt voltage;
	int rc = mic_get_volt(&voltage);

	if (!rc)
		*val = (u64)voltage;

	debugfs_log_return_status(rc, __func__, *val);
	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(debugfs_mic_volt_fops,
			debugfs_mic_get_volt,
			debugfs_mic_set_volt,
			"%llu\n");

static uint16_t debugfs_flash_led_milliamps;
static int debugfs_flash_led_set_current_execute(void *data, u64 val)
{
	int rc = flash_led_set_current((const uint16_t)val);

	if (!rc)
		debugfs_flash_led_milliamps = (const uint16_t)val;

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}

static int debugfs_flash_led_set_current_get_cached(void *data, u64 *val)
{
	*val = (u64)debugfs_flash_led_milliamps;

	debugfs_log_return_status(0, __func__, *val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debugfs_flash_led_current_fops,
			debugfs_flash_led_set_current_get_cached,
			debugfs_flash_led_set_current_execute,
			"%llu\n");

static uint16_t debugfs_lcd_intensity;
static int debugfs_set_lcd_intensity(void *data, u64 val)
{
	int rc = set_led_intensity(LED_LCD, (const uint16_t)val);

	if (!rc)
		debugfs_lcd_intensity = (const uint16_t)val;

	debugfs_log_return_status(rc, __func__, val);
	return rc;
}

static int debugfs_set_lcd_intensity_get_cached(void *data, u64 *val)
{
	*val = (u64)debugfs_lcd_intensity;

	debugfs_log_return_status(0, __func__, *val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debugfs_set_lcd_intensity_fops,
			debugfs_set_lcd_intensity_get_cached,
			debugfs_set_lcd_intensity,
			"%llu\n");

static int __init debugfs_speaker_init(void)
{
	struct dentry *dent = debugfs_create_dir("pmic", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s: debugfs_create_dir fail, error %ld\n",
			__func__, PTR_ERR(dent));
		return (int)PTR_ERR(dent);
	}
	debugfs_create_file("spkr_left_en", 0644, dent, (void *)LEFT_SPKR,
				&debugfs_spkr_en_chan_fops);
	debugfs_create_file("spkr_right_en", 0644, dent, (void *)RIGHT_SPKR,
				&debugfs_spkr_en_chan_fops);
	debugfs_create_file("spkr_left_gain", 0644, dent, (void *)LEFT_SPKR,
				&debugfs_spkr_gain_fops);
	debugfs_create_file("spkr_right_gain", 0644, dent, (void *)RIGHT_SPKR,
				&debugfs_spkr_gain_fops);
	debugfs_create_file("spkr_cmd", 0644, dent, NULL,
				&debugfs_speaker_cmd_fops);
	debugfs_create_file("spkr_config", 0644, dent, NULL,
				&debugfs_spkr_configuration_fops);
	debugfs_create_file("mic_en", 0644, dent, NULL, &debugfs_mic_en_fops);
	debugfs_create_file("mic_volt", 0644, dent, NULL,
				&debugfs_mic_volt_fops);
	debugfs_create_file("flash_led_current", 0644, dent, NULL,
				&debugfs_flash_led_current_fops);
	debugfs_create_file("set_lcd_intensity", 0644, dent, NULL,
				&debugfs_set_lcd_intensity_fops);
	return 0;
}

late_initcall(debugfs_speaker_init);

static int __init speaker_init(void)
{
	/* try to connect initially, ignore any errors for now */
	check_and_connect();
	return 0;
}

device_initcall(speaker_init);
#endif
