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


#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VERS	0x00010002

#define BATTERY_RPC_CB_PROG	0x31000089
#define BATTERY_RPC_CB_VERS	0x00010001

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VERS		0x00010001

#define BATTERY_REGISTER_PROC 				2
#define BATTERY_GET_CLIENT_INFO_PROC 			3
#define BATTERY_MODIFY_CLIENT_PROC 			4
#define BATTERY_DEREGISTER_CLIENT_PROC 			5
#define BATTERY_SERVICE_TABLES_PROC 			6
#define BATTERY_IS_SERVICING_TABLES_ENABLED_PROC 	7
#define BATTERY_ENABLE_TABLE_SERVICING_PROC 		8
#define BATTERY_DISABLE_TABLE_SERVICING_PROC 		9
#define BATTERY_READ_PROC 				10
#define BATTERY_MIMIC_LEGACY_VBATT_READ_PROC 		11
#define BATTERY_CB_TYPE_PROC 1
#define BATTERY_ENABLE_DISABLE_FILTERING_PROC       14
#define BATTERY_QUERY_ADC_PARAMS   16

#define BATTERY_CB_ID_ALL_ACTIV       1

#define BATTERY_CB_ID_LOW_VOL         2

#define BATTERY_LOW             3200
#define BATTERY_HIGH            4200

#define BATTERY_LOW_CORECTION   100

#define ONCRPC_CHG_IS_CHARGING_PROC 2
#define ONCRPC_CHG_IS_CHARGER_VALID_PROC 3
#define ONCRPC_CHG_IS_BATTERY_VALID_PROC 4
#define ONCRPC_CHG_UI_EVENT_READ_PROC 5

#define BATT_RPC_TIMEOUT    10000	/* 10 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))

#define DEBUG  1

#if DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do {} while (0)
#endif

enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	BATTERY_LAST_ERROR = 32,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

enum {
	CHG_UI_EVENT_IDLE,	/* Starting point, no charger.  */
	CHG_UI_EVENT_NO_POWER,	/* No/Weak Battery + Weak Charger. */
	CHG_UI_EVENT_VERY_LOW_POWER,	/* No/Weak Battery + Strong Charger. */
	CHG_UI_EVENT_LOW_POWER,	/* Low Battery + Strog Charger.  */
	CHG_UI_EVENT_NORMAL_POWER, /* Enough Power for most applications. */
	CHG_UI_EVENT_DONE,	/* Done charging, batt full.  */
	CHG_UI_EVENT_INVALID,
	CHG_UI_EVENT_MAX32 = 0x7fffffff
};

/* Generic type definition used to enable/disable charger functions */
enum {
	CHG_CMD_DISABLE,
	CHG_CMD_ENABLE,
	CHG_CMD_INVALID,
	CHG_CMD_MAX32 = 0x7fffffff
};

struct batt_client_registration_req {

	struct rpc_request_hdr hdr;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	struct rpc_reply_hdr hdr;
	u32 batt_clnt_handle;
};

struct msm_battery_info {

	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 batt_technology;

	u32 avail_chg_sources;
	u32 current_chg_source;

	u32 batt_status;
	u32 batt_health;
	u32 voltage_now;
	u32 charger_valid;
	u32 batt_valid;
	u32 batt_capacity;
    u32 batt_temperature;

	u32(*calculate_capacity) (u32 voltage);

	s32 batt_handle;

	spinlock_t lock;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;

	struct msm_rpc_endpoint *batt_ep;
	struct msm_rpc_endpoint *battevt_ep;
	struct msm_rpc_endpoint *chg_ep;

	struct workqueue_struct *msm_batt_wq;

	struct task_struct *cb_thread;

	wait_queue_head_t wait_q;

	struct early_suspend early_suspend;

	atomic_t stop_cb_thread;
	atomic_t cb_thread_stopped;
};

static void msm_batt_wait_for_batt_chg_event(struct work_struct *work);

static DECLARE_WORK(msm_batt_cb_work, msm_batt_wait_for_batt_chg_event);

static int msm_batt_cleanup(void);

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
};

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {

			val->intval = msm_batt_info.current_chg_source & AC_CHG
			    ? 1 : 0;
			printk(KERN_INFO "power supply=%s  online = %d \n",
			       psy->name, val->intval);
		}

		if (psy->type == POWER_SUPPLY_TYPE_USB) {

			val->intval = msm_batt_info.current_chg_source & USB_CHG
			    ? 1 : 0;
			printk(KERN_INFO "power supply=%s  online = %d \n",
			       psy->name, val->intval);
		}

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
};

static void msm_batt_update_psy_status(void);

static void msm_batt_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_batt_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_batt_info.voltage_now;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = (msm_batt_info.batt_temperature * 10);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
	.external_power_changed = msm_batt_external_power_changed,
};

static int msm_batt_get_batt_chg_status(u32 *batt_charging,
					u32 *charger_valid,
					u32 *chg_batt_event, u32 *temperature)
{
	struct rpc_request_hdr req_batt_chg;

	struct rpc_reply_batt_volt {
		struct rpc_reply_hdr hdr;
		u32 voltage;
	} rep_volt;

	struct rpc_reply_chg_reply {
		struct rpc_reply_hdr hdr;
		u32 chg_batt_data;
	} rep_chg;

    struct rpc_req_adc_params {
      struct rpc_request_hdr hdr;
      u32 ptr_not_null;
    } req_adc;

    struct rpc_resp_adc_params {
      struct rpc_reply_hdr hdr;
      u32 temp_data;
      u32 batt_adc_voltage;
      u32 prev_batt_adc_voltage;
      u32 batt_temperature;
      u32 batt_id;
    } resp_adc;

	int rc;
	*batt_charging = 0;
	*chg_batt_event = CHG_UI_EVENT_INVALID;
	*charger_valid = 0;
    *temperature = 0; 

	rc = msm_rpc_call_reply(msm_batt_info.batt_ep,
				BATTERY_READ_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_volt, sizeof(rep_volt),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, BATTERY_READ_PROC, rc);

		return rc;
	}
	msm_batt_info.voltage_now = be32_to_cpu(rep_volt.voltage);

	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_IS_CHARGING_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_chg, sizeof(rep_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_IS_CHARGING_PROC, rc);
		return rc;
	}
	*batt_charging = be32_to_cpu(rep_chg.chg_batt_data);

	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_IS_CHARGER_VALID_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_chg, sizeof(rep_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_IS_CHARGER_VALID_PROC, rc);
		return rc;
	}
	*charger_valid = be32_to_cpu(rep_chg.chg_batt_data);

	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_IS_BATTERY_VALID_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_chg, sizeof(rep_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_IS_BATTERY_VALID_PROC, rc);
		return rc;
	}
	msm_batt_info.batt_valid = be32_to_cpu(rep_chg.chg_batt_data);

	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_UI_EVENT_READ_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_chg, sizeof(rep_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_UI_EVENT_READ_PROC, rc);
		return rc;
	}
	*chg_batt_event = be32_to_cpu(rep_chg.chg_batt_data);


    req_adc.ptr_not_null = be32_to_cpu(1);
	rc = msm_rpc_call_reply(msm_batt_info.batt_ep,
				BATTERY_QUERY_ADC_PARAMS,
				&req_adc, sizeof(req_adc),
				&resp_adc, sizeof(resp_adc),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, BATTERY_QUERY_ADC_PARAMS, rc);
		return rc;
	}
    *temperature = be32_to_cpu(resp_adc.batt_temperature);

	return 0;
}

/* Added temerature updates to the driver,
   No official drop available from QCOM yet for this
   Please use this version of function when merging */
static void msm_batt_update_psy_status(void)
{
	u32 batt_charging = 0;
	u32 chg_batt_event = CHG_UI_EVENT_INVALID;
	u32 charger_valid = 0;
    u32 temperature = 0;

	msm_batt_get_batt_chg_status(&batt_charging, &charger_valid,
				     &chg_batt_event, &temperature);

	printk(KERN_INFO "batt_charging = %u  batt_valid = %u "
			" batt_volt = %u\n charger_valid = %u "
			" chg_batt_event = %u temperature = %u\n",
			batt_charging, msm_batt_info.batt_valid,
			msm_batt_info.voltage_now,
			charger_valid, chg_batt_event, temperature);

	printk(KERN_INFO "Previous charger valid status = %u"
			"  current charger valid status = %u\n",
			msm_batt_info.charger_valid, charger_valid);

    msm_batt_info.batt_temperature = temperature;

	if (msm_batt_info.charger_valid != charger_valid) {

		msm_batt_info.charger_valid = charger_valid;
		if (msm_batt_info.charger_valid)
			msm_batt_info.current_chg_source |= USB_CHG;
		else
			msm_batt_info.current_chg_source &= ~USB_CHG;
		power_supply_changed(&msm_psy_usb);
	}

	if (msm_batt_info.batt_valid) {

		if (msm_batt_info.voltage_now >
		    msm_batt_info.voltage_max_design)
			msm_batt_info.batt_health =
			    POWER_SUPPLY_HEALTH_OVERVOLTAGE;

		else if (msm_batt_info.voltage_now
			 < msm_batt_info.voltage_min_design)
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
		else
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;

		if (batt_charging && msm_batt_info.charger_valid)
			msm_batt_info.batt_status =
			    POWER_SUPPLY_STATUS_CHARGING;
		else if (!batt_charging)
			msm_batt_info.batt_status =
			    POWER_SUPPLY_STATUS_DISCHARGING;

		if (chg_batt_event == CHG_UI_EVENT_DONE)
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;

		msm_batt_info.batt_capacity =
		    msm_batt_info.calculate_capacity(msm_batt_info.voltage_now);

	} else {
		msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
		msm_batt_info.batt_capacity = 0;
	}

	power_supply_changed(&msm_psy_batt);
}

static int msm_batt_enable_disable_filtering(u32 handle, bool state)
{
    int rc;
    struct batt_filter_req {
        struct rpc_request_hdr hdr;
        u32 handle;
        u32 state;
        u32 filter_number;
    } batt_filter_rpc_req;

    struct batt_filter_reply {
        struct rpc_reply_hdr hdr;
        u32 batt_error;
    } batt_filter_rpc_reply;

    batt_filter_rpc_req.handle = cpu_to_be32(handle);
    batt_filter_rpc_req.state =  cpu_to_be32(state ? 1 : 0);
    batt_filter_rpc_req.filter_number = cpu_to_be32(2); /*this indexes to vbatt_filter_profile1 on A9*/

    rc = msm_rpc_call_reply(msm_batt_info.battevt_ep,
                BATTERY_ENABLE_DISABLE_FILTERING_PROC,
                &batt_filter_rpc_req,
                sizeof(batt_filter_rpc_req),
                &batt_filter_rpc_reply,
                sizeof(batt_filter_rpc_reply),
                msecs_to_jiffies(BATT_RPC_TIMEOUT));

    if(rc <0) {
        printk(KERN_ERR
            "%s: msm_rcp_call_reply failed! proc=%d rc=%d\n" ,
            __func__, BATTERY_ENABLE_DISABLE_FILTERING_PROC, rc);
        return rc;
    }

    if(batt_filter_rpc_reply.batt_error != 0) {
        printk(KERN_ERR
            "%s: vBatt enable_disable_filter failed "
            "proce_num = %d"
            "batt_clnt_handle = %d\n"
            "state = %d",
            __func__, BATTERY_ENABLE_DISABLE_FILTERING_PROC, handle, state);
        return -EIO;
    }

    return 0;
}




static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req req;
	struct batt_client_registration_rep rep;
	int rc;
    /* 
       MOT changes for registration errors
    */
    u32 local_more_data = sizeof(u32);
    u32 local_batt_error = BATTERY_LAST_ERROR;
    /* 
       end MOT changes for registration errors
    */

	req.desired_batt_voltage = cpu_to_be32(desired_batt_voltage);
	req.voltage_direction = cpu_to_be32(voltage_direction);
	req.batt_cb_id = cpu_to_be32(batt_cb_id);
	req.cb_data = cpu_to_be32(cb_data);
    /*
           MOT changes for registration errors
     */
    req.more_data = cpu_to_be32(local_more_data);
    req.batt_error = cpu_to_be32(local_batt_error);
    /* 
        end MOT changes for registration errors
    */

	rc = msm_rpc_call_reply(msm_batt_info.battevt_ep,
				BATTERY_REGISTER_PROC, &req,
				sizeof(req), &rep, sizeof(rep),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, BATTERY_REGISTER_PROC, rc);
		return rc;
	} else {
		rc = be32_to_cpu(rep.batt_clnt_handle);
		printk(KERN_INFO " batt_clnt_handle = %d\n", rc);
		return rc;
	}
}

static int msm_batt_deregister(u32 handle)
{
	int rc;
	struct batt_client_deregister_req {
		struct rpc_request_hdr req;
		s32 handle;
	} batt_deregister_rpc_req;

	struct batt_client_deregister_reply {
		struct rpc_reply_hdr hdr;
		u32 batt_error;
	} batt_deregister_rpc_reply;

	batt_deregister_rpc_req.handle = cpu_to_be32(handle);
	batt_deregister_rpc_reply.batt_error = cpu_to_be32(BATTERY_LAST_ERROR);

	rc = msm_rpc_call_reply(msm_batt_info.battevt_ep,
				BATTERY_DEREGISTER_CLIENT_PROC,
				&batt_deregister_rpc_req,
				sizeof(batt_deregister_rpc_req),
				&batt_deregister_rpc_reply,
				sizeof(batt_deregister_rpc_reply),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		printk(KERN_ERR
		       "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, BATTERY_DEREGISTER_CLIENT_PROC, rc);
		return rc;
	}

	if (be32_to_cpu(batt_deregister_rpc_reply.batt_error) !=
			BATTERY_DEREGISTRATION_SUCCESSFUL) {

		printk(KERN_ERR "%s: vBatt deregistration Failed "
		       "  proce_num = %d,"
		       " batt_clnt_handle = %d\n",
		       __func__, BATTERY_DEREGISTER_CLIENT_PROC, handle);
		return -EIO;
	}
	return 0;
}

static void msm_batt_suspend_cleanup(void)
{
	int rc;
	void *rpc_packet;
   
	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0) {
			printk(KERN_ERR
			       "%s(): msm_batt_deregister failed rc=%d\n",
			       __func__, rc);
		}
	}

	rpc_packet = NULL;

	do {
		rc = msm_rpc_read(msm_batt_info.batt_ep, &rpc_packet, -1, HZ);

		if (!rc) {
			printk(KERN_INFO "%s(): Packet left in Queue\n",
			       __func__);
			kfree(rpc_packet);
		} else if (rc != -ETIMEDOUT)
			printk(KERN_INFO "%s(): Packet read error\n", __func__);
		else if (rc == -ETIMEDOUT)
			printk(KERN_INFO "%s(): rpc read timed out. No packet"
			       "from modem. can susped now\n", __func__);

	} while (rc != -ETIMEDOUT);
}


static void msm_batt_wait_for_batt_chg_event(struct work_struct *work)
{
	void *rpc_packet;
	struct rpc_request_hdr *req;
	int rpc_packet_type;
	struct rpc_reply_hdr rpc_reply;
	int len;
	unsigned long flags;

	spin_lock_irqsave(&msm_batt_info.lock, flags);
	msm_batt_info.cb_thread = current;
	spin_unlock_irqrestore(&msm_batt_info.lock, flags);

	printk(KERN_INFO "%s: Batt RPC call back thread"
	       " started.\n", __func__);

	allow_signal(SIGCONT);

	while (!atomic_read(&msm_batt_info.stop_cb_thread)) {

		printk(KERN_INFO "%s: Update Batt status.\n", __func__);

		msm_batt_update_psy_status();

		rpc_packet = NULL;

		len = msm_rpc_read(msm_batt_info.battevt_ep, &rpc_packet, -1, -1);


		if (len == -ERESTARTSYS) {
			if (atomic_read(&msm_batt_info.stop_cb_thread)) {
				printk(KERN_INFO "%s(): batt call back thread"
				       " while  in msm_rpc_read got  signal"
				       " from early suspend thread to stop.\n",
				       __func__);

				flush_signals(current);

				msm_batt_suspend_cleanup();

			} else {
				printk(KERN_ERR "%s(): batt call back thread"
				       " while  in msm_rpc_read got  signal."
				       " Signal is not  from early suspend "
				       "thread.\n", __func__);

				flush_signals(current);
			}

			atomic_set(&msm_batt_info.cb_thread_stopped, 1);
			wake_up(&msm_batt_info.wait_q);
			break;
		}

		printk(KERN_INFO "%s: Got some packet from"
		       "modem Vbatt server\n", __func__);

		if (len < 0) {
			printk(KERN_ERR "%s: msm_rpc_read failed while "
			       "waiting for call back packet. rc= %d\n",
			       __func__, len);
			continue;
		}

		if (len < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			printk(KERN_ERR "%s: The pkt is neither req nor reply."
			       " len of pkt = %d\n", __func__, len);
			if (!rpc_packet)
				kfree(rpc_packet);
			continue;
		}

		req = (struct rpc_request_hdr *)rpc_packet;

		rpc_packet_type = be32_to_cpu(req->type);

		if (rpc_packet_type == RPC_TYPE_REPLY) {
			printk(KERN_ERR "%s: Should not get rpc reply"
			       " Type_of_packet = %d\n",
			       __func__, rpc_packet_type);
			kfree(rpc_packet);
			continue;
		}
		if (rpc_packet_type != RPC_TYPE_REQ) {
			printk(KERN_ERR "%s: Type_of_packet is neither req or"
			       " reply. Type_of_packet = %d\n",
			       __func__, rpc_packet_type);
			kfree(rpc_packet);
			continue;
		}

		req->type = be32_to_cpu(req->type);
		req->xid = be32_to_cpu(req->xid);
		req->rpc_vers = be32_to_cpu(req->rpc_vers);

		if (req->rpc_vers != 2) {
			printk(KERN_ERR "%s: incorrect rpc version = %d\n",
			       __func__, req->rpc_vers);
			kfree(rpc_packet);
			continue;
		}

		req->prog = be32_to_cpu(req->prog);
		if (req->prog != BATTERY_RPC_CB_PROG) {
			printk(KERN_ERR "%s: Invalid Prog number for rpc call"
			       " back req. prog number = %d\n",
			       __func__, req->prog);
			kfree(rpc_packet);
			continue;
		}

		req->procedure = be32_to_cpu(req->procedure);

		if (req->procedure != BATTERY_CB_TYPE_PROC) {
			printk(KERN_ERR "%s: Invalid procedure num  rpc call"
			       " back req. req->procedure = %d\n",
			       __func__, req->procedure);
			kfree(rpc_packet);
			continue;
		}

		rpc_reply.xid = cpu_to_be32(req->xid);
		rpc_reply.type = cpu_to_be32(RPC_TYPE_REPLY);
		rpc_reply.reply_stat = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);
		rpc_reply.data.acc_hdr.accept_stat =
		    cpu_to_be32(RPC_ACCEPTSTAT_SUCCESS);
		rpc_reply.data.acc_hdr.verf_flavor = 0;
		rpc_reply.data.acc_hdr.verf_length = 0;

		len = msm_rpc_write(msm_batt_info.battevt_ep,
				    &rpc_reply, sizeof(rpc_reply));
		if (len < 0)
			printk(KERN_ERR "%s: could not send rpc reply for"
			       " call back from  batt server."
			       " reply  write response %d\n", __func__, len);

		kfree(rpc_packet);
	}

	printk(KERN_INFO "%s: Batt RPC call back thread stopped.\n", __func__);
}

static int msm_batt_stop_cb_thread(void)
{
	int rc;
	unsigned long flags;

	rc = 0;

	spin_lock_irqsave(&msm_batt_info.lock, flags);

	if (msm_batt_info.cb_thread) {
		atomic_set(&msm_batt_info.stop_cb_thread, 1);
		send_sig(SIGCONT, msm_batt_info.cb_thread, 0);
		spin_unlock_irqrestore(&msm_batt_info.lock, flags);

		rc = wait_event_interruptible(msm_batt_info.wait_q,
			atomic_read(&msm_batt_info.  cb_thread_stopped));

		if (rc == -ERESTARTSYS)
			printk(KERN_ERR "%s(): suspend thread got signal"
				"while wating for batt thread to finish\n",
				__func__);
		else if (rc < 0)
			printk(KERN_ERR "%s(): suspend thread wait returned "
				"error while waiting for batt thread"
				"to finish. rc =%d\n", __func__, rc);
		else
			printk(KERN_INFO "%s(): suspend thread wait returned "
				"rc =%d\n", __func__, rc);

		atomic_set(&msm_batt_info.cb_thread_stopped, 0);
	} else {
		atomic_set(&msm_batt_info.stop_cb_thread, 1);
		spin_unlock_irqrestore(&msm_batt_info.lock, flags);
	}

	return rc;
}

static void msm_batt_start_cb_thread(void)
{
	atomic_set(&msm_batt_info.stop_cb_thread, 0);
	atomic_set(&msm_batt_info.cb_thread_stopped, 0);
	queue_work(msm_batt_info.msm_batt_wq, &msm_batt_cb_work);
}

static void msm_batt_early_suspend(struct early_suspend *h);

static int msm_batt_cleanup(void)
{
	int rc = 0;
	int rc_local;

	if (msm_batt_info.msm_batt_wq) {
		msm_batt_stop_cb_thread();
		destroy_workqueue(msm_batt_info.msm_batt_wq);
	}

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			printk(KERN_ERR
			       "%s: msm_batt_deregister failed rc=%d\n",
			       __func__, rc);
	}
	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);

	if (msm_batt_info.batt_ep) {
		rc_local = msm_rpc_close(msm_batt_info.batt_ep);
		if (rc_local < 0) {
			printk(KERN_ERR
			       "%s: msm_rpc_close failed for batt_ep rc=%d\n",
			       __func__, rc_local);
			if (!rc)
				rc = rc_local;
		}
	}

	if (msm_batt_info.battevt_ep) {
		rc_local = msm_rpc_close(msm_batt_info.battevt_ep);
		if (rc_local < 0) {
			printk(KERN_ERR
			       "%s: msm_rpc_close failed for battevt_ep rc=%d\n",
			       __func__, rc_local);
			if (!rc)
				rc = rc_local;
		}
	}

	if (msm_batt_info.chg_ep) {
		rc_local = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc_local < 0) {
			printk(KERN_ERR
			       "%s: msm_rpc_close failed for chg_ep rc=%d\n",
			       __func__, rc_local);
			if (!rc)
				rc = rc_local;
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
	return rc;
}

static u32 msm_batt_capacity(u32 current_voltage)
{
	u32 low_voltage = msm_batt_info.voltage_min_design;
	u32 high_voltage = msm_batt_info.voltage_max_design;

	return (current_voltage - low_voltage) * 100
	    / (high_voltage - low_voltage);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;

	printk(KERN_INFO "%s(): going to early suspend\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;

	printk(KERN_INFO "%s(): going to resume\n", __func__);
}
#endif

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

	if (pdata->avail_chg_sources & AC_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc=%d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc=%d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {

		dev_err(&pdev->dev,
			"%s: No external Power supply(AC or USB )"
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.calculate_capacity = pdata->calculate_capacity;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;

	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	if (!msm_batt_info.calculate_capacity)
		msm_batt_info.calculate_capacity = msm_batt_capacity;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.batt_handle = rc;

    rc = msm_batt_enable_disable_filtering(msm_batt_info.batt_handle, true);

    if (rc < 0) {
        printk(KERN_ERR
              "%s(): msm_batt_enable_disable_filtering failed rc=%d\n", __func__, rc);
    }


#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif
	msm_batt_start_cb_thread();

	return 0;
}

static struct platform_driver msm_batt_driver;
static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	spin_lock_init(&msm_batt_info.lock);

	msm_batt_info.msm_batt_wq =
	    create_singlethread_workqueue("msm_battery");

	if (!msm_batt_info.msm_batt_wq) {
		printk(KERN_ERR "%s: create workque failed \n", __func__);
		return -ENOMEM;
	}

	msm_batt_info.batt_ep =
	    msm_rpc_connect_compatible(BATTERY_RPC_PROG, BATTERY_RPC_VERS, 0);

	if (msm_batt_info.batt_ep == NULL) {
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.batt_ep)) {
		int rc = PTR_ERR(msm_batt_info.batt_ep);
		printk(KERN_ERR
		       "%s: rpc connect failed for BATTERY_RPC_PROG."
		       " rc = %d\n ", __func__, rc);
		msm_batt_info.batt_ep = NULL;
		return rc;
	}

	msm_batt_info.battevt_ep =
	    msm_rpc_connect_compatible(BATTERY_RPC_PROG, BATTERY_RPC_VERS, 0);

	if (msm_batt_info.battevt_ep == NULL) {
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.battevt_ep)) {
		int rc = PTR_ERR(msm_batt_info.battevt_ep);
		printk(KERN_ERR
		       "%s: rpc connect failed for BATTERY_RPC_PROG."
		       " rc = %d\n ", __func__, rc);
		msm_batt_info.battevt_ep = NULL;
		return rc;
	}

	msm_batt_info.chg_ep =
	    msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VERS, 0);

	if (msm_batt_info.chg_ep == NULL) {
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.chg_ep)) {
		int rc = PTR_ERR(msm_batt_info.chg_ep);
		printk(KERN_ERR
		       "%s: rpc connect failed for CHG_RPC_PROG. rc = %d\n",
		       __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0) {
		printk(KERN_ERR "%s: platform_driver_register failed for "
			"batt driver. rc = %d\n", __func__, rc);
		return rc;
	}

	init_waitqueue_head(&msm_batt_info.wait_q);

	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
		   },
};

static int __init msm_batt_init(void)
{
	int rc;

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		printk(KERN_ERR "%s: msm_batt_init_rpc Failed  rc=%d\n",
		       __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	return 0;
}

static void __exit msm_batt_exit(void)
{
	platform_driver_unregister(&msm_batt_driver);
}

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_battery");
