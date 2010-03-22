/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/pm_qos_params.h>
#include <linux/completion.h>
#include <linux/err.h>
#include "npa_remote.h"

struct npa_client_info {
	char *resource_name;
	char *request_name;
	s32 value;
	struct npa_client **npa_client;
};

static void npa_res_available_cb(void *u_data, unsigned t, void *d, unsigned n)
{
	struct npa_client_info *client_info = (struct npa_client_info *)u_data;
	struct npa_client *npa_client;

	/* Create NPA 'required' client. */
	npa_client = npa_create_sync_client(client_info->resource_name,
		client_info->request_name, NPA_CLIENT_REQUIRED);
	if (IS_ERR(npa_client)) {
		pr_crit("npa_pm_qos: Failed to create NPA client '%s' "
			"for resource '%s'. (Error %ld)\n",
			client_info->request_name, client_info->resource_name,
			PTR_ERR(npa_client));
		BUG();
	}
	*(client_info->npa_client) = npa_client;

	/* Issue default resource requirement. */
	npa_issue_required_request(npa_client, client_info->value);

	kfree(client_info);
	return;
}

int npa_pm_qos_add(struct pm_qos_object *class, char *request_name,
			s32 value, void **request_data)
{
	struct npa_client_info *client_info;

	/* Non-default NPA requirements are not allowed at boot since
	 * requirements for unavailable resources won't be honoured. */
	BUG_ON(value != class->default_value
		&& system_state == SYSTEM_BOOTING);

	client_info = kzalloc(sizeof(struct npa_client_info), GFP_KERNEL);
	if (!client_info)
		return -ENOMEM;
	client_info->resource_name = (char *)class->plugin->data;
	client_info->request_name = request_name;
	client_info->value = value;
	client_info->npa_client = (struct npa_client **)request_data;

	/* Create NPA client when resource is available. */
	npa_resource_available(client_info->resource_name,
		npa_res_available_cb, client_info);

	return 0;
}

int npa_pm_qos_update(struct pm_qos_object *class, char *request_name,
				s32 value, void **request_data)
{
	struct npa_client *npa_client = (struct npa_client *)(*request_data);
	int rc = 0;

	if (!npa_client) {
		pr_err("%s: Error: No NPA client for resource '%s'.\n",
			__func__, (char *)class->plugin->data);
		return -ENXIO;
	}

	if (value == class->default_value)
		npa_complete_request(npa_client);
	else
		rc = npa_issue_required_request(npa_client, value);

	return rc;
}

int npa_pm_qos_remove(struct pm_qos_object *class, char *request_name,
				s32 value, void **request_data)
{
	struct npa_client *npa_client = (struct npa_client *)(*request_data);

	npa_cancel_request(npa_client);
	npa_destroy_client(npa_client);

	return 0;
}

