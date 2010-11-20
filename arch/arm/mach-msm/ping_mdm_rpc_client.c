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

/*
 * SMD RPC PING MODEM Driver
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <mach/msm_rpcrouter.h>

#define PING_TEST_BASE 0x31

#define PTIOC_NULL_TEST _IO(PING_TEST_BASE, 1)
#define PTIOC_REG_TEST _IO(PING_TEST_BASE, 2)
#define PTIOC_DATA_REG_TEST _IO(PING_TEST_BASE, 3)
#define PTIOC_DATA_CB_REG_TEST _IO(PING_TEST_BASE, 4)

#define PING_MDM_PROG  0x30000081
#define PING_MDM_VERS  0x00010001
#define PING_MDM_CB_PROG  0x31000081
#define PING_MDM_CB_VERS  0x00010001

#define PING_MDM_NULL_PROC                        0
#define PING_MDM_RPC_GLUE_CODE_INFO_REMOTE_PROC   1
#define PING_MDM_REGISTER_PROC                    2
#define PING_MDM_UNREGISTER_PROC                  3
#define PING_MDM_REGISTER_DATA_PROC               4
#define PING_MDM_UNREGISTER_DATA_CB_PROC          5
#define PING_MDM_REGISTER_DATA_CB_PROC            6

#define PING_MDM_DATA_CB_PROC            1
#define PING_MDM_CB_PROC                 2

static struct msm_rpc_client *rpc_client;
static uint32_t open_count;
static DEFINE_MUTEX(ping_mdm_lock);

struct ping_mdm_register_cb_arg {
	uint32_t cb_id;
	int val;
};

struct ping_mdm_register_data_cb_cb_arg {
	uint32_t cb_id;
	uint32_t *data;
	uint32_t size;
	uint32_t sum;
};

struct ping_mdm_register_data_cb_cb_ret {
	uint32_t result;
};

static int ping_mdm_register_cb(struct msm_rpc_client *client,
				 void *buffer, int in_size)
{
	int rc;
	uint32_t accept_status;
	struct rpc_request_hdr *req;
	struct ping_mdm_register_cb_arg arg, *buf_ptr;
	void *cb_func;

	req = (struct rpc_request_hdr *)buffer;
	buf_ptr = (struct ping_mdm_register_cb_arg *)(req + 1);

	arg.cb_id = be32_to_cpu(buf_ptr->cb_id);
	arg.val = be32_to_cpu(buf_ptr->val);

	cb_func = msm_rpc_get_cb_func(client, arg.cb_id);
	if (cb_func) {
		rc = ((int (*)(struct ping_mdm_register_cb_arg *, void *))
		      cb_func)(&arg, NULL);
		if (rc)
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		else
			accept_status = RPC_ACCEPTSTAT_SUCCESS;
	} else
		accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;

	msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
				     accept_status);
	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc)
		pr_err("%s: send accepted reply failed: %d\n", __func__, rc);

	return rc;
}

static int ping_mdm_data_cb(struct msm_rpc_client *client,
			     void *buffer, int in_size)
{
	struct rpc_request_hdr *req;
	int rc, i;
	void *buf, *cb_func, *reply;
	uint32_t size, accept_status;
	struct ping_mdm_register_data_cb_cb_arg arg;
	struct ping_mdm_register_data_cb_cb_ret ret;

	req = (struct rpc_request_hdr *)buffer;
	buf = (void *)(req + 1);

	arg.cb_id = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);

	size = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	if (size) {
		arg.data = kmalloc((size * sizeof(*arg.data)), GFP_KERNEL);
		if (arg.data)
			for (i = 0; i < size; i++)
				arg.data[i] =
				  be32_to_cpu(*((uint32_t *)buf + i));
	}
	buf += sizeof(uint32_t) * size;

	arg.size = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);

	arg.sum = be32_to_cpu(*(uint32_t *)buf);

	cb_func = msm_rpc_get_cb_func(client, arg.cb_id);
	if (cb_func) {
		rc = ((int (*)
		       (struct ping_mdm_register_data_cb_cb_arg *,
			struct ping_mdm_register_data_cb_cb_ret *))
		      cb_func)(&arg, &ret);
		if (rc)
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		else
			accept_status = RPC_ACCEPTSTAT_SUCCESS;
	} else
		accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;

	reply = msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
					     accept_status);

	size = 0;
	if (accept_status == RPC_ACCEPTSTAT_SUCCESS) {
		*(uint32_t *)reply = cpu_to_be32(ret.result);
		size = sizeof(uint32_t);
	}
	rc = msm_rpc_send_accepted_reply(client, size);
	if (rc)
		pr_err("%s: send accepted reply failed: %d\n", __func__, rc);

	return rc;
}

static int ping_mdm_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;

	req = (struct rpc_request_hdr *)buffer;

	switch (be32_to_cpu(req->procedure)) {
	case PING_MDM_CB_PROC:
		rc = ping_mdm_register_cb(client, buffer, in_size);
		break;
	case PING_MDM_DATA_CB_PROC:
		rc = ping_mdm_data_cb(client, buffer, in_size);
		break;
	default:
		pr_err("%s: procedure not supported %d\n", __func__,
		       be32_to_cpu(req->procedure));
		msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
					     RPC_ACCEPTSTAT_PROC_UNAVAIL);
		rc = msm_rpc_send_accepted_reply(client, 0);
		if (rc)
			pr_err("%s: sending reply failed: %d\n", __func__, rc);
		break;
	}
	return rc;
}

struct ping_mdm_unregister_data_cb_arg {
	int (*cb_func)(
		struct ping_mdm_register_data_cb_cb_arg *arg,
		struct ping_mdm_register_data_cb_cb_ret *ret);
};

struct ping_mdm_register_data_cb_arg {
	int (*cb_func)(
		struct ping_mdm_register_data_cb_cb_arg *arg,
		struct ping_mdm_register_data_cb_cb_ret *ret);
	uint32_t num;
	uint32_t size;
	uint32_t interval_ms;
	uint32_t num_tasks;
};

struct ping_mdm_register_data_cb_ret {
	uint32_t result;
};

struct ping_mdm_unregister_data_cb_ret {
	uint32_t result;
};

static int ping_mdm_data_cb_register_arg(struct msm_rpc_client *client,
					 void *buf, void *data)
{
	struct ping_mdm_register_data_cb_arg *arg;
	int cb_id, size = 0;

	arg = (struct ping_mdm_register_data_cb_arg *)data;

	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	*((uint32_t *)buf) = cpu_to_be32((uint32_t)cb_id);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(arg->num);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(arg->size);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(arg->interval_ms);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(arg->num_tasks);
	size += sizeof(uint32_t);

	return size;
}

static int ping_mdm_data_cb_unregister_arg(struct msm_rpc_client *client,
					   void *buf, void *data)
{
	struct ping_mdm_unregister_data_cb_arg *arg;
	int cb_id;

	arg = (struct ping_mdm_unregister_data_cb_arg *)data;

	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	*((uint32_t *)buf) = cpu_to_be32((uint32_t)cb_id);

	return sizeof(uint32_t);
}

static int ping_mdm_data_cb_register_ret(struct msm_rpc_client *client,
					 void *buf, void *data)
{
	struct ping_mdm_register_data_cb_ret *data_ptr, *buf_ptr;

	data_ptr = (struct ping_mdm_register_data_cb_ret *)data;
	buf_ptr = (struct ping_mdm_register_data_cb_ret *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int ping_mdm_register_data_cb(
	struct msm_rpc_client *client,
	struct ping_mdm_register_data_cb_arg *arg,
	struct ping_mdm_register_data_cb_ret *ret)
{
	return msm_rpc_client_req(client,
				  PING_MDM_REGISTER_DATA_CB_PROC,
				  ping_mdm_data_cb_register_arg, arg,
				  ping_mdm_data_cb_register_ret, ret, -1);
}

static int ping_mdm_unregister_data_cb(
	struct msm_rpc_client *client,
	struct ping_mdm_unregister_data_cb_arg *arg,
	struct ping_mdm_unregister_data_cb_ret *ret)
{
	return msm_rpc_client_req(client,
				  PING_MDM_UNREGISTER_DATA_CB_PROC,
				  ping_mdm_data_cb_unregister_arg, arg,
				  ping_mdm_data_cb_register_ret, ret, -1);
}

struct ping_mdm_data_arg {
	uint32_t *data;
	uint32_t size;
};

struct ping_mdm_data_ret {
	uint32_t result;
};

static int ping_mdm_data_register_arg(struct msm_rpc_client *client,
				      void *buf, void *data)
{
	int i;
	struct ping_mdm_data_arg *data_ptr;

	data_ptr = (struct ping_mdm_data_arg *)data;

	*((uint32_t *)buf) = cpu_to_be32(data_ptr->size);
	buf += sizeof(data_ptr->size);
	for (i = 0; i < data_ptr->size; i++) {
		*((uint32_t *)buf) = cpu_to_be32(data_ptr->data[i]);
		buf += sizeof(*data_ptr->data);
	}

	*((uint32_t *)buf) = cpu_to_be32(data_ptr->size);

	return (data_ptr->size * sizeof(uint32_t)) +
		(sizeof(data_ptr->size) * 2);
}

static int ping_mdm_data_register_ret(struct msm_rpc_client *client,
				      void *buf, void *data)
{
	struct ping_mdm_data_ret *data_ptr, *buf_ptr;

	data_ptr = (struct ping_mdm_data_ret *)data;
	buf_ptr = (struct ping_mdm_data_ret *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int ping_mdm_data_register(
	struct msm_rpc_client *client,
	struct ping_mdm_data_arg *arg,
	struct ping_mdm_data_ret *ret)
{
	return msm_rpc_client_req(client,
				  PING_MDM_REGISTER_DATA_PROC,
				  ping_mdm_data_register_arg, arg,
				  ping_mdm_data_register_ret, ret, -1);
}

struct ping_mdm_register_arg {
	int (*cb_func)(struct ping_mdm_register_cb_arg *, void *);
	int num;
};

struct ping_mdm_unregister_arg {
	int (*cb_func)(struct ping_mdm_register_cb_arg *, void *);
};

struct ping_mdm_register_ret {
	uint32_t result;
};

struct ping_mdm_unregister_ret {
	uint32_t result;
};

static int ping_mdm_register_arg(struct msm_rpc_client *client,
				 void *buf, void *data)
{
	struct ping_mdm_register_arg *arg;
	int cb_id, size = 0;

	arg = (struct ping_mdm_register_arg *)data;

	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	*((uint32_t *)buf) = cpu_to_be32((uint32_t)cb_id);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((int32_t *)buf) = cpu_to_be32(arg->num);
	size += sizeof(uint32_t);

	return size;
}

static int ping_mdm_unregister_arg(struct msm_rpc_client *client,
				   void *buf, void *data)
{
	struct ping_mdm_unregister_arg *arg;
	int cb_id;

	arg = (struct ping_mdm_unregister_arg *)data;

	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	*((uint32_t *)buf) = cpu_to_be32((uint32_t)cb_id);

	return sizeof(uint32_t);
}

static int ping_mdm_register_ret(struct msm_rpc_client *client,
				 void *buf, void *data)
{
	struct ping_mdm_register_ret *data_ptr, *buf_ptr;

	data_ptr = (struct ping_mdm_register_ret *)data;
	buf_ptr = (struct ping_mdm_register_ret *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int ping_mdm_register(
	struct msm_rpc_client *client,
	struct ping_mdm_register_arg *arg,
	struct ping_mdm_register_ret *ret)
{
	return msm_rpc_client_req(client,
				  PING_MDM_REGISTER_PROC,
				  ping_mdm_register_arg, arg,
				  ping_mdm_register_ret, ret, -1);
}

static int ping_mdm_unregister(
	struct msm_rpc_client *client,
	struct ping_mdm_unregister_arg *arg,
	struct ping_mdm_unregister_ret *ret)
{
	return msm_rpc_client_req(client,
				  PING_MDM_UNREGISTER_PROC,
				  ping_mdm_unregister_arg, arg,
				  ping_mdm_register_ret, ret, -1);
}

static int ping_mdm_null(struct msm_rpc_client *client,
			 void *arg, void *ret)
{
	return msm_rpc_client_req(client, PING_MDM_NULL_PROC,
				  NULL, NULL, NULL, NULL, -1);
}

static int ping_mdm_close(void)
{
	mutex_lock(&ping_mdm_lock);
	if (--open_count == 0) {
		msm_rpc_unregister_client(rpc_client);
		pr_info("%s: disconnected from remote ping server\n",
			__func__);
	}
	mutex_unlock(&ping_mdm_lock);
	return 0;
}

static struct msm_rpc_client *ping_mdm_init(void)
{
	mutex_lock(&ping_mdm_lock);
	if (open_count == 0) {
		rpc_client = msm_rpc_register_client("pingdef",
						     PING_MDM_PROG,
						     PING_MDM_VERS, 1,
						     ping_mdm_cb_func);
		if (!IS_ERR(rpc_client))
			open_count++;
	}
	mutex_unlock(&ping_mdm_lock);
	return rpc_client;
}

static struct dentry *dent;

static DEFINE_MUTEX(ping_mdm_cb_lock);
static LIST_HEAD(ping_mdm_cb_list);
static uint32_t test_res;

static int reg_cb_num, reg_cb_num_req;
static int data_cb_num, data_cb_num_req;
static int reg_done_flag, data_cb_done_flag;
static DECLARE_WAIT_QUEUE_HEAD(reg_test_wait);
static DECLARE_WAIT_QUEUE_HEAD(data_cb_test_wait);

static int ping_mdm_data_register_test(void)
{
	int i, rc = 0;
	uint32_t my_data[64];
	uint32_t my_sum = 0;
	struct ping_mdm_data_arg data_arg;
	struct ping_mdm_data_ret data_ret;

	for (i = 0; i < 64; i++) {
		my_data[i] = (42 + i);
		my_sum ^= (42 + i);
	}

	data_arg.data = my_data;
	data_arg.size = 64;

	rc = ping_mdm_data_register(rpc_client, &data_arg, &data_ret);
	if (rc)
		return rc;

	if (my_sum != data_ret.result) {
		pr_err("%s: sum mismatch %d %d\n",
		       __func__, my_sum, data_ret.result);
		rc = -1;
	}

	return rc;
}

static int ping_mdm_test_register_data_cb(
	struct ping_mdm_register_data_cb_cb_arg *arg,
	struct ping_mdm_register_data_cb_cb_ret *ret)
{
	uint32_t i, sum = 0;

	pr_info("%s: received cb_id %d, size = %d, sum = %u\n",
		__func__, arg->cb_id, arg->size, arg->sum);

	if (arg->data)
		for (i = 0; i < arg->size; i++)
			sum ^= arg->data[i];

	if (sum != arg->sum)
		pr_err("%s: sum mismatch %d %d\n", __func__, sum, arg->sum);

	data_cb_num++;
	if (data_cb_num == data_cb_num_req) {
		data_cb_done_flag = 1;
		wake_up(&data_cb_test_wait);
	}

	ret->result = 1;
	return 0;
}

static int ping_mdm_data_cb_register_test(void)
{
	int rc = 0;
	struct ping_mdm_register_data_cb_arg reg_arg;
	struct ping_mdm_unregister_data_cb_arg unreg_arg;
	struct ping_mdm_register_data_cb_ret reg_ret;
	struct ping_mdm_unregister_data_cb_ret unreg_ret;

	data_cb_num = 0;
	data_cb_num_req = 10;
	data_cb_done_flag = 0;

	reg_arg.cb_func = ping_mdm_test_register_data_cb;
	reg_arg.num = 10;
	reg_arg.size = 64;
	reg_arg.interval_ms = 10;
	reg_arg.num_tasks = 1;

	rc = ping_mdm_register_data_cb(rpc_client, &reg_arg, &reg_ret);
	if (rc)
		return rc;

	pr_info("%s: data_cb_register result: 0x%x\n",
		__func__, reg_ret.result);
	wait_event(data_cb_test_wait, data_cb_done_flag);

	unreg_arg.cb_func = reg_arg.cb_func;
	rc = ping_mdm_unregister_data_cb(rpc_client, &unreg_arg, &unreg_ret);
	if (rc)
		return rc;

	pr_info("%s: data_cb_unregister result: 0x%x\n",
		__func__, unreg_ret.result);

	return 0;
}

static int ping_mdm_test_register_cb(
	struct ping_mdm_register_cb_arg *arg, void *ret)
{
	pr_info("%s: received cb_id %d, val = %d\n",
		__func__, arg->cb_id, arg->val);

	reg_cb_num++;
	if (reg_cb_num == reg_cb_num_req) {
		reg_done_flag = 1;
		wake_up(&reg_test_wait);
	}
	return 0;
}

static int ping_mdm_register_test(void)
{
	int rc = 0;
	struct ping_mdm_register_arg reg_arg;
	struct ping_mdm_unregister_arg unreg_arg;
	struct ping_mdm_register_ret reg_ret;
	struct ping_mdm_unregister_ret unreg_ret;

	reg_cb_num = 0;
	reg_cb_num_req = 10;
	reg_done_flag = 0;

	reg_arg.num = 10;
	reg_arg.cb_func = ping_mdm_test_register_cb;

	rc = ping_mdm_register(rpc_client, &reg_arg, &reg_ret);
	if (rc)
		return rc;

	pr_info("%s: register result: 0x%x\n",
		__func__, reg_ret.result);

	wait_event(reg_test_wait, reg_done_flag);

	unreg_arg.cb_func = ping_mdm_test_register_cb;
	rc = ping_mdm_unregister(rpc_client, &unreg_arg, &unreg_ret);
	if (rc)
		return rc;

	pr_info("%s: unregister result: 0x%x\n",
		__func__, unreg_ret.result);

	return 0;
}

static int ping_mdm_null_test(void)
{
	return ping_mdm_null(rpc_client, NULL, NULL);
}

static int ping_test_release(struct inode *ip, struct file *fp)
{
	return ping_mdm_close();
}

static int ping_test_open(struct inode *ip, struct file *fp)
{
	struct msm_rpc_client *client;

	client = ping_mdm_init();
	if (IS_ERR(client)) {
		pr_err("%s: couldn't open ping client\n", __func__);
		return PTR_ERR(client);
	} else
		pr_info("%s: connected to remote ping server\n",
			__func__);

	return 0;
}

static ssize_t ping_test_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	char _buf[16];

	snprintf(_buf, sizeof(_buf), "%i\n", test_res);

	return simple_read_from_buffer(buf, count, pos, _buf, strlen(_buf));
}

static ssize_t ping_test_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	unsigned char cmd[64];
	int len;

	if (count < 1)
		return 0;

	len = count > 63 ? 63 : count;

	if (copy_from_user(cmd, buf, len))
		return -EFAULT;

	cmd[len] = 0;

	/* lazy */
	if (cmd[len-1] == '\n') {
		cmd[len-1] = 0;
		len--;
	}

	if (!strncmp(cmd, "null_test", 64))
		test_res = ping_mdm_null_test();
	else if (!strncmp(cmd, "reg_test", 64))
		test_res = ping_mdm_register_test();
	else if (!strncmp(cmd, "data_reg_test", 64))
		test_res = ping_mdm_data_register_test();
	else if (!strncmp(cmd, "data_cb_reg_test", 64))
		test_res = ping_mdm_data_cb_register_test();
	else
		test_res = -EINVAL;

	return count;
}

static const struct file_operations debug_ops = {
	.owner = THIS_MODULE,
	.open = ping_test_open,
	.read = ping_test_read,
	.write = ping_test_write,
	.release = ping_test_release,
};

static void __exit ping_test_exit(void)
{
	debugfs_remove(dent);
}

static int __init ping_test_init(void)
{
	dent = debugfs_create_file("ping_mdm", 0444, 0, NULL, &debug_ops);
	test_res = 0;
	open_count = 0;
	return 0;
}

module_init(ping_test_init);
module_exit(ping_test_exit);

MODULE_DESCRIPTION("PING TEST Driver");
MODULE_LICENSE("Dual BSD/GPL");
