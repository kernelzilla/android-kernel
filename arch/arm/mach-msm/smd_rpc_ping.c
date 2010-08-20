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
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <mach/msm_rpcrouter.h>

#define PING_TEST_BASE 0x31

#define PTIOC_NULL_TEST _IO(PING_TEST_BASE, 1)
#define PTIOC_REG_TEST _IO(PING_TEST_BASE, 2)
#define PTIOC_DATA_REG_TEST _IO(PING_TEST_BASE, 3)
#define PTIOC_DATA_CB_REG_TEST _IO(PING_TEST_BASE, 4)
#define PTIOC_USE_MULTI_CLIENTS _IO(PING_TEST_BASE, 5)
#define PTIOC_USE_DEFAULT_CLIENT _IO(PING_TEST_BASE, 6)

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
static uint32_t test_res;
static uint32_t open_count;
static uint32_t ping_test_use_multi_clients;
static DEFINE_MUTEX(ping_mdm_lock);
static DEFINE_MUTEX(ping_mdm_cb_lock);
static LIST_HEAD(ping_mdm_cb_list);
static atomic_t next_cb_id = ATOMIC_INIT(1);

struct ping_mdm_cb_item {
	struct list_head list;

	uint32_t cb_id;

	wait_queue_head_t wait;
	int done_flag;

	int num;
	int num_req;
};

struct ping_mdm_cb_item *ping_mdm_get_cb_item(uint32_t cb_id)
{
	struct ping_mdm_cb_item *cb_item;

	mutex_lock(&ping_mdm_cb_lock);
	list_for_each_entry(cb_item, &ping_mdm_cb_list, list) {
		if (cb_item->cb_id == cb_id) {
			mutex_unlock(&ping_mdm_cb_lock);
			return cb_item;
		}
	}
	mutex_unlock(&ping_mdm_cb_lock);
	return NULL;
}

static void ping_mdm_register_cb(struct msm_rpc_client *client,
				 void *buffer, int in_size)
{
	struct rpc_request_hdr *req;
	int rc, num;
	void *buf;
	uint32_t cb_id;
	struct ping_mdm_cb_item *cb_item;

	req = (struct rpc_request_hdr *)buffer;
	buf = (void *)(req + 1);

	cb_id = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	num = be32_to_cpu(*(int32_t *)buf);
	buf += sizeof(int32_t);
	pr_info("%s: received cb_id %d, val = %d\n", __func__, cb_id, num);

	rc = msm_rpc_send_accepted_reply(client, be32_to_cpu(req->xid),
					 RPC_ACCEPTSTAT_SUCCESS, NULL, 0);
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return;
	}

	cb_item = ping_mdm_get_cb_item(cb_id);
	if (!cb_item) {
		pr_err("%s: no cb item found\n", __func__);
		return;
	}
	cb_item->num++;

	if (cb_item->num == cb_item->num_req) {
		cb_item->done_flag = 1;
		wake_up(&cb_item->wait);
	}
}

static void ping_mdm_data_cb(struct msm_rpc_client *client,
			     void *buffer, int in_size)
{
	struct rpc_request_hdr *req;
	int rc, i;
	void *buf;
	uint32_t cb_id = 0;
	uint32_t size, *data, req_size, sum, my_sum, my_data;
	struct ping_mdm_cb_item *cb_item;

	req = (struct rpc_request_hdr *)buffer;
	buf = (void *)(req + 1);

	my_sum = 0;
	cb_id = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	size = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	data = (uint32_t *)buf;
	buf += sizeof(uint32_t) * size;
	for (i = 0; i < size; i++)
		my_sum = be32_to_cpu(*(data + i)) ^ my_sum;

	req_size = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	sum = be32_to_cpu(*(uint32_t *)buf);
	pr_info("%s: received cb_id %d, xid = %d, size = %d,"
		"req_size = %d, sum = %u, my_sum = %u\n", __func__,
		cb_id, be32_to_cpu(req->xid), size, req_size, sum, my_sum);

	if (sum != my_sum)
		pr_err("%s: sum mismatch\n", __func__);

	my_data = cpu_to_be32(1);
	rc = msm_rpc_send_accepted_reply(client, be32_to_cpu(req->xid),
					 RPC_ACCEPTSTAT_SUCCESS,
					 (char *)&my_data, sizeof(my_data));
	if (rc)
		pr_err("%s: sending reply failed: %d\n", __func__, rc);

	cb_item = ping_mdm_get_cb_item(cb_id);
	if (!cb_item) {
		pr_err("%s: no cb item found\n", __func__);
		return;
	}
	cb_item->num++;

	if (cb_item->num == cb_item->num_req) {
		cb_item->done_flag = 1;
		wake_up(&cb_item->wait);
	}
}

static int ping_mdm_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	struct rpc_request_hdr *req;

	req = (struct rpc_request_hdr *)buffer;

	switch (be32_to_cpu(req->procedure)) {
	case PING_MDM_CB_PROC:
		ping_mdm_register_cb(client, buffer, in_size);
		break;
	case PING_MDM_DATA_CB_PROC:
		ping_mdm_data_cb(client, buffer, in_size);
		break;
	default:
		pr_err("%s: procedure not supported %d\n", __func__,
		       be32_to_cpu(req->procedure));
		break;
	}
	return 0;
}

static struct msm_rpc_client *ping_mdm_get_client(char *name)
{
	struct msm_rpc_client *client;

	if (ping_test_use_multi_clients) {
		client = msm_rpc_register_client(name,
						 PING_MDM_PROG,
						 PING_MDM_VERS, 0,
						 ping_mdm_cb_func);
		if (IS_ERR(client))
			pr_err("%s: could not open rpc client:%ld\n",
			       __func__, PTR_ERR(client));
	} else
		client = rpc_client;

	return client;
}

static struct ping_mdm_cb_item *ping_mdm_create_cb_item(uint32_t cb_id,
							int num_req)
{
	struct ping_mdm_cb_item *cb_item;

	cb_item = kmalloc(sizeof(struct ping_mdm_cb_item), GFP_KERNEL);
	if (!cb_item)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&cb_item->list);
	cb_item->num_req = num_req;
	cb_item->num = 0;
	init_waitqueue_head(&cb_item->wait);
	cb_item->done_flag = 0;
	cb_item->cb_id = cb_id;

	mutex_lock(&ping_mdm_cb_lock);
	list_add_tail(&cb_item->list, &ping_mdm_cb_list);
	mutex_unlock(&ping_mdm_cb_lock);

	return cb_item;
}

static void ping_mdm_destroy_cb_item(struct ping_mdm_cb_item *cb_item)
{
	mutex_lock(&ping_mdm_cb_lock);
	list_del(&cb_item->list);
	mutex_unlock(&ping_mdm_cb_lock);

	kfree(cb_item);
}

static int ping_mdm_data_cb_register_arg(void *buf, void *data)
{
	int size = 0;

	*((uint32_t *)buf) = cpu_to_be32((uint32_t)data);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(10);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(64);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(10);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	*((uint32_t *)buf) = cpu_to_be32(1);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	return size;
}

static int ping_mdm_data_cb_register_arg_unreg(void *buf, void *data)
{
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)data);
	return sizeof(uint32_t);
}

static int ping_mdm_data_cb_register_result(void *buf, void *data)
{
	uint32_t result;

	result = *((uint32_t *)buf);
	pr_info("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}

static int ping_mdm_data_cb_register_test(void)
{
	int rc;
	struct ping_mdm_cb_item *cb_item;
	struct msm_rpc_client *client;

	client = ping_mdm_get_client("pingdatacb");
	if (IS_ERR(client))
		return PTR_ERR(client);

	cb_item = ping_mdm_create_cb_item(atomic_add_return(1, &next_cb_id),
					  10);
	if (IS_ERR(cb_item)) {
		rc = PTR_ERR(cb_item);
		goto release_client;
	}

	rc = msm_rpc_client_req(client,
				PING_MDM_REGISTER_DATA_CB_PROC,
				ping_mdm_data_cb_register_arg,
				ping_mdm_data_cb_register_result,
				(void *)(cb_item->cb_id));
	if (rc)
		goto free_and_release_client;

	wait_event(cb_item->wait, cb_item->done_flag);

	rc = msm_rpc_client_req(client,
				PING_MDM_UNREGISTER_DATA_CB_PROC,
				ping_mdm_data_cb_register_arg_unreg,
				ping_mdm_data_cb_register_result,
				(void *)(cb_item->cb_id));


 free_and_release_client:
	ping_mdm_destroy_cb_item(cb_item);
 release_client:
	if (client != rpc_client)
		msm_rpc_unregister_client(client);

	return rc;
}

static int ping_mdm_data_register_arg(void *buf, void *data)
{
	int i, size = 0;

	*((uint32_t *)buf) = cpu_to_be32(64);
	size += sizeof(uint32_t);
	buf += size;
	for (i = 0; i < 64; i++) {
		*((uint32_t *)buf) = cpu_to_be32(42 + i);
		size += sizeof(uint32_t);
		buf += sizeof(uint32_t);
		*(uint32_t *)data = (*(uint32_t *)data) ^ (42 | i);
	}
	*((uint32_t *)buf) = cpu_to_be32(64);
	size += sizeof(uint32_t);

	return size;
}

static int ping_mdm_data_register_result(void *buf, void *data)
{
	uint32_t result;

	result = *((uint32_t *)buf);
	pr_info("%s: request completed: 0x%x\n", __func__, result);

	if (result != *(uint32_t *)data)
		pr_err("%s: sum mismatch\n", __func__);

	return 0;
}

static int ping_mdm_data_register_test(void)
{
	int rc = 0;
	struct msm_rpc_client *client;
	uint32_t my_sum = 0;

	client = ping_mdm_get_client("pingdata");
	if (IS_ERR(client))
		return PTR_ERR(client);


	rc = msm_rpc_client_req(client,
				PING_MDM_REGISTER_DATA_PROC,
				ping_mdm_data_register_arg,
				ping_mdm_data_register_result, &my_sum);

	if (client != rpc_client)
		msm_rpc_unregister_client(client);

	return rc;
}

static int ping_mdm_register_arg(void *buf, void *data)
{
	int num, size = 0;

	/* revist to pass in num also through data */
	num = 10;
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)data);
	size += sizeof(uint32_t);
	buf += size;

	*((int *)buf) = cpu_to_be32(num);
	size += sizeof(int);

	return size;
}

static int ping_mdm_register_arg_unreg(void *buf, void *data)
{
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)data);
	return sizeof(uint32_t);
}

static int ping_mdm_register_result(void *buf, void *data)
{
	uint32_t result;

	result = *((uint32_t *)buf);
	pr_info("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}

static int ping_mdm_register_test(void)
{
	int rc;
	struct ping_mdm_cb_item *cb_item;
	struct msm_rpc_client *client;

	client = ping_mdm_get_client("pingreg");
	if (IS_ERR(client))
		return PTR_ERR(client);

	cb_item = ping_mdm_create_cb_item(atomic_add_return(1, &next_cb_id),
					  10);
	if (IS_ERR(cb_item)) {
		rc = PTR_ERR(cb_item);
		goto release_client;
	}

	rc = msm_rpc_client_req(client,
				PING_MDM_REGISTER_PROC,
				ping_mdm_register_arg,
				ping_mdm_register_result,
				(void *)(cb_item->cb_id));
	if (rc)
		goto free_and_release_client;

	wait_event(cb_item->wait, cb_item->done_flag);

	rc = msm_rpc_client_req(client,
				PING_MDM_UNREGISTER_PROC,
				ping_mdm_register_arg_unreg,
				ping_mdm_register_result,
				(void *)(cb_item->cb_id));

 free_and_release_client:
	ping_mdm_destroy_cb_item(cb_item);
 release_client:
	if (client != rpc_client)
		msm_rpc_unregister_client(client);

	return rc;
}

static int ping_mdm_null_test(void)
{
	int rc;
	struct msm_rpc_client *client;

	client = ping_mdm_get_client("pingnull");
	if (IS_ERR(client))
		return PTR_ERR(client);

	rc = msm_rpc_client_req(client, PING_MDM_NULL_PROC, NULL, NULL, NULL);

	if (client != rpc_client)
		msm_rpc_unregister_client(client);

	return rc;
}

static int ping_test_release(struct inode *ip, struct file *fp)
{
	mutex_lock(&ping_mdm_lock);
	if (--open_count == 0) {
		msm_rpc_unregister_client(rpc_client);
		pr_info("%s: disconnected from remote ping server\n",
			__func__);
		kfree(fp->private_data);
	}
	mutex_unlock(&ping_mdm_lock);
	return 0;
}

static int ping_test_open(struct inode *ip, struct file *fp)
{
	mutex_lock(&ping_mdm_lock);
	if (open_count++ == 0) {
		rpc_client = msm_rpc_register_client("pingdef",
						     PING_MDM_PROG,
						     PING_MDM_VERS, 1,
						     ping_mdm_cb_func);
		if (IS_ERR(rpc_client)) {
			pr_err("%s: couldn't open rpc client\n", __func__);
			return PTR_ERR(rpc_client);
		}
		pr_info("%s: connected to remote ping server\n", __func__);
	}
	mutex_unlock(&ping_mdm_lock);
	return 0;
}

static int ping_test_ioctl(struct inode *ip, struct file *fp,
			   unsigned int cmd, unsigned long arg)
{
	int rc;

	switch (cmd) {
	default:
		return -ENOTTY;

	case PTIOC_NULL_TEST:
		rc = ping_mdm_null_test();
		break;
	case PTIOC_REG_TEST:
		rc = ping_mdm_register_test();
		break;
	case PTIOC_DATA_REG_TEST:
		rc = ping_mdm_data_register_test();
		break;
	case PTIOC_DATA_CB_REG_TEST:
		rc = ping_mdm_data_cb_register_test();
		break;
	case PTIOC_USE_MULTI_CLIENTS:
		ping_test_use_multi_clients = 1;
		pr_info("%s: tests to use multiple clients\n", __func__);
		return 0;
	case PTIOC_USE_DEFAULT_CLIENT:
		ping_test_use_multi_clients = 0;
		pr_info("%s: tests to use default client\n", __func__);
		return 0;
	}

	if (rc)
		pr_info("%s: ping mdm test FAILed: %d\n", __func__, rc);
	else
		pr_info("%s: ping mdm test PASSed\n", __func__);

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

	if (!strncmp(cmd, "null_test", 64)) {
		test_res = ping_mdm_null_test();
	} else if (!strncmp(cmd, "reg_test", 64)) {
		test_res = ping_mdm_register_test();
	} else if (!strncmp(cmd, "data_reg_test", 64)) {
		test_res = ping_mdm_data_register_test();
	} else if (!strncmp(cmd, "data_cb_reg_test", 64)) {
		test_res = ping_mdm_data_cb_register_test();
	} else if (!strncmp(cmd, "use_multi_clients", 64)) {
		ping_test_use_multi_clients = 1;
		pr_info("%s: tests to use multiple clients\n", __func__);
	} else if (!strncmp(cmd, "use_default_client", 64)) {
		ping_test_use_multi_clients = 0;
		pr_info("%s: tests to use default client\n", __func__);
	}

	return count;
}

static const struct file_operations ping_test_fops = {
	.owner = THIS_MODULE,
	.open = ping_test_open,
	.read = ping_test_read,
	.write = ping_test_write,
	.release = ping_test_release,
	.ioctl = ping_test_ioctl,
};

static struct miscdevice ping_test_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ping_test",
	.fops = &ping_test_fops,
};

static void __exit ping_test_exit(void)
{
	misc_deregister(&ping_test_device);
}

static int __init ping_test_init(void)
{
	int rc = 0;

	test_res = 0;
	open_count = 0;
	ping_test_use_multi_clients = 0;
	rc = misc_register(&ping_test_device);
	if (rc)
		pr_err("%s: ping_test_device reg fail\n", __func__);

	return rc;
}


module_init(ping_test_init);
module_exit(ping_test_exit);

MODULE_DESCRIPTION("PING TEST Driver");
MODULE_LICENSE("GPL v2");
