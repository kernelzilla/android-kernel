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
 * OEM RAPI CLIENT Driver source file
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <mach/msm_rpcrouter.h>
#include <mach/oem_rapi_client.h>

#define OEM_RAPI_PROG  0x3000006B
#define OEM_RAPI_VERS  0x00010001

#define OEM_RAPI_NULL_PROC                        0
#define OEM_RAPI_RPC_GLUE_CODE_INFO_REMOTE_PROC   1
#define OEM_RAPI_STREAMING_FUNCTION_PROC          2

#define OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE 128

static struct msm_rpc_client *rpc_client;
static uint32_t open_count;
static DEFINE_MUTEX(oem_rapi_client_lock);

static int oem_rapi_client_cb(struct msm_rpc_client *client,
			      void *buffer, int in_size)
{
	struct rpc_request_hdr *req;
	void *buf, *cb_func, *reply;
	uint32_t cb_id, accept_status, size;
	int rc;

	struct oem_rapi_client_streaming_func_cb_arg arg;
	struct oem_rapi_client_streaming_func_cb_ret ret;

	arg.input = NULL;
	ret.out_len = NULL;
	ret.output = NULL;

	req = (struct rpc_request_hdr *)buffer;
	buf = (void *)(req + 1);

	/* cb_id */
	cb_id = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);

	/* enum */
	arg.event = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);

	/* handle */
	arg.handle = (void *)be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);

	/* in_len */
	arg.in_len = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);

	/* input */
	size = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	if (size) {
		arg.input = kmalloc(size, GFP_KERNEL);
		if (arg.input)
			memcpy(arg.input, buf, size);
		else {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}
	buf += size;
	if (size & 0x3)
		buf += 4 - (size & 0x3);

	/* out_len */
	arg.out_len_valid = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	if (arg.out_len_valid) {
		ret.out_len = kmalloc(sizeof(*ret.out_len), GFP_KERNEL);
		if (!ret.out_len) {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}

	/* out */
	arg.output_valid = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	if (arg.output_valid) {
		arg.output_size = be32_to_cpu(*(uint32_t *)buf);
		buf += sizeof(uint32_t);
		ret.output = kmalloc(arg.output_size, GFP_KERNEL);
		if (!ret.output) {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}

	cb_func = msm_rpc_get_cb_func(client, cb_id);
	if (cb_func) {
		rc = ((int (*)(struct oem_rapi_client_streaming_func_cb_arg *,
			       struct oem_rapi_client_streaming_func_cb_ret *))
		      cb_func)(&arg, &ret);
		if (rc)
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		else
			accept_status = RPC_ACCEPTSTAT_SUCCESS;
	} else
		accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;

 oem_rapi_send_ack:
	reply = msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
					     accept_status);

	size = 0;
	if (accept_status == RPC_ACCEPTSTAT_SUCCESS) {
		*(uint32_t *)reply = cpu_to_be32((uint32_t)(ret.out_len != 0));
		reply += sizeof(uint32_t);
		size += sizeof(uint32_t);

		if (ret.out_len) {
			*(uint32_t *)reply = cpu_to_be32(*ret.out_len);
			reply += sizeof(uint32_t);
			size += sizeof(uint32_t);
		}

		if (ret.output && ret.out_len) {
			*(uint32_t *)reply =
				cpu_to_be32((uint32_t)(*ret.out_len));
			reply += sizeof(uint32_t);
			size += sizeof(uint32_t);

			memcpy(reply, ret.output, *ret.out_len);
			reply += *ret.out_len;
			size += *ret.out_len;
			if (*ret.out_len & 0x3) {
				memset(reply, 0, 4 - (*ret.out_len & 0x3));
				reply += 4 - (*ret.out_len & 0x3);
				size += 4 - (*ret.out_len & 0x3);
			}
		} else {
			*(uint32_t *)reply = cpu_to_be32(0);
			reply += sizeof(uint32_t);
			size += sizeof(uint32_t);
		}
	}
	rc = msm_rpc_send_accepted_reply(client, size);
	if (rc)
		pr_err("%s: sending reply failed: %d\n", __func__, rc);

	kfree(arg.input);
	kfree(ret.out_len);
	kfree(ret.output);

	return 0;
}

static int oem_rapi_client_streaming_function_arg(struct msm_rpc_client *client,
						  void *buf, void *data)
{
	int size = 0;
	int cb_id;
	struct oem_rapi_client_streaming_func_arg *arg = data;

	/* enum */
	*((uint32_t *)buf) = cpu_to_be32(arg->event);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	/* cb_id */
	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)cb_id);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	/* handle */
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)arg->handle);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	/* in_len */
	*((uint32_t *)buf) = cpu_to_be32(arg->in_len);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	/* input */
	*((uint32_t *)buf) = cpu_to_be32(arg->in_len);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);
	memcpy(buf, arg->input, arg->in_len);
	size += arg->in_len;
	buf += arg->in_len;
	if (arg->in_len & 0x3) {
		memset(buf, 0, 4 - (arg->in_len & 0x3));
		buf += 4 - (arg->in_len & 0x3);
		size += 4 - (arg->in_len & 0x3);
	}

	/* out_len */
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)(arg->out_len_valid));
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	/* output */
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)(arg->output_valid));
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);
	if (arg->output_valid) {
		*((uint32_t *)buf) = cpu_to_be32(arg->output_size);
		size += sizeof(uint32_t);
		buf += sizeof(uint32_t);
	}

	return size;
}

static int oem_rapi_client_streaming_function_ret(struct msm_rpc_client *client,
						  void *buf, void *data)
{
	uint32_t data_present, size;
	struct oem_rapi_client_streaming_func_ret *ret = data;

	/* out_len */
	data_present = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	if (data_present && ret->out_len) {
		*ret->out_len = be32_to_cpu(*(uint32_t *)buf);
		buf += sizeof(uint32_t);
	}

	/* output */
	size = be32_to_cpu(*(uint32_t *)buf);
	buf += sizeof(uint32_t);
	if (size && ret->output)
		memcpy(ret->output, buf, size);
	buf += size;
	if (size & 0x3)
		buf += 4 - (size & 0x3);

	return 0;
}

int oem_rapi_client_streaming_function(
	struct msm_rpc_client *client,
	struct oem_rapi_client_streaming_func_arg *arg,
	struct oem_rapi_client_streaming_func_ret *ret)
{
	return msm_rpc_client_req(client,
				  OEM_RAPI_STREAMING_FUNCTION_PROC,
				  oem_rapi_client_streaming_function_arg, arg,
				  oem_rapi_client_streaming_function_ret,
				  ret, -1);
}
EXPORT_SYMBOL(oem_rapi_client_streaming_function);

int oem_rapi_client_close(void)
{
	mutex_lock(&oem_rapi_client_lock);
	if (--open_count == 0) {
		msm_rpc_unregister_client(rpc_client);
		pr_info("%s: disconnected from remote oem rapi server\n",
			__func__);
	}
	mutex_unlock(&oem_rapi_client_lock);
	return 0;
}
EXPORT_SYMBOL(oem_rapi_client_close);

struct msm_rpc_client *oem_rapi_client_init(void)
{
	mutex_lock(&oem_rapi_client_lock);
	if (open_count == 0) {
		rpc_client = msm_rpc_register_client("oemrapiclient",
						     OEM_RAPI_PROG,
						     OEM_RAPI_VERS, 0,
						     oem_rapi_client_cb);
		if (!IS_ERR(rpc_client))
			open_count++;
	}
	mutex_unlock(&oem_rapi_client_lock);
	return rpc_client;
}
EXPORT_SYMBOL(oem_rapi_client_init);

#if defined(CONFIG_DEBUG_FS)

static struct dentry *dent;
static int oem_rapi_client_test_res;

static int oem_rapi_client_null(struct msm_rpc_client *client,
				void *arg, void *ret)
{
	return msm_rpc_client_req(client, OEM_RAPI_NULL_PROC,
				  NULL, NULL, NULL, NULL, -1);
}

static int oem_rapi_client_test_streaming_cb_func(
	struct oem_rapi_client_streaming_func_cb_arg *arg,
	struct oem_rapi_client_streaming_func_cb_ret *ret)
{
	uint32_t size;
	pr_info("oem rapi client test cb func\n");

	size = (arg->in_len < OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE) ?
		arg->in_len : OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE;

	if (ret->out_len != 0)
		*ret->out_len = size;

	if (ret->output != 0)
		memcpy(ret->output, arg->input, size);

	return 0;
}

static ssize_t debug_read(struct file *fp, char __user *buf,
			  size_t count, loff_t *pos)
{
	char _buf[16];

	snprintf(_buf, sizeof(_buf), "%i\n", oem_rapi_client_test_res);

	return simple_read_from_buffer(buf, count, pos, _buf, strlen(_buf));
}

static ssize_t debug_write(struct file *fp, const char __user *buf,
			   size_t count, loff_t *pos)
{
	char input[OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];
	char output[OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];
	uint32_t out_len;
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;

	unsigned char cmd[64];
	int len;

	if (count < 1)
		return 0;

	len = count > 63 ? 63 : count;

	if (copy_from_user(cmd, buf, len))
		return -EFAULT;

	cmd[len] = 0;

	if (cmd[len-1] == '\n') {
		cmd[len-1] = 0;
		len--;
	}

	if (!strncmp(cmd, "null", 64)) {
		oem_rapi_client_test_res = oem_rapi_client_null(rpc_client,
								NULL, NULL);
	} else if (!strncmp(cmd, "streaming_func", 64)) {
		memset(input, 5, 16);
		arg.event = 0;
		arg.cb_func = oem_rapi_client_test_streaming_cb_func;
		arg.handle = (void *)20;
		arg.in_len = 16;
		arg.input = input;
		arg.out_len_valid = 1;
		arg.output_valid = 1;
		arg.output_size = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE;

		ret.out_len = &out_len;
		ret.output = output;
		oem_rapi_client_test_res = oem_rapi_client_streaming_function(
			rpc_client, &arg, &ret);
	} else
		oem_rapi_client_test_res = -EINVAL;

	if (oem_rapi_client_test_res)
		pr_err("oem rapi client test fail %d\n",
		       oem_rapi_client_test_res);
	else
		pr_info("oem rapi client test passed\n");

	return count;
}

static int debug_release(struct inode *ip, struct file *fp)
{
	return oem_rapi_client_close();
}

static int debug_open(struct inode *ip, struct file *fp)
{
	struct msm_rpc_client *client;
	client = oem_rapi_client_init();
	if (IS_ERR(client)) {
		pr_err("%s: couldn't open oem rapi client\n", __func__);
		return PTR_ERR(client);
	} else
		pr_info("%s: connected to remote oem rapi server\n", __func__);

	return 0;
}

static const struct file_operations debug_ops = {
	.owner = THIS_MODULE,
	.open = debug_open,
	.release = debug_release,
	.read = debug_read,
	.write = debug_write,
};

static void __exit oem_rapi_client_mod_exit(void)
{
	debugfs_remove(dent);
}

static int __init oem_rapi_client_mod_init(void)
{
	dent = debugfs_create_file("oem_rapi", 0444, 0, NULL, &debug_ops);
	open_count = 0;
	oem_rapi_client_test_res = -1;
	return 0;
}

module_init(oem_rapi_client_mod_init);
module_exit(oem_rapi_client_mod_exit);

#endif

MODULE_DESCRIPTION("OEM RAPI CLIENT Driver");
MODULE_LICENSE("Dual BSD/GPL");
