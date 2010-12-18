/* arch/arm/mach-msm/smd_rpcrouter.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

/* TODO: handle cases where smd_write() will tempfail due to full fifo */
/* TODO: thread priority? schedule a work to bump it? */
/* TODO: maybe make server_list_lock a mutex */
/* TODO: pool fragments to avoid kmalloc/kfree churn */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <asm/byteorder.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <asm/byteorder.h>

#include <mach/msm_smd.h>
#include <mach/smem_log.h>
#include "smd_rpcrouter.h"
#include "modem_notifier.h"

enum {
	SMEM_LOG = 1U << 0,
	RTR_DBG = 1U << 1,
	R2R_MSG = 1U << 2,
	R2R_RAW = 1U << 3,
	RPC_MSG = 1U << 4,
	NTFY_MSG = 1U << 5,
	RAW_PMR = 1U << 6,
	RAW_PMW = 1U << 7,
	R2R_RAW_HDR = 1U << 8,
};
static int smd_rpcrouter_debug_mask;
module_param_named(debug_mask, smd_rpcrouter_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DIAG(x...) printk(KERN_ERR "[RR] ERROR " x)

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
#define D(x...) do { \
if (smd_rpcrouter_debug_mask & RTR_DBG) \
	printk(KERN_ERR x); \
} while (0)

#define RR(x...) do { \
if (smd_rpcrouter_debug_mask & R2R_MSG) \
	printk(KERN_ERR "[RR] "x); \
} while (0)

#define RAW(x...) do { \
if (smd_rpcrouter_debug_mask & R2R_RAW) \
	printk(KERN_ERR "[RAW] "x); \
} while (0)

#define RAW_HDR(x...) do { \
if (smd_rpcrouter_debug_mask & R2R_RAW_HDR) \
	printk(KERN_ERR "[HDR] "x); \
} while (0)

#define RAW_PMR(x...) do { \
if (smd_rpcrouter_debug_mask & RAW_PMR) \
	printk(KERN_ERR "[PMR] "x); \
} while (0)

#define RAW_PMR_NOMASK(x...) do { \
	printk(KERN_ERR "[PMR] "x); \
} while (0)

#define RAW_PMW(x...) do { \
if (smd_rpcrouter_debug_mask & RAW_PMW) \
	printk(KERN_ERR "[PMW] "x); \
} while (0)

#define RAW_PMW_NOMASK(x...) do { \
	printk(KERN_ERR "[PMW] "x); \
} while (0)

#define IO(x...) do { \
if (smd_rpcrouter_debug_mask & RPC_MSG) \
	printk(KERN_ERR "[RPC] "x); \
} while (0)

#define NTFY(x...) do { \
if (smd_rpcrouter_debug_mask & NTFY_MSG) \
	printk(KERN_ERR "[NOTIFY] "x); \
} while (0)
#else
#define D(x...) do { } while (0)
#define RR(x...) do { } while (0)
#define RAW(x...) do { } while (0)
#define RAW_HDR(x...) do { } while (0)
#define RAW_PMR(x...) do { } while (0)
#define RAW_PMR_NO_MASK(x...) do { } while (0)
#define RAW_PMW(x...) do { } while (0)
#define RAW_PMW_NO_MASK(x...) do { } while (0)
#define IO(x...) do { } while (0)
#define NTFY(x...) do { } while (0)
#endif


static LIST_HEAD(local_endpoints);
static LIST_HEAD(remote_endpoints);

static LIST_HEAD(server_list);

static smd_channel_t *smd_channel;
static int initialized;
static wait_queue_head_t newserver_wait;
static wait_queue_head_t smd_wait;

static DEFINE_SPINLOCK(local_endpoints_lock);
static DEFINE_SPINLOCK(remote_endpoints_lock);
static DEFINE_SPINLOCK(server_list_lock);
static DEFINE_SPINLOCK(smd_lock);

static struct workqueue_struct *rpcrouter_workqueue;
static struct wake_lock rpcrouter_wake_lock;
static int rpcrouter_need_len;

static atomic_t next_xid = ATOMIC_INIT(1);
static atomic_t pm_mid = ATOMIC_INIT(1);

static void do_read_data(struct work_struct *work);
static void do_create_pdevs(struct work_struct *work);
static void do_create_rpcrouter_pdev(struct work_struct *work);

static DECLARE_WORK(work_read_data, do_read_data);
static DECLARE_WORK(work_create_pdevs, do_create_pdevs);
static DECLARE_WORK(work_create_rpcrouter_pdev, do_create_rpcrouter_pdev);
static atomic_t rpcrouter_pdev_created = ATOMIC_INIT(0);

#define RR_STATE_IDLE    0
#define RR_STATE_HEADER  1
#define RR_STATE_BODY    2
#define RR_STATE_ERROR   3

/* After restart notification, local ep keep
 * state for server restart and for ep notify.
 * Server restart cleared by R-R new svr msg.
 * NTFY cleared by calling msm_rpc_clear_netreset
*/

#define RESTART_NORMAL 0
#define RESTART_PEND_SVR 1
#define RESTART_PEND_NTFY 2
#define RESTART_PEND_NTFY_SVR 3

/* State for remote ep following restart */
#define RESTART_QUOTA_ABORT  1

struct rr_context {
	struct rr_packet *pkt;
	uint8_t *ptr;
	uint32_t state; /* current assembly state */
	uint32_t count; /* bytes needed in this state */
};

struct rr_context the_rr_context;

static struct platform_device rpcrouter_pdev = {
	.name		= "oncrpc_router",
	.id		= -1,
};


static int rpcrouter_send_control_msg(union rr_control_msg *msg)
{
	struct rr_header hdr;
	unsigned long flags;
	int need;

	if (!(msg->cmd == RPCROUTER_CTRL_CMD_HELLO) && !initialized) {
		printk(KERN_ERR "rpcrouter_send_control_msg(): Warning, "
		       "router not initialized\n");
		return -EINVAL;
	}

	hdr.version = RPCROUTER_VERSION;
	hdr.type = msg->cmd;
	hdr.src_pid = RPCROUTER_PID_LOCAL;
	hdr.src_cid = RPCROUTER_ROUTER_ADDRESS;
	hdr.confirm_rx = 0;
	hdr.size = sizeof(*msg);
	hdr.dst_pid = 0;
	hdr.dst_cid = RPCROUTER_ROUTER_ADDRESS;

	/* TODO: what if channel is full? */

	need = sizeof(hdr) + hdr.size;
	spin_lock_irqsave(&smd_lock, flags);
	while (smd_write_avail(smd_channel) < need) {
		spin_unlock_irqrestore(&smd_lock, flags);
		msleep(250);
		spin_lock_irqsave(&smd_lock, flags);
	}
	smd_write(smd_channel, &hdr, sizeof(hdr));
	smd_write(smd_channel, msg, hdr.size);
	spin_unlock_irqrestore(&smd_lock, flags);
	return 0;
}

static void modem_reset_start_cleanup(void)
{
	struct msm_rpc_endpoint *ept;
	struct rr_remote_endpoint *r_ept;
	struct rr_packet *pkt, *tmp_pkt;
	struct rr_fragment *frag, *next;
	struct msm_rpc_reply *reply, *reply_tmp;
	unsigned long flags;

	spin_lock_irqsave(&local_endpoints_lock, flags);
	/* remove all partial packets received */
	list_for_each_entry(ept, &local_endpoints, list) {
		RR("modem_reset_start_clenup PID %x, remotepid:%d  \n",
		   ept->dst_pid, RPCROUTER_PID_REMOTE);
		/* remove replies */
		spin_lock(&ept->reply_q_lock);
		list_for_each_entry_safe(reply, reply_tmp,
					 &ept->reply_pend_q, list) {
			list_del(&reply->list);
			kfree(reply);
		}
		list_for_each_entry_safe(reply, reply_tmp,
					 &ept->reply_avail_q, list) {
			list_del(&reply->list);
			kfree(reply);
		}
		spin_unlock(&ept->reply_q_lock);
		if (ept->dst_pid == RPCROUTER_PID_REMOTE) {
			spin_lock(&ept->incomplete_lock);
			list_for_each_entry_safe(pkt, tmp_pkt,
						 &ept->incomplete, list) {
				list_del(&pkt->list);
				frag = pkt->first;
				while (frag != NULL) {
					next = frag->next;
					kfree(frag);
					frag = next;
				}
			kfree(pkt);
			}
			spin_unlock(&ept->incomplete_lock);
			/* remove all completed packets waiting to be read*/
			spin_lock(&ept->read_q_lock);
			list_for_each_entry_safe(pkt, tmp_pkt, &ept->read_q,
						 list) {
				list_del(&pkt->list);
				frag = pkt->first;
				while (frag != NULL) {
					next = frag->next;
					kfree(frag);
					frag = next;
				}
				kfree(pkt);
			}
			spin_unlock(&ept->read_q_lock);
			/* Set restart state for local ep */
			RR("EPT:0x%p, State %d  RESTART_PEND_NTFY_SVR "
			   "PROG:0x%08x VERS:0x%08x \n",
			   ept, ept->restart_state, be32_to_cpu(ept->dst_prog),
			   be32_to_cpu(ept->dst_vers));
			spin_lock(&ept->restart_lock);
			ept->restart_state = RESTART_PEND_NTFY_SVR;
			spin_unlock(&ept->restart_lock);
			wake_up(&ept->wait_q);
		}
	}

	spin_unlock_irqrestore(&local_endpoints_lock, flags);

    /* Unblock endpoints waiting for quota ack*/
	spin_lock_irqsave(&remote_endpoints_lock, flags);
	list_for_each_entry(r_ept, &remote_endpoints, list) {
		spin_lock(&r_ept->quota_lock);
		r_ept->quota_restart_state = RESTART_QUOTA_ABORT;
		RR("Set STATE_PENDING PID:0x%08x CID:0x%08x \n", r_ept->pid,
		   r_ept->cid);
		spin_unlock(&r_ept->quota_lock);
		wake_up(&r_ept->quota_wait);
	}
	spin_unlock_irqrestore(&remote_endpoints_lock, flags);

}


static struct rr_server *rpcrouter_create_server(uint32_t pid,
							uint32_t cid,
							uint32_t prog,
							uint32_t ver)
{
	struct rr_server *server;
	unsigned long flags;
	int rc;

	server = kmalloc(sizeof(struct rr_server), GFP_KERNEL);
	if (!server)
		return ERR_PTR(-ENOMEM);

	memset(server, 0, sizeof(struct rr_server));
	server->pid = pid;
	server->cid = cid;
	server->prog = prog;
	server->vers = ver;

	spin_lock_irqsave(&server_list_lock, flags);
	list_add_tail(&server->list, &server_list);
	spin_unlock_irqrestore(&server_list_lock, flags);

	if (pid == RPCROUTER_PID_REMOTE) {
		rc = msm_rpcrouter_create_server_cdev(server);
		if (rc < 0)
			goto out_fail;
	}
	return server;
out_fail:
	spin_lock_irqsave(&server_list_lock, flags);
	list_del(&server->list);
	spin_unlock_irqrestore(&server_list_lock, flags);
	kfree(server);
	return ERR_PTR(rc);
}

static void rpcrouter_destroy_server(struct rr_server *server)
{
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_del(&server->list);
	spin_unlock_irqrestore(&server_list_lock, flags);
	device_destroy(msm_rpcrouter_class, server->device_number);
	kfree(server);
}

static struct rr_server *rpcrouter_lookup_server(uint32_t prog, uint32_t ver)
{
	struct rr_server *server;
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->prog == prog
		 && server->vers == ver) {
			spin_unlock_irqrestore(&server_list_lock, flags);
			return server;
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
	return NULL;
}

static struct rr_server *rpcrouter_lookup_server_by_dev(dev_t dev)
{
	struct rr_server *server;
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->device_number == dev) {
			spin_unlock_irqrestore(&server_list_lock, flags);
			return server;
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
	return NULL;
}

struct msm_rpc_endpoint *msm_rpcrouter_create_local_endpoint(dev_t dev)
{
	struct msm_rpc_endpoint *ept;
	unsigned long flags;

	ept = kmalloc(sizeof(struct msm_rpc_endpoint), GFP_KERNEL);
	if (!ept)
		return NULL;
	memset(ept, 0, sizeof(struct msm_rpc_endpoint));
	ept->cid = (uint32_t) ept;
	ept->pid = RPCROUTER_PID_LOCAL;
	ept->dev = dev;

	if ((dev != msm_rpcrouter_devno) && (dev != MKDEV(0, 0))) {
		struct rr_server *srv;
		/*
		 * This is a userspace client which opened
		 * a program/ver devicenode. Bind the client
		 * to that destination
		 */
		srv = rpcrouter_lookup_server_by_dev(dev);
		/* TODO: bug? really? */
		BUG_ON(!srv);

		ept->dst_pid = srv->pid;
		ept->dst_cid = srv->cid;
		ept->dst_prog = cpu_to_be32(srv->prog);
		ept->dst_vers = cpu_to_be32(srv->vers);
	} else {
		/* mark not connected */
		ept->dst_pid = 0xffffffff;
	}

	init_waitqueue_head(&ept->wait_q);
	INIT_LIST_HEAD(&ept->read_q);
	spin_lock_init(&ept->read_q_lock);
	INIT_LIST_HEAD(&ept->reply_avail_q);
	INIT_LIST_HEAD(&ept->reply_pend_q);
	spin_lock_init(&ept->reply_q_lock);
	spin_lock_init(&ept->restart_lock);
	init_waitqueue_head(&ept->restart_wait);
	ept->restart_state = RESTART_NORMAL;
	wake_lock_init(&ept->read_q_wake_lock, WAKE_LOCK_SUSPEND, "rpc_read");
	INIT_LIST_HEAD(&ept->incomplete);
	spin_lock_init(&ept->incomplete_lock);

	spin_lock_irqsave(&local_endpoints_lock, flags);
	list_add_tail(&ept->list, &local_endpoints);
	spin_unlock_irqrestore(&local_endpoints_lock, flags);
	return ept;
}

int msm_rpcrouter_destroy_local_endpoint(struct msm_rpc_endpoint *ept)
{
	int rc;
	union rr_control_msg msg;
	struct msm_rpc_reply *reply, *reply_tmp;
	unsigned long flags;

	msg.cmd = RPCROUTER_CTRL_CMD_REMOVE_CLIENT;
	msg.cli.pid = ept->pid;
	msg.cli.cid = ept->cid;

	RR("x REMOVE_CLIENT id=%d:%08x\n", ept->pid, ept->cid);
	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		return rc;

	/* Free replies */
	spin_lock_irqsave(&ept->reply_q_lock, flags);
	list_for_each_entry_safe(reply, reply_tmp, &ept->reply_pend_q, list) {
		list_del(&reply->list);
		kfree(reply);
	}
	list_for_each_entry_safe(reply, reply_tmp, &ept->reply_avail_q, list) {
		list_del(&reply->list);
		kfree(reply);
	}
	spin_unlock_irqrestore(&ept->reply_q_lock, flags);

	wake_lock_destroy(&ept->read_q_wake_lock);
	list_del(&ept->list);
	kfree(ept);
	return 0;
}

static int rpcrouter_create_remote_endpoint(uint32_t cid)
{
	struct rr_remote_endpoint *new_c;
	unsigned long flags;

	new_c = kmalloc(sizeof(struct rr_remote_endpoint), GFP_KERNEL);
	if (!new_c)
		return -ENOMEM;
	memset(new_c, 0, sizeof(struct rr_remote_endpoint));

	new_c->cid = cid;
	new_c->pid = RPCROUTER_PID_REMOTE;
	init_waitqueue_head(&new_c->quota_wait);
	spin_lock_init(&new_c->quota_lock);

	spin_lock_irqsave(&remote_endpoints_lock, flags);
	list_add_tail(&new_c->list, &remote_endpoints);
	new_c->quota_restart_state = RESTART_NORMAL;
	spin_unlock_irqrestore(&remote_endpoints_lock, flags);
	return 0;
}

static struct msm_rpc_endpoint *rpcrouter_lookup_local_endpoint(uint32_t cid)
{
	struct msm_rpc_endpoint *ept;
	unsigned long flags;

	spin_lock_irqsave(&local_endpoints_lock, flags);
	list_for_each_entry(ept, &local_endpoints, list) {
		if (ept->cid == cid) {
			spin_unlock_irqrestore(&local_endpoints_lock, flags);
			return ept;
		}
	}
	spin_unlock_irqrestore(&local_endpoints_lock, flags);
	return NULL;
}

static struct rr_remote_endpoint *rpcrouter_lookup_remote_endpoint(uint32_t cid)
{
	struct rr_remote_endpoint *ept;
	unsigned long flags;

	spin_lock_irqsave(&remote_endpoints_lock, flags);
	list_for_each_entry(ept, &remote_endpoints, list) {
		if (ept->cid == cid) {
			spin_unlock_irqrestore(&remote_endpoints_lock, flags);
			return ept;
		}
	}
	spin_unlock_irqrestore(&remote_endpoints_lock, flags);
	return NULL;
}

static void handle_server_restart(struct rr_server *server, uint32_t cid,
				  uint32_t prog, uint32_t vers)
{
	struct rr_remote_endpoint *r_ept;
	struct msm_rpc_endpoint *ept;
	unsigned long flags;
	r_ept = rpcrouter_lookup_remote_endpoint(cid);
	if (r_ept && (r_ept->quota_restart_state !=
		      RESTART_NORMAL)) {
		spin_lock_irqsave(&r_ept->quota_lock, flags);
		r_ept->tx_quota_cntr = 0;
		r_ept->quota_restart_state =
		RESTART_NORMAL;
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		printk(KERN_INFO "rpcrouter: Remote EP %0x Reset\n",
			   (unsigned int)r_ept);
		wake_up(&r_ept->quota_wait);
	}
	spin_lock_irqsave(&local_endpoints_lock, flags);
	list_for_each_entry(ept, &local_endpoints, list) {
		if ((be32_to_cpu(ept->dst_prog) == prog) &&
		    (be32_to_cpu(ept->dst_vers) == vers) &&
		    (ept->restart_state & RESTART_PEND_SVR)) {
			spin_lock(&ept->restart_lock);
			ept->restart_state &= ~RESTART_PEND_SVR;
			spin_unlock(&ept->restart_lock);
			D("rpcrouter: Local EPT Reset %08x:%08x \n",
			  prog, vers);
			wake_up(&ept->restart_wait);
			wake_up(&ept->wait_q);
		}
	}
	spin_unlock_irqrestore(&local_endpoints_lock, flags);
}

static int process_control_msg(union rr_control_msg *msg, int len)
{
	union rr_control_msg ctl;
	struct rr_server *server;
	struct rr_remote_endpoint *r_ept;
	int rc = 0;
	unsigned long flags;
	static int first = 1;

	if (len != sizeof(*msg)) {
		printk(KERN_ERR "rpcrouter: r2r msg size %d != %d\n",
		       len, sizeof(*msg));
		return -EINVAL;
	}

	switch (msg->cmd) {
	case RPCROUTER_CTRL_CMD_HELLO:
		RR("o HELLO\n");

		RR("x HELLO\n");
		memset(&ctl, 0, sizeof(ctl));
		ctl.cmd = RPCROUTER_CTRL_CMD_HELLO;
		rpcrouter_send_control_msg(&ctl);

		initialized = 1;

		/* Send list of servers one at a time */
		ctl.cmd = RPCROUTER_CTRL_CMD_NEW_SERVER;

		/* TODO: long time to hold a spinlock... */
		spin_lock_irqsave(&server_list_lock, flags);
		list_for_each_entry(server, &server_list, list) {
			if (server->pid != RPCROUTER_PID_LOCAL)
				continue;
			ctl.srv.pid = server->pid;
			ctl.srv.cid = server->cid;
			ctl.srv.prog = server->prog;
			ctl.srv.vers = server->vers;

			RR("x NEW_SERVER id=%d:%08x prog=%08x:%08x\n",
			   server->pid, server->cid,
			   server->prog, server->vers);

			rpcrouter_send_control_msg(&ctl);
		}
		spin_unlock_irqrestore(&server_list_lock, flags);

		if (first) {
			first = 0;
			queue_work(rpcrouter_workqueue,
				   &work_create_rpcrouter_pdev);
		}
		break;

	case RPCROUTER_CTRL_CMD_RESUME_TX:
		RR("o RESUME_TX id=%d:%08x\n", msg->cli.pid, msg->cli.cid);

		r_ept = rpcrouter_lookup_remote_endpoint(msg->cli.cid);
		if (!r_ept) {
			printk(KERN_ERR
			       "rpcrouter: Unable to resume client\n");
			break;
		}
		spin_lock_irqsave(&r_ept->quota_lock, flags);
		r_ept->tx_quota_cntr = 0;
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		wake_up(&r_ept->quota_wait);
		break;

	case RPCROUTER_CTRL_CMD_NEW_SERVER:
		if (msg->srv.vers == 0) {
			pr_err(
			"rpcrouter: Server create rejected, version = 0, "
			"program = %08x\n", msg->srv.prog);
			break;
		}

		RR("o NEW_SERVER id=%d:%08x prog=%08x:%08x\n",
		   msg->srv.pid, msg->srv.cid, msg->srv.prog, msg->srv.vers);

		server = rpcrouter_lookup_server(msg->srv.prog, msg->srv.vers);

		if (!server) {
			server = rpcrouter_create_server(
				msg->srv.pid, msg->srv.cid,
				msg->srv.prog, msg->srv.vers);
			if (!server)
				return -ENOMEM;
			/*
			 * XXX: Verify that its okay to add the
			 * client to our remote client list
			 * if we get a NEW_SERVER notification
			 */
			if (!rpcrouter_lookup_remote_endpoint(msg->srv.cid)) {
				rc = rpcrouter_create_remote_endpoint(
					msg->srv.cid);
				if (rc < 0)
					printk(KERN_ERR
						"rpcrouter:Client create"
						"error (%d)\n", rc);
			}
			schedule_work(&work_create_pdevs);
			wake_up(&newserver_wait);
		} else {
			if ((server->pid == msg->srv.pid) &&
			    (server->cid == msg->srv.cid)) {
				handle_server_restart(server, msg->srv.cid,
						      msg->srv.prog,
						      msg->srv.vers);
			} else {
				server->pid = msg->srv.pid;
				server->cid = msg->srv.cid;
			}
		}
		break;

	case RPCROUTER_CTRL_CMD_REMOVE_SERVER:
		RR("o REMOVE_SERVER prog=%08x:%d\n",
		   msg->srv.prog, msg->srv.vers);
		server = rpcrouter_lookup_server(msg->srv.prog, msg->srv.vers);
		if (server)
			rpcrouter_destroy_server(server);
		break;

	case RPCROUTER_CTRL_CMD_REMOVE_CLIENT:
		RR("o REMOVE_CLIENT id=%d:%08x\n", msg->cli.pid, msg->cli.cid);
		if (msg->cli.pid != RPCROUTER_PID_REMOTE) {
			printk(KERN_ERR
			       "rpcrouter: Denying remote removal of "
			       "local client\n");
			break;
		}
		r_ept = rpcrouter_lookup_remote_endpoint(msg->cli.cid);
		if (r_ept) {
			spin_lock_irqsave(&remote_endpoints_lock, flags);
			list_del(&r_ept->list);
			spin_unlock_irqrestore(&remote_endpoints_lock, flags);
			kfree(r_ept);
		}

		/* Notify local clients of this event */
		printk(KERN_ERR "rpcrouter: LOCAL NOTIFICATION NOT IMP\n");
		rc = -ENOSYS;

		break;
	case RPCROUTER_CTRL_CMD_PING:
		/* No action needed for ping messages received */
		RR("o PING\n");
		break;
	default:
		RR("o UNKNOWN(%08x)\n", msg->cmd);
		rc = -ENOSYS;
	}

	return rc;
}

static void do_create_rpcrouter_pdev(struct work_struct *work)
{
	if (atomic_cmpxchg(&rpcrouter_pdev_created, 0, 1) == 0)
		platform_device_register(&rpcrouter_pdev);
}

static void do_create_pdevs(struct work_struct *work)
{
	unsigned long flags;
	struct rr_server *server;

	/* TODO: race if destroyed while being registered */
	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->pid == RPCROUTER_PID_REMOTE) {
			if (server->pdev_name[0] == 0) {
				spin_unlock_irqrestore(&server_list_lock,
						       flags);
				msm_rpcrouter_create_server_pdev(server);
				schedule_work(&work_create_pdevs);
				return;
			}
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
}

static void rpcrouter_smdnotify(void *_dev, unsigned event)
{
	if (event != SMD_EVENT_DATA)
		return;

	if (smd_read_avail(smd_channel) >= rpcrouter_need_len)
		wake_lock(&rpcrouter_wake_lock);
	wake_up(&smd_wait);
}

static void *rr_malloc(unsigned sz)
{
	void *ptr = kmalloc(sz, GFP_KERNEL);
	if (ptr)
		return ptr;

	printk(KERN_ERR "rpcrouter: kmalloc of %d failed, retrying...\n", sz);
	do {
		ptr = kmalloc(sz, GFP_KERNEL);
	} while (!ptr);

	return ptr;
}

/* TODO: deal with channel teardown / restore */
static int rr_read(void *data, int len)
{
	int rc;
	unsigned long flags;
//	printk("rr_read() %d\n", len);
	for(;;) {
		spin_lock_irqsave(&smd_lock, flags);
		if (smd_read_avail(smd_channel) >= len) {
			rc = smd_read(smd_channel, data, len);
			spin_unlock_irqrestore(&smd_lock, flags);
			if (rc == len)
				return 0;
			else
				return -EIO;
		}
		rpcrouter_need_len = len;
		wake_unlock(&rpcrouter_wake_lock);
		spin_unlock_irqrestore(&smd_lock, flags);

//		printk("rr_read: waiting (%d)\n", len);
		wait_event(smd_wait, smd_read_avail(smd_channel) >= len);
	}
	return 0;
}

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
static char *type_to_str(int i)
{
	switch (i) {
	case RPCROUTER_CTRL_CMD_DATA:
		return "data    ";
	case RPCROUTER_CTRL_CMD_HELLO:
		return "hello   ";
	case RPCROUTER_CTRL_CMD_BYE:
		return "bye     ";
	case RPCROUTER_CTRL_CMD_NEW_SERVER:
		return "new_srvr";
	case RPCROUTER_CTRL_CMD_REMOVE_SERVER:
		return "rmv_srvr";
	case RPCROUTER_CTRL_CMD_REMOVE_CLIENT:
		return "rmv_clnt";
	case RPCROUTER_CTRL_CMD_RESUME_TX:
		return "resum_tx";
	case RPCROUTER_CTRL_CMD_EXIT:
		return "cmd_exit";
	default:
		return "invalid";
	}
}
#endif

static uint32_t r2r_buf[RPCROUTER_MSGSIZE_MAX];

static void do_read_data(struct work_struct *work)
{
	struct rr_header hdr;
	struct rr_packet *pkt;
	struct rr_fragment *frag;
	struct msm_rpc_endpoint *ept;
#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
	struct rpc_request_hdr *rq;
#endif
	uint32_t pm, mid;
	unsigned long flags;

	if (rr_read(&hdr, sizeof(hdr)))
		goto fail_io;

	RR("- ver=%d type=%d src=%d:%08x crx=%d siz=%d dst=%d:%08x\n",
	   hdr.version, hdr.type, hdr.src_pid, hdr.src_cid,
	   hdr.confirm_rx, hdr.size, hdr.dst_pid, hdr.dst_cid);
	RAW_HDR("[r rr_h] "
	    "ver=%i,type=%s,src_pid=%08x,src_cid=%08x,"
	    "confirm_rx=%i,size=%3i,dst_pid=%08x,dst_cid=%08x\n",
	    hdr.version, type_to_str(hdr.type), hdr.src_pid, hdr.src_cid,
	    hdr.confirm_rx, hdr.size, hdr.dst_pid, hdr.dst_cid);

	if (hdr.version != RPCROUTER_VERSION) {
		DIAG("version %d != %d\n", hdr.version, RPCROUTER_VERSION);
		goto fail_data;
	}
	if (hdr.size > RPCROUTER_MSGSIZE_MAX) {
		DIAG("msg size %d > max %d\n", hdr.size, RPCROUTER_MSGSIZE_MAX);
		goto fail_data;
	}

	if (hdr.dst_cid == RPCROUTER_ROUTER_ADDRESS) {
		if (rr_read(r2r_buf, hdr.size))
			goto fail_io;
		process_control_msg((void*) r2r_buf, hdr.size);
		goto done;
	}

	if (hdr.size < sizeof(pm)) {
		DIAG("runt packet (no pacmark)\n");
		goto fail_data;
	}
	if (rr_read(&pm, sizeof(pm)))
		goto fail_io;

	hdr.size -= sizeof(pm);

	frag = rr_malloc(hdr.size + sizeof(*frag));
	frag->next = NULL;
	frag->length = hdr.size;
	if (rr_read(frag->data, hdr.size))
		goto fail_io;

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
	if ((smd_rpcrouter_debug_mask & RAW_PMR) &&
	    ((pm >> 30 & 0x1) || (pm >> 31 & 0x1))) {
		uint32_t xid = 0;
		if (pm >> 30 & 0x1) {
			rq = (struct rpc_request_hdr *) frag->data;
			xid = ntohl(rq->xid);
		}
		if ((pm >> 31 & 0x1) || (pm >> 30 & 0x1))
			RAW_PMR_NOMASK("xid:0x%03x first=%i,last=%i,mid=%3i,"
				       "len=%3i,dst_cid=%08x\n",
				       xid,
				       pm >> 30 & 0x1,
				       pm >> 31 & 0x1,
				       pm >> 16 & 0xF,
				       pm & 0xFFFF, hdr.dst_cid);
	}

	if (smd_rpcrouter_debug_mask & SMEM_LOG) {
		rq = (struct rpc_request_hdr *) frag->data;
		if (rq->xid == 0)
			smem_log_event(SMEM_LOG_PROC_ID_APPS |
				       RPC_ROUTER_LOG_EVENT_MID_READ,
				       PACMARK_MID(pm),
				       hdr.dst_cid,
				       hdr.src_cid);
		else
			smem_log_event(SMEM_LOG_PROC_ID_APPS |
				       RPC_ROUTER_LOG_EVENT_MSG_READ,
				       ntohl(rq->xid),
				       hdr.dst_cid,
				       hdr.src_cid);
	}
#endif

	ept = rpcrouter_lookup_local_endpoint(hdr.dst_cid);
	if (!ept) {
		DIAG("no local ept for cid %08x\n", hdr.dst_cid);
		kfree(frag);
		goto done;
	}

	/* See if there is already a partial packet that matches our mid
	 * and if so, append this fragment to that packet.
	 */
	mid = PACMARK_MID(pm);
	spin_lock_irqsave(&ept->incomplete_lock, flags);
	list_for_each_entry(pkt, &ept->incomplete, list) {
		if (pkt->mid == mid) {
			pkt->last->next = frag;
			pkt->last = frag;
			pkt->length += frag->length;
			if (PACMARK_LAST(pm)) {
				list_del(&pkt->list);
				spin_unlock_irqrestore(&ept->incomplete_lock,
						       flags);
				goto packet_complete;
			}
			spin_unlock_irqrestore(&ept->incomplete_lock, flags);
			goto done;
		}
	}
	spin_unlock_irqrestore(&ept->incomplete_lock, flags);
	/* This mid is new -- create a packet for it, and put it on
	 * the incomplete list if this fragment is not a last fragment,
	 * otherwise put it on the read queue.
	 */
	pkt = rr_malloc(sizeof(struct rr_packet));
	pkt->first = frag;
	pkt->last = frag;
	memcpy(&pkt->hdr, &hdr, sizeof(hdr));
	pkt->mid = mid;
	pkt->length = frag->length;
	if (!PACMARK_LAST(pm)) {
		list_add_tail(&pkt->list, &ept->incomplete);
		goto done;
	}

packet_complete:
	spin_lock_irqsave(&ept->read_q_lock, flags);
	wake_lock(&ept->read_q_wake_lock);
	list_add_tail(&pkt->list, &ept->read_q);
	wake_up(&ept->wait_q);
	spin_unlock_irqrestore(&ept->read_q_lock, flags);
done:

	if (hdr.confirm_rx) {
		union rr_control_msg msg;

		msg.cmd = RPCROUTER_CTRL_CMD_RESUME_TX;
		msg.cli.pid = hdr.dst_pid;
		msg.cli.cid = hdr.dst_cid;

		RR("x RESUME_TX id=%d:%08x\n", msg.cli.pid, msg.cli.cid);
		rpcrouter_send_control_msg(&msg);

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
		if (smd_rpcrouter_debug_mask & SMEM_LOG)
			smem_log_event(SMEM_LOG_PROC_ID_APPS |
				       RPC_ROUTER_LOG_EVENT_MSG_CFM_SNT,
				       RPCROUTER_PID_LOCAL,
				       hdr.dst_cid,
				       hdr.src_cid);
#endif

	}

	queue_work(rpcrouter_workqueue, &work_read_data);
	return;

fail_io:
fail_data:
	printk(KERN_ERR "rpc_router has died\n");
	wake_unlock(&rpcrouter_wake_lock);
}

void msm_rpc_setup_req(struct rpc_request_hdr *hdr, uint32_t prog,
		       uint32_t vers, uint32_t proc)
{
	memset(hdr, 0, sizeof(struct rpc_request_hdr));
	hdr->xid = cpu_to_be32(atomic_add_return(1, &next_xid));
	hdr->rpc_vers = cpu_to_be32(2);
	hdr->prog = cpu_to_be32(prog);
	hdr->vers = cpu_to_be32(vers);
	hdr->procedure = cpu_to_be32(proc);
}
EXPORT_SYMBOL(msm_rpc_setup_req);

struct msm_rpc_endpoint *msm_rpc_open(void)
{
	struct msm_rpc_endpoint *ept;

	ept = msm_rpcrouter_create_local_endpoint(MKDEV(0, 0));
	if (ept == NULL)
		return ERR_PTR(-ENOMEM);

	return ept;
}

int msm_rpc_close(struct msm_rpc_endpoint *ept)
{
	return msm_rpcrouter_destroy_local_endpoint(ept);
}
EXPORT_SYMBOL(msm_rpc_close);

static int msm_rpc_write_pkt(
	struct rr_header *hdr,
	struct msm_rpc_endpoint *ept,
	struct rr_remote_endpoint *r_ept,
	void *buffer,
	int count,
	int first,
	int last,
	uint32_t mid
	)
{
#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
	struct rpc_request_hdr *rq = buffer;
#endif
	uint32_t pacmark;
	unsigned long flags;
	int needed;

	DEFINE_WAIT(__wait);

	/* Create routing header */
	hdr->type = RPCROUTER_CTRL_CMD_DATA;
	hdr->version = RPCROUTER_VERSION;
	hdr->src_pid = ept->pid;
	hdr->src_cid = ept->cid;
	hdr->confirm_rx = 0;
	hdr->size = count + sizeof(uint32_t);

	for (;;) {
		prepare_to_wait(&ept->restart_wait, &__wait,
				TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&ept->restart_lock, flags);
		if (ept->restart_state == RESTART_NORMAL) {
			spin_unlock_irqrestore(&ept->restart_lock, flags);
			break;
		}
		if (signal_pending(current) &&
		   ((!(ept->flags & MSM_RPC_UNINTERRUPTIBLE)))) {
			spin_unlock_irqrestore(&ept->restart_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&ept->restart_lock, flags);
		schedule();
	}
	finish_wait(&ept->restart_wait, &__wait);

	if (signal_pending(current) &&
		(!(ept->flags & MSM_RPC_UNINTERRUPTIBLE))) {
		return -ERESTARTSYS;
	}

	for (;;) {
		prepare_to_wait(&r_ept->quota_wait, &__wait,
				TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&r_ept->quota_lock, flags);
		if ((r_ept->tx_quota_cntr < RPCROUTER_DEFAULT_RX_QUOTA) ||
		    (r_ept->quota_restart_state != RESTART_NORMAL))
			break;
		if (signal_pending(current) &&
		    (!(ept->flags & MSM_RPC_UNINTERRUPTIBLE)))
			break;
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		schedule();
	}
	finish_wait(&r_ept->quota_wait, &__wait);

	if (r_ept->quota_restart_state != RESTART_NORMAL) {
		spin_lock(&ept->restart_lock);
		ept->restart_state &= ~RESTART_PEND_NTFY;
		spin_unlock(&ept->restart_lock);
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		return -ENETRESET;
	}

	if (signal_pending(current) &&
	    (!(ept->flags & MSM_RPC_UNINTERRUPTIBLE))) {
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		return -ERESTARTSYS;
	}
	r_ept->tx_quota_cntr++;
	if (r_ept->tx_quota_cntr == RPCROUTER_DEFAULT_RX_QUOTA) {
		hdr->confirm_rx = 1;

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
		if (smd_rpcrouter_debug_mask & SMEM_LOG) {
			if (rq->xid == 0)
				smem_log_event(SMEM_LOG_PROC_ID_APPS |
					       RPC_ROUTER_LOG_EVENT_MID_CFM_REQ,
					       hdr->dst_pid,
					       hdr->dst_cid,
					       hdr->src_cid);
			else
				smem_log_event(SMEM_LOG_PROC_ID_APPS |
					       RPC_ROUTER_LOG_EVENT_MSG_CFM_REQ,
					       hdr->dst_pid,
					       hdr->dst_cid,
					       hdr->src_cid);
		}
#endif

	}
	pacmark = PACMARK(count, mid, first, last);

	spin_unlock_irqrestore(&r_ept->quota_lock, flags);

	spin_lock_irqsave(&smd_lock, flags);
	spin_lock(&ept->restart_lock);
	if (ept->restart_state != RESTART_NORMAL) {
		ept->restart_state &= ~RESTART_PEND_NTFY;
		spin_unlock(&ept->restart_lock);
		spin_unlock_irqrestore(&smd_lock, flags);
		return -ENETRESET;
	}

	needed = sizeof(*hdr) + hdr->size;
	while ((ept->restart_state == RESTART_NORMAL) &&
	       (smd_write_avail(smd_channel) < needed)) {
		spin_unlock(&ept->restart_lock);
		spin_unlock_irqrestore(&smd_lock, flags);
		msleep(250);
		spin_lock_irqsave(&smd_lock, flags);
		spin_lock(&ept->restart_lock);
	}
	if (ept->restart_state != RESTART_NORMAL) {
		ept->restart_state &= ~RESTART_PEND_NTFY;
		spin_unlock(&ept->restart_lock);
		spin_unlock_irqrestore(&smd_lock, flags);
		return -ENETRESET;
	}

	/* TODO: deal with full fifo */
	smd_write(smd_channel, hdr, sizeof(*hdr));
	RAW_HDR("[w rr_h] "
	    "ver=%i,type=%s,src_pid=%08x,src_cid=%08x,"
	    "confirm_rx=%i,size=%3i,dst_pid=%08x,dst_cid=%08x\n",
	    hdr->version, type_to_str(hdr->type), hdr->src_pid, hdr->src_cid,
	    hdr->confirm_rx, hdr->size, hdr->dst_pid, hdr->dst_cid);
	smd_write(smd_channel, &pacmark, sizeof(pacmark));

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
	if ((smd_rpcrouter_debug_mask & RAW_PMW) &&
	   ((pacmark >> 30 & 0x1) || (pacmark >> 31 & 0x1))) {
		uint32_t xid = 0;
		if (pacmark >> 30 & 0x1)
			xid = ntohl(rq->xid);
		if ((pacmark >> 31 & 0x1) || (pacmark >> 30 & 0x1))
			RAW_PMW_NOMASK("xid:0x%03x first=%i,last=%i,mid=%2i,"
				       "len=%3i,src_cid=%x\n",
				       xid,
				       pacmark >> 30 & 0x1,
				       pacmark >> 31 & 0x1,
#ifdef CONFIG_MACH_MOT
				       pacmark >> 16 & 0xFF,
#else
				       pacmark >> 16 & 0xF,
#endif
				       pacmark & 0xFFFF, hdr->src_cid);
	}
#endif

	smd_write(smd_channel, buffer, count);
	spin_unlock(&ept->restart_lock);
	spin_unlock_irqrestore(&smd_lock, flags);

#if defined(CONFIG_MSM_ONCRPCROUTER_DEBUG)
	if (smd_rpcrouter_debug_mask & SMEM_LOG) {
		if (rq->xid == 0)
			smem_log_event(SMEM_LOG_PROC_ID_APPS |
				       RPC_ROUTER_LOG_EVENT_MID_WRITTEN,
				       PACMARK_MID(pacmark),
				       hdr->dst_cid,
				       hdr->src_cid);
		else
			smem_log_event(SMEM_LOG_PROC_ID_APPS |
				       RPC_ROUTER_LOG_EVENT_MSG_WRITTEN,
				       ntohl(rq->xid),
				       hdr->dst_cid,
				       hdr->src_cid);
	}
#endif

	return needed;
}

static struct msm_rpc_reply *get_pend_reply(struct msm_rpc_endpoint *ept,
					    uint32_t xid)
{
	unsigned long flags;
	struct msm_rpc_reply *reply;
	spin_lock_irqsave(&ept->reply_q_lock, flags);
	list_for_each_entry(reply, &ept->reply_pend_q, list) {
		if (reply->xid == xid) {
			list_del(&reply->list);
			spin_unlock_irqrestore(&ept->reply_q_lock, flags);
			return reply;
		}
	}
	spin_unlock_irqrestore(&ept->reply_q_lock, flags);
	return NULL;
}

void get_requesting_client(struct msm_rpc_endpoint *ept, uint32_t xid,
			   struct msm_rpc_client_info *clnt_info)
{
	unsigned long flags;
	struct msm_rpc_reply *reply;

	if (!clnt_info)
		return;

	spin_lock_irqsave(&ept->reply_q_lock, flags);
	list_for_each_entry(reply, &ept->reply_pend_q, list) {
		if (reply->xid == xid) {
			clnt_info->pid = reply->pid;
			clnt_info->cid = reply->cid;
			clnt_info->prog = reply->prog;
			clnt_info->vers = reply->vers;
			spin_unlock_irqrestore(&ept->reply_q_lock, flags);
			return;
		}
	}
	spin_unlock_irqrestore(&ept->reply_q_lock, flags);
	return;
}

static void set_avail_reply(struct msm_rpc_endpoint *ept,
			    struct msm_rpc_reply *reply)
{
	unsigned long flags;
	spin_lock_irqsave(&ept->reply_q_lock, flags);
	list_add_tail(&reply->list, &ept->reply_avail_q);
	spin_unlock_irqrestore(&ept->reply_q_lock, flags);
}

static struct msm_rpc_reply *get_avail_reply(struct msm_rpc_endpoint *ept)
{
	struct msm_rpc_reply *reply;
	unsigned long flags;
	if (list_empty(&ept->reply_avail_q)) {
		if (ept->reply_cnt >= RPCROUTER_PEND_REPLIES_MAX) {
			printk(KERN_ERR
			       "exceeding max replies of %d \n",
			       RPCROUTER_PEND_REPLIES_MAX);
			return 0;
		}
		reply = kmalloc(sizeof(struct msm_rpc_reply), GFP_KERNEL);
		if (!reply)
			return 0;
		D("Adding reply 0x%08x \n", (unsigned int)reply);
		memset(reply, 0, sizeof(struct msm_rpc_reply));
		spin_lock_irqsave(&ept->reply_q_lock, flags);
		ept->reply_cnt++;
		spin_unlock_irqrestore(&ept->reply_q_lock, flags);
	} else {
		spin_lock_irqsave(&ept->reply_q_lock, flags);
		reply = list_first_entry(&ept->reply_avail_q,
					 struct msm_rpc_reply,
					 list);
		list_del(&reply->list);
		spin_unlock_irqrestore(&ept->reply_q_lock, flags);
	}
	return reply;
}

static void set_pend_reply(struct msm_rpc_endpoint *ept,
			   struct msm_rpc_reply *reply)
{
		unsigned long flags;
		spin_lock_irqsave(&ept->reply_q_lock, flags);
		list_add_tail(&reply->list, &ept->reply_pend_q);
		spin_unlock_irqrestore(&ept->reply_q_lock, flags);
}

int msm_rpc_write(struct msm_rpc_endpoint *ept, void *buffer, int count)
{
	struct rr_header hdr;
	struct rpc_request_hdr *rq = buffer;
	struct rr_remote_endpoint *r_ept;
	struct msm_rpc_reply *reply;
	int max_tx;
	int tx_cnt;
	char *tx_buf;
	int rc;
	int first_pkt = 1;
	uint32_t mid;

	/* snoop the RPC packet and enforce permissions */

	/* has to have at least the xid and type fields */
	if (count < (sizeof(uint32_t) * 2)) {
		printk(KERN_ERR "rr_write: rejecting runt packet\n");
		return -EINVAL;
	}

	if (rq->type == 0) {
		/* RPC CALL */
		if (count < (sizeof(uint32_t) * 6)) {
			printk(KERN_ERR
			       "rr_write: rejecting runt call packet\n");
			return -EINVAL;
		}
		if (ept->dst_pid == 0xffffffff) {
			printk(KERN_ERR "rr_write: not connected\n");
			return -ENOTCONN;
		}
		if ((ept->dst_prog != rq->prog) ||
		    ((be32_to_cpu(ept->dst_vers) & 0x0fff0000) !=
		     (be32_to_cpu(rq->vers) & 0x0fff0000))) {
			printk(KERN_ERR
			       "rr_write: cannot write to %08x:%08x "
			       "(bound to %08x:%08x)\n",
			       be32_to_cpu(rq->prog), be32_to_cpu(rq->vers),
			       be32_to_cpu(ept->dst_prog),
			       be32_to_cpu(ept->dst_vers));
			return -EINVAL;
		}
		hdr.dst_pid = ept->dst_pid;
		hdr.dst_cid = ept->dst_cid;
		IO("CALL to %08x:%d @ %d:%08x (%d bytes)\n",
		   be32_to_cpu(rq->prog), be32_to_cpu(rq->vers),
		   ept->dst_pid, ept->dst_cid, count);
	} else {
		/* RPC REPLY */
		reply = get_pend_reply(ept, rq->xid);
		if (!reply) {
			printk(KERN_ERR
			       "rr_write: rejecting, reply not found \n");
			return -EINVAL;
		}
		hdr.dst_pid = reply->pid;
		hdr.dst_cid = reply->cid;
		set_avail_reply(ept, reply);
		IO("REPLY to xid=%d @ %d:%08x (%d bytes)\n",
		   be32_to_cpu(rq->xid), hdr.dst_pid, hdr.dst_cid, count);
	}

	r_ept = rpcrouter_lookup_remote_endpoint(hdr.dst_cid);

	if (!r_ept) {
		printk(KERN_ERR
			"msm_rpc_write(): No route to ept "
			"[PID %x CID %x]\n", hdr.dst_pid, hdr.dst_cid);
		return -EHOSTUNREACH;
	}

	tx_cnt = count;
	tx_buf = buffer;
	mid = atomic_add_return(1, &pm_mid) & 0xFF;
	/* The modem's router can only take 500 bytes of data. The
	   first 8 bytes it uses on the modem side for addressing,
	   the next 4 bytes are for the pacmark header. */
	max_tx = RPCROUTER_MSGSIZE_MAX - 8 - sizeof(uint32_t);
	IO("Writing %d bytes, max pkt size is %d\n",
	   tx_cnt, max_tx);
	while (tx_cnt > 0) {
		if (tx_cnt > max_tx) {
			rc = msm_rpc_write_pkt(&hdr, ept, r_ept,
					       tx_buf, max_tx,
					       first_pkt, 0, mid);
			if (rc < 0)
				return rc;
			IO("Wrote %d bytes First %d, Last 0 mid %d\n",
			   rc, first_pkt, mid);
			tx_cnt -= max_tx;
			tx_buf += max_tx;
		} else {
			rc = msm_rpc_write_pkt(&hdr, ept, r_ept,
					       tx_buf, tx_cnt,
					       first_pkt, 1, mid);
			if (rc < 0)
				return rc;
			IO("Wrote %d bytes First %d Last 1 mid %d\n",
			   rc, first_pkt, mid);
			break;
		}
		first_pkt = 0;
	}

	return count;
}
EXPORT_SYMBOL(msm_rpc_write);

/*
 * NOTE: It is the responsibility of the caller to kfree buffer
 */
int msm_rpc_read(struct msm_rpc_endpoint *ept, void **buffer,
		 unsigned user_len, long timeout)
{
	struct rr_fragment *frag, *next;
	char *buf;
	int rc;

	rc = __msm_rpc_read(ept, &frag, user_len, timeout);
	if (rc <= 0)
		return rc;

	/* single-fragment messages conveniently can be
	 * returned as-is (the buffer is at the front)
	 */
	if (frag->next == 0) {
		*buffer = (void*) frag;
		return rc;
	}

	/* multi-fragment messages, we have to do it the
	 * hard way, which is rather disgusting right now
	 */
	buf = rr_malloc(rc);
	*buffer = buf;

	while (frag != NULL) {
		memcpy(buf, frag->data, frag->length);
		next = frag->next;
		buf += frag->length;
		kfree(frag);
		frag = next;
	}

	return rc;
}
EXPORT_SYMBOL(msm_rpc_read);

int msm_rpc_call(struct msm_rpc_endpoint *ept, uint32_t proc,
		 void *_request, int request_size,
		 long timeout)
{
	return msm_rpc_call_reply(ept, proc,
				  _request, request_size,
				  NULL, 0, timeout);
}
EXPORT_SYMBOL(msm_rpc_call);

int msm_rpc_call_reply(struct msm_rpc_endpoint *ept, uint32_t proc,
		       void *_request, int request_size,
		       void *_reply, int reply_size,
		       long timeout)
{
	struct rpc_request_hdr *req = _request;
	struct rpc_reply_hdr *reply;
	int rc;

	if (request_size < sizeof(*req))
		return -ETOOSMALL;

	if (ept->dst_pid == 0xffffffff)
		return -ENOTCONN;

	memset(req, 0, sizeof(*req));
	req->xid = cpu_to_be32(atomic_add_return(1, &next_xid));
	req->rpc_vers = cpu_to_be32(2);
	req->prog = ept->dst_prog;
	req->vers = ept->dst_vers;
	req->procedure = cpu_to_be32(proc);

	rc = msm_rpc_write(ept, req, request_size);
	if (rc < 0)
		return rc;

	for (;;) {
		rc = msm_rpc_read(ept, (void*) &reply, -1, timeout);
		if (rc < 0)
			return rc;
		if (rc < (3 * sizeof(uint32_t))) {
			rc = -EIO;
			break;
		}
		/* we should not get CALL packets -- ignore them */
		if (reply->type == 0) {
			kfree(reply);
			continue;
		}
		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req->xid) {
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != 0) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat != 0) {
			rc = -EINVAL;
			break;
		}
		if (_reply == NULL) {
			rc = 0;
			break;
		}
		if (rc > reply_size) {
			rc = -ENOMEM;
		} else {
			memcpy(_reply, reply, rc);
		}
		break;
	}
	kfree(reply);
	return rc;
}
EXPORT_SYMBOL(msm_rpc_call_reply);


static inline int ept_packet_available(struct msm_rpc_endpoint *ept)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&ept->read_q_lock, flags);
	ret = !list_empty(&ept->read_q);
	spin_unlock_irqrestore(&ept->read_q_lock, flags);
	return ret;
}

int __msm_rpc_read(struct msm_rpc_endpoint *ept,
		   struct rr_fragment **frag_ret,
		   unsigned len, long timeout)
{
	struct rr_packet *pkt;
	struct rpc_request_hdr *rq;
	struct msm_rpc_reply *reply;
	DEFINE_WAIT(__wait);
	unsigned long flags;
	int rc;

	IO("READ on ept %p\n", ept);
	spin_lock_irqsave(&ept->restart_lock, flags);
	if (ept->restart_state !=  RESTART_NORMAL) {
		ept->restart_state &= ~RESTART_PEND_NTFY;
		spin_unlock_irqrestore(&ept->restart_lock, flags);
		return -ENETRESET;
	}
	spin_unlock_irqrestore(&ept->restart_lock, flags);

	if (ept->flags & MSM_RPC_UNINTERRUPTIBLE) {
		if (timeout < 0) {
			wait_event(ept->wait_q, ept_packet_available(ept));
			if (!msm_rpc_clear_netreset(ept))
				return -ENETRESET;
		} else {
			rc = wait_event_timeout(
				ept->wait_q, ept_packet_available(ept),
				timeout);
			if (!msm_rpc_clear_netreset(ept))
				return -ENETRESET;
			if (rc == 0)
				return -ETIMEDOUT;
		}
	} else {
		if (timeout < 0) {
			rc = wait_event_interruptible(
				ept->wait_q, ept_packet_available(ept));
			if (!msm_rpc_clear_netreset(ept))
				return -ENETRESET;
			if (rc < 0)
				return rc;
		} else {
			rc = wait_event_interruptible_timeout(
				ept->wait_q, ept_packet_available(ept),
				timeout);
			if (!msm_rpc_clear_netreset(ept))
				return -ENETRESET;
			if (rc == 0)
				return -ETIMEDOUT;
		}
	}

	spin_lock_irqsave(&ept->read_q_lock, flags);
	if (list_empty(&ept->read_q)) {
		spin_unlock_irqrestore(&ept->read_q_lock, flags);
		return -EAGAIN;
	}
	pkt = list_first_entry(&ept->read_q, struct rr_packet, list);
	if (pkt->length > len) {
		spin_unlock_irqrestore(&ept->read_q_lock, flags);
		return -ETOOSMALL;
	}
	list_del(&pkt->list);
	if (list_empty(&ept->read_q))
		wake_unlock(&ept->read_q_wake_lock);
	spin_unlock_irqrestore(&ept->read_q_lock, flags);

	rc = pkt->length;

	*frag_ret = pkt->first;
	rq = (void*) pkt->first->data;
	if ((rc >= (sizeof(uint32_t) * 3)) && (rq->type == 0)) {
		/* RPC CALL */
		reply = get_avail_reply(ept);
		if (!reply)
			return -ENOMEM;
		reply->cid = pkt->hdr.src_cid;
		reply->pid = pkt->hdr.src_pid;
		reply->xid = rq->xid;
		reply->prog = rq->prog;
		reply->vers = rq->vers;
		set_pend_reply(ept, reply);
	}

	kfree(pkt);

	IO("READ on ept %p (%d bytes)\n", ept, rc);
	return rc;
}

int msm_rpc_is_compatible_version(uint32_t server_version,
				  uint32_t client_version)
{

	if ((server_version & RPC_VERSION_MODE_MASK) !=
	    (client_version & RPC_VERSION_MODE_MASK))
		return 0;

	if (server_version & RPC_VERSION_MODE_MASK)
		return server_version == client_version;

	return ((server_version & RPC_VERSION_MAJOR_MASK) ==
		 (client_version & RPC_VERSION_MAJOR_MASK)) &&
		((server_version & RPC_VERSION_MINOR_MASK) >=
		 (client_version & RPC_VERSION_MINOR_MASK));
}
EXPORT_SYMBOL(msm_rpc_is_compatible_version);

int msm_rpc_get_compatible_server(uint32_t prog,
					uint32_t ver,
					uint32_t *found_vers)
{
	struct rr_server *server;
	unsigned long     flags;
	uint32_t          found = -1;
	if (found_vers == NULL)
		return 0;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if ((server->prog == prog) &&
		    msm_rpc_is_compatible_version(server->vers, ver)) {
			*found_vers = server->vers;
			spin_unlock_irqrestore(&server_list_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
	return found;
}
EXPORT_SYMBOL(msm_rpc_get_compatible_server);

struct msm_rpc_endpoint *msm_rpc_connect_compatible(uint32_t prog,
			uint32_t vers, unsigned flags)
{
	uint32_t found_vers;
	int      ret;
	 ret = msm_rpc_get_compatible_server(prog, vers, &found_vers);
	 if (ret < 0)
		return ERR_PTR(-EHOSTUNREACH);
	if (found_vers != vers) {
		D("RPC Using new version 0x%08x(0x%08x) prog 0x%08x",
			vers, found_vers, prog);
		D(" ... Continuing\n");
	}
	return msm_rpc_connect(prog, found_vers, flags);
}
EXPORT_SYMBOL(msm_rpc_connect_compatible);

struct msm_rpc_endpoint *msm_rpc_connect(uint32_t prog, uint32_t vers, unsigned flags)
{
	struct msm_rpc_endpoint *ept;
	struct rr_server *server;

	server = rpcrouter_lookup_server(prog, vers);
	if (!server)
		return ERR_PTR(-EHOSTUNREACH);

	ept = msm_rpc_open();
	if (IS_ERR(ept))
		return ept;

	ept->flags = flags;
	ept->dst_pid = server->pid;
	ept->dst_cid = server->cid;
	ept->dst_prog = cpu_to_be32(prog);
	ept->dst_vers = cpu_to_be32(vers);

	return ept;
}
EXPORT_SYMBOL(msm_rpc_connect);

/* TODO: permission check? */
int msm_rpc_register_server(struct msm_rpc_endpoint *ept,
			    uint32_t prog, uint32_t vers)
{
	int rc;
	union rr_control_msg msg;
	struct rr_server *server;

	server = rpcrouter_create_server(ept->pid, ept->cid,
					 prog, vers);
	if (!server)
		return -ENODEV;

	msg.srv.cmd = RPCROUTER_CTRL_CMD_NEW_SERVER;
	msg.srv.pid = ept->pid;
	msg.srv.cid = ept->cid;
	msg.srv.prog = prog;
	msg.srv.vers = vers;

	RR("x NEW_SERVER id=%d:%08x prog=%08x:%08x\n",
	   ept->pid, ept->cid, prog, vers);

	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		return rc;

	return 0;
}

int msm_rpc_clear_netreset(struct msm_rpc_endpoint *ept)
{
	unsigned long flags;
	int rc = 1;
	RR("RESET RESTART FLAG for EPT:%08x \n", (unsigned int)ept);
	spin_lock_irqsave(&ept->restart_lock, flags);
	if (ept->restart_state !=  RESTART_NORMAL) {
		ept->restart_state &= ~RESTART_PEND_NTFY;
		rc = 0;
	}
	spin_unlock_irqrestore(&ept->restart_lock, flags);
	return rc;
}

/* TODO: permission check -- disallow unreg of somebody else's server */
int msm_rpc_unregister_server(struct msm_rpc_endpoint *ept,
			      uint32_t prog, uint32_t vers)
{
	struct rr_server *server;
	server = rpcrouter_lookup_server(prog, vers);

	if (!server)
		return -ENOENT;
	rpcrouter_destroy_server(server);
	return 0;
}

static int msm_rpcrouter_modem_notify(struct notifier_block *this,
				      unsigned long code,
				      void *_cmd)
{
	switch (code) {
	case MODEM_NOTIFIER_START_RESET:
		NTFY("%s: MODEM_NOTIFIER_START_RESET", __func__);
		modem_reset_start_cleanup();
		break;
	case MODEM_NOTIFIER_END_RESET:
		NTFY("%s: MODEM_NOTIFIER_END_RESET", __func__);
		break;
	default:
		NTFY("%s: default", __func__);
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block msm_rpcrouter_nb = {
	.notifier_call = msm_rpcrouter_modem_notify,
};

static int msm_rpcrouter_probe(struct platform_device *pdev)
{
	int rc;

	/* Initialize what we need to start processing */
	INIT_LIST_HEAD(&local_endpoints);
	INIT_LIST_HEAD(&remote_endpoints);

	init_waitqueue_head(&newserver_wait);
	init_waitqueue_head(&smd_wait);
	wake_lock_init(&rpcrouter_wake_lock, WAKE_LOCK_SUSPEND, "SMD_RPCCALL");

	rpcrouter_workqueue = create_singlethread_workqueue("rpcrouter");
	if (!rpcrouter_workqueue)
		return -ENOMEM;

	rc = msm_rpcrouter_init_devices();
	if (rc < 0)
		goto fail_destroy_workqueue;

	rc = modem_register_notifier(&msm_rpcrouter_nb);
	if (rc < 0)
		goto fail_remove_devices;

	/* Open up SMD channel 2 */
	initialized = 0;
	rc = smd_open("RPCCALL", &smd_channel, NULL, rpcrouter_smdnotify);
	if (rc < 0)
		goto fail_remove_reset_notifier;

	queue_work(rpcrouter_workqueue, &work_read_data);
	return 0;

 fail_remove_reset_notifier:
	modem_unregister_notifier(&msm_rpcrouter_nb);
 fail_remove_devices:
	msm_rpcrouter_exit_devices();
 fail_destroy_workqueue:
	destroy_workqueue(rpcrouter_workqueue);
	return rc;
}

static struct platform_driver msm_smd_channel2_driver = {
	.probe		= msm_rpcrouter_probe,
	.driver		= {
			.name	= "RPCCALL",
			.owner	= THIS_MODULE,
	},
};

#if defined(CONFIG_DEBUG_FS)
#define HSIZE 13

struct sym {
	uint32_t val;
	char *str;
	struct hlist_node node;
};

static struct sym oncrpc_syms[] = {
	{ 0x30000000, "CM" },
	{ 0x30000001, "DB" },
	{ 0x30000002, "SND" },
	{ 0x30000003, "WMS" },
	{ 0x30000004, "PDSM" },
	{ 0x30000005, "MISC_MODEM_APIS" },
	{ 0x30000006, "MISC_APPS_APIS" },
	{ 0x30000007, "JOYST" },
	{ 0x30000008, "VJOY" },
	{ 0x30000009, "JOYSTC" },
	{ 0x3000000a, "ADSPRTOSATOM" },
	{ 0x3000000b, "ADSPRTOSMTOA" },
	{ 0x3000000c, "I2C" },
	{ 0x3000000d, "TIME_REMOTE" },
	{ 0x3000000e, "NV" },
	{ 0x3000000f, "CLKRGM_SEC" },
	{ 0x30000010, "RDEVMAP" },
	{ 0x30000011, "FS_RAPI" },
	{ 0x30000012, "PBMLIB" },
	{ 0x30000013, "AUDMGR" },
	{ 0x30000014, "MVS" },
	{ 0x30000015, "DOG_KEEPALIVE" },
	{ 0x30000016, "GSDI_EXP" },
	{ 0x30000017, "AUTH" },
	{ 0x30000018, "NVRUIMI" },
	{ 0x30000019, "MMGSDILIB" },
	{ 0x3000001a, "CHARGER" },
	{ 0x3000001b, "UIM" },
	{ 0x3000001C, "ONCRPCTEST" },
	{ 0x3000001d, "PDSM_ATL" },
	{ 0x3000001e, "FS_XMOUNT" },
	{ 0x3000001f, "SECUTIL " },
	{ 0x30000020, "MCCMEID" },
	{ 0x30000021, "PM_STROBE_FLASH" },
	{ 0x30000022, "DS707_EXTIF" },
	{ 0x30000023, "SMD BRIDGE_MODEM" },
	{ 0x30000024, "SMD PORT_MGR" },
	{ 0x30000025, "BUS_PERF" },
	{ 0x30000026, "BUS_MON" },
	{ 0x30000027, "MC" },
	{ 0x30000028, "MCCAP" },
	{ 0x30000029, "MCCDMA" },
	{ 0x3000002a, "MCCDS" },
	{ 0x3000002b, "MCCSCH" },
	{ 0x3000002c, "MCCSRID" },
	{ 0x3000002d, "SNM" },
	{ 0x3000002e, "MCCSYOBJ" },
	{ 0x3000002f, "DS707_APIS" },
	{ 0x30000030, "DS_MP_SHIM_APPS_ASYNC" },
	{ 0x30000031, "DSRLP_APIS" },
	{ 0x30000032, "RLP_APIS" },
	{ 0x30000033, "DS_MP_SHIM_MODEM" },
	{ 0x30000034, "DSHDR_APIS" },
	{ 0x30000035, "DSHDR_MDM_APIS" },
	{ 0x30000036, "DS_MP_SHIM_APPS" },
	{ 0x30000037, "HDRMC_APIS" },
	{ 0x30000038, "SMD_BRIDGE_MTOA" },
	{ 0x30000039, "SMD_BRIDGE_ATOM" },
	{ 0x3000003a, "DPMAPP_OTG" },
	{ 0x3000003b, "DIAG" },
	{ 0x3000003c, "GSTK_EXP" },
	{ 0x3000003d, "DSBC_MDM_APIS" },
	{ 0x3000003e, "HDRMRLP_MDM_APIS" },
	{ 0x3000003f, "HDRMRLP_APPS_APIS" },
	{ 0x30000040, "HDRMC_MRLP_APIS" },
	{ 0x30000041, "PDCOMM_APP_API" },
	{ 0x30000042, "DSAT_APIS" },
	{ 0x30000043, "MISC_RF_APIS" },
	{ 0x30000044, "CMIPAPP" },
	{ 0x30000045, "DSMP_UMTS_MODEM_APIS" },
	{ 0x30000046, "DSMP_UMTS_APPS_APIS" },
	{ 0x30000047, "DSUCSDMPSHIM" },
	{ 0x30000048, "TIME_REMOTE_ATOM" },
	{ 0x3000004a, "SD" },
	{ 0x3000004b, "MMOC" },
	{ 0x3000004c, "WLAN_ADP_FTM" },
	{ 0x3000004d, "WLAN_CP_CM" },
	{ 0x3000004e, "FTM_WLAN" },
	{ 0x3000004f, "SDCC_CPRM" },
	{ 0x30000050, "CPRMINTERFACE" },
	{ 0x30000051, "DATA_ON_MODEM_MTOA_APIS" },
	{ 0x30000052, "DATA_ON_APPS_ATOM_APIS" },
	{ 0x30000053, "MISC_MODEM_APIS_NONWINMOB" },
	{ 0x30000054, "MISC_APPS_APIS_NONWINMOB" },
	{ 0x30000055, "PMEM_REMOTE" },
	{ 0x30000056, "TCXOMGR" },
	{ 0x30000057, "DSUCSDAPPIF_APIS" },
	{ 0x30000058, "BT" },
	{ 0x30000059, "PD_COMMS_API" },
	{ 0x3000005a, "PD_COMMS_CLIENT_API" },
	{ 0x3000005b, "PDAPI" },
	{ 0x3000005c, "LSA_SUPL_DSM" },
	{ 0x3000005d, "TIME_REMOTE_MTOA" },
	{ 0x3000005e, "FTM_BT" },
	{ 0X3000005f, "DSUCSDAPPIF_APIS" },
	{ 0X30000060, "PMAPP_GEN" },
	{ 0X30000061, "PM_LIB" },
	{ 0X30000062, "KEYPAD" },
	{ 0X30000063, "HSU_APP_APIS" },
	{ 0X30000064, "HSU_MDM_APIS" },
	{ 0X30000065, "ADIE_ADC_REMOTE_ATOM " },
	{ 0X30000066, "TLMM_REMOTE_ATOM" },
	{ 0X30000067, "UI_CALLCTRL" },
	{ 0X30000068, "UIUTILS" },
	{ 0X30000069, "PRL" },
	{ 0X3000006a, "HW" },
	{ 0X3000006b, "OEM_RAPI" },
	{ 0X3000006c, "WMSPM" },
	{ 0X3000006d, "BTPF" },
	{ 0X3000006e, "CLKRGM_SYNC_EVENT" },
	{ 0X3000006f, "USB_APPS_RPC" },
	{ 0X30000070, "USB_MODEM_RPC" },
	{ 0X30000071, "ADC" },
	{ 0X30000072, "CAMERAREMOTED" },
	{ 0X30000073, "SECAPIREMOTED" },
	{ 0X30000074, "DSATAPI" },
	{ 0X30000075, "CLKCTL_RPC" },
	{ 0X30000076, "BREWAPPCOORD" },
	{ 0X30000077, "ALTENVSHELL" },
	{ 0X30000078, "WLAN_TRP_UTILS" },
	{ 0X30000079, "GPIO_RPC" },
	{ 0X3000007a, "PING_RPC" },
	{ 0X3000007b, "DSC_DCM_API" },
	{ 0X3000007c, "L1_DS" },
	{ 0X3000007d, "QCHATPK_APIS" },
	{ 0X3000007e, "GPS_API" },
	{ 0X3000007f, "OSS_RRCASN_REMOTE" },
	{ 0X30000080, "PMAPP_OTG_REMOTE" },
	{ 0X30000081, "PING_MDM_RPC" },
	{ 0X30000082, "PING_KERNEL_RPC" },
	{ 0X30000083, "TIMETICK" },
	{ 0X30000084, "WM_BTHCI_FTM " },
	{ 0X30000085, "WM_BT_PF" },
	{ 0X30000086, "IPA_IPC_APIS" },
	{ 0X30000087, "UKCC_IPC_APIS" },
	{ 0X30000088, "CMIPSMS " },
	{ 0X30000089, "VBATT_REMOTE" },
	{ 0X3000008a, "MFPAL" },
	{ 0X3000008b, "DSUMTSPDPREG" },
	{ 0X3000fe00, "RESTART_DAEMON NUMBER 0" },
	{ 0X3000fe01, "RESTART_DAEMON NUMBER 1" },
	{ 0X3000feff, "RESTART_DAEMON NUMBER 255" },
	{ 0X3000fffe, "BACKWARDS_COMPATIBILITY_IN_RPC_CLNT_LOOKUP" },
	{ 0X3000ffff, "RPC_ROUTER_SERVER_PROGRAM" },
};

#define ONCRPC_SYM 0

static struct sym_tbl {
	struct sym *data;
	int size;
	struct hlist_head hlist[HSIZE];
} tbl[] = {
	{ oncrpc_syms, ARRAY_SIZE(oncrpc_syms) },
};

#define hash(val) (val % HSIZE)

static void init_syms(void)
{
	int i;
	int j;

	for (i = 0; i < ARRAY_SIZE(tbl); ++i)
		for (j = 0; j < HSIZE; ++j)
			INIT_HLIST_HEAD(&tbl[i].hlist[j]);

	for (i = 0; i < ARRAY_SIZE(tbl); ++i)
		for (j = 0; j < tbl[i].size; ++j) {
			INIT_HLIST_NODE(&tbl[i].data[j].node);
			hlist_add_head(&tbl[i].data[j].node,
				       &tbl[i].hlist[hash(tbl[i].data[j].val)]);
		}
}

static char *find_sym(uint32_t id, uint32_t val)
{
	struct hlist_node *n;
	struct sym *s;

	hlist_for_each(n, &tbl[id].hlist[hash(val)]) {
		s = hlist_entry(n, struct sym, node);
		if (s->val == val)
			return s->str;
	}

	return 0;
}

static int dump_servers(char *buf, int max)
{
	int i = 0;
	unsigned long flags;
	struct rr_server *svr;
	char *sym;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(svr, &server_list, list) {
		i += scnprintf(buf + i, max - i, "pdev_name: %s\n",
			       svr->pdev_name);
		i += scnprintf(buf + i, max - i, "pid: 0x%08x\n", svr->pid);
		i += scnprintf(buf + i, max - i, "cid: 0x%08x\n", svr->cid);
		i += scnprintf(buf + i, max - i, "prog: 0x%08x", svr->prog);
		sym = find_sym(ONCRPC_SYM, svr->prog);
		if (sym)
			i += scnprintf(buf + i, max - i, " (%s)\n", sym);
		else
			i += scnprintf(buf + i, max - i, "\n");
		i += scnprintf(buf + i, max - i, "vers: 0x%08x\n", svr->vers);
		i += scnprintf(buf + i, max - i, "\n");
	}
	spin_unlock_irqrestore(&server_list_lock, flags);

	return i;
}

static int dump_remote_endpoints(char *buf, int max)
{
	int i = 0;
	unsigned long flags;
	struct rr_remote_endpoint *ept;

	spin_lock_irqsave(&remote_endpoints_lock, flags);
	list_for_each_entry(ept, &remote_endpoints, list) {
		i += scnprintf(buf + i, max - i, "pid: 0x%08x\n", ept->pid);
		i += scnprintf(buf + i, max - i, "cid: 0x%08x\n", ept->cid);
		i += scnprintf(buf + i, max - i, "tx_quota_cntr: %i\n",
			       ept->tx_quota_cntr);
		i += scnprintf(buf + i, max - i, "quota_restart_state: %i\n",
			       ept->quota_restart_state);
		i += scnprintf(buf + i, max - i, "\n");
	}
	spin_unlock_irqrestore(&remote_endpoints_lock, flags);

	return i;
}

static int dump_msm_rpc_endpoint(char *buf, int max)
{
	int i = 0;
	unsigned long flags;
	struct msm_rpc_reply *reply;
	struct msm_rpc_endpoint *ept;
	struct rr_packet *pkt;
	char *sym;

	spin_lock_irqsave(&local_endpoints_lock, flags);
	list_for_each_entry(ept, &local_endpoints, list) {
		i += scnprintf(buf + i, max - i, "pid: 0x%08x\n", ept->pid);
		i += scnprintf(buf + i, max - i, "cid: 0x%08x\n", ept->cid);
		i += scnprintf(buf + i, max - i, "dst_pid: 0x%08x\n",
			       ept->dst_pid);
		i += scnprintf(buf + i, max - i, "dst_cid: 0x%08x\n",
			       ept->dst_cid);
		i += scnprintf(buf + i, max - i, "dst_prog: 0x%08x",
			       be32_to_cpu(ept->dst_prog));
		sym = find_sym(ONCRPC_SYM, be32_to_cpu(ept->dst_prog));
		if (sym)
			i += scnprintf(buf + i, max - i, " (%s)\n", sym);
		else
			i += scnprintf(buf + i, max - i, "\n");
		i += scnprintf(buf + i, max - i, "dst_vers: 0x%08x\n",
			       be32_to_cpu(ept->dst_vers));
		i += scnprintf(buf + i, max - i, "reply_cnt: %i\n",
			       ept->reply_cnt);
		i += scnprintf(buf + i, max - i, "restart_state: %i\n",
			       ept->restart_state);

		i += scnprintf(buf + i, max - i, "outstanding xids:\n");
		spin_lock(&ept->reply_q_lock);
		list_for_each_entry(reply, &ept->reply_pend_q, list)
			i += scnprintf(buf + i, max - i, "    xid = %u\n",
				       ntohl(reply->xid));
		spin_unlock(&ept->reply_q_lock);

		i += scnprintf(buf + i, max - i, "complete unread packets:\n");
		spin_lock(&ept->read_q_lock);
		list_for_each_entry(pkt, &ept->read_q, list) {
			i += scnprintf(buf + i, max - i, "    mid = %i\n",
				       pkt->mid);
			i += scnprintf(buf + i, max - i, "    length = %i\n",
				       pkt->length);
		}
		spin_unlock(&ept->read_q_lock);
		i += scnprintf(buf + i, max - i, "\n");
	}
	spin_unlock_irqrestore(&local_endpoints_lock, flags);

	return i;
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
			 struct dentry *dent,
			 int (*fill)(char *buf, int max))
{
	debugfs_create_file(name, mode, dent, fill, &debug_ops);
}

static void debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("smd_rpcrouter", 0);
	if (IS_ERR(dent))
		return;

	debug_create("dump_msm_rpc_endpoints", 0444, dent,
		     dump_msm_rpc_endpoint);
	debug_create("dump_remote_endpoints", 0444, dent,
		     dump_remote_endpoints);
	debug_create("dump_servers", 0444, dent,
		     dump_servers);

	init_syms();
}

#else
static void debugfs_init(void) {}
#endif


static int __init rpcrouter_init(void)
{
	int ret;

	ret = platform_driver_register(&msm_smd_channel2_driver);
	if (ret)
		return ret;

	debugfs_init();

	return ret;
}

module_init(rpcrouter_init);
MODULE_DESCRIPTION("MSM RPC Router");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
