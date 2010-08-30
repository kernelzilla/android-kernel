/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#ifndef DIAGCHAR_H
#define DIAGCHAR_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/msm_smd.h>
#include <asm/atomic.h>

/* Size of the USB buffers used for read and write*/
#define USB_MAX_BUF 4096

/* Size of the buffer used for deframing a packet
  reveived from the PC tool*/
#define HDLC_MAX 4096

/* Number of maximum USB requests that the USB layer should handle at
   one time. */
#define MAX_DIAG_USB_REQUESTS 12
#define MSG_MASK_SIZE 8000
#define LOG_MASK_SIZE 1000
#define EVENT_MASK_SIZE 1000
#define REG_TABLE_SIZE 25
#define PKT_SIZE 1000

struct diag_master_table {
	uint16_t cmd_code;
	uint16_t subsys_id;
	uint16_t cmd_code_lo;
	uint16_t cmd_code_hi;
	int process_id;
};

#ifdef CONFIG_MACH_MOT
extern void pretty_hex_dump (const char *title, unsigned char *buf, int len);
void diagchar_mot_ftm_reply (unsigned char *buf, int len);
#endif

struct diagchar_dev {

	/* State for the char driver */
	unsigned int major;
	unsigned int minor_start;
#ifdef CONFIG_MACH_MOT
	int *tcmd_client_map; 
#endif
	int num;
	struct cdev *cdev;
	char *name;
	int dropped_count;
	struct class *diagchar_class;
	int ref_count;
	struct mutex diagchar_mutex;
	wait_queue_head_t wait_q;
	int *client_map;
	int *data_ready;
	int num_clients;
	unsigned int itemsize;
	unsigned int poolsize;

	/* State for the mempool for the char driver */
	mempool_t *diagpool;
	struct mutex diagmem_mutex;
	int count;

	/* State for diag forwarding */
	unsigned char *usb_buf_in;
	unsigned char *usb_buf_in_qdsp;
	unsigned char *usb_buf_out;
	smd_channel_t *ch;
	smd_channel_t *chqdsp;
	int in_busy;
	int in_busy_qdsp;
	int read_len;
	unsigned char *hdlc_buf;
	unsigned hdlc_count;
	unsigned hdlc_escape;
	int usb_connected;
	struct workqueue_struct *diag_wq;
	struct work_struct diag_read_work;
	uint8_t *msg_masks;
	uint8_t *log_masks;
	uint8_t *event_masks;
	struct diag_master_table *table;
	uint8_t *pkt_buf;
	int pkt_length;
};

extern struct diagchar_dev *driver;
#endif
