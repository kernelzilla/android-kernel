/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef __ASM__ARCH_CAMERA_H
#define __ASM__ARCH_CAMERA_H

#include <linux/list.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#ifndef CONFIG_MACH_MOT
#include <linux/wakelock.h>
#endif
#include "linux/types.h"

#include <mach/board.h>
#include <media/msm_camera.h>

#ifdef CONFIG_MACH_MOT
//#define CAMERA_DBG_MSG

#undef CDBG
#ifdef CAMERA_DBG_MSG
#define CDBG(fmt, args...) printk(KERN_INFO "msm_camera: " fmt, ##args)
#else
#define CDBG(fmt, args...)
#endif
#undef CVBS
#ifdef CAMERA_DBG_MSG
#define CVBS(fmt, args...) printk(KERN_INFO "msm_camera: " fmt, ##args)
#else
#define CVBS(fmt, args...)
#endif

#else

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define CDBG(fmt, args...) printk(KERN_INFO "msm_camera: " fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
#endif

#define MSM_CAMERA_MSG 0
#define MSM_CAMERA_EVT 1
#define NUM_WB_EXP_NEUTRAL_REGION_LINES 4
#define NUM_WB_EXP_STAT_OUTPUT_BUFFERS  3
#define NUM_AUTOFOCUS_MULTI_WINDOW_GRIDS 16
#define NUM_AF_STAT_OUTPUT_BUFFERS      3

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
enum msm_queut_t {
	MSM_CAM_Q_IVALID,
	MSM_CAM_Q_CTRL,
	MSM_CAM_Q_VFE_EVT,
	MSM_CAM_Q_VFE_MSG,
	MSM_CAM_Q_V4L2_REQ,

	MSM_CAM_Q_MAX
};
#else
enum msm_queue {
	MSM_CAM_Q_CTRL,     /* control command or control command status */
	MSM_CAM_Q_VFE_EVT,  /* adsp event */
	MSM_CAM_Q_VFE_MSG,  /* adsp message */
	MSM_CAM_Q_V4L2_REQ, /* v4l2 request */
};
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
enum vfe_resp_msg_t {
#else
enum vfe_resp_msg {
#endif
	VFE_EVENT,
	VFE_MSG_GENERAL,
	VFE_MSG_SNAPSHOT,
	VFE_MSG_OUTPUT1,
	VFE_MSG_OUTPUT2,
	VFE_MSG_STATS_AF,
	VFE_MSG_STATS_WE,

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	VFE_MSG_INVALID
#endif
};

struct msm_vfe_phy_info {
	uint32_t sbuf_phy;
	uint32_t y_phy;
	uint32_t cbcr_phy;
};

#if defined(CONFIG_MACH_PITTSBURGH)
struct msm_vfe_resp_t {
	enum vfe_resp_msg_t type;
	struct msm_vfe_evt_msg_t evt_msg;
	struct msm_vfe_phy_info  phy;
	void    *extdata;
	int32_t extlen;
};
#elif defined(CONFIG_MACH_MOT)
struct msm_vfe_resp_t {
	enum vfe_resp_msg_t type;
	struct msm_vfe_evt_msg evt_msg;
	struct msm_vfe_phy_info  phy;
	void    *extdata;
	int32_t extlen;
};
#else
struct msm_vfe_resp {
	enum vfe_resp_msg type;
	struct msm_vfe_evt_msg evt_msg;
	struct msm_vfe_phy_info phy;
	void    *extdata;
	int32_t extlen;
};
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct msm_vfe_resp {
	void (*vfe_resp)(struct msm_vfe_resp_t *,
		enum msm_queut_t, void *syncdata);
};
#else
struct msm_vfe_callback {
	void (*vfe_resp)(struct msm_vfe_resp *,
		enum msm_queue, void *syncdata);
	void* (*vfe_alloc)(int, void *syncdata);
};
#endif

#if defined(CONFIG_MACH_PITTSBURGH)
struct msm_camvfe_fn_t {
	int (*vfe_init)      (struct msm_vfe_resp *, struct platform_device *);
	int (*vfe_enable)    (struct camera_enable_cmd_t *);
	int (*vfe_config)    (struct msm_vfe_cfg_cmd_t *, void *);
	int (*vfe_disable)   (struct camera_enable_cmd_t *,
		struct platform_device *dev);
	void (*vfe_release)  (struct platform_device *);
};
#elif defined(CONFIG_MACH_MOT)
struct msm_camvfe_fn_t {
	int (*vfe_init)      (struct msm_vfe_resp *, struct platform_device *);
	int (*vfe_enable)    (struct camera_enable_cmd *);
	int (*vfe_config)    (struct msm_vfe_cfg_cmd *, void *);
	int (*vfe_disable)   (struct camera_enable_cmd *,
		struct platform_device *dev);
	void (*vfe_release)  (struct platform_device *);
};
#else
struct msm_camvfe_fn {
	int (*vfe_init)(struct msm_vfe_callback *, struct platform_device *);
	int (*vfe_enable)(struct camera_enable_cmd *);
	int (*vfe_config)(struct msm_vfe_cfg_cmd *, void *);
	int (*vfe_disable)(struct camera_enable_cmd *,
		struct platform_device *dev);
	void (*vfe_release)(struct platform_device *);
};
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct msm_sensor_ctrl_t {
	int (*s_init)(struct msm_camera_sensor_info *);
	int (*s_release)(void);
	int (*s_config)(void __user *);
};
#else
struct msm_sensor_ctrl {
	int (*s_init)(const struct msm_camera_sensor_info *);
	int (*s_release)(void);
	int (*s_config)(void __user *);
};
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct msm_sync_t {
	spinlock_t msg_event_queue_lock;
	struct list_head msg_event_queue;
	wait_queue_head_t msg_event_wait;

	spinlock_t prev_frame_q_lock;
	struct list_head prev_frame_q;
	wait_queue_head_t prev_frame_wait;

	spinlock_t pict_frame_q_lock;
	struct list_head pict_frame_q;
	wait_queue_head_t pict_frame_wait;

	spinlock_t ctrl_status_lock;
	struct list_head ctrl_status_queue;
	wait_queue_head_t ctrl_status_wait;

	spinlock_t af_status_lock;
#if defined(CONFIG_MACH_PITTSBURGH)
	struct msm_ctrl_cmd_t af_status;
#else
	struct msm_ctrl_cmd af_status;
#endif
	int af_flag;
	wait_queue_head_t af_status_wait;
	struct hlist_head frame;
	struct hlist_head stats;
};
#else
struct msm_sync {
	/* These two queues are accessed from a process context only. */
	struct hlist_head frame; /* most-frequently accessed */
	struct hlist_head stats;

	/* The message queue is used by the control thread to send commands
	 * to the config thread, and also by the DSP to send messages to the
	 * config thread.  Thus it is the only queue that is accessed from
	 * both interrupt and process context.
	 */
	spinlock_t msg_event_q_lock;
	struct list_head msg_event_q;
	wait_queue_head_t msg_event_wait;

	/* This queue contains preview frames. It is accessed by the DSP (in
	 * in interrupt context, and by the frame thread.
	 */
	spinlock_t prev_frame_q_lock;
	struct list_head prev_frame_q;
	wait_queue_head_t prev_frame_wait;
	int unblock_poll_frame;

	/* This queue contains snapshot frames.  It is accessed by the DSP (in
	 * interrupt context, and by the control thread.
	 */
	spinlock_t pict_frame_q_lock;
	struct list_head pict_frame_q;
	wait_queue_head_t pict_frame_wait;

	struct msm_camera_sensor_info *sdata;
	struct msm_camvfe_fn vfefn;
	struct msm_sensor_ctrl sctrl;
	struct wake_lock wake_lock;
	struct platform_device *pdev;
	uint8_t opencnt;
	void *cropinfo;
	int  croplen;
	uint32_t pict_pp;
	struct mutex pict_pp_lock;
	uint8_t pp_sync_flag;

	const char *apps_id;

	struct mutex lock;
	struct list_head list;
};
#endif

#ifndef CONFIG_MACH_MOT
#define MSM_APPS_ID_V4L2 "msm_v4l2"
#define MSM_APPS_ID_PROP "msm_qct"
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct msm_device_t {
	struct msm_camvfe_fn_t vfefn;
	struct device *device;
	struct cdev cdev;
	struct platform_device *pdev;

	struct mutex msm_lock;
	uint8_t opencnt;

	const char *apps_id;

	void *cropinfo;
	int  croplen;

	struct mutex pict_pp_lock;
	uint32_t pict_pp;

	int sidx;
	struct msm_sensor_ctrl_t sctrl;

	struct mutex msm_sem;
	struct msm_sync_t sync;
};
#else
struct msm_device {
	struct msm_sync *sync; /* most-frequently accessed */
	struct device *device;
	struct cdev cdev;
	/* opened is meaningful only for the config and frame nodes,
	 * which may be opened only once.
	 */
	atomic_t opened;
};
#endif

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
struct msm_control_device_queue {
	spinlock_t ctrl_status_q_lock;
	struct list_head ctrl_status_q;
	wait_queue_head_t ctrl_status_wait;
};

struct msm_control_device {
	struct msm_device *pmsm;

	/* This queue used by the config thread to send responses back to the
	 * control thread.  It is accessed only from a process context.
	 */
	struct msm_control_device_queue ctrl_q;
};
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct msm_queue_cmd_t {
	struct list_head list;

	/* 1 - control command or control command status;
	 * 2 - adsp event;
	 * 3 - adsp message;
	 * 4 - v4l2 request;
	 */
	enum msm_queut_t type;
	void             *command;
};
#else
/* this structure is used in kernel */
struct msm_queue_cmd {
	struct list_head list;
	enum msm_queue type;
	void *command;
};
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct register_address_value_pair_t {
#else
struct register_address_value_pair {
#endif
	uint16_t register_address;
	uint16_t register_value;
};

struct msm_pmem_region {
	struct hlist_node list;
#if defined(CONFIG_MACH_PITTSBURGH)
	enum msm_pmem_t type;
#else
	int type;
#endif
	void *vaddr;
	unsigned long paddr;
	unsigned long len;
	struct file *file;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;
	uint8_t  active;
};

#if defined(CONFIG_MACH_PITTSBURGH)
struct axidata_t {
#else
struct axidata {
#endif
	uint32_t bufnum1;
	uint32_t bufnum2;
	struct msm_pmem_region *region;
};

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
int32_t mt9d112_probe_init(void *, void *);
int32_t mt9t013_probe_init(void *, void *);
int32_t mt9p012_probe_init(void *, void *);
int32_t s5k3e2fx_probe_init(void *, void *);
#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
int32_t mot_camera_probe_init(void *, void *);
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
#if defined(CONFIG_MACH_PITTSBURGH)
int32_t flash_set_led_state(enum msm_camera_led_state_t led_state);
#else
int msm_camera_flash_set_led_state(unsigned led_state);
#endif
#else
#ifdef CONFIG_MSM_CAMERA_FLASH
int msm_camera_flash_set_led_state(unsigned led_state);
#else
static inline int msm_camera_flash_set_led_state(unsigned led_state)
{
	return -ENOTSUPP;
}
#endif
#endif

/* Below functions are added for V4L2 kernel APIs */
#if defined(CONFIG_MACH_PITTSBURGH)
struct msm_driver {
	struct msm_device_t *vmsm;
	long (*init)(struct msm_device_t *);
	long (*ctrl)(struct msm_ctrl_cmd_t *,
		struct msm_device_t *);

	long (*reg_pmem)(struct msm_pmem_info_t *,
		struct msm_device_t *);

	long (*get_frame) (struct msm_frame_t *,
		struct msm_device_t *);

	long (*put_frame) (struct msm_frame_t *,
		struct msm_device_t *msm);

	long (*get_pict) (struct msm_ctrl_cmd_t *,
		struct msm_device_t *msm);

	unsigned int (*drv_poll) (struct file *, struct poll_table_struct *,
		struct msm_device_t *msm);
};

unsigned int msm_poll(struct file *, struct poll_table_struct *);

long msm_register(struct msm_driver *,
	const char *);
long msm_unregister(struct msm_driver *,
	const char *);

#elif defined(CONFIG_MACH_MOT)
struct msm_driver {
	struct msm_device_t *vmsm;
	long (*init)(struct msm_device_t *);
	long (*ctrl)(struct msm_ctrl_cmd *,
		struct msm_device_t *);

	long (*reg_pmem)(struct msm_pmem_info *,
		struct msm_device_t *);

	long (*get_frame) (struct msm_frame *,
		struct msm_device_t *);

	long (*put_frame) (struct msm_frame *,
		struct msm_device_t *msm);

	long (*get_pict) (struct msm_ctrl_cmd *,
		struct msm_device_t *msm);

	unsigned int (*drv_poll) (struct file *, struct poll_table_struct *,
		struct msm_device_t *msm);
};

unsigned int msm_poll(struct file *, struct poll_table_struct *);

long msm_register(struct msm_driver *,
	const char *);
long msm_unregister(struct msm_driver *,
	const char *);

#else
struct msm_v4l2_driver {
	struct msm_sync *sync;
	int (*open)(struct msm_sync *, const char *apps_id);
	int (*release)(struct msm_sync *);
	int (*ctrl)(struct msm_sync *, struct msm_ctrl_cmd *);
	int (*reg_pmem)(struct msm_sync *, struct msm_pmem_info *);
	int (*get_frame) (struct msm_sync *, struct msm_frame *);
	int (*put_frame) (struct msm_sync *, struct msm_frame *);
	int (*get_pict) (struct msm_sync *, struct msm_ctrl_cmd *);
	unsigned int (*drv_poll) (struct msm_sync *, struct file *,
				struct poll_table_struct *);
};

int msm_v4l2_register(struct msm_v4l2_driver *);
int msm_v4l2_unregister(struct msm_v4l2_driver *);
#endif

void msm_camvfe_init(void);
int msm_camvfe_check(void *);
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
void msm_camvfe_fn_init(struct msm_camvfe_fn_t *);
int msm_camera_drv_start(struct platform_device *);
int msm_camera_drv_remove(struct platform_device *);
#else
void msm_camvfe_fn_init(struct msm_camvfe_fn *, void *);
int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
					struct msm_sensor_ctrl *));
#endif

enum msm_camio_clk_type {
	CAMIO_VFE_MDC_CLK,
	CAMIO_MDC_CLK,
	CAMIO_VFE_CLK,
	CAMIO_VFE_AXI_CLK,

	CAMIO_MAX_CLK
};

enum msm_camio_clk_src_type {
	MSM_CAMIO_CLK_SRC_INTERNAL,
	MSM_CAMIO_CLK_SRC_EXTERNAL,
	MSM_CAMIO_CLK_SRC_MAX
};

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
enum msm_s_test_mode_t {
#else
enum msm_s_test_mode {
#endif
	S_TEST_OFF,
	S_TEST_1,
	S_TEST_2,
	S_TEST_3
};

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
enum msm_s_resolution_t {
#else
enum msm_s_resolution {
#endif
	S_QTR_SIZE,
	S_FULL_SIZE,
	S_INVALID_SIZE
};

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
enum msm_s_reg_update_t {
#else
enum msm_s_reg_update {
#endif
	/* Sensor egisters that need to be updated during initialization */
	S_REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	S_UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	S_UPDATE_ALL,
	/* Not valid update */
	S_UPDATE_INVALID
};

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
enum msm_s_setting_t {
#else
enum msm_s_setting {
#endif
	S_RES_PREVIEW,
	S_RES_CAPTURE
};

int msm_camio_enable(struct platform_device *dev);

int  msm_camio_clk_enable(enum msm_camio_clk_type clk);
int  msm_camio_clk_disable(enum msm_camio_clk_type clk);
int  msm_camio_clk_config(uint32_t freq);
void msm_camio_clk_rate_set(int rate);
void msm_camio_clk_axi_rate_set(int rate);

void msm_camio_camif_pad_reg_reset(void);
void msm_camio_camif_pad_reg_reset_2(void);

void msm_camio_vfe_blk_reset(void);

void msm_camio_clk_sel(enum msm_camio_clk_src_type);
void msm_camio_disable(struct platform_device *);
int msm_camio_probe_on(struct platform_device *);
int msm_camio_probe_off(struct platform_device *);
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
int request_axi_qos(void);
void release_axi_qos(void);
#endif
#endif
