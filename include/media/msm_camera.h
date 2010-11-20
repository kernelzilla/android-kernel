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
#ifndef __LINUX_MSM_CAMERA_H
#define __LINUX_MSM_CAMERA_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#endif
#include <linux/msm_adsp.h>


#define MSM_CAM_IOCTL_MAGIC 'm'

#if !defined(CONFIG_MACH_PITTSBURGH) 

/*
 * This is the MSM Camera API from Donut
 */
#define MSM_CAM_IOCTL_GET_SENSOR_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 1, struct msm_camsensor_info *)

#define MSM_CAM_IOCTL_REGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 2, struct msm_pmem_info *)

#define MSM_CAM_IOCTL_UNREGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAM_IOCTL_CTRL_COMMAND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 4, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_CONFIG_VFE  \
	_IOW(MSM_CAM_IOCTL_MAGIC, 5, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_STATS \
	_IOR(MSM_CAM_IOCTL_MAGIC, 6, struct msm_camera_stats_event_ctrl *)

#define MSM_CAM_IOCTL_GETFRAME \
	_IOR(MSM_CAM_IOCTL_MAGIC, 7, struct msm_camera_get_frame *)

#define MSM_CAM_IOCTL_ENABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 8, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_CTRL_CMD_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 9, struct camera_cmd *)

#define MSM_CAM_IOCTL_CONFIG_CMD \
	_IOW(MSM_CAM_IOCTL_MAGIC, 10, struct camera_cmd *)

#define MSM_CAM_IOCTL_DISABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 11, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_PAD_REG_RESET2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 12, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_VFE_APPS_RESET \
	_IOW(MSM_CAM_IOCTL_MAGIC, 13, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 14, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_STATS_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 15, struct msm_stats_buf *)

#define MSM_CAM_IOCTL_AXI_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 16, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_PICTURE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 17, struct msm_camera_ctrl_cmd *)

#define MSM_CAM_IOCTL_SET_CROP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 18, struct crop_info *)

#define MSM_CAM_IOCTL_PICT_PP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 19, uint8_t *)

#define MSM_CAM_IOCTL_PICT_PP_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 20, struct msm_snapshot_pp_status *)

#define MSM_CAM_IOCTL_SENSOR_IO_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 21, struct sensor_cfg_data *)

#define MSM_CAMERA_LED_OFF  0
#define MSM_CAMERA_LED_LOW  1
#define MSM_CAMERA_LED_HIGH 2

#define MSM_CAM_IOCTL_FLASH_LED_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 22, unsigned *)

#define MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME \
	_IO(MSM_CAM_IOCTL_MAGIC, 23)

#define MSM_CAM_IOCTL_CTRL_COMMAND_2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_AF_CTRL \
	_IOR(MSM_CAM_IOCTL_MAGIC, 25, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_AF_CTRL_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 26, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_DISABLE_LENS_MOVE \
         _IOW(MSM_CAM_IOCTL_MAGIC, 100, uint8_t*)

#define MSM_CAM_IOCTL_ENABLE_LENS_MOVE \
         _IOW(MSM_CAM_IOCTL_MAGIC, 101, uint8_t*)

#define MAX_SENSOR_NUM  3
#define MAX_SENSOR_NAME 32

#define PP_SNAP  0x01
#define PP_RAW_SNAP ((0x01)<<1)
#define PP_PREV  ((0x01)<<2)

#define MSM_CAM_CTRL_CMD_DONE  0
#define MSM_CAM_SENSOR_VFE_CMD 1

/*****************************************************
 *  structure
 *****************************************************/

/* define five type of structures for userspace <==> kernel
 * space communication:
 * command 1 - 2 are from userspace ==> kernel
 * command 3 - 4 are from kernel ==> userspace
 *
 * 1. control command: control command(from control thread),
 *                     control status (from config thread);
 */
struct msm_ctrl_cmd {
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
	uint32_t timeout_ms;
	int resp_fd; /* FIXME: to be used by the kernel, pass-through for now */
};

struct msm_vfe_evt_msg {
	unsigned short type;	/* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len;	/* size in, number of bytes out */
	void *data;
};

#define MSM_CAM_RESP_CTRL         0
#define MSM_CAM_RESP_STAT_EVT_MSG 1
#define MSM_CAM_RESP_V4L2         2
#define MSM_CAM_RESP_MAX          3

/* this one is used to send ctrl/status up to config thread */
struct msm_stats_event_ctrl {
	/* 0 - ctrl_cmd from control thread,
	 * 1 - stats/event kernel,
	 * 2 - V4L control or read request */
	int resptype;
	int timeout_ms;
	struct msm_ctrl_cmd ctrl_cmd;
	/* struct  vfe_event_t  stats_event; */
	struct msm_vfe_evt_msg stats_event;
};

/* 2. config command: config command(from config thread); */
struct msm_camera_cfg_cmd {
	/* what to config:
	 * 1 - sensor config, 2 - vfe config */
	uint16_t cfg_type;

	/* sensor config type */
	uint16_t cmd_type;
	uint16_t queue;
	uint16_t length;
	void *value;
};

#define CMD_GENERAL			0
#define CMD_AXI_CFG_OUT1		1
#define CMD_AXI_CFG_SNAP_O1_AND_O2	2
#define CMD_AXI_CFG_OUT2		3
#define CMD_PICT_T_AXI_CFG		4
#define CMD_PICT_M_AXI_CFG		5
#define CMD_RAW_PICT_AXI_CFG		6
#define CMD_STATS_AXI_CFG		7
#define CMD_STATS_AF_AXI_CFG		8
#define CMD_FRAME_BUF_RELEASE		9
#define CMD_PREV_BUF_CFG		10
#define CMD_SNAP_BUF_RELEASE		11
#define CMD_SNAP_BUF_CFG		12
#define CMD_STATS_DISABLE		13
#define CMD_STATS_AEC_AWB_ENABLE	14
#define CMD_STATS_AF_ENABLE		15
#define CMD_STATS_BUF_RELEASE		16
#define CMD_STATS_AF_BUF_RELEASE	17
#define CMD_STATS_ENABLE        18
#define UPDATE_STATS_INVALID		19

/* vfe config command: config command(from config thread)*/
struct msm_vfe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

#define MAX_CAMERA_ENABLE_NAME_LEN 32
struct camera_enable_cmd {
	char name[MAX_CAMERA_ENABLE_NAME_LEN];
};

#define MSM_PMEM_OUTPUT1		0
#define MSM_PMEM_OUTPUT2		1
#define MSM_PMEM_OUTPUT1_OUTPUT2	2
#define MSM_PMEM_THUMBNAIL		3
#define MSM_PMEM_MAINIMG		4
#define MSM_PMEM_RAW_MAINIMG		5
#define MSM_PMEM_AEC_AWB		6
#define MSM_PMEM_AF			7
#define MSM_PMEM_MAX			8

#define FRAME_PREVIEW_OUTPUT1		0
#define FRAME_PREVIEW_OUTPUT2		1
#define FRAME_SNAPSHOT			2
#define FRAME_THUMBAIL			3
#define FRAME_RAW_SNAPSHOT		4
#define FRAME_MAX			5

struct msm_pmem_info {
	int type;
	int fd;
	void *vaddr;
	uint32_t y_off;
	uint32_t cbcr_off;
	uint8_t active;
};

struct outputCfg {
	uint32_t height;
	uint32_t width;

	uint32_t window_height_firstline;
	uint32_t window_height_lastline;
};

#define OUTPUT_1	0
#define OUTPUT_2	1
#define OUTPUT_1_AND_2	2
#define CAMIF_TO_AXI_VIA_OUTPUT_2		3
#define OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2	4
#define OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1	5
#define LAST_AXI_OUTPUT_MODE_ENUM = OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 6

#define MSM_FRAME_PREV_1	0
#define MSM_FRAME_PREV_2	1
#define MSM_FRAME_ENC		2

struct msm_frame {
	int path;
	unsigned long buffer;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;

	void *cropinfo;
	int croplen;
};

#define STAT_AEAW	0
#define STAT_AF		1
#define STAT_MAX	2

struct msm_stats_buf {
	int type;
	unsigned long buffer;
	int fd;
};

#define MSM_V4L2_VID_CAP_TYPE	0
#define MSM_V4L2_STREAM_ON	1
#define MSM_V4L2_STREAM_OFF	2
#define MSM_V4L2_SNAPSHOT	3
#define MSM_V4L2_QUERY_CTRL	4
#define MSM_V4L2_GET_CTRL	5
#define MSM_V4L2_SET_CTRL	6
#define MSM_V4L2_QUERY		7
#define MSM_V4L2_GET_CROP	8
#define MSM_V4L2_SET_CROP	9
#define MSM_V4L2_MAX		10
#define V4L2_CAMERA_EXIT 	43

struct crop_info
{
	void *info;
	int len;
};

struct msm_postproc {
	int ftnum;
	struct msm_frame fthumnail;
	int fmnum;
	struct msm_frame fmain;
};

struct msm_snapshot_pp_status
{
	void *status;
};

#define CFG_SET_MODE				0
#define CFG_SET_EFFECT				1
#define CFG_START					2
#define CFG_PWR_UP					3
#define CFG_PWR_DOWN				4
#define CFG_WRITE_EXPOSURE_GAIN		5
#define CFG_SET_DEFAULT_FOCUS		6
#define CFG_MOVE_FOCUS				7
#define CFG_REGISTER_TO_REAL_GAIN	8
#define CFG_REAL_TO_REGISTER_GAIN	9
#define CFG_SET_FPS					10
#define CFG_SET_PICT_FPS			11
#define CFG_SET_BRIGHTNESS			12
#define CFG_SET_CONTRAST			13
#define CFG_SET_ZOOM				14
#define CFG_SET_EXPOSURE_MODE		15
#define CFG_SET_WB					16
#define CFG_SET_ANTIBANDING			17
#define CFG_SET_EXP_GAIN			18
#define CFG_SET_PICT_EXP_GAIN		19
#define CFG_SET_LENS_SHADING		20
#define CFG_GET_PICT_FPS			21
#define CFG_GET_PREV_L_PF			22
#define CFG_GET_PREV_P_PL			23
#define CFG_GET_PICT_L_PF			24
#define CFG_GET_PICT_P_PL			25
#define CFG_GET_AF_MAX_STEPS		26
#define CFG_GET_PICT_MAX_EXP_LC		27
#define CFG_LOAD_TEST_PATTERN_DLI	28
#define CFG_GET_WB_GAINS			29
#define CFG_SET_AWB_DECISION		30
#define CFG_GET_EEPROM_DATA			31
#define CFG_SET_MODULE_DATA			32
#define CFG_GET_CAL_DATA_STATUS		33
#define CFG_GET_AF_DEVICE_ADDRESS	34
#define CFG_MAX						35

#define MOVE_NEAR	0
#define MOVE_FAR	1

#define SENSOR_PREVIEW_MODE		0
#define SENSOR_SNAPSHOT_MODE		1
#define SENSOR_RAW_SNAPSHOT_MODE	2

#define SENSOR_QTR_SIZE			0
#define SENSOR_FULL_SIZE		1
#define SENSOR_INVALID_SIZE		2

#ifdef CONFIG_MACH_MOT
enum camera_effect_t {
	CAMERA_EFFECT_MIN_MINUS_1,
	CAMERA_EFFECT_OFF = 1,  /* This list must match aeecamera.h */
	CAMERA_EFFECT_MONO,
	CAMERA_EFFECT_NEGATIVE,
	CAMERA_EFFECT_SOLARIZE,
	CAMERA_EFFECT_PASTEL,
	CAMERA_EFFECT_MOSAIC,
	CAMERA_EFFECT_RESIZE,
	CAMERA_EFFECT_SEPIA,
	CAMERA_EFFECT_POSTERIZE,
	CAMERA_EFFECT_WHITEBOARD,
	CAMERA_EFFECT_BLACKBOARD,
	CAMERA_EFFECT_AQUA,
	CAMERA_EFFECT_MAX_PLUS_1
};
#else
#define CAMERA_EFFECT_OFF		0
#define CAMERA_EFFECT_MONO		1
#define CAMERA_EFFECT_NEGATIVE		2
#define CAMERA_EFFECT_SOLARIZE		3
#define CAMERA_EFFECT_SEPIA		4
#define CAMERA_EFFECT_POSTERIZE		5
#define CAMERA_EFFECT_WHITEBOARD	6
#define CAMERA_EFFECT_BLACKBOARD	7
#define CAMERA_EFFECT_AQUA		8
#define CAMERA_EFFECT_MAX		9
#endif

struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
#ifdef CONFIG_MACH_MOT
	int8_t is_outdoor;
#endif
};

struct focus_cfg {
	int32_t steps;
	int dir;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};

typedef struct
{
  uint16_t lscAddr;
  uint16_t lscVal;
} lsc_data_t;

#define WB_INCANDESCENT 0
#define WB_FLORESCENT 1
#define WB_DAYLIGHT 2
#define LSC_DATA_ENTRIES 102

typedef struct
{
  uint8_t valid_data;
  uint8_t lsc_count;
  lsc_data_t lsc[LSC_DATA_ENTRIES*3];
  uint16_t Rgain;
  uint16_t Ggain;
  uint16_t Bgain;
} module_data_t;

struct wb_gains_cfg {
  uint32_t g_gain;
  uint32_t r_gain;
  uint32_t b_gain;
};

#define MSM_LED_OFF     0
#define MSM_LED_LOW     1
#define MSM_LED_HIGH    2

struct sensor_cfg_data {
	int cfgtype;
	int mode;
	int rs;
	uint8_t max_steps;

	union {
		int8_t effect;
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
		int8_t awb_decision;
		uint8_t *eeprom_data_ptr;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg exp_gain;
		struct focus_cfg focus;
		struct fps_cfg fps;
		struct wb_gains_cfg wb_gains;
		module_data_t module_data;
		uint8_t af_device_address;
	} cfg;
};

#define GET_NAME			0
#define GET_PREVIEW_LINE_PER_FRAME	1
#define GET_PREVIEW_PIXELS_PER_LINE	2
#define GET_SNAPSHOT_LINE_PER_FRAME	3
#define GET_SNAPSHOT_PIXELS_PER_LINE	4
#define GET_SNAPSHOT_FPS		5
#define GET_SNAPSHOT_MAX_EP_LINE_CNT	6

struct msm_camsensor_info {
	char name[MAX_SENSOR_NAME];
	uint8_t flash_enabled;
	int8_t total_steps;
};

#else   /* defined(CONFIG_MACH_PITTSBURGH) */
/*
 * This is the MSM Camera API from Cupcake.
 */

#define MSM_CAM_IOCTL_GET_SENSOR_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 1, struct msm_camsensor_info_t *)

#define MSM_CAM_IOCTL_REGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 2, struct msm_pmem_info_t *)

#define MSM_CAM_IOCTL_UNREGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAM_IOCTL_CTRL_COMMAND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 4, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_CONFIG_VFE  \
	_IOW(MSM_CAM_IOCTL_MAGIC, 5, struct msm_camera_vfe_cfg_cmd_t *)

#define MSM_CAM_IOCTL_GET_STATS \
	_IOR(MSM_CAM_IOCTL_MAGIC, 6, struct msm_camera_stats_event_ctrl_t *)

#define MSM_CAM_IOCTL_GETFRAME \
	_IOR(MSM_CAM_IOCTL_MAGIC, 7, struct msm_camera_get_frame_t *)

#define MSM_CAM_IOCTL_ENABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 8, struct camera_enable_cmd_t *)

#define MSM_CAM_IOCTL_CTRL_CMD_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 9, struct camera_cmd_t *)

#define MSM_CAM_IOCTL_CONFIG_CMD \
	_IOW(MSM_CAM_IOCTL_MAGIC, 10, struct camera_cmd_t *)

#define MSM_CAM_IOCTL_DISABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 11, struct camera_enable_cmd_t *)

#define MSM_CAM_IOCTL_PAD_REG_RESET2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 12, struct camera_enable_cmd_t *)

#define MSM_CAM_IOCTL_VFE_APPS_RESET \
	_IOW(MSM_CAM_IOCTL_MAGIC, 13, struct camera_enable_cmd_t *)

#define MSM_CAM_IOCTL_RELEASE_FRAMEE_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 14, struct camera_enable_cmd_t *)

#define MSM_CAM_IOCTL_RELEASE_STATS_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 15, struct msm_stats_buf_t *)

#define MSM_CAM_IOCTL_AXI_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 16, struct msm_camera_vfe_cfg_cmd_t *)

#define MSM_CAM_IOCTL_GET_PICTURE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 17, struct msm_camera_ctrl_cmd_t *)

#define MSM_CAM_IOCTL_SET_CROP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 18, struct crop_info_t *)

#define MSM_CAM_IOCTL_PICT_PP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 19, uint8_t *)

#define MSM_CAM_IOCTL_PICT_PP_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 20, struct msm_snapshot_pp_status_t *)

#define MSM_CAM_IOCTL_SENSOR_IO_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 21, struct sensor_cfg_data_t *)

#define MSM_CAMERA_LED_OFF  0
#define MSM_CAMERA_LED_LOW  1
#define MSM_CAMERA_LED_HIGH 2

#define MSM_CAM_IOCTL_FLASH_LED_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 22, enum msm_camera_led_state_t *)

#define MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME \
	_IO(MSM_CAM_IOCTL_MAGIC, 23)

#define MSM_CAM_IOCTL_CTRL_COMMAND_2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_AF_CTRL \
	_IOR(MSM_CAM_IOCTL_MAGIC, 23, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_AF_CTRL_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmt_t *)

#define MAX_SENSOR_NUM  3
#define MAX_SENSOR_NAME 32

#define PP_SNAP  0x01
#define PP_RAW_SNAP ((0x01)<<1)
#define PP_PREV  ((0x01)<<2)

enum msm_camera_update_t {
	MSM_CAM_CTRL_CMD_DONE,
	MSM_CAM_SENSOR_VFE_CMD,
};

/*****************************************************
 *  structure
 *****************************************************/

/* define five type of structures for userspace <==> kernel
 * space communication:
 * command 1 - 2 are from userspace ==> kernel
 * command 3 - 4 are from kernel ==> userspace
 *
 * 1. control command: control command(from control thread),
 *                     control status (from config thread);
 */
struct msm_ctrl_cmd_t {
	int timeout_ms;
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
};

struct msm_vfe_evt_msg_t {
	unsigned short type;	/* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len;	/* size in, number of bytes out */
	unsigned char *data;
};

enum msm_camera_resp_t {
	MSM_CAM_RESP_CTRL,
	MSM_CAM_RESP_STAT_EVT_MSG,
	MSM_CAM_RESP_V4L2,

	MSM_CAM_RESP_MAX
};

/* this one is used to send ctrl/status up to config thread */
struct msm_stats_event_ctrl {
	/* 0 - ctrl_cmd from control thread,
	 * 1 - stats/event kernel,
	 * 2 - V4L control or read request */
	enum msm_camera_resp_t resptype;
	int timeout_ms;
	struct msm_ctrl_cmd_t ctrl_cmd;
	/* struct  vfe_event_t  stats_event; */
	struct msm_vfe_evt_msg_t stats_event;
};

/* 2. config command: config command(from config thread); */
struct msm_camera_cfg_cmd_t {
	/* what to config:
	 * 1 - sensor config, 2 - vfe config */
	uint16_t cfg_type;

	/* sensor config type */
	uint16_t cmd_type;
	uint16_t queue;
	uint16_t length;
	void *value;
};

enum cfg_cmd_type_t {
	CMD_GENERAL,
	CMD_AXI_CFG_OUT1,
	CMD_AXI_CFG_SNAP_O1_AND_O2,
	CMD_AXI_CFG_OUT2,
	CMD_PICT_T_AXI_CFG,
	CMD_PICT_M_AXI_CFG,
	CMD_RAW_PICT_AXI_CFG,
	CMD_STATS_AXI_CFG,
	CMD_STATS_AF_AXI_CFG,
	CMD_FRAME_BUF_RELEASE,
	CMD_PREV_BUF_CFG,
	CMD_SNAP_BUF_RELEASE,
	CMD_SNAP_BUF_CFG,
	CMD_STATS_DISABLE,
	CMD_STATS_AEC_AWB_ENABLE,
	CMD_STATS_AF_ENABLE,
	CMD_STATS_BUF_RELEASE,
	CMD_STATS_AF_BUF_RELEASE,
	CMD_STATS_ENABLE,
	UPDATE_STATS_INVALID
};

/* vfe config command: config command(from config thread)*/
struct msm_vfe_cfg_cmd {
	enum cfg_cmd_type_t cmd_type;
	uint16_t length;
	void *value;
};

struct camera_enable_cmd_t {
	char *name;
	uint16_t length;
};

enum msm_pmem_t {
	MSM_PMEM_OUTPUT1,
	MSM_PMEM_OUTPUT2,
	MSM_PMEM_OUTPUT1_OUTPUT2,
	MSM_PMEM_THUMBAIL,
	MSM_PMEM_MAINIMG,
	MSM_PMEM_RAW_MAINIMG,
	MSM_PMEM_AEC_AWB,
	MSM_PMEM_AF,

	MSM_PMEM_MAX
};

enum msm_camera_out_frame_t {
	FRAME_PREVIEW_OUTPUT1,
	FRAME_PREVIEW_OUTPUT2,
	FRAME_SNAPSHOT,
	FRAME_THUMBAIL,
	FRAME_RAW_SNAPSHOT,
	FRAME_MAX
};

struct msm_pmem_info_t {
	enum msm_pmem_t type;
	int fd;
	void *vaddr;
	uint32_t y_off;
	uint32_t cbcr_off;
	uint8_t active;
};

struct outputCfg_t {
	uint32_t height;
	uint32_t width;

	uint32_t window_height_firstline;
	uint32_t window_height_lastline;
};

enum vfeoutput_mode_t {
	OUTPUT_1,
	OUTPUT_2,
	OUTPUT_1_AND_2,
	CAMIF_TO_AXI_VIA_OUTPUT_2,
	OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2,
	OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1,
	LAST_AXI_OUTPUT_MODE_ENUM = OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1
};

enum msm_frame_path {
	MSM_FRAME_PREV_1,
	MSM_FRAME_PREV_2,
	MSM_FRAME_ENC,
};

struct msm_frame_t {
	enum msm_frame_path path;
	unsigned long buffer;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;

	void *cropinfo;
	int croplen;
};

enum stat_type {
	STAT_AEAW,
	STAT_AF,
	STAT_MAX,
};

struct msm_stats_buf_t {
	enum stat_type type;
	unsigned long buffer;
	int fd;
};

enum msm_v4l2_ctrl_t {
	MSM_V4L2_VID_CAP_TYPE,
	MSM_V4L2_STREAM_ON,
	MSM_V4L2_STREAM_OFF,
	MSM_V4L2_SNAPSHOT,
	MSM_V4L2_QUERY_CTRL,
	MSM_V4L2_GET_CTRL,
	MSM_V4L2_SET_CTRL,
	MSM_V4L2_QUERY,
	MSM_V4L2_MAX
};

struct crop_info_t {
	void *info;
	int len;
};

struct msm_postproc_t {
	int ftnum;
	struct msm_frame_t fthumnail;
	int fmnum;
	struct msm_frame_t fmain;
};

struct msm_snapshot_pp_status_t
{
	void *status;
};

enum sensor_cfg_t {
    CFG_SET_MODE,
    CFG_SET_EFFECT,
    CFG_START,
    CFG_PWR_UP,
    CFG_PWR_DOWN,
    CFG_WRITE_EXPOSURE_GAIN,
    CFG_SET_DEFAULT_FOCUS,
    CFG_MOVE_FOCUS,
    CFG_REGISTER_TO_REAL_GAIN,
    CFG_REAL_TO_REGISTER_GAIN,
    CFG_SET_FPS,
    CFG_SET_PICT_FPS,
    CFG_SET_BRIGHTNESS,
    CFG_SET_CONTRAST,
    CFG_SET_ZOOM,
    CFG_SET_EXPOSURE_MODE,
    CFG_SET_WB,
    CFG_SET_ANTIBANDING,
    CFG_SET_EXP_GAIN,
    CFG_SET_PICT_EXP_GAIN,
    CFG_SET_LENS_SHADING,
    CFG_GET_PICT_FPS,
    CFG_GET_PREV_L_PF,
    CFG_GET_PREV_P_PL,
    CFG_GET_PICT_L_PF,
    CFG_GET_PICT_P_PL,
    CFG_GET_AF_MAX_STEPS,
    CFG_GET_PICT_MAX_EXP_LC,
    CFG_LOAD_TEST_PATTERN_DLI,
    CFG_GET_WB_GAINS,
    CFG_SET_AWB_DECISION,
    CFG_GET_EEPROM_DATA,
    CFG_SET_MODULE_DATA,
    CFG_GET_CAL_DATA_STATUS,
    CFG_MAX
};

enum sensor_move_focus_t {
  MOVE_NEAR,
  MOVE_FAR
};

enum sensor_mode_t {
	SENSOR_PREVIEW_MODE,
	SENSOR_SNAPSHOT_MODE,
	SENSOR_RAW_SNAPSHOT_MODE
};

enum sensor_resolution_t {
	SENSOR_QTR_SIZE,
	SENSOR_FULL_SIZE,
	SENSOR_INVALID_SIZE,
};

enum camera_effect_t {
	CAMERA_EFFECT_MIN_MINUS_1,
	CAMERA_EFFECT_OFF = 1,  /* This list must match aeecamera.h */
	CAMERA_EFFECT_MONO,
	CAMERA_EFFECT_NEGATIVE,
	CAMERA_EFFECT_SOLARIZE,
	CAMERA_EFFECT_PASTEL,
	CAMERA_EFFECT_MOSAIC,
	CAMERA_EFFECT_RESIZE,
	CAMERA_EFFECT_SEPIA,
	CAMERA_EFFECT_POSTERIZE,
	CAMERA_EFFECT_WHITEBOARD,
	CAMERA_EFFECT_BLACKBOARD,
	CAMERA_EFFECT_AQUA,
	CAMERA_EFFECT_MAX_PLUS_1
};

struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
};

struct focus_cfg {
	int32_t steps;
	enum sensor_move_focus_t dir;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};

enum msm_camera_led_state_t {
  MSM_LED_OFF,
  MSM_LED_LOW,
  MSM_LED_HIGH
};

typedef struct
{
  uint16_t lscAddr;
  uint16_t lscVal;
} lsc_data_t;

#define WB_INCANDESCENT 0
#define WB_FLORESCENT 1
#define WB_DAYLIGHT 2
#define LSC_DATA_ENTRIES 102

typedef struct
{
  uint8_t valid_data;
  uint8_t lsc_count;
  lsc_data_t lsc[LSC_DATA_ENTRIES*3];
  uint16_t Rgain;
  uint16_t Ggain;
  uint16_t Bgain;
}module_data_t;

struct wb_gains_cfg {
  uint32_t g_gain;
  uint32_t r_gain;
  uint32_t b_gain;
};

struct sensor_cfg_data_t {
	enum sensor_cfg_t  cfgtype;
	enum sensor_mode_t mode;
	enum sensor_resolution_t rs;
	uint8_t max_steps;

    union {
        int8_t effect;
        uint8_t lens_shading;
        uint16_t prevl_pf;
        uint16_t prevp_pl;
        uint16_t pictl_pf;
        uint16_t pictp_pl;
        uint32_t pict_max_exp_lc;
        uint16_t p_fps;
        int8_t awb_decision;
        uint8_t *eeprom_data_ptr;
        struct sensor_pict_fps gfps;
        struct exp_gain_cfg    exp_gain;
        struct focus_cfg       focus;
        struct fps_cfg	       fps;
        struct wb_gains_cfg  wb_gains;
        module_data_t module_data;
    } cfg;
};

enum sensor_get_info_t {
	GET_NAME,
	GET_PREVIEW_LINE_PER_FRAME,
	GET_PREVIEW_PIXELS_PER_LINE,
	GET_SNAPSHOT_LINE_PER_FRAME,
	GET_SNAPSHOT_PIXELS_PER_LINE,
	GET_SNAPSHOT_FPS,
	GET_SNAPSHOT_MAX_EP_LINE_CNT,
};

struct msm_camsensor_info_t {
	char name[MAX_SENSOR_NAME];
  int8_t flash_enabled;
};

#endif /* defined(CONFIG_MACH_PITTSBURGH) */
#endif /* __LINUX_MSM_CAMERA_H */
