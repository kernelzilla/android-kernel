/* arch/arm/mach-msm/include/mach/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <linux/input.h>

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
	unsigned int max_axi_khz;
	unsigned int max_vdd;
	int (*acpu_set_vdd) (int mvolts);
};

enum msm_camera_flash_t {
  MSM_CAMERA_FLASH_NONE,
  MSM_CAMERA_FLASH_LED
};

struct msm_camera_sensor_info {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int mclk;
	const char *sensor_name;
	enum msm_camera_flash_t flash_type;
	int (*sensor_probe)(void *, void *);
};

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
};

struct msm_camera_device_platform_data{
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	uint8_t snum;
	struct msm_camera_sensor_info *sinfo;
	struct msm_camera_io_ext ioext;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
};

struct clk;

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

struct msm_panel_common_pdata {
	int gpio;
	int (*backlight_level)(int level);
	int (*pmic_backlight)(int level);
	int (*panel_num)(void);
	void (*panel_config_gpio)(int);
	int *gpio_num;
};

struct lcdc_platform_data {
	int (*lcdc_gpio_config)(int on);
};

struct tvenc_platform_data {
	int (*pm_vid_en)(int on);
};

struct mddi_platform_data {
	void (*mddi_power_save)(int on);
	int (*mddi_sel_clk)(u32 *clk_rate);
};

struct msm_fb_platform_data {
	int (*detect_client)(const char *name);
};

struct msm_i2c_platform_data { 
    int clk_freq; 
//    int lock_val; 
	uint32_t *rmutex;
	int rsl_id;
	uint32_t pm_lat;
	int pri_clk;
	int pri_dat;
	int aux_clk;
	int aux_dat;
    void (*msm_i2c_config_gpio)(int iface, int config_type); 
}; 

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_map_qsd8x50_io(void);
void __init msm_map_comet_io(void);
void __init msm_init_irq(void);
void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);

struct mmc_platform_data;
int __init msm_add_sdcc(unsigned int controller,
		struct mmc_platform_data *plat);

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K)
void msm_hsusb_set_vbus_state(int online);
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

extern int msm_shared_ram_phys; /* defined in arch/arm/mach-msm/io.c */

#endif
