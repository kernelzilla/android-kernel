/*
 * linux/drivers/video/omap2/dss/manager.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "MANAGER"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/display.h>

#include "dss.h"

static int num_managers;
static struct list_head manager_list;

static ssize_t manager_name_show(struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", mgr->name);
}

static ssize_t manager_display_show(struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			mgr->device ? mgr->device->name : "<none>");
}

static ssize_t manager_display_store(struct omap_overlay_manager *mgr, const char *buf, size_t size)
{
	int r = 0;
	size_t len = size;
	struct omap_dss_device *dssdev = NULL;

	int match(struct omap_dss_device *dssdev, void *data)
	{
		const char *str = data;
		return strcmp(dssdev->name, str) == 0;
	}

	if (buf[size-1] == '\n')
		--len;

	if (len > 0) {
		char name[64];
		int n;

		n = min(len, sizeof(name) - 1);
		strncpy(name, buf, n);
		name[n - 1] = 0;

		dssdev = omap_dss_find_device(name, match);
	}

	if (len > 0 && dssdev == NULL)
		return -EINVAL;

	if (dssdev)
		DSSDBG("display %s found\n", dssdev->name);

	if (mgr->device) {
		r = mgr->unset_device(mgr);
		if (r) {
			DSSERR("failed to unset display\n");
			goto put_device;
		}
	}

	if (dssdev) {
		r = mgr->set_device(mgr, dssdev);
		if (r) {
			DSSERR("failed to set manager\n");
			goto put_device;
		}

		r = mgr->apply(mgr);
		if (r) {
			DSSERR("failed to apply dispc config\n");
			goto put_device;
		}
	}

put_device:
	if (dssdev)
		omap_dss_put_device(dssdev);

	return r ? r : size;
}

static ssize_t manager_default_color_show(struct omap_overlay_manager *mgr,
					  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d",
			mgr->get_default_color(mgr));
}

static ssize_t manager_default_color_store(struct omap_overlay_manager *mgr,
					   const char *buf, size_t size)
{
	u32 default_color;

	if (sscanf(buf, "%d", &default_color) != 1)
		return -EINVAL;
	dispc_set_default_color(mgr->id, default_color);

	return size;
}

static const char *color_key_type_str[] = {
	"gfx-destination",
	"video-source",
};

static ssize_t manager_color_key_type_show(struct omap_overlay_manager *mgr,
					   char *buf)
{
	enum omap_dss_color_key_type key_type;

	mgr->get_trans_key_type_and_value(mgr, &key_type, NULL);
	BUG_ON(key_type >= ARRAY_SIZE(color_key_type_str));

	return snprintf(buf, PAGE_SIZE, "%s\n", color_key_type_str[key_type]);
}

static ssize_t manager_color_key_type_store(struct omap_overlay_manager *mgr,
					    const char *buf, size_t size)
{
	enum omap_dss_color_key_type key_type;
	u32 key_value;

	for (key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
			key_type < ARRAY_SIZE(color_key_type_str); key_type++) {
		if (sysfs_streq(buf, color_key_type_str[key_type]))
			break;
	}
	if (key_type == ARRAY_SIZE(color_key_type_str))
		return -EINVAL;
	/* OMAP does not support destination color key and alpha blending
	 * simultaneously.  So if alpha blending and color keying both are
	 * enabled then refrain from setting the color key type to
	 * gfx-destination
	 */
	if (!key_type) {
		bool color_key_enabled;
		bool alpha_blending_enabled;
		color_key_enabled = mgr->get_trans_key_status(mgr);
		alpha_blending_enabled = mgr->get_alpha_blending_status(mgr);
		if (color_key_enabled && alpha_blending_enabled)
			return -EINVAL;
	}

	mgr->get_trans_key_type_and_value(mgr, NULL, &key_value);
	mgr->set_trans_key_type_and_value(mgr, key_type, key_value);

	return size;
}

static ssize_t manager_color_key_value_show(struct omap_overlay_manager *mgr,
					    char *buf)
{
	u32 key_value;

	 mgr->get_trans_key_type_and_value(mgr, NULL, &key_value);

	return snprintf(buf, PAGE_SIZE, "%d\n", key_value);
}

static ssize_t manager_color_key_value_store(struct omap_overlay_manager *mgr,
					     const char *buf, size_t size)
{
	enum omap_dss_color_key_type key_type;
	u32 key_value;

	if (sscanf(buf, "%d", &key_value) != 1)
		return -EINVAL;
	mgr->get_trans_key_type_and_value(mgr, &key_type, NULL);
	mgr->set_trans_key_type_and_value(mgr, key_type, key_value);

	return size;
}

static ssize_t manager_color_key_enabled_show(struct omap_overlay_manager *mgr,
					      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
	mgr->get_trans_key_status(mgr));
}

static ssize_t manager_color_key_enabled_store(struct omap_overlay_manager *mgr,
					       const char *buf, size_t size)
{
	int enable;

	if (sscanf(buf, "%d", &enable) != 1)
		return -EINVAL;

	/* OMAP does not support destination color keying and
	 * alpha blending simultaneously.  so if alpha blending
	 * is enabled refrain from enabling destination color
	 * keying.
	 */
	if (enable) {
		bool enabled;
		enabled = mgr->get_alpha_blending_status(mgr);
		if (enabled) {
			enum omap_dss_color_key_type key_type;
			mgr->get_trans_key_type_and_value(mgr,
					&key_type, NULL);
			if (!key_type)
				return -EINVAL;
		}

	}
	mgr->enable_trans_key(mgr, enable);

	return size;
}
static ssize_t manager_alpha_blending_enabled_show(
		struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			mgr->get_alpha_blending_status(mgr));
}
static ssize_t manager_alpha_blending_enabled_store(
		struct omap_overlay_manager *mgr,
		const char *buf, size_t size)
{
	int enable;
	if (sscanf(buf, "%d", &enable) != 1)
		return -EINVAL;
	/* OMAP does not support destination color keying and
	 * alpha blending simultaneously.  so if destination
	 * color keying is enabled refrain from enabling
	 * alpha blending
	 */
	if (enable) {
		bool enabled;
		enabled = mgr->get_trans_key_status(mgr);
		if (enabled) {
			enum omap_dss_color_key_type key_type;
			mgr->get_trans_key_type_and_value(mgr, &key_type, NULL);
			if (!key_type)
				return -EINVAL;

		}

	}
	mgr->enable_alpha_blending(mgr, enable);
	return size;
}


struct manager_attribute {
	struct attribute attr;
	ssize_t (*show)(struct omap_overlay_manager *, char *);
	ssize_t	(*store)(struct omap_overlay_manager *, const char *, size_t);
};

#define MANAGER_ATTR(_name, _mode, _show, _store) \
	struct manager_attribute manager_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static MANAGER_ATTR(name, S_IRUGO, manager_name_show, NULL);
static MANAGER_ATTR(display, S_IRUGO|S_IWUSR,
		manager_display_show, manager_display_store);
static MANAGER_ATTR(default_color, S_IRUGO|S_IWUSR,
		manager_default_color_show, manager_default_color_store);
static MANAGER_ATTR(color_key_type, S_IRUGO|S_IWUSR,
		manager_color_key_type_show, manager_color_key_type_store);
static MANAGER_ATTR(color_key_value, S_IRUGO|S_IWUSR,
		manager_color_key_value_show, manager_color_key_value_store);
static MANAGER_ATTR(color_key_enabled, S_IRUGO|S_IWUSR,
		manager_color_key_enabled_show, manager_color_key_enabled_store);
static MANAGER_ATTR(alpha_blending_enabled, S_IRUGO|S_IWUSR,
		manager_alpha_blending_enabled_show,
		manager_alpha_blending_enabled_store);


static struct attribute *manager_sysfs_attrs[] = {
	&manager_attr_name.attr,
	&manager_attr_display.attr,
	&manager_attr_default_color.attr,
	&manager_attr_color_key_type.attr,
	&manager_attr_color_key_value.attr,
	&manager_attr_color_key_enabled.attr,
	&manager_attr_alpha_blending_enabled.attr,
	NULL
};

static ssize_t manager_attr_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct omap_overlay_manager *manager;
	struct manager_attribute *manager_attr;

	manager = container_of(kobj, struct omap_overlay_manager, kobj);
	manager_attr = container_of(attr, struct manager_attribute, attr);

	if (!manager_attr->show)
		return -ENOENT;

	return manager_attr->show(manager, buf);
}

static ssize_t manager_attr_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t size)
{
	struct omap_overlay_manager *manager;
	struct manager_attribute *manager_attr;

	manager = container_of(kobj, struct omap_overlay_manager, kobj);
	manager_attr = container_of(attr, struct manager_attribute, attr);

	if (!manager_attr->store)
		return -ENOENT;

	return manager_attr->store(manager, buf, size);
}

static struct sysfs_ops manager_sysfs_ops = {
	.show = manager_attr_show,
	.store = manager_attr_store,
};

static struct kobj_type manager_ktype = {
	.sysfs_ops = &manager_sysfs_ops,
	.default_attrs = manager_sysfs_attrs,
};

static int omap_dss_set_device(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev)
{
	int i;
	int r;

	if (dssdev->manager) {
		DSSERR("display '%s' already has a manager '%s'\n",
			       dssdev->name, dssdev->manager->name);
		return -EINVAL;
	}

	if ((mgr->supported_displays & dssdev->type) == 0) {
		DSSERR("display '%s' does not support manager '%s'\n",
			       dssdev->name, mgr->name);
		return -EINVAL;
	}

	for (i = 0; i < mgr->num_overlays; i++) {
		struct omap_overlay *ovl = mgr->overlays[i];

		if (ovl->manager != mgr || !ovl->info.enabled)
			continue;

		r = dss_check_overlay(ovl, dssdev);
		if (r)
			return r;
	}

	dssdev->manager = mgr;
	mgr->device = dssdev;

	return 0;
}

static int omap_dss_unset_device(struct omap_overlay_manager *mgr)
{
	if (!mgr->device) {
		DSSERR("failed to unset display, display not set.\n");
		return -EINVAL;
	}

	mgr->device->manager = NULL;
	mgr->device = NULL;

	return 0;
}


static int overlay_enabled(struct omap_overlay *ovl)
{
	return ovl->info.enabled && ovl->manager && ovl->manager->device;
}

/* We apply settings to both managers here so that we can use optimizations
 * like fifomerge. Shadow registers can be changed first and the non-shadowed
 * should be changed last, at the same time with GO */
static int omap_dss_mgr_apply(struct omap_overlay_manager *mgr)
{
	int i;
	int ret = 0;
	enum omap_dss_update_mode mode;
	struct omap_dss_device *dssdev;
	struct omap_overlay *ovl;
	bool ilace = 0;
	int outw, outh;
	int r;
	int num_planes_enabled = 0;

	DSSDBG("omap_dss_mgr_apply(%s)\n", mgr->name);

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	/* Configure normal overlay parameters and disable unused overlays */
	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
		ovl = omap_dss_get_overlay(i);

		if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
			continue;

		if (!overlay_enabled(ovl)) {
			dispc_enable_plane(ovl->id, 0);
			continue;
		}

		dssdev = ovl->manager->device;

		if (dss_check_overlay(ovl, dssdev)) {
			dispc_enable_plane(ovl->id, 0);
			continue;
		}

		++num_planes_enabled;

		/* On a manual update display, in manual update mode, update()
		 * handles configuring planes */
		mode = OMAP_DSS_UPDATE_AUTO;
		if (dssdev->get_update_mode)
			mode = dssdev->get_update_mode(dssdev);

		if (dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
				mode != OMAP_DSS_UPDATE_AUTO)
			continue;

		if (dssdev->type == OMAP_DISPLAY_TYPE_VENC)
			ilace = 1;

		if (ovl->info.out_width == 0)
			outw = ovl->info.width;
		else
			outw = ovl->info.out_width;

		if (ovl->info.out_height == 0)
			outh = ovl->info.height;
		else
			outh = ovl->info.out_height;

		r = dispc_setup_plane(ovl->id, ovl->manager->id,
				ovl->info.paddr,
				ovl->info.screen_width,
				ovl->info.pos_x,
				ovl->info.pos_y,
				ovl->info.width,
				ovl->info.height,
				outw,
				outh,
				ovl->info.color_mode,
				ilace,
				ovl->info.rotation_type,
				ovl->info.rotation,
				ovl->info.mirror,
				ovl->info.global_alpha);

		if (r) {
			DSSERR("dispc_setup_plane failed for ovl %d\n",
					ovl->id);
			dispc_enable_plane(ovl->id, 0);
			continue;
		}

		if (dss_use_replication(dssdev, ovl->info.color_mode))
			dispc_enable_replication(ovl->id, true);
		else
			dispc_enable_replication(ovl->id, false);

		dispc_enable_plane(ovl->id, 1);
	}

	/* Enable fifo merge if possible */
	dispc_enable_fifomerge(num_planes_enabled == 1);

	/* Go through overlays again. This time we configure fifos.  We have to
	 * do this after enabling/disabling fifomerge so that we have correct
	 * knowledge of fifo sizes */
	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
		ovl = omap_dss_get_overlay(i);

		if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
			continue;

		if (!overlay_enabled(ovl)) {
			continue;
		}

		ovl->manager->device->configure_overlay(ovl);
	}

	/* Issue GO for managers */
	list_for_each_entry(mgr, &manager_list, list) {
		if (!(mgr->caps & OMAP_DSS_OVL_MGR_CAP_DISPC))
			continue;

		dssdev = mgr->device;

		if (!dssdev)
			continue;

		/* We don't need GO with manual update display. LCD iface will
		 * always be turned off after frame, and new settings will
		 * be taken in to use at next update */
		if (dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE)
			continue;

		dispc_go(mgr->id);
	}

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	return ret;
}

static void omap_dss_mgr_set_def_color(struct omap_overlay_manager *mgr,
		u32 color)
{
	dispc_set_default_color(mgr->id, color);
}

static void omap_dss_mgr_set_trans_key_type_and_value(
		struct omap_overlay_manager *mgr,
		enum omap_dss_color_key_type type,
		u32 trans_key)
{
	dispc_set_trans_key(mgr->id, type, trans_key);
}
static void omap_dss_mgr_get_trans_key_type_and_value(
		struct omap_overlay_manager *mgr,
		enum omap_dss_color_key_type *type,
		u32 *trans_key)
{
	dispc_get_trans_key(mgr->id, type, trans_key);
}

static void omap_dss_mgr_enable_trans_key(struct omap_overlay_manager *mgr,
		bool enable)
{
	dispc_enable_trans_key(mgr->id, enable);
}
static void omap_dss_mgr_enable_alpha_blending(struct omap_overlay_manager *mgr,
		bool enable)
{
	dispc_enable_alpha_blending(mgr->id, enable);
}
static bool omap_dss_mgr_get_alpha_blending_status(
		struct omap_overlay_manager *mgr)
{
	return dispc_alpha_blending_enabled(mgr->id);
}
static u32 omap_dss_mgr_get_default_color(struct omap_overlay_manager *mgr)
{
	return dispc_get_default_color(mgr->id);
}
static bool omap_dss_mgr_get_trans_key_status(struct omap_overlay_manager *mgr)
{
	return dispc_trans_key_enabled(mgr->id);
}

static void omap_dss_add_overlay_manager(struct omap_overlay_manager *manager)
{
	++num_managers;
	list_add_tail(&manager->list, &manager_list);
}

int dss_init_overlay_managers(struct platform_device *pdev)
{
	int i, r;

	INIT_LIST_HEAD(&manager_list);

	num_managers = 0;

	for (i = 0; i < 2; ++i) {
		struct omap_overlay_manager *mgr;
		mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);

		BUG_ON(mgr == NULL);

		switch (i) {
		case 0:
			mgr->name = "lcd";
			mgr->id = OMAP_DSS_CHANNEL_LCD;
			mgr->supported_displays =
				OMAP_DISPLAY_TYPE_DPI | OMAP_DISPLAY_TYPE_DBI |
				OMAP_DISPLAY_TYPE_SDI | OMAP_DISPLAY_TYPE_DSI;
			break;
		case 1:
			mgr->name = "tv";
			mgr->id = OMAP_DSS_CHANNEL_DIGIT;
			mgr->supported_displays = OMAP_DISPLAY_TYPE_VENC;
			break;
		}

		mgr->set_device = &omap_dss_set_device;
		mgr->unset_device = &omap_dss_unset_device;
		mgr->apply = &omap_dss_mgr_apply;
		mgr->set_default_color = &omap_dss_mgr_set_def_color;
		mgr->set_trans_key_type_and_value =
			&omap_dss_mgr_set_trans_key_type_and_value;
		mgr->get_trans_key_type_and_value =
			&omap_dss_mgr_get_trans_key_type_and_value;
		mgr->enable_trans_key = &omap_dss_mgr_enable_trans_key;
		mgr->get_trans_key_status = &omap_dss_mgr_get_trans_key_status;
		mgr->enable_alpha_blending =
			&omap_dss_mgr_enable_alpha_blending;
		mgr->get_alpha_blending_status =
			omap_dss_mgr_get_alpha_blending_status;
		mgr->get_default_color = &omap_dss_mgr_get_default_color;
		mgr->caps = OMAP_DSS_OVL_MGR_CAP_DISPC;

		dss_overlay_setup_dispc_manager(mgr);

		omap_dss_add_overlay_manager(mgr);

		r = kobject_init_and_add(&mgr->kobj, &manager_ktype,
				&pdev->dev.kobj, "manager%d", i);

		if (r) {
			DSSERR("failed to create sysfs file\n");
			continue;
		}
	}

	return 0;
}

void dss_uninit_overlay_managers(struct platform_device *pdev)
{
	struct omap_overlay_manager *mgr;

	while (!list_empty(&manager_list)) {
		mgr = list_first_entry(&manager_list,
				struct omap_overlay_manager, list);
		list_del(&mgr->list);
		kobject_del(&mgr->kobj);
		kobject_put(&mgr->kobj);
		kfree(mgr);
	}

	num_managers = 0;
}

int omap_dss_get_num_overlay_managers(void)
{
	return num_managers;
}
EXPORT_SYMBOL(omap_dss_get_num_overlay_managers);

struct omap_overlay_manager *omap_dss_get_overlay_manager(int num)
{
	int i = 0;
	struct omap_overlay_manager *mgr;

	list_for_each_entry(mgr, &manager_list, list) {
		if (i++ == num)
			return mgr;
	}

	return NULL;
}
EXPORT_SYMBOL(omap_dss_get_overlay_manager);

#ifdef L4_EXAMPLE
static int ovl_mgr_apply_l4(struct omap_overlay_manager *mgr)
{
	DSSDBG("omap_dss_mgr_apply_l4(%s)\n", mgr->name);

	return 0;
}
#endif

