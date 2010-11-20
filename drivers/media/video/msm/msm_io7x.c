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

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>

#define CAMIF_CFG_RMSK 0x1fffff
#define CAM_SEL_BMSK 0x2
#define CAM_PCLK_SRC_SEL_BMSK 0x60000
#define CAM_PCLK_INVERT_BMSK 0x80000
#define CAM_PAD_REG_SW_RESET_BMSK 0x100000

#define EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define CAM_SEL_SHFT 0x1
#define CAM_PCLK_SRC_SEL_SHFT 0x11
#define CAM_PCLK_INVERT_SHFT 0x13
#define CAM_PAD_REG_SW_RESET_SHFT 0x14

#define EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define MDDI_CLK_CHICKEN_BIT_SHFT  0x7
#define APPS_RESET_OFFSET 0x00000210

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;

static struct msm_camera_io_ext camio_ext;
static struct resource *appio, *mdcio;
void __iomem *appbase, *mdcbase;

static struct msm_camera_io_ext camio_ext;
static struct resource *appio, *mdcio;
void __iomem *appbase, *mdcbase;

extern int clk_set_flags(struct clk *clk, unsigned long flags);

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = -1;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk = clk_get(NULL, "vfe_clk");
		break;

	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_enable(clk);
		rc = 0;
	}

	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = -1;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
		rc = 0;
	}

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_vfe_clk;

	if (clk != ERR_PTR(-ENOENT))
		clk_set_rate(clk, rate);
}

int msm_camio_enable(struct platform_device *pdev)
{
	int rc = 0;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_camera_device_platform_data *camdev =
		pdev->dev.platform_data;
#else
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
        struct msm_camera_device_platform_data *camdev = sinfo->pdata;
#endif


	camio_ext = camdev->ioext;

	appio = request_mem_region(camio_ext.appphy,
		camio_ext.appsz, pdev->name);
	if (!appio) {
		rc = -EBUSY;
		goto enable_fail;
	}

	appbase = ioremap(camio_ext.appphy,
		camio_ext.appsz);
	if (!appbase) {
		rc = -ENOMEM;
		goto apps_no_mem;
	}

	mdcio = request_mem_region(camio_ext.mdcphy,
		camio_ext.mdcsz, pdev->name);
	if (!mdcio) {
		rc = -EBUSY;
		goto mdc_busy;
	}

	mdcbase = ioremap(camio_ext.mdcphy,
		camio_ext.mdcsz);
	if (!mdcbase) {
		rc = -ENOMEM;
		goto mdc_no_mem;
	}

	camdev->camera_gpio_on();

	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	return 0;

mdc_no_mem:
	release_mem_region(camio_ext.mdcphy, camio_ext.mdcsz);
mdc_busy:
	iounmap(appbase);
apps_no_mem:
	release_mem_region(camio_ext.appphy, camio_ext.appsz);
enable_fail:
	return rc;
}

void msm_camio_disable(struct platform_device *pdev)
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_camera_device_platform_data *camdev =
		pdev->dev.platform_data;
#else
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
#endif


	iounmap(mdcbase);
	release_mem_region(camio_ext.mdcphy, camio_ext.mdcsz);
	iounmap(appbase);
	release_mem_region(camio_ext.appphy, camio_ext.appsz);

	camdev->camera_gpio_off();

	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
}

void msm_camio_camif_pad_reg_reset(void)
{
	uint32_t reg;
	uint32_t mask, value;

	/* select CLKRGM_VFE_SRC_CAM_VFE_SRC:  internal source */
	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;

	mask = CAM_SEL_BMSK |
		CAM_PCLK_SRC_SEL_BMSK |
		CAM_PCLK_INVERT_BMSK;

	value = 1 << CAM_SEL_SHFT |
		3 << CAM_PCLK_SRC_SEL_SHFT |
		0 << CAM_PCLK_INVERT_SHFT;

	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_EXTERNAL);
	mdelay(10);
}

void msm_camio_vfe_blk_reset(void)
{
	uint32_t val;

	/* do apps reset */
	val = readl(appbase + 0x00000210);
	val |= 0x1;
	writel(val, appbase + 0x00000210);
	mdelay(10);

	val = readl(appbase + 0x00000210);
	val &= ~0x1;
	writel(val, appbase + 0x00000210);
	mdelay(10);

	/* do axi reset */
	val = readl(appbase + 0x00000208);
	val |= 0x1;
	writel(val, appbase + 0x00000208);
	mdelay(10);

	val = readl(appbase + 0x00000208);
	val &= ~0x1;
	writel(val, appbase + 0x00000208);
	mdelay(10);
}

void msm_camio_camif_pad_reg_reset_2(void)
{
	uint32_t reg;
	uint32_t mask, value;

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	struct clk *clk = NULL;

	clk = camio_vfe_clk;

	if (clk != NULL && clk != ERR_PTR(-ENOENT)) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}

int msm_camio_probe_on(struct platform_device *pdev)
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_camera_device_platform_data *camdev =
		pdev->dev.platform_data;
#else
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
#endif


	camdev->camera_gpio_on();
	return msm_camio_clk_enable(CAMIO_VFE_CLK);
}

int msm_camio_probe_off(struct platform_device *pdev)
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_camera_device_platform_data *camdev =
		pdev->dev.platform_data;
#else
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
#endif

	camdev->camera_gpio_off();
	return msm_camio_clk_disable(CAMIO_VFE_CLK);
}
