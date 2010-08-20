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

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/init.h>
#include "../clock.h"
#include <mach/qdsp6/msm8k_ard_clk.h>

static struct clk	*rx_clk;
static struct clk	*tx_clk;
static struct clk	*ecodec_clk;
static struct clk	*sdac_clk;


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif


void ard_clk_enable_internal_codec_clk_tx(void)
{
	s32 rc = CAD_RES_SUCCESS;

	if (tx_clk != NULL)
		return;

	tx_clk = clk_get(NULL, "icodec_tx_clk");
	if (tx_clk == NULL) {
		pr_err("ard_clk: Invalid TX clk!\n");
		return;
	}

	ard_clk_set_icodec_tx_clk();

	rc = clk_enable(tx_clk);
	if (rc != CAD_RES_SUCCESS)
		pr_err("ard_clk: RX clk not enabled!\n");
}


void ard_clk_enable_internal_codec_clk_rx(void)
{
	s32 rc = CAD_RES_SUCCESS;

	if (rx_clk != NULL)
		return;

	rx_clk = clk_get(NULL, "icodec_rx_clk");

	if (rx_clk == NULL) {
		pr_err("ard_clk: Invalid RX clk!\n");
		return;
	}

	ard_clk_set_icodec_rx_clk();

	rc = clk_enable(rx_clk);
	if (rc != CAD_RES_SUCCESS)
		pr_err("ard_clk: RX clk not enabled!\n");
}


void ard_clk_enable_external_codec_clk(void)
{
	s32 rc = CAD_RES_SUCCESS;

	if (ecodec_clk != NULL)
		return;

	ecodec_clk = clk_get(NULL, "ecodec_clk");
	if (ecodec_clk == NULL) {
		pr_err("ard_clk: Invalid ECODEC clk!\n");
		return;
	}

	ard_clk_set_ecodec_clk();

	rc = clk_enable(ecodec_clk);
	if (rc != CAD_RES_SUCCESS)
		pr_err("ard_clk: ECODEC clk not enabled!\n");
}

void ard_clk_enable_sdac_rx_clk(void)
{
	/* Enable and Select SDAC clks if either I2S path is active */
	s32 rc = CAD_RES_SUCCESS;

	if (sdac_clk != NULL)
		return;

	sdac_clk = clk_get(NULL, "sdac_clk");
	if (sdac_clk == NULL) {
		pr_err("sdac_clk: Invalid RX clk!\n");
		return;
	}

	ard_clk_set_sdac_rx_clk();

	rc = clk_enable(sdac_clk);
	if (rc != CAD_RES_SUCCESS)
		pr_err("sdac_clk: RX clk not enabled!\n");
}

void ard_clk_enable_sdac_tx_clk(void)
{
	/* Enable and Select SDAC clks if either I2S path is active */
	s32 rc = CAD_RES_SUCCESS;

	if (sdac_clk != NULL)
		return;

	sdac_clk = clk_get(NULL, "sdac_clk");
	if (sdac_clk == NULL) {
		pr_err("sdac_clk: Invalid TX clk!\n");
		return;
	}

	ard_clk_set_sdac_tx_clk();

	rc = clk_enable(sdac_clk);
	if (rc != CAD_RES_SUCCESS)
		pr_err("sdac_clk: TX clk not enabled!\n");
}

void ard_clk_disable_internal_codec_clk_tx(void)
{
	if (tx_clk == NULL)
		return;

	clk_disable(tx_clk);
	clk_put(tx_clk);
	tx_clk = NULL;
}

void ard_clk_disable_internal_codec_clk_rx(void)
{
	if (rx_clk == NULL)
		return;

	clk_disable(rx_clk);
	clk_put(rx_clk);
	rx_clk = NULL;
}


void ard_clk_disable_external_codec_clk(void)
{
	if (ecodec_clk == NULL)
		return;

	clk_disable(ecodec_clk);
	clk_put(ecodec_clk);
	ecodec_clk = NULL;
}

void ard_clk_disable_sdac_clk(void)
{
	if (sdac_clk == NULL)
		return;

	clk_disable(sdac_clk);
	clk_put(sdac_clk);
	sdac_clk = NULL;
}


void ard_clk_set_icodec_rx_clk(void)
{
	s32 rc = CAD_RES_SUCCESS;
	/* Frequency in Hz - 48KHz */
	rc = clk_set_rate(rx_clk, 12288000);

	if (rc != CAD_RES_SUCCESS)
		pr_err("ard_clk: Rate on RX clk not set!\n");
}


void ard_clk_set_icodec_tx_clk(void)
{
	s32 rc = CAD_RES_SUCCESS;

	rc = clk_set_rate(tx_clk, g_clk_info.tx_clk_freq*256);

	if (rc != CAD_RES_SUCCESS)
		pr_err("ard_clk: Rate on TX clk not set!\n");
}

void ard_clk_set_ecodec_clk(void)
{

	s32 rc = CAD_RES_SUCCESS;
	/* Frequency in Hz - 8Khz for AUX PCM now, later need to change to
	 * support I2S
	 */
	rc = clk_set_rate(ecodec_clk, 2048000);
	if (rc != CAD_RES_SUCCESS)
		pr_err("ard_clk: Rate on ECODEC clk not set!\n");

}
void ard_clk_set_sdac_rx_clk(void)
{
	s32 rc = CAD_RES_SUCCESS;

	/* Frequency in Hz - 48KHz */
	rc = clk_set_rate(sdac_clk, 12288000);

	if (rc != CAD_RES_SUCCESS)
		pr_err("sdac_clk: Rate on RX clk not set!\n");
}

void ard_clk_set_sdac_tx_clk(void)
{
	s32 rc = CAD_RES_SUCCESS;

	/* Frequency in Hz - 8KHz */
	rc = clk_set_rate(sdac_clk, 2048000);

	if (rc != CAD_RES_SUCCESS)
		pr_err("sdac_clk: Rate on TX clk not set!\n");
}

void ard_clk_enable(u32 dev_id)
{
	switch (dev_id) {
	case 0:
		ard_clk_enable_internal_codec_clk_rx();
		D("ENABLE RX INT CLK, dev_id %d\n", dev_id);
		break;
	case 1:
		ard_clk_enable_internal_codec_clk_tx();
		D("ENABLE TX INT CLK, dev_id %d\n", dev_id);
		break;
	case 2:
	case 3:
		/* No seperate TX and RX clocks for external codec */
		ard_clk_enable_external_codec_clk();
		D("ENABLE EXT CLK, dev_id %d\n", dev_id);
		break;
	case 6:
		ard_clk_enable_sdac_rx_clk();
		D("ENABLE SDAC CLK I2S RX");
		break;
	case 7:
		ard_clk_enable_sdac_tx_clk();
		D("ARD ENABLE SDAC CLK I2S TX");
		break;
	default:
		pr_err("unsupported clk\n");
	}
}

void ard_clk_disable(u32 dev_id)
{
	switch (dev_id) {
	case 0:
		ard_clk_disable_internal_codec_clk_rx();
		D("DISABLE RX INT CLK, dev_id %d\n", dev_id);
		break;
	case 1:
		ard_clk_disable_internal_codec_clk_tx();
		D("DISABLE TX INT CLK, dev_id %d\n", dev_id);
		break;
	case 2:
	case 3:
		/* No seperate TX and RX clocks for external codec */
		ard_clk_disable_external_codec_clk();
		D("DISABLE EXT CLK, dev_id %d\n", dev_id);
		break;
	case 6:
	case 7:
		ard_clk_disable_sdac_clk();
		D("ARD ENABLE SDAC CLK, dev_id %d", dev_id);
		break;
	default:
		pr_err("unsupported clk setting\n");
	}
}
