/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ARDCLK_H
#define ARDCLK_H

#include "msm8k_ardi.h"

void ard_clk_enable(u32 dev_id);
void ard_clk_disable(u32 dev_id);

void ard_clk_set_icodec_rx_clk(void);
void ard_clk_set_icodec_tx_clk(void);

void ard_clk_enable_internal_codec_clk_rx(void);
void ard_clk_enable_internal_codec_clk_tx(void);
void ard_clk_disable_internal_codec_clk_rx(void);
void ard_clk_disable_internal_codec_clk_tx(void);

void ard_clk_enable_external_codec_clk(void);
void ard_clk_disable_external_codec_clk(void);
void ard_clk_set_ecodec_clk(void);

void ard_clk_enable_sdac_rx_clk(void);
void ard_clk_enable_sdac_tx_clk(void);
void ard_clk_set_sdac_rx_clk(void);
void ard_clk_set_sdac_tx_clk(void);
void ard_clk_disable_sdac_clk(void);

struct clk_info {

	u32 tx_clk_freq;
	u16 open_rec_sessions;
};

extern struct clk_info g_clk_info;

#endif


