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

#include <mach/qdsp6/msm8k_cad_q6dtmf_session.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_volume.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_command.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

s32 cad_dtmf_session_init(struct q6dtmf_session *self)
{
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_dinit(struct q6dtmf_session *self)
{
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_open(struct q6dtmf_session *self, s32 session_id,
				struct cad_open_struct_type *open_param)
{
	self->session_id = session_id;
	D("CAD:DTMF ===> Session open successful\n");
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_close(struct q6dtmf_session *self)
{
	self->session_id = 0;
	D("CAD:DTMF ===> Session close successful\n");
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_ioctl(struct q6dtmf_session *self, s32 cmd_code,
				void *cmd_buf, s32 cmd_len)
{
	union adsp_audio_event			return_status;
	struct adsp_audio_dtmf_start_command	dtmf_cmd;
	struct cad_cmd_gen_dtmf			*data;
	s32					result = CAD_RES_SUCCESS;

	switch (cmd_code) {
	case CAD_IOCTL_CMD_GEN_DTMF:
		data = (struct cad_cmd_gen_dtmf *)cmd_buf;
		dtmf_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_SESSION_DTMF_START;
		dtmf_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;
		dtmf_cmd.tone1_hz = data->dtmf_hi;
		dtmf_cmd.tone2_hz = data->dtmf_low;
		dtmf_cmd.duration_usec = data->duration * 1000;
		dtmf_cmd.gain_mb = data->rx_gain;
		D("CAD:DTMF ===> send %d, %d, %d, %d\n", dtmf_cmd.tone1_hz,
			dtmf_cmd.tone2_hz, dtmf_cmd.duration_usec,
			dtmf_cmd.gain_mb);

		/* send the dtmf start with the configuration */
		result = cad_rpc_control(self->session_id, self->group_id,
			(void *)&dtmf_cmd, sizeof(dtmf_cmd), &return_status);
		break;
	}

	return result;
}
