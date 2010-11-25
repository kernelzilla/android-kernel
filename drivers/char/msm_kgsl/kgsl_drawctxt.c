/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/string.h>
#include <linux/types.h>
#include <linux/msm_kgsl.h>

#include "kgsl_drawctxt.h"

#include "yamato_reg.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"


/*
*
*  Memory Map for Register, Constant & Instruction Shadow, and Command Buffers
*  (34.5KB)
*
*  +---------------------+------------+-------------+---+---------------------+
*  | ALU Constant Shadow | Reg Shadow | C&V Buffers |Tex| Shader Instr Shadow |
*  +---------------------+------------+-------------+---+---------------------+
*    ________________________________/               \____________________
*   /                                                                     |
*  +--------------+-----------+------+-----------+------------------------+
*  | Restore Regs | Save Regs | Quad | Gmem Save | Gmem Restore | unused  |
*  +--------------+-----------+------+-----------+------------------------+
*
* 		 8K - ALU Constant Shadow (8K aligned)
* 		 4K - H/W Register Shadow (8K aligned)
* 		 4K - Command and Vertex Buffers
* 				- Indirect command buffer : Const/Reg restore
* 					- includes Loop & Bool const shadows
* 				- Indirect command buffer : Const/Reg save
* 				- Quad vertices & texture coordinates
* 				- Indirect command buffer : Gmem save
* 				- Indirect command buffer : Gmem restore
* 				- Unused (padding to 8KB boundary)
* 		<1K - Texture Constant Shadow (768 bytes) (8K aligned)
*       18K - Shader Instruction Shadow
*               - 6K vertex (32 byte aligned)
*               - 6K pixel  (32 byte aligned)
*               - 6K shared (32 byte aligned)
*
*  Note: Reading constants into a shadow, one at a time using REG_TO_MEM, takes
*  3 DWORDS per DWORD transfered, plus 1 DWORD for the shadow, for a total of
*  16 bytes per constant.  If the texture constants were transfered this way,
*  the Command & Vertex Buffers section would extend past the 16K boundary.
*  By moving the texture constant shadow area to start at 16KB boundary, we
*  only require approximately 40 bytes more memory, but are able to use the
*  LOAD_CONSTANT_CONTEXT shadowing feature for the textures, speeding up
*  context switching.
*
*  [Using LOAD_CONSTANT_CONTEXT shadowing feature for the Loop and/or Bool
*  constants would require an additional 8KB each, for alignment.]
*
*/

/* Constants */

#define ALU_CONSTANTS	2048	/* DWORDS */
#define NUM_REGISTERS	1024	/* DWORDS */
#define CMD_BUFFER_LEN	1024	/* DWORDS */
#define TEX_CONSTANTS		(32*6)	/* DWORDS */
#define BOOL_CONSTANTS		8	/* DWORDS */
#define LOOP_CONSTANTS		56	/* DWORDS */
#define SHADER_INSTRUCT_LOG2	9U	/* 2^n == SHADER_INSTRUCTIONS */

#if defined(PM4_IM_STORE)
/* 96-bit instructions */
#define SHADER_INSTRUCT		(1<<SHADER_INSTRUCT_LOG2)
#else
#define SHADER_INSTRUCT		0
#endif

/* LOAD_CONSTANT_CONTEXT shadow size */
#define LCC_SHADOW_SIZE		0x2000	/* 8KB */

#define ALU_SHADOW_SIZE		LCC_SHADOW_SIZE	/* 8KB */
#define REG_SHADOW_SIZE		0x1000	/* 4KB */
#define CMD_BUFFER_SIZE		0x1000 /* 4KB */
#define TEX_SHADOW_SIZE		(TEX_CONSTANTS*4)	/* 768 bytes */
#define SHADER_SHADOW_SIZE      (SHADER_INSTRUCT*12)	/* 6KB */

#define REG_OFFSET		LCC_SHADOW_SIZE
#define CMD_OFFSET		(REG_OFFSET + REG_SHADOW_SIZE)
#define TEX_OFFSET		(CMD_OFFSET + CMD_BUFFER_SIZE)
#define	SHADER_OFFSET		((TEX_OFFSET + TEX_SHADOW_SIZE + 32) & ~31)

#define CONTEXT_SIZE		(SHADER_OFFSET + 3 * SHADER_SHADOW_SIZE)

/* Flags */
#define CTXT_FLAGS_NOT_IN_USE		0x00000000
#define CTXT_FLAGS_IN_USE		0x00000001

/* state shadow memory allocated */
#define CTXT_FLAGS_STATE_SHADOW		0x00000010
/* gmem shadow memory allocated */
#define CTXT_FLAGS_GMEM_SHADOW		0x00000100
/* gmem must be copied to shadow */
#define CTXT_FLAGS_GMEM_SAVE		0x00000200
/* gmem can be restored from shadow */
#define CTXT_FLAGS_GMEM_RESTORE		0x00000400
/* shader must be copied to shadow */
#define CTXT_FLAGS_SHADER_SAVE		0x00002000
/* shader can be restored from shadow */
#define CTXT_FLAGS_SHADER_RESTORE	0x00004000

/* temporary work structure */
struct tmp_ctx {
	unsigned int *start;	/* Command & Vertex buffer start */
	unsigned int *cmd;	/* Next available dword in C&V buffer */

	/* address of buffers, needed when creating IB1 command buffers. */
	uint32_t bool_shadow;	/* bool constants */
	uint32_t loop_shadow;	/* loop constants */
	uint32_t quad_vertices;	/* vertices of quad */
	uint32_t quad_texcoord;	/* tex-coordinates of quad */

#if defined(PM4_IM_STORE)
	uint32_t shader_shared;	/* shared shader instruction shadow */
	uint32_t shader_vertex;	/* vertex shader instruction shadow */
	uint32_t shader_pixel;	/* pixel shader instruction shadow */
#endif

	/* Addresses in command buffer where separately handled registers
	 * are saved
	 */
	uint32_t reg_values[2];

	uint32_t gmem_base;	/* Base gpu address of GMEM */

	/* 256 KB GMEM surface = 4 bytes-per-pixel x 256 pixels/row x 256 rows.
	 * width & height must be a multiples of 32, in case tiled textures are
	 * used.
	 */
	int gmem_size;		/* Size of surface used to store GMEM */
	int width;		/* Width of surface used to store GMEM */
	int height;		/* Height of surface used to store GMEM */
	int pitch;		/* Pitch of surface used to store GMEM */
} ctx_t;

/* Helper function to calculate IEEE754 single precision float values
*  without FPU
*/
unsigned int uint2float(unsigned int uintval)
{
	unsigned int exp = 0;
	unsigned int frac = 0;
	unsigned int u = uintval;

	/* Find log2 of u */
	if (u >= 0x10000) {
		exp += 16;
		u >>= 16;
	}
	if (u >= 0x100) {
		exp += 8;
		u >>= 8;
	}
	if (u >= 0x10) {
		exp += 4;
		u >>= 4;
	}
	if (u >= 0x4) {
		exp += 2;
		u >>= 2;
	}
	if (u >= 0x2) {
		exp += 1;
		u >>= 1;
	}

	/* Calculate fraction */
	frac = (uintval & (~(1 << exp))) << (23 - exp);

	/* Exp is biased by 127 and shifted 23 bits */
	exp = (exp + 127) << 23;

	return exp | frac;
}

/* context save (gmem -> sys) */

/* pre-compiled vertex shader program
*
*  attribute vec4  P;
*  void main(void)
*  {
*    gl_Position = P;
*  }
*/
#define GMEM2SYS_VTX_PGM_LEN	0x12

static unsigned int gmem2sys_vtx_pgm[GMEM2SYS_VTX_PGM_LEN] = {
	0x00011003, 0x00001000, 0xc2000000,
	0x00001004, 0x00001000, 0xc4000000,
	0x00001005, 0x00002000, 0x00000000,
	0x1cb81000, 0x00398a88, 0x00000003,
	0x140f803e, 0x00000000, 0xe2010100,
	0x14000000, 0x00000000, 0xe2000000
};

/* pre-compiled fragment shader program
*
*  precision highp float;
*  uniform   vec4  clear_color;
*  void main(void)
*  {
*     gl_FragColor = clear_color;
*  }
*/

#define GMEM2SYS_FRAG_PGM_LEN	0x0c

static unsigned int gmem2sys_frag_pgm[GMEM2SYS_FRAG_PGM_LEN] = {
	0x00000000, 0x1002c400, 0x10000000,
	0x00001003, 0x00002000, 0x00000000,
	0x140f8000, 0x00000000, 0x22000000,
	0x14000000, 0x00000000, 0xe2000000
};

/* context restore (sys -> gmem) */
/* pre-compiled vertex shader program
*
*  attribute vec4 position;
*  attribute vec4 texcoord;
*  varying   vec4 texcoord0;
*  void main()
*  {
*     gl_Position = position;
*     texcoord0 = texcoord;
*  }
*/

#define SYS2GMEM_VTX_PGM_LEN	0x18

static unsigned int sys2gmem_vtx_pgm[SYS2GMEM_VTX_PGM_LEN] = {
	0x00052003, 0x00001000, 0xc2000000, 0x00001005,
	0x00001000, 0xc4000000, 0x00001006, 0x10071000,
	0x20000000, 0x18981000, 0x0039ba88, 0x00000003,
	0x12982000, 0x40257b08, 0x00000002, 0x140f803e,
	0x00000000, 0xe2010100, 0x140f8000, 0x00000000,
	0xe2020200, 0x14000000, 0x00000000, 0xe2000000
};

/* pre-compiled fragment shader program
*
*  precision mediump   float;
*  uniform   sampler2D tex0;
*  varying   vec4      texcoord0;
*  void main()
*  {
*     gl_FragColor = texture2D(tex0, texcoord0.xy);
*  }
*/

#define SYS2GMEM_FRAG_PGM_LEN	0x0f

static unsigned int sys2gmem_frag_pgm[SYS2GMEM_FRAG_PGM_LEN] = {
	0x00011002, 0x00001000, 0xc4000000, 0x00001003,
	0x10041000, 0x20000000, 0x10000001, 0x1ffff688,
	0x00000002, 0x140f8000, 0x00000000, 0xe2000000,
	0x14000000, 0x00000000, 0xe2000000
};

/* shader texture constants (sysmem -> gmem)  */
#define SYS2GMEM_TEX_CONST_LEN	6

static unsigned int sys2gmem_tex_const[SYS2GMEM_TEX_CONST_LEN] = {
	/* Texture, FormatXYZW=Unsigned, ClampXYZ=Wrap/Repeat,
	 * RFMode=ZeroClamp-1, Dim=1:2d
	 */
	0x00100002 | 0 << 31,	/* Pitch = TBD */

	/* Format=6:8888_WZYX, EndianSwap=0:None, ReqSize=0:256bit, DimHi=0,
	 * NearestClamp=1:OGL Mode
	 */
	0x00000806,		/* Address[31:12] = TBD */

	/* Width, Height, EndianSwap=0:None */
	0,			/* Width & Height = TBD */

	/* NumFormat=0:RF, DstSelXYZW=XYZW, ExpAdj=0, MagFilt=MinFilt=0:Point,
	 * Mip=2:BaseMap
	 */
	0 << 1 | 1 << 4 | 2 << 7 | 3 << 10 | 2 << 23,

	/* VolMag=VolMin=0:Point, MinMipLvl=0, MaxMipLvl=1, LodBiasH=V=0,
	 * Dim3d=0
	 */
	1 << 6,

	/* BorderColor=0:ABGRBlack, ForceBC=0:diable, TriJuice=0, Aniso=0,
	 * Dim=1:2d, MipPacking=0
	 */
	1 << 9			/* Mip Address[31:12] = TBD */
};

/* quad for copying GMEM to context shadow */
#define QUAD_LEN				12

static unsigned int gmem_copy_quad[QUAD_LEN] = {
	0x00000000, 0x00000000, 0x3f800000,
	0x00000000, 0x00000000, 0x3f800000,
	0x00000000, 0x00000000, 0x3f800000,
	0x00000000, 0x00000000, 0x3f800000
};

#define TEXCOORD_LEN			8

static unsigned int gmem_copy_texcoord[TEXCOORD_LEN] = {
	0x00000000, 0x3f000000,
	0x3f000000, 0x3f000000,
	0x00000000, 0x00000000,
	0x3f000000, 0x00000000
};

/* shader linkage info */
#define SHADER_CONST_ADDR	(11 * 6 + 3)

/* gmem command buffer length */
#define PM4_REG(reg)		((0x4 << 16) | (GSL_HAL_SUBBLOCK_OFFSET(reg)))

/* functions */
static void config_gmemsize(struct tmp_ctx *ctx, int gmem_size)
{
	int w = 64, h = 64;	/* 16KB surface, minimum */

	/* convert from bytes to 32-bit words */
	gmem_size = (gmem_size + 3) / 4;

	/* find the right surface size, close to a square. */
	while (w * h < gmem_size)
		if (w < h)
			w *= 2;
		else
			h *= 2;

	ctx->width = w;
	ctx->pitch = w;
	ctx->height = h;

	ctx->gmem_size = ctx->pitch * ctx->height * 4;
}

static unsigned int gpuaddr(unsigned int *cmd, struct kgsl_memdesc *memdesc)
{
	return memdesc->gpuaddr + ((char *)cmd - (char *)memdesc->hostptr);
}

static void
create_ib1(struct kgsl_drawctxt *drawctxt, unsigned int *cmd,
	   unsigned int *start, unsigned int *end)
{
	cmd[0] = PM4_HDR_INDIRECT_BUFFER_PFD;
	cmd[1] = gpuaddr(start, &drawctxt->gpustate);
	cmd[2] = end - start;
}

static unsigned int *program_shader(unsigned int *cmds, int vtxfrag,
				    unsigned int *shader_pgm, int dwords)
{
	/* load the patched vertex shader stream */
	*cmds++ = pm4_type3_packet(PM4_IM_LOAD_IMMEDIATE, 2 + dwords);
	/* 0=vertex shader, 1=fragment shader */
	*cmds++ = vtxfrag;
	/* instruction start & size (in 32-bit words) */
	*cmds++ = ((0 << 16) | dwords);

	memcpy(cmds, shader_pgm, dwords << 2);
	cmds += dwords;

	return cmds;
}

static unsigned int *reg_to_mem(unsigned int *cmds, uint32_t dst,
				uint32_t src, int dwords)
{
	while (dwords-- > 0) {
		*cmds++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
		*cmds++ = src++;
		*cmds++ = dst;
		dst += 4;
	}

	return cmds;
}

/* save h/w regs, alu constants, texture contants, etc. ...
*  requires: bool_shadow_gpuaddr, loop_shadow_gpuaddr
*/
static void build_regsave_cmds(struct kgsl_drawctxt *drawctxt,
				struct tmp_ctx *ctx)
{
	unsigned int *start = ctx->cmd;
	unsigned int *cmd = start;

	/* Insert a wait for idle packet before reading the registers.
	 * This is to fix a hang/reset seen during stress testing.
	 */
	*cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
	*cmd++ = 0x0;

	/* H/w registers are already shadowed; just need to disable shadowing
	 * to prevent corruption.
	 */
	*cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
	*cmd++ = 4 << 16;	/* regs, start=0 */
	*cmd++ = 0x0;		/* count = 0 */

	/* ALU constants are already shadowed; just need to disable shadowing
	 * to prevent corruption.
	 */
	*cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = drawctxt->gpustate.gpuaddr & 0xFFFFE000;
	*cmd++ = 0 << 16;	/* ALU, start=0 */
	*cmd++ = 0x0;		/* count = 0 */

	/* Tex constants are already shadowed; just need to disable shadowing
	 *  to prevent corruption.
	 */
	*cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = (drawctxt->gpustate.gpuaddr + TEX_OFFSET) & 0xFFFFE000;
	*cmd++ = 1 << 16;	/* Tex, start=0 */
	*cmd++ = 0x0;		/* count = 0 */

	/* Need to handle some of the registers separately */
	*cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
	*cmd++ = REG_PA_SU_FACE_DATA;
	*cmd++ = ctx->reg_values[0];

	*cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
	*cmd++ = REG_SQ_GPR_MANAGEMENT;
	*cmd++ = ctx->reg_values[1];

	/* Copy Boolean constants */
	cmd = reg_to_mem(cmd, ctx->bool_shadow, REG_SQ_CF_BOOLEANS,
			 BOOL_CONSTANTS);

	/* Copy Loop constants */
	cmd = reg_to_mem(cmd, ctx->loop_shadow, REG_SQ_CF_LOOP, LOOP_CONSTANTS);

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->reg_save, start, cmd);

	ctx->cmd = cmd;
}


/*copy colour, depth, & stencil buffers from graphics memory to system memory*/
static void build_gmem2sys_cmds(struct kgsl_drawctxt *drawctxt,
				struct tmp_ctx *ctx)
{
	unsigned int *start = ctx->cmd;
	unsigned int *cmds = start;

	/* program shader */

	/* load shader vtx constants ... 5 dwords */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 4);
	*cmds++ = (0x1 << 16) | SHADER_CONST_ADDR;
	*cmds++ = 0;
	/* valid(?) vtx constant flag & addr */
	*cmds++ = ctx->quad_vertices | 0x3;
	/* limit = 12 dwords */
	*cmds++ = 0x00000030;

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_VGT_INDX_OFFSET);
	*cmds++ = 0x00000000;

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_VGT_MAX_VTX_INDX);
	*cmds++ = 0x00ffffff;	/* REG_VGT_MAX_VTX_INDX */
	*cmds++ = 0x0;		/* REG_VGT_MIN_VTX_INDX */

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_PA_SC_AA_MASK);
	*cmds++ = 0x0000ffff;	/* REG_PA_SC_AA_MASK */

	/* load the patched vertex shader stream */
	cmds = program_shader(cmds, 0, gmem2sys_vtx_pgm, GMEM2SYS_VTX_PGM_LEN);

	/* Load the patched fragment shader stream */
	cmds =
	    program_shader(cmds, 1, gmem2sys_frag_pgm, GMEM2SYS_FRAG_PGM_LEN);

	/* SQ_PROGRAM_CNTL / SQ_CONTEXT_MISC */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_SQ_PROGRAM_CNTL);
	*cmds++ = 0x10010001;
	*cmds++ = 0x00000008;

	/* resolve */

	/* PA_CL_VTE_CNTL */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_PA_CL_VTE_CNTL);
	/* disable X/Y/Z transforms, X/Y/Z are premultiplied by W */
	*cmds++ = 0x00000b00;

	/* change colour buffer to RGBA8888, MSAA = 1, and matching pitch */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_RB_SURFACE_INFO);
	*cmds++ = ctx->pitch;	/* pitch, MSAA = 1 */

	/* RB_COLOR_INFO Endian=none, Linear, Format=RGBA8888, Swap=0,
	 *                Base=gmem_base
	 */
	/* gmem base assumed 4K aligned. */
	BUG_ON(ctx->gmem_base & 0xFFF);
	*cmds++ = (COLORX_8_8_8_8 << RB_COLOR_INFO__COLOR_FORMAT__SHIFT) |
	    ctx->gmem_base;

	/* disable Z */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_DEPTHCONTROL);
	*cmds++ = 0;

	/* set REG_PA_SU_SC_MODE_CNTL
	 *              Front_ptype = draw triangles
	 *              Back_ptype = draw triangles
	 *              Provoking vertex = last
	 */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_PA_SU_SC_MODE_CNTL);
	*cmds++ = 0x00080240;

	/* set the scissor to the extents of the draw surface */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_PA_SC_SCREEN_SCISSOR_TL);
	*cmds++ = (0 << 16) | 0;
	*cmds++ = (ctx->height << 16) | ctx->width;
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_PA_SC_WINDOW_SCISSOR_TL);
	*cmds++ = (unsigned int)((1U << 31) | (0 << 16) | 0);
	*cmds++ = (ctx->height << 16) | ctx->width;

	/* set the viewport to the extents of the draw surface */
	{
		unsigned int xscale = uint2float(ctx->width / 2);
		unsigned int xoffset = xscale;
		unsigned int yscale = uint2float(-ctx->height / 2);
		unsigned int yoffset = uint2float(ctx->height / 2);

		*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 5);
		*cmds++ = PM4_REG(REG_PA_CL_VPORT_XSCALE);
		*cmds++ = xscale;
		*cmds++ = xoffset;
		*cmds++ = yscale;
		*cmds++ = yoffset;
	}

	/* load the viewport so that z scale = clear depth and
	*  z offset = 0.0f
	*/
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_PA_CL_VPORT_ZSCALE);
	*cmds++ = 0xbf800000;	/* -1.0f */
	*cmds++ = 0x0;

	/* load the stencil ref value
	 * $AAM - do this later
	 */

	/* load the COPY state */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 6);
	*cmds++ = PM4_REG(REG_RB_COPY_CONTROL);
	*cmds++ = 0;		/* RB_COPY_CONTROL */
	*cmds++ = drawctxt->gmemshadow.gpuaddr;	/* RB_COPY_DEST_BASE */
	*cmds++ = ctx->pitch >> 5;	/* RB_COPY_DEST_PITCH */

	/* Endian=none, Linear, Format=RGBA8888,Swap=0,!Dither,
	 *  MaskWrite:R=G=B=A=1
	 */
	*cmds++ = 0x0003c058;
	*cmds++ = 0;		/* RB_COPY_DEST_PIXEL_OFFSET */

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_MODECONTROL);
	*cmds++ = 0x6;		/* EDRAM copy */

	/* queue the draw packet */
	*cmds++ = pm4_type3_packet(PM4_DRAW_INDX, 2);
	*cmds++ = 0;		/* viz query info. */
	/* PrimType=RectList, NumIndices=3, SrcSel=AutoIndex */
	*cmds++ = 0x00030088;

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->gmem_save, start, cmds);

	ctx->cmd = cmds;
}

/* context restore */

/*copy colour, depth, & stencil buffers from system memory to graphics memory*/
static void build_sys2gmem_cmds(struct kgsl_drawctxt *drawctxt,
				struct tmp_ctx *ctx)
{
	unsigned int *start = ctx->cmd;
	unsigned int *cmds = start;

	/* shader constants */

	/* vertex buffer constants */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 7);

	*cmds++ = (0x1 << 16) | (9 * 6);
	/* valid(?) vtx constant flag & addr */
	*cmds++ = ctx->quad_vertices | 0x3;
	/* limit = 12 dwords */
	*cmds++ = 0x00000030;
	/* valid(?) vtx constant flag & addr */
	*cmds++ = ctx->quad_texcoord | 0x3;
	/* limit = 8 dwords */
	*cmds++ = 0x00000020;
	*cmds++ = 0;
	*cmds++ = 0;

	/* load the patched vertex shader stream */
	cmds = program_shader(cmds, 0, sys2gmem_vtx_pgm, SYS2GMEM_VTX_PGM_LEN);

	/* Load the patched fragment shader stream */
	cmds =
	    program_shader(cmds, 1, sys2gmem_frag_pgm, SYS2GMEM_FRAG_PGM_LEN);

	/* SQ_PROGRAM_CNTL / SQ_CONTEXT_MISC */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_SQ_PROGRAM_CNTL);
	*cmds++ = 0x10030002;
	*cmds++ = 0x00000008;

	/* PA_SC_VIZ_QUERY */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_PA_SC_VIZ_QUERY);
	*cmds++ = 0x0;		/*REG_PA_SC_VIZ_QUERY */

	/* RB_COLORCONTROL */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_COLORCONTROL);
	*cmds++ = 0x00000c20;

	/* texture constants */
	*cmds++ =
	    pm4_type3_packet(PM4_SET_CONSTANT, (SYS2GMEM_TEX_CONST_LEN + 1));
	*cmds++ = (0x1 << 16) | (0 * 6);
	memcpy(cmds, sys2gmem_tex_const, SYS2GMEM_TEX_CONST_LEN << 2);
	cmds[0] |= (ctx->pitch >> 5) << 22;
	cmds[1] |= drawctxt->gmemshadow.gpuaddr;
	cmds[2] |= (ctx->width - 1) | (ctx->height - 1) << 13;
	cmds[5] |= drawctxt->gmemshadow.gpuaddr;
	cmds += SYS2GMEM_TEX_CONST_LEN;

	/* PA_CL_VTE_CNTL */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_PA_CL_VTE_CNTL);
	/* disable X/Y/Z transforms, X/Y/Z are premultiplied by W */
	*cmds++ = 0x00000b00;

	/* change colour buffer to RGBA8888, MSAA = 1, and matching pitch */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_RB_SURFACE_INFO);
	*cmds++ = ctx->pitch;	/* pitch, MSAA = 1 */

	/* RB_COLOR_INFO Endian=none, Linear, Format=RGBA8888, Swap=0,
	 *                Base=gmem_base
	 */
	*cmds++ = (COLORX_8_8_8_8 << RB_COLOR_INFO__COLOR_FORMAT__SHIFT) |
	    ctx->gmem_base;

	/* RB_DEPTHCONTROL */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_DEPTHCONTROL);
	*cmds++ = 0;		/* disable Z */

	/* set REG_PA_SU_SC_MODE_CNTL
	 *              Front_ptype = draw triangles
	 *              Back_ptype = draw triangles
	 *              Provoking vertex = last
	 */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_PA_SU_SC_MODE_CNTL);
	*cmds++ = 0x00080240;

	/* set the scissor to the extents of the draw surface */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_PA_SC_SCREEN_SCISSOR_TL);
	*cmds++ = (0 << 16) | 0;
	*cmds++ = (ctx->height << 16) | ctx->width;
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_PA_SC_WINDOW_SCISSOR_TL);
	*cmds++ = (unsigned int)((1U << 31) | (0 << 16) | 0);
	*cmds++ = (ctx->height << 16) | ctx->width;

	/* set the viewport to the extents of the draw surface */
	{
		unsigned int xscale = uint2float(ctx->width / 2);
		unsigned int xoffset = xscale;
		unsigned int yscale = uint2float(-ctx->height / 2);
		unsigned int yoffset = uint2float(ctx->height / 2);

		*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 5);
		*cmds++ = PM4_REG(REG_PA_CL_VPORT_XSCALE);
		*cmds++ = xscale;
		*cmds++ = xoffset;
		*cmds++ = yscale;
		*cmds++ = yoffset;
	}

	/*load the viewport so that z scale = clear depth and z offset = 0.0f */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_PA_CL_VPORT_ZSCALE);
	*cmds++ = 0xbf800000;
	*cmds++ = 0x0;

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_COLOR_MASK);
	*cmds++ = 0x0000000f;	/* R = G = B = 1:enabled */

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_COLOR_DEST_MASK);
	*cmds++ = 0xffffffff;

	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
	*cmds++ = PM4_REG(REG_SQ_WRAPPING_0);
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;

	/* load the stencil ref value
	 *  $AAM - do this later
	 */
	*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
	*cmds++ = PM4_REG(REG_RB_MODECONTROL);
	/* draw pixels with color and depth/stencil component */
	*cmds++ = 0x4;

	/* queue the draw packet */
	*cmds++ = pm4_type3_packet(PM4_DRAW_INDX, 2);
	*cmds++ = 0;		/* viz query info. */
	/* PrimType=RectList, NumIndices=3, SrcSel=AutoIndex */
	*cmds++ = 0x00030088;

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->gmem_restore, start, cmds);

	ctx->cmd = cmds;
}

/* restore h/w regs, alu constants, texture constants, etc. ... */
static unsigned *reg_range(unsigned int *cmd, unsigned int start,
			   unsigned int end)
{
	*cmd++ = PM4_REG(start);	/* h/w regs, start addr */
	*cmd++ = end - start + 1;	/* count */
	return cmd;
}

static void build_regrestore_cmds(struct kgsl_drawctxt *drawctxt,
					 struct tmp_ctx *ctx)
{
	unsigned int *start = ctx->cmd;
	unsigned int *cmd = start;

	/*
	   *cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
	   *cmd++ = 0;
	*/


	/* H/W Registers */
	/* deferred pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, ???); */
	cmd++;
	*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;

	cmd = reg_range(cmd, REG_RB_SURFACE_INFO, REG_PA_SC_SCREEN_SCISSOR_BR);
	cmd = reg_range(cmd, REG_PA_SC_WINDOW_OFFSET,
			REG_PA_SC_WINDOW_SCISSOR_BR);
	cmd = reg_range(cmd, REG_VGT_MAX_VTX_INDX, REG_PA_CL_VPORT_ZOFFSET);
	cmd = reg_range(cmd, REG_SQ_PROGRAM_CNTL, REG_SQ_WRAPPING_1);
	cmd = reg_range(cmd, REG_RB_DEPTHCONTROL, REG_RB_MODECONTROL);
	cmd = reg_range(cmd, REG_PA_SU_POINT_SIZE,
			REG_PA_SC_VIZ_QUERY /*REG_VGT_ENHANCE */);
	cmd = reg_range(cmd, REG_PA_SC_LINE_CNTL, REG_RB_COLOR_DEST_MASK);
	cmd = reg_range(cmd, REG_PA_SU_POLY_OFFSET_FRONT_SCALE,
			REG_PA_SU_POLY_OFFSET_BACK_OFFSET);

	/* Now we know how many register blocks we have, we can compute command
	 * length
	 */
	start[0] =
	    pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, (cmd - start) - 1);
	/* Enable shadowing for the entire register block. */
	start[2] |= (1 << 24) | (4 << 16);

	/* Need to handle some of the registers separately */
	*cmd++ = pm4_type0_packet(REG_PA_SU_FACE_DATA, 1);
	ctx->reg_values[0] = gpuaddr(cmd, &drawctxt->gpustate);
	*cmd++ = 0;

	*cmd++ = pm4_type0_packet(REG_SQ_GPR_MANAGEMENT, 1);
	ctx->reg_values[1] = gpuaddr(cmd, &drawctxt->gpustate);
	*cmd++ = 0x00040400;

	/* ALU Constants */
	*cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = drawctxt->gpustate.gpuaddr & 0xFFFFE000;
	*cmd++ = (1 << 24) | (0 << 16) | 0;
	*cmd++ = ALU_CONSTANTS;

	/* Texture Constants */
	*cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = (drawctxt->gpustate.gpuaddr + TEX_OFFSET) & 0xFFFFE000;
	*cmd++ = (1 << 24) | (1 << 16) | 0;
	*cmd++ = TEX_CONSTANTS;

	/* Boolean Constants */
	*cmd++ = pm4_type3_packet(PM4_SET_CONSTANT, 1 + BOOL_CONSTANTS);
	*cmd++ = (2 << 16) | 0;

	/* the next BOOL_CONSTANT dwords is the shadow area for
	 *  boolean constants.
	 */
	ctx->bool_shadow = gpuaddr(cmd, &drawctxt->gpustate);
	cmd += BOOL_CONSTANTS;

	/* Loop Constants */
	*cmd++ = pm4_type3_packet(PM4_SET_CONSTANT, 1 + LOOP_CONSTANTS);
	*cmd++ = (3 << 16) | 0;

	/* the next LOOP_CONSTANTS dwords is the shadow area for
	 * loop constants.
	 */
	ctx->loop_shadow = gpuaddr(cmd, &drawctxt->gpustate);
	cmd += LOOP_CONSTANTS;

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->reg_restore, start, cmd);

	ctx->cmd = cmd;
}

/* quad for saving/restoring gmem */
static void build_quad_vtxbuff(struct kgsl_drawctxt *drawctxt,
				struct tmp_ctx *ctx)
{
	unsigned int *cmd = ctx->cmd;

	/* quad vertex buffer location (in GPU space) */
	ctx->quad_vertices = gpuaddr(cmd, &drawctxt->gpustate);

	/* set vertex buffer values */
	gmem_copy_quad[1] = uint2float(ctx->height);
	gmem_copy_quad[3] = uint2float(ctx->width);
	gmem_copy_quad[4] = uint2float(ctx->height);
	gmem_copy_quad[9] = uint2float(ctx->width);

	/* copy quad data to vertex buffer */
	memcpy(cmd, gmem_copy_quad, QUAD_LEN << 2);
	cmd += QUAD_LEN;

	/* tex coord buffer location (in GPU space) */
	ctx->quad_texcoord = gpuaddr(cmd, &drawctxt->gpustate);

	/* copy tex coord data to tex coord buffer */
	memcpy(cmd, gmem_copy_texcoord, TEXCOORD_LEN << 2);
	cmd += TEXCOORD_LEN;

	ctx->cmd = cmd;
}

static void
build_shader_save_restore_cmds(struct kgsl_drawctxt *drawctxt,
				struct tmp_ctx *ctx)
{
	unsigned int *cmd = ctx->cmd;
	unsigned int *save, *restore, *fixup;
#if defined(PM4_IM_STORE)
	unsigned int *startSizeVtx, *startSizePix, *startSizeShared, *boolean1,
	    *boolean2, *boolean3;
#endif
	unsigned int *partition1;
	unsigned int *shaderBases, *partition2;

#if defined(PM4_IM_STORE)
	/* compute vertex, pixel and shared instruction shadow GPU addresses */
	ctx->shader_vertex = drawctxt->gpustate.gpuaddr + SHADER_OFFSET;
	ctx->shader_pixel = ctx->shader_vertex + SHADER_SHADOW_SIZE;
	ctx->shader_shared = ctx->shader_pixel + SHADER_SHADOW_SIZE;
#endif

	/* restore shader partitioning and instructions */

	restore = cmd;		/* start address */

	/* Invalidate Vertex & Pixel instruction code address and sizes */
	*cmd++ = pm4_type3_packet(PM4_INVALIDATE_STATE, 1);
	*cmd++ = 0x00000300;	/* 0x100 = Vertex, 0x200 = Pixel */

	/* Restore previous shader vertex & pixel instruction bases. */
	*cmd++ = pm4_type3_packet(PM4_SET_SHADER_BASES, 1);
	shaderBases = cmd++;	/* TBD #5: shader bases (from fixup) */

	/* write the shader partition information to a scratch register */
	*cmd++ = pm4_type0_packet(REG_SQ_INST_STORE_MANAGMENT, 1);
	partition1 = cmd++;	/* TBD #4a: partition info (from save) */

#if defined(PM4_IM_STORE)
	/* load vertex shader instructions from the shadow. */
	*cmd++ = pm4_type3_packet(PM4_IM_LOAD, 2);
	*cmd++ = ctx->shader_vertex + 0x0;	/* 0x0 = Vertex */
	startSizeVtx = cmd++;	/* TBD #1: start/size (from save) */

	/* load pixel shader instructions from the shadow. */
	*cmd++ = pm4_type3_packet(PM4_IM_LOAD, 2);
	*cmd++ = ctx->shader_pixel + 0x1;	/* 0x1 = Pixel */
	startSizePix = cmd++;	/* TBD #2: start/size (from save) */

	/* load shared shader instructions from the shadow. */
	*cmd++ = pm4_type3_packet(PM4_IM_LOAD, 2);
	*cmd++ = ctx->shader_shared + 0x2;	/* 0x2 = Shared */
	startSizeShared = cmd++;	/* TBD #3: start/size (from save) */
#endif

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->shader_restore, restore, cmd);

	/*
	 *  fixup SET_SHADER_BASES data
	 *
	 *  since self-modifying PM4 code is being used here, a seperate
	 *  command buffer is used for this fixup operation, to ensure the
	 *  commands are not read by the PM4 engine before the data fields
	 *  have been written.
	 */

	fixup = cmd;		/* start address */

	/* write the shader partition information to a scratch register */
	*cmd++ = pm4_type0_packet(REG_SCRATCH_REG2, 1);
	partition2 = cmd++;	/* TBD #4b: partition info (from save) */

	/* mask off unused bits, then OR with shader instruction memory size */
	*cmd++ = pm4_type3_packet(PM4_REG_RMW, 3);
	*cmd++ = REG_SCRATCH_REG2;
	/* AND off invalid bits. */
	*cmd++ = 0x0FFF0FFF;
	/* OR in instruction memory size */
	*cmd++ = (unsigned int)((SHADER_INSTRUCT_LOG2 - 5U) << 29);

	/* write the computed value to the SET_SHADER_BASES data field */
	*cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
	*cmd++ = REG_SCRATCH_REG2;
	/* TBD #5: shader bases (to restore) */
	*cmd++ = gpuaddr(shaderBases, &drawctxt->gpustate);

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->shader_fixup, fixup, cmd);

	/* save shader partitioning and instructions */

	/* Reserve space for boolean values used for conditional
	 * shared shader saving.
	 */
	boolean1 = cmd++;
	boolean2 = cmd++;
	boolean3 = cmd++;
	save = cmd;		/* start address */

	*boolean1 = 1;
	*boolean2 = 1;
	*boolean3 = 0;

	*cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
	*cmd++ = 0;

	/* fetch the SQ_INST_STORE_MANAGMENT register value,
	 *  store the value in the data fields of the SET_CONSTANT commands
	 *  above.
	 */
	*cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
	*cmd++ = REG_SQ_INST_STORE_MANAGMENT;
	/* TBD #4a: partition info (to restore) */
	*cmd++ = gpuaddr(partition1, &drawctxt->gpustate);
	*cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
	*cmd++ = REG_SQ_INST_STORE_MANAGMENT;
	/* TBD #4b: partition info (to fixup) */
	*cmd++ = gpuaddr(partition2, &drawctxt->gpustate);

#if defined(PM4_IM_STORE)

	/* Write zero into boolean value */
	*cmd++ = pm4_type3_packet(PM4_MEM_WRITE, 2);
	*cmd++ = gpuaddr(boolean1, &drawctxt->gpustate);
	*cmd++ = 0x00000000;
	/* Make sure the memory write has finished */
	*cmd++ = pm4_type3_packet(PM4_WAIT_REG_MEM, 5);
	*cmd++ = 0x00000013;	/* MEMSPACE = memory, FUNCTION = equal */
	*cmd++ = gpuaddr(boolean1, &drawctxt->gpustate);
	*cmd++ = 0x00000000;	/* Reference */
	*cmd++ = 0xffffffff;	/* Mask */
	*cmd++ = 0x00000001;	/* Wait interval */

	/* Write one into boolean value if vertex shader size is nonzero */
	*cmd++ = pm4_type3_packet(PM4_COND_WRITE, 6);
	/* Write mem, poll register, function = not equal */
	*cmd++ = 0x00000104;
	*cmd++ = REG_SQ_VS_PROGRAM;
	*cmd++ = 0x00000000;
	*cmd++ = 0x00fff000;
	*cmd++ = gpuaddr(boolean1, &drawctxt->gpustate);
	*cmd++ = 0x00000001;

	/* Conditional execution based on the boolean value */
	*cmd++ = pm4_type3_packet(PM4_COND_EXEC, 2);
	*cmd++ = gpuaddr(boolean1, &drawctxt->gpustate) >> 2;
	*cmd++ = 3;

	/* store the vertex shader instructions */
	*cmd++ = pm4_type3_packet(PM4_IM_STORE, 2);
	*cmd++ = ctx->shader_vertex + 0x0;	/* 0x0 = Vertex */
	/* TBD #1: start/size (to restore) */
	*cmd++ = gpuaddr(startSizeVtx, &drawctxt->gpustate);

	/* Write zero into boolean value */
	*cmd++ = pm4_type3_packet(PM4_MEM_WRITE, 2);
	*cmd++ = gpuaddr(boolean2, &drawctxt->gpustate);
	*cmd++ = 0x00000000;
	/* Make sure the memory write has finished */
	*cmd++ = pm4_type3_packet(PM4_WAIT_REG_MEM, 5);
	*cmd++ = 0x00000013;	/* MEMSPACE = memory, FUNCTION = equal */
	*cmd++ = gpuaddr(boolean2, &drawctxt->gpustate);
	*cmd++ = 0x00000000;	/* Reference */
	*cmd++ = 0xffffffff;	/* Mask */
	*cmd++ = 0x00000001;	/* Wait interval */

	/* Write one into boolean value if pixel shader size is nonzero */
	*cmd++ = pm4_type3_packet(PM4_COND_WRITE, 6);
	/* Write mem, poll register, function = not equal */
	*cmd++ = 0x00000104;
	*cmd++ = REG_SQ_PS_PROGRAM;
	*cmd++ = 0x00000000;
	*cmd++ = 0x00fff000;
	*cmd++ = gpuaddr(boolean2, &drawctxt->gpustate);
	*cmd++ = 0x00000001;

	/* Conditional execution based on the boolean value */
	*cmd++ = pm4_type3_packet(PM4_COND_EXEC, 2);
	*cmd++ = gpuaddr(boolean2, &drawctxt->gpustate) >> 2;
	*cmd++ = 3;

	/* store the pixel shader instructions */
	*cmd++ = pm4_type3_packet(PM4_IM_STORE, 2);
	*cmd++ = ctx->shader_pixel + 0x1;	/* 0x1 = Pixel */
	/* TBD #2: start/size (to restore) */
	*cmd++ = gpuaddr(startSizePix, &drawctxt->gpustate);

	/* store the shared shader instructions if vertex base is nonzero */

	/* Write zero into boolean value */
	*cmd++ = pm4_type3_packet(PM4_MEM_WRITE, 2);
	*cmd++ = gpuaddr(boolean3, &drawctxt->gpustate);
	*cmd++ = 0x00000000;
	/* Make sure the memory write has finished */
	*cmd++ = pm4_type3_packet(PM4_WAIT_REG_MEM, 5);
	*cmd++ = 0x00000013;	/* MEMSPACE = memory, FUNCTION = equal */
	*cmd++ = gpuaddr(boolean3, &drawctxt->gpustate);
	*cmd++ = 0x00000000;	/* Reference */
	*cmd++ = 0xffffffff;	/* Mask */
	*cmd++ = 0x00000001;	/* Wait interval */

	/* Write one into boolean value if vertex base is nonzero */
	*cmd++ = pm4_type3_packet(PM4_COND_WRITE, 6);
	/* Write mem, poll register, function = not equal */
	*cmd++ = 0x00000104;
	*cmd++ = REG_SQ_INST_STORE_MANAGMENT;
	*cmd++ = 0x00000000;
	*cmd++ = 0xffff0000;
	*cmd++ = gpuaddr(boolean3, &drawctxt->gpustate);
	*cmd++ = 0x00000001;

	/* Conditional execution based on the boolean value */
	*cmd++ = pm4_type3_packet(PM4_COND_EXEC, 2);
	*cmd++ = gpuaddr(boolean3, &drawctxt->gpustate) >> 2;
	*cmd++ = 3;

	*cmd++ = pm4_type3_packet(PM4_IM_STORE, 2);
	*cmd++ = ctx->shader_shared + 0x2;	/* 0x2 = Shared */
	/* TBD #3: start/size (to restore) */
	*cmd++ = gpuaddr(startSizeShared, &drawctxt->gpustate);

#endif

	*cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
	*cmd++ = 0;

	/* create indirect buffer command for above command sequence */
	create_ib1(drawctxt, drawctxt->shader_save, save, cmd);

	ctx->cmd = cmd;
}

/* create buffers for saving/restoring registers, constants, & GMEM */
static int
create_gpustate_shadow(struct kgsl_device *device,
			struct kgsl_drawctxt *drawctxt,
			struct tmp_ctx *ctx)
{
	uint32_t flags;

	flags = (KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN8K);

	/* allocate memory to allow HW to save sub-blocks for efficient context
	 *  save/restore
	 */
	if (kgsl_sharedmem_alloc(flags, CONTEXT_SIZE, &drawctxt->gpustate) != 0)
		return -ENOMEM;

	if (kgsl_mmu_map(drawctxt->pagetable,
				drawctxt->gpustate.physaddr,
				drawctxt->gpustate.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
				&drawctxt->gpustate.gpuaddr)) {
		return -EINVAL;
	}

	drawctxt->flags |= CTXT_FLAGS_STATE_SHADOW;

	/* Blank out h/w register, constant, and command buffer shadows. */
	kgsl_sharedmem_set(&drawctxt->gpustate, 0, 0, CONTEXT_SIZE);

	/* set-up command and vertex buffer pointers */
	ctx->cmd = ctx->start
	    = (unsigned int *)((char *)drawctxt->gpustate.hostptr + CMD_OFFSET);

	/* build indirect command buffers to save & restore regs/constants */
	build_regrestore_cmds(drawctxt, ctx);
	build_regsave_cmds(drawctxt, ctx);

	build_shader_save_restore_cmds(drawctxt, ctx);

	return 0;
}

/* create buffers for saving/restoring registers, constants, & GMEM */
static int
create_gmem_shadow(struct kgsl_device *device, struct kgsl_drawctxt *drawctxt,
		   struct tmp_ctx *ctx)
{
	unsigned int flags = KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN8K;

	config_gmemsize(ctx, device->gmemspace.sizebytes);
	ctx->gmem_base = device->gmemspace.gpu_base;

	/* allocate memory for GMEM shadow */
	if (kgsl_sharedmem_alloc(flags, ctx->gmem_size,
				 &drawctxt->gmemshadow) != 0)
		return -ENOMEM;

	if (kgsl_mmu_map(drawctxt->pagetable,
				drawctxt->gmemshadow.physaddr,
				drawctxt->gmemshadow.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
				&drawctxt->gmemshadow.gpuaddr)) {
		return -EINVAL;
	}

	/* we've allocated the shadow, when swapped out, GMEM must be saved. */
	drawctxt->flags |= CTXT_FLAGS_GMEM_SHADOW | CTXT_FLAGS_GMEM_SAVE;

	/* blank out gmem shadow. */
	kgsl_sharedmem_set(&drawctxt->gmemshadow, 0, 0, ctx->gmem_size);

	/* build quad vertex buffer */
	build_quad_vtxbuff(drawctxt, ctx);

	/* build indirect command buffers to save & restore gmem */
	build_gmem2sys_cmds(drawctxt, ctx);
	build_sys2gmem_cmds(drawctxt, ctx);

	return 0;
}

/* init draw context */

int kgsl_drawctxt_init(struct kgsl_device *device)
{
	return 0;
}

/* close draw context */
int kgsl_drawctxt_close(struct kgsl_device *device)
{
	return 0;
}

/* create a new drawing context */

int
kgsl_drawctxt_create(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable,
			unsigned int flags,
			unsigned int *drawctxt_id)
{
	struct kgsl_drawctxt *drawctxt;
	int index;
	struct tmp_ctx ctx;

	KGSL_CTXT_INFO("pt %p flags %08x\n", pagetable, flags);
	if (device->drawctxt_count >= KGSL_CONTEXT_MAX)
		return -EINVAL;

	/* find a free context slot */
	index = 0;
	while (index < KGSL_CONTEXT_MAX) {
		if (device->drawctxt[index].flags == CTXT_FLAGS_NOT_IN_USE)
			break;

		index++;
	}

	if (index >= KGSL_CONTEXT_MAX)
		return -EINVAL;

	drawctxt = &device->drawctxt[index];
	drawctxt->pagetable = pagetable;
	drawctxt->flags = CTXT_FLAGS_IN_USE;

	device->drawctxt_count++;

	if (create_gpustate_shadow(device, drawctxt, &ctx)
			!= 0) {
		kgsl_drawctxt_destroy(device, index);
		return -EINVAL;
	}

	/* Save the shader instruction memory on context switching */
	drawctxt->flags |= CTXT_FLAGS_SHADER_SAVE;

	if (!(flags & KGSL_CONTEXT_NO_GMEM_ALLOC)) {
		/* create gmem shadow */
		if (create_gmem_shadow(device, drawctxt, &ctx)
				!= 0) {
			kgsl_drawctxt_destroy(device, index);
			return -EINVAL;
		}
	}

	BUG_ON(ctx.cmd - ctx.start > CMD_BUFFER_LEN);

	*drawctxt_id = index;

	KGSL_CTXT_INFO("return drawctxt_id %d\n", *drawctxt_id);
	return 0;
}

/* destroy a drawing context */

int kgsl_drawctxt_destroy(struct kgsl_device *device, unsigned int drawctxt_id)
{
	struct kgsl_drawctxt *drawctxt;

	drawctxt = &device->drawctxt[drawctxt_id];

	KGSL_CTXT_INFO("drawctxt_id %d ptr %p\n", drawctxt_id, drawctxt);
	if (drawctxt->flags != CTXT_FLAGS_NOT_IN_USE) {
		/* deactivate context */
		if (device->drawctxt_active == drawctxt) {
			/* no need to save GMEM or shader, the context is
			 * being destroyed.
			 */
			drawctxt->flags &= ~(CTXT_FLAGS_GMEM_SAVE |
					     CTXT_FLAGS_SHADER_SAVE |
					     CTXT_FLAGS_GMEM_SHADOW |
					     CTXT_FLAGS_STATE_SHADOW);

			kgsl_drawctxt_switch(device, NULL, 0);
		}

		kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

		/* destroy state shadow, if allocated */
		if (drawctxt->gpustate.gpuaddr != 0) {
			kgsl_mmu_unmap(drawctxt->pagetable,
					drawctxt->gpustate.gpuaddr,
					drawctxt->gpustate.size);
			drawctxt->gpustate.gpuaddr = 0;
		}
		if (drawctxt->gpustate.physaddr != 0)
			kgsl_sharedmem_free(&drawctxt->gpustate);

		/* destroy gmem shadow, if allocated */
		if (drawctxt->gmemshadow.gpuaddr != 0)
			kgsl_mmu_unmap(drawctxt->pagetable,
					drawctxt->gmemshadow.gpuaddr,
					drawctxt->gmemshadow.size);
			drawctxt->gmemshadow.gpuaddr = 0;

		if (drawctxt->gmemshadow.physaddr != 0)
			kgsl_sharedmem_free(&drawctxt->gmemshadow);

		drawctxt->flags = CTXT_FLAGS_NOT_IN_USE;

		BUG_ON(device->drawctxt_count == 0);
		device->drawctxt_count--;
	}
	KGSL_CTXT_INFO("return\n");
	return 0;
}

/* switch drawing contexts */
void
kgsl_drawctxt_switch(struct kgsl_device *device, struct kgsl_drawctxt *drawctxt,
			unsigned int flags)
{
	struct kgsl_drawctxt *active_ctxt = device->drawctxt_active;

	/* already current? */
	if (active_ctxt == drawctxt)
		return;

	KGSL_CTXT_INFO("from %p to %p flags %d\n",
			device->drawctxt_active, drawctxt, flags);
	/* save old context*/
	if (active_ctxt != NULL) {
		KGSL_CTXT_INFO("active_ctxt flags %08x\n", active_ctxt->flags);
		/* save registers and constants. */
		KGSL_CTXT_DBG("save regs");
		kgsl_ringbuffer_issuecmds(device, active_ctxt, 0,
					  active_ctxt->reg_save, 3, flags);

		if (active_ctxt->flags & CTXT_FLAGS_SHADER_SAVE) {
			/* save shader partitioning and instructions. */
			KGSL_CTXT_DBG("save shader");
			kgsl_ringbuffer_issuecmds(device, active_ctxt, 1,
						  active_ctxt->shader_save, 3,
						  flags);

			/* fixup shader partitioning parameter for
			 *  SET_SHADER_BASES.
			 */
			KGSL_CTXT_DBG("save shader fixup");
			kgsl_ringbuffer_issuecmds(device, active_ctxt, 0,
						  active_ctxt->shader_fixup, 3,
						  flags);

			active_ctxt->flags |= CTXT_FLAGS_SHADER_RESTORE;
		}

		if (active_ctxt->flags & CTXT_FLAGS_GMEM_SAVE
			&& flags & KGSL_CONTEXT_SAVE_GMEM) {
			/* save gmem.
			 * (note: changes shader. shader must already be saved.)
			 */
			KGSL_CTXT_DBG("save gmem");
			kgsl_ringbuffer_issuecmds(device, active_ctxt, 1,
						  active_ctxt->gmem_save, 3,
						  flags);

			active_ctxt->flags |= CTXT_FLAGS_GMEM_RESTORE;
		}
	}

	device->drawctxt_active = drawctxt;

	if (drawctxt == NULL)
		kgsl_mmu_setpagetable(device, device->mmu.defaultpagetable);

	/* restore new context */
	if (drawctxt != NULL) {

		KGSL_CTXT_INFO("drawctxt flags %08x\n", drawctxt->flags);
		KGSL_CTXT_DBG("restore pagetable");
		kgsl_mmu_setpagetable(device, drawctxt->pagetable);

		/* restore gmem.
		 *  (note: changes shader. shader must not already be restored.)
		 */
		if (drawctxt->flags & CTXT_FLAGS_GMEM_RESTORE) {
			KGSL_CTXT_DBG("restore gmem");
			kgsl_ringbuffer_issuecmds(device, drawctxt, 1,
						  drawctxt->gmem_restore, 3,
						  flags);
			drawctxt->flags &= ~CTXT_FLAGS_GMEM_RESTORE;
		}

		/* restore registers and constants. */
		KGSL_CTXT_DBG("restore regs");
		kgsl_ringbuffer_issuecmds(device, drawctxt, 0,
					  drawctxt->reg_restore, 3, flags);

		/* restore shader instructions & partitioning. */
		if (drawctxt->flags & CTXT_FLAGS_SHADER_RESTORE)
			KGSL_CTXT_DBG("restore shader");
			kgsl_ringbuffer_issuecmds(device, drawctxt, 0,
						  drawctxt->shader_restore, 3,
						  flags);
	} else {
		KGSL_CTXT_DBG("restore default pagetable");
		kgsl_mmu_setpagetable(device, device->mmu.defaultpagetable);
	}
	KGSL_CTXT_INFO("return\n");
}
