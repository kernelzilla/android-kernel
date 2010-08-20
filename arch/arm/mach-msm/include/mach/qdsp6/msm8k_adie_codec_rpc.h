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

#include <linux/kernel.h>
#include "../dal.h"

#define DALDEVICEID_ADIE_CODEC 0x02000029
#define ADIE_CODEC_PORT_NAME "DAL_AM_AUD"

/* need to confirm the CPU # of Arm9 and change
 * the below define accordingly
 */
#define ADIE_CODEC_CPU SMD_APPS_MODEM
#define ADIE_CODEC_DUMMY_ARG (u32)0
#define ADIE_CODEC_DUMMY_PTR_ARG (void *)NULL


/** These are the predefined bringup/teardown hooks
 *  The numerical value of these is important
 *  Example could be:
 *  DAL_AdieCodec_ProceedToStage(ClientCtxt,DIGITAL_READY)
 */

/** This value is the state after the client sets the path */
#define ADIE_CODEC_PATH_OFF                                        0x0050
/** State to which client asks the drv to proceed to where it can
 * set up the clocks and 0-fill PCM buffers */

#define ADIE_CODEC_DIGITAL_READY                                   0x0100
/** State to which client asks the drv to proceed to where it can
 *  start sending data after internal steady state delay */

#define ADIE_CODEC_DIGITAL_ANALOG_READY                            0x1000

/** Client Asks adie to switch off the Analog portion of the
 *  the internal codec. After the use of this path
 * */
#define ADIE_CODEC_ANALOG_OFF                                      0x0750

/** Client Asks adie to switch off the digital portion of the
 *  the internal codec. After switching off the analog portion.
 *
 *  0-fill PCM may or maynot be sent at this point
 * */
#define ADIE_CODEC_DIGITAL_OFF                                     0x0600


enum {
	ADIE_CODEC_GET_NUM_PATHS = DALDEVICE_FIRST_DEVICE_API_IDX,
	ADIE_CODEC_GET_ALL_PATH_IDS,
	ADIE_CODEC_SET_PATH,
	ADIE_CODEC_GET_NUM_PATH_FREQUENCY_PLANS,
	ADIE_CODEC_GET_PATH_FREQUENCY_PLANS,
	ADIE_CODEC_SET_PATH_FREQUENCY_PLAN,
	ADIE_CODEC_PROCEED_TO_STAGE,
	ADIE_CODEC_MUTE_PATH
};

enum _adie_codec_mute_state_enum {
	ADIE_CODEC_UNMUTED = 0, /**< ADIE_CODEC_UNMUTED :- Codec path is
				not muted*/
	ADIE_CODEC_MUTED        /**< ADIE_CODEC_MUTED :- Codec path is muted*/
};

/** Enumeration that represent the standard value of available path
 * types
 */
enum adie_codec_path_type_enum{
	ADIE_CODEC_RX = 0,  /**< ADIE_CODEC_RX :- Path Type is Rx*/
	ADIE_CODEC_TX,      /**< ADIE_CODEC_TX :- Path Type is Tx*/
	ADIE_CODEC_LB       /**< ADIE_CODEC_LB :- Path Type is Loopback*/
};

/** Enumeration that represents the Adie DAL Error Codes
 * This will be returned as a s32.
 *
 * @note Developers are free to use any value over 512
 *  as a client specific s32.
 */
enum adie_codec_error_code {
	ADIE_CODEC_E_INCOMPLETE = 513,	/**< Results are not complete
					   call the function again after
					   rectifying the error*/
	ADIE_CODEC_E_NO_SUCH_PATH,      /**< PathId does not exist*/
	ADIE_CODEC_E_NO_MORE_HANDLES,   /**< We currently only allow one client
					 if that client is occupied and another
					 "client" calls open this would be
					returned */
	ADIE_CODEC_E_NO_SUCH_FREQ,      /**< The frequency plan for that path
					does notinclude the frequency that the
					user sets*/
	ADIE_CODEC_E_NO_SUCH_STAGE,	/**< The stage given for this path is
					incorrect*/
	ADIE_CODEC_E_CONCURRENCY,       /**< There has been a concurrency error,
					for example: Client tried to initialize
					another Rx Path when one was already
					available.*/
	ADIE_CODEC_E_DATABASE,          /**< There has been a database error*/
	ADIE_CODEC_E_PARAM_ERROR,       /**< General Parameter error*/
	ADIE_CODEC_E_PATH_UNSET,        /**< No Path has been set*/
	ADIE_CODEC_E_EMPTY              /**< The query returned no results */
};


u32 adie_codec_path_id;
u32 adie_codec_path_type;
u32 adie_codec_path_stage;
u32 adie_codec_path_freq_plan;
u32 adie_codec_mute_state;

int rpc_adie_codec_get_num_paths(void *handle, u32 *numpath)
{
	return dalrpc_fcn_2(ADIE_CODEC_GET_NUM_PATHS, handle,
		ADIE_CODEC_DUMMY_ARG, numpath);
}

int rpc_adie_codec_get_all_path_ids(void *handle,
				u32 *ids,
				u32 array_in_size,
				u32 *array_out_size)
{
	return dalrpc_fcn_12(ADIE_CODEC_GET_ALL_PATH_IDS,
			handle,
			ADIE_CODEC_DUMMY_ARG,
			(void *)ids,
			array_in_size,
			array_out_size);
}

int rpc_adie_codec_set_path(void *handle, u32 id,
				u32 path_type)
{
	return dalrpc_fcn_1(ADIE_CODEC_SET_PATH, handle, id, path_type);
}

int rpc_adie_codec_get_num_path_freq_plans(void *handle,
				u32 id,
				u32 path_type,
				u32 *num_plans)
{
	return dalrpc_fcn_4(ADIE_CODEC_GET_NUM_PATH_FREQUENCY_PLANS,
			handle,
			id,
			path_type,
			num_plans);
}

int rpc_adie_codec_get_path_freq_plans(
					void *handle,
					u32 id,
					u32 path_type,
					u32 *plans,
					u32 array_in_size,
					u32 *array_out_size)
{
	return dalrpc_fcn_14(ADIE_CODEC_GET_PATH_FREQUENCY_PLANS,
			handle,
			(void *)&id,
			sizeof(u32),
			(void *)&path_type,
			sizeof(u32),
			(void *)plans,
			array_in_size,
			array_out_size);
}


int rpc_adie_codec_set_path_freq_plan(void *handle,
					u32 path_type,
					u32 plan)
{
	return dalrpc_fcn_1(ADIE_CODEC_SET_PATH_FREQUENCY_PLAN, handle,
		path_type, plan);
}

int rpc_adie_codec_proceed_to_stage(void *handle,
					u32 path_type,
					u32 stage)
{
	return dalrpc_fcn_1(ADIE_CODEC_PROCEED_TO_STAGE,
		handle, path_type, stage);
}

int rpc_adie_codec_mute_path(void *handle,
					u32 path_type,
					u32 mute_state)
{
	return dalrpc_fcn_1(ADIE_CODEC_MUTE_PATH, handle, path_type,
				mute_state);
}
