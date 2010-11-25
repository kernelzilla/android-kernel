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
#include <linux/errno.h>
#include <mach/pmic.h>
#include <mach/camera.h>

#if CONFIG_MACH_MOT
// MOTOROLA : Camera LED Class driver interfaces
extern int cam_led_burst(void);
extern int cam_led_torch(void);
extern int cam_led_timed_torch(unsigned msec);
extern int cam_led_off(void);

// Used to eliminate LED "off" gap between torch & burst
static int extend_torch = 0; 
#endif

int32_t msm_camera_flash_set_led_state(unsigned led_state)
{
	int32_t rc;

  CDBG("flash_set_led_state: %d\n", led_state);
  switch (led_state) {
#ifdef CONFIG_MACH_MOT
  case MSM_LED_OFF:
    // MOTOROLA : Disable current sources via Camera LED class driver
    if (extend_torch) {
        // Now extend the torch interval (by 500 msec).
        rc = cam_led_timed_torch(500);

        // Clear the flag to avoid re-extending (until the next torch)
        extend_torch = 0;
    } else {
        // In this case, we really want the LED off
    rc = cam_led_off();
    }
#else
  case MSM_CAMERA_LED_OFF:
    rc = flash_led_set_current(0);
#endif
    break;

#if CONFIG_MACH_MOT
  case MSM_LED_LOW:
    // MOTOROLA : Enable low current source via Camera LED class driver 
    rc = cam_led_torch();

    // Set the flag indicating torch mode should be extended when the 
    // ISP tells us to turn it off.  (We must eliminate the ~500 msec
    // LED "off" interval between torch & burst modes.)
    extend_torch = 1;
#else
  case MSM_CAMERA_LED_LOW:
    rc = flash_led_set_current(30);
#endif
    break;

#if CONFIG_MACH_MOT
  case MSM_LED_HIGH:
    // MOTOROLA : Enable high current source via Camera LED class driver 
    rc = cam_led_burst();

    // Clear the flag to ensure that we don't extend the burst mode.
    extend_torch = 0;
#else
  case MSM_CAMERA_LED_HIGH:
    rc = flash_led_set_current(100);
#endif
    break;

  default:
    rc = -EFAULT;
    break;
  }
  CDBG("flash_set_led_state: return %d\n", rc);

  return rc;
}
