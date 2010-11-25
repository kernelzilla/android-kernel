/* timed_vibrator.h
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

#ifndef _TIMED_VIBRATOR_H
#define _TIMED_VIBRATOR_H


struct vibrator_platform_data {
	const char *name;
	unsigned    gpio_en;
	unsigned    gpio_pwm;
        int         hz;
	unsigned char 	   active_low;
	int         max_timeout;
};

#define STIMULUS_A_VIBRATOR_START_HZ 163
#define STIMULUS_A_VIBRATOR_END_HZ 178
#define STIMULUS_A_VIBRATOR_SWEEP_TIME_MS 200
#define STIMULUS_B_VIBRATOR_HZ 175

#endif
