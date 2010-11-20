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

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "logger.h"

#define MODULE_NAME "logger_test"

/* 1/2 second for timer delay */
#define TIMER_DELAY_JIFFIES (msecs_to_jiffies(500))
/* 1/2 second for timer delay */
#define THREAD_SLEEP_TIME_JIFFIES (msecs_to_jiffies(500))
#define LOGGER_TEST_PRIORITY (LOG_PRIORITY_SILENT)

static struct task_struct *kthread;
static struct timer_list my_timer;

static void timer_func(unsigned long ptr)
{
	static int counter;
	int ret = logger_write(LOG_RADIO_IDX,
		LOGGER_TEST_PRIORITY,
		"MYTAG-INTERRUPT",
		"From timer interrupt: %d\n",
		counter);
	if (ret)
		printk(KERN_ERR "interrupt %d logger write ret: %d\n",
			counter, ret);

	counter++;
	my_timer.expires = jiffies + TIMER_DELAY_JIFFIES;
	add_timer(&my_timer);
}

static int thread(void *n)
{
	int counter = 0;
	do {
		int ret;

		schedule_timeout_interruptible(THREAD_SLEEP_TIME_JIFFIES);
		ret = logger_write(LOG_RADIO_IDX,
			LOGGER_TEST_PRIORITY,
			"MYTAG-THREAD",
			"From Thread: %d\n",
			counter);
		if (ret)
			printk(KERN_ERR "thread %d logger write ret: %d\n",
				counter, ret);
		counter++;
	} while (!kthread_should_stop());
	return 0;
}

static int __init logger_test_init(void)
{
	int junk = 25;
	int ret;

	/* start kernel thread */
	kthread = kthread_run((void *)thread, NULL, MODULE_NAME"_thread");
	if (IS_ERR(kthread)) {
		printk(KERN_INFO MODULE_NAME
			": unable to start kernel thread\n");
		return -ENOMEM;
	}

	init_timer(&my_timer);
	my_timer.function = timer_func;
	my_timer.data = 0;
	my_timer.expires = jiffies + TIMER_DELAY_JIFFIES;
	add_timer(&my_timer);

	ret = logger_write(LOG_RADIO_IDX,
		LOG_PRIORITY_DEBUG,
		"MYTAG",
		"This should not be present in the log, %d\n",
		junk++);
	if (ret)
		printk(KERN_ERR "logger write 1 returned %d\n", ret);


	ret = logger_write(LOG_RADIO_IDX,
		LOGGER_TEST_PRIORITY,
		"MYTAG",
		"This >should< be present in the log, %d\n",
		junk++);
	if (ret)
		printk(KERN_ERR "logger write 2 returned %d\n", ret);

	ret = logger_write(LOG_RADIO_IDX,
		LOGGER_TEST_PRIORITY,
		"MYTAG1",
		"This >should< be present in the log with a new tag, %d\n",
		junk++);
	if (ret)
		printk(KERN_ERR "logger write 3 returned %d\n", ret);

	return 0;
}

static void __exit logger_test_exit(void)
{
	del_timer_sync(&my_timer);
	kthread_stop(kthread);
}

MODULE_LICENSE("Dual BSD/GPL");

device_initcall(logger_test_init);
module_exit(logger_test_exit);
