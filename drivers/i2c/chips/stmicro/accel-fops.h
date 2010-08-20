/**
 *	Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation version 2 of the License.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	GNU General Public License <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#ifndef ACCEL_FOPS_H_
#define ACCEL_FOPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_FF_ALLOW_CLIENT	0x0000001
#define ACCEL_FF_IS_CLIENT	0x0000010
#define ACCEL_FF_IS_VALID	0x0000020
#define ACCEL_FF_SUSPENDED	0x0000040

#define ACCEL_ST_DATA_READY	0x1000000	/* data ready */
#define ACCEL_ST_READY_EVENT	0x2000000	/* event data ready */

#define TOGGLE_ON		1
#define TOGGLE_OFF		0

#define ACCEL_BITS_IN_ULONG	31
#define ACCEL_BITS_IN_UCHAR	7


typedef struct file_private {
	struct file_private *prev;
	struct file_private *next;
	pid_t mypid;			/* pid of the client called open */
	wait_queue_head_t readq;	/* readers wait queue */
	volatile unsigned long access_flags;	/* fd access flags */
	volatile unsigned long events_mask;
	unsigned status;		/* status flags */
	struct input_event iev;		/* input event structure to hold data */
        int nreaders, nwriters;         /* number of openings for r/w */
        struct fasync_struct *asyncq; 	/* asynchronous readers */
        struct semaphore sem;           /* mutual exclusion semaphore */
} accel_file_private_t;


typedef struct client_private {
	struct client_private *prev;
	struct client_private *next;
	pid_t pid;
	accel_file_private_t **file_private;
} accel_client_t;


extern struct file_operations accel_fops;
extern u8 portrait[], landscape[];

extern accel_file_private_t *accel_find_private_by_pid (pid_t pid);
extern void accel_clients_notify (unsigned long event, struct input_event *data);


#ifdef __cplusplus
}
#endif
#endif
