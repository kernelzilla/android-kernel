/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/
#ifndef __SMS_CHAR_IOCTL_H__
#define __SMS_CHAR_IOCTL_H__

#include <linux/ioctl.h>

struct smschar_buffer_t {
	unsigned long offset;	/* offset in common buffer (mapped to user) */
	int size;
};

struct smschar_get_fw_filename_ioctl_t {
	int mode;
	char filename[200];
};

struct smschar_send_fw_file_ioctl_t {
	char *fw_buf;
	int fw_size;
};

#define SMSCHAR_SET_DEVICE_MODE		_IOW('K', 0, int)
#define SMSCHAR_GET_DEVICE_MODE		_IOR('K', 1, int)
#define SMSCHAR_GET_BUFFER_SIZE		_IOR('K', 2, int)
#define SMSCHAR_WAIT_GET_BUFFER		_IOR('K', 3, struct smschar_buffer_t)
#define SMSCHAR_IS_DEVICE_PNP_EVENT 	_IOR('K', 4, int)
#define SMSCHAR_GET_FW_FILE_NAME	\
	_IOWR('K', 5, struct smschar_get_fw_filename_ioctl_t)
#define SMSCHAR_SEND_FW_FILE		\
	_IOW('K', 6, struct smschar_send_fw_file_ioctl_t)
#define SMSCHAR_CANCEL_WAIT_BUFFER	_IO('K', 7)

/*##w21558, SMS1130 power control*/
#define SMSCHAR_SET_POWER_UP   _IOW('K', 8, int)
#define SMSCHAR_SET_POWER_DOWN _IOW('K', 9, int)
/*##w21558, to reset device mode*/
#define SMSCHAR_RESET_DEVICE_MODE _IOW('K', 10, int)

#endif /* __SMS_CHAR_IOCTL_H__ */
