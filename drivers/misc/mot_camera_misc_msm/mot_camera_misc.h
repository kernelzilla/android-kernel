//#if defined (CONFIG_MOT_KERNEL)

/*

 JK: a more appropriate header will be added later

 Revision History:

 Modification                         Tracking
 Author                 Date          Number     Description of Changes
 Joe Kreidler (qa1818)  23Mar2009                Initial Version
*/

#ifndef _CAMERA_MISC_H
#define _CAMERA_MISC_H

#define CAMERA_DATA_ARRAY_LEN       4

enum {
    CAMERA_RESET_WRITE     = 100,
    CAMERA_POWERDOWN_WRITE = 101,
    CAMERA_CLOCK_DISABLE   = 102,
    CAMERA_CLOCK_ENABLE    = 103,
};

#define CAMERA_DEVICE0  "/dev/camera0"

#endif /* _CAMERA_MISC_H */


//#endif
