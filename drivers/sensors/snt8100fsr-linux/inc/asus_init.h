#include "sonacomm.h"

#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
//#include <linux/wakelock.h>
#include "grip_Wakelock.h"
#include "locking.h"
#include "file_control.h"
#include <linux/of_gpio.h>

#define asus_grip_queue "snt8100fsr-asus_queue"

#define GRIP_PM8150_GPIO4_LOOKUP_STATE	"grip_clk32"

extern void snt_set_pinctrl(struct device *dev, char *str);
extern void set_1V2_2V8_pin_func(struct work_struct *work_orig);
extern void asus_init_probe(void);
extern void check_i2c_error(void);

