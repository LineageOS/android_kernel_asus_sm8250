#include <linux/fcntl.h> 
//#include <stdio.h>
//#include <stdlib.h>
#include <linux/unistd.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/hid.h>
#include <linux/semaphore.h>

//For drm panel notify
#include <drm/drm_panel.h>
#include <linux/notifier.h>
#include <linux/fb.h>
static struct drm_panel *active_panel;
static bool drm_registered;

//Include extcon register
#include <../extcon/extcon.h>

//For HID wait for completion
#include <linux/completion.h>

#define	CLASS_NAME		    "ec_hid"
#define	TEST_STRING		    "ROG_DONGLE"
//#define HID_PATCH			"/dev/hidraw0"

enum asus_dongle_type
{
	Dongle_NO_INSERT = 0,
	Dongle_INBOX3,
	Dongle_Station2,
	Dongle_DT1,
	Dongle_PCIE,
	Dongle_ERROR,
	Dongle_Others,
	Dongle_default_status = 255,
};

/*
 * 	gDongleEvent : only for Station ( gDongleType == 2 )
 *
 * 	0 	: Normal mode
 * 	1 	: Upgrade mode
 * 	2 	: Low Battery mode
 * 	3 	: ShutDown & Virtual remove mode
 */
enum asus_DongleEvent_type
{
	DongleEvent_Normal_mode = 0,
	DongleEvent_Upgrade_mode,
	DongleEvent_LowBattery_mode,
	DongleEvent_shutdown_remove_mode,
};

extern struct hidraw *g_hidraw;
static struct class *ec_hid_class;

struct ec_hid_data *g_hid_data;
EXPORT_SYMBOL(g_hid_data);

bool station_shutdown = false;
EXPORT_SYMBOL(station_shutdown);

// Block HID input report
bool block_hid_input = false;
EXPORT_SYMBOL(block_hid_input);

bool is_ec_adc_tm5_run = false;

// Record HID used status
bool hid_used = false;
EXPORT_SYMBOL(hid_used);

// Register Hall sensor
bool suspend_by_hall = false;
EXPORT_SYMBOL(suspend_by_hall);

// Sync update status with Station touch
int station_touch_recovery = 0;
EXPORT_SYMBOL(station_touch_recovery);

struct ec_hid_data {
	dev_t devt;
	struct device *dev;
	
	uint8_t previous_event;

	u8 fw_version;

	int pogo_det;
	int pogo_sleep;
	int pogo_fan_on;
	int pogo_aura_en;
	int pogo_temp_intr;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_default;

	bool lock;
	struct mutex report_mutex;
	struct mutex pogo_mutex;
	struct semaphore pogo_sema;
	struct mutex pogo_id_mutex;

	struct notifier_block display_notifier;  //For drm panel notify
};

void hid_switch_usb_autosuspend(bool flag){
	struct hid_device *hdev;
	struct usb_interface *intf;

	if (g_hidraw == NULL || g_hid_data->lock) {
		//printk("[EC_HID] g_hidraw is NULL or lock %d\n", g_hid_data->lock);
		return;
	}

	hdev = g_hidraw->hid;
	intf = to_usb_interface(hdev->dev.parent);

	printk("[EC_HID] hid_swithc_usb_autosuspend %d\n", flag);
	if(flag) {
		usb_enable_autosuspend(interface_to_usbdev(intf));
	}else {
		usb_disable_autosuspend(interface_to_usbdev(intf));
	}

	return;
}
EXPORT_SYMBOL_GPL(hid_switch_usb_autosuspend);

static int check_panel(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			printk("[EC_HID] check_panel success, set active_panel!!!\n");
			active_panel = panel;
			return 0;
		}
	}

	return -ENODEV;
}
