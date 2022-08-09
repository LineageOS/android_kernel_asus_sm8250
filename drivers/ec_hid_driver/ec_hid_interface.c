/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 * Edit by ASUS Deeo, deeo_ho@asus.com
 * V3
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hidraw.h>
#include <linux/usb.h>

//ASUS_BSP Deeo : EXPORT g_hidraw for JEDI dongle driver
struct hidraw *g_hidraw;
EXPORT_SYMBOL_GPL(g_hidraw);

//ASUS_BSP : add for check ec init state +++
#include <linux/delay.h>

extern bool hid_used;
extern void ec_hid_uevent(void);
extern int ec_mutex_lock(char *);
extern int ec_mutex_unlock(char *);
extern bool ENE_upgrade_mode;
//ASUS_BSP : add for check ec init state ---

extern uint8_t gDongleType;

#ifdef CONFIG_PM
static int ec_usb_resume(struct hid_device *hdev)
{
	return 0;
}

static int ec_usb_suspend(struct hid_device *hdev, pm_message_t message)
{
	return 0;
}
#endif /* CONFIG_PM */

static int ec_usb_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	return 0;
}

static int ec_usb_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *intf;
	int ret;
	unsigned int cmask = HID_CONNECT_DEFAULT;

	printk("[EC_USB] ec_usb_probe\n");
	printk("[EC_USB] hid->name : %s\n", hdev->name);
	printk("[EC_USB] hid->vendor  : 0x%x\n", hdev->vendor);
	printk("[EC_USB] hid->product : 0x%x\n", hdev->product);
	ASUSEvtlog("[EC_USB] Station ENE 6K7750 connect\n");

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "[EC_USB] parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, cmask);
	if (ret) {
		hid_err(hdev, "[EC_USB] hw start failed\n");
		goto err_free;
	}

	if (hdev->product == 0x7750)
		ENE_upgrade_mode = false;
	else
		ENE_upgrade_mode = true;

	if (ENE_upgrade_mode){
		printk("[EC_USB] In Upgrade mode, trigger update UI.");
		ec_hid_uevent();
		printk("[EC_USB] ec_usb_probe : %d\n", ENE_upgrade_mode);
		return 0;
	}

	printk("[EC_USB] usb_disable_autosuspend\n");
	intf = to_usb_interface(hdev->dev.parent);
	usb_disable_autosuspend(interface_to_usbdev(intf));

	g_hidraw = hdev->hidraw;

	return 0;
	
err_free:
	printk("[EC_USB] ec_usb_probe fail.\n");
	return ret;
}

static void ec_usb_remove(struct hid_device *hdev)
{
	printk("[EC_USB] ec_usb_remove\n");
	//ASUSEvtlog("[EC_HID] Station ITE 8910 disconnect!!!\n");

	ec_mutex_lock("hidraw");
	printk("[EC_HID] hid_used is %d\n", hid_used);
	g_hidraw = NULL;	//ASUS_BSP Deeo : clean g_hidraw for JEDI dongle driver
	hid_used = false;
	ec_mutex_unlock("hidraw");

	printk("[EC_USB] hid_hw_stop\n");
	hid_hw_stop(hdev);

	if (hdev->product != 0x7750)
		gDongleType = 0;

	ENE_upgrade_mode = false;
}

static struct hid_device_id ec_idtable[] = {
	{ HID_USB_DEVICE(0x0CF2, 0x7750),
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x0CF2, 0x7758),
		.driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, ec_idtable);

static struct hid_driver ec_usb_driver = {
	.name		= "ec_hid_interface",
	.id_table		= ec_idtable,
	.probe			= ec_usb_probe,
	.remove			= ec_usb_remove,
	.raw_event		= ec_usb_raw_event,
#ifdef CONFIG_PM
	.suspend          = ec_usb_suspend,
	.resume			= ec_usb_resume,
#endif
};

static int __init ec_usb_init(void)
{
	printk("[EC_USB] ec_usb_init\n");

	return hid_register_driver(&ec_usb_driver);
}

static void __exit ec_usb_exit(void)
{
	printk("[EC_USB] ec_usb_exit\n");

	hid_unregister_driver(&ec_usb_driver);
}
module_init(ec_usb_init);
module_exit(ec_usb_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("JEDI dongle EC USB driver");
MODULE_LICENSE("GPL v2");
