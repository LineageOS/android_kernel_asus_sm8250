/*
 * Dummy Charger driver for test RT1711
 *
 * Copyright (c) 2012 Marvell International Ltd.
 * Author:	Jeff Chang <jeff_chagn@mrichtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/usb/tcpci.h>
#include <linux/workqueue.h>
#include <linux/usb/rt_tcpm.h>
#include <linux/device.h>
#include <linux/usb/pd_policy_engine.h>
#include <linux/usb.h>
#include "supply/qcom/smb5-lib.h"
#include <linux/wakelock.h>

extern struct smb_charger *smbchg_dev;

int test_flag;
int gamepad_active = 0;
bool BTM_a2c_cable = false;
static int tcpc_swap_state;
static int tcpc_typec_state;
static int tcpc_pre_typec_state;
extern int asus_request_BTM_otg_en(int enable);
extern int asus_request_DPDM_flag(int enable);
extern int asus_set_invalid_audio_dongle(int set);
#ifdef CONFIG_ASUS_PD_CHARGER
extern int rt_charger_set_usb_property_notifier(enum power_supply_property psp, int value);
#endif
extern int get_net_status(void);
extern int get_prodock_state (void);
extern int get_DT_state(void);
extern void asus_request_SIDE_otg_en(int enable);
extern void DT_active_mode(bool active);
extern void BTM_host_notify(int enable);
extern bool g_hpd;
extern uint8_t gDongleType;
extern uint16_t vid_ext;
extern int aura_screen_on;
static int tcpc_sink_voltage;
static int tcpc_sink_current;
static int isPD2active = 0;
static int usb1_active=0;
static int usb2_active=0;
static int usb2_stopPower=0;
static int usb2_stopPower_screen=0;
static int usb2_stopPower_screen_DT=0;
static int usb1_stopPower_screen=0;
static int pro_dock_active_side = 0;
static int pro_dock_active_bottom = 0;
static int gamevice_active = 0;
static int btm_vid=0, btm_pid=0;
static int lastcall = 0;
static int need_extcon_host_sync = 0;
uint16_t vid=0;
struct notifier_block	host_nb;
struct delayed_work close5vwork;
struct workqueue_struct *close5v_wq;
static void close5v_by_suspend(struct work_struct *work);
struct delayed_work open5vwork;
struct workqueue_struct *open5v_wq;
static void open5v_by_suspend(struct work_struct *work);
struct wake_lock rt_wake_lock;

struct rt_charger_info {
	struct device *dev;
	struct power_supply *chg;
	struct notifier_block nb;
	struct tcpc_device *tcpc;
	struct delayed_work dwork;
	int vbus_gpio;
	u8 status;
	u8 online:1;

	bool pd_apdo_connected;
	bool pd_start_direct_charging;
	bool peer_usb_comm;
};

int rt_chg_connect_none(struct notifier_block *nb) {
#ifdef CONFIG_DUAL_PD_PORT
	struct power_supply *psy_dc;
	union power_supply_propval val = {0};
    struct rt_charger_info *info =
		container_of(nb, struct rt_charger_info, nb);

	printk("[Bottom_PD] Enter pe_idle2_entry\n");
	printk("[Bottom_PD] Enter stop direct charging\n");

	/* When pd charger disconnected */
	/* Stop the direct charging */
	info->pd_apdo_connected = false;
	if (info->pd_start_direct_charging == true) {
		printk("[Bottom_PD] Stop PCA9468 charging\n");
		/* Stop Direct Charging */
		/* Get power supply name */
		psy_dc = power_supply_get_by_name("pca9468-mains");
		if (psy_dc == NULL) {
			printk("[Bottom_PD] Error Get the direct charing psy\n");
		} else {
			/* Set the disable direct charging */
			val.intval = POWER_SUPPLY_CHARGING_DISABLED_RT;
			power_supply_set_property(psy_dc, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

			/* Set PD port to None */
			val.intval = POWER_SUPPLY_PD_PORT_NONE_RT;
			power_supply_set_property(psy_dc, POWER_SUPPLY_PROP_PD_PORT, &val);
		}
	}
	info->pd_start_direct_charging = false;
#endif
	    return 0;
}

int rt_chg_hard_reset(struct notifier_block *nb) {
#ifdef CONFIG_DUAL_PD_PORT
	struct power_supply *psy_dc;
	union power_supply_propval val = {0};

    struct rt_charger_info *info =
		container_of(nb, struct rt_charger_info, nb);

	printk("[Bottom_PD] Enter pe_snk_wait_for_capabilities_entry\n");
	printk("[Bottom_PD] Enter stop direct charging\n");
	/* When PD Hardreset happened or start PD at the first time */
	/* Stop the direct charging */
	info->pd_apdo_connected = false;
	if (info->pd_start_direct_charging == true) {
		printk("[Bottom_PD] Stop PCA9468 charging\n");
		/* Stop Direct Charging */
		/* Get power supply name */
		psy_dc = power_supply_get_by_name("pca9468-mains");
		if (psy_dc == NULL) {
			printk("[Bottom_PD] Error Get the direct charing psy\n");
		} else {
			/* Set the enable direct charging */
			val.intval = POWER_SUPPLY_CHARGING_DISABLED_RT;
			power_supply_set_property(psy_dc, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		}
	}
	info->pd_start_direct_charging = false;
#endif
	    return 0;
}

int rt_chg_ready_snk_apdo(struct notifier_block *nb) {
#ifdef CONFIG_DUAL_PD_PORT
    struct power_supply *usb_psy;
	struct power_supply *psy_dc;
    union power_supply_propval val = {0};

    struct rt_charger_info *info =
		container_of(nb, struct rt_charger_info, nb);

	printk("[Bottom_PD] Enter pe_snk_ready_entry\n");

	if (info->pd_apdo_connected == false) {

			printk("[Bottom_PD] Detect Augmented PDO\n");
			/* If PPS TA is connected, start the direct charging and prevent user space from controling TA */
			val.intval = 0;
			usb_psy = power_supply_get_by_name("usb");
			power_supply_set_property(usb_psy, POWER_SUPPLY_PROP_PD_ALLOWED, &val);
			info->pd_apdo_connected = true;

			/* Get power supply name */
			psy_dc = power_supply_get_by_name("pca9468-mains");
			if (psy_dc == NULL) {
				printk("[Bottom_PD] Error Get the direct charing psy\n");
			} else {
				/* If PPS TA is connected, set the RT1715 PD port */
				val.intval = POWER_SUPPLY_PD_PORT_RT;
				printk("[Bottom_PD] POWER_SUPPLY_PROP_PD_PORT = %d",POWER_SUPPLY_PD_PORT_RT);
				/* Set pd port property */
				power_supply_set_property(psy_dc, POWER_SUPPLY_PROP_PD_PORT, &val);
			}

	}

	if (( info->pd_apdo_connected == true) && (info->pd_start_direct_charging ==false)) {
		/* Get power supply name */
		psy_dc = power_supply_get_by_name("pca9468-mains");
		if (psy_dc == NULL) {
			printk("[Bottom_PD] Error Get the direct charing psy\n");
		} else {
			/* Set the enable direct charging */
			val.intval = POWER_SUPPLY_CHARGING_ENABLED_RT;
			printk("[Bottom_PD] Enter power_supply_set_property at pe_snk_ready_entry\n");
			power_supply_set_property(psy_dc, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
			info->pd_start_direct_charging = true;
		}
	}
#endif
    return 0;
}

int rt_chg_get_vid(void) {
    //int i;
    int ret;
    struct tcpc_device *tcpc;
    uint32_t vdos[VDO_MAX_NR];

    tcpc = tcpc_dev_get_by_name("type_c_port0");

    ret = tcpm_inquire_pd_partner_inform(tcpc, vdos);
	if(ret < 0) {
		pr_info("%s: Get vid fail, error number = %d\n", __func__, ret);
		return ret;
	}

	//for (i = 0; i < VDO_MAX_NR; i++)
		//pr_info("%s vdos[%d] = 0x%08x\n", __func__,
				//i, vdos[i]);
	//pr_info("%s: \n", __func__);

	vid = vdos[0] & 0xFFFF;
	pr_info("%s: vid = 0x%04x\n", __func__, vid);

    return vid;
}
EXPORT_SYMBOL(rt_chg_get_vid);

bool rt_chg_check_asus_vid(void) {
	pr_info("%s: vid = 0x%04x, vid_ext = 0x%04x\n", __func__, vid, vid_ext);
	if(vid==2821 || vid_ext==2821)
		return true;
	else
		return false;
}

int rt_chg_get_curr_state(void) {
	int curr;
    struct tcpc_device *tcpc;
    tcpc = tcpc_dev_get_by_name("type_c_port0");

    curr = tcpm_inquire_typec_remote_rp_curr(tcpc);
    pr_info("%s: curr = %d\n", __func__, curr);
    return curr;
}
EXPORT_SYMBOL(rt_chg_get_curr_state);

int rt_chg_get_during_swap(void) {
    struct tcpc_device *tcpc;
    tcpc = tcpc_dev_get_by_name("type_c_port0");

    pr_info("%s: during swap = %d\n", __func__, tcpc->pd_port.pe_data.during_swap);
    return tcpc->pd_port.pe_data.during_swap;
}
EXPORT_SYMBOL(rt_chg_get_during_swap);

int typec_disable_function(bool disable) {
    struct tcpc_device *tcpc;
    tcpc = tcpc_dev_get_by_name("type_c_port0");

    tcpm_typec_disable_function(tcpc, disable);
    pr_info("%s enter, disable= %d\n", __func__, disable);
    return 0;
}
EXPORT_SYMBOL(typec_disable_function);

void set_pd2_active(int active){
	pr_info("[Bottom_PD] set_pd2_active = %d\n", active);
	isPD2active = active;
	rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD2_ACTIVE, active);
}

//int tcpc_check_vsafe0v(struct tcpc_device *tcpc)
//{
	//pr_info("%s !!!!!!!!!!Please inpement check vsafe0v function !!!!!!!!!\n", __func__);
	//return 0;
//}

//static int chg_enable_vbus(struct rt_charger_info *info, int enable)
//{
	////pd_dbg_info("%s enable = %d\n", __func__, enable);
	//gpio_set_value(info->vbus_gpio, enable);
	//info->status = enable ?
		//POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
	//return 0;
//}

static int rt_chg_handle_sink_vbus(struct tcp_notify *tcp_noti)
{
	tcpc_sink_voltage = tcp_noti->vbus_state.mv;
	tcpc_sink_current = tcp_noti->vbus_state.ma;
	pr_info("%s tcpc_sink_voltage = %d mV, tcpc_sink_current = %d mA\n", __func__,tcpc_sink_voltage, tcpc_sink_current);
	rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, tcpc_sink_voltage*1000);
	rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD_CURRENT_MAX, tcpc_sink_current*1000);
	return 0;
}

static int rt_chg_handle_source_vbus(struct tcp_notify *tcp_noti)
{
	bool enable = (tcp_noti->vbus_state.mv > 0) ? true : false;
	int ret;

	/* if vbus boost comes from charger ic */
	ret = asus_request_BTM_otg_en(enable && !usb2_stopPower && !usb2_stopPower_screen);
	pr_info("%s asus_request_BTM_otg_en = %d\n", __func__, enable);
	if(ret)
	pr_err("%s failed\n",__func__);

	return 0;
}

//static int rt_chg_handle_source_vbus(struct tcp_notify *tcp_noti, int enable)
//{
	//struct power_supply *chg;
	//struct tcpc_device *tcpc;
	//union power_supply_propval val;

	//chg = power_supply_get_by_name("rt-chg");
	//if (!chg) {
		//pr_err("%s: no rt-charger psy\n", __func__);
		//return -ENODEV;
	//}

	//val.intval = enable;
	//chg->desc->set_property(chg, POWER_SUPPLY_PROP_ONLINE, &val);

	//tcpc = tcpc_dev_get_by_name("type_c_port0");
	//if (!tcpc)
		//return -EINVAL;

//#ifdef CONFIG_USB_POWER_DELIVERY
	//tcpm_notify_vbus_stable(tcpc);
//#endif

	//return 0;
//}

static int chg_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct rt_charger_info *info =
		power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->online;
		break;
#ifdef CONFIG_DUAL_PD_PORT
	case POWER_SUPPLY_PROP_PD_CAP:
		val->intval = 0;
		break;
#endif
	default:
		return -ENODEV;
	}
	return 0;
}

static int chg_set_prop(struct power_supply *psy,
                     enum power_supply_property psp,
                     const union power_supply_propval *val)
{
#ifdef CONFIG_DUAL_PD_PORT
	int mv, ma;
	int ret = 0, fixed_pdo;
	uint8_t max_power_policy = 0x21;
#endif
	struct rt_charger_info *info =
		power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		info->online = val->intval;
		//chg_enable_vbus(info, info->online);
		break;
#ifdef CONFIG_DUAL_PD_PORT
	case POWER_SUPPLY_PROP_PD_CAP:
		//if(rt_chg_check_asus_vid()){
			 /* call tcpm*/
			 mv = ((val->intval)>>16) & 0xFFFF;
			 ma = (val->intval) & 0x7FFF;
			 fixed_pdo = ((val->intval) & 0x8000)>>15;

			 pr_info("%s mv = %d, ma = %d, fixed_pdo = %d\n", __func__, mv, ma, fixed_pdo);

			 if (info->tcpc->pd_port.pe_pd_state != PE_SNK_READY){
				ret = -EBUSY;
			 }
			 if (fixed_pdo == 1) {
				/* reqeust fixed pdo */
				tcpm_set_pd_charging_policy(info->tcpc, max_power_policy, NULL);
				tcpm_dpm_pd_request(info->tcpc, mv, ma, NULL);
			 }
			 else {
				/* request APDO */
				ret = tcpm_set_apdo_charging_policy(info->tcpc, DPM_CHARGING_POLICY_PPS, mv, ma, NULL);
			}
		//}
		break;
#endif
           default:
                     return -ENODEV;
           }
#ifdef CONFIG_DUAL_PD_PORT
           return ret;
#else
           return 0;
#endif
}

static enum power_supply_property rt_chg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
#ifdef CONFIG_DUAL_PD_PORT
	POWER_SUPPLY_PROP_PD_CAP,
#endif
};

static int rtchg_init_vbus(struct rt_charger_info *info)
{
//#ifdef CONFIG_OF
	//struct device_node *np = info->dev->of_node;
	//int ret;

	//if (np == NULL) {
		//pr_err("Error: rt-changer np = NULL\n");
		//return -1;
	//}

	//info->vbus_gpio = of_get_named_gpio(np, "rt,vbus_gpio", 0);
	//ret = gpio_request(info->vbus_gpio, "DUMMY CHG VBUS CONTRL");
	//if (ret < 0) {
		//pr_err("Error: failed to request GPIO %d\n", info->vbus_gpio);
		//return ret;
	//}
	//ret = gpio_direction_output(info->vbus_gpio, 0);
	//if (ret < 0) {
		//pr_err("Error: failed to set GPIO as output pin\n");
		//return ret;
	//}
//#endif /* CONFIG_OF */

	//pr_info("%s: OK\n", __func__);
	return 0;
}

/* Please handle the notification in notifier call function,
 * User should control the Power here when you got SOURCE_VBUS notification
 * and SINK_VBUS notification
 */
static int chg_tcp_notifer_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *tcp_noti = data;
	struct rt_charger_info *info = container_of(nb, struct rt_charger_info, nb);
	uint32_t vdos[VDO_MAX_NR];
	uint16_t dpm_flags;
	int ret = 0;
	struct tcpc_device *tcpc;
	tcpc = tcpc_dev_get_by_name("type_c_port0");

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
		pr_info("%s TCP_NOTIFY_PR_SWAP\n", __func__);
		asus_request_DPDM_flag(1);
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		/* Do what you want to do here */
		break;
	case TCP_NOTIFY_HARD_RESET_STATE:
		pr_info("%s TCP_NOTIFY_HARD_RESET_STATE\n", __func__);
		break;
	case TCP_NOTIFY_PD_STATE:
		pr_info("%s TCP_NOTIFY_PD_STATE, pd state = %d\n", __func__,tcp_noti->pd_state.connected);
		switch (tcp_noti->pd_state.connected) {
		case PD_CONNECT_PE_READY_SNK:
		case PD_CONNECT_PE_READY_SNK_PD30:
			dpm_flags = tcpm_inquire_dpm_flags(tcpc);
			BTM_a2c_cable = false;
			info->peer_usb_comm = (dpm_flags &= DPM_FLAGS_PARTNER_USB_COMM);
			pr_info("%s tcpc_pd_state = PD_CONNECT_PE_READY_SNK/PD3.0, USB_COMM = %d\n", __func__, info->peer_usb_comm);
			if (info->peer_usb_comm)
				extcon_set_state_sync(smbchg_dev->extcon, EXTCON_USB, true);
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			BTM_a2c_cable = false;
			pr_info("%s tcpc_pd_state = PD_CONNECT_PE_READY_SNK_APDO\n", __func__);
			rt_chg_get_vid();
			//if(rt_chg_check_asus_vid()){
				rt_chg_ready_snk_apdo(nb);
			//}
			/* TODO: pps event */
			break;
		case PD_CONNECT_NONE:
			pr_info("%s tcpc_pd_state = PD_CONNECT_NONE\n", __func__);
			if(isPD2active) {
				rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, 0);
				rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD_CURRENT_MAX, 0);
				set_pd2_active(0);
			}
			//if(rt_chg_check_asus_vid()){
				rt_chg_connect_none(nb);
			//}
			vid = 0;
			vid_ext = 0;
			break;
		case PD_CONNECT_HARD_RESET:
			pr_info("%s tcpc_pd_state = PD_CONNECT_HARD_RESET,\n", __func__);
			if(isPD2active){
				rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, 0);
				rt_charger_set_usb_property_notifier(POWER_SUPPLY_PROP_PD_CURRENT_MAX, 0);
				set_pd2_active(0);
			}
			//if(rt_chg_check_asus_vid()){
				rt_chg_hard_reset(nb);
			//}
			vid = 0;
			vid_ext = 0;
			break;
		case PD_CONNECT_PE_READY_SRC:
		case PD_CONNECT_PE_READY_SRC_PD30:
			set_pd2_active(1);
			ret = tcpm_inquire_pd_partner_inform(tcpc, vdos);
			vid = vdos[0] & 0xFFFF;
			dpm_flags = tcpm_inquire_dpm_flags(tcpc);
			info->peer_usb_comm = (dpm_flags &= DPM_FLAGS_PARTNER_USB_COMM);
			pr_info("%s tcpc_pd_state = PD_CONNECT_PE_READY_SRC, VID = %x, ret = %d, USB_COMM = %d\n",__func__, vid, ret, info->peer_usb_comm);
			if (vid == 0x0b05) { //Gamepad VID = 0xb05
				gamepad_active = 1;
				pr_info("%s gamepad_active = %d\n", __func__, gamepad_active);
				if (gDongleType == 1 || gDongleType == 2) {
					BTM_host_notify(0); //direct disable host
					asus_request_BTM_otg_en(2);
					msleep(50);
					asus_request_BTM_otg_en(3);
				}
			}
			break;
		default:
			break;
		}
		break;
	case TCP_NOTIFY_DIS_VBUS_CTRL:
		/* Implement disable power path (otg & charger) behavior here */
		//rt_chg_handle_source_vbus(tcp_noti, 0);
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("%s TCP_NOTIFY_SOURCE_VBUS\n", __func__);
		rt_chg_handle_source_vbus(tcp_noti);
		/* Implement source vbus behavior here */
		//rt_chg_handle_source_vbus(
			//tcp_noti, (tcp_noti->vbus_state.mv > 0) ? 1 : 0);
		break;
	case TCP_NOTIFY_SINK_VBUS:
		pr_info("%s TCP_NOTIFY_SINK_VBUS\n", __func__);
		/* Implement sink vubs behavior here */
		if(isPD2active)
			rt_chg_handle_sink_vbus(tcp_noti);
		rt_chg_get_curr_state();
		asus_request_DPDM_flag(0);
		break;
	case TCP_NOTIFY_SOURCE_VCONN:
		pr_info("%s TCP_NOTIFY_SOURCE_VCONN\n", __func__);
		break;
	case TCP_NOTIFY_DR_SWAP:
		pr_info("%s TCP_NOTIFY_DR_SWAP\n", __func__);
		tcpc_swap_state = tcp_noti->swap_state.new_role;
		if (tcpc_swap_state==0){
		    pr_info("%s TCP_NOTIFY_DR_SWAP: PD_ROLE_UFP\n", __func__);
		    asus_request_BTM_otg_en(2);
		    if (info->peer_usb_comm)
				extcon_set_state_sync(smbchg_dev->extcon, EXTCON_USB, true);
		}
		if (tcpc_swap_state==1){
		    pr_info("%s TCP_NOTIFY_DR_SWAP: PD_ROLE_DFP\n", __func__);
		    extcon_set_state_sync(smbchg_dev->extcon, EXTCON_USB, false);
		     asus_request_BTM_otg_en(3);
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		pr_info("%s TCP_NOTIFY_TYPEC_STATE\n", __func__);
		tcpc_pre_typec_state = tcpc_typec_state;
		tcpc_typec_state = tcp_noti->typec_state.new_state;
		BTM_a2c_cable = tcp_noti->typec_state.a2c_cable;
		pr_info("%s tcpc_typec_state = %d, tcpc_pre_typec_state = %d, BTM_a2c_cable = %d\n", __func__, tcpc_typec_state, tcpc_pre_typec_state, BTM_a2c_cable);
		if (tcpc_typec_state == TYPEC_ATTACHED_SNK) {
			pr_info("%s tcpc_typec_state = TYPEC_ATTACHED_SNK\n", __func__);
		}
		if (tcpc_typec_state == TYPEC_ATTACHED_SRC) {
			pr_info("%s tcpc_typec_state = TYPEC_ATTACHED_SRC, gDongleType = %d\n", __func__, gDongleType);
			//Enable host except Inbox or Station
			if (gDongleType == 0) {
				pr_info("%s OTG Plug in\n", __func__);
				if (need_extcon_host_sync){
					need_extcon_host_sync = 0;
					asus_request_BTM_otg_en(2);
				}
				if (tcpc_pre_typec_state != TYPEC_ATTACHED_SNK)
					asus_request_BTM_otg_en(3);
			}
		}
		//analog audio dongle
		if (tcpc_typec_state == TYPEC_ATTACHED_AUDIO) {
			pr_info("%s tcpc_typec_state = TYPEC_ATTACHED_AUDIO\n", __func__);
			ret = asus_set_invalid_audio_dongle(1);
		}
		else if (tcpc_pre_typec_state == TYPEC_ATTACHED_AUDIO) {
			pr_info("%s tcpc_pre_typec_state = TYPEC_ATTACHED_AUDIO\n", __func__);
			ret = asus_set_invalid_audio_dongle(0);
		}
		if (tcpc_typec_state == TYPEC_UNATTACHED) {
			pr_info("%s tcpc_typec_state = TYPEC_UNATTACHED\n", __func__);
			pr_info("%s OTG Plug out, gDongleType = %d\n", __func__, gDongleType);
			//Unplug device is Gamepad
			if (gamepad_active) {
				gamepad_active = 0;
				pr_info("%s gamepad_active = %d\n", __func__, gamepad_active);
				asus_request_BTM_otg_en(2);
				if (gDongleType == 1) {
					//Inbox need retart host
					msleep(50);
					BTM_host_notify(1);
				}
			}
			//Unplug device is normal device
			else {
				//Inbox didn't need to disable host
				if (gDongleType == 1)
					need_extcon_host_sync = 1;
				else
					asus_request_BTM_otg_en(2);
			}
			info->peer_usb_comm = false;
		}
		break;
	default:
		pr_info("%s TCP_NOTIFY_DEFAULT, event = %d\n", __func__, (int)event);
		break;
	};

	return NOTIFY_OK;
}

//void test_set_flag(int en)
//{
	//test_flag = en;
//}

//void dwork_func(struct work_struct *work)
//{
	//struct rt_charger_info *info =
		//container_of(work, struct rt_charger_info, dwork.work);

	//if (test_flag)
		//pr_info("WHATWHATWHATWHATWHATWHATWWWWWWWHATHATHATHATHATHATHAT\n");

	//schedule_delayed_work(&info->dwork, msecs_to_jiffies(10));
//}

static char * rt_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc rt_power_supply_desc = {
	.name		= "rt-chg",
	.type 		= POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= rt_chg_props,
	.num_properties = ARRAY_SIZE(rt_chg_props),
	.get_property	= chg_get_prop,
	.set_property	= chg_set_prop,
};

static int rt_chg_power_supply_init(struct rt_charger_info *info)
{
	struct power_supply_config psy_cfg = { .drv_data = info, };

	psy_cfg.supplied_to = rt_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(rt_charger_supplied_to);

	info->chg = power_supply_register(info->dev, &rt_power_supply_desc,
					&psy_cfg);

	return PTR_ERR_OR_ZERO(info->chg);
}

static int isCharging(void){
	struct power_supply *usb_psy;
	union power_supply_propval val = {0};
	int vbus_present = 0;

	usb_psy = power_supply_get_by_name("usb");
	power_supply_get_property(usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	vbus_present = val.intval;

	pr_info("[Bottom_PD] rt_drm_notifier, isCharging, vbus_present = %d\n", vbus_present);
	return vbus_present;
}

static int isKeep5v(int vid, int pid){
	pr_info("rt_host_notifier, isKeep5v VID = %04x, PID = %04x\n", vid, pid);
	/*
	 * White List        pid   vid
	 * host controller : 7531  2     (0x1d6b 0x0002)
	 * host controller : 7531  3     (0x1d6b 0x0003)
	 * Kunai bumper    : 2821  30976 (0x0b05 0x7900)
	 * Kunai holder    : 2821  30977 (0x0b05 0x7901)
	 * Kunai 3 bumper  : 2821  30980 (0x0b05 0x7904)
	 * Kunai 3 holder  : 2821  30981 (0x0b05 0x7905)
	 */
	if ((vid == 2821 && pid == 30980) || (vid == 2821 && pid == 30981) ||
	(vid == 2821 && pid == 30976) || (vid == 2821 && pid == 30977) ||
	(vid == 7531 && pid == 2) || (vid == 7531 && pid == 3))
		return 1;
	else
		return 0;
}

static void checkOTGdevice(int vid, int pid, char* ipro, int from, int event){
	char prodock[]="ASUS Pro-Dock USB2.0";
	char gamevice[]="GV187";
	//pr_info("rt_host_notifier, checkOTGdevice, VID = %04x, PID = %04x, iProduct = %s, last call = %d, event = %d\n", vid, pid, ipro, from, event);
	if (vid == 8457 && pid == 10263){
		if (from == 1){
			if (event == USB_DEVICE_ADD && !strcmp(ipro, prodock))
				pro_dock_active_side = 1;
			else if (event == USB_DEVICE_REMOVE)
				pro_dock_active_side = 0;
		}
		else if (from == 2){
			if (event == USB_DEVICE_ADD && !strcmp(ipro, prodock))
				pro_dock_active_bottom= 1;
			else if (event == USB_DEVICE_REMOVE)
				pro_dock_active_bottom = 0;
		}
		pr_info("rt_host_notifier, pro_dock_active side/bottom = %d/%d\n", pro_dock_active_side, pro_dock_active_bottom);
	} else if (vid == 10232 && pid == 3005 && (from == 2)){
		if (event == USB_DEVICE_ADD && !strcmp(ipro, gamevice))
			gamevice_active = 1;
		else if (event == USB_DEVICE_REMOVE)
			gamevice_active = 0;
		pr_info("rt_host_notifier, gamevice_active = %d\n", gamevice_active);
	}
}

static void open5v_by_suspend(struct work_struct *work) {
	if(usb2_stopPower_screen){
		pr_info("[Bottom_PD] rt_drm_notifier, panel power on -- reopen power\n");
		if(!usb2_stopPower){
			typec_disable_function(1);
			msleep(100);
			typec_disable_function(0);
		}
		usb2_stopPower_screen = 0;
	}
	if(usb1_stopPower_screen){
		pr_info("[Bottom_PD] rt_drm_notifier, panel power on -- reopen side power\n");
		asus_request_SIDE_otg_en(1);
		usb1_stopPower_screen = 0;
	}
	if(usb2_stopPower_screen_DT){
		pr_info("[Bottom_PD] rt_drm_notifier, panel power on -- reopen bottom OTG, DT\n");
		DT_active_mode(1);
		usb2_stopPower_screen_DT = 0;
	}
	cancel_delayed_work(&close5vwork);
}

static void close5v_by_suspend(struct work_struct *work) {
	if(!isCharging() && pro_dock_active_side && (get_net_status() == 0) && (get_prodock_state() ==1) && !g_hpd){
		pr_info("[Bottom_PD] rt_drm_notifier, panel power off -- stop side power\n");
		asus_request_SIDE_otg_en(0);
		usb1_stopPower_screen = 1;
	}

	if((gamepad_active && !aura_screen_on) || (!isCharging() && (gamevice_active || (pro_dock_active_bottom && (get_net_status() == 0) && (get_prodock_state() ==1))))){
		pr_info("[Bottom_PD] rt_drm_notifier, panel power off -- stop power\n");
		asus_request_BTM_otg_en(2);
		asus_request_BTM_otg_en(0);
		usb2_stopPower_screen = 1;
	}

	if(get_DT_state() == 1 && !g_hpd && (get_net_status()==0)){
		pr_info("[Bottom_PD] rt_drm_notifier, panel power off -- stop bottom OTG, DT\n");
		DT_active_mode(0);
		usb2_stopPower_screen_DT = 1;
	}
}

void rt_send_screen_suspend(void){
	pr_info("[Bottom_PD] rt_drm_notifier, rt_send_screen_suspend, charging = %d, dongletype = %d\n", isCharging(), gDongleType);
	if((get_DT_state() >= 1) || (get_prodock_state() >=1) || gamevice_active || (gamepad_active && !aura_screen_on)){
		queue_delayed_work(close5v_wq, &close5vwork, HZ * 5);
	}
}
EXPORT_SYMBOL(rt_send_screen_suspend);

void rt_send_screen_resume(void){
	pr_info("[Bottom_PD] rt_drm_notifier, rt_send_screen_resume\n");
	queue_delayed_work(open5v_wq, &open5vwork, 0);
}
EXPORT_SYMBOL(rt_send_screen_resume);

static int rt_host_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr){
	struct usb_device *udev = ptr;
	struct usb_bus *ubus = ptr;
	char usb1[]="a600000.dwc3";
	char usb2[]="a800000.dwc3";
	char buf[24], devname[15];

	if(event <=2) {
		usb_string(udev, udev->descriptor.iProduct,	buf, sizeof(buf));
		pr_info("rt_host_notifier, VID = %04x, PID = %04x, iProduct = %s\n", le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct), buf);
		checkOTGdevice(le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct), buf, lastcall, event);
		if (!strcmp(dev_name(udev->bus->controller->parent), usb2) && event == USB_DEVICE_ADD){
			btm_vid = le16_to_cpu(udev->descriptor.idVendor);
			btm_pid = le16_to_cpu(udev->descriptor.idProduct);
		}
	} else{
		//pr_info("rt_host_notifier, ubus->controller = %s\n", dev_name(ubus->controller));
		if(ubus->controller->parent) {
			strcpy(devname, dev_name(ubus->controller->parent));
			//pr_info("rt_host_notifier,  devname = %s\n", devname);
			if(!strcmp(devname, usb1)){
				if (event == USB_BUS_ADD){
					usb1_active = 1;
					lastcall = 1;
					usb1_stopPower_screen = 0;
				}
				else if (event == USB_BUS_REMOVE){
					usb1_active = 0;
				}
			}

			if(!strcmp(devname, usb2)){
				if (event == USB_BUS_ADD){
					usb2_active = 1;
					lastcall = 2;
					usb2_stopPower = 0;
					usb2_stopPower_screen = 0;
					usb2_stopPower_screen_DT = 0;
				}
				else if (event == USB_BUS_REMOVE){
					usb2_active = 0;
					btm_vid = 0;
					btm_pid = 0;
				}
				pr_info("rt_host_notifier,  usb1_active = %d, usb2_active = %d, event = %lu\n", usb1_active, usb2_active, event);
			}
		}
	}

	if (usb1_active && usb2_active && (event == USB_DEVICE_ADD) && (gDongleType == 0)){
		if(!isKeep5v(btm_vid, btm_pid)){
			asus_request_BTM_otg_en(2);
			asus_request_BTM_otg_en(0);
			usb2_stopPower = 1;
			pr_info("rt_host_notifier,  close bottom 5v\n");
		} else{
			//pr_info("rt_host_notifier,  bottom is Gamevice do not close bottom 5v\n");
		}
	} else if (usb2_stopPower && !usb1_active){
		typec_disable_function(1);
		msleep(100);
		typec_disable_function(0);
		usb2_stopPower = 0;
		pr_info("rt_host_notifier,  reopen bottom 5v\n");
	}
	return NOTIFY_DONE;
}

static int rt_charger_probe(struct platform_device *pdev)
{
	struct rt_charger_info *info;
	int ret;

	pr_info("%s\n", __func__);
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	info->pd_apdo_connected = false;
	info->pd_start_direct_charging = false;

	ret = rt_chg_power_supply_init(info);

 	if (ret < 0) {
		dev_err(&pdev->dev, "chg register power supply fail\n");
 		return -EINVAL;
 	}

	ret = rtchg_init_vbus(info);
	if (ret < 0) {
		pr_err("%s gpio init fail\n", __func__);
		return -EINVAL;
	}

	/* Get tcpc device by tcpc_device'name */
	info->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!info->tcpc) {
		dev_err(&pdev->dev, "get rt1711-tcpc fail\n");
		//power_supply_unregister(&info->chg);
		return -ENODEV;
	}

	/* register tcpc notifier */
	info->nb.notifier_call = chg_tcp_notifer_call;
	ret = register_tcp_dev_notifier(info->tcpc, &info->nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		dev_err(&pdev->dev, "register tcpc notifer fail\n");
		return -EINVAL;
	}

	//INIT_DELAYED_WORK(&info->dwork, dwork_func);

	//schedule_delayed_work(&info->dwork, 0);

	host_nb.notifier_call = rt_host_notifier;
	usb_register_notify(&host_nb);

	close5v_wq = create_singlethread_workqueue("close5v_by_suspend");
	INIT_DELAYED_WORK(&close5vwork, close5v_by_suspend);

	open5v_wq = create_singlethread_workqueue("open5v_by_suspend");
	INIT_DELAYED_WORK(&open5vwork, open5v_by_suspend);

	wake_lock_init(&rt_wake_lock, WAKE_LOCK_SUSPEND, "RT_wake_lock");
	wake_lock_timeout(&rt_wake_lock, msecs_to_jiffies(15000));

	pr_info("%s: OK!\n", __func__);
	return 0;
}

static int rt_charger_remove(struct platform_device *pdev)
{
	wake_lock_destroy(&rt_wake_lock);

	//struct rt_charger_info *info = platform_get_drvdata(pdev);

	//power_supply_unregister(info->chg);
	return 0;
}

static struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt-charger",},
};

static struct platform_driver rt_charger_driver = {
	.driver = {
		.name = "rt-charger",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
	},
	.probe = rt_charger_probe,
	.remove = rt_charger_remove,
};

static int __init rt_chg_init(void)
{
	int ret;

	ret = platform_driver_register(&rt_charger_driver);
	if(ret)
		pr_info("%s: unable to register driver (%d)\n", __func__, ret);

	return ret;
}

static void __exit rt_chg_exit(void)
{
	platform_driver_unregister(&rt_charger_driver);
}
late_initcall(rt_chg_init);
module_exit(rt_chg_exit);

MODULE_DESCRIPTION("Dummy Charger driver  for kernel-4.9");
MODULE_LICENSE("GPL");
