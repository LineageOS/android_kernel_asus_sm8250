// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/usb/usbpd.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>

#include "dp_usbpd.h"
#include "dp_debug.h"

/* DP specific VDM commands */
#define DP_USBPD_VDM_STATUS	0x10
#define DP_USBPD_VDM_CONFIGURE	0x11

/* USBPD-TypeC specific Macros */
#define VDM_VERSION		0x0
#define USB_C_DP_SID		0xFF01

/* ASUS BSP DP +++ */
struct dp_usbpd_private *asus_usbpd;
struct mutex asus_gs_mutex;
extern uint8_t gDongleType;
extern struct completion usb_host_complete1;
extern int hid_to_set_ultra_power_mode(u8 type); // 1:in 0:out
extern int get_prodock_state(void);
extern bool g_station_sleep;
bool g_hpd = false;
bool g_skip_ss_lanes = false;
bool g_station_dp_disconnect = false;
EXPORT_SYMBOL(g_station_sleep);
EXPORT_SYMBOL(g_skip_ss_lanes);
EXPORT_SYMBOL(g_station_dp_disconnect);

extern bool dp_display_is_enable(void);
/* ASUS BSP DP --- */

enum dp_usbpd_pin_assignment {
	DP_USBPD_PIN_A,
	DP_USBPD_PIN_B,
	DP_USBPD_PIN_C,
	DP_USBPD_PIN_D,
	DP_USBPD_PIN_E,
	DP_USBPD_PIN_F,
	DP_USBPD_PIN_MAX,
};

enum dp_usbpd_events {
	DP_USBPD_EVT_DISCOVER,
	DP_USBPD_EVT_ENTER,
	DP_USBPD_EVT_STATUS,
	DP_USBPD_EVT_CONFIGURE,
	DP_USBPD_EVT_CC_PIN_POLARITY,
	DP_USBPD_EVT_EXIT,
	DP_USBPD_EVT_ATTENTION,
};

enum dp_usbpd_alt_mode {
	DP_USBPD_ALT_MODE_NONE	    = 0,
	DP_USBPD_ALT_MODE_INIT	    = BIT(0),
	DP_USBPD_ALT_MODE_DISCOVER  = BIT(1),
	DP_USBPD_ALT_MODE_ENTER	    = BIT(2),
	DP_USBPD_ALT_MODE_STATUS    = BIT(3),
	DP_USBPD_ALT_MODE_CONFIGURE = BIT(4),
};

struct dp_usbpd_capabilities {
	enum dp_usbpd_port port;
	bool receptacle_state;
	u8 ulink_pin_config;
	u8 dlink_pin_config;
};

struct dp_usbpd_private {
	bool forced_disconnect;
	u32 vdo;
	struct device *dev;
	struct usbpd *pd;
	struct usbpd_svid_handler svid_handler;
	struct dp_hpd_cb *dp_cb;
	struct dp_usbpd_capabilities cap;
	struct dp_usbpd dp_usbpd;
	enum dp_usbpd_alt_mode alt_mode;
	u32 dp_usbpd_config;
};

static const char *dp_usbpd_pin_name(u8 pin)
{
	switch (pin) {
	case DP_USBPD_PIN_A: return "DP_USBPD_PIN_ASSIGNMENT_A";
	case DP_USBPD_PIN_B: return "DP_USBPD_PIN_ASSIGNMENT_B";
	case DP_USBPD_PIN_C: return "DP_USBPD_PIN_ASSIGNMENT_C";
	case DP_USBPD_PIN_D: return "DP_USBPD_PIN_ASSIGNMENT_D";
	case DP_USBPD_PIN_E: return "DP_USBPD_PIN_ASSIGNMENT_E";
	case DP_USBPD_PIN_F: return "DP_USBPD_PIN_ASSIGNMENT_F";
	default: return "UNKNOWN";
	}
}

static const char *dp_usbpd_port_name(enum dp_usbpd_port port)
{
	switch (port) {
	case DP_USBPD_PORT_NONE: return "DP_USBPD_PORT_NONE";
	case DP_USBPD_PORT_UFP_D: return "DP_USBPD_PORT_UFP_D";
	case DP_USBPD_PORT_DFP_D: return "DP_USBPD_PORT_DFP_D";
	case DP_USBPD_PORT_D_UFP_D: return "DP_USBPD_PORT_D_UFP_D";
	default: return "DP_USBPD_PORT_NONE";
	}
}

static const char *dp_usbpd_cmd_name(u8 cmd)
{
	switch (cmd) {
	case USBPD_SVDM_DISCOVER_MODES: return "USBPD_SVDM_DISCOVER_MODES";
	case USBPD_SVDM_ENTER_MODE: return "USBPD_SVDM_ENTER_MODE";
	case USBPD_SVDM_ATTENTION: return "USBPD_SVDM_ATTENTION";
	case DP_USBPD_VDM_STATUS: return "DP_USBPD_VDM_STATUS";
	case DP_USBPD_VDM_CONFIGURE: return "DP_USBPD_VDM_CONFIGURE";
	default: return "DP_USBPD_VDM_ERROR";
	}
}

static void dp_usbpd_init_port(enum dp_usbpd_port *port, u32 in_port)
{
	switch (in_port) {
	case 0:
		*port = DP_USBPD_PORT_NONE;
		break;
	case 1:
		*port = DP_USBPD_PORT_UFP_D;
		break;
	case 2:
		*port = DP_USBPD_PORT_DFP_D;
		break;
	case 3:
		*port = DP_USBPD_PORT_D_UFP_D;
		break;
	default:
		*port = DP_USBPD_PORT_NONE;
	}
	DP_DEBUG("port:%s\n", dp_usbpd_port_name(*port));
}

static void dp_usbpd_get_capabilities(struct dp_usbpd_private *pd)
{
	struct dp_usbpd_capabilities *cap = &pd->cap;
	u32 buf = pd->vdo;
	int port = buf & 0x3;

	cap->receptacle_state = (buf & BIT(6)) ? true : false;
	cap->dlink_pin_config = (buf >> 8) & 0xff;
	cap->ulink_pin_config = (buf >> 16) & 0xff;

	dp_usbpd_init_port(&cap->port, port);
}

static void dp_usbpd_get_status(struct dp_usbpd_private *pd)
{
	struct dp_usbpd *status = &pd->dp_usbpd;
	u32 buf = pd->vdo;
	int port = buf & 0x3;

	status->low_pow_st     = (buf & BIT(2)) ? true : false;
	status->adaptor_dp_en  = (buf & BIT(3)) ? true : false;
	status->base.multi_func = (buf & BIT(4)) ? true : false;
	status->usb_config_req = (buf & BIT(5)) ? true : false;
	status->exit_dp_mode   = (buf & BIT(6)) ? true : false;
	status->base.hpd_high  = (buf & BIT(7)) ? true : false;
	status->base.hpd_irq   = (buf & BIT(8)) ? true : false;

	DP_DEBUG("low_pow_st = %d, adaptor_dp_en = %d, multi_func = %d\n",
			status->low_pow_st, status->adaptor_dp_en,
			status->base.multi_func);
	DP_DEBUG("usb_config_req = %d, exit_dp_mode = %d, hpd_high =%d\n",
			status->usb_config_req,
			status->exit_dp_mode, status->base.hpd_high);
	DP_DEBUG("hpd_irq = %d\n", status->base.hpd_irq);

	g_hpd = status->base.hpd_high; /* ASUS BSP DP +++ */
	dp_usbpd_init_port(&status->port, port);
}

static u32 dp_usbpd_gen_config_pkt(struct dp_usbpd_private *pd)
{
	u8 pin_cfg, pin;
	u32 config = 0;
	const u32 ufp_d_config = 0x2, dp_ver = 0x1;

	if (pd->cap.receptacle_state)
		pin_cfg = pd->cap.ulink_pin_config;
	else
		pin_cfg = pd->cap.dlink_pin_config;

	for (pin = DP_USBPD_PIN_A; pin < DP_USBPD_PIN_MAX; pin++) {
		if (pin_cfg & BIT(pin)) {
			if (pd->dp_usbpd.base.multi_func) {
				if (pin == DP_USBPD_PIN_D)
					break;
			} else {
				break;
			}
		}
	}

	if (pin == DP_USBPD_PIN_MAX)
		pin = DP_USBPD_PIN_C;

	DP_DEBUG("pin assignment: %s\n", dp_usbpd_pin_name(pin));

	config |= BIT(pin) << 8;

	config |= (dp_ver << 2);
	config |= ufp_d_config;

	DP_DEBUG("config = 0x%x\n", config);
	return config;
}

static void dp_usbpd_send_event(struct dp_usbpd_private *pd,
		enum dp_usbpd_events event)
{
	u32 config;

	switch (event) {
	case DP_USBPD_EVT_DISCOVER:
		usbpd_send_svdm(pd->pd, USB_C_DP_SID,
			USBPD_SVDM_DISCOVER_MODES,
			SVDM_CMD_TYPE_INITIATOR, 0x0, 0x0, 0x0);
		break;
	case DP_USBPD_EVT_ENTER:
		usbpd_send_svdm(pd->pd, USB_C_DP_SID,
			USBPD_SVDM_ENTER_MODE,
			SVDM_CMD_TYPE_INITIATOR, 0x1, 0x0, 0x0);
		break;
	case DP_USBPD_EVT_EXIT:
		usbpd_send_svdm(pd->pd, USB_C_DP_SID,
			USBPD_SVDM_EXIT_MODE,
			SVDM_CMD_TYPE_INITIATOR, 0x1, 0x0, 0x0);
		break;
	case DP_USBPD_EVT_STATUS:
		config = 0x1; /* DFP_D connected */
		usbpd_send_svdm(pd->pd, USB_C_DP_SID, DP_USBPD_VDM_STATUS,
			SVDM_CMD_TYPE_INITIATOR, 0x1, &config, 0x1);
		break;
	case DP_USBPD_EVT_CONFIGURE:
		config = dp_usbpd_gen_config_pkt(pd);
		usbpd_send_svdm(pd->pd, USB_C_DP_SID, DP_USBPD_VDM_CONFIGURE,
			SVDM_CMD_TYPE_INITIATOR, 0x1, &config, 0x1);
		break;
	default:
		DP_ERR("unknown event:%d\n", event);
	}
}

static void dp_usbpd_connect_cb(struct usbpd_svid_handler *hdlr,
		bool peer_usb_comm)
{
	struct dp_usbpd_private *pd;

	pd = container_of(hdlr, struct dp_usbpd_private, svid_handler);
	if (!pd) {
		DP_ERR("get_usbpd phandle failed\n");
		return;
	}

	DP_DEBUG("peer_usb_comm: %d\n", peer_usb_comm);
	pd->dp_usbpd.base.peer_usb_comm = peer_usb_comm;
	dp_usbpd_send_event(pd, DP_USBPD_EVT_DISCOVER);
}

static void dp_usbpd_disconnect_cb(struct usbpd_svid_handler *hdlr)
{
	struct dp_usbpd_private *pd;

	pd = container_of(hdlr, struct dp_usbpd_private, svid_handler);
	if (!pd) {
		DP_ERR("get_usbpd phandle failed\n");
		return;
	}

	pd->alt_mode = DP_USBPD_ALT_MODE_NONE;
	pd->dp_usbpd.base.alt_mode_cfg_done = false;
	/* ASUS BSP DP +++ */
	DP_LOG("%s\n", __func__);

	if (pd->dp_cb && pd->dp_cb->disconnect) {
		g_hpd = false; /* ASUS BSP DP +++ */
		pd->dp_cb->disconnect(pd->dev);
	}
}

static int dp_usbpd_validate_callback(u8 cmd,
	enum usbpd_svdm_cmd_type cmd_type, int num_vdos)
{
	int ret = 0;

	if (cmd_type == SVDM_CMD_TYPE_RESP_NAK) {
		DP_ERR("error: NACK\n");
		ret = -EINVAL;
		goto end;
	}

	if (cmd_type == SVDM_CMD_TYPE_RESP_BUSY) {
		DP_ERR("error: BUSY\n");
		ret = -EBUSY;
		goto end;
	}

	if (cmd == USBPD_SVDM_ATTENTION) {
		if (cmd_type != SVDM_CMD_TYPE_INITIATOR) {
			DP_ERR("error: invalid cmd type for attention\n");
			ret = -EINVAL;
			goto end;
		}

		if (!num_vdos) {
			DP_ERR("error: no vdo provided\n");
			ret = -EINVAL;
			goto end;
		}
	} else {
		if (cmd_type != SVDM_CMD_TYPE_RESP_ACK) {
			DP_ERR("error: invalid cmd type\n");
			ret = -EINVAL;
		}
	}
end:
	return ret;
}


static int dp_usbpd_get_ss_lanes(struct dp_usbpd_private *pd)
{
	int rc = 0;
	int timeout = 250;

	/* ASUS BSP DP, ss lanes still keeping because self reconnect in dp +++*/
	if (g_skip_ss_lanes) {
		g_skip_ss_lanes = false;
		DP_LOG("skip request ss lanes.\n");
		return rc;
	}
	/* ASUS BSP DP, ss lanes still keeping because self reconnect in dp ---*/

	/*
	 * By default, USB reserves two lanes for Super Speed.
	 * Which means DP has remaining two lanes to operate on.
	 * If multi-function is not supported, request USB to
	 * release the Super Speed lanes so that DP can use
	 * all four lanes in case DPCD indicates support for
	 * four lanes.
	 */
	if (!pd->dp_usbpd.base.multi_func) {
		while (timeout) {
			/* ASUS BSP DP +++ */
			/* To sync usb host if not DT dock */
			if (gDongleType != 3) {
				if (!wait_for_completion_timeout(&usb_host_complete1, HZ * 2))
					DP_LOG("usb host timeout\n");
			}
			/* ASUS BSP DP --- */

			rc = pd->svid_handler.request_usb_ss_lane(
					pd->pd, &pd->svid_handler);
			if (rc != -EBUSY)
				break;

			DP_WARN("USB busy, retry\n");

			/* wait for hw recommended delay for usb */
			msleep(20);
			timeout--;
		}
	}

	DP_LOG("%s: multi_func=%d, rc=%d\n", __func__, pd->dp_usbpd.base.multi_func, rc);
	return rc;
}

static void dp_usbpd_response_cb(struct usbpd_svid_handler *hdlr, u8 cmd,
				enum usbpd_svdm_cmd_type cmd_type,
				const u32 *vdos, int num_vdos)
{
	struct dp_usbpd_private *pd;
	int rc = 0;

	pd = container_of(hdlr, struct dp_usbpd_private, svid_handler);

	DP_WARN("[DisplayPort] callback -> cmd: %s, *vdos = 0x%x, num_vdos = %d\n",
				dp_usbpd_cmd_name(cmd), *vdos, num_vdos);

	if (dp_usbpd_validate_callback(cmd, cmd_type, num_vdos)) {
		DP_DEBUG("invalid callback received\n");
		return;
	}

	switch (cmd) {
	case USBPD_SVDM_DISCOVER_MODES:
		pd->vdo = *vdos;
		dp_usbpd_get_capabilities(pd);

		pd->alt_mode |= DP_USBPD_ALT_MODE_DISCOVER;

		if (pd->cap.port & BIT(0))
			dp_usbpd_send_event(pd, DP_USBPD_EVT_ENTER);
		break;
	case USBPD_SVDM_ENTER_MODE:
		pd->alt_mode |= DP_USBPD_ALT_MODE_ENTER;

		dp_usbpd_send_event(pd, DP_USBPD_EVT_STATUS);
		break;
	case USBPD_SVDM_ATTENTION:
		if (pd->forced_disconnect)
			break;

		pd->vdo = *vdos;
		dp_usbpd_get_status(pd);

		if (!pd->dp_usbpd.base.alt_mode_cfg_done) {
			if (pd->dp_usbpd.port & BIT(1))
				dp_usbpd_send_event(pd, DP_USBPD_EVT_CONFIGURE);
			break;
		}

		if (pd->dp_cb && pd->dp_cb->attention)
			pd->dp_cb->attention(pd->dev);

		break;
	case DP_USBPD_VDM_STATUS:
		pd->vdo = *vdos;
		dp_usbpd_get_status(pd);

		if (!(pd->alt_mode & DP_USBPD_ALT_MODE_CONFIGURE)) {
			pd->alt_mode |= DP_USBPD_ALT_MODE_STATUS;

			if (pd->dp_usbpd.port & BIT(1))
				dp_usbpd_send_event(pd, DP_USBPD_EVT_CONFIGURE);
		}
		break;
	case DP_USBPD_VDM_CONFIGURE:
		pd->alt_mode |= DP_USBPD_ALT_MODE_CONFIGURE;
		pd->dp_usbpd.base.alt_mode_cfg_done = true;
		dp_usbpd_get_status(pd);

		pd->dp_usbpd.base.orientation =
			usbpd_get_plug_orientation(pd->pd);

		rc = dp_usbpd_get_ss_lanes(pd);
		if (rc) {
			DP_ERR("failed to get SuperSpeed lanes\n");
			break;
		}

		if (pd->dp_cb && pd->dp_cb->configure)
			pd->dp_cb->configure(pd->dev);
		break;
	default:
		DP_ERR("unknown cmd: %d\n", cmd);
		break;
	}
}

static int dp_usbpd_simulate_connect(struct dp_hpd *dp_hpd, bool hpd)
{
	int rc = 0;
	struct dp_usbpd *dp_usbpd;
	struct dp_usbpd_private *pd;

	if (!dp_hpd) {
		DP_ERR("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	dp_usbpd = container_of(dp_hpd, struct dp_usbpd, base);
	pd = container_of(dp_usbpd, struct dp_usbpd_private, dp_usbpd);

	dp_usbpd->base.hpd_high = hpd;
	pd->forced_disconnect = !hpd;
	pd->dp_usbpd.base.alt_mode_cfg_done = hpd;

	DP_DEBUG("hpd_high=%d, forced_disconnect=%d, orientation=%d\n",
			dp_usbpd->base.hpd_high, pd->forced_disconnect,
			pd->dp_usbpd.base.orientation);
	if (hpd)
		pd->dp_cb->configure(pd->dev);
	else
		pd->dp_cb->disconnect(pd->dev);

error:
	return rc;
}

static int dp_usbpd_simulate_attention(struct dp_hpd *dp_hpd, int vdo)
{
	int rc = 0;
	struct dp_usbpd *dp_usbpd;
	struct dp_usbpd_private *pd;

	dp_usbpd = container_of(dp_hpd, struct dp_usbpd, base);
	if (!dp_usbpd) {
		DP_ERR("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	pd = container_of(dp_usbpd, struct dp_usbpd_private, dp_usbpd);

	pd->vdo = vdo;
	dp_usbpd_get_status(pd);

	if (pd->dp_cb && pd->dp_cb->attention)
		pd->dp_cb->attention(pd->dev);
error:
	return rc;
}

int dp_usbpd_register(struct dp_hpd *dp_hpd)
{
	struct dp_usbpd *dp_usbpd;
	struct dp_usbpd_private *usbpd;
	int rc = 0;

	if (!dp_hpd)
		return -EINVAL;

	dp_usbpd = container_of(dp_hpd, struct dp_usbpd, base);

	usbpd = container_of(dp_usbpd, struct dp_usbpd_private, dp_usbpd);

	rc = usbpd_register_svid(usbpd->pd, &usbpd->svid_handler);
	if (rc)
		DP_ERR("pd registration failed\n");

	return rc;
}

static void dp_usbpd_wakeup_phy(struct dp_hpd *dp_hpd, bool wakeup)
{
	struct dp_usbpd *dp_usbpd;
	struct dp_usbpd_private *usbpd;

	dp_usbpd = container_of(dp_hpd, struct dp_usbpd, base);
	usbpd = container_of(dp_usbpd, struct dp_usbpd_private, dp_usbpd);

	if (!usbpd->pd) {
		DP_ERR("usbpd pointer invalid");
		return;
	}

	usbpd_vdm_in_suspend(usbpd->pd, wakeup);
}

struct dp_hpd *dp_usbpd_get(struct device *dev, struct dp_hpd_cb *cb)
{
	int rc = 0;
	const char *pd_phandle = "qcom,dp-usbpd-detection";
	struct usbpd *pd = NULL;
	struct dp_usbpd_private *usbpd;
	struct dp_usbpd *dp_usbpd;
	struct usbpd_svid_handler svid_handler = {
		.svid		= USB_C_DP_SID,
		.vdm_received	= NULL,
		.connect	= &dp_usbpd_connect_cb,
		.svdm_received	= &dp_usbpd_response_cb,
		.disconnect	= &dp_usbpd_disconnect_cb,
	};

	if (!cb) {
		DP_ERR("invalid cb data\n");
		rc = -EINVAL;
		goto error;
	}

	pd = devm_usbpd_get_by_phandle(dev, pd_phandle);
	if (IS_ERR(pd)) {
		DP_ERR("usbpd phandle failed (%ld)\n", PTR_ERR(pd));
		rc = PTR_ERR(pd);
		goto error;
	}

	usbpd = devm_kzalloc(dev, sizeof(*usbpd), GFP_KERNEL);
	if (!usbpd) {
		rc = -ENOMEM;
		goto error;
	}

	usbpd->dev = dev;
	usbpd->pd = pd;
	usbpd->svid_handler = svid_handler;
	usbpd->dp_cb = cb;

	dp_usbpd = &usbpd->dp_usbpd;
	dp_usbpd->base.simulate_connect = dp_usbpd_simulate_connect;
	dp_usbpd->base.simulate_attention = dp_usbpd_simulate_attention;
	dp_usbpd->base.register_hpd = dp_usbpd_register;
	dp_usbpd->base.wakeup_phy = dp_usbpd_wakeup_phy;
	/* ASUS BSP DP +++ */
	asus_usbpd = usbpd;
	mutex_init(&asus_gs_mutex);
	/* ASUS BSP DP --- */

	return &dp_usbpd->base;
error:
	return ERR_PTR(rc);
}

void dp_usbpd_put(struct dp_hpd *dp_hpd)
{
	struct dp_usbpd *dp_usbpd;
	struct dp_usbpd_private *usbpd;

	dp_usbpd = container_of(dp_hpd, struct dp_usbpd, base);
	if (!dp_usbpd)
		return;

	usbpd = container_of(dp_usbpd, struct dp_usbpd_private, dp_usbpd);

	usbpd_unregister_svid(usbpd->pd, &usbpd->svid_handler);

	devm_kfree(usbpd->dev, usbpd);
}
/* ASUS BSP DP +++ */
void asus_dp_disconnect(void)
{
	DP_LOG("DP self disconnect. gDongleType(%d)\n", gDongleType);

	/* RETURN if DP_STATE_CONFIGURED but not DP_STATE_ENABLED
	 * waiting for connect process done
	 */
	if (!(dp_display_is_enable())) {
		DP_LOG("return asus_dp_disconnect\n");
		return;
	}

	if(asus_usbpd->svid_handler.discovered){
		asus_usbpd->svid_handler.disconnect(&asus_usbpd->svid_handler);
		asus_usbpd->svid_handler.discovered = false;
	}
	usbpd_unregister_svid(asus_usbpd->pd, &asus_usbpd->svid_handler);

	if (gDongleType == 2)
		g_skip_ss_lanes = true;
}

void asus_dp_connect(void)
{
	int rc = 0;
	DP_LOG("DP self connect. gDongleType(%d)\n", gDongleType);

	rc = usbpd_register_svid(asus_usbpd->pd, &asus_usbpd->svid_handler);
	if (rc)
		DP_LOG("svid is registered\n");
}

//type = 0, call from lid
//type = 1, call from display on/off
//type = 2, call from usb
//type = 4, call from ec firmware update
void asus_dp_change_state(bool mode, int type)
{
	static bool virtual_remove = false;

	DP_LOG("state changed, mode=%d, type=%d\n", mode, type);

	mutex_lock(&asus_gs_mutex);
	if ((type == 0 || type == 2) && gDongleType == 2)
	{
		if (mode) {
			if (g_station_sleep || type == 2){
				//hid_to_set_ultra_power_mode(0);
				g_station_dp_disconnect = false;
				asus_dp_connect();
				virtual_remove = false;
			}
		} else {
			if (g_station_sleep || type == 2){
				asus_dp_disconnect();
				virtual_remove = true;
				g_station_dp_disconnect = true;
				//hid_to_set_ultra_power_mode(1);
			}
		}
	} else if (type == 1) {
		int prodock = get_prodock_state();
		static bool needConnect = false;

		if (mode) {
			if (needConnect) {
				asus_dp_connect();
				needConnect = false;
			}
		} else if (prodock != 0){
			if (!g_hpd) {
				asus_dp_disconnect();
				needConnect = true;
			}
		}
	} else if (type == 4) {
		if (mode) {
			asus_dp_connect();
		} else {
			asus_dp_disconnect();
		}
	} else if (type == 0 && gDongleType == 200 && mode && virtual_remove) {
		DP_LOG("DP register after DP disconnect when covered\n");
		g_station_dp_disconnect = false;
		asus_dp_connect();
		g_skip_ss_lanes = false;
		virtual_remove = false;
	}
	mutex_unlock(&asus_gs_mutex);

	return;
}
EXPORT_SYMBOL(asus_dp_change_state);
// ASUS BSP DP ---
